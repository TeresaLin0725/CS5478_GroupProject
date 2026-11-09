"""Freeze and execute every declared native paired episode exactly once."""
import argparse,hashlib,json,os,shutil,subprocess,sys,time
from dataclasses import asdict,replace
from datetime import datetime,timezone
from pathlib import Path
from benchmark_protocol import ARMS,POLICIES,TRIALS,apply_arm,task_specs,validate_pairing
from benchmark_episode import atomic,read
from benchmark_watchdog import WatchdogChannel

ROOT=Path(__file__).resolve().parents[1]
HARNESSES=('benchmark_protocol.py','benchmark_episode.py','benchmark_watchdog.py','run_paired_benchmark.py','build_benchmark_report.py','check_llm_connection.py')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def utc():return datetime.now(timezone.utc).isoformat()
def module_deadline_expired(watch,folder,protocol,now):
    if not watch.get('deadline_at') or now<=watch['deadline_at']+2:return False
    # Shared memory is authoritative; an older queued disk event cannot excuse
    # a genuinely hung current invocation of the same module.
    if protocol.get('watchdog_transport')=='shared_memory_v1':return True
    return not module_finished_in_log(folder,watch['pending_module'])

def module_finished_in_log(folder, module):
    """A flushed end event precedes watchdog-file replacement on Windows."""
    path=Path(folder)/'events.jsonl'
    if not path.exists():return False
    with path.open('rb') as stream:
        stream.seek(0,2);size=stream.tell();stream.seek(max(0,size-65536))
        lines=stream.read().splitlines()
    for line in reversed(lines):
        try:event=json.loads(line)
        except (ValueError,UnicodeError):continue
        if event.get('event') in ('module_begin','module_end'):
            return event.get('event')=='module_end' and event.get('module')==module
    return False

def export_partial_calls(folder):
    """Recover call exports from flushed events when a watchdog kills a worker."""
    calls=[];path=Path(folder)/'events.jsonl'
    if path.exists():
        for line in path.read_text(encoding='utf-8').splitlines():
            try:event=json.loads(line)
            except json.JSONDecodeError:continue
            if event.get('event')=='llm_call':calls.append(event['record'])
    atomic(Path(folder)/'llm-calls.json',calls)
    return calls
def python():
    p=ROOT/'data/venv-igibson/Scripts/python.exe'
    return str(p if p.exists() else Path(sys.executable))

def benchmark_manifest():
    """Shared A/B geometry for new suites, without rewriting historical manifests."""
    from frontier_search.runner.integrated import shared_round_manifest_for
    return replace(shared_round_manifest_for('template'),initialization_deadline_s=240.)

def noise_for_job(config,job):
    """Same stream across policies in a task; heading blocks have distinct seeds."""
    return replace(config,seed=(config.seed+job['environment_seed']) % (2**32))

def freeze(out,workers,arm=None,sensor_noise=None):
    if (out/'protocol.json').exists():raise FileExistsError('Choose a fresh suite directory; use run to resume the queue')
    sys.path.insert(0,str(ROOT/'src'))
    from frontier_search.runner.integrated import source_fingerprint
    from frontier_search.policies.provider import load_local_env
    from frontier_search.contracts import Pose
    env_file=next(p for p in (ROOT/'.env',ROOT.parent/'.env') if p.is_file())
    load_local_env(env_file)
    engine=source_fingerprint()
    tests=read(ROOT/'report/evidence/demo/checks-multiview/tests.json')
    assert tests['returncode']==0 and tests['source_unchanged'] and tests['source_fingerprint']==engine,'regression certificate does not match'
    out.mkdir(parents=True,exist_ok=True);snapshot=out/'frozen'
    shutil.copytree(ROOT/'src',snapshot/'src',ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    (snapshot/'weights/yolo11').mkdir(parents=True)
    shutil.copy2(ROOT/'weights/yolo11/yolo11s.pt',snapshot/'weights/yolo11/yolo11s.pt')
    (snapshot/'harness').mkdir()
    for name in HARNESSES:shutil.copy2(ROOT/'scripts'/name,snapshot/'harness'/name)
    jobs=task_specs();base=benchmark_manifest()
    if sensor_noise is not None:
        base=replace(base,sensor_noise=sensor_noise)
    for job in jobs:
        recovery=replace(base.recovery,**apply_arm(job,arm)) if arm else base.recovery
        manifest=replace(base,episode_id=job['job_id'],policy_name=job['policy'],recovery=recovery,
            start_pose=Pose(**job['start_pose']),environment_seed=job['environment_seed'],
            perception_seed=job['perception_seed'],policy_seed=job['policy_seed'],
            sensor_noise=noise_for_job(base.sensor_noise,job))
        job['sensor_noise']=asdict(manifest.sensor_noise)
        manifest.validate();path=Path('manifests')/(job['job_id']+'.json')
        atomic(out/path,asdict(manifest));job['manifest_path']=path.as_posix();job['manifest_sha256']=sha(out/path)
    validate_pairing(jobs)
    assets=ROOT/'data/igibson-data';asset_hashes={}
    for directory in (assets/'g_dataset/Rs',assets/'robot_assets/assets/models/locobot',
                      assets/'robot_assets/assets/models/ycb/025_mug',assets/'robot_assets/assets/models/ycb/006_mustard_bottle'):
        for path in sorted(directory.rglob('*')):
            if path.is_file():asset_hashes[str(path.relative_to(assets)).replace('\\','/')]=sha(path)
    seals={str(p.relative_to(snapshot)).replace('\\','/'):sha(p) for p in snapshot.rglob('*') if p.is_file()}
    protocol=dict(schema_version='1.1',status='frozen_before_first_episode',created_utc=utc(),
        run_root=str(out),snapshot_root=str(snapshot),env_file=str(env_file),engine_fingerprint=engine,
        watchdog_transport='shared_memory_v1',
        navigation_preset='shared_round',navigation_options=dict(start_margin_m=0.0),
        sensor_noise=asdict(base.sensor_noise),
        noise_seed_semantics='Per-job noise seed = (base noise seed + environment seed) mod 2**32. Independent Gaussian/dropout streams keyed by capture sequence; same across policies, not necessarily the same physical observations.',
        scope='Current Rs benchmark: one physical house, three scene conditions, three seed/initial-heading blocks, four policies; 36 fresh native episodes. Not multi-house generalization.',
        jobs=jobs,seed_heading_blocks=[dict(seed=seed,yaw_rad=yaw) for seed,yaw in TRIALS],
        seed_semantics='Geometry and cup position are fixed. Initial heading is a predeclared task variation. Dedicated policy seed controls Random; deterministic baselines and YOLO need not vary with seed. Hosted LLM is not reproducible from a local seed.',
        asset_paths=dict(robot_assets=str(assets/'robot_assets/assets'),scenes=str(assets/'g_dataset')),
        asset_hashes=asset_hashes,snapshot_seals=seals,base_manifest=asdict(base),
        policies=list(POLICIES),profiles=['apartment','clutter','dynamic'],
        target_evaluator_only=dict(model='025_mug',class_name='cup',position_world=[-1.4,2.95,.9213],
            reachability='Native collision/floor oracle preflight, parked cart if dynamic; oracle files never read by episode worker.'),
        runtime=dict(workers=workers,torch_threads=4,device='0',python=sys.version,startup_deadline_s=300.,
            wall_time_interpretation='Concurrent shared-resource wall times; not isolated single-robot latency.'),
        regression_certificate=tests,no_episode_retries=True,arm=arm,arm_settings=ARMS.get(arm),
        reporting=dict(primary_unit='profile x seed/initial-heading paired block',penalized_failure_s=600.,
            confidence='Wilson descriptive success intervals; 10000 paired stratified bootstrap draws, RNG 20261018; exact McNemar with Holm correction for six comparisons.',
            include_all_terminations=True,success_conditioned_metrics_include_n=True))
    atomic(out/'protocol.json',protocol)
    (out/'protocol.sha256').write_text(sha(out/'protocol.json')+'\n',encoding='ascii')
    # Human/repository entry points hold the same fully populated manifest.
    # An arm never overwrites the original frozen batch's config.
    if sensor_noise is None:
        atomic(ROOT/'configs'/('experiment.rs-20261018-%s.json'%arm if arm else 'experiment.rs-20261018.json'),protocol)
    atomic(out/'progress.json',dict(status='preflight_pending',active=[],completed=0,planned=len(jobs),updated_utc=utc()))
    from build_benchmark_report import build
    build(out)
    print(json.dumps(dict(run_root=str(out),episodes=len(jobs),engine_fingerprint=engine,assets=len(asset_hashes))),flush=True)

def frozen_env(protocol):
    return dict(os.environ,PYTHONPATH=str(Path(protocol['snapshot_root'])/'src'),PYTHONIOENCODING='utf-8',
        GIBSON_ASSETS_PATH=protocol['asset_paths']['robot_assets'],GIBSON_DATASET_PATH=protocol['asset_paths']['scenes'],
        YOLO_CONFIG_DIR=str(Path(protocol['run_root'])/'yolo-config'))

def check_seals(out,protocol,assets=False):
    assert sha(out/'protocol.json')==(out/'protocol.sha256').read_text().strip(),'protocol mutated'
    for name,digest in protocol['snapshot_seals'].items():assert sha(Path(protocol['snapshot_root'])/name)==digest,('snapshot mutated',name)
    if assets:
        assetroot=Path(protocol['asset_paths']['scenes']).parent
        for name,digest in protocol['asset_hashes'].items():assert sha(assetroot/name)==digest,('asset mutated',name)

def command(protocol,*args):
    return [python(),str(Path(protocol['snapshot_root'])/'harness/benchmark_episode.py'),
            '--protocol',str(Path(protocol['run_root'])/'protocol.json'),*args]

def preflights(out,protocol):
    processes=[]
    for profile in protocol['profiles']:
        folder=out/'preflight'/profile;folder.mkdir(parents=True,exist_ok=True)
        if (folder/'preflight.json').exists():
            assert read(folder/'preflight.json')['passed'],('failed preflight',profile)
            continue
        log=(folder/'console.log').open('w',encoding='utf-8')
        process=subprocess.Popen(command(protocol,'--preflight',profile),cwd=str(ROOT),env=frozen_env(protocol),
            stdout=log,stderr=subprocess.STDOUT,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        processes.append((profile,process,log,time.time()))
    while processes:
        for item in list(processes):
            profile,p,log,started=item
            if time.time()-started>600:p.kill()
            if p.poll() is not None:
                log.close();processes.remove(item)
                path=out/'preflight'/profile/'preflight.json'
                assert p.returncode==0 and path.exists() and read(path)['passed'],('native preflight failed',profile)
                print('preflight_passed',profile,flush=True)
        time.sleep(1)
    reports={p:read(out/'preflight'/p/'preflight.json') for p in protocol['profiles']}
    atomic(out/'preflight/index.json',dict(passed=all(r['passed'] for r in reports.values()),profiles=reports,checked_utc=utc(),oracle_data_isolation=True))

def result_row(out,job,protocol):
    folder=out/'episodes'/job['job_id'];summary=read(folder/'summary.json')
    events=[]
    if (folder/'events.jsonl').exists():
        for line in (folder/'events.jsonl').open(encoding='utf-8'):
            try:events.append(json.loads(line))
            except json.JSONDecodeError:continue
    calls=[e['record'] for e in events if e['event']=='llm_call']
    success=summary['termination']=='success'
    checks=dict(frozen_source=summary.get('source_fingerprint')==protocol['engine_fingerprint'] and summary.get('source_unchanged') is True,
        manifest_matches=(not events or next((e['manifest'] for e in events if e['event']=='episode_start'),None)==read(out/job['manifest_path'])),
        independent_success_evidence=(not success or all(summary.get('evaluation',{}).get(k) is True for k in
            ('w3_valid','distinct_times','stationary_evidence','near_evidence','terminal_stopped'))),
        success_evidence_file=(not success or (folder/'evaluation-only.json').exists()))
    captures=[e for e in events if e['event']=='capture_processed']
    checks['fresh_capture_sequences']=all(b['sequence']>a['sequence'] for a,b in zip(captures,captures[1:]))
    decisions=[e for e in events if e['event']=='decision']
    checks['decision_revision']=all(e['context']['evaluation']['state_revision']==e['state']['revision'] for e in decisions)
    model_calls=sum(bool(c.get('model_called',
        c.get('fallback_reason') not in ('unconfigured_client','call_cap'))) for c in calls)
    checks['llm_count']=summary.get('partial_metrics',False) or (
        model_calls==summary.get('llm_calls',0)
        and len(calls)==summary.get('llm_attempts',len(calls)))
    if 'sensor_noise' in job:
        started=next((e['manifest'] for e in events if e['event']=='episode_start'),{})
        recorded=started.get('sensor_noise') if summary.get('partial_metrics',False) else summary.get('sensor_noise')
        checks['sensor_noise_configuration']=recorded==job['sensor_noise']
    if not all(checks.values()):success=False
    usage=[c.get('usage') or {} for c in calls]
    costs=[c['cost'] for c in calls if isinstance(c.get('cost'),(int,float))]
    row=dict(job,termination=summary['termination'] if all(checks.values()) else 'integrity_error',
        original_termination=summary['termination'],success=success,integrity_checks=checks,
        simulated_time_s=summary.get('simulated_time_s'),actual_distance_m=summary.get('actual_distance_m'),
        wall_time_s=summary.get('wall_time_s',0),collisions=summary.get('collisions',0),
        decisions=summary.get('decisions',0),attempts=summary.get('attempts',0),
        llm_attempts=len(calls),llm_calls=model_calls,
        llm_fallbacks=sum(bool(c.get('fallback_reason')) for c in calls),
        llm_fallback_reasons=[c['fallback_reason'] for c in calls if c.get('fallback_reason')],
        llm_latencies_s=[c['latency_s'] for c in calls],llm_tokens_total=sum(u.get('total_tokens',0) or 0 for u in usage),
        recorded_cost_total=sum(costs),calls_with_recorded_cost=len(costs),
        response_models=sorted({c.get('provider',{}).get('response_model','unknown') for c in calls if c.get('provider')}),
        local_replans=sum(e['event']=='navigation_map_feedback' and e.get('kind')=='replan' for e in events),
        no_progress_attempts=sum(e['event']=='progress' and e['new_known_area_m2']<protocol['base_manifest']['recovery']['minimum_progress_m2'] for e in events),
        frontier_visits=sum(e['event']=='dispatch' and e['request']['subgoal']['kind']=='frontier' for e in events),
        source_fingerprint=summary.get('source_fingerprint'),partial_metrics=summary.get('partial_metrics',False))
    if 'sensor_noise' in job:
        noise=job['sensor_noise']
        row.update(noise_enabled=noise['enabled'],noise_seed=noise['seed'],
            depth_noise_std_m=noise['depth_std_m'],depth_dropout_prob=noise['depth_dropout_prob'],
            sensor_noise_stats=summary.get('sensor_noise_stats'))
    atomic(folder/'result.json',row);return row

def select_jobs(jobs,only_baselines=False,only_llm=False,job_ids=None):
    if sum(bool(value) for value in (only_baselines,only_llm,job_ids))>1:
        raise ValueError('execution scopes are mutually exclusive')
    if job_ids:
        requested=list(job_ids)
        if len(requested)!=len(set(requested)):raise ValueError('duplicate job ID in execution scope')
        known={j['job_id'] for j in jobs};unknown=set(requested)-known
        if unknown:raise ValueError('unknown job IDs: '+', '.join(sorted(unknown)))
        selected=set(requested)
        return [j for j in jobs if j['job_id'] in selected]
    if only_baselines:return [j for j in jobs if j['policy']!='llm_guided']
    if only_llm:return [j for j in jobs if j['policy']=='llm_guided']
    return list(jobs)

def run(out,only_baselines=False,only_llm=False,job_ids=None):
    protocol=read(out/'protocol.json');check_seals(out,protocol,True)
    preflights(out,protocol)
    from build_benchmark_report import build
    jobs=protocol['jobs'];execution_jobs=select_jobs(jobs,only_baselines,only_llm,job_ids)
    scope=('baselines' if only_baselines else 'llm_guided' if only_llm else
           'jobs:'+','.join(j['job_id'] for j in execution_jobs) if job_ids else 'all')
    queue=[];active=[];finished=[]
    for j in execution_jobs:
        folder=out/'episodes'/j['job_id']
        if (folder/'summary.json').exists():finished.append(result_row(out,j,protocol))
        elif (folder/'console.log').exists():raise RuntimeError('Interrupted episode has no summary; retain and adjudicate before resuming: '+j['job_id'])
        else:queue.append(j)
    start=time.time();last_report=0
    while queue or active:
        if (out/'cancel.request').exists():
            # Existing workers observe the same cancellation request. Queued jobs
            # remain unstarted, never silently labeled as benchmark failures.
            queue=[]
        while queue and len(active)<protocol['runtime']['workers']:
            j=queue.pop(0);folder=out/'episodes'/j['job_id'];folder.mkdir(parents=True,exist_ok=True)
            log=(folder/'console.log').open('x',encoding='utf-8')
            proc=subprocess.Popen(command(protocol,'--job',j['job_id']),cwd=str(ROOT),env=frozen_env(protocol),
                stdout=log,stderr=subprocess.STDOUT,creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
            active.append(dict(job=j,proc=proc,log=log,started=time.time(),watch={}))
            print('episode_started',j['job_id'],flush=True)
        for item in list(active):
            j=item['job'];folder=out/'episodes'/j['job_id'];p=item['proc'];watch=folder/'watchdog.json';reason=None
            if protocol.get('watchdog_transport')=='shared_memory_v1':
                try:
                    if 'channel' not in item:item['channel']=WatchdogChannel(folder)
                    sample=item['channel'].read()
                    if sample is not None:item['watch']=sample
                except (FileNotFoundError,ValueError,json.JSONDecodeError):pass
            elif watch.exists():

                try:item['watch']=read(watch)
                except (OSError,json.JSONDecodeError):pass
            w=item['watch'];now=time.time()
            if not w and now-item['started']>protocol['runtime'].get('startup_deadline_s',180):reason='startup_deadline'
            if module_deadline_expired(w,folder,protocol,now):reason='module_deadline:'+w['pending_module']
            if now-item['started']>protocol['base_manifest']['limits']['max_wall_time_s']+180:reason='hard_wall_deadline'
            if reason and p.poll() is None:p.kill()
            if p.poll() is not None:
                if 'channel' in item:item['channel'].close()
                item['log'].close();active.remove(item)
                if not (folder/'summary.json').exists():
                    calls=export_partial_calls(folder)
                    atomic(folder/'summary.json',dict(j,termination='pipeline_error',detail=reason or 'worker_exit_without_summary',
                        wall_time_s=now-item['started'],simulated_time_s=w.get('simulated_time_s'),
                        actual_distance_m=w.get('actual_distance_m'),collisions=w.get('collisions',0),
                        decisions=w.get('decisions',0),attempts=w.get('attempts',0),steps=w.get('steps',0),
                        partial_metrics=True,source_fingerprint=protocol['engine_fingerprint'],source_unchanged=True,
                        llm_attempts=len(calls),llm_calls=sum(bool(c.get('model_called',
                            c.get('fallback_reason') not in ('unconfigured_client','call_cap'))) for c in calls),
                        llm_fallbacks=sum(bool(c.get('fallback_reason')) for c in calls)))
                atomic(folder/'supervisor.json',dict(returncode=p.returncode,detail=reason,
                    process_wall_s=now-item['started'],workers=protocol['runtime']['workers'],finished_utc=utc()))
                r=result_row(out,j,protocol);finished.append(r)
                print('episode_finished',j['job_id'],r['termination'],'sim',r['simulated_time_s'],
                    'completed',str(len(finished))+'/'+str(len(jobs)),flush=True)
        if time.time()-last_report>=25 or not (queue or active):
            atomic(out/'progress.json',dict(status='running' if queue or active else ('complete' if len(finished)==len(execution_jobs) else ('cancelled' if (out/'cancel.request').exists() else 'incomplete')),
                execution_scope=scope,completed=len(finished),planned=len(execution_jobs),benchmark_planned=len(jobs),active=[dict(job_id=x['job']['job_id'],**x['watch']) if 'job_id' not in x['watch'] else x['watch'] for x in active],
                queued=len(queue),updated_utc=utc(),elapsed_process_s=time.time()-start))
            build(out);last_report=time.time()
        time.sleep(1)
    check_seals(out,protocol,True)
    stats=build(out,plots=True)
    scope_complete=len(finished)==len(execution_jobs)
    atomic(out/'suite-verification.json',dict(complete=stats['complete'],execution_scope_complete=scope_complete,
        execution_scope=scope,planned=len(execution_jobs),benchmark_planned=len(jobs),completed=len(finished),
        complete_pairs=stats['complete_pairs'],frozen_snapshot_unchanged=True,shared_assets_unchanged=True,
        no_retries=True,protocol_sha256=sha(out/'protocol.json'),engine_fingerprint=protocol['engine_fingerprint'],
        all_result_integrity_checks_passed=all(all(r['integrity_checks'].values()) for r in finished),finished_utc=utc()))
    label=('BENCHMARK_COMPLETE' if stats['complete'] else 'RUN_SCOPE_COMPLETE' if scope_complete else 'BENCHMARK_INCOMPLETE')
    print(label,json.dumps(stats['paired_policy_stats']),flush=True)

def main():
    sys.path.insert(0,str(ROOT/'src'))
    from frontier_search.simulation.sensor_noise import add_noise_arguments,noise_from_arguments,noise_arguments_supplied
    p=argparse.ArgumentParser();p.add_argument('action',choices=('freeze','run','all'));p.add_argument('--output',type=Path,required=True);p.add_argument('--workers',type=int,default=3)
    scope=p.add_mutually_exclusive_group();scope.add_argument('--only-baselines',action='store_true');scope.add_argument('--only-llm',action='store_true');scope.add_argument('--job-id',dest='job_ids',action='append')
    p.add_argument('--arm',choices=sorted(ARMS),default=None)
    add_noise_arguments(p)
    a=p.parse_args();out=a.output.resolve()
    supplied=noise_arguments_supplied(a)
    if a.action=='run' and supplied:
        p.error('noise settings are frozen; pass them to freeze/all in a fresh directory, not run')
    try:
        noise=noise_from_arguments(a) if supplied else None
    except (ValueError, OSError, TypeError) as exc:
        p.error(str(exc))
    if a.action in ('freeze','all'):freeze(out,a.workers,a.arm,noise)
    if a.action in ('run','all'):run(out,a.only_baselines,a.only_llm,a.job_ids)
if __name__=='__main__':main()
