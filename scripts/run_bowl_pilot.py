"""Run a frozen, sequential four-policy bowl pilot with identical depth noise."""
import argparse
from dataclasses import asdict,replace
from datetime import datetime,timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))


def write(path,value):
    path=Path(path)
    temp=path.with_suffix(path.suffix+'.tmp')
    temp.write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf-8')
    temp.replace(path)


def tail_event(path):
    if not path.exists():
        return {}
    with path.open('rb') as stream:
        size=path.stat().st_size
        stream.seek(max(0,size-65536))
        if size>65536:
            stream.readline()
        lines=stream.read().decode('utf-8',errors='replace').splitlines()
    for line in reversed(lines):
        try:
            row=json.loads(line)
        except ValueError:
            continue
        if row.get('event')=='capture_processed':
            return {k:row.get(k) for k in ('sequence','simulated_time_s','known_cells','revision')}
    return {}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--llm-check',type=Path)
    parser.add_argument('--policies',nargs='+',choices=('nearest','information_gain','random','llm_guided'),
                        default=['nearest','information_gain','random','llm_guided'])
    args=parser.parse_args()
    if len(set(args.policies))!=len(args.policies):
        parser.error('Duplicate policies are not allowed')
    with_llm='llm_guided' in args.policies
    if with_llm:
        if args.llm_check is None:
            parser.error('A successful live LLM connectivity check is required')
        connectivity=json.loads(args.llm_check.read_text(encoding='utf-8'))
        if not connectivity.get('live_model_available'):
            parser.error('A successful live LLM connectivity check is required')
    out=args.output.resolve()
    out.mkdir(parents=True,exist_ok=False)
    from frontier_search.config import SensorNoiseConfig
    from frontier_search.policies.provider import load_local_env
    from frontier_search.runner.integrated import task_manifest_for,source_fingerprint
    from frontier_search.simulation.rs_tasks import KITCHEN_BOWL
    if with_llm:
        load_local_env(ROOT/'.env')
    source=source_fingerprint()
    snapshot=out/'frozen'
    shutil.copytree(ROOT/'src',snapshot/'src',ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    (snapshot/'weights/yolo11').mkdir(parents=True)
    shutil.copy2(ROOT/'weights/yolo11/yolo11s.pt',snapshot/'weights/yolo11/yolo11s.pt')
    shutil.copy2(__file__,out/'manager.py')
    assert source_fingerprint()==source,'source changed while freezing'
    noise=SensorNoiseConfig(enabled=True,depth_std_m=.01,depth_dropout_prob=.01,seed=0)
    jobs=[]
    for policy in args.policies:
        manifest=replace(task_manifest_for(policy,policy,'kitchen_bowl'),sensor_noise=noise)
        manifest.validate()
        jobs.append(dict(policy=policy,manifest=json.loads(json.dumps(asdict(manifest)))))
    protocol=dict(created_utc=datetime.now(timezone.utc).isoformat(),
        scope='One episode per policy; noisy bowl pilot, not a statistical benchmark.',
        source_fingerprint=source,snapshot_root=str(snapshot),target=asdict(KITCHEN_BOWL),
        noise=asdict(noise),jobs=jobs,llm_model=os.environ.get('LLM_MODEL'),
        snapshot_seals={p.relative_to(snapshot).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
                        for p in snapshot.rglob('*') if p.is_file()})
    write(out/'protocol.json',protocol)
    if with_llm:
        shutil.copy2(args.llm_check,out/'llm-connectivity.json')
    env=dict(os.environ,PYTHONPATH=str(snapshot/'src'),PYTHONIOENCODING='utf-8',
        PYTHONDONTWRITEBYTECODE='1',YOLO_CONFIG_DIR=str(out/'yolo-config'),
        GIBSON_ASSETS_PATH=str(ROOT/'data/igibson-data/robot_assets/assets'),
        GIBSON_DATASET_PATH=str(ROOT/'data/igibson-data/g_dataset'))
    results=[]
    for job in jobs:
        if (out/'cancel.request').exists():
            break
        policy=job['policy']
        folder=out/policy
        folder.mkdir()
        print('START',policy,flush=True)
        started=time.monotonic()
        command=[sys.executable,'-B','-m','frontier_search.runner.integrated',
            '--task','kitchen_bowl','--profile','apartment','--policy',policy,
            '--noise','--depth-noise-std-m','.01','--depth-dropout-prob','.01','--noise-seed','0',
            '--env-file',str(ROOT/'.env') if with_llm else str(out/'no-llm.env'),'--output',str(folder)]
        supervisor=None
        last_print=0.
        with (folder/'console.log').open('w',encoding='utf-8') as log:
            process=subprocess.Popen(command,cwd=str(ROOT),env=env,stdout=log,stderr=subprocess.STDOUT,
                creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
            write(out/'progress.json',dict(status='running',active=policy,pid=process.pid,completed=len(results)))
            while process.poll() is None:
                elapsed=time.monotonic()-started
                if (out/'cancel.request').exists():
                    (folder/'cancel.request').touch()
                if elapsed>2760:
                    supervisor='process_wall_deadline'
                    process.terminate()
                    process.wait(timeout=30)
                    break
                if elapsed-last_print>=30:
                    progress=dict(status='running',active=policy,pid=process.pid,completed=len(results),
                                  wall_elapsed_s=round(elapsed,1),**tail_event(folder/'events.jsonl'))
                    write(out/'progress.json',progress)
                    print('PROGRESS',json.dumps(progress),flush=True)
                    last_print=elapsed
                time.sleep(1)
        summary_path=folder/'summary.json'
        summary=json.loads(summary_path.read_text(encoding='utf-8')) if summary_path.exists() else None
        if summary:
            assert summary['source_fingerprint']==source and summary['source_unchanged'],'source mismatch'
            assert summary['sensor_noise']==asdict(noise),'noise mismatch'
            assert summary['task']=='kitchen_bowl','task mismatch'
            assert json.loads((folder/'manifest.json').read_text(encoding='utf-8'))==job['manifest'],'manifest mismatch'
        result=dict(policy=policy,returncode=process.returncode,supervisor=supervisor,summary=summary)
        results.append(result)
        write(out/'results.json',dict(completed=len(results),planned=len(jobs),results=results))
        print('DONE',policy,json.dumps(summary or result),flush=True)
    write(out/'progress.json',dict(status='complete' if len(results)==len(jobs) else 'cancelled',completed=len(results)))
    lines=['# Kitchen bowl: Noisy policy pilot','',
        'Each method has only 1 episodes, no statistical superiority claim. Depth noise standard deviation 1 cm, dropout probability 1%, seed 0. ',
        'Shared budget: 600 Simulation s, 18000 control steps, 2400 wall-clock s; fixed start near the bedroom, full-circle initial scan. ','',
        '| Method | Termination reason | Simulation s | Distance / m | Collision ticks | LLM Calls / fallback |',
        '| --- | --- | ---: | ---: | ---: | --- |']
    for result in results:
        s=result['summary'] or {}
        lines.append('| %s | %s | %.2f | %.2f | %s | %s / %s |' % (
            result['policy'],s.get('termination','process_error'),s.get('simulated_time_s',0.),
            s.get('actual_distance_m',0.),s.get('collisions','—'),s.get('llm_calls',0),s.get('llm_fallbacks',0)))
    (out/'SUMMARY.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    return int(any(r['summary'] is None for r in results))


if __name__=='__main__':
    raise SystemExit(main())
