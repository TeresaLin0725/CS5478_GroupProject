"""Sequential apartment-s17 baseline validation; completed results survive resume."""
import argparse
from dataclasses import asdict, replace
from datetime import datetime, timezone
import hashlib
import html
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src'), str(ROOT/'scripts')]
from benchmark_episode import atomic, navigation_options, validate_shared_geometry
from benchmark_watchdog import WatchdogChannel


def read(p):
    return json.loads(Path(p).read_text(encoding='utf-8'))


def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def utc():
    return datetime.now(timezone.utc).isoformat()


def freeze(out):
    from frontier_search.runner.integrated import shared_round_manifest_for, source_fingerprint
    from frontier_search.policies.provider import load_local_env
    env_file = ROOT/'.env'
    load_local_env(env_file)
    snapshot = out/'frozen'
    shutil.copytree(ROOT/'src', snapshot/'src', ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
    (snapshot/'weights/yolo11').mkdir(parents=True)
    shutil.copy2(ROOT/'weights/yolo11/yolo11s.pt', snapshot/'weights/yolo11/yolo11s.pt')
    (snapshot/'harness').mkdir()
    for name in ('benchmark_episode.py', 'benchmark_watchdog.py'):
        shutil.copy2(ROOT/'scripts'/name, snapshot/'harness'/name)
    base = replace(shared_round_manifest_for('template'), environment_seed=17, perception_seed=10017,
                   policy_seed=20017, initialization_deadline_s=240.)
    base = replace(base, limits=replace(base.limits, max_wall_time_s=7200.))
    jobs = []
    for policy in ('random', 'nearest', 'information_gain'):
        job_id = 'apartment-s17-' + policy
        m = replace(base, episode_id=job_id, policy_name=policy)
        m.validate()
        path = Path('manifests')/(job_id+'.json')
        atomic(out/path, asdict(m))
        jobs.append(dict(job_id=job_id, pair_id='apartment-s17', profile='apartment', policy=policy,
                         environment_seed=17, perception_seed=10017, policy_seed=20017,
                         start_pose=asdict(m.start_pose), manifest_path=path.as_posix(), manifest_sha256=sha(out/path)))
    assets = ROOT/'data/igibson-data'
    protocol = dict(schema_version='simple-baselines-v2-shared-round', created_utc=utc(), run_root=str(out),
        snapshot_root=str(snapshot), env_file=str(env_file), engine_fingerprint=source_fingerprint(),
        source_commit=subprocess.check_output(['git','rev-parse','HEAD'], cwd=str(ROOT), text=True).strip(),
        scope='Three sequential native apartment-s17 baselines; no LLM or video; not a full benchmark.',
        navigation_preset='shared_round', jobs=jobs, base_manifest=asdict(base),
        runtime=dict(workers=1, torch_threads=4, device='0', startup_deadline_s=300.),
        asset_paths=dict(robot_assets=str(assets/'robot_assets/assets'), scenes=str(assets/'g_dataset')),
        watchdog_transport='shared_memory_v1',
        previous_run=str(ROOT/'runs/b-recovered-benchmark-20261024'),
        snapshot_seals={p.relative_to(snapshot).as_posix():sha(p) for p in snapshot.rglob('*') if p.is_file()})
    # Resolve and validate shared geometry before any simulator is launched.
    validate_shared_geometry(protocol, base)
    assert navigation_options(protocol) == dict(start_margin_m=0.)
    atomic(out/'protocol.json', protocol)
    shutil.copy2(__file__, out/'manager.py')
    return protocol


def results(out, protocol):
    rows = []
    for job in protocol['jobs']:
        p = out/'episodes'/job['job_id']/'summary.json'
        if p.exists():
            s = read(p)
            if s['termination'] != 'externally_cancelled':
                rows.append(dict(job=job, summary=s))
    return rows


def report(out, protocol, status, active=None):
    rows = results(out, protocol)
    atomic(out/'results.json', dict(status=status, completed=len(rows), planned=3, results=rows))
    atomic(out/'progress.json', dict(status=status, completed=len(rows), planned=3, active=active, updated_utc=utc()))
    labels = dict(random='Random policy', nearest='Nearest frontier', information_gain='Information gain')
    outcomes = dict(success='Success', search_limit='Search or time limit', no_feasible_frontiers='No reachable frontiers',
                    no_frontiers='No frontiers', pipeline_error='Pipeline error')
    geometry = 'A/B Shared round footprint' if protocol.get('navigation_preset') == 'shared_round' else 'B Round-footprint preset'
    lines = ['# Three-policy apartment validation', '', 'Single process; '+geometry+'; 600 simulation s, 18000 steps, 7200 s total wall-clock limit; Per operation 400 s. ',
             '', 'Status: '+status, '', '| Policy | Current outcome | Current simulation s | Current wall-clock s | Current distance m | Previous outcome | Previous simulation s | Previous wall-clock s |',
             '|---|---|---:|---:|---:|---|---:|---:|']
    cells = []
    for row in rows:
        j, s = row['job'], row['summary']
        old = read(Path(protocol['previous_run'])/'episodes'/j['job_id']/'summary.json')
        values = [labels[j['policy']], outcomes.get(s['termination'],s['termination']),
                  '%.1f'%s['simulated_time_s'], '%.1f'%s['wall_time_s'], '%.2f'%s['actual_distance_m'],
                  outcomes.get(old['termination'],old['termination']), '%.1f'%old['simulated_time_s'], '%.1f'%old['wall_time_s']]
        lines.append('| '+' | '.join(values)+' |')
        cells.append('<tr>'+''.join('<td>'+html.escape(v)+'</td>' for v in values)+'</tr>')
    lines += ['', 'Differences from the earlier batch include A frontier/target-verification changes and concurrency differences, speed changes cannot be attributed solely to fusion optimization. ',
              'Cancelled unfinished episodes are archived separately, excluded from the results; resume unfinished episodes from their starting pose, retain completed outcomes. ']
    (out/'SUMMARY.md').write_text('\n'.join(lines)+'\n', encoding='utf-8')
    report_dir = out/'report'; report_dir.mkdir(exist_ok=True)
    page = '<!doctype html><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">'
    if status == 'running': page += '<meta http-equiv="refresh" content="30">'
    page += '<title>Three-policy apartment validation</title><style>body{font:17px "Microsoft YaHei",sans-serif;background:#101a25;color:#e1ebf5;padding:32px}td,th{padding:14px;border-bottom:1px solid #354759}table{border-collapse:collapse}a{color:#67d9ba}</style>'
    page += '<h1>Three-policy apartment validation</h1><p>Status: '+html.escape(status)+'; Complete '+str(len(rows))+'/3</p>'
    page += '<p>Single process · '+geometry+' · Simulation budget 600 s · Wall-clock limit 7200 s</p>'
    if active: page += '<p>Current: '+html.escape(active['job_id'])+' · simulation '+str(active.get('simulated_time_s',0))+' s</p>'
    page += '<div style="overflow:auto"><table><tr>'+''.join('<th>'+v+'</th>' for v in ['Policy','Current outcome','Current simulation s','Current wall-clock s','Current distance m','Previous outcome','Previous simulation s','Previous wall-clock s'])+'</tr>'+''.join(cells)+'</table></div>'
    page += '<p>The earlier batch used concurrency and A perception configurations differ, not a controlled benchmark of fusion optimization. </p><a href="../SUMMARY.md">Details</a>'
    (report_dir/'index.html').write_text(page, encoding='utf-8')


def run(out, protocol):
    snap = Path(protocol['snapshot_root'])
    assert all(sha(snap/n)==v for n,v in protocol['snapshot_seals'].items()), 'Frozen snapshot changed'
    env = dict(os.environ, PYTHONPATH=str(snap/'src'), PYTHONIOENCODING='utf-8',
               PYTHONDONTWRITEBYTECODE='1', YOLO_CONFIG_DIR=str(out/'yolo-config'))
    for job in protocol['jobs']:
        if (out/'cancel.request').exists(): break
        folder = out/'episodes'/job['job_id']
        if (folder/'summary.json').exists() and read(folder/'summary.json')['termination'] != 'externally_cancelled': continue
        if folder.exists():
            archive = out/'interrupted'/job['job_id']/str(time.time_ns())
            folder.resolve().relative_to(out.resolve())
            archive.resolve().relative_to(out.resolve())
            archive.parent.mkdir(parents=True, exist_ok=True)
            folder.rename(archive)
        folder.mkdir(parents=True)
        start, channel, watch, last_report, cancel_started = time.time(), None, {}, 0., None
        print('START', job['job_id'], flush=True)
        with (folder/'console.log').open('w', encoding='utf-8') as log:
            proc = subprocess.Popen([sys.executable,'-B',str(snap/'harness/benchmark_episode.py'),
                '--protocol',str(out/'protocol.json'),'--job',job['job_id']], cwd=str(ROOT), env=env,
                stdout=log, stderr=subprocess.STDOUT, creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
            try:
                while proc.poll() is None:
                    now = time.time()
                    if channel is None:
                        try: channel = WatchdogChannel(folder)
                        except FileNotFoundError: pass
                    if channel is not None:
                        try: watch = channel.read() or watch
                        except (ValueError,BufferError): pass
                    reason = None
                    if not watch and now-start > 300: reason = 'startup_deadline'
                    if watch.get('deadline_at') and now > watch['deadline_at']+5: reason = 'module_deadline'
                    if now-start > 7560: reason = 'process_wall_deadline'
                    if (out/'cancel.request').exists():
                        cancel_started = cancel_started or now
                        if now-cancel_started > 90: reason = 'user_cancel_deadline'
                    if now-last_report > 10:
                        report(out, protocol, 'running', dict(watch, job_id=job['job_id'], process_elapsed_s=now-start))
                        last_report = now
                    if reason:
                        proc.terminate(); proc.wait(timeout=30)
                        atomic(folder/'supervisor.json', dict(reason=reason, elapsed_s=now-start))
                        break
                    time.sleep(1)
            finally:
                if proc.poll() is None:
                    proc.terminate(); proc.wait(timeout=30)
                if channel is not None: channel.close()
        if not (folder/'summary.json').exists():
            if (out/'cancel.request').exists(): break
            report(out, protocol, 'pipeline_error', dict(watch,job_id=job['job_id']))
            raise RuntimeError('Worker ended without summary; stopping suite: '+job['job_id'])
        summary = read(folder/'summary.json')
        assert summary['source_unchanged'] and summary['source_fingerprint']==protocol['engine_fingerprint']
        print('FINISH', job['job_id'], summary['termination'], flush=True)
        if summary['termination']=='externally_cancelled': break
        if summary['termination'] in ('pipeline_error','initialization_failed'):
            report(out, protocol, 'pipeline_error'); return
    report(out, protocol, 'complete' if len(results(out,protocol))==len(protocol['jobs']) else 'paused_by_user')


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--resume',action='store_true')
    a = p.parse_args(); out = a.output.resolve()
    out.relative_to(ROOT.resolve())
    out.mkdir(parents=True,exist_ok=True)
    with (out/'run.lock').open('x',encoding='utf-8') as f: json.dump(dict(pid=os.getpid(),started_utc=utc()),f)
    try:
        if (out/'protocol.json').exists():
            if not a.resume: raise RuntimeError('Existing batch requires explicit --resume')
            protocol = read(out/'protocol.json')
            if (out/'cancel.request').exists(): (out/'cancel.request').unlink()
        else:
            if a.resume: raise RuntimeError('Cannot resume a batch without protocol')
            protocol = freeze(out)
        report(out, protocol, 'running')
        run(out, protocol)
    finally:
        (out/'run.lock').unlink()


if __name__=='__main__':
    main()
