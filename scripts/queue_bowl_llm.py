"""Append an explicitly authorized LLM trial after the local baseline queue."""
import argparse
from datetime import datetime,timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def write(path,value):
    temp=path.with_suffix('.tmp')
    temp.write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf-8')
    temp.replace(path)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-root',type=Path,required=True)
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    out=args.run_root.resolve()
    protocol=json.loads((out/'protocol.json').read_text())
    snapshot=Path(protocol['snapshot_root'])
    sys.path.insert(0,str(snapshot/'src'))
    from frontier_search.policies.provider import load_local_env
    load_local_env(root/'.env')
    authorization=dict(authorized=True,authorized_utc=datetime.now(timezone.utc).isoformat(),
        user_instruction='I want a live report. Also allow LLM.',
        destination='https://generativelanguage.googleapis.com/v1beta/openai/chat/completions',
        payload='Simulated target class, observed objects and coordinates, robot state, candidate search points and execution feedback',
        model=os.environ.get('LLM_MODEL','gemini-3.5-flash'))
    write(out/'llm-authorization.json',authorization)
    folder=out/'llm_guided'
    if folder.exists():
        raise FileExistsError('Never overwrite an LLM trial')
    write(out/'llm-progress.json',dict(status='waiting_for_baselines',authorized=True,model=authorization['model']))
    print('LLM authorized; waiting for the three baselines to finish.',flush=True)
    while True:
        if (out/'cancel.request').exists():
            write(out/'llm-progress.json',dict(status='cancelled_before_start',authorized=True))
            return 0
        progress=json.loads((out/'progress.json').read_text())
        if progress.get('status')=='complete':
            break
        if progress.get('status')=='cancelled':
            write(out/'llm-progress.json',dict(status='cancelled_before_start',authorized=True))
            return 0
        time.sleep(3)
    for name,digest in protocol['snapshot_seals'].items():
        assert hashlib.sha256((snapshot/name).read_bytes()).hexdigest()==digest,'Frozen source changed'
    folder.mkdir()
    env=dict(os.environ,PYTHONPATH=str(snapshot/'src'),PYTHONIOENCODING='utf-8',PYTHONDONTWRITEBYTECODE='1',
        YOLO_CONFIG_DIR=str(out/'yolo-config'),GIBSON_ASSETS_PATH=str(root/'data/igibson-data/robot_assets/assets'),
        GIBSON_DATASET_PATH=str(root/'data/igibson-data/g_dataset'))
    command=[sys.executable,'-B','-m','frontier_search.runner.integrated','--task','kitchen_bowl',
        '--profile','apartment','--policy','llm_guided','--noise','--depth-noise-std-m','.01',
        '--depth-dropout-prob','.01','--noise-seed','0','--env-file',str(root/'.env'),'--output',str(folder)]
    started=time.monotonic()
    with (folder/'console.log').open('w',encoding='utf-8') as log:
        process=subprocess.Popen(command,cwd=str(root),env=env,stdout=log,stderr=subprocess.STDOUT,
            creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        while process.poll() is None:
            elapsed=time.monotonic()-started
            write(out/'llm-progress.json',dict(status='running',authorized=True,pid=process.pid,
                wall_elapsed_s=round(elapsed,1),model=authorization['model']))
            if (out/'cancel.request').exists():
                (folder/'cancel.request').touch()
            if elapsed>2760:
                process.terminate()
                process.wait(timeout=30)
                break
            time.sleep(3)
    summary_path=folder/'summary.json'
    summary=json.loads(summary_path.read_text()) if summary_path.exists() else None
    if summary:
        assert summary['source_fingerprint']==protocol['source_fingerprint'] and summary['source_unchanged']
        assert summary['sensor_noise']==protocol['noise']
        manifest=json.loads((folder/'manifest.json').read_text())
        expected=protocol['jobs'][0]['manifest']
        assert manifest['target_class']==expected['target_class'] and manifest['start_pose']==expected['start_pose']
        assert manifest['limits']==expected['limits'] and manifest['observation']==expected['observation']
    write(out/'llm-results.json',dict(policy='llm_guided',returncode=process.returncode,summary=summary))
    write(out/'llm-progress.json',dict(status='complete' if summary else 'process_error',authorized=True,
        model=authorization['model'],termination=summary.get('termination') if summary else None))
    print('LLM trial complete:',summary.get('termination') if summary else 'process_error',flush=True)
    return int(summary is None)


if __name__=='__main__':
    raise SystemExit(main())
