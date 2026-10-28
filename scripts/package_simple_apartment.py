"""Publish the completed apartment-s17 validation without weights, assets or credentials."""
import csv
import hashlib
import json
import os
from pathlib import Path
import shutil
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def read(p):
    return json.loads(p.read_text(encoding='utf-8'))


def main():
    base = ROOT/'runs/a-simple-baselines-20261028'
    llm = ROOT/'runs/a-simple-llm-20261028'
    previous = ROOT/'runs/b-recovered-benchmark-20261024'
    out = ROOT/'validation-report-20261028'
    out.mkdir(exist_ok=False)
    shutil.copy2(llm/'SUMMARY.md',out/'SUMMARY.md')
    shutil.copytree(llm/'report',out/'report')
    shutil.copy2(ROOT/'Documentation/chinese/simple_apartment_validation.md',out/'README.md')
    readme=out/'README.md'
    readme.write_text(readme.read_text(encoding='utf-8')
        .replace('../../validation-report-20261028/report/index.html','report/index.html')
        .replace('(fusion_profiling.md)','(../Documentation/chinese/fusion_profiling.md)'),encoding='utf-8')
    data = out/'data';data.mkdir()
    shutil.copy2(base/'protocol.json',data/'protocol-baselines.json')
    shutil.copy2(llm/'protocol.json',data/'protocol-llm.json')
    shutil.copy2(llm/'connectivity.json',data/'connectivity.json')
    rows = []
    raw = []
    for name,run in [('baselines',base),('llm',llm)]:
        protocol = read(run/'protocol.json')
        snapshot = Path(protocol['snapshot_root'])
        assert all(hashlib.sha256((snapshot/n).read_bytes()).hexdigest()==h
                   for n,h in protocol['snapshot_seals'].items()), 'Frozen source changed'
        for job in protocol['jobs']:
            job_id=job['job_id'];folder=run/'episodes'/job_id
            s=read(folder/'summary.json');old=read(previous/'episodes'/job_id/'summary.json')
            assert s['termination']=='success' and s['evaluation']['w3_valid'] and s['collisions']==0
            assert s['source_unchanged'] and s['source_fingerprint']==protocol['engine_fingerprint']
            target=data/'episodes'/job_id;target.mkdir(parents=True)
            for filename in ['summary.json','final-state.json','evaluation-only.json','llm-calls.json','last-processed-rgb.png']:
                if (folder/filename).exists():shutil.copy2(folder/filename,target/filename)
            shutil.copy2(run/job['manifest_path'],target/'manifest.json')
            shutil.copy2(previous/'episodes'/job_id/'summary.json',target/'previous-summary.json')
            for filename in ['events.jsonl','console.log']:
                if (folder/filename).exists():raw.append((folder/filename,'episodes/'+job_id+'/'+filename))
            rows.append(dict(job_id=job_id,policy=job['policy'],termination=s['termination'],
                simulated_time_s=s['simulated_time_s'],wall_time_s=s['wall_time_s'],distance_m=s['actual_distance_m'],
                collisions=s['collisions'],llm_calls=s['llm_calls'],llm_fallbacks=s['llm_fallbacks'],
                previous_termination=old['termination'],previous_simulated_time_s=old['simulated_time_s'],
                previous_wall_time_s=old['wall_time_s'],
                simulation_change_percent=100*(s['simulated_time_s']/old['simulated_time_s']-1),
                wall_change_percent=100*(s['wall_time_s']/old['wall_time_s']-1)))
    with (data/'episodes.csv').open('w',encoding='utf-8',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    (data/'results.json').write_text(json.dumps(rows,indent=2)+'\n',encoding='utf-8')
    # Check the actual configured credential without displaying or saving it.
    secrets=[os.environ.get('LLM_API_KEY','')]
    env=ROOT/'.env'
    if env.exists():
        for line in env.read_text(encoding='utf-8-sig').splitlines():
            k,sep,v=line.strip().partition('=')
            if sep and k=='LLM_API_KEY':secrets.append(v.strip().strip('\"\''))
    candidates=[p for p in out.rglob('*') if p.is_file()]+[p for p,_ in raw]
    for p in candidates:
        content=p.read_bytes()
        assert not any(value and value.encode() in content for value in secrets), 'Credential found; publication stopped'
    with zipfile.ZipFile(out/'raw-logs.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p,name in raw:z.write(p,name)
    page=out/'report/index.html'
    with page.open('a',encoding='utf-8') as f:
        f.write('<p><a href="../data/episodes.csv">下载 CSV</a> · <a href="../raw-logs.zip">原始日志</a> · <a href="../README.md">验证说明</a></p>')
    seals={p.relative_to(out).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
           for p in out.rglob('*') if p.is_file()}
    (data/'artifact-sha256.json').write_text(json.dumps(seals,indent=2)+'\n',encoding='utf-8')
    print('Packaged',len(rows),'successful native episodes;',len(raw),'raw log files;',
          round((out/'raw-logs.zip').stat().st_size/1024**2,2),'MiB compressed logs')


if __name__=='__main__':main()
