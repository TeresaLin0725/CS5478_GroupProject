"""Observe native batch progress and recover only proven post-terminal exports."""
import argparse,json,time
from pathlib import Path
from recover_benchmark_exports import recover
from benchmark_episode import read

def main():
    p=argparse.ArgumentParser();p.add_argument('run_root',type=Path);a=p.parse_args();root=a.run_root.resolve()
    while True:
        fixed=recover(root);progress=read(root/'progress.json')
        rows=[read(p) for p in (root/'episodes').glob('*/result.json')]
        complete={}
        for r in rows:complete.setdefault(r['pair_id'],[]).append(r)
        pairs=[rs for rs in complete.values() if len(rs)==4]
        stats={policy:dict(n=len(pairs),success=sum(r['success'] for rs in pairs for r in rs if r['policy']==policy))
            for policy in ('random','nearest','information_gain','llm_guided')}
        print(json.dumps(dict(status=progress['status'],completed=progress['completed'],planned=progress['planned'],
            paired=stats,recovered=fixed,active=[dict(job=x['job_id'],simulated_s=x.get('simulated_time_s'),
                decisions=x.get('decisions'),contact_ticks=x.get('collisions')) for x in progress.get('active',[])])),flush=True)
        if progress['status'] in ('complete','cancelled'):break
        time.sleep(45)

if __name__=='__main__':main()
