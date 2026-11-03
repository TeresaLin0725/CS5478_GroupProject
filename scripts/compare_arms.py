"""Compare ablation arms of the paired benchmark, job by job (Python 3.8, standard library only).

Each argument is ``LABEL=path`` to a run root that holds ``episodes.csv``. Jobs are paired across arms by ``job_id``
(profile x seed x policy), which the frozen task list keeps identical in every arm, so A0 (the original batch) pairs
with A1, A2 and A3. Failures are never dropped.
"""
import argparse,csv,json,sys
from math import comb
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from benchmark_protocol import wilson


def load(root):
    with open(Path(root)/'episodes.csv',encoding='utf-8-sig',newline='') as f:
        rows=list(csv.DictReader(f))
    return {r['job_id']:dict(r,success=str(r['success']).lower()=='true',collisions=int(float(r['collisions'] or 0)))
            for r in rows}


def exact_two_sided(b,c):
    """Exact McNemar p-value from the discordant counts (b: only the first succeeded, c: only the second)."""
    n=b+c
    if n==0:return 1.0
    tail=sum(comb(n,i) for i in range(min(b,c)+1))/2**n
    return min(1.0,2*tail)


def summarize(rows):
    n=len(rows);k=sum(r['success'] for r in rows.values())
    by_policy={}
    for r in rows.values():
        d=by_policy.setdefault(r['policy'],[0,0]);d[0]+=r['success'];d[1]+=1
    return dict(n=n,successes=k,rate=k/n if n else None,wilson95=wilson(k,n) if n else None,
        no_feasible_frontiers=sum(r['termination']=='no_feasible_frontiers' for r in rows.values()),
        collisions=sum(r['collisions'] for r in rows.values()),
        by_policy={p:'%d/%d'%tuple(v) for p,v in sorted(by_policy.items())})


def compare(base,other):
    """Paired comparison of ``other`` against ``base`` over the jobs both contain."""
    shared=sorted(set(base)&set(other))
    gained=[j for j in shared if other[j]['success'] and not base[j]['success']]
    lost=[j for j in shared if base[j]['success'] and not other[j]['success']]
    return dict(paired_jobs=len(shared),only_other_succeeded=len(gained),only_base_succeeded=len(lost),
        net=len(gained)-len(lost),mcnemar_p=exact_two_sided(len(lost),len(gained)),gained=gained,lost=lost,
        missing_from_other=sorted(set(base)-set(other)))


def report(arms):
    labels=list(arms);base=arms[labels[0]]
    out=dict(baseline=labels[0],summaries={l:summarize(r) for l,r in arms.items()},
             versus_baseline={l:compare(base,arms[l]) for l in labels[1:]})
    return out


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('arms',nargs='+',metavar='LABEL=ROOT',help='first one is the baseline, e.g. A0=report/evidence/benchmark')
    p.add_argument('--json',type=Path)
    a=p.parse_args()
    arms={}
    for item in a.arms:
        label,_,root=item.partition('=')
        if not root:p.error('expected LABEL=ROOT, got %r'%item)
        arms[label]=load(root)
    result=report(arms)
    for label,s in result['summaries'].items():
        print('%-3s success %d/%d (%.1f%%, Wilson95 %.1f-%.1f%%)  no_feasible_frontiers %d  contact steps %d  %s'%(
            label,s['successes'],s['n'],100*s['rate'],100*s['wilson95'][0],100*s['wilson95'][1],
            s['no_feasible_frontiers'],s['collisions'],s['by_policy']))
    for label,c in result['versus_baseline'].items():
        print('%s vs %s: %d paired jobs; only %s succeeded %d, only %s succeeded %d, exact McNemar p=%.3f'%(
            label,result['baseline'],c['paired_jobs'],label,c['only_other_succeeded'],
            result['baseline'],c['only_base_succeeded'],c['mcnemar_p']))
    if a.json:a.json.write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')


if __name__=='__main__':main()
