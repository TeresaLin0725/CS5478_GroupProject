"""Pure pairing and statistics for the frozen native Rs benchmark (Python 3.8)."""
from collections import Counter
from itertools import combinations
from math import comb,sqrt
from statistics import mean,median
import random

POLICIES=('random','nearest','information_gain','llm_guided')
PROFILES=('apartment','clutter','dynamic')
TRIALS=((17,1.5707963267948966),(29,0.0),(43,-1.5707963267948966))

def task_specs():
    rows=[]
    for seed,yaw in TRIALS:
        for profile in PROFILES:
            pair='%s-s%d'%(profile,seed)
            for policy in POLICIES:
                rows.append(dict(job_id=pair+'-'+policy,pair_id=pair,profile=profile,policy=policy,
                    environment_seed=seed,perception_seed=10000+seed,policy_seed=20000+seed,
                    start_pose=dict(x_m=.8,y_m=-2.4,z_m=0.,yaw_rad=yaw)))
    return rows

def validate_pairing(jobs):
    groups={}
    for job in jobs:
        groups.setdefault(job['pair_id'],[]).append(job)
    assert groups and len({j['job_id'] for j in jobs})==len(jobs),'duplicate job'
    for pair,rows in groups.items():
        assert sorted(r['policy'] for r in rows)==sorted(POLICIES),(pair,'policy coverage')
        def common(r):return {k:v for k,v in r.items() if k not in ('job_id','policy','manifest_path','manifest_sha256')}
        assert all(common(r)==common(rows[0]) for r in rows),(pair,'pair mismatch')
    return len(groups)

def wilson(k,n):
    if not n:return None
    z=1.959963984540054;p=k/n;d=1+z*z/n
    center=(p+z*z/(2*n))/d;half=z*sqrt(p*(1-p)/n+z*z/(4*n*n))/d
    return [max(0.,center-half),min(1.,center+half)]

def percentile(values,q):
    if not values:return None
    a=sorted(values);x=(len(a)-1)*q;i=int(x)
    return a[i]+(a[min(i+1,len(a)-1)]-a[i])*(x-i)

def paired_interval(rows,a,b,metric,draws=10000):
    groups={}
    for row in rows:groups.setdefault(row['pair_id'],{})[row['policy']]=row
    strata={}
    for pair,g in groups.items():
        if a in g and b in g:
            strata.setdefault(g[a]['profile'],[]).append(metric(g[a])-metric(g[b]))
    differences=[d for ds in strata.values() for d in ds]
    if not differences:return dict(n=0,mean_difference=None,ci95=None)
    rng=random.Random(20261018);samples=[]
    for _ in range(draws):
        samples.append(mean(d for ds in strata.values() for d in (rng.choice(ds) for _ in ds)))
    return dict(n=len(differences),mean_difference=mean(differences),
                ci95=[percentile(samples,.025),percentile(samples,.975)])

def exact_mcnemar(rows,a,b):
    pairs={}
    for r in rows:pairs.setdefault(r['pair_id'],{})[r['policy']]=r['success']
    wins=sum(g[a] and not g[b] for g in pairs.values() if a in g and b in g)
    losses=sum(g[b] and not g[a] for g in pairs.values() if a in g and b in g)
    n=wins+losses
    p=min(1.,2*sum(comb(n,k) for k in range(min(wins,losses)+1))/(2**n)) if n else 1.
    return dict(a_only_success=wins,b_only_success=losses,discordant_pairs=n,p_value=p)

def policy_stats(rows,budget=600.):
    result={}
    for policy in POLICIES:
        rs=[r for r in rows if r['policy']==policy];good=[r for r in rs if r['success']]
        times=[r['simulated_time_s'] for r in rs if r.get('simulated_time_s') is not None]
        paths=[r['actual_distance_m'] for r in rs if r.get('actual_distance_m') is not None]
        calls=sum(r.get('llm_calls',0) for r in rs);fallbacks=sum(r.get('llm_fallbacks',0) for r in rs)
        latencies=[x for r in rs for x in r.get('llm_latencies_s',[])]
        result[policy]=dict(n=len(rs),successes=len(good),success_rate=len(good)/len(rs) if rs else None,
            success_ci95_wilson=wilson(len(good),len(rs)),terminations=dict(Counter(r['termination'] for r in rs)),
            false_claims=sum(r['termination']=='false_target_claim' for r in rs),
            mean_simulated_time_s=mean(times) if times else None,
            mean_penalized_time_s=mean(r['simulated_time_s'] if r['success'] else budget for r in rs) if rs else None,
            mean_actual_distance_m=mean(paths) if paths else None,
            success_mean_time_s=mean(r['simulated_time_s'] for r in good) if good else None,
            success_mean_distance_m=mean(r['actual_distance_m'] for r in good) if good else None,
            mean_wall_time_s=mean(r['wall_time_s'] for r in rs) if rs else None,
            contact_ticks=sum(r.get('collisions',0) or 0 for r in rs),
            episodes_with_contact=sum((r.get('collisions',0) or 0)>0 for r in rs),
            replans=sum(r.get('local_replans',0) for r in rs),
            no_progress_attempts=sum(r.get('no_progress_attempts',0) for r in rs),
            frontier_visits=sum(r.get('frontier_visits',0) for r in rs),
            llm_calls=calls,llm_fallbacks=fallbacks,fallback_rate=fallbacks/calls if calls else None,
            llm_latency_median_s=median(latencies) if latencies else None,
            llm_latency_p95_s=percentile(latencies,.95),
            llm_tokens_total=sum(r.get('llm_tokens_total',0) for r in rs),
            recorded_cost_total=sum(r.get('recorded_cost_total',0) for r in rs),
            calls_with_recorded_cost=sum(r.get('calls_with_recorded_cost',0) for r in rs))
    return result

def aggregate(rows,jobs,budget=600.,draws=10000):
    expected={j['job_id']:j for j in jobs}
    assert len({r['job_id'] for r in rows})==len(rows),'duplicate result'
    assert all(r['job_id'] in expected for r in rows),'unexpected result'
    by_pair={}
    for r in rows:by_pair.setdefault(r['pair_id'],[]).append(r)
    complete=[pair for pair,rs in by_pair.items() if {r['policy'] for r in rs}==set(POLICIES)]
    paired=[r for r in rows if r['pair_id'] in complete]
    comparisons=[]
    for a,b in combinations(POLICIES,2):
        both=[r for r in paired if all(g['success'] for g in by_pair[r['pair_id']] if g['policy'] in (a,b))]
        comparisons.append(dict(a=a,b=b,
            success_difference=paired_interval(paired,a,b,lambda r:float(r['success']),draws),
            penalized_time_difference_s=paired_interval(paired,a,b,
                lambda r:r['simulated_time_s'] if r['success'] else budget,draws),
            jointly_successful_time_difference_s=paired_interval(both,a,b,lambda r:r['simulated_time_s'],draws),
            jointly_successful_distance_difference_m=paired_interval(both,a,b,lambda r:r['actual_distance_m'],draws),
            exact_mcnemar=exact_mcnemar(paired,a,b)))
    previous=0.
    for rank,c in enumerate(sorted(comparisons,key=lambda c:c['exact_mcnemar']['p_value'])):
        previous=max(previous,min(1.,(len(comparisons)-rank)*c['exact_mcnemar']['p_value']))
        c['exact_mcnemar']['p_value_holm']=previous
    return dict(planned=len(jobs),completed=len(rows),complete=len(rows)==len(jobs),
        complete_pairs=len(complete),planned_pairs=validate_pairing(jobs),
        completed_policy_stats=policy_stats(rows,budget),
        paired_policy_stats=policy_stats(paired,budget),
        per_profile={p:policy_stats([r for r in paired if r['profile']==p],budget) for p in PROFILES},
        paired_comparisons=comparisons,results=rows,
        uncertainty='Wilson intervals are descriptive; paired percentile bootstrap uses 10000 resamples within each profile over seed/heading blocks. These are not multi-house generalization intervals.')
