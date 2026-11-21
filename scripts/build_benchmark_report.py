"""Evidence-linked, self-contained live/final benchmark report and scientific plots."""
import argparse,csv,html,json
from pathlib import Path
from datetime import datetime,timezone
from benchmark_protocol import POLICIES,PROFILES,aggregate,policy_stats
from benchmark_episode import atomic,replace_shared

LABELS={'random':'Random','nearest':'Nearest','information_gain':'Information Gain','llm_guided':'LLM-guided'}
SCENES={'apartment':'Original apartment','clutter':'Cluttered apartment','dynamic':'Dynamic obstacles'}
def read(p):return json.loads(Path(p).read_text(encoding='utf-8'))
def number(x,d=1):return '—' if x is None else ('%.*f'%(d,x))
def percent(x):return '—' if x is None else '%.1f%%'%(100*x)
def interval(xs,scale=1):return '—' if xs is None else '[%.1f, %.1f]'%(xs[0]*scale,xs[1]*scale)

def collect(root,protocol):
    rows=[]
    for job in protocol['jobs']:
        path=root/'episodes'/job['job_id']/'result.json'
        if path.is_file():rows.append(read(path))
    return rows

def live_status_panel(root,protocol,stats,rows):
    """Show completed baselines before the four-policy paired table is populated."""
    launch=read(root/'launch-status.json') if (root/'launch-status.json').exists() else {}
    phase=html.escape(str(launch.get('phase','benchmark')))
    detail=html.escape(str(launch.get('detail','Results refresh while the queue is running.')))
    if stats['complete']:
        phase='complete'
        detail='All planned episodes and paired task blocks are complete.'
    panel='<section><h2>Live execution status</h2><p><strong>'+phase+'</strong> · '+detail+'</p>'
    panel+='<p>Completed episodes: '+str(stats['completed'])+' / '+str(stats['planned'])+'. Updates approximately every 25 seconds; this page refreshes every 30 seconds.</p>'
    panel+='<table><thead><tr><th>Policy</th><th>Finished / planned</th><th>Verified successes / finished</th><th>LLM calls / fallbacks</th></tr></thead><tbody>'
    for policy in POLICIES:
        s=stats['completed_policy_stats'][policy]
        planned=sum(j['policy']==policy for j in protocol['jobs'])
        panel+='<tr><td>'+LABELS[policy]+'</td><td>'+str(s['n'])+' / '+str(planned)+'</td><td>'+str(s['successes'])+' / '+str(s['n'])+'</td><td>'+str(s['llm_calls'])+' / '+str(s['llm_fallbacks'])+'</td></tr>'
    panel+='</tbody></table><p class="note">These are descriptive counts of completed episodes. The paired comparison below requires all four policies in a task block.</p>'
    model=protocol.get('base_manifest',{}).get('policy',{}).get('llm_model','unspecified')
    panel+='<p>Configured LLM: <code>'+html.escape(str(model))+'</code>. Task success can include fallback behavior; it does not establish that the model worked.</p>'
    target=protocol.get('target_evaluator_only',{})
    start=protocol.get('base_manifest',{}).get('start_pose',{})
    if protocol.get('design')=='bowl48':
        panel+='<p>Target: bowl, kitchen or bathroom (one per episode). Starts: A original living room, B bedroom doorway. Fixed noise seed 140; 12 paired task blocks.</p>'
    else:
        panel+='<p>Target: <strong>'+html.escape(target.get('class_name','cup'))+'</strong> · start: ('+str(start.get('x_m'))+', '+str(start.get('y_m'))+') m. '+('Fixed initial heading; three seed blocks.' if protocol.get('fixed_start_heading') else 'Three seed/heading blocks.')+'</p>'
    invalid=[r['job_id'] for r in rows if r['policy']=='llm_guided'
             and r.get('llm_calls',0)>0 and r.get('llm_fallbacks',0)>=r['llm_calls']]
    if invalid:
        panel+='<p style="color:#f3b186"><strong>LLM validity warning:</strong> all hosted calls fell back in: '+html.escape(', '.join(invalid))+'. Do not treat those episodes as valid hosted-LLM comparisons.</p>'
    return panel+'</section>'

def build(root,plots=False):
    root=Path(root).resolve();protocol=read(root/'protocol.json');rows=collect(root,protocol)
    stats=aggregate(rows,protocol['jobs'],draws=10000 if plots else 1000)
    if protocol.get('design')=='bowl48':
        stats['per_target']={t:policy_stats([r for r in rows if r['target_location']==t]) for t in ('kitchen','bathroom')}
        stats['per_start']={s:policy_stats([r for r in rows if r['start_id']==s]) for s in ('A','B')}
        stats['uncertainty']='Descriptive fixed-house results; paired bootstrap within profile over target/start blocks. Not independent-house generalization.'
    stats.update(protocol_hash=(root/'protocol.sha256').read_text().strip(),engine_fingerprint=protocol['engine_fingerprint'],
        scope=protocol['scope'],generated_utc=datetime.now(timezone.utc).isoformat(),
        sensor_noise=protocol.get('sensor_noise'))
    atomic(root/'results.json',stats)
    flat=[{k:v for k,v in r.items() if isinstance(v,(str,int,float,bool)) or v is None} for r in rows]
    if flat:
        with (root/'episodes.csv.tmp').open('w',encoding='utf-8-sig',newline='') as f:
            w=csv.DictWriter(f,fieldnames=sorted({k for row in flat for k in row}));w.writeheader();w.writerows(flat)
        replace_shared(root/'episodes.csv.tmp',root/'episodes.csv')
    report=root/'report';report.mkdir(exist_ok=True)
    if plots:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        import numpy as np
        ps=stats['paired_policy_stats'];xs=np.arange(4)
        fig,axes=plt.subplots(1,2,figsize=(11,4),layout='constrained')
        rates=[(ps[p]['success_rate'] or 0)*100 for p in POLICIES]
        lows=[(ps[p]['success_ci95_wilson'] or [0,0])[0]*100 for p in POLICIES]
        highs=[(ps[p]['success_ci95_wilson'] or [0,0])[1]*100 for p in POLICIES]
        axes[0].bar(xs,rates,color=['#93a7c7','#5e9bbb','#8d85b7','#46ab95'])
        axes[0].errorbar(xs,rates,yerr=[np.maximum(0,np.array(rates)-lows),np.maximum(0,np.array(highs)-rates)],fmt='none',ecolor='#26394d',capsize=4)
        axes[0].set(ylabel='Success (%)',ylim=(0,105),title='Verified success / all paired episodes')
        axes[1].bar(xs,[ps[p]['mean_penalized_time_s'] or 0 for p in POLICIES],color=['#93a7c7','#5e9bbb','#8d85b7','#46ab95'])
        axes[1].set(ylabel='Simulated seconds',title='Penalized search time (failure = 600 s)')
        for ax in axes:
            ax.set_xticks(xs,[LABELS[p].replace('Information Gain','IG') for p in POLICIES]);ax.spines[['top','right']].set_visible(False)
            ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
        if protocol.get('design')=='bowl48':
            subtitle='Rs: 2 bowl locations x 2 starts x 3 conditions; fixed noise seed 140'
        else:
            subtitle='Rs: 3 conditions x 3 '+('seed blocks; fixed initial heading' if protocol.get('fixed_start_heading') else 'seed/heading blocks; same native pipeline')
        fig.suptitle(subtitle,fontsize=11)
        for ext in ('png','svg'):fig.savefig(report/('performance.'+ext),dpi=180)
        plt.close(fig)
    primary=stats['paired_policy_stats'];cards=''
    for p in POLICIES:
        s=primary[p];cards+='<div class="metric"><small>'+LABELS[p]+'</small><strong>'+str(s['successes'])+'/'+str(s['n'])+'</strong><span>'+percent(s['success_rate'])+' Success</span></div>'
    table=''
    for p in POLICIES:
        s=primary[p]
        table+='<tr>'+''.join('<td>'+str(x)+'</td>' for x in [LABELS[p],str(s['successes'])+'/'+str(s['n']),percent(s['success_rate']),interval(s['success_ci95_wilson'],100),
            number(s['mean_penalized_time_s']),number(s['success_mean_time_s'])+' (n='+str(s['successes'])+')',number(s['success_mean_distance_m'],2),
            s['contact_ticks'],s['llm_calls'],s['llm_fallbacks']])+'</tr>'
    scenes=''
    for profile in PROFILES:
        scenes+='<tr><td>'+SCENES[profile]+'</td>'+''.join('<td>%d/%d</td>'%(stats['per_profile'][profile][p]['successes'],stats['per_profile'][profile][p]['n']) for p in POLICIES)+'</tr>'
    comparisons=''
    for c in stats['paired_comparisons']:
        d=c['success_difference'];t=c['penalized_time_difference_s'];mc=c['exact_mcnemar']
        comparisons+='<tr>'+''.join('<td>'+str(x)+'</td>' for x in [LABELS[c['a']]+' − '+LABELS[c['b']],d['n'],number(d['mean_difference']*100 if d['mean_difference'] is not None else None),interval(d['ci95'],100),number(t['mean_difference']),interval(t['ci95']),number(mc['p_value_holm'],3)])+'</tr>'
    details=''
    for r in rows:
        folder='../episodes/'+r['job_id']+'/'
        status='Success' if r['success'] else r['termination']
        details+='<tr data-policy="'+r['policy']+'"><td>'+r['pair_id']+'</td><td>'+LABELS[r['policy']]+'</td><td class="'+('good' if r['success'] else 'bad')+'">'+status+'</td><td>'+number(r.get('simulated_time_s'))+'</td><td>'+number(r.get('actual_distance_m'),2)+'</td><td>'+str(r.get('collisions',0))+'</td><td>'+str(r.get('llm_calls',0))+' / '+str(r.get('llm_fallbacks',0))+'</td><td><a href="'+folder+'summary.json">Summary</a> · <a href="'+folder+'events.jsonl">log</a> · <a href="'+folder+'result.json">verification</a></td></tr>'
    active=''
    progress=read(root/'progress.json') if (root/'progress.json').exists() else {}
    for item in progress.get('active',[]):
        active+='<li>'+html.escape(item['job_id'])+' · '+number(item.get('simulated_time_s'))+' s · '+str(item.get('steps',0))+' steps · '+html.escape(item.get('pending_module') or 'running')+'</li>'
    refresh='<meta http-equiv="refresh" content="30">' if not stats['complete'] else ''
    image='<img class="plot" src="performance.png" alt="Paired success rate and failure-penalized search time"><p><a href="performance.svg">Export SVG figure</a></p>' if (report/'performance.png').exists() else ''
    state='Complete' if stats['complete'] else 'In progress'
    page='''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">__REFRESH__<title>FRONTIER · Paired benchmark performance</title>
<style>:root{color-scheme:dark}*{box-sizing:border-box}body{margin:0;background:#0a111c;color:#dce7f4;font:14px/1.8 "Segoe UI","Microsoft YaHei",sans-serif}main{max-width:1500px;margin:auto;padding:32px}header{border-bottom:1px solid #2b3b4f;margin-bottom:24px}h1{font-size:34px;font-weight:550}h2{font-size:21px;font-weight:500;margin:0 0 16px}small,.note{color:#91a5bd}.eyebrow{color:#78e4c4;font:12px Consolas,monospace;letter-spacing:.15em}section{background:#111d2c;border:1px solid #2b3b4f;border-radius:12px;padding:24px;margin:24px 0}.cards{display:grid;grid-template-columns:repeat(4,1fr);gap:16px}.metric{border-left:2px solid #78e4c4;padding:8px 20px}.metric>*{display:block}.metric strong{font-size:30px}.metric span{color:#a2c9bf}a{color:#7fdfc0;text-decoration:none}a:hover{text-decoration:underline}.scroll{overflow:auto}table{border-collapse:collapse;width:100%;font-size:12px}th,td{padding:12px;border-bottom:1px solid #2a3b4d;text-align:left;white-space:nowrap}th{color:#92a9c1;background:#17283a}td.good{color:#78e4c4}td.bad{color:#f3b186}.badge{color:#78e4c4}select{background:#17283a;color:#dce7f4;border:1px solid #39516c;padding:7px;border-radius:5px}.plot{width:100%;max-width:1150px;background:white;border-radius:7px}code{word-break:break-all;color:#a4bbd3}li{margin:6px 0}@media(max-width:650px){main{padding:16px}h1{font-size:26px}.cards{grid-template-columns:repeat(2,1fr)}section{padding:16px}.metric{padding:5px 10px}.metric strong{font-size:24px}}@media print{body,section{background:white;color:#172735}.scroll{overflow:visible}th{background:#eef3f8;color:#172735}a{color:#17654c}}</style>
<main><header><p class="eyebrow">FRONTIER / PAIRED NATIVE BENCHMARK</p><h1>Search performance</h1><p class="badge">__STATE__ · __COMPLETED__ / __PLANNED__ episodes · Complete paired groups __PAIRS__ / 9 groups</p><p class="note">Same Gibson Rs house under original,, cluttered, and dynamic-obstacle conditions × 3 seed/initial-heading blocks × Four policies. Native RGB-D, YOLO11s, Locobot, shared navigation and recovery, independent stationary close-range verification. Using 1.1 extension and 0.65 m verified initialization region. </p></header>
<div class="cards">__CARDS__</div><section><h2>Paired primary results</h2><p class="note">The main table includes only paired groups completed by all four policies, avoiding bias from execution order. The success denominator includes all failures. Every failure receives a time penalty of 600 s; Interpret successful time and distance alongside the successful sample count.. </p><div class="scroll"><table><thead><tr><th>Policy</th><th>Success / Total</th><th>Success rate</th><th>Wilson 95% interval %</th><th>Penalized time s</th><th>Successful time s</th><th>Successful distance m</th><th>Contact ticks</th><th>LLM Calls</th><th>fallback</th></tr></thead><tbody>__TABLE__</tbody></table></div>__IMAGE__</section>
<section><h2>Three scene conditions</h2><div class="scroll"><table><thead><tr><th>Condition</th><th>Random</th><th>Nearest</th><th>IG</th><th>LLM-guided</th></tr></thead><tbody>__SCENES__</tbody></table></div></section>
<section><h2>Paired policy differences</h2><p class="note">A − B: success-rate differences are in percentage points; penalized-time differences are in seconds, negative values mean A faster. Intervals are stratified by scene condition, using seed/heading paired groups are resampled; Exact tests of success differences: McNemar tests apply to all six comparisons with Holm adjusted. Small-sample intervals do not quantify generalization across houses.. </p><div class="scroll"><table><thead><tr><th>A − B</th><th>Pair count</th><th>Success-rate difference pp</th><th>bootstrap 95% interval pp</th><th>Penalized-time difference s</th><th>bootstrap 95% interval s</th><th>adjusted p</th></tr></thead><tbody>__COMPARISONS__</tbody></table></div></section>
<section><h2>Per-episode results and evidence</h2><label>Filter <select id="filter"><option value="all">All policies</option><option>random</option><option>nearest</option><option>information_gain</option><option>llm_guided</option></select></label><div class="scroll"><table><thead><tr><th>Paired task</th><th>Policy</th><th>Termination outcome</th><th>Simulation time s</th><th>Actual distance m</th><th>Contact ticks</th><th>Calls / fallback</th><th>Evidence</th></tr></thead><tbody id="episodes">__DETAILS__</tbody></table></div></section>
<section><h2>Progress and interpretation</h2><ul>__ACTIVE__</ul><p>Each combination runs once, no success filtering or failure retries. All termination outcomes are preserved. All four policies share the start,, Target, Perception, control, Observation, recovery and budgets; Target ground truth and full-map reachability are used only for isolated preflight and terminal verification.. </p><p class="note">Rs Geometry and target positions are fixed; Seeds do not generate new houses. Three predefined initial headings vary observations and tasks.. Nearest, IG and the detector are deterministic for identical inputs, LLM hosted model responses are not reproducible from local seeds alone. __WORKERS__ concurrent workers share local resources, Wall-clock time is recorded separately in CSV/JSON in, not a comparison of isolated single-robot latency. </p><p><a href="../protocol.json">Frozen configuration</a> · <a href="../results.json">Full statistics JSON</a> · <a href="../episodes.csv">Per episode CSV</a> · <a href="../preflight/index.json">Isolated reachability preflight</a></p><p class="note">Engine fingerprint: <code>__FINGERPRINT__</code>. Updated: __TIME__. </p></section></main><script>document.getElementById('filter').onchange=e=>{for(const r of document.querySelectorAll('#episodes tr'))r.hidden=e.target.value!=='all'&&r.dataset.policy!==e.target.value;};</script></html>'''
    page=page.replace('__PAIRS__ / 9','__PAIRS__ / __PLANNED_PAIRS__')
    values={'REFRESH':refresh,'STATE':state,'COMPLETED':str(stats['completed']),'PLANNED':str(stats['planned']),
        'PLANNED_PAIRS':str(stats['planned_pairs']),
        'PAIRS':str(stats['complete_pairs']),'CARDS':cards,'TABLE':table,'SCENES':scenes,
        'COMPARISONS':comparisons,'DETAILS':details,'ACTIVE':active or '<li>No active episodes.. </li>',
        'IMAGE':image,'FINGERPRINT':protocol['engine_fingerprint'],'TIME':stats['generated_utc'],'WORKERS':str(protocol['runtime']['workers'])}
    noise=protocol.get('sensor_noise')
    label=('ON' if noise['enabled'] else 'OFF') if noise is not None else 'not recorded (legacy protocol)'
    detail=html.escape(json.dumps(noise,sort_keys=True)) if noise is not None else ''
    noise_note='<section><h2>Sensor noise: '+label+'</h2><p><code>'+detail+'</code></p><p>Base settings; per-job seeds are recorded in each manifest. Evaluator truth remains clean.</p></section>'
    page=page.replace('<div class="cards">',live_status_panel(root,protocol,stats,rows)+noise_note+'<div class="cards">',1)
    if protocol.get('design')=='bowl48':
        design_note='<section><h2>48-run design</h2><p>Two starts (A original, B bedroom doorway) x three conditions x four policies x two bowl locations. Kitchen 24 first, bathroom 24 second. Seed streams fixed; actual depth noise seed 140. Full initial scan at each start. All outcomes retained.</p></section>'
        page=page.replace('<main>','<main>'+design_note,1)
        page=page.replace('3 seed/initial-heading blocks','2 starting poses and 2 target locations')
        page=page.replace('seed/heading paired groups','Target/start-pose paired groups')
        page=page.replace('Three predefined initial headings vary observations and tasks.. ','The noise seed is fixed; Task variation comes from two starts and two target positions.. ')
    for k,v in values.items():page=page.replace('__'+k+'__',v)
    if protocol.get('fixed_start_heading'):
        page=page.replace('3 seed/initial-heading blocks','3 seed blocks (fixed initial heading) ')
        page=page.replace('seed/heading paired groups','seed paired groups')
        page=page.replace('Three predefined initial headings vary observations and tasks.. ','Initial position and heading are fixed, three seed blocks use separate policy streams,, perception and noise streams. ')
    temporary=report/'index.tmp.html';temporary.write_text(page,encoding='utf-8');replace_shared(temporary,report/'index.html')
    return stats

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run-root',type=Path,required=True);p.add_argument('--plots',action='store_true');a=p.parse_args()
    s=build(a.run_root,a.plots);print(json.dumps({k:s[k] for k in ('planned','completed','complete_pairs','complete')}))
