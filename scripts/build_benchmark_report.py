"""Evidence-linked, self-contained live/final benchmark report and scientific plots."""
import argparse,csv,html,json
from pathlib import Path
from datetime import datetime,timezone
from benchmark_protocol import POLICIES,PROFILES,aggregate
from benchmark_episode import atomic,replace_shared

LABELS={'random':'Random','nearest':'Nearest','information_gain':'Information Gain','llm_guided':'LLM-guided'}
SCENES={'apartment':'原始公寓','clutter':'拥挤公寓','dynamic':'动态障碍'}
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
    invalid=[r['job_id'] for r in rows if r['policy']=='llm_guided'
             and r.get('llm_calls',0)>0 and r.get('llm_fallbacks',0)>=r['llm_calls']]
    if invalid:
        panel+='<p style="color:#f3b186"><strong>LLM validity warning:</strong> all hosted calls fell back in: '+html.escape(', '.join(invalid))+'. Do not treat those episodes as valid hosted-LLM comparisons.</p>'
    return panel+'</section>'

def build(root,plots=False):
    root=Path(root).resolve();protocol=read(root/'protocol.json');rows=collect(root,protocol)
    stats=aggregate(rows,protocol['jobs'],draws=10000 if plots else 1000)
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
        fig.suptitle('Rs: 3 conditions x 3 seed/heading blocks; same native pipeline',fontsize=11)
        for ext in ('png','svg'):fig.savefig(report/('performance.'+ext),dpi=180)
        plt.close(fig)
    primary=stats['paired_policy_stats'];cards=''
    for p in POLICIES:
        s=primary[p];cards+='<div class="metric"><small>'+LABELS[p]+'</small><strong>'+str(s['successes'])+'/'+str(s['n'])+'</strong><span>'+percent(s['success_rate'])+' 成功</span></div>'
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
        status='成功' if r['success'] else r['termination']
        details+='<tr data-policy="'+r['policy']+'"><td>'+r['pair_id']+'</td><td>'+LABELS[r['policy']]+'</td><td class="'+('good' if r['success'] else 'bad')+'">'+status+'</td><td>'+number(r.get('simulated_time_s'))+'</td><td>'+number(r.get('actual_distance_m'),2)+'</td><td>'+str(r.get('collisions',0))+'</td><td>'+str(r.get('llm_calls',0))+' / '+str(r.get('llm_fallbacks',0))+'</td><td><a href="'+folder+'summary.json">汇总</a> · <a href="'+folder+'events.jsonl">日志</a> · <a href="'+folder+'result.json">核验</a></td></tr>'
    active=''
    progress=read(root/'progress.json') if (root/'progress.json').exists() else {}
    for item in progress.get('active',[]):
        active+='<li>'+html.escape(item['job_id'])+' · '+number(item.get('simulated_time_s'))+' s · '+str(item.get('steps',0))+' 步 · '+html.escape(item.get('pending_module') or 'running')+'</li>'
    refresh='<meta http-equiv="refresh" content="30">' if not stats['complete'] else ''
    image='<img class="plot" src="performance.png" alt="配对成功率和失败计罚搜索时间"><p><a href="performance.svg">导出 SVG 图</a></p>' if (report/'performance.png').exists() else ''
    state='已完成' if stats['complete'] else '进行中'
    page='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">__REFRESH__<title>FRONTIER · 配对基准表现</title>
<style>:root{color-scheme:dark}*{box-sizing:border-box}body{margin:0;background:#0a111c;color:#dce7f4;font:14px/1.8 "Segoe UI","Microsoft YaHei",sans-serif}main{max-width:1500px;margin:auto;padding:32px}header{border-bottom:1px solid #2b3b4f;margin-bottom:24px}h1{font-size:34px;font-weight:550}h2{font-size:21px;font-weight:500;margin:0 0 16px}small,.note{color:#91a5bd}.eyebrow{color:#78e4c4;font:12px Consolas,monospace;letter-spacing:.15em}section{background:#111d2c;border:1px solid #2b3b4f;border-radius:12px;padding:24px;margin:24px 0}.cards{display:grid;grid-template-columns:repeat(4,1fr);gap:16px}.metric{border-left:2px solid #78e4c4;padding:8px 20px}.metric>*{display:block}.metric strong{font-size:30px}.metric span{color:#a2c9bf}a{color:#7fdfc0;text-decoration:none}a:hover{text-decoration:underline}.scroll{overflow:auto}table{border-collapse:collapse;width:100%;font-size:12px}th,td{padding:12px;border-bottom:1px solid #2a3b4d;text-align:left;white-space:nowrap}th{color:#92a9c1;background:#17283a}td.good{color:#78e4c4}td.bad{color:#f3b186}.badge{color:#78e4c4}select{background:#17283a;color:#dce7f4;border:1px solid #39516c;padding:7px;border-radius:5px}.plot{width:100%;max-width:1150px;background:white;border-radius:7px}code{word-break:break-all;color:#a4bbd3}li{margin:6px 0}@media(max-width:650px){main{padding:16px}h1{font-size:26px}.cards{grid-template-columns:repeat(2,1fr)}section{padding:16px}.metric{padding:5px 10px}.metric strong{font-size:24px}}@media print{body,section{background:white;color:#172735}.scroll{overflow:visible}th{background:#eef3f8;color:#172735}a{color:#17654c}}</style>
<main><header><p class="eyebrow">FRONTIER / PAIRED NATIVE BENCHMARK</p><h1>目前的搜索表现</h1><p class="badge">__STATE__ · __COMPLETED__ / __PLANNED__ 回合 · 完整配对 __PAIRS__ / 9 组</p><p class="note">同一 Gibson Rs 房屋的原始、拥挤、动态障碍三种条件 × 3 组种子与起始朝向 × 四策略。原生 RGB-D、YOLO11s、Locobot、共享导航与恢复、独立近距离停稳验收。当前采用 1.1 扩展与 0.65 m 已验证初始化区域。</p></header>
<div class="cards">__CARDS__</div><section><h2>配对主结果</h2><p class="note">主表只包含四策略都已结束的配对组，避免运行先后造成偏差。成功分母包括全部失败。失败计罚时间统一为 600 s；成功耗时与路程需结合成功样本数阅读。</p><div class="scroll"><table><thead><tr><th>策略</th><th>成功 / 总数</th><th>成功率</th><th>Wilson 95% 区间 %</th><th>计罚时间 s</th><th>成功耗时 s</th><th>成功路程 m</th><th>接触步数</th><th>LLM 调用</th><th>回退</th></tr></thead><tbody>__TABLE__</tbody></table></div>__IMAGE__</section>
<section><h2>三种场景条件</h2><div class="scroll"><table><thead><tr><th>条件</th><th>Random</th><th>Nearest</th><th>IG</th><th>LLM-guided</th></tr></thead><tbody>__SCENES__</tbody></table></div></section>
<section><h2>策略间配对差异</h2><p class="note">A − B：成功率差为百分点；计罚时间差为秒，负值表示 A 更快。区间按条件分层、以种子/朝向配对组重采样；成功差异的精确 McNemar 检验对六个比较做 Holm 校正。小样本区间不能代表不同房屋的泛化不确定性。</p><div class="scroll"><table><thead><tr><th>A − B</th><th>配对数</th><th>成功率差 pp</th><th>重采样 95% 区间 pp</th><th>计罚时间差 s</th><th>重采样 95% 区间 s</th><th>校正 p</th></tr></thead><tbody>__COMPARISONS__</tbody></table></div></section>
<section><h2>逐回合结果与证据</h2><label>筛选 <select id="filter"><option value="all">全部策略</option><option>random</option><option>nearest</option><option>information_gain</option><option>llm_guided</option></select></label><div class="scroll"><table><thead><tr><th>配对任务</th><th>策略</th><th>终止结果</th><th>实际仿真 s</th><th>实际路程 m</th><th>接触步数</th><th>调用 / 回退</th><th>证据</th></tr></thead><tbody id="episodes">__DETAILS__</tbody></table></div></section>
<section><h2>运行进度与口径</h2><ul>__ACTIVE__</ul><p>每个组合只运行一次，没有成功筛选或失败重试。所有终止结果均保存。四策略共享起点、目标、感知、控制、观察、恢复和预算；目标真值与全图可达性只用于隔离的预检查和终止验收。</p><p class="note">Rs 几何与目标位置固定；种子本身不生成新房屋。三个预先固定的初始朝向使观测与任务有所变化。Nearest、IG 和检测器在相同输入下确定，LLM 服务端不能由本地种子保证重复。__WORKERS__ 个并发工作进程共享本机资源，计算墙钟单独保存在 CSV/JSON 中，不作为单机器人独占硬件的延迟比较。</p><p><a href="../protocol.json">冻结配置</a> · <a href="../results.json">完整统计 JSON</a> · <a href="../episodes.csv">逐回合 CSV</a> · <a href="../preflight/index.json">隔离可达性预检查</a></p><p class="note">引擎指纹：<code>__FINGERPRINT__</code>。更新时间：__TIME__。</p></section></main><script>document.getElementById('filter').onchange=e=>{for(const r of document.querySelectorAll('#episodes tr'))r.hidden=e.target.value!=='all'&&r.dataset.policy!==e.target.value;};</script></html>'''
    values={'REFRESH':refresh,'STATE':state,'COMPLETED':str(stats['completed']),'PLANNED':str(stats['planned']),
        'PAIRS':str(stats['complete_pairs']),'CARDS':cards,'TABLE':table,'SCENES':scenes,
        'COMPARISONS':comparisons,'DETAILS':details,'ACTIVE':active or '<li>没有正在执行的回合。</li>',
        'IMAGE':image,'FINGERPRINT':protocol['engine_fingerprint'],'TIME':stats['generated_utc'],'WORKERS':str(protocol['runtime']['workers'])}
    noise=protocol.get('sensor_noise')
    label=('ON' if noise['enabled'] else 'OFF') if noise is not None else 'not recorded (legacy protocol)'
    detail=html.escape(json.dumps(noise,sort_keys=True)) if noise is not None else ''
    noise_note='<section><h2>Sensor noise: '+label+'</h2><p><code>'+detail+'</code></p><p>Base settings; per-job seeds are recorded in each manifest. Evaluator truth remains clean.</p></section>'
    page=page.replace('<div class="cards">',live_status_panel(root,protocol,stats,rows)+noise_note+'<div class="cards">',1)
    for k,v in values.items():page=page.replace('__'+k+'__',v)
    temporary=report/'index.tmp.html';temporary.write_text(page,encoding='utf-8');replace_shared(temporary,report/'index.html')
    return stats

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run-root',type=Path,required=True);p.add_argument('--plots',action='store_true');a=p.parse_args()
    s=build(a.run_root,a.plots);print(json.dumps({k:s[k] for k in ('planned','completed','complete_pairs','complete')}))
