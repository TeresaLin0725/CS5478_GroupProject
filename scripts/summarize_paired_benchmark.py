# coding: utf-8
"""Publish supplementary failure/usage tables and native-trajectory plots after audit."""
import argparse,html,json
from pathlib import Path
from collections import Counter

POLICIES=('random','nearest','information_gain','llm_guided')
LABELS=('Random','Nearest','Information Gain','LLM-guided')
SCENES={'apartment':'原始公寓','clutter':'拥挤公寓','dynamic':'动态障碍'}
def read(p):return json.loads(Path(p).read_text(encoding='utf-8'))
def fmt(x,d=1):return '—' if x is None else ('%.*f'%(d,x))

def publish(root):
    root=Path(root).resolve();stats=read(root/'results.json');protocol=read(root/'protocol.json')
    assert stats['complete'] and read(root/'audit.json')['passed'],'Publish only fully audited results'
    rows=stats['results'];ps=stats['paired_policy_stats'];report=root/'report'
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    colors=('#dc8f40','#407eae','#9768ac','#209883')
    fig,axes=plt.subplots(3,3,figsize=(13,14),layout='constrained',sharex=True,sharey=True)
    failure_facts=[]
    for rowi,profile in enumerate(SCENES):
        grid=read(root/'preflight'/profile/'oracle-grid.json');a=np.asarray(grid['data'])
        extent=(grid['origin_x_m'],grid['origin_x_m']+grid['width_cells']*grid['resolution_m'],
            grid['origin_y_m'],grid['origin_y_m']+grid['height_cells']*grid['resolution_m'])
        for coli,block in enumerate(protocol['seed_heading_blocks']):
            ax=axes[rowi,coli];ax.imshow(a,origin='lower',extent=extent,cmap='Greys',vmin=0,vmax=160,alpha=.3)
            for policy,label,color in zip(POLICIES,LABELS,colors):
                r=next(r for r in rows if r['profile']==profile and r['environment_seed']==block['seed'] and r['policy']==policy)
                points=[];last_decision=None;current_sim=0.;operation_starts={};operation_timeouts=[]
                with (root/'episodes'/r['job_id']/'events.jsonl').open(encoding='utf-8') as f:
                    for line in f:
                        e=json.loads(line)
                        if e['event']=='motion_sample':
                            points.append((e['pose']['x_m'],e['pose']['y_m']));current_sim=e['simulated_time_s']
                        if e['event']=='decision':last_decision=e
                        if e['event'] in ('dispatch','observation_start'):operation_starts[e['request']['goal_id']]=current_sim
                        if e['event']=='operation_timeout':
                            elapsed=current_sim-operation_starts[e['goal_id']]
                            limit=protocol['base_manifest']['navigation']['navigation_deadline_s'] if e['kind']=='navigate' else protocol['base_manifest']['observation']['operation_deadline_s']
                            operation_timeouts.append(dict(goal_id=e['goal_id'],kind=e['kind'],elapsed_sim_s=elapsed,
                                deadline_s=limit,wall_bound_triggered_before_sim_bound=elapsed<limit-1e-6))
                if not r['success']:
                    statefile=root/'episodes'/r['job_id']/'final-state.json'
                    state=read(statefile) if statefile.exists() else {}
                    context=last_decision['context'] if last_decision else {}
                    failure_facts.append(dict(job_id=r['job_id'],termination=r['termination'],
                        final_frontier_count=len(state.get('frontiers',[])),last_eligible_frontier_count=len(context.get('eligible_frontier_ids',[])),
                        last_exclusion_reasons=dict(Counter(e['reason'] for e in context.get('exclusions',[]))),
                        observed_cup_tracks=sum(o['class_name']=='cup' for o in state.get('semantic_objects',[])),
                        target_claim_present=state.get('target_claim') is not None,final_pose=state.get('robot_pose'),operation_timeouts=operation_timeouts))
                if points:
                    xs,ys=zip(*points);ax.plot(xs,ys,color=color,lw=1.1,alpha=.85,label=label)
                    ax.plot(xs[-1],ys[-1],marker='o' if r['success'] else 'x',ms=6,color=color)
            ax.plot(.8,-2.4,'k^',ms=5)
            ax.plot(-1.4,2.95,'*',color='#c53f32',ms=10)
            ax.set_title('%s / seed %d, yaw %+.0f deg'%(profile,block['seed'],np.degrees(block['yaw_rad'])),fontsize=10)
            ax.set_aspect('equal');ax.set_xlabel('World x (m)');ax.set_ylabel('World y (m)')
    handles,labels=axes[0,0].get_legend_handles_labels()
    fig.legend(handles,labels,loc='outside lower center',ncol=4)
    fig.suptitle('Native executed trajectories: o success, x failure, triangle start, star target\nEvaluator-only grey geometry; dynamic background shows the parked cart',fontsize=12)
    for ext in ('png','svg'):fig.savefig(report/('trajectories.'+ext),dpi=160)
    plt.close(fig)
    (report/'failure-facts.json').write_text(json.dumps(failure_facts,ensure_ascii=False,indent=2),encoding='utf-8')
    lines=['# 当前四策略配对表现','',
        '完整完成 36 回合、9 组配对：同一 Gibson Rs 房屋，3 种条件 × 3 个种子/起始朝向 × 4 策略。所有失败纳入统计，没有单回合重试。',
        '','| 策略 | 成功 / 9 | 成功率 | Wilson 95% 区间 | 失败计 600 s 的平均时间 | 成功平均时间 | 成功平均路程 |',
        '| --- | ---: | ---: | --- | ---: | ---: | ---: |']
    for policy,label in zip(POLICIES,LABELS):
        s=ps[policy];lo,hi=s['success_ci95_wilson']
        lines.append('| %s | %d/9 | %.1f%% | %.1f%%–%.1f%% | %.1f s | %s s (n=%d) | %s m |'%(label,s['successes'],100*s['success_rate'],100*lo,100*hi,s['mean_penalized_time_s'],fmt(s['success_mean_time_s']),s['successes'],fmt(s['success_mean_distance_m'],2)))
    lines+=['','成功率优先于仅成功样本的效率；失败计罚时间避免“很早失败”被误当作高效率。区间描述当前小样本，不代表跨房屋泛化。',
        '','## 逐任务结果','','| 场景 / 种子 / 朝向 | Random | Nearest | IG | LLM-guided |','| --- | --- | --- | --- | --- |']
    gridrows=''
    for block in protocol['seed_heading_blocks']:
        for profile in SCENES:
            pair='%s-s%d'%(profile,block['seed']);cells=[];htmlcells=[]
            for policy in POLICIES:
                r=next(r for r in rows if r['pair_id']==pair and r['policy']==policy)
                text=('成功' if r['success'] else r['termination'])+' · '+fmt(r['simulated_time_s'])+' s'
                cells.append(text)
                htmlcells.append('<td class="%s"><a href="../episodes/%s/summary.json">%s</a></td>'%('good' if r['success'] else 'bad',r['job_id'],html.escape(text)))
            title='%s / %d / %+.0f°'%(SCENES[profile],block['seed'],block['yaw_rad']*180/3.141592653589793)
            lines.append('| '+title+' | '+' | '.join(cells)+' |')
            gridrows+='<tr><td>'+title+'</td>'+''.join(htmlcells)+'</tr>'
    lines+=['','## 失败与运行成本','','| 策略 | 终止分布 | 误声明 | 接触步数 / 有接触回合 | 重规划 | 无进展尝试 | 前沿访问 | 平均实际计算耗时 |',
        '| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |']
    detailrows=''
    for policy,label in zip(POLICIES,LABELS):
        s=ps[policy];terminations='; '.join('%s: %d'%(k,v) for k,v in s['terminations'].items())
        values=[label,terminations,str(s['false_claims']),str(s['contact_ticks'])+' / '+str(s['episodes_with_contact']),str(s['replans']),str(s['no_progress_attempts']),str(s['frontier_visits']),fmt(s['mean_wall_time_s'])+' s']
        lines.append('| '+' | '.join(values)+' |');detailrows+='<tr>'+''.join('<td>'+html.escape(v)+'</td>' for v in values)+'</tr>'
    llm=ps['llm_guided']
    reasons=Counter(reason for r in rows for reason in r.get('llm_fallback_reasons',[]))
    models=sorted({model for r in rows for model in r.get('response_models',[])})
    recovery=read(root/'export-recovery.json')
    recovery_note='有 %d 回合在搜索结束后发生调用记录导出异常；已从完整原生日志恢复原终止结果与调用记录，修复前文件保留，没有重跑或排除回合。'%len(recovery['records'])
    usage='LLM 共 %d 次调用，%d 次回退，回退率 %s；记录 token 合计 %d；调用延迟中位数 %s s，P95 %s s。'%(llm['llm_calls'],llm['llm_fallbacks'],fmt(100*llm['fallback_rate'])+'%' if llm['fallback_rate'] is not None else '—',llm['llm_tokens_total'],fmt(llm['llm_latency_median_s'],2),fmt(llm['llm_latency_p95_s'],2))
    cost='服务未返回数值费用，无法报告实际总费用。' if not llm['calls_with_recorded_cost'] else '有 %d 次调用记录数值费用，合计 %s（其余调用费用不可得）。'%(llm['calls_with_recorded_cost'],llm['recorded_cost_total'])
    lines+=['',usage,cost,'','响应模型：'+', '.join(models)+'。回退原因：'+(str(dict(reasons)) if reasons else '无')+'。',
        '','实际计算耗时来自 3 个并发工作进程，共享本机资源，不能解释为单机器人独占硬件延迟。固定单操作墙钟超时也可终止搜索，并发计算会影响这类终止；本批描述当前并发模式下的整条管线表现。',
        '','## 复核与限制','','原始事件、manifest、失败回合、原始 LLM 回答、私有验收证据均保留。audit.json 重算全部成功证据与 10,000 次配对重采样统计；原生代码、模型与资产校验和前后保持一致。',
        '','Rs 几何与目标位置固定；种子对应预先声明的初始朝向，Random 另有独立策略 RNG。Nearest / IG 与检测器在同样输入下确定；在线 LLM 不能由本地种子保证完全重复。初始化包含 0.65 m 已验证自由区域，多视角近距离停稳确认对四策略一致。',
        '',recovery_note,'','旧 v1 只有预检查；旧 v2 全批次因 Windows 监控文件锁冲突作废。当前 36 回合全部为修复工具后的全新实验，未从旧批次选取任何结果。',
        '','[主统计与配对区间](index.html) · [实际轨迹图](trajectories.png) · [完整统计 JSON](../results.json) · [独立复核](../audit.json)']
    (report/'findings.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    template='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>36 回合逐任务表现</title><style>:root{color-scheme:dark}body{margin:0;background:#0a111c;color:#dce7f4;font:14px/1.8 "Segoe UI","Microsoft YaHei",sans-serif}main{max-width:1450px;margin:auto;padding:32px}section{margin:26px 0;padding:24px;border:1px solid #2b3b4f;border-radius:12px;background:#111d2c}h1,h2{font-weight:500}a{color:#7fdfc0}.scroll{overflow:auto}table{width:100%;border-collapse:collapse;font-size:12px}td,th{padding:11px;text-align:left;white-space:nowrap;border-bottom:1px solid #2b3b4f}.good a{color:#78e4c4}.bad a{color:#f3b186}.note{color:#91a5bd}img{width:100%;border-radius:7px;background:white}code{overflow-wrap:anywhere}@media(max-width:650px){main{padding:16px}section{padding:14px}}</style><main><p><a href="index.html">← 主统计与配对差异</a> · <a href="findings.md">完整结论 Markdown</a> · <a href="../audit.json">独立复核证据</a></p><h1>36 回合，逐任务核对</h1><p class="note">同一 Rs 房屋 · 三种条件 · 三个种子/起始朝向 · 四策略。所有成功和失败均计入。</p><section><h2>9 组配对的实际结果</h2><div class="scroll"><table><thead><tr><th>条件 / 种子 / 起始朝向</th><th>Random</th><th>Nearest</th><th>IG</th><th>LLM-guided</th></tr></thead><tbody>__GRID__</tbody></table></div><p class="note">秒数为本回合实际仿真时间，失败仍在主表中计 600 s。点击单元格查看原始汇总。</p></section><section><h2>机器人实际走了哪些路</h2><img src="trajectories.png" alt="九组任务的原生机器人实际执行轨迹"><p class="note">圆点：成功；叉号：失败；三角：起点；星号：目标。灰色是实验后叠加的独立验收几何，未提供给策略。<a href="trajectories.svg">导出 SVG</a></p></section><section><h2>失败、恢复与计算耗时</h2><div class="scroll"><table><thead><tr><th>策略</th><th>终止分布</th><th>误声明</th><th>接触步 / 回合</th><th>重规划</th><th>无进展</th><th>前沿访问</th><th>平均计算墙钟</th></tr></thead><tbody>__DETAILS__</tbody></table></div><p class="note">3 个并发进程共享本机资源，墙钟不是单机器人独占硬件延迟。</p></section><section><h2>在线模型的实际使用量</h2><p>__USAGE__</p><p>__COST__</p><p class="note">响应模型：__MODELS__。回退原因：__REASONS__。</p></section></main></html>'''
    for key,value in dict(GRID=gridrows,DETAILS=detailrows,USAGE=html.escape(usage),COST=html.escape(cost),MODELS=html.escape(', '.join(models)),REASONS=html.escape(str(dict(reasons)) if reasons else '无')).items():template=template.replace('__'+key+'__',value)
    failure_rows=''
    for f in failure_facts:
        timeout_text='; '.join(t['kind']+(' / wall' if t['wall_bound_triggered_before_sim_bound'] else ' / sim')+' / '+fmt(t['elapsed_sim_s'])+' sim s' for t in f['operation_timeouts']) or '—'
        failure_rows+='<tr>'+''.join('<td>'+html.escape(str(v))+'</td>' for v in (f['job_id'],f['termination'],f['final_frontier_count'],f['last_eligible_frontier_count'],f['last_exclusion_reasons'],f['observed_cup_tracks'],timeout_text))+'<td><a href="../episodes/'+f['job_id']+'/last-processed-rgb.png">最后处理的 RGB</a></td></tr>'
    failure_section='<section><h2>失败回合最后留下了什么</h2><p class="note">以下为保存状态与最后一次决策的直接统计；排除原因反映执行筛选，不等于已经证明的根因。固定操作墙钟超时也可终止搜索；并发计算会影响墙钟和这类终止，因此本批属于当前并发运行模式下的整条管线表现。</p><div class="scroll"><table><thead><tr><th>任务</th><th>终止</th><th>地图前沿数</th><th>最后合格前沿数</th><th>排除原因</th><th>已观测 cup 轨迹</th><th>操作超时 / 触发边界</th><th>相机</th></tr></thead><tbody>'+failure_rows+'</tbody></table></div><p><a href="failure-facts.json">导出失败事实 JSON</a></p></section>'
    template=template.replace('灰色是实验后叠加的独立验收几何，未提供给策略。','灰色是实验后叠加的独立验收几何，未提供给策略；动态条件背景为预检查的推车停妥状态。')
    template=template.replace('</main>',failure_section+'<section><h2>原始数据与导出记录</h2><p>'+html.escape(recovery_note)+'</p><p><a href="../export-recovery.json">逐回合处理依据与原文件校验和</a> · <a href="../superseded-suites.json">早期整批诊断溯源</a></p></section></main>')
    (report/'analysis.html').write_text(template,encoding='utf-8')
    print(json.dumps(dict(published=str(report/'analysis.html'),completed=stats['completed']),ensure_ascii=True))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run_root',type=Path);publish(p.parse_args().run_root)
