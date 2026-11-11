"""Build a portable Chinese HTML report and editable Markdown from frozen results."""
import argparse
import base64
import hashlib
import html
import json
from pathlib import Path
from statistics import mean

POLICIES = ['random', 'nearest', 'information_gain', 'llm_guided']
LABELS = dict(zip(POLICIES, ['Random', 'Nearest', 'Information Gain', 'LLM-guided']))
TARGETS = {'kitchen': '厨房', 'bathroom': '卫生间'}
PROFILES = {'apartment': '普通公寓', 'clutter': '拥挤场景', 'dynamic': '动态障碍'}


class Report:
    def __init__(self, out):
        self.out, self.web, self.md = out, [], []

    def heading(self, text, level=2):
        self.web.append(f'<h{level}>{html.escape(text)}</h{level}>')
        self.md.append('#' * level + ' ' + text + '\n')

    def paragraph(self, text):
        self.web.append('<p>' + html.escape(text) + '</p>')
        self.md.append(text + '\n')

    def table(self, headers, rows):
        self.web.append('<div class="table-wrap"><table><thead><tr>' +
                        ''.join('<th>' + html.escape(str(x)) + '</th>' for x in headers) +
                        '</tr></thead><tbody>' + ''.join('<tr>' + ''.join(
                            '<td>' + html.escape(str(x)) + '</td>' for x in row) +
                            '</tr>' for row in rows) + '</tbody></table></div>')
        self.md.extend(['| ' + ' | '.join(map(str, headers)) + ' |',
                        '| ' + ' | '.join(['---'] * len(headers)) + ' |'])
        self.md.extend('| ' + ' | '.join(map(str, row)) + ' |' for row in rows)
        self.md.append('')

    def figure(self, path, caption):
        encoded = base64.b64encode(path.read_bytes()).decode('ascii')
        self.web.append(f'<figure><img src="data:image/png;base64,{encoded}" '
                        f'alt="{html.escape(caption)}"><figcaption>{html.escape(caption)}</figcaption></figure>')
        self.md.append(f'![{caption}]({path.name})\n\n{caption}\n')

    def link(self, label, url):
        self.web.append(f'<p><a href="{html.escape(url)}">{html.escape(label)}</a></p>')
        self.md.append(f'[{label}]({url})\n')

    def save(self):
        css = '''body{font:16px/1.85 "Microsoft YaHei","Noto Sans CJK SC",sans-serif;color:#172b3a;background:#fff;margin:0}
main{max-width:1100px;margin:40px auto;padding:0 32px 70px}h1{font-size:32px;line-height:1.35;margin-bottom:20px;color:#000}
h2{font-size:23px;margin:38px 0 14px;color:#000}h3{font-size:18px;margin:26px 0 12px;color:#000}p{margin:12px 0}
table{border-collapse:collapse;width:100%;font-size:14px;line-height:1.6;margin:14px 0 22px}th,td{border:1px solid #d9d9d9;padding:9px 12px;text-align:right}
th{background:#21445b;color:white;font-weight:600}td:first-child,th:first-child{text-align:left}tbody tr:nth-child(even){background:#f1f5f7}
.table-wrap{overflow-x:auto}figure{margin:24px 0}img{width:100%;height:auto}figcaption{font-size:13px;color:#526270;line-height:1.6;margin-top:8px}
a{color:#116e9a}header{border-bottom:1px solid #d9d9d9;padding-bottom:16px}footer{font-size:13px;color:#526270;margin-top:32px}
@media print{@page{size:A4;margin:17mm}body{font-size:10pt;line-height:1.65}main{max-width:none;margin:0;padding:0}h1{font-size:22pt}h2{font-size:15pt;break-after:avoid}h3{break-after:avoid}table{font-size:8.5pt}th{color:black;background:#e8eef2}thead{display:table-header-group}tr,figure{break-inside:avoid}a{color:inherit}img{max-height:190mm;object-fit:contain}.table-wrap{overflow:visible}}
'''
        doc = '<!doctype html><html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>48轮有噪声碗搜索实验报告</title><style>' + css + '</style></head><body><main>' + '\n'.join(self.web) + '</main></body></html>'
        (self.out / 'bowl48-report-zh.html').write_text(doc, encoding='utf-8')
        (self.out / 'bowl48-report-zh.md').write_text('\n'.join(self.md), encoding='utf-8')


def figures(root, out, data):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    font = FontProperties(fname='C:/Windows/Fonts/msyh.ttc')
    plt.rcParams['font.family'] = font.get_name()
    plt.rcParams['axes.unicode_minus'] = False
    colors = ['#8b9eb5', '#4d91b2', '#8b78b2', '#249b84']
    fig, axes = plt.subplots(2, 2, figsize=(11, 7.4), layout='constrained')
    for col, target in enumerate(TARGETS):
        for row, (metric, unit) in enumerate([('mean_penalized_time_s', '平均计罚仿真时间  秒'),
                                             ('success_mean_distance_m', '成功轮平均实际路径  米')]):
            ax = axes[row, col]
            vals = [data['per_target'][target][p][metric] for p in POLICIES]
            ax.bar(range(4), vals, color=colors, width=.65)
            ax.set_xticks(range(4), ['Random', 'Nearest', 'IG', 'LLM'])
            ax.set_ylabel(unit)
            ax.set_title(TARGETS[target] + '碗  每种方法6轮')
            shared_max = max(data['per_target'][t][p][metric] for t in TARGETS for p in POLICIES)
            ax.set_ylim(0, shared_max * 1.2)
            for i, v in enumerate(vals):
                ax.text(i, v + max(vals) * .025, f'{v:.1f}', ha='center', fontsize=10)
            ax.spines[['top', 'right']].set_visible(False)
            ax.grid(axis='y', alpha=.18)
            ax.set_axisbelow(True)
    fig.savefig(out / 'target-performance.png', dpi=170)
    plt.close(fig)
    byid = {r['job_id']: r for r in data['results']}
    for target, start in [('kitchen', 'A'), ('kitchen', 'B'), ('bathroom', 'A'), ('bathroom', 'B')]:
        fig, axes = plt.subplots(1, 4, figsize=(15.5, 6.2), dpi=170)
        fig.subplots_adjust(left=.01, right=.99, top=.83, bottom=.08, wspace=.035)
        for ax, policy in zip(axes, POLICIES):
            job_id = f'{target}-apartment-start{start}-{policy}'
            r = byid[job_id]
            ax.imshow(plt.imread(root / 'episodes' / job_id / 'trajectory-scene.png'))
            ax.axis('off')
            status = '（未成功）' if not r['success'] else ''
            ax.set_title(f'{LABELS[policy]}{status}\n{r["simulated_time_s"]:.1f} 秒   {r["actual_distance_m"]:.2f} 米', fontsize=11, pad=9)
        fig.suptitle(f'{TARGETS[target]}碗  普通公寓  起点{start}  四种方法的实际轨迹', fontsize=18, y=.98)
        fig.text(.5, .025, '蓝线为实际行驶轨迹  绿点为起点  红星为目标  三角为终点朝向', ha='center', fontsize=10)
        fig.savefig(out / f'trajectory-{target}-apartment-start{start}.png', dpi=170)
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-root', type=Path, required=True)
    args = parser.parse_args()
    root = args.run_root.resolve()
    read = lambda name: json.loads((root / name).read_text(encoding='utf-8'))
    data, protocol, verify = read('results.json'), read('protocol.json'), read('suite-verification.json')
    assert data['completed'] == data['planned'] == 48 and data['complete_pairs'] == 12
    assert verify['all_result_integrity_checks_passed'] and verify['no_retries']
    rows = data['results']
    assert len({r['job_id'] for r in rows}) == 48
    assert all(r['sensor_noise']['enabled'] and r['noise_seed'] == 140 for r in rows)
    out = root / 'report-zh'
    out.mkdir(exist_ok=True)
    figures(root, out, data)
    r = Report(out)
    r.heading('48轮有噪声碗搜索实验报告', 1)
    r.paragraph('面向项目组的结果汇总　实验完成日期 2026年11月10日　报告日期 2026年11月11日')
    r.paragraph('本报告总结厨房和卫生间两处碗目标、两个起点、三种场景条件下四种搜索方法的48轮实验。共45轮成功，全部无碰撞；LLM-guided与Information Gain均为12/12成功。LLM在厨房任务中用时更少、路径更短，而卫生间任务由Information Gain表现更好。整体上，LLM的路径均值最短，IG的时间均值最低，现有配对区间不足以确认两者整体效率存在稳定差异。')

    r.heading('实验设置与比较范围')
    r.table(['项目', '本次设置'], [
        ['任务规模', '2处目标 × 2个起点 × 3种条件 × 4种方法 = 48轮'],
        ['目标', '每轮仅一个YCB碗024_bowl，位于厨房台面或卫生间马桶盖上'],
        ['起点A', '客厅原起点  x=0.8 m，y=−2.4 m，初始朝向90°'],
        ['起点B', '客厅靠卧室门  x=−1.5 m，y=0.6 m，初始朝向180°'],
        ['场景条件', '普通公寓；新增家具的拥挤场景；带移动推车的动态障碍场景'],
        ['方法', 'Random、Nearest、Information Gain、LLM-guided'],
        ['噪声', '深度高斯噪声标准差0.01 m；深度丢失概率1%；实际噪声种子140'],
        ['其他种子', '环境17；感知10017；策略20017，均固定'],
        ['模型', protocol['base_manifest']['policy']['llm_model']],
        ['共同预算', '最多600仿真秒、18000控制步、80次决策、45次尝试；墙钟上限2400秒'],
        ['执行顺序', '厨房24轮先运行，随后卫生间24轮；最多两轮并发'],
    ])
    r.paragraph('各方法共享感知、导航、目标接近与成功验收流程，启动时执行完整一周观察。LLM接收已观察物体类别与坐标、候选点、路径代价和执行反馈；不直接接收RGB图像、隐藏地图或目标真实位置。12组物理可行性预检查均通过；卧室门旁起点的卫生间启动检查中，40帧均未看到目标像素，也未形成碗轨迹。')
    r.paragraph('本次采用bowl48_accessible布局。为保证卫生间可达，拥挤与动态场景中原位于(−2.70, 1.0)的新增椅子统一移至(−3.35, 0.85)。该布局同时用于厨房和卫生间任务及全部方法。未运行搜索的旧准备批次保留在记录中；本报告仅使用正式完成的20261110-02批次。')

    r.heading('指标定义与整体结果')
    r.paragraph('成功率按全部轮次统计。平均计罚时间对每个失败统一记600秒，防止提前失败看起来更快。成功轮时间和路径只对各方法成功的轮次求均值，因此样本集合可能不同；公平的同任务效率比较另列在配对分析中。路径来自实际运动记录。仿真时间不等于模型调用或整个程序的墙钟耗时；本机并发运行的墙钟时间不作为独占硬件速度结论。')
    headers = ['方法', '成功', '计罚时间 秒', '成功时间 秒', '成功路径 米']
    def metric_rows(stats):
        return [[LABELS[p], f'{stats[p]["successes"]}/{stats[p]["n"]}',
                 f'{stats[p]["mean_penalized_time_s"]:.1f}', f'{stats[p]["success_mean_time_s"]:.1f}',
                 f'{stats[p]["success_mean_distance_m"]:.2f}'] for p in POLICIES]
    r.table(headers, metric_rows(data['completed_policy_stats']))
    r.paragraph('与IG相比，LLM的平均路径由9.44米降至8.24米，缩短12.6%；平均计罚时间由101.6秒增至104.0秒，增加2.4%。与Random和Nearest相比，LLM的计罚时间均值分别降低49.5%和51.7%，但这两项差异包含失败被计600秒的影响，不能直接解释为成功搜索速度提高同样比例。')

    r.heading('目标位置改变了方法排序')
    for target in TARGETS:
        r.heading(TARGETS[target] + '碗', 3)
        r.table(headers, metric_rows(data['per_target'][target]))
    r.paragraph('厨房的6组任务中，LLM与IG均全部成功，LLM平均时间减少25.4%，路径缩短41.9%。卫生间的6组任务中，两者也均全部成功，但LLM比IG耗时多51.5%、路径长60.6%。因此，厨房结果中的优势不能直接推广到卫生间。目标位置同时改变可见性、几何路线和语义关联，当前实验不能单独识别“房间常识”造成的效果。')
    r.figure(out / 'target-performance.png', '图1 两处目标的平均计罚时间与成功轮路径。每种方法每处目标6轮，路径均值的成功样本数见表。')

    r.heading('起点与场景条件')
    start_rows = []
    for start in ('A', 'B'):
        start_rows.extend([[start] + row for row in metric_rows(data['per_start'][start])])
    r.table(['起点'] + headers, start_rows)
    r.paragraph('起点B处，LLM与IG的平均时间接近，分别为93.4秒与92.9秒，路径分别为7.41米与7.75米。Nearest的两次失败均发生在起点B的拥挤场景。这里比较的是相同房屋中的两个固定起点，并非对全部可能起点的估计。')
    profile_rows = []
    for profile in PROFILES:
        profile_rows.extend([[PROFILES[profile]] + row for row in metric_rows(data['per_profile'][profile])])
    r.table(['条件'] + headers, profile_rows)
    r.paragraph('LLM在拥挤条件的平均时间和成功路径均最低；普通和动态条件的时间均值由IG最低。各条件只有每种方法4轮，不能仅凭“拥挤”或“动态”标签预期方法一定更慢。新增家具改变初始可见范围、候选点集合和后续路线，任务难度可能出现非单调变化。')

    r.heading('配对分析与不确定性')
    comparisons = [c for c in data['paired_comparisons'] if c['b'] == 'llm_guided']
    paired = []
    for c in comparisons:
        baseline = {x['pair_id']: x for x in rows if x['policy'] == c['a']}
        llm = {x['pair_id']: x for x in rows if x['policy'] == 'llm_guided'}
        keys = [k for k in baseline if baseline[k]['success'] and llm[k]['success']]
        paired.append([LABELS[c['a']] + ' / LLM', len(keys),
                       f'{mean(baseline[k]["simulated_time_s"] for k in keys):.1f} / {mean(llm[k]["simulated_time_s"] for k in keys):.1f}',
                       f'{mean(baseline[k]["actual_distance_m"] for k in keys):.2f} / {mean(llm[k]["actual_distance_m"] for k in keys):.2f}'])
    r.table(['基线与LLM', '共同成功配对数', '平均时间 秒', '平均路径 米'], paired)
    ci_rows = []
    for c in comparisons:
        t, dist = c['jointly_successful_time_difference_s'], c['jointly_successful_distance_difference_m']
        ci_rows.append([LABELS[c['a']] + ' − LLM', t['n'],
                        f'{t["mean_difference"]:.2f} [{t["ci95"][0]:.2f}, {t["ci95"][1]:.2f}]',
                        f'{dist["mean_difference"]:.2f} [{dist["ci95"][0]:.2f}, {dist["ci95"][1]:.2f}]'])
    r.table(['差值方向', '配对数', '时间差及95%区间 秒', '路径差及95%区间 米'], ci_rows)
    r.paragraph('正差值表示基线用时更长或路径更长。区间来自既有报告按场景条件分层、对目标与起点配对组进行的重采样，仅描述这套固定房屋中的任务变化。IG与LLM的时间和路径差区间均跨过零，Nearest与LLM在共同成功任务中的差异区间也跨过零。Random与LLM的共同成功配对中，两项差异区间均为正。成功率比较中，所有涉及LLM的Holm校正McNemar检验p值均为1.0，不能把12/12对11/12或10/12解释为已证实的成功率优势。')

    r.heading('轨迹观察与决策解释')
    r.paragraph('以下四组示例覆盖普通公寓中厨房与卫生间两个目标位置、起点A与起点B的全部组合，便于观察目标与起点改变后的路线。它们用于说明行为，整体结论仍依据全部12组配对。背景由原始Rs平面图与仿真器实际渲染的新增物体图层合成；动态场景的推车显示初始位置。所有48轮均另存了带场景物体的trajectory-scene.png。')
    r.figure(out / 'trajectory-kitchen-apartment-startA.png', '图2 厨房碗与起点A的四方法轨迹。图中给出的耗时为该轮实际仿真时间，路径为该轮实际行驶距离。')
    r.figure(out / 'trajectory-kitchen-apartment-startB.png', '图3 厨房碗与起点B的四方法轨迹。四种方法均成功；LLM该轮耗时87.0秒，实际路径5.33米。')
    r.figure(out / 'trajectory-bathroom-apartment-startA.png', '图4 卫生间碗与起点A的四方法轨迹。Random以search_limit终止，未成功；图中564.7秒和31.38米是失败前的实际记录，主结果对该轮计罚600秒。其余三种方法均成功。')
    r.figure(out / 'trajectory-bathroom-apartment-startB.png', '图5 卫生间碗与起点B的四方法轨迹。该普通公寓实例中，LLM先探索餐桌方向，随后返回卫生间。')
    behavior = []
    for profile in PROFILES:
        x = next(x for x in rows if x['job_id'] == f'bathroom-{profile}-startB-llm_guided')
        behavior.append([PROFILES[profile], '先餐桌方向，后卫生间' if profile == 'apartment' else '先朝卫生间方向探索',
                         f'{x["simulated_time_s"]:.1f}', f'{x["actual_distance_m"]:.2f}'])
    r.table(['卫生间起点B', 'LLM初始搜索方向', '实际时间 秒', '实际路径 米'], behavior)
    r.paragraph('起点B的3轮卫生间LLM任务中，拥挤和动态两轮先朝卫生间方向，均为47.0秒、3.02米，并与对应IG路线及指标一致。普通公寓轮先探索餐桌方向，为137.0秒、11.33米；对应IG为75.0秒、3.16米。这三轮首次LLM输入均没有碗观测。两次直接朝卫生间行进的LLM理由主要强调信息增益与路径代价，因此不能单凭短路线证明模型推断出了卫生间中的碗。')
    r.paragraph('当前信息增益是从候选观测位置预计可见的未知网格面积；已知障碍会挡住预测视线。IG策略按“信息增益／路径代价加小常数”排序。起点B的卫生间方向候选在普通与拥挤场景中均为4.11平方米增益、1.31米路径代价，而餐桌方向的候选集合随家具变化。普通公寓LLM选择餐桌方向时，其增益代价比约1.63，低于卫生间方向约2.91，说明LLM选择并非始终等于IG排序。区域描述还把餐桌、马桶等类别混在一个较大区域中，限制了语义与具体方向的准确关联。')

    r.heading('失败案例与模型调用')
    failures = [x for x in rows if not x['success']]
    r.table(['目标', '条件', '起点', '方法', '终止原因', '实际秒', '实际米'], [
        [TARGETS[x['target_location']], PROFILES[x['profile']], x['start_id'], LABELS[x['policy']],
         x['termination'], f'{x["simulated_time_s"]:.1f}', f'{x["actual_distance_m"]:.2f}'] for x in failures])
    r.paragraph('no_feasible_frontiers表示当时没有可执行的候选点，不能据此断言整个房屋或目标在物理上不可达；它可能与观测地图、导航可行性及恢复流程有关。search_limit是搜索预算类终止标签，也不必然意味着恰好耗尽600仿真秒。上述三个失败均按既定规则保留，没有重跑替换。当前汇总未逐一完成失败根因重建，因此不将失败归因于特定感知或规划错误。')
    llm = data['completed_policy_stats']['llm_guided']
    r.paragraph(f'LLM共调用{llm["llm_calls"]}次，发生1次客户端运行错误回退，占2.8%；该回退位于卫生间、动态场景、起点A。其余轮次无回退，全部LLM轮次最终成功。调用延迟中位数为{llm["llm_latency_median_s"]:.2f}秒，95百分位为{llm["llm_latency_p95_s"]:.2f}秒，记录的总token数为{llm["llm_tokens_total"]:,}。结果未记录货币费用，不能将记录中的零费用解释为免费。')

    r.heading('结论与下一步')
    r.paragraph('这批实验支持的结论是：LLM在厨房碗任务中减少了搜索时间和行驶距离，并在全部12轮保持成功；其整体路径均值优于IG，但卫生间任务显著削弱了时间优势。报告应同时呈现这两个目标位置的结果，避免仅用厨房案例概括LLM的表现。')
    r.paragraph('优先改进方向是让候选点获得更局部、准确的物体关联，并单独分析Nearest在拥挤起点B的两次无可行候选终止。如果测试更重视语义的提示词，应将其定义为新的方法变体，在厨房和卫生间任务上统一验证。后续增加独立噪声实现或不同房屋，才能更可靠地评价稳定性和泛化。历史杯子与36轮厨房碗实验采用不同任务分组和部分布局设置，本报告不将它们与本次48轮合并统计。')

    r.heading('逐轮结果与数据来源')
    r.paragraph('下表列出全部48轮实际结束时间与路径；失败项在主结果中的计罚时间另按600秒处理。每个任务块的四行具有相同目标、起点、场景和种子。')
    r.table(['目标', '条件', '起点', '方法', '结果', '实际秒', '实际米'], [
        [TARGETS[x['target_location']], PROFILES[x['profile']], x['start_id'], LABELS[x['policy']],
         '成功' if x['success'] else x['termination'], f'{x["simulated_time_s"]:.1f}', f'{x["actual_distance_m"]:.2f}'] for x in rows])
    r.paragraph('本报告直接读取冻结批次的results.json、protocol.json、逐轮summary.json和记录轨迹生成。套件校验确认48轮齐全、12组配对完整、冻结源代码与共享资产未变、所有结果完整性检查通过。实验后生成的可视化不会影响搜索决策或指标。HTML内嵌本报告的五张图，可独立分享；逐轮证据链接需保留原实验目录结构。')
    for name, label in [('results.json', '完整统计JSON'), ('episodes.csv', '逐轮CSV'),
                        ('protocol.json', '冻结实验配置'), ('suite-verification.json', '实验完整性校验'),
                        ('trajectory-scene-status.json', '48张场景轨迹图导出状态')]:
        r.link(label, '../' + name)
    r.heading('逐轮轨迹图索引', 3)
    for x in rows:
        r.link(x['job_id'], '../episodes/' + x['job_id'] + '/trajectory-scene.png')
    r.save()
    manifest = {'source_run': root.name, 'planned': 48, 'completed': 48, 'successes': sum(x['success'] for x in rows),
                'source_results_sha256': hashlib.sha256((root/'results.json').read_bytes()).hexdigest(),
                'protocol_sha256': (root/'protocol.sha256').read_text().strip(),
                'outputs': ['bowl48-report-zh.html', 'bowl48-report-zh.md'],
                'figures': ['target-performance.png', 'trajectory-kitchen-apartment-startA.png',
                            'trajectory-kitchen-apartment-startB.png', 'trajectory-bathroom-apartment-startA.png',
                            'trajectory-bathroom-apartment-startB.png']}
    (out / 'report-provenance.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2), encoding='utf-8')
    print(out)


if __name__ == '__main__':
    main()
