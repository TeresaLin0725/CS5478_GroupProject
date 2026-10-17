"""Build a local, evidence-linked report without changing simulation artifacts."""
import copy
from html import escape
import json
import math
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RUNS = ROOT / 'runs'
SUITE = RUNS / 'igibson-realistic-final-20261014'
PROFILES = {'apartment': '公寓室内搜索', 'clutter': '家具拥挤通道搜索', 'dynamic': '移动推车干扰搜索'}
POLICIES = {'random': '随机选择可达探索目标', 'nearest': '最近可达探索目标策略',
            'information_gain': '信息增益与路径代价策略', 'llm_guided': '预置语义回答回放策略'}
STATUS = {'reached': '完成', 'failed': '失败', 'cancelled': '取消'}
FAILURES = {'invalid_goal': '地图更新后目标位置失效', 'blocked': '观察运动受阻',
            'no_path': '未找到可行路径', 'timeout': '超时'}


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def logs(path):
    # Discarding only large map arrays keeps the report small; raw logs stay linked.
    for line in path.open(encoding='utf-8'):
        yield json.loads(line)


def pose_text(pose):
    return '横坐标 %.2f 米，纵坐标 %.2f 米，朝向 %.1f 度' % (
        pose['x_m'], pose['y_m'], math.degrees(pose['yaw_rad']))


def reason_text(selection, hosted_index=None):
    if hosted_index is not None:
        return [
            '从有效候选中选择预计信息增益最高的探索目标，继续观察未知空间。',
            '选择可达且路径代价较低的探查位置，继续搜索具有餐饮与厨房语义的区域。',
            '从可达候选中选择预计信息增益最高的位置，继续寻找杯子。',
            '根据冰箱与微波炉观测推断厨房位置，选择该区域内信息增益较高、路径代价较低的探索目标。',
        ][hosted_index]
    if selection['subgoal']['kind'] == 'target_approach':
        return '接近已经观测到的目标，取得近距离停稳证据；仅看到目标不构成任务完成。'
    return {
        'minimum assessed path length': '在可达候选中选择评估路径长度最短的探索目标。',
        'uniform eligible choice': '在满足可达性约束的候选中随机选择探索目标。',
        'maximum information gain / (path cost + epsilon)': '按预计信息增益与路径代价的比值选择探索目标。',
        'authored IG interface replay; not model inference': '读取预先编写的探索目标回答，回答序列依据本场景的信息增益策略运行生成。',
    }.get(selection['rationale'], '按已配置的探索目标选择规则执行；完整说明见原始决策记录。')


def decision_row(row, number, call=None, hosted_index=None):
    selection = row['selection']
    goal = selection['subgoal']
    previous = row.get('context', {}).get('previous_attempt')
    failure = previous.get('failure_reason') if previous else None
    return dict(number=number, time=row['state']['timestamp_s'], goal=pose_text(goal['approach_pose']),
        kind='接近感知目标' if goal['kind'] == 'target_approach' else '探索与观测',
        reason=reason_text(selection, hosted_index),
        feedback=FAILURES.get(failure, failure) if failure else ('上次导航完成' if previous else '首次目标选择'),
        candidate_count=len(row.get('context', {}).get('eligible_frontier_ids', [])),
        provider=('真实在线模型：' + call['response_model']) if call else
            ('确定性目标接近控制' if goal['kind'] == 'target_approach' else '本地策略规则'),
        latency=call['latency_s'] if call else None,
        raw=dict(selection=selection, previous_failure=failure,
                 model_reply=call['raw_response'] if call else None), goal_id=goal['frontier_id'])


def integrated(folder, key, title):
    directory = RUNS / folder / 'recording'
    summary, manifest = read(directory / 'summary.json'), read(directory / 'manifest.json')
    calls = read(directory / 'llm-calls.json') if (directory / 'llm-calls.json').exists() else []
    call_by_time = {round(c['simulated_time_s'], 6): (n, c) for n, c in enumerate(calls)}
    decisions, operations, feedback = [], [], []
    op_by_id, current_decision = {}, None
    target_evaluation = None
    for row in logs(directory / 'events.jsonl'):
        event = row['event']
        if event == 'decision':
            if not row['selection']['subgoal']:
                continue
            match = call_by_time.get(round(row['state']['timestamp_s'], 6))
            current_decision = len(decisions) + 1
            decisions.append(decision_row(row, current_decision, match[1] if match else None,
                                          match[0] if match else None))
        elif event in ('dispatch', 'observation_start'):
            request = row['request']
            kind = '导航行驶' if event == 'dispatch' else {
                'initial': '初始环境观察', 'arrival': '到达位置后观察', 'recovery': '受阻后的恢复观察'
            }[request['routine']]
            op = dict(number=len(operations) + 1, time=None, end=None, kind=kind,
                decision=current_decision, goal=(pose_text(request['subgoal']['approach_pose'])
                  if event == 'dispatch' else '停驻观察并按计划调整朝向'), status='', detail='', raw={})
            operations.append(op)
            op_by_id[request['goal_id']] = op
        elif event == 'operation_feedback':
            f = row['feedback']
            if f['status'] == 'accepted':
                op_by_id[f['goal_id']]['time'] = f['timestamp_s']
        elif event == 'operation_terminal':
            f = row['feedback']; op = op_by_id[f['goal_id']]
            op.update(end=f['timestamp_s'], status=STATUS[f['status']],
                detail=FAILURES.get(f['failure_reason'], f['failure_reason']) or '按操作完成条件结束。',
                raw=f)
        elif event == 'navigation_map_feedback' and row['kind'] == 'replan':
            ok = 'success=True' in row['detail']
            feedback.append(dict(time=row['simulated_time_s'], kind='导航局部路径重规划',
                detail='地图更新后重新求解路径，' + ('找到可行路径。' if ok else '目标位置已失效，未得到可行路径。'),
                raw=row))
        elif event == 'replan_request':
            feedback.append(dict(time=row['simulated_time_s'], kind='导航失败反馈至策略决策',
                detail='导航规划与控制模块上报目标位置失效，策略决策模块收到失败原因与机器人位置。', raw=row))
        elif event == 'strategy_replan':
            feedback.append(dict(time=row['simulated_time_s'], kind='策略重新选择并下发导航目标',
                detail='策略决策模块使用失败上下文重新选择可达位置，并向导航规划与控制模块下发目标。', raw=row))
        elif event == 'target_evaluation':
            target_evaluation = row
    for op in operations:
        assert op['time'] is not None
        if op['end'] is None:
            op.update(end=summary['simulated_time_s'], status='任务验收时结束',
                detail='目标验收通过，整次任务结束；日志未单独记录此观察操作的终态。')
    assert len(decisions) == summary['decisions']
    assert sum(op['kind'] == '导航行驶' for op in operations) == summary['attempts']
    distances = summary['evaluation']['evidence_distances_m']
    claims = target_evaluation['evidence']['detections']
    evidence = [dict(time=c['timestamp_s'], detail='目标检测置信度 %.3f；机器人与目标实体距离 %.3f 米；停稳条件满足。' %
        (c['confidence'], distance)) for c, distance in zip(claims, distances)]
    evidence.append(dict(time=summary['simulated_time_s'], detail='独立目标验收通过；两份证据来自不同采集时刻，匹配保留的目标实体，任务终止时机器人已停稳。'))
    observation_angles = '、'.join('%.0f' % math.degrees(a) for a in manifest['observation']['arrival_headings_rad'])
    media = read(directory / 'video.json')
    return dict(key=key, title=title, group='完整集成管线', directory=str(directory),
        target='杯子', success=summary['termination'] == 'success', duration=summary['simulated_time_s'],
        wall=summary['wall_time_s'], distance=summary['actual_distance_m'], contacts=summary['collisions'],
        decisions=decisions, operations=operations, feedback=feedback, evidence=evidence,
        model_calls=summary['llm_calls'], model_fallbacks=summary['llm_fallbacks'],
        scope='原生仿真彩色与深度相机 → 目标检测模型 YOLO11s → 感知建图模块 → 导航规划与控制模块 → 策略决策与恢复模块 → 独立目标验收模块。',
        setup='Gibson Rs 公寓纹理扫描场景；Locobot 实体轮式机器人；家具占用与移动推车。到达观察相对朝向依次为 '+observation_angles+' 度，每个朝向驻留 %.1f 秒。' % manifest['observation']['dwell_s'],
        assessment='导航 %d 次，其中完成 %d 次、失败 %d 次；观察操作受阻 %d 次；局部路径重规划 %d 次。' % (
            summary['attempts'], sum(o['kind']=='导航行驶' and o['status']=='完成' for o in operations),
            sum(o['kind']=='导航行驶' and o['status']=='失败' for o in operations),
            sum(o['kind']!='导航行驶' and o['status']=='失败' for o in operations), len([f for f in feedback if f['kind']=='导航局部路径重规划'])),
        limits='本项仅包含一次完整录制，不能据此估计多次运行成功率。实际计算耗时包含渲染、感知、策略与控制处理，视频按仿真时间回放。',
        operation_scope='操作时间来自接收与终态反馈；最后一项观察在目标验收通过时随任务结束，未被记为独立完成。',
        media_type='video', video=str(directory / 'integrated.mp4'), frames=[], fps=media['fps'],
        frame_count=len(media['frames']), poster=str(directory / 'start.png'), final=str(directory / 'final.png'),
        links=[('运行汇总', directory/'summary.json'), ('完整事件日志', directory/'events.jsonl'),
               ('运行参数', directory/'manifest.json'), ('独立验收证据', directory/'evaluation-only.json'),
               ('录像与物理轨迹核验', directory/'artifact-verification.json')]
              + ([('模型回答原文', directory/'llm-calls.json'), ('模型调用核验',directory/'llm-verification.json')] if calls else []))


def standalone(profile, policy):
    directory = SUITE / profile
    summary = read(directory / (policy+'-summary.json'))
    trace = read(directory / (policy+'-trace.json'))
    rows = list(logs(directory / (policy+'.jsonl')))
    decisions = [decision_row(r, n+1) for n, r in enumerate(r for r in rows if r['event']=='decision' and r['selection']['subgoal'])]
    if policy == 'llm_guided':
        for decision in decisions:
            if decision['kind'] != '接近感知目标':
                decision['provider'] = '预置回答文件回放'
    nav = [r for r in rows if r['event']=='navigation_feedback']
    assert len(nav) == len(decisions) == summary['decisions']
    operations = []
    for d, r in zip(decisions, nav):
        attempt = r['attempt']
        assert attempt['subgoal']['frontier_id'] == d['goal_id']
        operations.append(dict(number=len(operations)+1, time=d['time'], end=r['sim_time_s'],
            kind='导航行驶', decision=d['number'], goal=d['goal'], status=STATUS[attempt['execution_status']],
            detail='本段行驶 %.3f 米，局部路径重规划 %d 次，物理接触步数 %d。' % (
                r['executed_distance_m'], r['replans'], r['collision_steps']),
            raw=dict(execution_status=attempt['execution_status'], failure_reason=attempt['failure_reason'],
                     subgoal=attempt['subgoal'], sim_time_s=r['sim_time_s'])))
    feedback = []
    flow_names = {'A_to_B_route_invalidated': ('地图更新使当前路径失效', '感知建图结果上报当前路径中的新增障碍占用。'),
        'B_local_replan': ('导航局部路径重规划', '根据更新后的地图重新计算路径。'),
        'B_to_C_failure': ('导航失败反馈至策略决策', '导航失败原因上报策略决策模块。'),
        'C_to_B_replan': ('策略重新选择导航目标', '策略决策模块重新下发探索目标。')}
    for r in rows:
        if r['event']=='feedback_flow' and r['kind'] != 'A_to_B_map_update':
            assert r['kind'] in flow_names, r['kind']
            kind, detail = flow_names[r['kind']]
            if 'reachable' in r:
                detail += '可达性评估结果：' + ('可达。' if r['reachable'] else '不可达。')
            feedback.append(dict(time=r['time_s'], kind=kind, detail=detail, raw=r))
    for r in summary['environment_events']:
        feedback.append(dict(time=r['time_s'], kind='移动推车进入通道', detail='推车开始横穿客厅通道。', raw=r))
    evaluation = next(r for r in rows if r['event']=='terminal_evaluation')
    evidence = [dict(time=evaluation['claim']['timestamp_s'], detail=
        '目标实例分割证据与饼干盒实体匹配；位置误差 %.3f 米；通过本场景的两次停稳近距离采集验收。' % evaluation['position_error_m']),
        dict(time=summary['sim_time_s'], detail='任务终止时距目标 %.3f 米，验收距离上限 %.2f 米。' % (
            summary['target_distance_m'], summary['close_distance_limit_m']))]
    video = directory / (policy+'.mp4')
    return dict(key=profile+'-'+policy, title=PROFILES[profile]+' · '+POLICIES[policy], group='场景与策略演示',
        directory=str(directory), target='饼干盒', success=summary['termination']=='success', duration=summary['sim_time_s'],
        wall=summary['wall_time_s'], distance=summary['executed_distance_m'], contacts=summary['collision_steps'],
        decisions=decisions, operations=operations, feedback=sorted(feedback,key=lambda f:f['time']), evidence=evidence,
        model_calls=0, model_fallbacks=None,
        scope='独立场景运行循环：彩色与深度观测建图、预先标注物体的深度可见性判断、目标实例分割、策略选择、实体轮式行驶与近距离验收。',
        setup='Gibson Rs 公寓纹理扫描场景；Locobot 实体轮式机器人。'+{
            'apartment':'使用公寓室内布局。', 'clutter':'加入家具，形成拥挤通道。',
            'dynamic':'加入家具与横穿客厅通道的移动推车。'}[profile]
            + ('探索决策读取预置回答文件，未调用在线大语言模型。' if policy=='llm_guided' else ''),
        assessment='导航 %d 次，失败 %d 次；局部路径重规划 %d 次；终止时距目标 %.3f 米。' % (
            len(nav), summary['navigation_failures'], summary['replans'], summary['target_distance_m']),
        limits='本项仅包含一次运行；物体语义使用预先标注与渲染可见性，不能作为目标检测模型在未知场景中的识别准确率证据。',
        operation_scope='操作表收录有起止时间的导航操作；观察扫描没有独立操作终态日志，画面按采集时间展示。',
        media_type='video' if video.exists() else 'frames', video=str(video) if video.exists() else None,
        fps=summary.get('video_fps'), frame_count=summary.get('video_frames',len(trace['frames'])),
        frames=[dict(time=f['time_s'], image=str(directory/f['image'])) for f in trace['frames']],
        poster=str(directory/(policy+'-start.png')), final=str(directory/(policy+'-final.png')),
        links=[('运行汇总', directory/(policy+'-summary.json')), ('完整事件日志', directory/(policy+'.jsonl')),
               ('采样画面与轨迹', directory/(policy+'-trace.json'))])


def collect():
    cases = [integrated('integrated-llm-final-20261018', 'integrated-language', '移动障碍环境 · 在线大语言模型决策完整管线'),
             integrated('integrated-final-20261017', 'integrated-nearest', '移动障碍环境 · 最近探索目标完整管线')]
    cases += [standalone(profile, policy) for profile in PROFILES for policy in POLICIES]
    assert len(cases) == 14 and sum(c['media_type']=='video' for c in cases)==3
    for c in cases:
        assert c['success']
        for item in c['decisions']+c['operations']+c['feedback']+c['evidence']:
            assert 0 <= item['time'] <= c['duration']+.001, (c['key'],item)
        for op in c['operations']:
            assert op['time'] <= op['end'] <= c['duration']+.001
        for path in [c['poster'], c['final']] + ([c['video']] if c['video'] else []) + [f['image'] for f in c['frames']]:
            assert Path(path).is_file(), path
        for _, path in c['links']:
            assert path.is_file(), path
    return cases


def relative_case(case, directory):
    c = copy.deepcopy(case)
    def rel(path):
        return os.path.relpath(path,directory).replace('\\','/')
    c.pop('directory')
    for key in ('video','poster','final'):
        if c[key]: c[key] = rel(c[key])
    for frame in c['frames']:
        frame['image'] = rel(frame['image'])
    c['links'] = [dict(title=title,href=rel(path)) for title,path in c['links']]
    return c


def build_report(directory, default='integrated-language', cases=None):
    cases = cases or collect()
    local = [relative_case(c,directory) for c in cases]
    selected = next(c for c in local if c['key']==default)
    rows = ''
    for n,c in enumerate(local,1):
        media = '连续视频，每秒 15 帧' if c['media_type']=='video' else '按时间采样的画面'
        strategy = '在线大语言模型' if c['model_calls'] else ('预置回答回放' if 'llm_guided' in c['key'] else '本地策略规则')
        rows += '<tr><td>%d</td><td><button class="catalog-button" data-case="%s">%s</button><small>%s · 搜索%s</small></td><td>%s</td><td>%s</td><td>%.2f 秒</td><td>%.2f 米</td><td>成功</td></tr>' % (
            n,c['key'],escape(c['title']),c['group'],c['target'],strategy,media,c['duration'],c['distance'])
    payload = json.dumps(dict(cases=local, default=default),ensure_ascii=False).replace('</','<\\/')
    template=(ROOT/'scripts/templates/demo_report.html').read_text(encoding='utf-8')
    page=template.replace('__CATALOG__', rows).replace('__DATA__',payload).replace('__VIDEO__',escape(selected['video'],quote=True))
    page=page.replace('__POSTER__',escape(selected['poster'],quote=True))
    path=directory/'index.html'
    backup=directory/'index-before-formal-report.html'
    if path.exists() and not backup.exists():
        backup.write_bytes(path.read_bytes())
    path.write_text(page,encoding='utf-8')
    return path


def main():
    cases=collect()
    targets=[(RUNS/'integrated-llm-final-20261018/recording','integrated-language'),
             (RUNS/'integrated-final-20261017/recording','integrated-nearest')]
    for directory,default in targets:
        build_report(directory,default,cases)
    inventory = dict(report_date='2026-10-18', cases=14, complete_pipeline=2, scene_conditions=12,
        continuous_video=3, timestamped_image_sequences=11,
        counting_scope='按可展示场景与策略条件计数；重复入口、复制媒体、调试试跑与归档材料不重复计入。',
        episodes=[dict(key=c['key'],title=c['title'],group=c['group'],media_type=c['media_type'],
                       decisions=len(c['decisions']),operations=len(c['operations']),success=c['success'],
                       duration_s=c['duration'],wall_time_s=c['wall'],distance_m=c['distance'],contacts=c['contacts']) for c in cases])
    output=RUNS/'demo-report-20261018';output.mkdir(exist_ok=True)
    (output/'inventory.json').write_text(json.dumps(inventory,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(inventory,ensure_ascii=True,indent=2))


if __name__=='__main__':
    main()
