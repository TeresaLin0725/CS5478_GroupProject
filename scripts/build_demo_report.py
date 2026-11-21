"""Build a local, evidence-linked report without changing simulation artifacts."""
import argparse
import copy
from datetime import datetime, timedelta, timezone
from html import escape
import json
import math
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RUNS = ROOT / 'runs'
SUITE = RUNS / 'igibson-realistic-final-20261014'
PROFILES = {'apartment': 'Apartment object search', 'clutter': 'Cluttered-passage search', 'dynamic': 'Object search with a moving cart'}
POLICIES = {'random': 'Uniformly random reachable frontier', 'nearest': 'Nearest reachable frontier policy',
            'information_gain': 'Information-gain-to-path-cost policy', 'llm_guided': 'Authored semantic-response replay'}
STATUS = {'reached': 'Completed', 'failed': 'Failed', 'cancelled': 'Cancelled'}
FAILURES = {'invalid_goal': 'Goal invalidated by a map update', 'blocked': 'Motion blocked',
            'no_path': 'No feasible path found', 'timeout': 'Timeout'}


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def logs(path):
    # Discarding only large map arrays keeps the report small; raw logs stay linked.
    for line in path.open(encoding='utf-8'):
        yield json.loads(line)


def pose_text(pose):
    return 'x %.2f m, y %.2f m, heading %.1f degrees' % (
        pose['x_m'], pose['y_m'], math.degrees(pose['yaw_rad']))


def reason_text(selection):
    if not selection['subgoal']:
        return 'No navigation goal dispatched in this decision; Stop reason: %s. See operation records for subsequent recovery and termination. ' % selection['stop_reason']
    if selection['subgoal']['kind'] == 'target_approach':
        return 'Approach the observed target, collect stationary close-range evidence; seeing the target alone does not complete the task. '
    return {
        'minimum assessed path length': 'Select the reachable frontier with the shortest assessed path.. ',
        'uniform eligible choice': 'Choose uniformly among reachable frontier candidates.. ',
        'maximum information gain / (path cost + epsilon)': 'Select the frontier maximizing predicted information gain divided by path cost.. ',
        'authored IG interface replay; not model inference': 'Read authored frontier responses, the reply sequence comes from an information-gain run in this scene. ',
    }.get(selection['rationale'], 'Follow the configured frontier-selection rule; See the original decision record for details. ')


def decision_row(row, number, call=None):
    selection = row['selection']
    goal = selection['subgoal']
    previous = row.get('context', {}).get('previous_attempt')
    failure = previous.get('failure_reason') if previous else None
    return dict(number=number, time=row['state']['timestamp_s'], goal=pose_text(goal['approach_pose']) if goal else 'No navigation goal dispatched',
        kind=('Approach the detected target' if goal['kind'] == 'target_approach' else 'Explore and observe') if goal else 'Stop request and recovery assessment',
        reason=call['reason'] if call else reason_text(selection),
        feedback=FAILURES.get(failure, failure) if failure else ('Previous navigation completed' if previous else 'First goal selection'),
        candidate_count=len(row.get('context', {}).get('eligible_frontier_ids', [])),
        provider=('Live hosted model: ' + call['response_model']) if call else
            ('Deterministic target-approach control' if goal and goal['kind'] == 'target_approach' else 'Local policy rules'),
        latency=call['latency_s'] if call else None,
        raw=dict(selection=selection, previous_failure=failure,
                 model_reply=call['raw_response'] if call else None), goal_id=goal['frontier_id'] if goal else None)


def record_dispatch(decision, request, action_source=None):
    """Show the dispatched goal while preserving the original policy response."""
    goal = request['subgoal']
    policy_goal = decision['raw']['selection']['subgoal']
    changed = policy_goal is None or policy_goal['frontier_id'] != goal['frontier_id']
    source = action_source or ('recovery' if changed else 'policy')
    decision['raw'].update(executed_subgoal=goal, action_source=source)
    decision.update(goal=pose_text(goal['approach_pose']), goal_id=goal['frontier_id'])
    if source != 'policy':
        label = {'epsilon_retry': 'Probabilistic recovery retry', 'stuck_retry': 'Recovery retry after no eligible goals'}.get(source, 'Recovery retry')
        original = policy_goal['frontier_id'] if policy_goal else 'No navigation goal selected'
        decision.update(kind='Recovery retry', provider='Shared recovery rules',
            reason='Original policy: %s; %s, Dispatched goal: %s. ' % (original, label, goal['frontier_id']))


def integrated(folder, key, title, recording=None):
    directory = Path(recording).resolve() if recording else RUNS / folder / 'recording'
    summary, manifest = read(directory / 'summary.json'), read(directory / 'manifest.json')
    calls = read(directory / 'llm-calls.json') if (directory / 'llm-calls.json').exists() else []
    if not calls:
        for row in logs(directory / 'events.jsonl'):
            if row['event'] == 'llm_call':
                record = row['record']; provider = record.get('provider') or {}
                if provider.get('raw_response') and not record.get('fallback_reason'):
                    calls.append(dict(simulated_time_s=record['input'].get('timestamp_s', 0),
                        state_revision=record['state_revision'], response_model=provider.get('response_model', record['model']),
                        reason=provider.get('reason', ''), latency_s=record['latency_s'],
                        raw_response=provider['raw_response']))
    call_by_time = {round(c['simulated_time_s'], 6): (n, c) for n, c in enumerate(calls)}
    call_by_revision = {c['state_revision']: (n, c) for n, c in enumerate(calls) if 'state_revision' in c}
    decisions, operations, feedback = [], [], []
    op_by_id, current_decision, action_source = {}, None, None
    target_evaluation = None
    for row in logs(directory / 'events.jsonl'):
        event = row['event']
        if event == 'decision':
            action_source = None
            match = call_by_revision.get(row['state']['revision']) or call_by_time.get(round(row['state']['timestamp_s'], 6))
            current_decision = len(decisions) + 1
            decisions.append(decision_row(row, current_decision, match[1] if match else None))
        elif event == 'epsilon_probe':
            action_source = 'epsilon_retry' if row['mode'] == 'epsilon' else 'stuck_retry'
        elif event == 'action_selected':
            action_source = row['action_source']
        elif event in ('dispatch', 'observation_start'):
            request = row['request']
            if event == 'dispatch' and current_decision is not None:
                decision = decisions[current_decision-1]
                record_dispatch(decision, request, action_source)
                if decision['raw']['action_source'] != 'policy':
                    feedback.append(dict(time=row.get('simulated_time_s', decision['time']),
                        kind='Recovery dispatched goal', detail=decision['reason'], raw=row))
            kind = 'Navigation drive' if event == 'dispatch' else {
                'initial': 'Initial environment observation', 'arrival': 'Arrival observation', 'recovery': 'Recovery observation after obstruction',
                'inspection': 'Stationary multi-view target inspection'
            }[request['routine']]
            op = dict(number=len(operations) + 1, time=None, end=None, time_basis=None, kind=kind,
                decision=current_decision, goal=(pose_text(request['subgoal']['approach_pose'])
                  if event == 'dispatch' else 'Observe while stationary and follow the scheduled headings'), status='', detail='', raw={})
            operations.append(op)
            op_by_id[request['goal_id']] = op
        elif event == 'operation_feedback':
            f = row['feedback']
            if f['status'] == 'accepted':
                op_by_id[f['goal_id']]['time'] = f['timestamp_s']
                op_by_id[f['goal_id']]['time_basis'] = 'accepted'
        elif event == 'inspection_start':
            operations.append(dict(number=len(operations)+1, time=row['simulated_time_s'], end=None,
                time_basis='inspection_start', kind='Stationary target inspection', decision=current_decision,
                goal='Keep the brakes applied, continue physics steps and RGB-D capture', status='',
                detail='Perception determines whether stationary evidence is available, the independent evaluator verifies the claim. ', raw=row))
        elif event == 'inspection_end':
            op=next(o for o in reversed(operations) if o['kind']=='Stationary target inspection' and o['end'] is None)
            op.update(end=row['simulated_time_s'],status='Completed',
                detail='Stationary observation window ended; continue searching if no target claim was formed. ')
        elif event == 'operation_terminal':
            f = row['feedback']; op = op_by_id[f['goal_id']]
            op.update(end=f['timestamp_s'], status=STATUS[f['status']],
                detail=FAILURES.get(f['failure_reason'], f['failure_reason']) or (
                    'Stop request received, operation cancelled. ' if f['status']=='cancelled' else 'Ended according to the operation completion conditions. '),
                raw=f)
            if op['time'] is None:
                op.update(time=f['timestamp_s'],time_basis='terminal_only',
                    detail='Rejected at operation start, the log records only the terminal timestamp; '+op['detail'])
        elif event == 'navigation_map_feedback' and row['kind'] == 'replan':
            ok = 'success=True' in row['detail']
            feedback.append(dict(time=row['simulated_time_s'], kind='Local navigation replanning',
                detail='Recompute the path after a map update, ' + ('Feasible path found. ' if ok else 'Goal position invalidated, No feasible path obtained. '),
                raw=row))
        elif event == 'replan_request':
            feedback.append(dict(time=row['simulated_time_s'], kind='Navigation failure reported to the policy',
                detail='Navigation planning and control reported an invalid goal, The policy receives the failure reason and robot pose. ', raw=row))
        elif event == 'strategy_replan':
            recovery = row.get('action_source', 'policy') != 'policy'
            feedback.append(dict(time=row['simulated_time_s'], kind='Recovery redispatched goal' if recovery else 'Policy selected and dispatched a new navigation goal',
                detail='Final goal: %s; Source: %s. ' % (row['selected']['frontier_id'],
                    'Shared recovery rules' if recovery else 'Search policy'), raw=row))
        elif event == 'target_evaluation':
            target_evaluation = row
    for op in operations:
        assert op['time'] is not None
        if op['end'] is None:
            op.update(end=summary['simulated_time_s'], status='Ended at task verification',
                detail='Target verification passed, the episode ended; this observation has no separately logged terminal status. ')
    scene_names={'cart_route_verified':'Dynamic cart route passed scene validation',
                 'cart_crossing':'Dynamic cart started along the verified route',
                 'cart_parked':'Dynamic cart parked beside the passage',
                 'cart_blocked':'Dynamic cart stopped because its swept volume was blocked'}
    for scene_event in summary.get('scene_events',[]):
        feedback.append(dict(time=scene_event['time_s'],kind=scene_names.get(scene_event['event'],scene_event['event']),
            detail='Scene events originate in the native simulation; Cart movement affects mapping only through actual RGB-D observations. ',raw=scene_event))
    assert len(decisions) == summary['decisions']
    assert sum(op['kind'] == 'Navigation drive' for op in operations) == summary['attempts']
    distances = summary['evaluation']['evidence_distances_m']
    claims = target_evaluation['evidence']['detections']
    evidence = [dict(time=c['timestamp_s'], detail='Target detection confidence %.3f; robot-to-target-instance distance %.3f m; stationary requirement satisfied. ' %
        (c['confidence'], distance)) for c, distance in zip(claims, distances)]
    evidence.append(dict(time=summary['simulated_time_s'], detail='Independent target verification passed; the two evidence frames have distinct capture times, matching the retained target instance, the robot is stationary at task termination. '))
    observation_angles = ', '.join('%.0f' % math.degrees(a) for a in manifest['observation']['arrival_headings_rad'])
    media = read(directory / 'video.json')
    return dict(key=key, title=title, group='Integrated pipeline', directory=str(directory),
        source_fingerprint=summary['source_fingerprint'], policy=summary['policy'],
        target='Cup', success=summary['termination'] == 'success', duration=summary['simulated_time_s'],
        wall=summary['wall_time_s'], distance=summary['actual_distance_m'], contacts=summary['collisions'],
        decisions=decisions, operations=operations, feedback=sorted(feedback,key=lambda f:f['time']), evidence=evidence,
        model_calls=summary['llm_calls'], model_fallbacks=summary['llm_fallbacks'],
        llm_failure_reselections=sum(d['latency'] is not None and bool(d['raw']['previous_failure']) for d in decisions),
        scope='Native simulated RGB-D camera → object detector YOLO11s → Perception and mapping → Navigation planning and control → Policy decisions and recovery → Independent target verifier. ',
        setup='Gibson Rs textured apartment scan; Locobot embodied wheeled robot; '+{
            'apartment':'Original scanned layout. ','clutter':'Furniture occupancy and narrow passages. ','dynamic':'Furniture occupancy and a moving cart. '}[summary['profile']]
            +'Relative observation headings at arrival: '+observation_angles+' degrees, dwell at each heading for %.1f s. ' % manifest['observation']['dwell_s'],
        assessment='Navigation %d times, completed %d times, Failed %d times; observation operations blocked %d times; Local path replanning %d times. ' % (
            summary['attempts'], sum(o['kind']=='Navigation drive' and o['status']=='Completed' for o in operations),
            sum(o['kind']=='Navigation drive' and o['status']=='Failed' for o in operations),
            sum(o['kind']!='Navigation drive' and o['status']=='Failed' for o in operations), len([f for f in feedback if f['kind']=='Local navigation replanning'])),
        limits='This item contains one complete recording, this does not estimate success over repeated runs. Wall-clock time includes rendering,, Perception, policy and control processing, Video playback follows simulation time. ',
        operation_scope='Operation start and end times come from acceptance and terminal feedback; Operations rejected at start have only a terminal timestamp, execution duration is not calculated. Stops or cancellations at target verification retain their logged status. ',
        media_type='video', video=str(directory / 'integrated.mp4'), frames=[], fps=media['fps'],
        frame_count=len(media['frames']), poster=str(directory / 'start.png'), final=str(directory / 'final.png'),
        links=[('Run summary', directory/'summary.json'), ('Full event log', directory/'events.jsonl'),
               ('Run parameters', directory/'manifest.json'), ('Independent verification evidence', directory/'evaluation-only.json'),
               ('Video and physical-trajectory verification', directory/'artifact-verification.json')]
              + [(label, path) for label, path in [('Original model response', directory/'llm-calls.json'),
                  ('Model-call verification', directory/'llm-verification.json'),
                  ('Motion controller configuration', directory/'controller.json')] if path.is_file()])


def relative_case(case, directory):
    result = copy.deepcopy(case)
    result.pop('directory')
    def relative(path):
        return os.path.relpath(path, directory).replace('\\', '/')
    for name in ('video', 'poster', 'final'):
        if result.get(name):
            result[name] = relative(result[name])
    result['links'] = [dict(title=title, href=relative(path)) for title, path in result['links']]
    for frame in result['frames']:
        frame['image'] = relative(frame['image'])
    return result


def build_report(directory, default=None, cases=None, notes=None):
    """Render explicitly supplied recordings; no author-machine run folders needed."""
    if not cases:
        raise ValueError('Provide at least one recording.')
    directory = Path(directory).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    local = [relative_case(case, directory) for case in cases]
    payload = dict(cases=local, default=default or local[0]['key'])
    encoded = json.dumps(payload, ensure_ascii=False).replace('</', r'<\/')
    template = (ROOT/'scripts/templates/demo_report.html').read_text(encoding='utf-8')
    page = directory/'index.html'
    page.write_text(template.replace('__DATA__', encoded), encoding='utf-8')
    return page


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--recording', type=Path, action='append', required=True,
                        help='Verified recording directory; repeat for multiple policies.')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    cases = []
    for number, recording in enumerate(args.recording):
        summary = read(recording/'summary.json')
        artifact = read(recording/'artifact-verification.json')
        if summary['termination'] != 'success' or not artifact['passed']:
            parser.error('Each recording must have passed independent artifact verification.')
        title = summary['profile'].title() + ' / ' + summary['policy']
        cases.append(integrated(None, 'recording-'+str(number), title, recording=recording))
    directory = (args.output or args.recording[0]).resolve()
    page = build_report(directory, cases=cases)
    inventory = [dict(key=case['key'], title=case['title'], duration_s=case['duration'],
                      distance_m=case['distance'], contacts=case['contacts'],
                      model_calls=case['model_calls'], source_fingerprint=case['source_fingerprint'])
                 for case in cases]
    (directory/'inventory.json').write_text(json.dumps(inventory, indent=2), encoding='utf-8')
    print(page)


if __name__ == '__main__':
    main()
