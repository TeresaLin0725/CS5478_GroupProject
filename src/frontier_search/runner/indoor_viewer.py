"""Portable, offline viewer of real iGibson rendered frames (no invented poses)."""
import json
from pathlib import Path

from ..simulation.indoor_challenges import PROFILES

LABELS = {'random': 'Random', 'nearest': 'Nearest', 'information_gain': 'Information Gain',
          'llm_guided': 'LLM Interface replay'}


def export_viewer(directory):
    directory = Path(directory)
    episodes = []
    for summary_file in sorted(directory.rglob('*-summary.json')):
        summary = json.loads(summary_file.read_text(encoding='utf-8'))
        if summary.get('source') != 'igibson_static_scan' or 'profile' not in summary:
            continue
        name = summary['policy']
        base = summary_file.parent
        trace_file = base / (name + '-trace.json')
        if not trace_file.exists():
            continue
        trace = json.loads(trace_file.read_text(encoding='utf-8'))
        prefix = base.relative_to(directory).as_posix()
        prefix = '' if prefix == '.' else prefix + '/'
        for frame in trace['frames']:
            frame['image'] = prefix + frame['image']
        video = trace.get('video')
        if video:
            video['file'] = prefix + video['file']
        events = []
        for line in (base / (name + '.jsonl')).read_text(encoding='utf-8').splitlines():
            row = json.loads(line)
            if row['event'] == 'decision':
                goal = row['selection']['subgoal']
                if goal:
                    events.append({'time_s': row['state']['timestamp_s'], 'text':
                        ('Approach the observed target' if goal['kind'] == 'target_approach' else 'C selected ' + goal['frontier_id'])})
            if row['event'] == 'navigation_feedback':
                events.append({'time_s': row['sim_time_s'], 'text':
                    'B returned ' + row['attempt']['execution_status'] + ' · Replans ' + str(row['replans']) + ' times'})
            if row['event'] == 'feedback_flow':
                kind = row['kind']
                labels = {
                    'A_to_B_route_invalidated': 'A Detected invalidation of the original route → B Stop and check',
                    'B_local_replan': 'B Replan the current goal: ' + ('Reachable' if row.get('reachable') else 'No safe path'),
                    'B_to_C_replan_request': 'B → C Request reselection: ' + row.get('detail',''),
                    'C_to_B_strategy_replan': 'C → B Dispatch new goal ' + row.get('selected_goal',''),
                }
                if kind in labels:
                    events.append({'time_s': row['time_s'], 'text': labels[kind]})
        events += trace['environment_events']
        episodes.append({'id': summary['profile'] + '/' + name, 'summary': summary,
            'frames': trace['frames'], 'video': video, 'events': sorted(events, key=lambda e: e['time_s']),
            'log': prefix + name + '.jsonl', 'summary_file': prefix + name + '-summary.json',
            'label': 'LLM (live calls) ' if name == 'llm_guided' and summary.get('backend') == 'live' else LABELS[name]})
    payload = json.dumps({'episodes': episodes, 'profiles': PROFILES}, ensure_ascii=False).replace('</', r'<\/')
    template = Path(__file__).with_name('indoor_viewer.html').read_text(encoding='utf-8')
    (directory / 'index.html').write_text(template.replace('__DEMO_DATA__', payload), encoding='utf-8')
    return len(episodes)
