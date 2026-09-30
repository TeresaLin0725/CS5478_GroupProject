"""Copy one iGibson run into Documentation/igibson_demo with a compact results.json."""
import argparse
import json
from pathlib import Path
import shutil

NAMES = ('nearest', 'llm_guided')


def episode(run, name):
    result = {'decisions': [], 'navigation': []}
    config = None
    for line in (run / (name + '.jsonl')).read_text(encoding='utf-8').splitlines():
        record = json.loads(line)
        event = record['event']
        if event == 'demo_config':
            config = {k: v for k, v in record.items()
                      if k not in ('event', 'model', 'episode_id', 'sequence', 'timestamp', 'wall_time')}
        elif event == 'decision':
            context, selection = record['context'], record['selection']
            costs = {a['frontier_id']: a['path_cost_m'] for a in context['evaluation']['assessments']}
            result['decisions'].append({
                'state_revision': record['state']['revision'],
                'candidates': [{'frontier_id': f['frontier_id'], 'region_id': f['region_id'],
                                'path_cost_m': costs[f['frontier_id']],
                                'eligible': f['frontier_id'] in context['eligible_frontier_ids']}
                               for f in record['state']['frontiers']],
                'observed_classes': sorted({o['class_name'] for o in record['state']['semantic_objects']}),
                'selected': (selection['subgoal'] or {}).get('frontier_id'),
                'stop_reason': selection['stop_reason'], 'rationale': selection['rationale']})
        elif event == 'llm_call':
            call = record['record']
            result['decisions'][-1]['llm'] = {'latency_s': round(call['latency_s'], 3),
                                              'fallback_reason': call['fallback_reason'], 'usage': call['usage']}
        elif event == 'navigation_feedback':
            attempt = record['attempt']
            result['navigation'].append({'frontier_id': attempt['subgoal']['frontier_id'],
                                         'execution': attempt['execution_status'],
                                         'failure': attempt['failure_reason'],
                                         'executed_distance_m': record['executed_distance_m'],
                                         'replans': record['replans']})
        elif event == 'terminal_evaluation':
            result['evaluation'] = {'valid_target': record['valid_target'],
                                    'position_error_m': record['position_error_m'],
                                    'evidence': record['claim']['evidence_observation_ids']}
    result['summary'] = json.loads((run / (name + '-summary.json')).read_text(encoding='utf-8'))
    return config, result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--destination', type=Path, default=Path('Documentation/igibson_demo'))
    args = parser.parse_args()
    args.destination.mkdir(parents=True, exist_ok=True)
    output = {'run_directory': args.run.as_posix(), 'episodes': {}}
    for name in NAMES:
        output['config'], output['episodes'][name] = episode(args.run, name)
        for suffix in ('.gif', '-start.png', '-final.png'):
            shutil.copy(args.run / (name + suffix), args.destination)
    (args.destination / 'results.json').write_text(json.dumps(output, indent=2, ensure_ascii=False),
                                                   encoding='utf-8')
    print('Exported', args.destination.resolve())


if __name__ == '__main__':
    main()
