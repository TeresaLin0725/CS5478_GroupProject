"""Run and summarize the semantic-navigation diagnostic suite."""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[1]
REGIONS = {
    'R1': (-2.05, -4.2, 1.95, 1.5),
    'R2': (-4.1, -2.2, -2.05, 1.5),
    'R3': (-4.1, 1.5, -2.1, 3.6),
    'R4': (-2.1, 1.5, 1.95, 3.6),
}


def utc():
    return datetime.now(timezone.utc).isoformat()


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def atomic(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')
    temporary.replace(path)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def region_for(x, y):
    for name, (x0, y0, x1, y1) in REGIONS.items():
        if x0 <= x <= x1 and y0 <= y <= y1:
            return name
    return None


LLM_VALID_MIN_COVERAGE = .8


def jobs(suite, task_ids, policies=()):
    selected = [task for task in suite['tasks'] if not task_ids or task['task_id'] in task_ids]
    missing = sorted(set(task_ids) - {task['task_id'] for task in selected})
    if missing:
        raise ValueError('unknown task id(s): ' + ', '.join(missing))
    unknown = sorted(set(policies) - set(suite['policies']))
    if unknown:
        raise ValueError('unknown policy(ies): ' + ', '.join(unknown))
    return [dict(task=task, policy=policy,
                 job_id=task['task_id'] + '-' + policy)
            for task in selected for policy in suite['policies']
            if not policies or policy in policies]


def llm_quota_probe():
    """One minimal request; returns None when the hosted model answers, else the error."""
    sys.path.insert(0, str(ROOT / 'src'))
    from frontier_search.policies.provider import load_local_env
    load_local_env(ROOT.parent / '.env')
    key = os.environ.get('LLM_API_KEY')
    if not key:
        return 'LLM_API_KEY missing'
    base = os.environ.get('LLM_BASE_URL',
                          'https://generativelanguage.googleapis.com/v1beta/openai').rstrip('/')
    request = dict(url=base + '/chat/completions', key=key, timeout=30,
                   body=dict(model=os.environ.get('LLM_MODEL', 'gemini-3.5-flash'),
                             messages=[dict(role='user', content='Reply OK')]))
    result = subprocess.run([sys.executable, '-m', 'frontier_search.policies.transport'],
                            input=json.dumps(request), stdout=subprocess.PIPE, encoding='utf-8',
                            cwd=str(ROOT), env=dict(os.environ, PYTHONPATH=str(ROOT / 'src')),
                            timeout=45, check=False)
    try:
        return json.loads(result.stdout).get('error')
    except (json.JSONDecodeError, AttributeError):
        return 'probe transport failed'


def is_llm(policy):
    return policy.startswith('llm_guided')


def policy_arguments(policy, suite=None):
    # 'llm_guided_v2' is the same runtime policy with the v2 system prompt.
    values = (['--policy', 'llm_guided', '--llm-prompt-version', 'v2'] if policy == 'llm_guided_v2'
              else ['--policy', policy])
    if is_llm(policy) and suite and suite.get('llm_deadline_s'):
        values += ['--llm-deadline-s', str(suite['llm_deadline_s'])]
    return values


def command(suite, job, folder):
    task, target = job['task'], job['task']['target']
    values = [sys.executable, '-m', 'frontier_search.runner.integrated',
              '--output', str(folder), '--profile', suite['profile'],
              *policy_arguments(job['policy'], suite), '--target-class', target['class_name'],
              '--target-model', target['model'], '--target-position',
              *map(str, target['position_world']), '--target-roll', str(target.get('roll_rad', 0.)),
              '--target-pitch', str(target.get('pitch_rad', 0.)),
              '--target-yaw', str(target.get('yaw_rad', 0.)),
              '--target-instance-id', target['instance_id'], '--start-pose',
              *map(str, suite['start_pose']), '--environment-seed', str(suite['environment_seed']),
              '--perception-seed', str(suite['perception_seed']),
              '--policy-seed', str(suite['policy_seed'])]
    values += noise_arguments(suite)
    if suite.get('robust_mapping'):
        values.append('--robust-mapping')
    if suite.get('initialization_deadline_s') is not None:
        values += ['--initialization-deadline-s', str(suite['initialization_deadline_s'])]
    return values


def noise_arguments(suite):
    """Clean depth unless the suite enables noise; the per-job seed follows the bowl48 rule
    (base seed + environment seed), so every policy sees the same noise stream."""
    noise = suite.get('sensor_noise') or {}
    if not noise.get('enabled'):
        return ['--no-noise']
    return ['--noise', '--depth-noise-std-m', str(noise['depth_std_m']),
            '--depth-dropout-prob', str(noise['depth_dropout_prob']),
            '--noise-seed', str(noise['base_seed'] + suite['environment_seed'])]


def event_rows(folder):
    path = folder / 'events.jsonl'
    if not path.exists():
        return []
    rows = []
    for line in path.open(encoding='utf-8'):
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return rows


def result_row(root, suite, job):
    folder = root / 'episodes' / job['job_id']
    summary = read(folder / 'summary.json')
    events = event_rows(folder)
    decisions = [event for event in events if event.get('event') == 'decision']
    first_subgoal = ((decisions[0].get('selection') or {}).get('subgoal') or {}) if decisions else {}
    pose = first_subgoal.get('approach_pose') or {}
    first_region = region_for(pose.get('x_m', float('inf')), pose.get('y_m', float('inf')))
    selected_regions = []
    for event in decisions:
        subgoal = ((event.get('selection') or {}).get('subgoal') or {})
        candidate = subgoal.get('approach_pose') or {}
        region = region_for(candidate.get('x_m', float('inf')), candidate.get('y_m', float('inf')))
        if region and region not in selected_regions:
            selected_regions.append(region)
    row = dict(job_id=job['job_id'], task_id=job['task']['task_id'],
               family=job['task']['family'], condition=job['task']['condition'],
               hypothesis=job['task']['hypothesis'], target_region=job['task']['target_region'],
               policy=job['policy'], termination=summary.get('termination'),
               success=summary.get('termination') == 'success',
               simulated_time_s=summary.get('simulated_time_s'),
               actual_distance_m=summary.get('actual_distance_m'),
               wall_time_s=summary.get('wall_time_s'), collisions=summary.get('collisions', 0),
               decisions=summary.get('decisions', 0), attempts=summary.get('attempts', 0),
               llm_attempts=summary.get('llm_attempts', 0), llm_calls=summary.get('llm_calls', 0),
               llm_fallbacks=summary.get('llm_fallbacks', 0),
               first_selected_region=first_region,
               first_choice_matches_target=first_region == job['task']['target_region'],
               selected_regions=selected_regions,
               evaluation=summary.get('evaluation', {}),
               source_unchanged=summary.get('source_unchanged'))
    # A nominal llm_guided episode only measures the model when the model actually chose.
    model_decisions = row['llm_calls'] - row['llm_fallbacks']
    coverage = model_decisions / row['llm_calls'] if row['llm_calls'] else 0.
    row.update(llm_model_decisions=model_decisions, llm_decision_coverage=coverage,
               llm_valid=(not is_llm(job['policy']) or coverage >= LLM_VALID_MIN_COVERAGE))
    atomic(folder / 'semantic-result.json', row)
    return row


def write_reports(root, suite, selected_jobs):
    rows = [result_row(root, suite, job) for job in selected_jobs
            if (root / 'episodes' / job['job_id'] / 'summary.json').exists()]
    by_task = []
    for task in {job['task']['task_id']: job['task'] for job in selected_jobs}.values():
        task_rows = [row for row in rows if row['task_id'] == task['task_id']]
        by_task.append(dict(task_id=task['task_id'], family=task['family'],
            condition=task['condition'], target_region=task['target_region'],
            hypothesis=task['hypothesis'], results=task_rows))
    complete = len(rows) == len(selected_jobs)
    report = dict(schema_version='1.0', complete=complete, planned=len(selected_jobs),
                  completed=len(rows), generated_utc=utc(), results=rows, by_task=by_task)
    atomic(root / 'results.json', report)
    lines = ['# Semantic navigation diagnostic results', '',
             f"Completed: {len(rows)}/{len(selected_jobs)} episodes.", '',
             '| Task | Condition | Policy | Result | Sim s | Distance m | First region | LLM calls/fallbacks | Model-chosen decisions | Valid |',
             '|---|---|---|---:|---:|---:|---|---:|---:|---|']
    for row in rows:
        lines.append('| {task_id} | {condition} | {policy} | {termination} | {sim} | {distance} | {region} | {calls}/{fallbacks} | {chosen} | {valid} |'.format(
            **row, sim='-' if row['simulated_time_s'] is None else f"{row['simulated_time_s']:.1f}",
            distance='-' if row['actual_distance_m'] is None else f"{row['actual_distance_m']:.2f}",
            region=row['first_selected_region'] or '-', calls=row['llm_calls'], fallbacks=row['llm_fallbacks'],
            chosen='-' if not is_llm(row['policy']) else f"{row['llm_model_decisions']}/{row['llm_calls']}",
            valid='yes' if row['llm_valid'] else 'NO (provider fallback)'))
    lines += ['', 'Interpretation is diagnostic: one episode per task/policy is not a statistical superiority claim.',
              f'llm_guided rows need >= {LLM_VALID_MIN_COVERAGE:.0%} model-chosen decisions to count as LLM evidence; '
              'otherwise they replay the nearest fallback.', '']
    (root / 'report.md').write_text('\n'.join(lines), encoding='utf-8')
    return report


def run_one(root, suite, job):
    folder = root / 'episodes' / job['job_id']
    if (folder / 'summary.json').exists():
        return job['job_id'], 'retained', 0
    if folder.exists() and any(folder.iterdir()):
        raise RuntimeError('interrupted episode retained without summary: ' + job['job_id'])
    folder.mkdir(parents=True, exist_ok=True)
    environment = dict(os.environ, PYTHONPATH=str(ROOT / 'src'), PYTHONIOENCODING='utf-8',
                       YOLO_CONFIG_DIR=str(root / 'yolo-config'))
    started = time.time()
    with (folder / 'console.log').open('x', encoding='utf-8') as log:
        process = subprocess.run(command(suite, job, folder), cwd=str(ROOT), env=environment,
                                 stdout=log, stderr=subprocess.STDOUT,
                                 creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
    atomic(folder / 'supervisor.json', dict(returncode=process.returncode,
           process_wall_s=time.time() - started, finished_utc=utc()))
    if not (folder / 'summary.json').exists():
        raise RuntimeError(job['job_id'] + ' exited without summary; see console.log')
    return job['job_id'], read(folder / 'summary.json').get('termination'), process.returncode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('plan', 'run', 'summarize', 'all'))
    parser.add_argument('--config', type=Path, default=ROOT / 'configs/semantic-navigation-smoke.json')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--preflight', type=Path,
                        help='Passed preflight.json to reuse when planning a fresh run root.')
    parser.add_argument('--workers', type=int, default=2)
    parser.add_argument('--task-id', action='append', default=[])
    parser.add_argument('--policy', action='append', default=[],
                        help='Restrict to these policies (repeatable), e.g. to rerun only llm_guided.')
    parser.add_argument('--skip-llm-probe', action='store_true',
                        help='Do not check provider quota before starting llm_guided episodes.')
    args = parser.parse_args()
    suite, root = read(args.config), args.output.resolve()
    selected_jobs = jobs(suite, args.task_id, args.policy)
    root.mkdir(parents=True, exist_ok=True)
    if args.action in ('plan', 'all'):
        preflight_path = (args.preflight or root / 'preflight' / 'preflight.json').resolve()
        preflight = read(preflight_path)
        expected = {job['task']['task_id'] for job in selected_jobs}
        passed = {row['task_id'] for row in preflight['results'] if row['passed']}
        if not preflight.get('passed') or not expected <= passed:
            raise RuntimeError('full semantic target preflight has not passed')
        plan = dict(schema_version='1.0', created_utc=utc(), config=str(args.config.resolve()),
                    config_sha256=sha256(args.config), workers=args.workers,
                    preflight=str(preflight_path), preflight_sha256=sha256(preflight_path),
                    scope='diagnostic smoke; no statistical superiority claim',
                    jobs=[dict(job_id=job['job_id'], policy=job['policy'], task=job['task'])
                          for job in selected_jobs])
        atomic(root / 'experiment-plan.json', plan)
        atomic(root / 'progress.json', dict(status='planned', planned=len(selected_jobs),
                                             completed=0, active=[], updated_utc=utc()))
    if args.action in ('run', 'all'):
        if not (root / 'experiment-plan.json').exists():
            raise RuntimeError('plan is missing; run plan or all first')
        pending = [job for job in selected_jobs
                   if not (root / 'episodes' / job['job_id'] / 'summary.json').exists()]
        completed = len(selected_jobs) - len(pending)
        if not args.skip_llm_probe and any(is_llm(job['policy']) for job in pending):
            error = llm_quota_probe()
            if error:
                raise RuntimeError(f'LLM provider unavailable ({error}); llm_guided episodes would '
                                   'silently replay the nearest fallback. Fix quota or pass --skip-llm-probe.')
        with ThreadPoolExecutor(max_workers=args.workers) as pool:
            futures = {pool.submit(run_one, root, suite, job): job for job in pending}
            for future in as_completed(futures):
                job_id, termination, returncode = future.result()
                completed += 1
                print('episode_finished', job_id, termination, f'{completed}/{len(selected_jobs)}', flush=True)
                atomic(root / 'progress.json', dict(status='running' if completed < len(selected_jobs) else 'complete',
                    planned=len(selected_jobs), completed=completed,
                    active=[item['job_id'] for item in pending if not (root / 'episodes' / item['job_id'] / 'summary.json').exists()],
                    last_job=job_id, last_returncode=returncode, updated_utc=utc()))
                write_reports(root, suite, selected_jobs)
    if args.action in ('summarize', 'all'):
        report = write_reports(root, suite, selected_jobs)
        print(json.dumps(dict(complete=report['complete'], completed=report['completed'],
                              planned=report['planned']), ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
