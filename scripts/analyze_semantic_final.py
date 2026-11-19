"""Summarise the final-setup semantic batch (runs/semantic-final-20261111).

Per episode: outcome, simulated time, distance, time until the true target is first tracked
within 0.5 m, LLM calls and fallbacks, and how often the LLM chose the candidate with the
highest information gain per metre of path.
"""
import argparse
import csv
import io
import json
import math
from pathlib import Path
from statistics import mean, median

ROOT = Path(__file__).resolve().parents[1]
CAP_S = 600.
GROUP = {'cup-kitchen-aligned': 'typical', 'banana-kitchen-aligned': 'typical',
         'cup-bathroom-semantic-decoy': 'atypical', 'banana-bathroom-misaligned': 'atypical'}


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def time_to_find(folder, target_class, tx, ty):
    first = None
    for line in io.open(folder / 'events.jsonl', encoding='utf-8'):
        if '"semantic_objects"' not in line:
            continue
        state = json.loads(line).get('state') or {}
        for obj in state.get('semantic_objects') or []:
            p = obj.get('position_world')
            if obj.get('class_name') == target_class and p and math.hypot(p['x_m'] - tx, p['y_m'] - ty) <= .5:
                t = state.get('timestamp_s')
                first = t if first is None else min(first, t)
    return first


def llm_geometry(folder):
    """(model decisions, decisions equal to the max gain-per-path candidate)."""
    path = folder / 'llm-calls.json'
    if not path.exists():
        return 0, 0
    data = read(path)
    calls = data if isinstance(data, list) else data.get('calls', data.get('records', []))
    total = same = 0
    for call in calls:
        if call.get('fallback_reason') or call.get('fallback_selected_action'):
            continue
        chosen = call.get('selected_id') or (call.get('output') or {}).get('frontier_id')
        if chosen is None and isinstance(call.get('provider'), dict):
            raw = call['provider'].get('raw_response') or ''
            try:
                chosen = json.loads(raw.strip('`').replace('json\n', '', 1)).get('frontier_id')
            except (ValueError, AttributeError):
                chosen = None
        cands = (call.get('input') or {}).get('candidates') or []
        if not chosen or not cands:
            continue
        best = max(cands, key=lambda c: c['information_gain_m2'] / (c['path_cost_m'] + 1e-6))['frontier_id']
        total += 1
        same += chosen == best
    return total, same


def collect(run_root):
    rows = []
    for start_dir in sorted(p for p in run_root.glob('S*') if p.is_dir()):
        cfg = read(ROOT / 'configs' / f'semantic-final-start-{start_dir.name}.json')
        targets = {t['task_id']: t['target'] for t in cfg['tasks']}
        for folder in sorted((start_dir / 'episodes').glob('*/')):
            summary_path = folder / 'summary.json'
            if not summary_path.exists():
                continue
            s = read(summary_path)
            task, policy = folder.name.rsplit('-', 1)[0], folder.name.rsplit('-', 1)[1]
            for t in targets:
                if folder.name.startswith(t + '-'):
                    task, policy = t, folder.name[len(t) + 1:]
            tgt = targets[task]
            tx, ty = tgt['position_world'][:2]
            found = time_to_find(folder, tgt['class_name'], tx, ty)
            calls, fallbacks = s.get('llm_calls', 0), s.get('llm_fallbacks', 0)
            decided, same = llm_geometry(folder)
            rows.append(dict(start=start_dir.name, task=task, group=GROUP.get(task), policy=policy,
                             termination=s['termination'], success=s['termination'] == 'success',
                             time_s=round(s['simulated_time_s'], 1),
                             capped_s=round(s['simulated_time_s'] if s['termination'] == 'success' else CAP_S, 1),
                             distance_m=round(s['actual_distance_m'], 2),
                             find_s=None if found is None else round(found, 1),
                             llm_calls=calls, llm_fallbacks=fallbacks,
                             llm_valid=policy != 'llm_guided' or (calls and (calls - fallbacks) / calls >= .8),
                             llm_decided=decided, llm_same_as_geometry=same))
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run-root', type=Path, default=ROOT / 'runs/semantic-final-20261111')
    args = ap.parse_args()
    rows = collect(args.run_root)
    out = args.run_root / 'analysis'
    out.mkdir(exist_ok=True)
    with open(out / 'episodes.csv', 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    pairs = {}
    for r in rows:
        pairs.setdefault((r['start'], r['task']), {})[r['policy']] = r
    summary = dict(episodes=len(rows), pairs=[])
    for (start, task), d in sorted(pairs.items()):
        ig, llm = d.get('information_gain'), d.get('llm_guided')
        summary['pairs'].append(dict(start=start, task=task, group=GROUP.get(task),
                                     ig=ig and {k: ig[k] for k in ('termination', 'time_s', 'distance_m', 'find_s')},
                                     llm=llm and {k: llm[k] for k in ('termination', 'time_s', 'distance_m', 'find_s',
                                                                     'llm_calls', 'llm_fallbacks', 'llm_valid')}))
    for pol in ('information_gain', 'llm_guided'):
        for grp in ('typical', 'atypical'):
            sel = [r for r in rows if r['policy'] == pol and r['group'] == grp]
            finds = [r['find_s'] if r['find_s'] is not None else CAP_S for r in sel]
            summary[f'{pol}_{grp}'] = dict(n=len(sel), success=sum(r['success'] for r in sel),
                                           mean_capped_s=round(mean(r['capped_s'] for r in sel), 1),
                                           mean_find_s=round(mean(finds), 1), median_find_s=round(median(finds), 1),
                                           found=sum(r['find_s'] is not None for r in sel))
    llm_rows = [r for r in rows if r['policy'] == 'llm_guided']
    summary['llm_decisions'] = sum(r['llm_decided'] for r in llm_rows)
    summary['llm_same_as_geometry'] = sum(r['llm_same_as_geometry'] for r in llm_rows)
    summary['llm_calls'] = sum(r['llm_calls'] for r in llm_rows)
    summary['llm_fallbacks'] = sum(r['llm_fallbacks'] for r in llm_rows)
    (out / 'summary.json').write_text(json.dumps(summary, indent=1), encoding='utf-8')
    print(json.dumps(summary, indent=1))


if __name__ == '__main__':
    main()
