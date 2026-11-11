"""Aggregate semantic-navigation replicates across start poses and policies.

Sources (S0 is split over three earlier run roots; S1-S3 live under the replicate root):
  S0 nearest / information_gain : runs/semantic-navigation-smoke-20261110-r2  (Gemini-era root; baselines only)
  S0 llm_guided (v1, DeepSeek)  : runs/semantic-llm-deepseek-20261111
  S0 llm_guided_v2, S1..S3 all  : runs/semantic-replicates-20261111/<start>
"""
import argparse
from collections import Counter, defaultdict
import glob
import json
from math import comb
import os
from pathlib import Path
from statistics import mean, median

ROOT = Path(__file__).resolve().parents[1]
CAP_S = 600.
POLICIES = ('nearest', 'information_gain', 'llm_guided', 'llm_guided_v2')
GROUPS = {  # semantic relation between the target and where it was placed
    'scissors-desk-aligned': 'consistent', 'cup-kitchen-aligned': 'consistent',
    'banana-kitchen-aligned': 'consistent', 'bottle-kitchen-ambiguous': 'consistent',
    'scissors-kitchen-misaligned': 'anomalous', 'cup-bathroom-semantic-decoy': 'anomalous',
    'banana-bathroom-misaligned': 'anomalous', 'bottle-bathroom-ambiguous': 'anomalous',
}


def load(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def sources(rep_root):
    yield 'S0', ROOT / 'runs/semantic-navigation-smoke-20261110-r2', ('nearest', 'information_gain')
    yield 'S0', ROOT / 'runs/semantic-llm-deepseek-20261111', ('llm_guided',)
    for start in ('S0', 'S1', 'S2', 'S3'):
        yield start, rep_root / start, POLICIES


def rows(rep_root):
    out = []
    for start, root, policies in sources(rep_root):
        if not (root / 'results.json').exists():
            continue
        for row in load(root / 'results.json')['results']:
            if row['policy'] in policies:
                out.append(dict(row, start=start, root=str(root)))
    return out


FIND_RADIUS_M = .5


def target_positions():
    out = {}
    for path in glob.glob(str(ROOT / 'configs/semantic-navigation-*.json')):
        for task in load(path).get('tasks', ()):
            out[task['task_id']] = task['target']['position_world'][:2]
    return out


def time_to_find(row, positions):
    """First time a target-class track lay within FIND_RADIUS_M of the true target.

    Separates search efficiency from the downstream two-frame verification; None if never found."""
    path = Path(row['root']) / 'episodes' / row['job_id'] / 'final-state.json'
    if not path.exists():
        return None
    state = load(path)
    tx, ty = positions[row['task_id']]
    times = [track['first_seen_s'] for track in state.get('semantic_objects', ())
             if track.get('class_name') == state['target_class'] and track.get('position_world')
             and ((track['position_world']['x_m'] - tx) ** 2 +
                  (track['position_world']['y_m'] - ty) ** 2) ** .5 <= FIND_RADIUS_M]
    return min(times) if times else None


def paired_table(lines, index, title, metric):
    lines += ['', title, '',
              '| A vs B | Pairs | A faster | Tie | B faster | Mean A-B s | Sign-test p |',
              '|---|---:|---:|---:|---:|---:|---:|']
    for a, b in (('llm_guided', 'information_gain'), ('llm_guided_v2', 'information_gain'),
                 ('llm_guided_v2', 'llm_guided'), ('information_gain', 'nearest'),
                 ('llm_guided', 'nearest'), ('llm_guided_v2', 'nearest')):
        pairs = [(index[(s, t, a)], index[(s, t, b)]) for (s, t, p) in index
                 if p == a and (s, t, b) in index]
        if not pairs:
            continue
        diffs = [metric(x) - metric(y) for x, y in pairs]
        win = sum(d < -1 for d in diffs)
        loss = sum(d > 1 for d in diffs)
        lines.append(f'| {a} vs {b} | {len(pairs)} | {win} | {len(pairs) - win - loss} | {loss} | '
                     f'{mean(diffs):+.1f} | {sign_test(win, loss):.3f} |')


def capped(row):
    return row['simulated_time_s'] if row['success'] else CAP_S


def sign_test(wins, losses):
    n = wins + losses
    if n == 0:
        return 1.
    k = min(wins, losses)
    return min(1., 2 * sum(comb(n, i) for i in range(k + 1)) / 2 ** n)


def llm_behaviour(row):
    """Decision-level statistics from the episode's llm-calls.json."""
    path = Path(row['root']) / 'episodes' / row['job_id'] / 'llm-calls.json'
    if not path.exists():
        return None
    stats = Counter()
    target_probs, first_prob, tokens = [], None, 0
    for record in load(path):
        provider = record.get('provider') or {}
        usage = provider.get('usage') or {}
        tokens += usage.get('total_tokens') or 0
        if record['fallback_reason']:
            stats['fallback'] += 1
            continue
        candidates = record['input']['candidates']
        pick = record['response']
        best = max(candidates, key=lambda c: c.get('gain_per_path_m') or 0)['frontier_id']
        stats['n'] += 1
        stats['max_gain_per_path'] += pick == best
        probs = (provider.get('answer') or {}).get('region_probabilities')
        if isinstance(probs, dict):
            value = probs.get(row['target_region'])
            if isinstance(value, (int, float)):
                target_probs.append(value)
                if first_prob is None:
                    first_prob = value
    return dict(stats=stats, target_probs=target_probs, first_target_prob=first_prob, tokens=tokens)


def fmt(value, digits=1):
    return '-' if value is None else f'{value:.{digits}f}'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--replicates', type=Path, default=ROOT / 'runs/semantic-replicates-20261111')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    data = rows(args.replicates)
    lines = ['# Semantic navigation replicate analysis', '']
    starts = sorted({row['start'] for row in data})
    lines.append(f'Episodes: {len(data)} across starts {", ".join(starts)}. '
                 f'Failures are scored at the {CAP_S:.0f} s search cap in capped-time metrics.')
    invalid = [row['job_id'] + '@' + row['start'] for row in data if not row.get('llm_valid', True)]
    if invalid:
        lines.append('Excluded LLM episodes (provider fallback > 20%): ' + ', '.join(invalid))
    data = [row for row in data if row.get('llm_valid', True)]
    positions = target_positions()
    for row in data:
        row['found_s'] = time_to_find(row, positions)
        row['found_capped_s'] = CAP_S if row['found_s'] is None else row['found_s']

    # 1. Overall table per policy.
    lines += ['', '## Overall', '',
              '| Policy | Episodes | Success | False claims | Mean capped time s | Median capped s | '
              'Mean time on success s | Mean distance on success m |', '|---|---:|---:|---:|---:|---:|---:|---:|']
    for policy in POLICIES:
        group = [row for row in data if row['policy'] == policy]
        if not group:
            continue
        wins = [row for row in group if row['success']]
        lines.append('| {} | {} | {}/{} ({:.0%}) | {} | {} | {} | {} | {} |'.format(
            policy, len(group), len(wins), len(group), len(wins) / len(group),
            sum(row['termination'] == 'false_target_claim' for row in group),
            fmt(mean(map(capped, group))), fmt(median(map(capped, group))),
            fmt(mean(row['simulated_time_s'] for row in wins) if wins else None),
            fmt(mean(row['actual_distance_m'] for row in wins) if wins else None, 2)))

    # 1b. Search efficiency independent of the downstream verification.
    lines += ['', '## Search efficiency: time until the true target is first tracked', '',
              f'Target-class track within {FIND_RADIUS_M} m of the true position; never found = {CAP_S:.0f} s.', '',
              '| Policy | Found | Mean find time s (capped) | Median s | Found but task not completed |',
              '|---|---:|---:|---:|---:|']
    for policy in POLICIES:
        group = [row for row in data if row['policy'] == policy]
        if group:
            found = [row for row in group if row['found_s'] is not None]
            lines.append(f"| {policy} | {len(found)}/{len(group)} | "
                         f"{mean(r['found_capped_s'] for r in group):.1f} | "
                         f"{median(r['found_capped_s'] for r in group):.1f} | "
                         f"{sum(not r['success'] for r in found)} |")
    lines += ['', '| Group | Policy | Mean find time s (capped) |', '|---|---|---:|']
    for name in ('consistent', 'anomalous'):
        for policy in POLICIES:
            group = [row for row in data if row['policy'] == policy and GROUPS[row['task_id']] == name]
            if group:
                lines.append(f"| {name} | {policy} | {mean(r['found_capped_s'] for r in group):.1f} |")

    # 2. By semantic group.
    lines += ['', '## By semantic relation of placement', '',
              '| Group | Policy | Success | Mean capped time s |', '|---|---|---:|---:|']
    for name in ('consistent', 'anomalous'):
        for policy in POLICIES:
            group = [row for row in data if row['policy'] == policy and GROUPS[row['task_id']] == name]
            if group:
                lines.append(f"| {name} | {policy} | {sum(r['success'] for r in group)}/{len(group)} | "
                             f"{fmt(mean(map(capped, group)))} |")

    # 3. Per task x policy.
    lines += ['', '## Per task (success count / capped mean s over starts)', '',
              '| Task | ' + ' | '.join(POLICIES) + ' |', '|---|' + '---:|' * len(POLICIES)]
    for task in GROUPS:
        cells = []
        for policy in POLICIES:
            group = [row for row in data if row['policy'] == policy and row['task_id'] == task]
            cells.append('-' if not group else
                         f"{sum(r['success'] for r in group)}/{len(group)} / {mean(map(capped, group)):.0f}")
        lines.append(f'| {task} | ' + ' | '.join(cells) + ' |')

    # 4. Paired comparisons on identical (start, task).
    index = {(row['start'], row['task_id'], row['policy']): row for row in data}
    paired_table(lines, index, '## Paired comparisons on task completion '
                 '(same start and task; capped time, 1 s tie tolerance)', capped)
    paired_table(lines, index, '## Paired comparisons on time-to-find the true target',
                 lambda row: row['found_capped_s'])

    # 5. LLM decision behaviour.
    lines += ['', '## LLM decision behaviour', '',
              '| Policy | Model decisions | = max gain/path | Fallbacks | Mean tokens / episode | '
              'Mean wall s / episode |', '|---|---:|---:|---:|---:|---:|']
    behaviour = {}
    for policy in ('llm_guided', 'llm_guided_v2'):
        group = [row for row in data if row['policy'] == policy]
        info = [(row, llm_behaviour(row)) for row in group]
        info = [(row, b) for row, b in info if b]
        behaviour[policy] = info
        if not info:
            continue
        n = sum(b['stats']['n'] for _, b in info)
        same = sum(b['stats']['max_gain_per_path'] for _, b in info)
        lines.append(f"| {policy} | {n} | {same} ({same / n:.0%}) | "
                     f"{sum(b['stats']['fallback'] for _, b in info)} | "
                     f"{mean(b['tokens'] for _, b in info):.0f} | {mean(row['wall_time_s'] for row, _ in info):.0f} |")

    v2 = behaviour.get('llm_guided_v2') or []
    if v2:
        lines += ['', '## v2 belief about the true target region', '',
                  'Probability v2 assigned to the region that actually held the target '
                  '(uniform over 4 regions = 0.25).', '',
                  '| Group | Episodes | First-decision prob (mean) | All-decision prob (mean) |',
                  '|---|---:|---:|---:|']
        for name in ('consistent', 'anomalous'):
            sel = [b for row, b in v2 if GROUPS[row['task_id']] == name]
            first = [b['first_target_prob'] for b in sel if b['first_target_prob'] is not None]
            every = [p for b in sel for p in b['target_probs']]
            lines.append(f'| {name} | {len(sel)} | {fmt(mean(first) if first else None, 2)} | '
                         f'{fmt(mean(every) if every else None, 2)} |')

    # 6. First selected region match.
    lines += ['', '## First selected region equals target region', '']
    for policy in POLICIES:
        group = [row for row in data if row['policy'] == policy]
        if group:
            hit = sum(bool(row['first_choice_matches_target']) for row in group)
            lines.append(f'- {policy}: {hit}/{len(group)}; first regions: '
                         + ', '.join(f'{k} x{v}' for k, v in Counter(r['first_selected_region'] for r in group).most_common()))

    # 6b. Navigation traps: no feasible frontier very early (robot blocked, planner finds no path).
    lines += ['', '## Early navigation traps (no_feasible_frontiers before 100 s sim)', '']
    for policy in POLICIES:
        trapped = [row for row in data if row['policy'] == policy
                   and row['termination'] == 'no_feasible_frontiers' and row['simulated_time_s'] < 100]
        group = [row for row in data if row['policy'] == policy]
        if group:
            lines.append(f'- {policy}: {len(trapped)}/{len(group)}' + (
                ' (' + ', '.join(r['task_id'] + '@' + r['start'] for r in trapped) + ')' if trapped else ''))

    # 7. Termination mix.
    lines += ['', '## Terminations', '']
    for policy in POLICIES:
        group = [row for row in data if row['policy'] == policy]
        if group:
            lines.append(f'- {policy}: ' + ', '.join(f'{k} {v}' for k, v in Counter(r['termination'] for r in group).most_common()))

    text = '\n'.join(lines) + '\n'
    if args.output:
        args.output.write_text(text, encoding='utf-8')
    print(text)


if __name__ == '__main__':
    main()
