"""Compare bowl48 llm_guided episodes across hosted models and export per-call CoT.

Baseline rows (Gemini + Random/Nearest/Information Gain) come from the archived
bowl48 report; each model folder under --runs is a model-swapped llm_guided rerun.
Writes comparison.md, episodes.csv and <model>/llm-cot.jsonl into --runs.
"""
import argparse
import csv
import json
from pathlib import Path
from statistics import mean

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'report/bowl-48-fixed-20261116-01'
POLICY_LABEL = {'random': 'Random', 'nearest': 'Nearest', 'information_gain': 'Information Gain'}


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def baseline_rows():
    rows = []
    with (BASE / 'episodes.csv').open(encoding='utf-8-sig') as f:
        for r in csv.DictReader(f):
            label = 'gemini-3.5-flash-lite' if r['policy'] == 'llm_guided' else POLICY_LABEL[r['policy']]
            rows.append(dict(method=label, pair_id=r['pair_id'], target=r['target_location'],
                             success=r['success'] == 'True', termination=r['termination'],
                             sim_s=float(r['simulated_time_s']), path_m=float(r['actual_distance_m']),
                             wall_s=float(r['wall_time_s']), decisions=int(r['decisions']),
                             collisions=int(r['collisions']), llm_calls=int(r['llm_calls']),
                             llm_fallbacks=int(r['llm_fallbacks']), cot_calls=0, cot_chars=0))
    return rows


def model_rows(folder):
    rows, cot = [], []
    for ep in sorted((folder / 'episodes').iterdir()):
        s = read(ep / 'summary.json')
        calls = read(ep / 'llm-calls.json') if (ep / 'llm-calls.json').exists() else []
        called = [c for c in calls if c.get('model_called')]
        fallbacks = [c for c in calls if c.get('fallback_reason')]
        n_cot = 0
        for i, c in enumerate(calls):
            p = c.get('provider') or {}
            text = p.get('reasoning_content') or p.get('reasoning') or ''
            n_cot += bool(text)
            cot.append(dict(job_id=s['job_id'], call_index=i, state_revision=c.get('state_revision'),
                            model=c.get('model'), response_model=p.get('response_model'),
                            model_called=c.get('model_called'), fallback_reason=c.get('fallback_reason'),
                            selected=c.get('response'), validated_ranking=c.get('validated_ranking'),
                            latency_s=c.get('latency_s'), usage=c.get('usage'),
                            reasoning_content=text or None, reason=p.get('reason'),
                            raw_response=p.get('raw_response'), input=c.get('input')))
        rows.append(dict(method=folder.name, pair_id=s['pair_id'], target=s['target_location'],
                         success=s['termination'] == 'success', termination=s['termination'],
                         sim_s=s['simulated_time_s'], path_m=s['actual_distance_m'],
                         wall_s=s['wall_time_s'], decisions=s['decisions'], collisions=s['collisions'],
                         llm_calls=len(called), llm_fallbacks=len(fallbacks), cot_calls=n_cot,
                         cot_chars=sum(len(x['reasoning_content'] or '') for x in cot if x['job_id'] == s['job_id'])))
    with (folder / 'llm-cot.jsonl').open('w', encoding='utf-8') as f:
        for item in cot:
            f.write(json.dumps(item, ensure_ascii=False) + '\n')
    return rows


def table(rows, methods):
    out = ['| Method | Success | Kitchen time (s) | Kitchen path (m) | Bathroom time (s) | Bathroom path (m) '
           '| Mean time (s) | Mean path (m) | LLM calls | Fallbacks | Calls with CoT | Contacts |',
           '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
    for m in methods:
        r = [x for x in rows if x['method'] == m]
        if not r:
            continue
        k = [x for x in r if x['target'] == 'kitchen']
        b = [x for x in r if x['target'] == 'bathroom']
        f = lambda xs, key: '%.2f' % mean(x[key] for x in xs) if xs else '-'
        out.append('| %s | %d/%d | %s | %s | %s | %s | %s | %s | %d | %d | %d | %d |' % (
            m, sum(x['success'] for x in r), len(r), f(k, 'sim_s'), f(k, 'path_m'), f(b, 'sim_s'),
            f(b, 'path_m'), f(r, 'sim_s'), f(r, 'path_m'), sum(x['llm_calls'] for x in r),
            sum(x['llm_fallbacks'] for x in r), sum(x['cot_calls'] for x in r),
            sum(x['collisions'] for x in r)))
    return '\n'.join(out)


def paired(rows, a, b):
    """Per matched setup: does model a beat model b on simulated time?"""
    ra = {x['pair_id']: x for x in rows if x['method'] == a}
    rb = {x['pair_id']: x for x in rows if x['method'] == b}
    keys = sorted(set(ra) & set(rb))
    wins = sum(ra[k]['sim_s'] < rb[k]['sim_s'] - 1e-9 for k in keys)
    losses = sum(ra[k]['sim_s'] > rb[k]['sim_s'] + 1e-9 for k in keys)
    return '%s vs %s on %d setups: %d faster / %d ties / %d slower (simulated time)' % (
        a, b, len(keys), wins, len(keys) - wins - losses, losses)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--runs', type=Path, default=ROOT / 'runs/bowl-llm-models-20261117')
    p.add_argument('--models', nargs='+', default=['deepseek-flash', 'gpt-4o'])
    a = p.parse_args()
    rows = baseline_rows()
    present = [m for m in a.models if (a.runs / m / 'episodes').is_dir()]
    for m in present:
        rows += model_rows(a.runs / m)
    methods = ['Random', 'Nearest', 'Information Gain', 'gemini-3.5-flash-lite'] + present
    lines = ['# Bowl48 llm_guided: model comparison', '',
             'Only the hosted model differs from the archived bowl48 Gemini run (same manifests, '
             'seeds, noise, prompt v1, 30 s LLM deadline). Times are simulated seconds; means over '
             '6 setups per target.', '', table(rows, methods), '']
    for m in present:
        lines.append('- ' + paired(rows, m, 'gemini-3.5-flash-lite'))
    for m in present:
        lines.append('- ' + paired(rows, m, 'Information Gain'))
    if len(present) == 2:
        lines.append('- ' + paired(rows, present[0], present[1]))
    (a.runs / 'comparison.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    with (a.runs / 'episodes.csv').open('w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print('\n'.join(lines))


if __name__ == '__main__':
    main()
