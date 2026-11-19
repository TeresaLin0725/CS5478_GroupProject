"""Code every bowl48 LLM decision: what the CoT/reason talks about and how the pick relates
to candidate metrics and (evaluator-only) bowl position. Writes cot-calls.csv and
cot-analysis.json into --runs."""
import argparse
import csv
import json
import math
import re
from collections import Counter
from pathlib import Path
from statistics import mean, median

ROOT = Path(__file__).resolve().parents[1]
COCO = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light',
        'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
        'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
        'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
        'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
        'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
        'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard',
        'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase',
        'scissors', 'teddy bear', 'hair drier', 'toothbrush']
ROOMS = {'kitchen': r'kitchen', 'dining': r'dining', 'bathroom': r'bath(room)?|restroom|toilet room',
         'bedroom': r'bedroom', 'living': r'living', 'hallway': r'hall(way)?|corridor'}
FACTORS = {'semantic_prior': r'likely|semantic|typical|usually|common',
           'info_gain': r'info(rmation)?[ _]?gain|gain', 'path_cost': r'path|cost|closer|distance|nearest',
           'explored_fraction': r'explored', 'feedback': r'feedback|fail|attempt|previous|retry|stuck',
           'exclusion': r'exclu|no_path'}
HESITATION = r'\b(hmm|wait|but|however|maybe|perhaps|alternatively|actually)\b'
TARGETS = {'kitchen': (-1.3502419985830785, 2.9511864975094797),
           'bathroom': (-3.400241998583078, 3.0011864975094804)}


def ranks(cands, chosen, key, reverse):
    order = sorted(cands, key=lambda c: key(c), reverse=reverse)
    vals = [key(c) for c in order]
    v = key(chosen)
    return 1 + sum(x > v + 1e-9 if reverse else x < v - 1e-9 for x in vals)


def code_call(r, target):
    inp = r['input'] or {}
    cands = [c for c in inp.get('candidates', []) if c['frontier_id'] in inp.get('eligible_frontier_ids', [])]
    by_id = {c['frontier_id']: c for c in cands}
    chosen = by_id.get(r['selected'])
    regions = {g['region_id']: g for g in inp.get('regions', [])}
    observed = {o['class_name'] for o in inp.get('observed_objects', [])} | {c for g in inp.get('regions', []) for c in (g.get('observed_object_classes') or [])}
    row = dict(model=r['model'], job_id=r['job_id'], target=r['job_id'].split('-')[0], call=r['call_index'],
               fallback=r['fallback_reason'], n_eligible=len(cands), selected=r['selected'],
               latency_s=round(r['latency_s'] or 0, 2))
    if chosen and len(cands) > 0:
        tx, ty = target
        dist = lambda c: math.hypot(c['approach_pose']['x_m'] - tx, c['approach_pose']['y_m'] - ty)
        row.update(rank_gain_per_path=ranks(cands, chosen, lambda c: c['gain_per_path_m'], True),
                   rank_info_gain=ranks(cands, chosen, lambda c: c['information_gain_m2'], True),
                   rank_path_cost=ranks(cands, chosen, lambda c: c['path_cost_m'], False),
                   rank_to_bowl=ranks(cands, chosen, dist, False),
                   chosen_to_bowl_m=round(dist(chosen), 2), best_to_bowl_m=round(min(map(dist, cands)), 2),
                   chosen_region=chosen.get('region_id'),
                   chosen_region_explored=round(regions.get(chosen.get('region_id'), {}).get('explored_fraction') or 0, 3),
                   chosen_region_objects='|'.join(regions.get(chosen.get('region_id'), {}).get('observed_object_classes') or []),
                   least_explored_region=min(regions.values(), key=lambda g: g.get('explored_fraction') or 1)['region_id'] if regions else None)
    for source in ('cot', 'reason'):
        text = (r['reasoning_content'] if source == 'cot' else r['reason']) or ''
        low = text.lower()
        row[source + '_chars'] = len(text)
        if not text:
            continue
        for name, pat in ROOMS.items():
            row['%s_room_%s' % (source, name)] = len(re.findall(pat, low))
        for name, pat in FACTORS.items():
            row['%s_factor_%s' % (source, name)] = len(re.findall(pat, low))
        mentioned = {c for c in COCO if re.search(r'\b' + re.escape(c) + r's?\b', low)} - {'bowl'}
        row[source + '_unobserved_objects'] = '|'.join(sorted(mentioned - observed))
        if source == 'cot':
            row['cot_hesitation'] = len(re.findall(HESITATION, low))
            ids = re.findall(r'(F\d{6}|probe-[\w-]+)', text)
            picks = re.findall(r'(?:choose|chose|select|selected|pick|go with|frontier_id"?\s*[:=]?\s*"?)\s*"?'
                               r'(F\d{6}|probe-[\w-]+)', text, re.IGNORECASE)
            row['cot_last_id'] = picks[-1] if picks else None
            row['cot_last_id_matches'] = (picks[-1] == r['selected']) if picks and r['selected'] else None
            # Did the stated choice change during the CoT (A -> B -> ...)?
            changes = [p for i, p in enumerate(picks) if i == 0 or p != picks[i - 1]]
            row['cot_choice_switches'] = max(0, len(changes) - 1)
            row['cot_ids_considered'] = len(set(ids))
    return row


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--runs', type=Path, default=ROOT / 'runs/bowl-llm-models-20261117')
    a = p.parse_args()
    rows = []
    for m in ('deepseek-flash', 'gpt-4o'):
        for line in (a.runs / m / 'llm-cot.jsonl').read_text(encoding='utf-8').splitlines():
            r = json.loads(line)
            rows.append(code_call(r, TARGETS[r['job_id'].split('-')[0]]))
    keys = sorted({k for r in rows for k in r}, key=lambda k: (k not in rows[0], k))
    with (a.runs / 'cot-calls.csv').open('w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    summary = {}
    for m in ('deepseek-flash', 'gpt-4o'):
        rs = [r for r in rows if r['model'] == m and 'rank_to_bowl' in r and not r['fallback']]
        multi = [r for r in rs if r['n_eligible'] > 1]
        s = dict(decisions=len(rs), multi_choice=len(multi))
        for key in ('rank_gain_per_path', 'rank_info_gain', 'rank_path_cost', 'rank_to_bowl'):
            s[key + '_top1'] = sum(r[key] == 1 for r in multi)
        s['to_bowl_top1_by_target'] = {t: '%d/%d' % (sum(r['rank_to_bowl'] == 1 for r in multi if r['target'] == t),
                                                     sum(r['target'] == t for r in multi)) for t in TARGETS}
        s['chose_least_explored_region'] = sum(r['chosen_region'] == r['least_explored_region'] for r in multi)
        s['mean_n_eligible'] = round(mean(r['n_eligible'] for r in multi), 1)
        for src in ('cot', 'reason'):
            withtext = [r for r in rows if r['model'] == m and r.get(src + '_chars')]
            if not withtext:
                continue
            s[src] = dict(n=len(withtext), median_chars=median(r[src + '_chars'] for r in withtext),
                          rooms_calls={k: sum(bool(r.get('%s_room_%s' % (src, k))) for r in withtext) for k in ROOMS},
                          factor_calls={k: sum(bool(r.get('%s_factor_%s' % (src, k))) for r in withtext) for k in FACTORS},
                          unobserved_object_mentions=Counter(o for r in withtext for o in (r.get(src + '_unobserved_objects') or '').split('|') if o))
            if src == 'cot':
                s[src].update(median_hesitation=median(r['cot_hesitation'] for r in withtext),
                              last_id_matches='%d/%d' % (sum(bool(r['cot_last_id_matches']) for r in withtext), len(withtext)),
                              median_ids_considered=median(r['cot_ids_considered'] for r in withtext),
                              calls_with_choice_switch=sum(bool(r.get('cot_choice_switches')) for r in withtext))
        summary[m] = s
    (a.runs / 'cot-analysis.json').write_text(json.dumps(summary, indent=2, ensure_ascii=False), encoding='utf-8')
    print(json.dumps(summary, indent=1, ensure_ascii=False))


if __name__ == '__main__':
    main()
