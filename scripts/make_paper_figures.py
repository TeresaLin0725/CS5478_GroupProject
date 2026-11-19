"""Paper figures for the bowl48 model comparison (DeepSeek / GPT-4o / Gemini).

Reads runs/bowl-llm-models-20261117/episodes.csv and the per-episode
trajectory.png files written by export_run_trajectories.py. Writes PDFs and PNGs
to report/paper-figures/.
"""
import csv
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
RUN = ROOT / 'runs/bowl-llm-models-20261117'
OUT = ROOT / 'report/paper-figures'
OUT.mkdir(parents=True, exist_ok=True)

plt.rcParams.update({'font.family': 'serif', 'font.size': 8, 'axes.spines.top': False,
                     'axes.spines.right': False, 'axes.edgecolor': '#8a887f',
                     'axes.labelcolor': '#333', 'xtick.color': '#555', 'ytick.color': '#555'})

METHODS = [('Random', 'Random', '#c9c7be'), ('Nearest', 'Nearest', '#8a887f'),
           ('Information Gain', 'IG', '#2a78d6'), ('gemini-3.5-flash-lite', 'Gemini', '#eb6834'),
           ('deepseek-flash', 'DeepSeek', '#1baf7a'), ('gpt-4o', 'GPT-4o', '#eda100')]


def save(fig, name):
    for ext in ('pdf', 'png'):
        fig.savefig(OUT / (name + '.' + ext), dpi=300, bbox_inches='tight', pad_inches=0.02)
    plt.close(fig)


def efficiency():
    rows = list(csv.DictReader((RUN / 'episodes.csv').open(encoding='utf-8')))
    acc = defaultdict(list)
    for r in rows:
        acc[(r['method'], r['target'], 'sim_s')].append(float(r['sim_s']))
        acc[(r['method'], r['target'], 'path_m')].append(float(r['path_m']))
    mean = {k: sum(v) / len(v) for k, v in acc.items()}
    fig, axes = plt.subplots(1, 2, figsize=(7.0, 2.1))
    width = 0.13
    for ax, (metric, label) in zip(axes, [('sim_s', 'Mean simulated time (s)'), ('path_m', 'Mean path length (m)')]):
        for i, (key, short, color) in enumerate(METHODS):
            xs = [g + (i - 2.5) * (width + 0.01) for g in range(2)]
            ys = [mean[(key, t, metric)] for t in ('kitchen', 'bathroom')]
            ax.bar(xs, ys, width, color=color, label=short, edgecolor='white', linewidth=0.5)
            for x, y in zip(xs, ys):
                ax.text(x, y, ('%.0f' if metric == 'sim_s' else '%.1f') % y, ha='center', va='bottom',
                        fontsize=5.5, color='#333')
        ax.set_xticks([0, 1])
        ax.set_xticklabels(['Kitchen bowl\n(typical location)', 'Bathroom bowl\n(atypical location)'])
        ax.set_ylabel(label)
        ax.yaxis.grid(True, color='#e6e5e0', linewidth=0.5)
        ax.set_axisbelow(True)
    axes[0].legend(ncol=6, loc='lower left', bbox_to_anchor=(0.0, 1.02), frameon=False, fontsize=7,
                   handlelength=1.0, columnspacing=1.0)
    save(fig, 'bowl-efficiency')


def crop(img):
    bg = Image.new(img.mode, img.size, (255, 255, 255, 255) if img.mode == 'RGBA' else (255, 255, 255))
    from PIL import ImageChops
    box = ImageChops.difference(img, bg).getbbox()
    return img.crop(box) if box else img


def trajectories():
    sim = {(r['method'], r['pair_id']): r for r in csv.DictReader((RUN / 'episodes.csv').open(encoding='utf-8'))}
    cases = [('kitchen-apartment-startA', 'Kitchen bowl, start A'), ('bathroom-apartment-startB', 'Bathroom bowl, start B')]
    models = [('deepseek-flash', 'DeepSeek'), ('gpt-4o', 'GPT-4o')]
    fig, axes = plt.subplots(1, 4, figsize=(7.0, 2.55))
    i = 0
    for pair, title in cases:
        for key, short in models:
            img = crop(Image.open(RUN / key / 'episodes' / (pair + '-llm_guided') / 'trajectory.png').convert('RGB'))
            r = sim[(key, pair)]
            axes[i].imshow(img)
            axes[i].axis('off')
            axes[i].set_title('%s\n%s: %.0f s, %.1f m' % (title, short, float(r['sim_s']), float(r['path_m'])),
                              fontsize=7)
            i += 1
    fig.subplots_adjust(wspace=0.04)
    save(fig, 'trajectory-models')


def decisions():
    # Percentages of multi-candidate decisions (DeepSeek n=35, GPT-4o n=36) from cot-analysis.json.
    labels = ['Picked top gain/path', 'Picked candidate\nclosest to bowl', 'Reason cites\nroom semantics',
              'Reason cites\nexecution feedback']
    ds = [19 / 35, 2 / 35, 26 / 35, 22 / 35]
    gpt = [28 / 36, 9 / 36, 5 / 36, 3 / 36]
    fig, ax = plt.subplots(figsize=(2.9, 2.0))
    ys = range(len(labels))
    h = 0.36
    ax.barh([y - h / 2 - 0.01 for y in ys], [v * 100 for v in ds], h, color='#1baf7a', label='DeepSeek')
    ax.barh([y + h / 2 + 0.01 for y in ys], [v * 100 for v in gpt], h, color='#eda100', label='GPT-4o')
    for y, a, b in zip(ys, ds, gpt):
        ax.text(a * 100 + 1, y - h / 2 - 0.01, '%.0f%%' % (a * 100), va='center', fontsize=6)
        ax.text(b * 100 + 1, y + h / 2 + 0.01, '%.0f%%' % (b * 100), va='center', fontsize=6)
    ax.set_yticks(list(ys))
    ax.set_yticklabels(labels, fontsize=7)
    ax.invert_yaxis()
    ax.set_xlim(0, 100)
    ax.set_xlabel('Share of LLM decisions (%)')
    ax.xaxis.grid(True, color='#e6e5e0', linewidth=0.5)
    ax.set_axisbelow(True)
    ax.legend(frameon=False, fontsize=7, ncol=2, loc='lower left', bbox_to_anchor=(0, 1.0))
    save(fig, 'decision-profile')


ORIG = ROOT / 'report/bowl-48-noise-20261110-02'
POLICIES = [('random', 'Random'), ('nearest', 'Nearest'), ('information_gain', 'IG'), ('llm_guided', 'LLM')]


def original_trajectories(target):
    """Original 48-episode bowl suite, apartment variant: start A and B rows x four policies."""
    rows = {r['job_id']: r for r in csv.DictReader((ORIG / 'episodes.csv').open(encoding='utf-8-sig'))}
    fig, axes = plt.subplots(2, 4, figsize=(7.0, 4.3))
    for i, start in enumerate('AB'):
        for j, (pol, short) in enumerate(POLICIES):
            job = '%s-apartment-start%s-%s' % (target, start, pol)
            ax = axes[i][j]
            ax.imshow(crop(Image.open(ORIG / 'episodes' / job / 'trajectory-scene.webp').convert('RGB')))
            ax.set_xticks([])
            ax.set_yticks([])
            for s in ax.spines.values():
                s.set_visible(False)
            r = rows[job]
            status = '' if r['termination'] == 'success' else ' (failed)'
            ax.set_title('%s%s\n%.0f s, %.1f m' % (short, status, float(r['simulated_time_s']), float(r['actual_distance_m'])),
                         fontsize=13)
            if j == 0:
                ax.set_ylabel('Start %s' % start, fontsize=15)
    fig.subplots_adjust(wspace=0.04, hspace=0.32)
    save(fig, 'bowl-original-%s' % target)


def model_behaviour():
    """One row: bathroom/start-B trajectories for the three LLMs + decision shares."""
    sim = {(r['method'], r['pair_id']): r for r in csv.DictReader((RUN / 'episodes.csv').open(encoding='utf-8'))}
    orig = {r['job_id']: r for r in csv.DictReader((ORIG / 'episodes.csv').open(encoding='utf-8-sig'))}
    pair = 'bathroom-apartment-startB'
    fig = plt.figure(figsize=(7.0, 2.35))
    gs = fig.add_gridspec(1, 4, width_ratios=[1, 1, 1, 2.3], wspace=0.08)
    # Gemini: per-step poses of the final suite are not stored locally; use the recorded
    # original-suite route of the same setup (same manifest, 11.33 m in both suites).
    g = orig[pair + '-llm_guided']
    panels = [('Gemini', Image.open(ORIG / 'episodes' / (pair + '-llm_guided') / 'trajectory-scene.webp'),
               float(g['simulated_time_s']), float(g['actual_distance_m']))]
    for key, short in [('deepseek-flash', 'DeepSeek'), ('gpt-4o', 'GPT-4o')]:
        r = sim[(key, pair)]
        panels.append((short, Image.open(RUN / key / 'episodes' / (pair + '-llm_guided') / 'trajectory.png'),
                       float(r['sim_s']), float(r['path_m'])))
    for k, (short, img, t, d) in enumerate(panels):
        ax = fig.add_subplot(gs[0, k])
        ax.imshow(crop(img.convert('RGB')))
        ax.axis('off')
        ax.set_title('(%s) %s\n%.0f s, %.1f m' % ('abc'[k], short, t, d), fontsize=7)
    ax = fig.add_subplot(gs[0, 3])
    labels = ['Picked top gain/path', 'Picked candidate\nclosest to bowl', 'Reason cites\nroom semantics',
              'Reason cites\nexecution feedback']
    ds = [19 / 35, 2 / 35, 26 / 35, 22 / 35]
    gpt = [28 / 36, 9 / 36, 5 / 36, 3 / 36]
    ys = range(len(labels))
    h = 0.36
    ax.barh([y - h / 2 - 0.01 for y in ys], [v * 100 for v in ds], h, color='#1baf7a', label='DeepSeek (35 calls)')
    ax.barh([y + h / 2 + 0.01 for y in ys], [v * 100 for v in gpt], h, color='#eda100', label='GPT-4o (36 calls)')
    for y, a, b in zip(ys, ds, gpt):
        ax.text(a * 100 + 1, y - h / 2 - 0.01, '%.0f%%' % (a * 100), va='center', fontsize=6)
        ax.text(b * 100 + 1, y + h / 2 + 0.01, '%.0f%%' % (b * 100), va='center', fontsize=6)
    ax.set_yticks(list(ys))
    ax.set_yticklabels(labels, fontsize=6.5)
    ax.yaxis.tick_right()
    ax.invert_yaxis()
    ax.set_xlim(0, 100)
    ax.set_xlabel('(d) Share of decisions (%)', fontsize=7)
    ax.spines['left'].set_visible(False)
    ax.spines['right'].set_visible(True)
    ax.xaxis.grid(True, color='#e6e5e0', linewidth=0.5)
    ax.set_axisbelow(True)
    ax.legend(frameon=False, fontsize=6.5, ncol=2, loc='lower left', bbox_to_anchor=(0, 1.0))
    save(fig, 'model-behaviour')


def model_routes():
    """Bathroom bowl, start B: routes of the three LLMs (no bar chart)."""
    sim = {(r['method'], r['pair_id']): r for r in csv.DictReader((RUN / 'episodes.csv').open(encoding='utf-8'))}
    orig = {r['job_id']: r for r in csv.DictReader((ORIG / 'episodes.csv').open(encoding='utf-8-sig'))}
    pair = 'bathroom-apartment-startB'
    g = orig[pair + '-llm_guided']
    panels = [('Gemini', Image.open(ORIG / 'episodes' / (pair + '-llm_guided') / 'trajectory-scene.webp'),
               float(g['simulated_time_s']), float(g['actual_distance_m']))]
    for key, short in [('deepseek-flash', 'DeepSeek'), ('gpt-4o', 'GPT-4o')]:
        r = sim[(key, pair)]
        panels.append((short, Image.open(RUN / key / 'episodes' / (pair + '-llm_guided') / 'trajectory.png'),
                       float(r['sim_s']), float(r['path_m'])))
    fig, axes = plt.subplots(1, 3, figsize=(5.2, 2.5))
    for k, (short, img, t, d) in enumerate(panels):
        axes[k].imshow(crop(img.convert('RGB')))
        axes[k].axis('off')
        axes[k].set_title('(%s) %s: %.0f s, %.1f m' % ('abc'[k], short, t, d), fontsize=7)
    fig.subplots_adjust(wspace=0.04)
    save(fig, 'model-routes')


if __name__ == '__main__':
    model_routes()
    efficiency()
    trajectories()
    decisions()
    model_behaviour()
    original_trajectories('kitchen')
    original_trajectories('bathroom')
    print('Wrote', OUT)
