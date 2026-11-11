"""Compare recorded start-B bathroom routes with schematic added furniture.

The source floorplan is the original Rs scan. Furniture markers show initial
URDF footprints from the run's scene layout, not newly rendered camera pixels.
"""
import argparse
import json
from math import cos, sin
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from frontier_search.simulation.indoor_challenges import profile_props  # noqa: E402


def load(path):
    return json.loads(path.read_text(encoding='utf-8'))


def rotated_box(x, y, width, height, yaw):
    corners = [(-width / 2, -height / 2), (width / 2, -height / 2),
               (width / 2, height / 2), (-width / 2, height / 2)]
    return [(x + dx * cos(yaw) - dy * sin(yaw),
             y + dx * sin(yaw) + dy * cos(yaw)) for dx, dy in corners]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-root', type=Path, required=True)
    args = parser.parse_args()
    root = args.run_root.resolve()
    protocol = load(root / 'protocol.json')
    layout = protocol['scene_layout']

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon, Patch
    from matplotlib.lines import Line2D

    fig, axes = plt.subplots(1, 3, figsize=(15, 8.7), dpi=180, facecolor='white')
    fig.subplots_adjust(left=.018, right=.982, bottom=.14, top=.88, wspace=.04)
    background = plt.imread(ROOT / 'data/igibson-data/g_dataset/Rs/floorplan.png')
    records = []
    for ax, profile in zip(axes, ('apartment', 'clutter', 'dynamic')):
        job_id = f'bathroom-{profile}-startB-llm_guided'
        folder = root / 'episodes' / job_id
        summary = load(folder / 'summary.json')
        manifest = load(root / 'manifests' / (job_id + '.json'))
        start = manifest['start_pose']
        target = protocol['targets_evaluator_only']['bathroom']['position_world']
        positions = [(start['x_m'], start['y_m'])]
        with (folder / 'events.jsonl').open(encoding='utf-8') as stream:
            for line in stream:
                event = json.loads(line)
                if event['event'] == 'motion_sample':
                    pose = event['pose']
                    positions.append((pose['x_m'], pose['y_m']))

        ax.imshow(background, origin='lower', extent=(-5, 5, -5, 5),
                  interpolation='bilinear')
        furniture = []
        for index, (kind, position, yaw) in enumerate(profile_props(profile, layout), 1):
            if kind == 'cart':
                cart_route = next(event for event in summary['scene_events']
                                  if event['event'] == 'cart_route_verified')
                position = cart_route['start']
            x, y, _ = position
            if kind == 'ottoman':
                patch = Circle((x, y), .255, facecolor='#ff8a22',
                               edgecolor='#a33900', linewidth=2.2, alpha=.72)
            else:
                size = (.44, .44) if kind == 'chair' else (.48, .40)
                patch = Polygon(rotated_box(x, y, *size, yaw), closed=True,
                                facecolor='#ff8a22', edgecolor='#a33900',
                                linewidth=2.2, alpha=.72)
            ax.add_patch(patch)
            ax.annotate(str(index), (x, y), ha='center', va='center',
                        fontsize=10, weight='bold', color='#4b1e02')
            furniture.append({'number': index, 'kind': kind,
                              'initial_position_xy_m': [x, y], 'yaw_rad': yaw})

        ax.plot(*zip(*positions), color='#087da5', linewidth=1.4, zorder=4)
        ax.scatter([start['x_m']], [start['y_m']], s=60, c='#2a9f75',
                   edgecolors='white', linewidths=1.4, zorder=6)
        ax.scatter([target[0]], [target[1]], s=180, marker='*', c='#d85b42',
                   edgecolors='white', linewidths=1.3, zorder=6)
        ax.set_xlim(-4.4, 2.2)
        ax.set_ylim(-4.4, 3.8)
        ax.set_aspect('equal')
        ax.axis('off')
        ax.set_title(f'{profile.title()}  |  {summary["simulated_time_s"]:.0f} s  |  '
                     f'{summary["actual_distance_m"]:.2f} m', fontsize=13, pad=13)
        if not furniture:
            ax.text(.5, .04, 'No added furniture', transform=ax.transAxes,
                    ha='center', fontsize=11, color='#66320e', weight='bold',
                    bbox={'facecolor': 'white', 'alpha': .9, 'edgecolor': 'none'})
        records.append({'job_id': job_id, 'furniture': furniture,
                        'motion_samples': len(positions) - 1})

    fig.suptitle('Bathroom bowl, start B: recorded LLM routes and added furniture',
                 fontsize=18, weight='bold', y=.97)
    fig.legend(handles=[Line2D([], [], color='#087da5', lw=2, label='Recorded route'),
                        Line2D([], [], marker='o', ls='', color='#2a9f75', label='Start'),
                        Line2D([], [], marker='*', ls='', color='#d85b42', markersize=12,
                               label='Bowl'),
                        Patch(facecolor='#ff8a22', edgecolor='#a33900',
                              label='Added furniture at initial position')],
               loc='lower center', bbox_to_anchor=(.5, .082), ncol=4,
               frameon=False, fontsize=10)
    fig.text(.5, .055,
             'Clutter: 1 ottoman, 2-3 chairs.  Dynamic: 1 ottoman, 2 chair, 3 cart.',
             ha='center', fontsize=9.5, color='#66320e')
    fig.text(.5, .025,
             'Furniture is a schematic footprint on the original Rs scan; the scene itself was not re-rendered.',
             ha='center', fontsize=9, color='#59636d')
    output = root / 'furniture-comparison-bathroom-startB-llm.png'
    fig.savefig(output, dpi=180, facecolor='white')
    plt.close(fig)
    (root / 'furniture-comparison-bathroom-startB-llm.json').write_text(
        json.dumps({'layout': layout, 'background': 'original Rs scan floorplan',
                    'marker_semantics': 'schematic initial URDF footprints',
                    'episodes': records}, indent=2), encoding='utf-8')
    print(output)


if __name__ == '__main__':
    main()
