"""Create labelled analytic fusion data at the integrated camera sampling density.

The mapping/robot settings are copied from the integrated manifest; the geometry
is an analytic floor/wall and removable box, not a native simulator acceptance.
"""

import argparse
from dataclasses import replace
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))

from frontier_search.contracts import Pose
from frontier_search.fixtures.analytic_rgbd import render_frame
from frontier_search.fixtures.sequence import footprint_cells, write_sequence
from frontier_search.runner.integrated import manifest_for


def make_fixture(output):
    output = Path(output)
    if output.exists():
        raise FileExistsError(str(output))
    manifest = replace(manifest_for('fusion-analytic-640x480-v1', 'nearest'),
                       source='synthetic', scene_id='analytic_floor_wall',
                       start_pose=Pose(0., 0., 0., 0.), region_priors=(),
                       enable_target_evaluation=False, initial_clearance_mode='footprint',
                       initial_clearance_radius_m=0.)
    box = (((1.1, -.5, 0.), (1.5, .5, 1.2)),)
    # A hit, one complete fresh frame, a repeated timestamp, another hit that
    # breaks the streak, then three fresh unobstructed views.
    cases = ((1., box), (2., ()), (2., ()), (3., box), (4., ()), (5., ()), (6., ()))
    frames = tuple(replace(render_frame(sequence=index + 1, episode_id=manifest.episode_id,
                                        width=640, height=480, pitch_down=0., boxes=boxes),
                           timestamp_s=timestamp)
                   for index, (timestamp, boxes) in enumerate(cases))
    write_sequence(output, manifest, frames, footprint_cells(manifest),
                   'Analytic floor at z=0 and wall x=4; removable box starts x=1.1; '
                   'initial footprint is clear. No native simulator execution.',
                   sources=tuple(dict(kind='analytic_fusion_fixture', width=640, height=480,
                                      box_present=bool(boxes), timestamp_s=timestamp)
                                 for timestamp, boxes in cases))
    return len(frames)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='New RGB-D package directory')
    args = parser.parse_args()
    count = make_fixture(args.output)
    print('Created %d fixed frames at %s' % (count, args.output))


if __name__ == '__main__':
    main()
