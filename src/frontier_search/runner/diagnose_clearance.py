"""Offline mapping evidence diagnostics; reads private buffers, never changes them."""

import argparse
from collections import Counter
import json
from math import ceil, floor, hypot
from pathlib import Path

from ..contracts import Pose
from ..fixtures.sequence import load_sequence, mapper_init
from ..navigation.costmap import Costmap
from ..navigation.geometry import cells_overlapping_footprint, cell_center, world_to_cell
from ..navigation.planner import goal_is_valid, start_is_usable
from ..perception.mapper import RGBDMapper


def missing_bins(mask, low, high, height):
    extent = (high-low)/height
    nearest = round(extent)
    if nearest > 0 and abs(extent-nearest) < 1e-12:
        extent = float(nearest)
    return [i for i in range(ceil(extent)) if not mask & (1 << i)]


def nearest_visible_floor(observation, floor_z):
    """Bottom-edge ideal flat-floor intersections, ignoring all occlusion."""
    k = observation.camera_intrinsics
    matrix = observation.camera_to_world.matrix
    distances = []
    for u in range(k.width):
        ray = ((u-k.cx)/k.fx, (k.height-1-k.cy)/k.fy, 1.0)
        direction = [sum(matrix[i][j]*ray[j] for j in range(3)) for i in range(3)]
        if direction[2] < 0 and matrix[2][3] > floor_z:
            t = (floor_z-matrix[2][3])/direction[2]
            distances.append(t*hypot(direction[0], direction[1]))
    return min(distances) if distances else None


def diagnose(dataset, output):
    sequence = load_sequence(dataset)
    manifest = sequence.manifest
    mapper = RGBDMapper()
    mapper.reset(mapper_init(manifest, sequence.initial_clearance_cells))
    records = []
    for frame in sequence.observations:
        state = mapper.update(frame)
        grid = state.occupancy_map
        private = mapper._state  # Explicit developer diagnostic; not a policy feature.
        low, high = private.collision_z
        step = manifest.mapping.clearance_bin_height_m
        costmap = Costmap.from_robot(grid, manifest.robot)

        def evidence(cells):
            result = []
            for r, c in sorted(set(cells)):
                if not (0 <= r < grid.height_cells and 0 <= c < grid.width_cells):
                    result.append(dict(row=r, column=c, occupancy='outside_map'))
                elif grid.data[r][c] != 0:
                    result.append(dict(row=r, column=c, occupancy=grid.data[r][c],
                        floor_seen=private.floor_seen[r][c],
                        missing_height_bins=missing_bins(private.free_masks[r][c], low, high, step)))
            return result

        def exact_cells(pose):
            return cells_overlapping_footprint(grid, manifest.robot.footprint_base, pose,
                                               manifest.robot.clearance_margin_m)

        def lattice_cells(pose):
            cell = world_to_cell(grid, pose.x_m, pose.y_m)
            if cell is None:
                return ((floor((pose.y_m-grid.origin_y_m)/grid.resolution_m),
                         floor((pose.x_m-grid.origin_x_m)/grid.resolution_m)),)
            centre = cell_center(grid, cell.row, cell.column)
            return cells_overlapping_footprint(grid, costmap._disc,
                Pose(centre.x_m, centre.y_m, pose.z_m, 0.0), manifest.robot.clearance_margin_m)

        goals = []
        for frontier in state.frontiers:
            pose = frontier.approach_pose
            goals.append(dict(frontier_id=frontier.frontier_id,
                goal_valid=goal_is_valid(costmap, pose), exact_pose_valid=costmap.pose_valid(pose),
                exact_blocked_cells=evidence(exact_cells(pose)),
                lattice_disc_blocked_cells=evidence(lattice_cells(pose))))
        unknown_floor = [(r, c) for r in range(grid.height_cells) for c in range(grid.width_cells)
                         if grid.data[r][c] == -1 and private.floor_seen[r][c]]
        histogram = Counter(i for r, c in unknown_floor
                            for i in missing_bins(private.free_masks[r][c], low, high, step))
        count = len(missing_bins(0, low, high, step))
        records.append(dict(sequence=frame.sequence,
            nearest_bottom_edge_floor_from_camera_xy_m=nearest_visible_floor(frame, manifest.mapping.floor_height_m),
            known_cells=sum(v != -1 for row in grid.data for v in row),
            free_cells=sum(v == 0 for row in grid.data for v in row),
            exact_start_valid=costmap.pose_valid(state.robot_pose),
            planner_start_usable=start_is_usable(costmap, state.robot_pose),
            start_blocked_cells=evidence(exact_cells(state.robot_pose)),
            start_lattice_blocked_cells=evidence(lattice_cells(state.robot_pose)),
            floor_seen_unknown_cells=len(unknown_floor),
            missing_height_histogram=[dict(bin=i, z_min_m=low+i*step,
                z_max_m=min(high, low+(i+1)*step), cells=histogram[i]) for i in range(count)],
            goals=goals))
    report = dict(dataset=str(dataset), scope='Offline private evidence diagnostic; no geometry or safety changes',
                  input_sha256=sequence.digest, frames=records)
    output = Path(output)
    with output.open('x', encoding='utf-8') as stream:
        json.dump(report, stream, indent=2)
    for row in records:
        print('frame %d: free=%d, exact start=%s, planner start=%s, valid goals=%d/%d, floor-seen unknown=%d' % (
            row['sequence'], row['free_cells'], row['exact_start_valid'], row['planner_start_usable'],
            sum(g['goal_valid'] for g in row['goals']), len(row['goals']), row['floor_seen_unknown_cells']))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('choose a new diagnostic output file')
    diagnose(args.dataset, args.output)


if __name__ == '__main__':
    main()
