"""Bounded egress from unknown cells already intersecting the live footprint.

This never edits the map or permits entering new unknown/occupied space. It is
only an observation-recovery primitive, not relaxed global path planning.
"""
from dataclasses import dataclass
from math import ceil, cos, hypot, sin

from ..contracts import Pose
from .geometry import cells_overlapping_footprint, cell_value


@dataclass(frozen=True)
class PoseEgress:
    origin: Pose
    goal: Pose
    unknown_cells: frozenset
    direction: int


def _distance_to_cell(grid, pose, cell):
    r, c = cell
    x, y = grid.origin_x_m+c*grid.resolution_m, grid.origin_y_m+r*grid.resolution_m
    return hypot(max(x-pose.x_m, 0., pose.x_m-x-grid.resolution_m),
                 max(y-pose.y_m, 0., pose.y_m-y-grid.resolution_m))


def egress_path_valid(costmap, live, egress):
    grid = costmap.grid
    distance = hypot(egress.goal.x_m-live.x_m, egress.goal.y_m-live.y_m)
    if hypot(live.x_m-egress.origin.x_m, live.y_m-egress.origin.y_m) > .22:
        return False
    if any(_distance_to_cell(grid, live, cell)+.002 <
           _distance_to_cell(grid, egress.origin, cell) for cell in egress.unknown_cells):
        return False
    samples = max(1, ceil(distance/.01))
    for i in range(samples+1):
        t = i/samples
        pose = Pose(live.x_m+t*(egress.goal.x_m-live.x_m),
                    live.y_m+t*(egress.goal.y_m-live.y_m), live.z_m, live.yaw_rad)
        for cell in egress.unknown_cells:
            # Do not move deeper into the initially overlapped unknown cells.
            if _distance_to_cell(grid, pose, cell)+.002 < _distance_to_cell(grid, live, cell):
                return False
        for r, c in cells_overlapping_footprint(grid, costmap.footprint_base, pose):
            value = cell_value(grid, r, c)
            if value != 0 and not (value == -1 and (r,c) in egress.unknown_cells):
                return False
    return True


def choose_pose_egress(costmap, live):
    if costmap.start_pose_valid(live):
        return None
    grid = costmap.grid
    blocked = {(r,c) for r,c in cells_overlapping_footprint(grid, costmap.footprint_base, live)
               if cell_value(grid,r,c) != 0}
    if not blocked or any(cell_value(grid,r,c) != -1 for r,c in blocked):
        return None
    for distance in (.10, .15, .20, .05):
        for direction in (1, -1):
            goal = Pose(live.x_m+direction*distance*cos(live.yaw_rad),
                        live.y_m+direction*distance*sin(live.yaw_rad), live.z_m, live.yaw_rad)
            candidate = PoseEgress(live, goal, frozenset(blocked), direction)
            if (costmap.full_rotation_valid(goal) and
                    egress_path_valid(costmap, live, candidate)):
                return candidate
    return None
