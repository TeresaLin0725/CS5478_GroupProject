"""Planar unknown-area prediction from the observed map and fixed camera sweep."""

from math import atan2, ceil, cos, floor, hypot, pi, sin
from typing import Iterator, Tuple

from ..config import MappingConfig, ObservationConfig
from ..contracts import OccupancyGrid, Pose, RGBDObservation


def camera_yaw_offset(observation: RGBDObservation) -> float:
    """Horizontal optical-axis bearing relative to the capture-time robot yaw."""
    matrix = observation.camera_to_world.matrix
    angle = atan2(matrix[1][2], matrix[0][2]) - observation.robot_pose.yaw_rad
    return (angle + pi) % (2 * pi) - pi


def _ray_cells(x: float, y: float, angle: float, distance: float,
               grid: OccupancyGrid) -> Iterator[Tuple[int, int]]:
    """Grid traversal in world distance; include corner-touching side cells.

    An occupied side cell at a diagonal crossing blocks visibility, preventing
    rays from looking through a zero-width gap between two touching obstacles.
    """
    res = grid.resolution_m
    c, r = floor((x - grid.origin_x_m) / res), floor((y - grid.origin_y_m) / res)
    dx, dy = cos(angle), sin(angle)
    sx, sy = (1 if dx > 0 else -1), (1 if dy > 0 else -1)
    tx = ((grid.origin_x_m + (c + (dx > 0)) * res - x) / dx
          if abs(dx) > 1e-12 else float('inf'))
    ty = ((grid.origin_y_m + (r + (dy > 0)) * res - y) / dy
          if abs(dy) > 1e-12 else float('inf'))
    stepx = res / abs(dx) if abs(dx) > 1e-12 else float('inf')
    stepy = res / abs(dy) if abs(dy) > 1e-12 else float('inf')
    while 0 <= r < grid.height_cells and 0 <= c < grid.width_cells:
        yield r, c
        if min(tx, ty) > distance:
            break
        if abs(tx - ty) <= 1e-10:
            # Side cells are yielded before the diagonal destination. The
            # caller stops on any occupied or out-of-bounds cell.
            yield r, c + sx
            yield r + sy, c
            r, c = r + sy, c + sx
            tx, ty = tx + stepx, ty + stepy
        elif tx < ty:
            c, tx = c + sx, tx + stepx
        else:
            r, ty = r + sy, ty + stepy


def estimate_gain(grid: OccupancyGrid, approach: Pose, observation: RGBDObservation,
                  mapping: MappingConfig, routine: ObservationConfig) -> float:
    """Count each predicted visible unknown cell once over the arrival sweep.

    Uses a radial planar range (capped by axial depth_max), the calibrated
    horizontal FOV, capture-derived camera mount translation/yaw, and shortest
    turns through the configured arrival offsets. Known occupied cells occlude;
    unknown cells do not. Vertical visibility and hidden obstacles are not
    predicted. A vertically pointing optical axis yields zero planar gain.
    """
    matrix = observation.camera_to_world.matrix
    if hypot(matrix[0][2], matrix[1][2]) < 1e-9:
        return 0.0
    intrinsics = observation.camera_intrinsics
    left = -atan2(intrinsics.width - 0.5 - intrinsics.cx, intrinsics.fx)
    right = -atan2(-0.5 - intrinsics.cx, intrinsics.fx)
    offset = camera_yaw_offset(observation)
    mount_x = matrix[0][3] - observation.robot_pose.x_m
    mount_y = matrix[1][3] - observation.robot_pose.y_m
    radius = min(mapping.gain_range_m, mapping.depth_max_m)
    visible = set()
    # Sample robot headings along the motion, so camera mount translation also
    # follows the sweep. Sampling spacing is tied to the configured ray step.
    headings = {0.0}
    previous = 0.0
    for target in routine.arrival_headings_rad:
        turn = (target - previous + pi) % (2 * pi) - pi
        count = max(1, ceil(abs(turn) / mapping.gain_ray_step_rad))
        headings.update(previous + turn * i / count for i in range(count + 1))
        previous += turn
    ray_count = max(1, ceil((right - left) / mapping.gain_ray_step_rad))
    for heading in sorted(headings):
        yaw = approach.yaw_rad + heading
        rotation = (yaw - observation.robot_pose.yaw_rad + pi) % (2 * pi) - pi
        x = approach.x_m + cos(rotation) * mount_x - sin(rotation) * mount_y
        y = approach.y_m + sin(rotation) * mount_x + cos(rotation) * mount_y
        bearing = yaw + offset
        for i in range(ray_count + 1):
            angle = bearing + left + (right - left) * i / ray_count
            for r, c in _ray_cells(x, y, angle, radius, grid):
                if not (0 <= r < grid.height_cells and 0 <= c < grid.width_cells):
                    break
                value = grid.data[r][c]
                if value == 100:
                    break
                if value == -1:
                    dx = grid.origin_x_m + (c + 0.5) * grid.resolution_m - x
                    dy = grid.origin_y_m + (r + 0.5) * grid.resolution_m - y
                    relative = (atan2(dy, dx) - bearing + pi) % (2 * pi) - pi
                    if hypot(dx, dy) <= radius and left <= relative <= right:
                        visible.add((r, c))
    return len(visible) * grid.resolution_m ** 2
