"""Owner B: grid/world geometry and footprint collision primitives.

Robotics background
-------------------
* Occupancy grid: the world is discretised into cells with values -1 (unknown),
  0 (free) and 100 (occupied). Conventions follow Interface Freeze section 2:
  the origin is the lower-left corner of cell (row=0, column=0), rows increase
  with world y, columns with world x, and cell centres are
  ``origin + (index + 0.5) * resolution``.
* Rigid-body transform (SE(2)): a footprint polygon given in the robot-base
  frame is rotated by yaw and translated by (x, y) to obtain its world-frame
  outline at a given pose.
* Collision checking via the Separating Axis Theorem (SAT): two convex shapes
  are disjoint if and only if some axis separates their projections. For a
  convex polygon against an axis-aligned cell square, the only axes to test are
  the two grid axes and the polygon's edge normals.
* Conservative safety: unknown cells and out-of-bounds space are treated as
  blocked, because unknown is not free for collision checking.

Only the standard library is used, and annotations stay Python 3.8 compatible.
"""

from math import cos, floor, pi, sin
from typing import Iterator, List, Optional, Sequence, Tuple

from ..contracts import GridCell, OccupancyGrid, Point2, Pose

UNKNOWN = -1
FREE = 0
OCCUPIED = 100

_Vertex = Tuple[float, float]


# --- grid <-> world ---------------------------------------------------------

def in_bounds(grid: OccupancyGrid, row: int, column: int) -> bool:
    return 0 <= row < grid.height_cells and 0 <= column < grid.width_cells


def cell_center(grid: OccupancyGrid, row: int, column: int) -> Point2:
    """World position of a cell centre: origin + (index + 0.5) * resolution."""
    return Point2(grid.origin_x_m + (column + 0.5) * grid.resolution_m,
                  grid.origin_y_m + (row + 0.5) * grid.resolution_m)


def world_to_cell(grid: OccupancyGrid, x_m: float, y_m: float) -> Optional[GridCell]:
    """Cell containing a world point, or None if the point is outside the grid.

    Uses floor() so each cell owns the half-open square [lower, upper).
    """
    column = int(floor((x_m - grid.origin_x_m) / grid.resolution_m))
    row = int(floor((y_m - grid.origin_y_m) / grid.resolution_m))
    if not in_bounds(grid, row, column):
        return None
    return GridCell(row, column)


def cell_value(grid: OccupancyGrid, row: int, column: int) -> int:
    """Occupancy value; out-of-bounds counts as occupied (non-traversable)."""
    if not in_bounds(grid, row, column):
        return OCCUPIED
    return grid.data[row][column]


# --- footprints -------------------------------------------------------------

def circle_footprint(radius_m: float, sides: int = 16) -> Tuple[Point2, ...]:
    """Polygon approximating a circular robot of the given radius.

    A regular n-gon inscribed in a circle lies *inside* it and could miss
    obstacles. To stay conservative the vertices are placed on a circle of
    radius ``r / cos(pi / n)`` so the polygon fully contains the true disc.
    """
    if radius_m <= 0 or sides < 3:
        raise ValueError("radius must be positive and sides >= 3")
    circumradius = radius_m / cos(pi / sides)
    return tuple(Point2(circumradius * cos(2 * pi * k / sides),
                        circumradius * sin(2 * pi * k / sides))
                 for k in range(sides))


def footprint_in_world(footprint_base: Sequence[Point2], x_m: float, y_m: float,
                       yaw_rad: float) -> List[_Vertex]:
    """Transform a base-frame polygon to world coordinates (rotate, translate)."""
    c, s = cos(yaw_rad), sin(yaw_rad)
    return [(x_m + c * p.x_m - s * p.y_m, y_m + s * p.x_m + c * p.y_m)
            for p in footprint_base]


# --- collision (SAT) --------------------------------------------------------

def _project(vertices: Sequence[_Vertex], axis: _Vertex) -> Tuple[float, float]:
    values = [vx * axis[0] + vy * axis[1] for vx, vy in vertices]
    return min(values), max(values)


def polygon_intersects_square(polygon: Sequence[_Vertex], x_min: float, y_min: float,
                              x_max: float, y_max: float) -> bool:
    """True if a convex polygon overlaps (or touches) an axis-aligned square."""
    square = [(x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max)]
    axes = [(1.0, 0.0), (0.0, 1.0)]
    count = len(polygon)
    for i in range(count):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % count]
        axes.append((-(y2 - y1), x2 - x1))  # edge normal (need not be unit length)
    for axis in axes:
        p_min, p_max = _project(polygon, axis)
        s_min, s_max = _project(square, axis)
        if p_max < s_min or s_max < p_min:
            return False  # found a separating axis -> disjoint
    return True


def cells_overlapping_footprint(grid: OccupancyGrid, footprint_base: Sequence[Point2],
                                pose: Pose, margin_m: float = 0.0) -> Iterator[Tuple[int, int]]:
    """Yield (row, column) for every cell the inflated footprint overlaps.

    ``margin_m`` is the clearance margin. Each cell square is grown by the
    margin on every side, a slightly conservative (Chebyshev) approximation of
    inflating the robot by that distance. Cells outside the grid are yielded
    too, so callers can treat them as blocked.
    """
    polygon = footprint_in_world(footprint_base, pose.x_m, pose.y_m, pose.yaw_rad)
    res = grid.resolution_m
    xs = [v[0] for v in polygon]
    ys = [v[1] for v in polygon]
    col_lo = int(floor((min(xs) - margin_m - grid.origin_x_m) / res))
    col_hi = int(floor((max(xs) + margin_m - grid.origin_x_m) / res))
    row_lo = int(floor((min(ys) - margin_m - grid.origin_y_m) / res))
    row_hi = int(floor((max(ys) + margin_m - grid.origin_y_m) / res))
    for row in range(row_lo, row_hi + 1):
        for column in range(col_lo, col_hi + 1):
            x0 = grid.origin_x_m + column * res
            y0 = grid.origin_y_m + row * res
            if polygon_intersects_square(polygon, x0 - margin_m, y0 - margin_m,
                                         x0 + res + margin_m, y0 + res + margin_m):
                yield row, column


def footprint_collides(grid: OccupancyGrid, footprint_base: Sequence[Point2],
                       pose: Pose, margin_m: float = 0.0) -> bool:
    """True if the footprint at ``pose`` touches an occupied, unknown or
    out-of-bounds cell. Unknown is never treated as free."""
    # Free cells cannot cause a collision. Skip their polygon/SAT work, which
    # dominates when A checks many alternative poses in observed free space.
    polygon = footprint_in_world(footprint_base, pose.x_m, pose.y_m, pose.yaw_rad)
    res = grid.resolution_m
    xs, ys = [v[0] for v in polygon], [v[1] for v in polygon]
    col_lo = int(floor((min(xs) - margin_m - grid.origin_x_m) / res))
    col_hi = int(floor((max(xs) + margin_m - grid.origin_x_m) / res))
    row_lo = int(floor((min(ys) - margin_m - grid.origin_y_m) / res))
    row_hi = int(floor((max(ys) + margin_m - grid.origin_y_m) / res))
    for row in range(row_lo, row_hi + 1):
        for column in range(col_lo, col_hi + 1):
            if cell_value(grid, row, column) == FREE:
                continue
            x0, y0 = grid.origin_x_m + column * res, grid.origin_y_m + row * res
            if polygon_intersects_square(polygon, x0 - margin_m, y0 - margin_m,
                                         x0 + res + margin_m, y0 + res + margin_m):
                return True
    return False
