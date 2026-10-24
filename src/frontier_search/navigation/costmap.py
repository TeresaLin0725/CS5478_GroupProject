"""Owner B: footprint-aware navigation costmap derived from A's occupancy grid.

Robotics background
-------------------
* Configuration space (C-space): planning for an extended robot body is reduced
  to planning for a *point* by growing the obstacles by the robot's shape. For a
  circular robot able to turn in place, the C-space is two-dimensional (x, y)
  and obstacles are inflated by the robot radius plus a clearance margin
  (Lozano-Perez, 1983). A cell is traversable when the robot, centred on it,
  would not touch any occupied cell.
* Why unknown is blocked: unknown cells carry no evidence of free space, so a
  safe planner treats them like obstacles (Interface Freeze section 2).
* Separation of concerns: A publishes *uninflated* physical occupancy evidence;
  B derives this separate costmap. Inflation is never baked into A's map.
* Conservative planning disc: a general polygon footprint makes C-space
  three-dimensional (x, y, yaw). To keep a 2-D grid planner both fast and safe,
  cell traversability uses the *circumscribed disc* of the footprint, i.e. the
  smallest circle about the base origin that contains the polygon. This never
  under-estimates the robot's extent. The exact polygon is still used to verify
  individual poses, driving segments and rotations (``pose_valid``,
  ``segment_valid``, ``rotation_valid``).
* Corner cutting: an 8-connected diagonal step between two free cells can still
  clip the corner of an obstacle that sits at one of the two flanking cells, so
  diagonal moves require both flanking cells to be traversable.

Only the standard library is used; annotations stay Python 3.8 compatible.
"""

from collections import deque
from math import atan2, hypot, pi
from typing import Dict, Iterator, List, Optional, Tuple

from ..contracts import GridCell, OccupancyGrid, Point2, Pose, RobotConfig
from .geometry import FREE, cell_center, circle_footprint, footprint_collides, in_bounds

# 8-connected neighbourhood: (d_row, d_col, step length in cells)
_SQRT2 = 2 ** 0.5
# Length of the first drive away from the start pose that is checked with ``start_margin_m`` (m).
START_ZONE_M = 0.10

NEIGHBOURS = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
              (1, 1, _SQRT2), (1, -1, _SQRT2), (-1, 1, _SQRT2), (-1, -1, _SQRT2))


def circumradius(footprint_base: Tuple[Point2, ...]) -> float:
    """Radius of the smallest origin-centred disc containing the polygon."""
    return max(hypot(p.x_m, p.y_m) for p in footprint_base)


def inscribed_radius(footprint_base: Tuple[Point2, ...]) -> float:
    """Radius of the largest origin-centred disc that fits inside the polygon.

    The smallest distance from the base origin to any edge. A robot whose centre is closer than
    this to an obstacle certainly collides, whatever its heading.
    """
    best = float("inf")
    n = len(footprint_base)
    for i in range(n):
        a, b = footprint_base[i], footprint_base[(i + 1) % n]
        ex, ey = b.x_m - a.x_m, b.y_m - a.y_m
        length2 = ex * ex + ey * ey
        t = 0.0 if length2 == 0 else max(0.0, min(1.0, -(a.x_m * ex + a.y_m * ey) / length2))
        best = min(best, hypot(a.x_m + t * ex, a.y_m + t * ey))
    return best


class Costmap:
    """Immutable-by-convention costmap for one occupancy-grid revision.

    A new map revision means a new ``Costmap``; nothing here mutates the grid.
    """

    def __init__(self, grid: OccupancyGrid, footprint_base: Tuple[Point2, ...],
                 clearance_margin_m: float, sample_step_m: Optional[float] = None,
                 planning_radius_m: Optional[float] = None,
                 start_margin_m: Optional[float] = None) -> None:
        if not footprint_base or len(footprint_base) < 3:
            raise ValueError("footprint needs at least three vertices")
        if clearance_margin_m < 0:
            raise ValueError("clearance margin must be nonnegative")
        if start_margin_m is not None and start_margin_m < 0:
            raise ValueError("start margin must be nonnegative")
        self.grid = grid
        self.footprint_base = tuple(footprint_base)
        self.margin_m = clearance_margin_m
        # Margin used ONLY to decide whether the robot's current pose is usable as a plan start. None keeps
        # the planning margin (default behaviour). The clearance margin is a planning safety buffer in
        # configuration space, not physics: a robot that is not touching anything is still free to
        # leave a pose that lies inside the buffer, so a start check at margin 0 avoids declaring a
        # physically free robot immobile.
        self.start_margin_m = clearance_margin_m if start_margin_m is None else start_margin_m
        self.radius_m = circumradius(self.footprint_base)
        # Planning disc. Default: the circumscribed disc, safe at every heading (conservative for a
        # long or non-square robot). An explicit smaller radius, never below the inscribed radius,
        # lets the robot use gaps it physically fits through; the exact-footprint checks on the first
        # move, in-place turns and sweeps (see first_move_valid / the navigator guards) then matter.
        if planning_radius_m is None:
            planning_radius_m = self.radius_m
        else:
            low, high = self.planning_radius_bounds(self.footprint_base)
            if not (low - 1e-9 <= planning_radius_m <= high + 1e-9):
                raise ValueError("planning radius %.4f m must lie between the inscribed radius %.4f m and "
                                 "the circumscribed radius %.4f m" % (planning_radius_m, low, high))
        self.planning_radius_m = planning_radius_m
        self._disc = circle_footprint(planning_radius_m)
        # Sample at no more than half a cell so a thin obstacle cannot be skipped.
        self.sample_step_m = sample_step_m or grid.resolution_m / 2.0
        self._cell_cache = {}  # type: Dict[Tuple[int, int], bool]
        self._disc_radius_m = circumradius(self._disc)
        self._clearance = None  # type: Optional[List[List[int]]]

    def prepare(self) -> None:
        """Precompute the clearance field once (O(cells)); optional, else lazy."""
        if self._clearance is None:
            self._clearance = self._build_clearance()

    def _build_clearance(self) -> List[List[int]]:
        """Chebyshev distance (in cells) from each cell to the nearest cell that is
        not known-free; out-of-bounds counts as not free.

        Multi-source breadth-first search over the 8-connected grid. The value is
        only a *proof of safety* used to skip the exact collision test far from
        obstacles; cells close to obstacles still use the exact polygon test, so
        the traversable set is unchanged.
        """
        grid = self.grid
        h, w = grid.height_cells, grid.width_cells
        far = h + w + 1
        dist = [[far] * w for _ in range(h)]
        queue = deque()
        for r in range(h):
            row = grid.data[r]
            for c in range(w):
                # A cell touching the grid border has an out-of-bounds neighbour at 1.
                if row[c] != FREE:
                    dist[r][c] = 0
                    queue.append((r, c))
                elif r == 0 or c == 0 or r == h - 1 or c == w - 1:
                    dist[r][c] = 1
                    queue.append((r, c))
        # Seeds are not in distance order (0s and border 1s mixed): fix with a
        # stable two-pass queue by processing zero-distance seeds first.
        queue = deque(sorted(queue, key=lambda rc: dist[rc[0]][rc[1]]))
        while queue:
            r, c = queue.popleft()
            d = dist[r][c] + 1
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < h and 0 <= nc < w and dist[nr][nc] > d:
                        dist[nr][nc] = d
                        queue.append((nr, nc))
        return dist

    @classmethod
    def from_robot(cls, grid: OccupancyGrid, robot: RobotConfig,
                   planning_radius_m: Optional[float] = None,
                   start_margin_m: Optional[float] = None,
                   margin_m: Optional[float] = None) -> "Costmap":
        # ``margin_m`` overrides the robot's configured clearance margin for planning (None = as configured).
        margin = robot.clearance_margin_m if margin_m is None else margin_m
        return cls(grid, robot.footprint_base, margin, planning_radius_m=planning_radius_m,
                   start_margin_m=start_margin_m)

    @staticmethod
    def planning_radius_bounds(footprint_base: Tuple[Point2, ...]) -> Tuple[float, float]:
        """(inscribed, circumscribed) radius: the allowed range for an explicit planning radius."""
        return inscribed_radius(tuple(footprint_base)), circumradius(tuple(footprint_base))

    # --- per-cell (planning lattice) ---------------------------------------

    def cell_traversable(self, row: int, column: int) -> bool:
        """True if the circumscribed disc centred on the cell touches no
        occupied, unknown or out-of-bounds cell (cached)."""
        if not in_bounds(self.grid, row, column):
            return False
        key = (row, column)
        cached = self._cell_cache.get(key)
        if cached is None:
            self.prepare()
            d = self._clearance[row][column]
            if d == 0:
                cached = False  # the cell itself is not known-free
                self._cell_cache[key] = cached
                return cached
            # The nearest blocked cell is d cells away, so its (margin-grown) square
            # is at least (d - 0.5) * res - margin from the cell centre along one
            # axis. If that exceeds the disc radius, no collision is possible.
            if (d - 0.5) * self.grid.resolution_m - self.margin_m > self._disc_radius_m:
                cached = True
                self._cell_cache[key] = cached
                return cached
            centre = cell_center(self.grid, row, column)
            pose = Pose(centre.x_m, centre.y_m, 0.0, 0.0)
            cached = not footprint_collides(self.grid, self._disc, pose, self.margin_m)
            self._cell_cache[key] = cached
        return cached

    def diagonal_ok(self, row: int, column: int, d_row: int, d_col: int) -> bool:
        """Corner-cutting rule: both flanking cells of a diagonal step must be
        traversable. Orthogonal steps have no flanks and always pass."""
        if d_row == 0 or d_col == 0:
            return True
        return (self.cell_traversable(row + d_row, column)
                and self.cell_traversable(row, column + d_col))

    def neighbours(self, cell: GridCell) -> Iterator[Tuple[GridCell, float]]:
        """Traversable 8-connected neighbours with step cost in metres."""
        res = self.grid.resolution_m
        for d_row, d_col, factor in NEIGHBOURS:
            row, column = cell.row + d_row, cell.column + d_col
            if self.cell_traversable(row, column) and self.diagonal_ok(
                    cell.row, cell.column, d_row, d_col):
                yield GridCell(row, column), factor * res

    # --- exact polygon checks (execution safety) ---------------------------

    def pose_valid(self, pose: Pose, margin_m: Optional[float] = None) -> bool:
        """Exact footprint at the pose (with yaw) touches no blocked cell.

        ``margin_m`` None = the planning margin; the start-pose checks pass ``start_margin_m``.
        """
        margin = self.margin_m if margin_m is None else margin_m
        return not footprint_collides(self.grid, self.footprint_base, pose, margin)

    def start_pose_valid(self, pose: Pose) -> bool:
        """Exact footprint at the start pose touches no blocked cell, using ``start_margin_m``."""
        return not footprint_collides(self.grid, self.footprint_base, pose, self.start_margin_m)

    def near_obstacle(self, pose: Pose, extra_m: float) -> bool:
        """True if the exact footprint, grown by margin + ``extra_m``, touches a blocked cell.

        Used by the controller to detect proximity (a speed-limit zone around obstacles); it never
        changes what the planner accepts.
        """
        return footprint_collides(self.grid, self.footprint_base, pose, self.margin_m + extra_m)

    def segment_valid(self, start: Pose, end: Pose, start_zone_m: float = 0.0) -> bool:
        """Driving from start to end along a straight line is collision-free.

        The footprint is checked at spacing <= ``sample_step_m`` using the
        heading of motion, including both end points. Samples within ``start_zone_m`` of ``start``
        use ``start_margin_m`` instead of the planning margin (0 = none, the default): a robot parked
        inside the planning buffer must be able to drive OUT of it (see ``first_move_valid``).
        """
        dx, dy = end.x_m - start.x_m, end.y_m - start.y_m
        length = hypot(dx, dy)
        heading = atan2(dy, dx) if length > 1e-9 else start.yaw_rad
        steps = max(1, int(length / self.sample_step_m + 0.999999))
        for i in range(steps + 1):
            t = i / steps
            pose = Pose(start.x_m + t * dx, start.y_m + t * dy, start.z_m, heading)
            margin = self.start_margin_m if t * length <= start_zone_m + 1e-9 else None
            if not self.pose_valid(pose, margin):
                return False
        return True

    def first_move_valid(self, start: Pose, target_x_m: float, target_y_m: float) -> bool:
        """The robot can leave its REAL pose toward a waypoint: turn in place, then drive there.

        Planning uses the circumscribed disc, which is safe at cell centres, but the start pose is
        generally off-centre and its cell may be inflation-blocked, so the turn must be checked
        with the exact footprint. The controller is rotate-then-translate, so the turn is the
        shortest arc from the current yaw to the heading of the move, followed by the straight
        segment. Only the turn that is actually needed is checked (a move straight ahead needs none).

        The turn and the first ``START_ZONE_M`` of the drive are checked with ``start_margin_m``, the
        same margin that accepted the start pose. Otherwise a robot parked inside the planning buffer
        (a stopping error, or an obstacle cell that appeared under it) is "valid where it stands" yet
        every first step fails, so it can never leave. The planning buffer is configuration-space
        inflation, not physics; the exact footprint is still checked, and past the zone the normal
        planning margin applies again.
        """
        dx, dy = target_x_m - start.x_m, target_y_m - start.y_m
        heading = atan2(dy, dx) if hypot(dx, dy) > 1e-9 else start.yaw_rad
        if not self.rotation_valid(start, heading, margin_m=self.start_margin_m):
            return False
        turned = Pose(start.x_m, start.y_m, start.z_m, heading)
        return self.segment_valid(turned, Pose(target_x_m, target_y_m, start.z_m, heading),
                                  start_zone_m=START_ZONE_M)

    def rotation_valid(self, pose: Pose, target_yaw_rad: float, step_rad: float = 0.1,
                       margin_m: Optional[float] = None) -> bool:
        """Turning in place from ``pose.yaw_rad`` to ``target_yaw_rad`` along the
        shortest arc is collision-free (checked at ``step_rad`` increments).
        ``margin_m`` None = the planning margin."""
        delta = (target_yaw_rad - pose.yaw_rad + pi) % (2 * pi) - pi
        return self._sweep_valid(pose, pose.yaw_rad, delta, step_rad, margin_m)

    def full_rotation_valid(self, pose: Pose, step_rad: float = 0.1,
                            margin_m: Optional[float] = None) -> bool:
        """A complete 360-degree turn in place is collision-free."""
        return self._sweep_valid(pose, pose.yaw_rad, 2 * pi, step_rad, margin_m)

    def _sweep_valid(self, pose: Pose, start_yaw: float, total_rad: float,
                     step_rad: float, margin_m: Optional[float] = None) -> bool:
        if step_rad <= 0:
            raise ValueError("step_rad must be positive")
        steps = max(1, int(abs(total_rad) / step_rad + 0.999999))
        for i in range(steps + 1):
            yaw = start_yaw + total_rad * i / steps
            if not self.pose_valid(Pose(pose.x_m, pose.y_m, pose.z_m, yaw), margin_m):
                return False
        return True
