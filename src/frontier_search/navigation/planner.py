"""Owner B: A* path planner and Dijkstra distance field on the costmap.

Robotics background
-------------------
* A* (Hart, Nilsson and Raphael, 1968) expands nodes in order of
  f(n) = g(n) + h(n), where g is the cost so far and h estimates the cost to go.
  If h never overestimates (admissible) and is consistent, the first time the
  goal is expanded the path is optimal and no node is re-expanded.
* Heuristic: on an 8-connected grid with step costs 1 (straight) and sqrt(2)
  (diagonal) the exact obstacle-free cost is the *octile distance*
  ``(dx + dy) + (sqrt(2) - 2) * min(dx, dy)`` (in cells). It is admissible and
  consistent here because obstacles can only make paths longer.
* Dijkstra field: A* with h = 0 run from the start until the frontier is
  exhausted gives the cheapest cost to *every* reachable cell. One run can
  answer many candidate goals, which is the amortisation allowed for
  ``evaluate_candidates`` (a one-to-many query instead of one A* per frontier).
* Completeness: if a path exists on the discretised costmap the search finds
  it; if the open list empties first, no path exists at this resolution and
  inflation (``NO_PATH``). A time limit gives ``TIMEOUT``, which is
  *inconclusive*, not proof of unreachability.
* Costs are metres of path length, the quantity the Nearest and Information
  Gain policies consume (``path_cost_m``).

Only the standard library is used; annotations stay Python 3.8 compatible.
"""

from heapq import heappop, heappush
from math import atan2, hypot
from time import monotonic
from typing import Callable, Dict, List, Optional, Tuple

from ..contracts import FailureReason, GridCell, Pose, PlannerResult
from .costmap import Costmap
from .geometry import cell_center, world_to_cell

_SQRT2 = 2 ** 0.5
Cell = Tuple[int, int]


def octile_distance_m(a: Cell, b: Cell, resolution_m: float) -> float:
    """Admissible, consistent heuristic for the 8-connected grid (metres)."""
    dx, dy = abs(a[1] - b[1]), abs(a[0] - b[0])
    return resolution_m * ((dx + dy) + (_SQRT2 - 2.0) * min(dx, dy))


def _endpoint_cell(costmap: Costmap, pose: Pose) -> Optional[GridCell]:
    return world_to_cell(costmap.grid, pose.x_m, pose.y_m)


def _offset_m(costmap: Costmap, pose: Pose, cell: GridCell) -> float:
    """Straight-line distance between a pose and its cell centre."""
    centre = cell_center(costmap.grid, cell.row, cell.column)
    return hypot(pose.x_m - centre.x_m, pose.y_m - centre.y_m)


def _first_move_ok(costmap: Costmap, start: Pose, cell: GridCell) -> bool:
    """Leaving the real start pose toward this neighbour is turn-safe with the exact footprint."""
    centre = cell_center(costmap.grid, cell.row, cell.column)
    return costmap.first_move_valid(start, centre.x_m, centre.y_m)


def start_is_usable(costmap: Costmap, start: Pose) -> bool:
    """The robot's exact footprint at the start pose must be collision-free.

    The start cell itself is allowed to be inflation-blocked (the robot may sit
    close to a wall after a verified spawn); only the exact pose must be valid. The margin used for
    this check is ``costmap.start_margin_m`` (default: the planning margin).
    """
    return costmap.start_pose_valid(start) and _endpoint_cell(costmap, start) is not None


def goal_is_valid(costmap: Costmap, goal: Pose) -> bool:
    cell = _endpoint_cell(costmap, goal)
    return (cell is not None and costmap.cell_traversable(cell.row, cell.column)
            and costmap.pose_valid(goal))


def plan(costmap: Costmap, start: Pose, goal: Pose, deadline_s: Optional[float] = None,
         clock: Callable[[], float] = monotonic, smooth: bool = False) -> PlannerResult:
    """Plan a path from ``start`` to ``goal`` with A*.

    Failure mapping: ``INVALID_GOAL`` if the goal cannot hold the robot,
    ``NO_PATH`` if the start is unusable or the goal is unreachable on this map,
    ``TIMEOUT`` if the optional deadline passes first.

    ``smooth=True`` post-processes the grid path with line-of-sight shortcuts
    (see ``smooth_path``). The reported cost is then the smoothed length, which
    can be below the Dijkstra-field cost that ``evaluate_candidates`` reports.
    """
    if not goal_is_valid(costmap, goal):
        return _fail(FailureReason.INVALID_GOAL)
    if not start_is_usable(costmap, start):
        return _fail(FailureReason.NO_PATH)
    res = costmap.grid.resolution_m
    start_cell = _endpoint_cell(costmap, start)
    goal_cell = _endpoint_cell(costmap, goal)
    s, g = (start_cell.row, start_cell.column), (goal_cell.row, goal_cell.column)
    deadline = None if deadline_s is None else clock() + deadline_s

    best = {s: 0.0}  # type: Dict[Cell, float]
    parent = {}  # type: Dict[Cell, Cell]
    closed = set()  # type: set
    tie = 0  # deterministic tie-break: insertion order
    open_heap = [(octile_distance_m(s, g, res), tie, s)]
    expanded = 0
    while open_heap:
        _, _, node = heappop(open_heap)
        if node in closed:
            continue
        if node == g:
            result = _success(costmap, start, goal, start_cell, goal_cell, _backtrack(parent, node),
                              best[node])
            return _smoothed(costmap, result) if smooth else result
        closed.add(node)
        expanded += 1
        if deadline is not None and expanded % 64 == 0 and clock() > deadline:
            return _fail(FailureReason.TIMEOUT)
        for neighbour, step_m in costmap.neighbours(GridCell(node[0], node[1])):
            if node == s and not _first_move_ok(costmap, start, neighbour):
                continue  # the first turn/drive from the real pose would sweep into an obstacle
            n = (neighbour.row, neighbour.column)
            cost = best[node] + step_m
            if n not in closed and cost < best.get(n, float("inf")):
                best[n] = cost
                parent[n] = node
                tie += 1
                heappush(open_heap, (cost + octile_distance_m(n, g, res), tie, n))
    return _fail(FailureReason.NO_PATH)


def distance_field(costmap: Costmap, start: Pose, deadline_s: Optional[float] = None,
                   clock: Callable[[], float] = monotonic) -> Optional[Tuple[Dict[Cell, float], bool]]:
    """Dijkstra from the start cell to every reachable cell.

    Returns ``(costs, complete)`` where costs maps cell -> metres along the grid
    path (excluding the pose-to-centre offsets), and ``complete`` is False if the
    deadline cut the search short (unlisted cells are then *unknown*, not
    unreachable). Returns None if the start pose is unusable.
    """
    if not start_is_usable(costmap, start):
        return None
    start_cell = _endpoint_cell(costmap, start)
    s = (start_cell.row, start_cell.column)
    deadline = None if deadline_s is None else clock() + deadline_s
    best = {s: 0.0}  # type: Dict[Cell, float]
    done = set()  # type: set
    tie = 0
    heap = [(0.0, tie, s)]
    expanded = 0
    while heap:
        cost, _, node = heappop(heap)
        if node in done:
            continue
        done.add(node)
        expanded += 1
        if deadline is not None and expanded % 64 == 0 and clock() > deadline:
            return {c: v for c, v in best.items() if c in done}, False
        for neighbour, step_m in costmap.neighbours(GridCell(node[0], node[1])):
            if node == s and not _first_move_ok(costmap, start, neighbour):
                continue  # same rule as plan(), so evaluation and execution agree
            n = (neighbour.row, neighbour.column)
            new_cost = cost + step_m
            if n not in done and new_cost < best.get(n, float("inf")):
                best[n] = new_cost
                tie += 1
                heappush(heap, (new_cost, tie, n))
    return {c: v for c, v in best.items() if c in done}, True


def cost_to_goal_m(costmap: Costmap, start: Pose, goal: Pose, field: Dict[Cell, float]) -> Optional[float]:
    """Path cost to ``goal`` read from a distance field; None if not in the field.

    Matches the cost that ``plan`` reports for the same start and goal.
    """
    start_cell, goal_cell = _endpoint_cell(costmap, start), _endpoint_cell(costmap, goal)
    if start_cell is None or goal_cell is None:
        return None
    grid_cost = field.get((goal_cell.row, goal_cell.column))
    if grid_cost is None:
        return None
    return grid_cost + _offset_m(costmap, start, start_cell) + _offset_m(costmap, goal, goal_cell)


def path_valid(costmap: Costmap, path: Tuple[Pose, ...]) -> bool:
    """Exact-footprint check of every segment of a path (execution safety)."""
    if not path or not costmap.pose_valid(path[0]):
        return False
    return all(costmap.segment_valid(a, b) for a, b in zip(path, path[1:]))


def smooth_path(costmap: Costmap, path: Tuple[Pose, ...]) -> Tuple[Pose, ...]:
    """Greedy line-of-sight shortcutting ("string pulling").

    A grid path is optimal for 8-connected moves but zig-zags in 45-degree
    steps. From each anchor, jump to the farthest later waypoint that is visible
    with the exact footprint (``segment_valid``), then repeat. The result is
    never longer than the input and every segment is verified collision-free.
    Intermediate headings follow the new segments; the first and last poses keep
    the start yaw and the goal yaw.
    """
    if len(path) <= 2:
        return path
    keep = [0]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not costmap.segment_valid(path[i], path[j]):
            j -= 1
        keep.append(j)
        i = j
    points = [path[k] for k in keep]
    poses = []  # type: List[Pose]
    for n, p in enumerate(points):
        if n == 0 or n == len(points) - 1:
            poses.append(p)
        else:
            nxt = points[n + 1]
            poses.append(Pose(p.x_m, p.y_m, p.z_m, atan2(nxt.y_m - p.y_m, nxt.x_m - p.x_m)))
    return tuple(poses)


def path_length_m(path: Tuple[Pose, ...]) -> float:
    return sum(hypot(b.x_m - a.x_m, b.y_m - a.y_m) for a, b in zip(path, path[1:]))


def _smoothed(costmap: Costmap, result: PlannerResult) -> PlannerResult:
    path = smooth_path(costmap, result.path)
    return PlannerResult(True, path, path_length_m(path), None)


# --- helpers ----------------------------------------------------------------

def _fail(reason: FailureReason) -> PlannerResult:
    return PlannerResult(False, None, None, reason)


def _backtrack(parent: Dict[Cell, Cell], node: Cell) -> List[Cell]:
    cells = [node]
    while node in parent:
        node = parent[node]
        cells.append(node)
    cells.reverse()
    return cells


def _success(costmap: Costmap, start: Pose, goal: Pose, start_cell: GridCell,
             goal_cell: GridCell, cells: List[Cell], grid_cost_m: float) -> PlannerResult:
    """Build the pose path: exact start, cell centres, exact goal.

    Intermediate headings follow the direction of travel; the final pose takes
    the requested goal yaw (arrival requires position and yaw tolerances).
    """
    waypoints = [(start.x_m, start.y_m)]
    for row, column in cells:
        centre = cell_center(costmap.grid, row, column)
        waypoints.append((centre.x_m, centre.y_m))
    waypoints.append((goal.x_m, goal.y_m))
    poses = []  # type: List[Pose]
    for i, (x, y) in enumerate(waypoints):
        if i == 0:
            yaw = start.yaw_rad
        elif i == len(waypoints) - 1:
            yaw = goal.yaw_rad
        else:
            nx, ny = waypoints[i + 1]
            yaw = atan2(ny - y, nx - x) if (nx, ny) != (x, y) else poses[-1].yaw_rad
        poses.append(Pose(x, y, start.z_m, yaw))
    total = grid_cost_m + _offset_m(costmap, start, start_cell) + _offset_m(costmap, goal, goal_cell)
    return PlannerResult(True, tuple(poses), total, None)
