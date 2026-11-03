"""Owner B: why was a frontier judged unreachable? (read-only failure diagnostics for the report)

Robotics background
-------------------
Planning inflates obstacles by the robot's radius plus a safety margin (configuration space, Lozano-Perez 1983).
A frontier judged NO_PATH / INVALID_GOAL is therefore unreachable *for that inflated model*. This module re-asks the
question with progressively smaller inflation radii and reports the smallest clearance at which the frontier becomes
reachable. Three causes follow, and they need different fixes:

* ``reachable``: not a failure (the strict model finds a path).
* ``conservative_planning``: reachable with a smaller inflation, so the failure is a modelling choice (footprint or
  margin too large), not the map. ``slack_m`` says by how much; this is what a round footprint or smaller margin fixes.
* ``blocked``: unreachable even at the smallest physically possible disc (the inscribed radius, margin 0): walls,
  unknown space or a corridor the robot cannot fit. Only new map evidence or another route helps.
* ``start_unusable`` / ``goal_out_of_bounds``: even at margin 0 the robot's own pose cannot be planned from, or the
  goal lies outside the map. ``strict_start_usable`` False with ``conservative_planning`` means only the strict start
  check (planning margin) failed.

The measure is limited by the grid: with 0.1 m cells, clearance changes in steps, so ``min_reachable_clearance_m``
is often the inscribed radius itself (a binary "fits / does not fit"), not a fine threshold.

Nothing here moves the robot or changes the active operation. Only the standard library is used.
"""

from math import floor, hypot
from typing import Dict, List, Optional, Sequence, Tuple

from ..contracts import OccupancyGrid, Pose, RobotConfig
from . import planner
from .costmap import Costmap
from .geometry import world_to_cell

STEP_M = 0.01


def _clearance_to_blocked_m(grid: OccupancyGrid, pose: Pose, limit_m: float = 1.0) -> Optional[float]:
    """Distance from the pose to the edge of the nearest non-free cell, None if none within ``limit_m``."""
    res = grid.resolution_m
    col = floor((pose.x_m - grid.origin_x_m) / res)
    row = floor((pose.y_m - grid.origin_y_m) / res)
    reach = int(limit_m / res) + 1
    best = None
    for r in range(row - reach, row + reach + 1):
        for c in range(col - reach, col + reach + 1):
            inside = 0 <= r < grid.height_cells and 0 <= c < grid.width_cells
            if inside and grid.data[r][c] == 0:
                continue
            x0, y0 = grid.origin_x_m + c * res, grid.origin_y_m + r * res
            dx = max(x0 - pose.x_m, 0.0, pose.x_m - (x0 + res))
            dy = max(y0 - pose.y_m, 0.0, pose.y_m - (y0 + res))
            d = hypot(dx, dy)
            if d <= limit_m and (best is None or d < best):
                best = d
    return best


def _settings(robot: RobotConfig, strict_radius_m: float, strict_margin_m: float) -> List[Tuple[float, float]]:
    """(planning radius, margin) pairs in ascending total clearance, up to the strict model (exclusive)."""
    low, high = Costmap.planning_radius_bounds(robot.footprint_base)
    high = min(high, strict_radius_m)
    out = []  # type: List[Tuple[float, float]]
    radius = low
    while radius < high - 1e-9:
        out.append((radius, 0.0))
        radius += STEP_M
    out.append((high, 0.0))
    margin = STEP_M
    while margin < strict_margin_m - 1e-9:
        out.append((high, margin))
        margin += STEP_M
    return out


def _reachable(costmap: Costmap, start: Pose, goals: Dict[str, Pose]) -> Optional[Dict[str, bool]]:
    if not planner.start_is_usable(costmap, start):
        return None
    result = planner.distance_field(costmap, start, None)
    if result is None:
        return None
    field = result[0]
    return {fid: planner.goal_is_valid(costmap, g) and planner.cost_to_goal_m(costmap, start, g, field) is not None
            for fid, g in goals.items()}


def diagnose_goals(grid: OccupancyGrid, robot: RobotConfig, start: Pose, goals: Dict[str, Pose],
                   strict_radius_m: Optional[float] = None, strict_margin_m: Optional[float] = None,
                   strict_start_margin_m: Optional[float] = None) -> Dict[str, Dict[str, object]]:
    """Per goal: cause, the smallest clearance that makes it reachable, and how conservative the strict model is."""
    low, high = Costmap.planning_radius_bounds(robot.footprint_base)
    radius = high if strict_radius_m is None else strict_radius_m
    margin = robot.clearance_margin_m if strict_margin_m is None else strict_margin_m
    strict_clearance = radius + margin
    strict = Costmap.from_robot(grid, robot, radius, strict_start_margin_m, margin)
    strict.prepare()
    strict_ok = _reachable(strict, start, goals)
    found = {}  # type: Dict[str, float]
    # A strict start check can fail only because of the planning margin; the relaxed search uses start margin 0
    # (physics), so it still runs and tells a conservative start check from a robot that really cannot move.
    pending = {fid: g for fid, g in goals.items() if strict_ok is None or not strict_ok[fid]}
    start_ok = strict_ok is not None
    for r, m in _settings(robot, radius, margin):
        if not pending:
            break
        relaxed = Costmap.from_robot(grid, robot, r, 0.0, m)
        relaxed.prepare()
        ok = _reachable(relaxed, start, pending)
        if ok is None:
            continue
        start_ok = True
        for fid, good in ok.items():
            if good:
                found[fid] = r + m
        pending = {fid: g for fid, g in pending.items() if fid not in found}
    report = {}  # type: Dict[str, Dict[str, object]]
    for fid, goal in goals.items():
        in_bounds = world_to_cell(grid, goal.x_m, goal.y_m) is not None
        if not in_bounds:
            cause = 'goal_out_of_bounds'
        elif strict_ok is not None and strict_ok[fid]:
            cause = 'reachable'
        elif fid in found:
            cause = 'conservative_planning'
        elif not start_ok:
            cause = 'start_unusable'
        else:
            cause = 'blocked'
        needed = found.get(fid)
        report[fid] = dict(
            cause=cause,
            goal_clearance_m=None if not in_bounds else _clearance_to_blocked_m(grid, goal),
            strict_start_usable=strict_ok is not None,
            strict_clearance_m=strict_clearance,
            min_reachable_clearance_m=strict_clearance if cause == 'reachable' else needed,
            slack_m=None if needed is None else strict_clearance - needed,
            inscribed_radius_m=low,
        )
    return report
