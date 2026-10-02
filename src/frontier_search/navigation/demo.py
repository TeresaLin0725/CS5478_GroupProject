"""Owner B: runnable demo of planning and execution on the mock simulator.

    PYTHONPATH=src python3 -m frontier_search.navigation.demo [fixture]

Fixtures: open_space, wall_detour (default), narrow_doorway, unknown_barrier,
blocked_update. Prints an ASCII map ('#' occupied, '?' unknown, '.' free), the
planned path ('*'), the path the robot actually drove ('o'), and summary numbers.
The robot is the B placeholder in ``dev_robot.py``, not a frozen team choice.
"""

import argparse
from typing import List, Tuple

from ..config import NavigationConfig, ObservationConfig
from ..contracts import (NavigationRequest, NavigationStatus, NavigatorInit, PolicyName, Subgoal)
from ..fixtures.loader import load_navigation_fixture
from ..simulation.mock_adapter import MockSimulator
from .dev_robot import PLACEHOLDER_ROBOT
from .geometry import world_to_cell
from .navigator import TeamNavigator

_TERMINAL = (NavigationStatus.REACHED, NavigationStatus.FAILED, NavigationStatus.CANCELLED)


def _render(grid, planned, driven, start, goal) -> str:
    chars = [['#' if v == 100 else '?' if v == -1 else '.' for v in row] for row in grid.data]

    def mark(pose, char):
        cell = world_to_cell(grid, pose.x_m, pose.y_m)
        if cell is not None:
            chars[cell.row][cell.column] = char

    for pose in planned:
        mark(pose, '*')
    for pose in driven:
        mark(pose, 'o')
    mark(start, 'S')
    mark(goal, 'G')
    return "\n".join("".join(row) for row in reversed(chars))  # row 0 (low y) at the bottom


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("fixture", nargs="?", default="wall_detour")
    parser.add_argument("--max-steps", type=int, default=3000)
    args = parser.parse_args(argv)

    fixture = load_navigation_fixture(args.fixture)
    sim = MockSimulator(fixture.grid, PLACEHOLDER_ROBOT)
    sim.reset("demo", "mock", fixture.start_pose, 0)
    navigator = TeamNavigator(sim)
    navigator.reset(NavigatorInit("demo", PLACEHOLDER_ROBOT, ObservationConfig(), NavigationConfig()))
    subgoal = Subgoal("F1", fixture.goal_pose, fixture.grid.revision, PolicyName.NEAREST)
    feedback = navigator.start(NavigationRequest("demo", "g1", fixture.grid, fixture.start_pose, subgoal))
    planned = feedback.planner_result.path if feedback.planner_result and feedback.planner_result.success else ()
    planned_cost = feedback.planner_result.path_cost_m if planned else None  # first plan, before any replan
    print("fixture: %s | start -> %s (%s)" % (args.fixture, feedback.status.name,
                                              feedback.failure_reason.name if feedback.failure_reason else "plan ok"))

    driven = []  # type: List
    for step in range(args.max_steps if feedback.status == NavigationStatus.ACCEPTED else 0):
        sim.step()
        navigator.on_step()
        driven.append(sim.current_pose())
        if args.fixture == "blocked_update" and fixture.updated_grid is not None and step == 20:
            navigator.update_map("demo", "g1", fixture.updated_grid)
        feedback = navigator.poll("demo", "g1")
        if feedback.status in _TERMINAL:
            break

    print(_render(fixture.grid, planned, driven, fixture.start_pose, fixture.goal_pose))
    cost = "%.2f m" % planned_cost if planned_cost is not None else "n/a"
    print("\nfinal status: %s%s" % (feedback.status.name,
                                    " (%s)" % feedback.failure_reason.name if feedback.failure_reason else ""))
    print("planned cost: %s | executed distance: %.2f m | sim time: %.1f s | collisions: %d | replans: %d"
          % (cost, sim.executed_distance_m, sim.sim_time_s(), sim.collisions,
             sum(1 for kind, _ in navigator.events if kind == "replan")))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
