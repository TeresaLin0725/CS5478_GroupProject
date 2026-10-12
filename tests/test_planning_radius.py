"""P6: an opt-in planning radius between the robot's inscribed and circumscribed radius.

Default (None) keeps the conservative circumscribed disc. A smaller disc lets a rectangular robot use
gaps it physically fits through; the exact-footprint checks (first move, in-place turns, margin)
still apply. Evidence on the ideal mock: 0.20-0.22 m threads 0.5 and 0.6 m gaps without contact;
NOT validated on iGibson.
"""

from math import pi
from random import Random
import unittest

from frontier_search.config import NavigationConfig, ObservationConfig
from frontier_search.contracts import (FailureReason, NavigationRequest, NavigationStatus, NavigatorInit,
                                       OccupancyGrid, PolicyName, Pose, Subgoal)
from frontier_search.navigation.costmap import Costmap
from frontier_search.navigation.locobot_robot import LOCOBOT_DEV_ROBOT as ROBOT
from frontier_search.navigation.navigator import NavigatorBoundaryError, TeamNavigator
from frontier_search.navigation.planner import cost_to_goal_m, distance_field, plan
from frontier_search.simulation.mock_adapter import MockSimulator

DONE = (NavigationStatus.REACHED, NavigationStatus.FAILED, NavigationStatus.CANCELLED)
RADIUS = 0.22


def world(gap_cells, blocked_wall=True):
    """Free room (rows 5..34, columns 5..54, 0.1 m) with a wall at column 30 and a gap of N cells."""
    cells = [[-1] * 60 for _ in range(40)]
    for r in range(5, 35):
        for c in range(5, 55):
            cells[r][c] = 0
    for r in range(5, 35):
        cells[r][30] = 100
    low = 20 - gap_cells // 2
    for r in range(low, low + gap_cells):
        cells[r][30] = 0
    return OccupancyGrid('w', 1, .1, 0, 0, 60, 40, tuple(map(tuple, cells))), (low + gap_cells / 2) * 0.1


def bent_corridor(width_cells):
    """Free L-shaped corridor: a horizontal arm, then a vertical arm going up (a 90 degree bend)."""
    cells = [[-1] * 50 for _ in range(50)]
    for r in range(5, 5 + width_cells):
        for c in range(5, 40):
            cells[r][c] = 0
    for r in range(5, 45):
        for c in range(40 - width_cells, 40):
            cells[r][c] = 0
    return OccupancyGrid('w', 1, .1, 0, 0, 50, 50, tuple(map(tuple, cells)))


def corridor_trip(width_cells):
    grid = bent_corridor(width_cells)
    return grid, Pose(0.9, (5 + width_cells / 2) * 0.1, 0, 0), Pose((40 - width_cells / 2) * 0.1, 4.0, 0, 0)


def drive(test, grid, start, goal, planning_radius_m):
    sim = MockSimulator(grid, ROBOT)
    test.addCleanup(sim.close)
    sim.reset('x', 'm', start, 0)
    nav = TeamNavigator(sim, planning_radius_m=planning_radius_m)
    nav.reset(NavigatorInit('x', ROBOT, ObservationConfig(), NavigationConfig(navigation_deadline_s=600)))
    feedback = nav.start(NavigationRequest('x', 'g', grid, start, Subgoal('F1', goal, 1, PolicyName.NEAREST)))
    first = feedback
    for _ in range(14000):
        if feedback.status in DONE:
            break
        sim.step(); nav.on_step(); feedback = nav.poll('x', 'g')
    return first, feedback, sim


class PlanningRadiusTests(unittest.TestCase):
    def test_default_is_unchanged_circumscribed_disc(self):
        for gap, expect_path in ((5, False), (6, False), (7, True)):
            grid, yc = world(gap)
            first, last, sim = drive(self, grid, Pose(2.0, yc, 0, 0), Pose(4.0, yc, 0, 0), None)
            with self.subTest(gap_cells=gap):
                if expect_path:
                    self.assertEqual((last.status, sim.collisions), (NavigationStatus.REACHED, 0))
                else:
                    self.assertEqual((first.status, first.failure_reason), (NavigationStatus.FAILED, FailureReason.NO_PATH))

    def test_smaller_radius_threads_half_and_six_tenths_metre_gaps_without_contact(self):
        for gap in (5, 6):
            grid, yc = world(gap)
            first, last, sim = drive(self, grid, Pose(2.0, yc, 0, 0), Pose(4.0, yc, 0, 0), RADIUS)
            with self.subTest(gap_cells=gap):
                self.assertEqual(first.status, NavigationStatus.ACCEPTED)
                self.assertEqual((last.status, sim.collisions), (NavigationStatus.REACHED, 0))

    def test_random_starts_goals_and_yaws_also_pass_without_contact(self):
        rng = Random(21)
        for gap in (5, 6):
            for _ in range(6):
                grid, yc = world(gap)
                start = Pose(rng.uniform(1.0, 2.5), rng.uniform(1.0, 3.0), 0, rng.uniform(-pi, pi))
                goal = Pose(rng.uniform(3.5, 5.0), rng.uniform(1.0, 3.0), 0, rng.uniform(-pi, pi))
                first, last, sim = drive(self, grid, start, goal, RADIUS)
                with self.subTest(gap=gap, start=start, goal=goal):
                    self.assertEqual((last.status, sim.collisions), (NavigationStatus.REACHED, 0))

    def test_margin_still_binds_a_gap_narrower_than_body_plus_margin(self):
        inscribed, _ = Costmap.planning_radius_bounds(ROBOT.footprint_base)
        grid, yc = world(4)                                  # 0.40 m < 0.362 m + 2 * 0.02 m margin
        first, _, _ = drive(self, grid, Pose(2.0, yc, 0, 0), Pose(4.0, yc, 0, 0), inscribed)
        self.assertEqual((first.status, first.failure_reason), (NavigationStatus.FAILED, FailureReason.NO_PATH))

    def test_evaluation_and_execution_planning_agree_with_the_smaller_radius(self):
        rng, checked = Random(4), 0
        for gap in (5, 6, 7):
            grid, yc = world(gap)
            costmap = Costmap.from_robot(grid, ROBOT, RADIUS)
            for _ in range(10):
                start = Pose(rng.uniform(1.0, 2.5), rng.uniform(1.0, 3.0), 0, rng.uniform(-pi, pi))
                goal = Pose(rng.uniform(3.5, 5.0), rng.uniform(1.0, 3.0), 0, rng.uniform(-pi, pi))
                field = distance_field(costmap, start)
                if field is None:
                    continue
                checked += 1
                self.assertEqual(cost_to_goal_m(costmap, start, goal, field[0]) is not None,
                                 plan(costmap, start, goal).success)
        self.assertGreater(checked, 15)

    def test_first_turn_protection_still_applies_next_to_a_wall(self):
        cells = [[-1] * 40 for _ in range(40)]
        for r in range(5, 35):
            for c in range(5, 35):
                cells[r][c] = 0
        grid = OccupancyGrid('w', 1, .1, 0, 0, 40, 40, tuple(map(tuple, cells)))
        start, goal = Pose(1.55, 0.5 + 0.21, 0, 0), Pose(1.55, 2.5, 0, 0)     # 0.21 m from the wall, 90 degree turn
        first, last, sim = drive(self, grid, start, goal, 0.2)
        self.assertEqual((first.status, first.failure_reason), (NavigationStatus.FAILED, FailureReason.NO_PATH))
        self.assertEqual(sim.collisions, 0)

    def test_known_limit_a_bend_inside_a_narrow_corridor_is_blocked_not_collided(self):
        # KNOWN LIMITATION of the small planning radius, recorded on purpose. Going straight through a
        # 0.5 m corridor is fine, but the 45 degree turn at its bend needs the full rotation clearance
        # (0.256 m + 0.02 m margin each side) that a 0.5 m corridor does not have. The planner does not
        # know that, so it accepts the trip; the runtime turn guard then stops the robot at the bend,
        # BEFORE it turns. The guard is deliberately conservative: it checks with the 0.02 m margin, and
        # without it this particular turn would just fit physically (it would then be stopped later by the
        # final-yaw guard in the narrow arm). If planning becomes rotation-aware this test must change to
        # "refused up front".
        grid, start, goal = corridor_trip(5)
        costmap = Costmap.from_robot(grid, ROBOT, RADIUS)
        self.assertTrue(plan(costmap, start, goal).success)             # evaluation/planning say reachable
        first, last, sim = drive(self, grid, start, goal, RADIUS)
        self.assertEqual(first.status, NavigationStatus.ACCEPTED)
        self.assertEqual((last.status, last.failure_reason), (NavigationStatus.FAILED, FailureReason.BLOCKED))
        self.assertEqual(sim.collisions, 0)                              # stopped safely, never touched a wall
        self.assertLess(sim.current_pose().y_m, 1.0)                     # stopped at the bend, never entered the vertical arm
        # A 0.6 m corridor has room to turn at the bend: it completes. The default circumscribed disc
        # refuses both narrow corridors at the start and accepts the 0.7 m one.
        grid, start, goal = corridor_trip(6)
        first, last, sim = drive(self, grid, start, goal, RADIUS)
        self.assertEqual((last.status, sim.collisions), (NavigationStatus.REACHED, 0))
        for width, expect_reached in ((5, False), (6, False), (7, True)):
            grid, start, goal = corridor_trip(width)
            first, last, sim = drive(self, grid, start, goal, None)
            with self.subTest(default_radius_corridor_cells=width):
                if expect_reached:
                    self.assertEqual((last.status, sim.collisions), (NavigationStatus.REACHED, 0))
                else:
                    self.assertEqual(first.status, NavigationStatus.FAILED)
                    self.assertEqual(sim.executed_distance_m, 0.0)

    def test_radius_must_lie_between_inscribed_and_circumscribed(self):
        inscribed, circum = Costmap.planning_radius_bounds(ROBOT.footprint_base)
        self.assertAlmostEqual(inscribed, 0.181, places=6)
        self.assertAlmostEqual(circum, 0.181 * 2 ** 0.5, places=6)
        grid, _ = world(7)
        for bad in (inscribed - 0.02, circum + 0.02, 0.0, -1.0):
            with self.subTest(radius=bad), self.assertRaises(ValueError):
                Costmap.from_robot(grid, ROBOT, bad)
            sim = MockSimulator(grid, ROBOT)
            self.addCleanup(sim.close)
            with self.subTest(navigator_radius=bad), self.assertRaises(NavigatorBoundaryError):
                TeamNavigator(sim, planning_radius_m=bad).reset(
                    NavigatorInit('x', ROBOT, ObservationConfig(), NavigationConfig()))
        Costmap.from_robot(grid, ROBOT, inscribed)       # the bounds themselves are allowed
        Costmap.from_robot(grid, ROBOT, circum)


if __name__ == '__main__':
    unittest.main()
