"""Regression tests for two ways B's navigator could get stuck.

1. A scan heading just outside the tolerance (0.0503 rad against 0.05): the proportional turn command is
   too small for a real chassis (stiction), and a robot jittering across the tolerance restarted the stall
   window on every dwell, so nothing ever finished or failed until the operation deadline.
2. A robot parked inside the planning buffer (start margin 0, planning margin 1 cm): its start pose was
   accepted, but the in-place turn and the first drive away were checked at the planning margin, so every
   first move failed and the robot could never leave.
"""

from dataclasses import replace
from math import pi
import unittest

from frontier_search.config import NavigationConfig, ObservationConfig
from frontier_search.contracts import (FailureReason, NavigationRequest, NavigationStatus, NavigatorInit,
                                       OccupancyGrid, ObservationRequest, PolicyName, Pose, Subgoal)
from frontier_search.navigation.costmap import START_ZONE_M, Costmap
from frontier_search.navigation.dev_robot import PLACEHOLDER_ROBOT
from frontier_search.navigation.navigator import MIN_TURN_RADPS, TeamNavigator, _turn_rate
from frontier_search.navigation.planner import path_valid, plan
from frontier_search.simulation.mock_adapter import MockSimulator

DONE = (NavigationStatus.REACHED, NavigationStatus.FAILED, NavigationStatus.CANCELLED)
WALL_EDGE_Y = 0.5          # free room is rows/columns 5..34 of a 0.1 m grid; unknown outside
RADIUS = 0.18              # PLACEHOLDER_ROBOT is a disc of this radius
IN_BUFFER_Y = WALL_EDGE_Y + RADIUS + 0.005   # 5 mm of clearance: inside the 1 cm planning buffer
BUFFER_OPTIONS = dict(planning_margin_m=0.01, start_margin_m=0.0)


def room():
    cells = [[-1] * 40 for _ in range(40)]
    for r in range(5, 35):
        for c in range(5, 35):
            cells[r][c] = 0
    return OccupancyGrid('w', 1, .1, 0, 0, 40, 40, tuple(map(tuple, cells)))


class StictionSimulator(MockSimulator):
    """Chassis that ignores turn commands below ``min_w`` (static friction)."""

    def __init__(self, *args, min_w=0.15, **kwargs):
        super().__init__(*args, **kwargs)
        self._min_w = min_w

    def set_velocity(self, linear_mps, angular_radps):
        super().set_velocity(linear_mps, 0.0 if 0 < abs(angular_radps) < self._min_w else angular_radps)


class JitterSimulator(MockSimulator):
    """Heading that never settles: 3 steps inside the 0.05 tolerance, 3 just outside, repeat."""

    def __init__(self, *args, target_yaw, **kwargs):
        super().__init__(*args, **kwargs)
        self._target, self._n = target_yaw, 0

    def step(self):
        inside = (self._n // 3) % 2 == 0
        p = self._pose
        self._pose = Pose(p.x_m, p.y_m, p.z_m, self._target - (0.0499 if inside else 0.0503))
        self._n += 1
        self._t += self._dt


def make_nav(test, sim, start, obs=None, **options):
    sim.reset('x', 'm', start, 0)
    test.addCleanup(sim.close)
    nav = TeamNavigator(sim, **options)
    nav.reset(NavigatorInit('x', PLACEHOLDER_ROBOT, obs or ObservationConfig(), NavigationConfig()))
    return nav


def run(sim, nav, goal_id, max_s):
    feedback = nav.poll('x', goal_id)
    while feedback.status not in DONE and sim.sim_time_s() < max_s:
        sim.step(); nav.on_step(); feedback = nav.poll('x', goal_id)
    return feedback


class TurnRateTests(unittest.TestCase):
    def test_small_error_gets_the_floor_and_the_right_sign(self):
        self.assertEqual(_turn_rate(0.0503, 1.0), MIN_TURN_RADPS)
        self.assertEqual(_turn_rate(-0.0503, 1.0), -MIN_TURN_RADPS)

    def test_large_error_is_still_proportional_and_capped(self):
        self.assertAlmostEqual(_turn_rate(0.3, 1.0), 0.6)
        self.assertEqual(_turn_rate(2.0, 1.0), 1.0)

    def test_floor_never_exceeds_the_robots_own_limit(self):
        self.assertEqual(_turn_rate(0.06, 0.1), 0.1)


class ScanStopsAtTheToleranceEdgeTests(unittest.TestCase):
    def test_scan_heading_just_outside_tolerance_completes_on_a_chassis_with_stiction(self):
        grid, start = room(), Pose(1.55, 1.55, 0.0, 0.0)
        sim = StictionSimulator(grid, PLACEHOLDER_ROBOT)
        obs = replace(ObservationConfig(), arrival_headings_rad=(0.0503,), dwell_s=0.2)
        nav = make_nav(self, sim, start, obs)
        nav.start_observation(ObservationRequest('x', 'o', grid, start, 'arrival', 0.0))
        feedback = run(sim, nav, 'o', 30.0)
        self.assertEqual(feedback.status, NavigationStatus.REACHED)
        self.assertEqual(len(nav.capture_marks('o')), 1)

    def test_a_heading_that_never_settles_is_reported_blocked_not_left_to_the_deadline(self):
        grid, start = room(), Pose(1.55, 1.55, 0.0, 0.0)
        sim = JitterSimulator(grid, PLACEHOLDER_ROBOT, target_yaw=0.5)
        obs = replace(ObservationConfig(), arrival_headings_rad=(0.5,), dwell_s=0.6, operation_deadline_s=400.0)
        nav = make_nav(self, sim, start, obs)
        nav.start_observation(ObservationRequest('x', 'o', grid, start, 'arrival', 0.0))
        feedback = run(sim, nav, 'o', 60.0)
        self.assertEqual((feedback.status, feedback.failure_reason),
                         (NavigationStatus.FAILED, FailureReason.BLOCKED))
        self.assertLess(sim.sim_time_s(), 15.0)

    def test_a_long_inspection_dwell_is_not_mistaken_for_a_stall(self):
        grid, start = room(), Pose(1.55, 1.55, 0.0, 0.0)
        sim = MockSimulator(grid, PLACEHOLDER_ROBOT)
        obs = replace(ObservationConfig(), arrival_headings_rad=(0.0, 0.0), dwell_s=7.0, operation_deadline_s=60.0)
        nav = make_nav(self, sim, start, obs)
        nav.start_observation(ObservationRequest('x', 'o', grid, start, 'arrival', 0.0))
        self.assertEqual(run(sim, nav, 'o', 60.0).status, NavigationStatus.REACHED)


class StartInsideThePlanningBufferTests(unittest.TestCase):
    def setUp(self):
        self.grid = room()
        self.start = Pose(1.55, IN_BUFFER_Y, 0.0, 0.0)
        self.costmap = Costmap(self.grid, PLACEHOLDER_ROBOT.footprint_base, 0.01, start_margin_m=0.0)

    def test_the_setup_really_is_inside_the_buffer(self):
        self.assertTrue(self.costmap.start_pose_valid(self.start))
        self.assertFalse(self.costmap.pose_valid(self.start))

    def test_first_move_away_from_the_wall_is_allowed(self):
        self.assertTrue(self.costmap.first_move_valid(self.start, 1.55, 1.05))
        self.assertTrue(self.costmap.first_move_valid(self.start, 1.95, 1.05))

    def test_first_move_into_the_wall_is_still_refused(self):
        self.assertFalse(self.costmap.first_move_valid(self.start, 1.55, 0.55))

    def test_default_margins_are_unchanged(self):
        strict = Costmap(self.grid, PLACEHOLDER_ROBOT.footprint_base, 0.01)
        self.assertFalse(strict.start_pose_valid(self.start))
        self.assertFalse(strict.first_move_valid(self.start, 1.55, 1.05))

    def test_zone_ends_where_the_planning_margin_resumes(self):
        # driving ALONG the wall stays inside the buffer, so it is legal only for the first START_ZONE_M
        along = Pose(1.55 + START_ZONE_M, IN_BUFFER_Y, 0.0, 0.0)
        far = Pose(1.55 + 3 * START_ZONE_M, IN_BUFFER_Y, 0.0, 0.0)
        self.assertTrue(self.costmap.segment_valid(self.start, along, START_ZONE_M))
        self.assertFalse(self.costmap.segment_valid(self.start, far, START_ZONE_M))
        self.assertFalse(self.costmap.segment_valid(self.start, along))

    def test_planner_finds_a_path_and_it_stays_valid(self):
        goal = Pose(1.55, 2.05, 0.0, 0.0)
        result = plan(self.costmap, self.start, goal)
        self.assertTrue(result.success, result.failure_reason)
        self.assertTrue(path_valid(self.costmap, (self.start,) + tuple(result.path[1:])))

    def test_navigation_leaves_the_buffer_and_reaches_the_goal(self):
        sim = MockSimulator(self.grid, PLACEHOLDER_ROBOT)
        nav = make_nav(self, sim, self.start, **BUFFER_OPTIONS)
        goal = Pose(1.55, 2.05, 0.0, 0.0)
        first = nav.start(NavigationRequest('x', 'g', self.grid, self.start,
                                            Subgoal('F1', goal, 1, PolicyName.NEAREST)))
        self.assertEqual(first.status, NavigationStatus.ACCEPTED)
        self.assertEqual(run(sim, nav, 'g', 200.0).status, NavigationStatus.REACHED)
        self.assertEqual(sim.collisions, 0)

    def test_scan_in_place_is_not_blocked_by_the_planning_buffer(self):
        sim = MockSimulator(self.grid, PLACEHOLDER_ROBOT)
        nav = make_nav(self, sim, self.start, **BUFFER_OPTIONS)
        feedback = nav.start_observation(ObservationRequest('x', 'o', self.grid, self.start, 'arrival', 0.0))
        self.assertEqual(feedback.status, NavigationStatus.ACCEPTED)
        self.assertEqual(run(sim, nav, 'o', 60.0).status, NavigationStatus.REACHED)
        self.assertEqual(sim.collisions, 0)


if __name__ == '__main__':
    unittest.main()
