"""B's opt-in soft-contact options: start-pose margin, near-obstacle speed limit, contact severity record.

Goal: like a robot vacuum, a light touch is acceptable but a hard collision (contact, then stuck) is not.
All new behaviour is opt-in; the defaults must stay exactly as before. Mock simulator and a fake backend
only: this does not test iGibson physics.
"""

from types import SimpleNamespace
import unittest

from frontier_search.config import NavigationConfig, ObservationConfig
from frontier_search.contracts import (NavigationRequest, NavigationStatus, NavigatorInit, OccupancyGrid,
                                       PolicyName, Point2, Pose, Subgoal)
from frontier_search.fixtures.w2_episode import mock_manifest
from frontier_search.navigation import planner
from frontier_search.navigation.costmap import Costmap
from frontier_search.navigation.dev_robot import PLACEHOLDER_ROBOT
from math import hypot

from frontier_search.navigation.locobot_robot import (LOCOBOT_DEV_ROBOT as ROBOT, LOCOBOT_ROUND_RADIUS_M, LOCOBOT_ROUND_ROBOT,
                                                      ROUND_FOOTPRINT_NAV_OPTIONS, ROUND_FOOTPRINT_OVERRIDE_NAV_OPTIONS,
                                                      SOFT_CONTACT_NAV_OPTIONS, round_footprint)
from frontier_search.navigation.navigator import NearObstacleLimit, NavigatorBoundaryError, TeamNavigator
from frontier_search.simulation.igibson_adapter import AdapterConfig, IGibsonAdapter
from frontier_search.simulation.mock_adapter import MockSimulator

from tests.integration.test_igibson_adapter import FakeBackend, START

try:
    import numpy as np
except ImportError:
    np = None

DONE = (NavigationStatus.REACHED, NavigationStatus.FAILED, NavigationStatus.CANCELLED)


def room():
    """Free room (rows 5..34, columns 5..54, 0.1 m); the unknown row 4 below it blocks, so its edge is y = 0.5 m."""
    cells = [[-1] * 60 for _ in range(40)]
    for r in range(5, 35):
        for c in range(5, 55):
            cells[r][c] = 0
    return OccupancyGrid('w', 1, .1, 0, 0, 60, 40, tuple(map(tuple, cells)))


def wall_east_grid():
    """Free area columns 5..34 (x 0.5..3.5 m); an occupied column starts at x = 3.5 m."""
    cells = [[-1] * 40 for _ in range(40)]
    for r in range(5, 35):
        for c in range(5, 35):
            cells[r][c] = 0
        cells[r][35] = 100
    return OccupancyGrid('w', 1, .1, 0, 0, 40, 40, tuple(map(tuple, cells)))


class StartMarginTests(unittest.TestCase):
    # A pose whose body edge is 1 cm from the wall: free of contact, but inside the 2 cm planning margin.
    POSE = Pose(3.5 - 0.01 - 0.181, 2.0, 0.0, 0.0)

    def test_default_start_check_uses_the_planning_margin_as_before(self):
        costmap = Costmap.from_robot(wall_east_grid(), ROBOT)
        self.assertEqual(costmap.start_margin_m, ROBOT.clearance_margin_m)
        self.assertFalse(costmap.pose_valid(self.POSE))
        self.assertFalse(planner.start_is_usable(costmap, self.POSE))

    def test_zero_start_margin_accepts_a_pose_that_touches_nothing_but_keeps_goal_checks_unchanged(self):
        costmap = Costmap.from_robot(wall_east_grid(), ROBOT, start_margin_m=0.0)
        self.assertTrue(planner.start_is_usable(costmap, self.POSE))
        self.assertFalse(planner.goal_is_valid(costmap, self.POSE))      # goals still need the full margin
        self.assertFalse(costmap.pose_valid(self.POSE))                  # and so do poses on a path

    def test_a_pose_that_really_touches_the_wall_stays_unusable(self):
        costmap = Costmap.from_robot(wall_east_grid(), ROBOT, start_margin_m=0.0)
        touching = Pose(3.5 + 0.005 - 0.181, 2.0, 0.0, 0.0)
        self.assertFalse(planner.start_is_usable(costmap, touching))

    def test_negative_start_margin_is_rejected(self):
        with self.assertRaises(ValueError):
            Costmap.from_robot(wall_east_grid(), ROBOT, start_margin_m=-0.01)
        with self.assertRaises(ValueError):
            TeamNavigator(MockSimulator(wall_east_grid(), ROBOT), start_margin_m=-0.01)


class NearObstacleLimitTests(unittest.TestCase):
    def navigator(self, limit):
        sim = MockSimulator(wall_east_grid(), ROBOT)
        self.addCleanup(sim.close)
        return TeamNavigator(sim, near_obstacle_limit=limit)

    def op(self):
        return SimpleNamespace(costmap=Costmap.from_robot(wall_east_grid(), ROBOT))

    def test_default_is_no_limit(self):
        near = Pose(3.5 - 0.03 - 0.181, 2.0, 0.0, 0.0)
        self.assertEqual(self.navigator(None)._limit_near_obstacle(self.op(), near, 0.3, 1.0), (0.3, 1.0))

    def test_caps_both_speeds_inside_the_zone_and_keeps_the_sign_of_the_turn_rate(self):
        limit = NearObstacleLimit(distance_m=0.06, max_linear_mps=0.08, max_angular_radps=0.6)
        nav, op = self.navigator(limit), self.op()
        near = Pose(3.5 - 0.03 - 0.181, 2.0, 0.0, 0.0)          # body edge 3 cm from the wall
        self.assertEqual(nav._limit_near_obstacle(op, near, 0.3, 1.0), (0.08, 0.6))
        self.assertEqual(nav._limit_near_obstacle(op, near, 0.05, -1.0), (0.05, -0.6))

    def test_open_floor_is_not_slowed(self):
        limit = NearObstacleLimit()
        far = Pose(2.0, 2.0, 0.0, 0.0)
        self.assertEqual(self.navigator(limit)._limit_near_obstacle(self.op(), far, 0.3, 1.0), (0.3, 1.0))

    def test_zone_is_measured_from_the_planning_margin_plus_distance(self):
        # zone = margin 0.02 + distance 0.06 = 8 cm: a body edge 7 cm from the wall is inside, 9 cm is outside
        limit = NearObstacleLimit(distance_m=0.06)
        nav, op = self.navigator(limit), self.op()
        self.assertEqual(nav._limit_near_obstacle(op, Pose(3.5 - 0.07 - 0.181, 2.0, 0, 0), 0.3, 1.0), (0.08, 0.6))
        self.assertEqual(nav._limit_near_obstacle(op, Pose(3.5 - 0.09 - 0.181, 2.0, 0, 0), 0.3, 1.0), (0.3, 1.0))

    def test_invalid_limits_are_rejected(self):
        for bad in (dict(distance_m=-0.1), dict(max_linear_mps=0.0), dict(max_angular_radps=-1.0)):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                NearObstacleLimit(**bad)

    def test_driving_along_a_wall_stays_under_the_cap_only_when_enabled(self):
        # The robot's own max speed is 0.1 m/s; the cap is 0.08 m/s. A path 3 cm from the wall edge needs the
        # inscribed planning radius (the default circumscribed disc would refuse it). The start is aligned with the
        # path: a turn this close to the wall is refused by the existing first-move guard.
        inscribed, _ = Costmap.planning_radius_bounds(ROBOT.footprint_base)
        grid = room()
        start, goal = Pose(1.55, 0.75, 0, 0), Pose(4.45, 0.75, 0, 0)     # body edge 6.9 cm from the wall edge at y 0.5

        def run(limit):
            sim = MockSimulator(grid, ROBOT)
            self.addCleanup(sim.close)
            sim.reset('x', 'm', start, 0)
            nav = TeamNavigator(sim, planning_radius_m=inscribed, near_obstacle_limit=limit)
            nav.reset(NavigatorInit('x', ROBOT, ObservationConfig(), NavigationConfig(navigation_deadline_s=600)))
            fb = nav.start(NavigationRequest('x', 'g', grid, start, Subgoal('F1', goal, 1, PolicyName.NEAREST)))
            self.assertEqual(fb.status, NavigationStatus.ACCEPTED)
            fastest = 0.0
            for _ in range(14000):
                if fb.status in DONE:
                    break
                sim.step(); nav.on_step(); fb = nav.poll('x', 'g')
                fastest = max(fastest, abs(sim._v))
            return fb.status, sim.collisions, fastest

        status, collisions, fastest = run(None)
        self.assertEqual((status, collisions), (NavigationStatus.REACHED, 0))
        self.assertGreater(fastest, 0.09)                                # default: full speed next to the wall
        status, collisions, fastest = run(NearObstacleLimit())
        self.assertEqual((status, collisions), (NavigationStatus.REACHED, 0))
        self.assertLessEqual(fastest, 0.08 + 1e-9)


def corner_grid():
    """Free 2 m x 2 m area with one occupied cell at x,y 1.0..1.1 m (a wall-stub corner at (1.0, 1.0))."""
    cells = [[0] * 20 for _ in range(20)]
    cells[10][10] = 100
    return OccupancyGrid('w', 1, .1, 0, 0, 20, 20, tuple(map(tuple, cells)))


class PlanningMarginTests(unittest.TestCase):
    """The cell whose centre is 0.15 m from the corner in x and y (0.212 m away) is the real benchmark's
    left-room entrance geometry: it is refused with the 2 cm margin and accepted with 1 cm."""
    CELL = (8, 8)

    def costmap(self, margin_m=None):
        inscribed, _ = Costmap.planning_radius_bounds(ROBOT.footprint_base)
        return Costmap.from_robot(corner_grid(), ROBOT, inscribed, margin_m=margin_m)

    def test_default_margin_is_the_robots_configured_margin(self):
        self.assertEqual(self.costmap().margin_m, ROBOT.clearance_margin_m)
        self.assertFalse(self.costmap().cell_traversable(*self.CELL))

    def test_a_smaller_planning_margin_makes_that_cell_traversable(self):
        self.assertEqual(self.costmap(0.01).margin_m, 0.01)
        self.assertTrue(self.costmap(0.01).cell_traversable(*self.CELL))

    def test_the_navigator_passes_the_margin_override_to_its_costmap(self):
        inscribed, _ = Costmap.planning_radius_bounds(ROBOT.footprint_base)
        sim = MockSimulator(corner_grid(), ROBOT)
        self.addCleanup(sim.close)
        nav = TeamNavigator(sim, planning_radius_m=inscribed, planning_margin_m=0.01)
        nav.reset(NavigatorInit('x', ROBOT, ObservationConfig(), NavigationConfig()))
        costmap = nav._costmap_for(corner_grid())
        self.assertEqual(costmap.margin_m, 0.01)
        self.assertTrue(costmap.cell_traversable(*self.CELL))

    def test_negative_planning_margin_is_rejected(self):
        with self.assertRaises(ValueError):
            TeamNavigator(MockSimulator(room(), ROBOT), planning_margin_m=-0.01)


class RoundFootprintTests(unittest.TestCase):
    def test_vertices_lie_on_the_measured_radius_and_contain_the_inscribed_circle(self):
        for p in round_footprint():
            self.assertAlmostEqual(hypot(p.x_m, p.y_m), LOCOBOT_ROUND_RADIUS_M, places=9)
        low, high = Costmap.planning_radius_bounds(LOCOBOT_ROUND_ROBOT.footprint_base)
        self.assertAlmostEqual(high, LOCOBOT_ROUND_RADIUS_M, places=9)
        self.assertGreater(low, 0.185)                                   # a 16-gon is nearly a disc

    def test_invalid_round_footprints_are_rejected(self):
        for bad in (dict(radius_m=0.0), dict(sides=4)):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                round_footprint(**bad)

    def test_an_in_place_turn_next_to_a_wall_is_valid_for_the_round_body_but_not_for_the_square_one(self):
        # body centre 0.25 m from the wall edge: the square's corners (0.257 m) sweep into the wall, the round body's do not
        grid = wall_east_grid()
        pose = Pose(3.5 - 0.25, 2.0, 0.0, 0.0)
        square = Costmap.from_robot(grid, ROBOT, margin_m=0.0)
        round_ = Costmap.from_robot(grid, LOCOBOT_ROUND_ROBOT, margin_m=0.0)
        self.assertFalse(square.full_rotation_valid(pose))
        self.assertTrue(round_.full_rotation_valid(pose))

    def test_options_build_a_valid_navigator_for_the_round_robot(self):
        sim = MockSimulator(room(), LOCOBOT_ROUND_ROBOT)
        self.addCleanup(sim.close)
        nav = TeamNavigator(sim, **ROUND_FOOTPRINT_NAV_OPTIONS)
        nav.reset(NavigatorInit('x', LOCOBOT_ROUND_ROBOT, ObservationConfig(), NavigationConfig()))


class FootprintOverrideTests(unittest.TestCase):
    """B replaces the footprint internally; the NavigatorInit robot (the shared manifest's square) is left as it is."""

    def navigator(self, **options):
        sim = MockSimulator(wall_east_grid(), ROBOT)
        self.addCleanup(sim.close)
        nav = TeamNavigator(sim, **options)
        nav.reset(NavigatorInit('x', ROBOT, ObservationConfig(), NavigationConfig(navigation_deadline_s=600)))
        return nav, sim

    def test_default_keeps_the_manifest_footprint(self):
        nav, _ = self.navigator()
        self.assertEqual(nav._costmap_for(wall_east_grid()).footprint_base, ROBOT.footprint_base)

    def test_override_replaces_the_footprint_in_the_costmap(self):
        nav, _ = self.navigator(**ROUND_FOOTPRINT_OVERRIDE_NAV_OPTIONS)
        costmap = nav._costmap_for(wall_east_grid())
        self.assertEqual(costmap.footprint_base, round_footprint())
        self.assertEqual(costmap.margin_m, 0.01)

    def test_a_turn_next_to_a_wall_is_valid_only_with_the_override(self):
        pose = Pose(3.5 - 0.25, 2.0, 0.0, 0.0)
        default, _ = self.navigator(planning_margin_m=0.0)
        override, _ = self.navigator(footprint_override=round_footprint(), planning_margin_m=0.0)
        self.assertFalse(default._costmap_for(wall_east_grid()).full_rotation_valid(pose))
        self.assertTrue(override._costmap_for(wall_east_grid()).full_rotation_valid(pose))

    def test_the_planner_refuses_a_turning_start_beside_a_wall_unless_the_override_is_on(self):
        grid = wall_east_grid()
        start, goal = Pose(3.5 - 0.25, 2.0, 0.0, 0.0), Pose(2.0, 2.0, 0.0, 0.0)       # must turn through 180 degrees
        for options, expected in (({}, NavigationStatus.FAILED), (dict(footprint_override=round_footprint(), planning_margin_m=0.0), NavigationStatus.ACCEPTED)):
            with self.subTest(options=sorted(options)):
                nav, sim = self.navigator(**options)
                sim.reset('x', 'm', start, 0)
                fb = nav.start(NavigationRequest('x', 'g', grid, start, Subgoal('F1', goal, 1, PolicyName.NEAREST)))
                self.assertEqual(fb.status, expected)

    def test_planning_radius_is_checked_against_the_overriding_footprint(self):
        sim = MockSimulator(room(), ROBOT)
        self.addCleanup(sim.close)
        nav = TeamNavigator(sim, footprint_override=round_footprint(), planning_radius_m=0.10)
        with self.assertRaises(NavigatorBoundaryError):
            nav.reset(NavigatorInit('x', ROBOT, ObservationConfig(), NavigationConfig()))

    def test_degenerate_overrides_are_rejected(self):
        sim = MockSimulator(room(), ROBOT)
        self.addCleanup(sim.close)
        for bad in ((Point2(0, 0), Point2(1, 0)), (Point2(0, 0), Point2(1, 0), Point2(2, 0))):
            with self.subTest(vertices=len(bad)), self.assertRaises(ValueError):
                TeamNavigator(sim, footprint_override=bad)

    def test_the_shared_robot_configuration_is_not_modified(self):
        before = ROBOT.footprint_base
        self.navigator(**ROUND_FOOTPRINT_OVERRIDE_NAV_OPTIONS)
        self.assertEqual(ROBOT.footprint_base, before)


class PresetTests(unittest.TestCase):
    def test_preset_builds_a_valid_navigator_for_the_development_robot(self):
        sim = MockSimulator(room(), ROBOT)
        self.addCleanup(sim.close)
        nav = TeamNavigator(sim, **SOFT_CONTACT_NAV_OPTIONS)
        nav.reset(NavigatorInit('x', ROBOT, ObservationConfig(), NavigationConfig()))   # radius within bounds

    def test_preset_does_not_turn_on_the_speed_cap(self):
        self.assertNotIn('near_obstacle_limit', SOFT_CONTACT_NAV_OPTIONS)

    def test_preset_radius_outside_the_robot_bounds_is_still_refused(self):
        sim = MockSimulator(room(), ROBOT)
        self.addCleanup(sim.close)
        nav = TeamNavigator(sim, planning_radius_m=0.10)
        with self.assertRaises(NavigatorBoundaryError):
            nav.reset(NavigatorInit('x', ROBOT, ObservationConfig(), NavigationConfig()))


@unittest.skipIf(np is None, "capture conversion requires NumPy")
class ContactSeverityTests(unittest.TestCase):
    def make(self, **backend_attrs):
        cls = type("B", (FakeBackend,), backend_attrs)
        manifest = mock_manifest()
        adapter = IGibsonAdapter(PLACEHOLDER_ROBOT, manifest.mapping, AdapterConfig(), lambda cfg: cls(cfg))
        self.addCleanup(adapter.close)
        return adapter

    def drive_into_box(self, adapter, steps=60):
        adapter.reset("ep", "Rs_int", START, 7)
        adapter.set_velocity(0.5, 0.0)
        for _ in range(steps):
            adapter.step()

    def test_peak_force_and_longest_run_are_recorded_next_to_the_unchanged_step_counter(self):
        forces = iter([0.4, 2.5, 1.0] + [0.7] * 1000)
        adapter = self.make(boxes=((1.20, 1.4, 1.5, 1.7, 0.0, 0.5),),
                            peak_contact_force=lambda self, base_z, min_h: next(forces))
        self.drive_into_box(adapter)
        self.assertGreater(adapter.collisions, 3)
        self.assertEqual(adapter.peak_contact_force_n, 2.5)
        self.assertEqual(adapter.longest_contact_run_steps, adapter.collisions)   # one unbroken contact run

    def test_backend_without_force_support_still_counts_steps_and_reports_zero_force(self):
        adapter = self.make(boxes=((1.20, 1.4, 1.5, 1.7, 0.0, 0.5),))
        self.drive_into_box(adapter)
        self.assertGreater(adapter.collisions, 0)
        self.assertEqual(adapter.peak_contact_force_n, 0.0)

    def test_reset_clears_the_severity_record(self):
        adapter = self.make(boxes=((1.20, 1.4, 1.5, 1.7, 0.0, 0.5),),
                            peak_contact_force=lambda self, base_z, min_h: 3.0)
        self.drive_into_box(adapter)
        self.assertEqual(adapter.peak_contact_force_n, 3.0)
        adapter.reset("ep2", "Rs_int", START, 7)
        self.assertEqual((adapter.peak_contact_force_n, adapter.longest_contact_run_steps, adapter.collisions), (0.0, 0, 0))

    def test_runs_are_separated_when_contact_stops(self):
        adapter = self.make(boxes=((1.20, 1.4, 1.5, 1.7, 0.0, 0.5),),
                            peak_contact_force=lambda self, base_z, min_h: 1.0)
        self.drive_into_box(adapter, steps=60)
        first_run = adapter.longest_contact_run_steps
        adapter.set_velocity(-0.5, 0.0)
        for _ in range(60):
            adapter.step()
        adapter.set_velocity(0.5, 0.0)
        for _ in range(60):
            adapter.step()
        self.assertGreaterEqual(adapter.longest_contact_run_steps, first_run)
        self.assertLess(adapter.longest_contact_run_steps, adapter.collisions)   # contact happened in two runs


if __name__ == '__main__':
    unittest.main()
