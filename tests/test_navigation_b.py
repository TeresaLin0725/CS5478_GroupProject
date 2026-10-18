"""B (Navigation & Planning) checks: geometry, costmap, planner, navigator, mock simulator.

Uses only the synthetic navigation fixtures and the in-memory mock simulator. The
planning/execution checks follow Interface Freeze section 7 (B's share).
"""

from dataclasses import replace
from math import hypot, pi
import unittest

from frontier_search.config import NavigationConfig, ObservationConfig
from frontier_search.contracts import (
    CandidateEvaluationRequest, FailureReason, GridCell, NavigationRequest, NavigationStatus,
    NavigatorInit, ObservationRequest, PolicyName, Point2, Pose, Subgoal,
)
from frontier_search.fixtures.loader import load_navigation_fixture, load_policy_fixture
from frontier_search.navigation import planner
from frontier_search.navigation.costmap import Costmap
from frontier_search.navigation.dev_robot import PLACEHOLDER_ROBOT as ROBOT
from frontier_search.navigation.geometry import (
    cell_center, circle_footprint, footprint_collides, world_to_cell,
)
from frontier_search.navigation.navigator import NavigatorBoundaryError, STALL_WINDOW_S, TeamNavigator
from frontier_search.simulation.mock_adapter import MockSimulator
from frontier_search.validation import validate_evaluation

TERMINAL = (NavigationStatus.REACHED, NavigationStatus.FAILED, NavigationStatus.CANCELLED)


def costmap(name):
    fixture = load_navigation_fixture(name)
    return fixture, Costmap.from_robot(fixture.grid, ROBOT)


def make_navigator(fixture, start=None, **nav_overrides):
    sim = MockSimulator(fixture.grid, ROBOT)
    sim.reset('E', 'mock', start or fixture.start_pose, 0)
    navigator = TeamNavigator(sim)
    navigator.reset(NavigatorInit('E', ROBOT, ObservationConfig(), NavigationConfig(**nav_overrides)))
    return sim, navigator


def goal_of(fixture):
    return Subgoal('F1', fixture.goal_pose, 1, PolicyName.NEAREST)


def run_until_terminal(sim, navigator, goal_id, max_steps=3000, hook=None):
    for i in range(max_steps):
        sim.step()
        navigator.on_step()
        if hook is not None:
            hook(i)
        feedback = navigator.poll('E', goal_id)
        if feedback.status in TERMINAL:
            return feedback
    return feedback


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.grid = load_navigation_fixture('open_space').grid

    def test_cell_centre_and_world_to_cell_follow_contract(self):
        self.assertEqual(cell_center(self.grid, 0, 0).x_m, 0.05)
        self.assertEqual(world_to_cell(self.grid, 1.05, 1.05), GridCell(10, 10))
        self.assertIsNone(world_to_cell(self.grid, -0.01, 1.0))
        self.assertIsNone(world_to_cell(self.grid, 4.0, 1.0))  # upper bound is exclusive

    def test_circle_footprint_contains_the_true_disc(self):
        radius = 0.2
        polygon = circle_footprint(radius)
        self.assertTrue(all(hypot(p.x_m, p.y_m) >= radius for p in polygon))

    def test_collision_treats_unknown_and_out_of_bounds_as_blocked(self):
        footprint = circle_footprint(0.2)
        self.assertFalse(footprint_collides(self.grid, footprint, Pose(2.0, 2.0, 0.0, 0.0), 0.05))
        self.assertTrue(footprint_collides(self.grid, footprint, Pose(0.55, 2.0, 0.0, 0.0), 0.0))  # touches unknown
        self.assertTrue(footprint_collides(self.grid, footprint, Pose(-1.0, -1.0, 0.0, 0.0), 0.0))

    def test_non_circular_footprint_depends_on_yaw(self):
        from frontier_search.contracts import Point2
        bar = (Point2(-0.4, -0.05), Point2(0.4, -0.05), Point2(0.4, 0.05), Point2(-0.4, 0.05))
        # 0.7 m from the room edge: a long bar fits when pointing along x, not along y.
        self.assertFalse(footprint_collides(self.grid, bar, Pose(0.95, 2.0, 0.0, pi / 2), 0.0))
        self.assertTrue(footprint_collides(self.grid, bar, Pose(0.8, 2.0, 0.0, 0.0), 0.0))


class CostmapTests(unittest.TestCase):
    def test_inflation_blocks_cells_near_walls_and_unknown(self):
        _, cm = costmap('open_space')
        self.assertTrue(cm.cell_traversable(20, 20))
        self.assertFalse(cm.cell_traversable(5, 5))     # touches the unknown border
        self.assertFalse(cm.cell_traversable(-1, 0))    # out of bounds

    def test_doorway_passable_only_when_robot_fits(self):
        _, wide = costmap('wall_detour')
        _, narrow = costmap('narrow_doorway')
        self.assertTrue(wide.cell_traversable(20, 20))
        self.assertFalse(narrow.cell_traversable(20, 20))

    def test_corner_cutting_needs_both_flanking_cells(self):
        _, cm = costmap('wall_detour')
        # (18,19) is next to the doorway; a diagonal across the wall corner must be refused
        # whenever a flanking cell is blocked.
        for d_row, d_col in ((1, 1), (-1, 1), (1, -1), (-1, -1)):
            for row in range(5, 35):
                for column in range(5, 35):
                    if cm.cell_traversable(row, column) and cm.cell_traversable(row + d_row, column + d_col) \
                            and cm.diagonal_ok(row, column, d_row, d_col):
                        self.assertTrue(cm.cell_traversable(row + d_row, column))
                        self.assertTrue(cm.cell_traversable(row, column + d_col))

    def test_segment_through_wall_rejected_and_through_door_accepted(self):
        _, cm = costmap('wall_detour')
        self.assertFalse(cm.segment_valid(Pose(1.05, 1.05, 0, 0), Pose(3.05, 1.05, 0, 0)))
        self.assertTrue(cm.segment_valid(Pose(1.05, 2.05, 0, 0), Pose(3.05, 2.05, 0, 0)))

    def test_rotation_safety(self):
        _, cm = costmap('wall_detour')
        self.assertTrue(cm.full_rotation_valid(Pose(1.05, 1.05, 0, 0)))
        self.assertFalse(cm.full_rotation_valid(Pose(1.85, 1.05, 0, 0)))  # wall within reach


class PlannerTests(unittest.TestCase):
    def test_open_space_is_straight_line(self):
        fixture, cm = costmap('open_space')
        result = planner.plan(cm, fixture.start_pose, fixture.goal_pose)
        self.assertTrue(result.success)
        self.assertAlmostEqual(result.path_cost_m, 2.0, places=6)
        self.assertEqual(result.path[0], fixture.start_pose)
        self.assertEqual(result.path[-1], fixture.goal_pose)
        self.assertTrue(planner.path_valid(cm, result.path))

    def test_wall_detour_goes_through_the_door_and_is_longer(self):
        fixture, cm = costmap('wall_detour')
        result = planner.plan(cm, fixture.start_pose, fixture.goal_pose)
        self.assertTrue(result.success)
        self.assertGreater(result.path_cost_m, 2.0)
        self.assertTrue(planner.path_valid(cm, result.path))

    def test_failure_reasons(self):
        for name in ('narrow_doorway', 'unknown_barrier'):
            fixture, cm = costmap(name)
            result = planner.plan(cm, fixture.start_pose, fixture.goal_pose)
            self.assertEqual((result.success, result.failure_reason, result.path, result.path_cost_m),
                             (False, FailureReason.NO_PATH, None, None))
        fixture, cm = costmap('open_space')
        for goal in (Pose(0.1, 0.1, 0, 0), Pose(9.0, 9.0, 0, 0)):  # unknown / off grid
            self.assertEqual(planner.plan(cm, fixture.start_pose, goal).failure_reason,
                             FailureReason.INVALID_GOAL)
        self.assertEqual(planner.plan(cm, Pose(0.1, 0.1, 0, 0), fixture.goal_pose).failure_reason,
                         FailureReason.NO_PATH)  # start inside blocked space

    def test_distance_field_matches_astar_cost(self):
        for name in ('open_space', 'wall_detour'):
            fixture, cm = costmap(name)
            result = planner.plan(cm, fixture.start_pose, fixture.goal_pose)
            field, complete = planner.distance_field(cm, fixture.start_pose)
            self.assertTrue(complete)
            self.assertAlmostEqual(planner.cost_to_goal_m(cm, fixture.start_pose, fixture.goal_pose, field),
                                   result.path_cost_m, places=9)

    def test_planning_timeout_is_not_no_path(self):
        fixture, cm = costmap('wall_detour')  # needs well over 64 expansions, the check interval
        ticks = iter(range(10 ** 6))
        result = planner.plan(cm, fixture.start_pose, fixture.goal_pose, deadline_s=0.5,
                              clock=lambda: float(next(ticks)))
        self.assertEqual(result.failure_reason, FailureReason.TIMEOUT)

    def test_octile_heuristic_is_admissible(self):
        fixture, cm = costmap('open_space')
        res = cm.grid.resolution_m
        field, _ = planner.distance_field(cm, fixture.start_pose)
        origin = (10, 10)
        for cell, cost in field.items():
            self.assertLessEqual(planner.octile_distance_m(origin, cell, res), cost + 1e-9)


class NavigatorTests(unittest.TestCase):
    def evaluate_with_poses(self, clock=None, **nav):
        policy = load_policy_fixture('selection')
        state = policy.state
        # Test-local reachable approach poses (shared fixture file is not modified).
        frontiers = tuple(replace(f, approach_pose=Pose(1.05 + 2.0 * i, y, 0.0, pi))
                          for i, (f, y) in enumerate(zip(state.frontiers, (1.05, 1.05, 2.0))))
        state = replace(state, frontiers=frontiers)
        sim = MockSimulator(state.occupancy_map, ROBOT)
        sim.reset(state.episode_id, 'mock', state.robot_pose, 0)
        navigator = TeamNavigator(sim, **({'wall_clock': clock} if clock else {}))
        navigator.reset(NavigatorInit(state.episode_id, ROBOT, ObservationConfig(), NavigationConfig(**nav)))
        evaluation = navigator.evaluate_candidates(CandidateEvaluationRequest(state.episode_id, state))
        return state, evaluation

    def test_evaluation_is_contract_valid_and_read_only(self):
        state, evaluation = self.evaluate_with_poses()
        validate_evaluation(state, evaluation)
        self.assertEqual(evaluation.state_revision, state.revision)
        self.assertEqual(evaluation.start_pose, state.robot_pose)
        by_id = {a.frontier_id: a for a in evaluation.assessments}
        self.assertTrue(by_id['F1'].reachable)
        self.assertAlmostEqual(by_id['F1'].path_cost_m, 0.0, places=6)  # start == F1 approach

    def test_evaluation_marks_unsafe_approach_poses_invalid_goal(self):
        policy = load_policy_fixture('selection')
        sim = MockSimulator(policy.state.occupancy_map, ROBOT)
        sim.reset(policy.state.episode_id, 'mock', policy.state.robot_pose, 0)
        navigator = TeamNavigator(sim)
        navigator.reset(NavigatorInit(policy.state.episode_id, ROBOT, ObservationConfig(), NavigationConfig()))
        evaluation = navigator.evaluate_candidates(
            CandidateEvaluationRequest(policy.state.episode_id, policy.state))
        validate_evaluation(policy.state, evaluation)
        self.assertTrue(all(a.reachable is False and a.failure_reason == FailureReason.INVALID_GOAL
                            for a in evaluation.assessments))

    def test_evaluation_timeout_is_inconclusive_not_unreachable(self):
        ticks = iter(range(10 ** 6))
        state, evaluation = self.evaluate_with_poses(clock=lambda: float(next(ticks)), batch_deadline_s=0.5)
        validate_evaluation(state, evaluation)
        far = [a for a in evaluation.assessments if a.frontier_id == 'F2'][0]
        self.assertIsNone(far.reachable)
        self.assertEqual(far.failure_reason, FailureReason.TIMEOUT)
        self.assertIsNone(far.path_cost_m)

    def test_detour_is_executed_and_terminal_outcome_is_latched(self):
        fixture = load_navigation_fixture('wall_detour')
        sim, navigator = make_navigator(fixture)
        accepted = navigator.start(NavigationRequest('E', 'g1', fixture.grid, fixture.start_pose, goal_of(fixture)))
        self.assertEqual(accepted.status, NavigationStatus.ACCEPTED)
        self.assertTrue(accepted.planner_result.success)
        feedback = run_until_terminal(sim, navigator, 'g1')
        self.assertEqual(feedback.status, NavigationStatus.REACHED)
        self.assertIsNone(feedback.failure_reason)
        self.assertLessEqual(hypot(sim.current_pose().x_m - fixture.goal_pose.x_m,
                                   sim.current_pose().y_m - fixture.goal_pose.y_m), ROBOT.position_tolerance_m)
        self.assertLessEqual(abs(sim.current_pose().yaw_rad - fixture.goal_pose.yaw_rad), ROBOT.yaw_tolerance_rad)
        self.assertEqual(sim.collisions, 0)
        self.assertGreaterEqual(sim.executed_distance_m, feedback.planner_result.path_cost_m * 0.9)  # actual vs planned
        self.assertIs(navigator.poll('E', 'g1'), feedback)
        self.assertIs(navigator.cancel('E', 'g1', 'late'), feedback)  # terminal wins; idempotent

    def test_poll_never_advances_simulation(self):
        fixture = load_navigation_fixture('open_space')
        sim, navigator = make_navigator(fixture)
        navigator.start(NavigationRequest('E', 'g1', fixture.grid, fixture.start_pose, goal_of(fixture)))
        before = sim.sim_time_s()
        for _ in range(5):
            navigator.poll('E', 'g1')
        self.assertEqual(sim.sim_time_s(), before)

    def test_boundary_errors(self):
        fixture = load_navigation_fixture('open_space')
        sim, navigator = make_navigator(fixture)
        navigator.start(NavigationRequest('E', 'g1', fixture.grid, fixture.start_pose, goal_of(fixture)))
        request = NavigationRequest('E', 'g2', fixture.grid, fixture.start_pose, goal_of(fixture))
        with self.assertRaises(NavigatorBoundaryError):
            navigator.start(request)                                  # overlapping operation
        with self.assertRaises(NavigatorBoundaryError):
            navigator.poll('WRONG', 'g1')                             # wrong episode
        with self.assertRaises(NavigatorBoundaryError):
            navigator.reset(NavigatorInit('E', ROBOT, ObservationConfig(), NavigationConfig()))  # active op
        navigator.cancel('E', 'g1', 'test')
        with self.assertRaises(NavigatorBoundaryError):
            navigator.start(replace(request, goal_id='g1'))           # reused ID
        with self.assertRaises(NavigatorBoundaryError):
            navigator.poll('E', 'nope')

    def test_cancellation_is_idempotent_and_works_before_a_plan_exists(self):
        fixture = load_navigation_fixture('open_space')
        sim, navigator = make_navigator(fixture)
        early = navigator.cancel('E', 'never-started', 'preempt')
        self.assertEqual(early.status, NavigationStatus.CANCELLED)
        self.assertIsNone(early.planner_result)
        self.assertIs(navigator.cancel('E', 'never-started', 'again'), early)
        navigator.start(NavigationRequest('E', 'g1', fixture.grid, fixture.start_pose, goal_of(fixture)))
        run_for = [sim.step() or navigator.on_step() for _ in range(10)]
        first = navigator.cancel('E', 'g1', 'claim')
        self.assertEqual(first.status, NavigationStatus.CANCELLED)
        self.assertIsNone(first.failure_reason)  # cancellation is not a failed frontier
        self.assertIs(navigator.cancel('E', 'g1', 'again'), first)
        pose = sim.current_pose()
        sim.step()
        self.assertEqual(sim.current_pose(), pose)  # motion stopped

    def test_planning_failures_are_reported_immediately(self):
        for name in ('narrow_doorway', 'unknown_barrier'):
            fixture = load_navigation_fixture(name)
            sim, navigator = make_navigator(fixture)
            feedback = navigator.start(NavigationRequest('E', 'g', fixture.grid, fixture.start_pose, goal_of(fixture)))
            self.assertEqual((feedback.status, feedback.failure_reason),
                             (NavigationStatus.FAILED, FailureReason.NO_PATH))
            self.assertFalse(feedback.planner_result.success)

    def test_stale_start_pose_fails_at_start(self):
        fixture = load_navigation_fixture('open_space')
        sim, navigator = make_navigator(fixture)
        feedback = navigator.start(NavigationRequest('E', 'g', fixture.grid, Pose(1.5, 1.5, 0.0, 0.0),
                                                     goal_of(fixture)))
        self.assertEqual(feedback.status, NavigationStatus.FAILED)
        self.assertIsNotNone(feedback.failure_reason)

    def test_execution_can_fail_after_a_successful_plan(self):
        fixture = load_navigation_fixture('wall_detour')
        sim, navigator = make_navigator(fixture, navigation_deadline_s=2.0)
        navigator.start(NavigationRequest('E', 'g', fixture.grid, fixture.start_pose, goal_of(fixture)))
        feedback = run_until_terminal(sim, navigator, 'g')
        self.assertEqual((feedback.status, feedback.failure_reason),
                         (NavigationStatus.FAILED, FailureReason.TIMEOUT))
        self.assertTrue(feedback.planner_result.success)  # planning and execution are separate outcomes

    def test_newer_blocking_map_stops_and_replans_to_same_goal(self):
        fixture = load_navigation_fixture('blocked_update')
        sim, navigator = make_navigator(fixture)
        navigator.start(NavigationRequest('E', 'g', fixture.grid, fixture.start_pose, goal_of(fixture)))
        navigator.update_map('E', 'g', fixture.grid)  # equal revision: no-op
        self.assertFalse(any(kind == 'replan' for kind, _ in navigator.events))
        feedback = run_until_terminal(
            sim, navigator, 'g', hook=lambda i: navigator.update_map('E', 'g', fixture.updated_grid) if i == 20 else None)
        self.assertEqual((feedback.status, feedback.failure_reason),
                         (NavigationStatus.FAILED, FailureReason.NO_PATH))
        self.assertEqual(sum(1 for kind, _ in navigator.events if kind == 'replan'), 1)
        pose = sim.current_pose()
        sim.step()
        self.assertEqual(sim.current_pose(), pose)  # stopped

    def test_map_geometry_change_is_rejected(self):
        fixture = load_navigation_fixture('open_space')
        sim, navigator = make_navigator(fixture)
        navigator.start(NavigationRequest('E', 'g', fixture.grid, fixture.start_pose, goal_of(fixture)))
        with self.assertRaises(NavigatorBoundaryError):
            navigator.update_map('E', 'g', replace(fixture.grid, revision=2, resolution_m=0.2))

    def test_observation_sweep_reaches_every_heading_and_marks_frames(self):
        fixture = load_navigation_fixture('open_space')
        start = Pose(2.05, 2.05, 0.0, 0.0)
        sim, navigator = make_navigator(fixture, start=start)
        accepted = navigator.start_observation(ObservationRequest('E', 'o', fixture.grid, start, 'arrival', 1.0))
        self.assertEqual(accepted.status, NavigationStatus.ACCEPTED)
        self.assertIsNone(accepted.planner_result)
        feedback = run_until_terminal(sim, navigator, 'o')
        self.assertEqual(feedback.status, NavigationStatus.REACHED)
        self.assertEqual(len(navigator.capture_marks('o')), len(ObservationConfig().arrival_headings_rad))
        self.assertLess(hypot(sim.current_pose().x_m - start.x_m, sim.current_pose().y_m - start.y_m), 1e-6)

    def test_unsafe_observation_fails_instead_of_rotating_blindly(self):
        fixture = load_navigation_fixture('open_space')
        near_unknown = Pose(0.6, 2.05, 0.0, 0.0)
        for routine in ('arrival','inspection'):
            with self.subTest(routine=routine):
                sim, navigator = make_navigator(fixture, start=near_unknown)
                feedback = navigator.start_observation(
                    ObservationRequest('E', 'o', fixture.grid, near_unknown, routine, 0.0))
                self.assertEqual((feedback.status, feedback.failure_reason),
                                 (NavigationStatus.FAILED, FailureReason.BLOCKED))

    def test_observation_timeout_is_bounded(self):
        fixture = load_navigation_fixture('open_space')
        start = Pose(2.05, 2.05, 0.0, 0.0)
        sim = MockSimulator(fixture.grid, ROBOT)
        sim.reset('E', 'mock', start, 0)
        navigator = TeamNavigator(sim)
        navigator.reset(NavigatorInit('E', ROBOT, replace(ObservationConfig(), operation_deadline_s=0.5),
                                      NavigationConfig()))
        navigator.start_observation(ObservationRequest('E', 'o', fixture.grid, start, 'arrival', 1.0))
        feedback = run_until_terminal(sim, navigator, 'o')
        self.assertEqual((feedback.status, feedback.failure_reason),
                         (NavigationStatus.FAILED, FailureReason.TIMEOUT))


class MockSimulatorTests(unittest.TestCase):
    def setUp(self):
        self.fixture = load_navigation_fixture('open_space')
        self.sim = MockSimulator(self.fixture.grid, ROBOT)
        self.sim.reset('E', 'mock', Pose(2.0, 2.0, 0.0, 0.0), 0)

    def test_unicycle_motion_and_speed_limits(self):
        self.sim.set_velocity(10.0, 0.0)  # clipped to the robot's limit
        for _ in range(20):
            self.sim.step()
        self.assertAlmostEqual(self.sim.current_pose().x_m, 2.0 + ROBOT.max_linear_speed_mps * 1.0, places=6)
        self.assertAlmostEqual(self.sim.executed_distance_m, ROBOT.max_linear_speed_mps * 1.0, places=6)

    def test_physical_collision_blocks_motion_and_is_counted(self):
        self.sim.set_velocity(0.5, 0.0)
        for _ in range(200):
            self.sim.step()
        self.assertGreater(self.sim.collisions, 0)
        self.assertLess(self.sim.current_pose().x_m, 3.6)  # stopped before the unknown border

    def test_spawn_clearance_is_verified_or_rejected(self):
        cells = self.sim.verify_spawn_clearance()
        self.assertTrue(cells)
        self.sim.reset('E', 'mock', Pose(0.1, 0.1, 0.0, 0.0), 0)
        with self.assertRaises(ValueError):
            self.sim.verify_spawn_clearance()

    def test_capture_sequences_increase_and_pause_blocks_stepping(self):
        first, second = self.sim.capture(), self.sim.capture()
        self.assertEqual((first.sequence, second.sequence), (1, 2))
        self.sim.pause()
        with self.assertRaises(RuntimeError):
            self.sim.step()
        self.sim.resume()
        self.sim.step()
        self.assertGreater(self.sim.sim_time_s(), 0.0)


RECTANGLE = replace(ROBOT, footprint_base=(Point2(-0.3, -0.15), Point2(0.3, -0.15),
                                           Point2(0.3, 0.15), Point2(-0.3, 0.15)))  # 0.6 x 0.3 m bar


class CostmapSpeedupTests(unittest.TestCase):
    def test_clearance_fast_path_matches_exact_collision_test(self):
        """The shortcut is only a proof of safety: results must equal brute force."""
        for name in ('open_space', 'wall_detour', 'narrow_doorway', 'unknown_barrier', 'blocked_update'):
            fixture = load_navigation_fixture(name)
            for robot in (ROBOT, RECTANGLE):
                cm = Costmap.from_robot(fixture.grid, robot)
                for row in range(-2, 42):
                    for column in range(-2, 42):
                        centre = cell_center(fixture.grid, row, column)
                        inside = 0 <= row < 40 and 0 <= column < 40
                        exact = inside and not footprint_collides(
                            fixture.grid, cm._disc, Pose(centre.x_m, centre.y_m, 0.0, 0.0), cm.margin_m)
                        self.assertEqual(cm.cell_traversable(row, column), exact, (name, row, column))


class SmoothingTests(unittest.TestCase):
    def test_smoothing_shortens_keeps_endpoints_and_stays_collision_free(self):
        fixture, cm = costmap('wall_detour')
        raw = planner.plan(cm, fixture.start_pose, fixture.goal_pose)
        smooth = planner.plan(cm, fixture.start_pose, fixture.goal_pose, smooth=True)
        self.assertTrue(smooth.success)
        self.assertLess(len(smooth.path), len(raw.path))
        self.assertLessEqual(smooth.path_cost_m, raw.path_cost_m + 1e-9)
        self.assertAlmostEqual(smooth.path_cost_m, planner.path_length_m(smooth.path), places=9)
        self.assertEqual((smooth.path[0], smooth.path[-1]), (fixture.start_pose, fixture.goal_pose))
        self.assertTrue(planner.path_valid(cm, smooth.path))

    def test_navigator_can_drive_a_smoothed_path(self):
        fixture = load_navigation_fixture('wall_detour')
        sim = MockSimulator(fixture.grid, ROBOT)
        sim.reset('E', 'mock', fixture.start_pose, 0)
        navigator = TeamNavigator(sim, smooth_paths=True)
        navigator.reset(NavigatorInit('E', ROBOT, ObservationConfig(), NavigationConfig()))
        navigator.start(NavigationRequest('E', 'g', fixture.grid, fixture.start_pose, goal_of(fixture)))
        feedback = run_until_terminal(sim, navigator, 'g')
        self.assertEqual(feedback.status, NavigationStatus.REACHED)
        self.assertEqual(sim.collisions, 0)


class ControllerPacingTests(unittest.TestCase):
    def test_faster_grid_following_still_reaches_and_stops_without_contact(self):
        fixture = load_navigation_fixture('wall_detour')
        robot = replace(ROBOT, max_linear_speed_mps=.32, max_angular_speed_radps=1.1,
                        position_tolerance_m=.03)
        times = []
        for gain in (1., 2.):
            sim = MockSimulator(fixture.grid, robot, dt_s=1/30)
            sim.reset('E', 'mock', fixture.start_pose, 0)
            nav = TeamNavigator(sim, linear_gain=gain)
            nav.reset(NavigatorInit('E', robot, ObservationConfig(), NavigationConfig()))
            nav.start(NavigationRequest('E', 'g', fixture.grid, fixture.start_pose, goal_of(fixture)))
            feedback = run_until_terminal(sim, nav, 'g', max_steps=6000)
            self.assertEqual(feedback.status, NavigationStatus.REACHED)
            self.assertEqual(sim.collisions, 0)
            self.assertLessEqual(hypot(sim.current_pose().x_m-fixture.goal_pose.x_m,
                                       sim.current_pose().y_m-fixture.goal_pose.y_m), .03)
            stopped_pose = sim.current_pose()
            sim.step()
            self.assertEqual(sim.current_pose(), stopped_pose)
            times.append(feedback.timestamp_s)
        self.assertLess(times[1], times[0] * .8)


class StallDetectionTests(unittest.TestCase):
    def test_unseen_obstacle_is_reported_blocked_instead_of_hanging(self):
        """Belief map is open, but the true world has a wall: the robot presses against it."""
        believed = load_navigation_fixture('open_space')
        true_world = load_navigation_fixture('wall_detour')
        sim = MockSimulator(true_world.grid, ROBOT)
        sim.reset('E', 'mock', believed.start_pose, 0)
        navigator = TeamNavigator(sim)
        navigator.reset(NavigatorInit('E', ROBOT, ObservationConfig(), NavigationConfig()))
        navigator.start(NavigationRequest('E', 'g', believed.grid, believed.start_pose, goal_of(believed)))
        feedback = run_until_terminal(sim, navigator, 'g')
        self.assertEqual((feedback.status, feedback.failure_reason),
                         (NavigationStatus.FAILED, FailureReason.BLOCKED))
        self.assertGreater(sim.collisions, 0)
        self.assertLess(feedback.timestamp_s, NavigationConfig().navigation_deadline_s)  # before the overall deadline
        self.assertTrue(feedback.planner_result.success)  # planning succeeded; execution was blocked

    def test_deliberate_dwell_is_not_a_stall(self):
        fixture = load_navigation_fixture('open_space')
        start = Pose(2.05, 2.05, 0.0, 0.0)
        sim = MockSimulator(fixture.grid, ROBOT)
        sim.reset('E', 'mock', start, 0)
        navigator = TeamNavigator(sim)
        slow = replace(ObservationConfig(), arrival_headings_rad=(0.0, 0.5), dwell_s=STALL_WINDOW_S + 1.0,
                       operation_deadline_s=60.0)
        navigator.reset(NavigatorInit('E', ROBOT, slow, NavigationConfig()))
        navigator.start_observation(ObservationRequest('E', 'o', fixture.grid, start, 'arrival', 0.0))
        feedback = run_until_terminal(sim, navigator, 'o')
        self.assertEqual(feedback.status, NavigationStatus.REACHED)


class RobotShapeTests(unittest.TestCase):
    """What changes when the robot is not a circle (see B_REPORT_NOTES.md)."""

    def test_planner_refuses_robots_that_cannot_turn_in_place(self):
        fixture = load_navigation_fixture('open_space')
        car_like = replace(ROBOT, can_turn_in_place=False, motion_model='ackermann')
        sim = MockSimulator(fixture.grid, ROBOT)
        sim.reset('E', 'mock', fixture.start_pose, 0)
        with self.assertRaises(NavigatorBoundaryError):
            TeamNavigator(sim).reset(NavigatorInit('E', car_like, ObservationConfig(), NavigationConfig()))

    def test_rectangular_robot_drives_open_space_without_collision(self):
        fixture = load_navigation_fixture('open_space')
        sim = MockSimulator(fixture.grid, RECTANGLE)
        sim.reset('E', 'mock', fixture.start_pose, 0)
        navigator = TeamNavigator(sim)
        navigator.reset(NavigatorInit('E', RECTANGLE, ObservationConfig(), NavigationConfig()))
        navigator.start(NavigationRequest('E', 'g', fixture.grid, fixture.start_pose, goal_of(fixture)))
        feedback = run_until_terminal(sim, navigator, 'g')
        self.assertEqual(feedback.status, NavigationStatus.REACHED)
        self.assertEqual(sim.collisions, 0)

    def test_rectangular_planning_is_conservative_known_limitation(self):
        """Planning on the circumscribed disc rejects a door the bar could pass heading east.

        Safe but incomplete: a false NO_PATH. A full fix needs yaw-aware (3-D) search.
        """
        fixture, cm = costmap('wall_detour')
        bar = Costmap.from_robot(fixture.grid, RECTANGLE)
        door_centre = Pose(2.05, 2.05, 0.0, 0.0)
        self.assertTrue(bar.pose_valid(door_centre))                                   # fits heading east
        self.assertFalse(bar.pose_valid(replace(door_centre, yaw_rad=pi / 2)))         # not heading north
        self.assertFalse(bar.cell_traversable(20, 20))                                 # disc says no
        self.assertEqual(planner.plan(bar, fixture.start_pose, fixture.goal_pose).failure_reason,
                         FailureReason.NO_PATH)
        self.assertTrue(planner.plan(cm, fixture.start_pose, fixture.goal_pose).success)  # circle robot passes

    def test_rectangle_needs_more_room_to_turn_in_place(self):
        fixture = load_navigation_fixture('open_space')
        pose = Pose(0.8, 2.05, 0.0, 0.0)  # 0.3 m from the unknown border
        self.assertTrue(Costmap.from_robot(fixture.grid, ROBOT).full_rotation_valid(pose))
        self.assertFalse(Costmap.from_robot(fixture.grid, RECTANGLE).full_rotation_valid(pose))


if __name__ == '__main__':
    unittest.main()
