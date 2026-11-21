"""Reachability resolution, scoring, and dispatch of A's alternative poses."""
from dataclasses import replace
import json
from math import cos, hypot, sin
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from frontier_search.config import MappingConfig, NavigationConfig, ObservationConfig, PolicyConfig
from frontier_search.contracts import (CandidateAssessment, CandidateEvaluation, CandidateEvaluationRequest,
    ExploredGrid, FailureReason, Frontier, GridCell, NavigationRequest, NavigationStatus, NavigatorInit,
    OccupancyGrid, Point2, PolicyName, Pose, RobotConfig, WorldState)
from frontier_search.navigation.costmap import Costmap
from frontier_search.navigation.locobot_robot import shared_round_robot
from frontier_search.navigation.navigator import TeamNavigator
from frontier_search.navigation import planner
from frontier_search.perception.frontiers import (_approaches, _clearance, assign_identities,
                                                 build_candidates, resolve_approaches)
from frontier_search.perception.gain import estimate_gain
from frontier_search.perception.mapper import RGBDMapper
from frontier_search.policies.baselines import BaselinePolicy
from frontier_search.policies.llm import LLMGuidedPolicy
from frontier_search.runner.episode import build_context, decide, TerminationReason
from frontier_search.sensors import CameraIntrinsics
from frontier_search.simulation.mock_adapter import MockSimulator
from frontier_search.validation import ContractError, validate_evaluation
from tests.unit.test_mapper_update import request, scene_frame
from tests.integration.test_w2_runner import make_runner
from tests.unit.test_frontiers import boundary_map


def world(grid, pose, frontiers):
    explored = ExploredGrid(grid.frame_id, grid.revision, grid.resolution_m, grid.origin_x_m,
        grid.origin_y_m, grid.width_cells, grid.height_cells,
        tuple(tuple(v != -1 for v in row) for row in grid.data))
    return WorldState('1.1', 'alternatives', grid.revision, 0., 1, 'cup', pose,
                      grid, explored, (), (), frontiers, None)


def disconnected():
    data = tuple(tuple(0 if 2 <= r <= 12 and (2 <= c <= 10 or 17 <= c <= 27)
                       else -1 if r == 13 else 100 for c in range(30)) for r in range(16))
    grid = OccupancyGrid('world', 1, .1, 0., 0., 30, 16, data)
    pose = Pose(.65, .65, 0., 0.)
    frontier = Frontier('F1', (), (GridCell(12, 5),), Pose(2.15, .65, 0., 0.), None, 99., 1.5,
                        alternative_approach_poses=(Pose(.65, .85, 0., 0.),))
    return world(grid, pose, (frontier,)), shared_round_robot(request().robot_config)


def navigator(state, robot, clock=None):
    control = MockSimulator(state.occupancy_map, robot)
    control.reset(state.episode_id, 'mock', state.robot_pose, 0)
    nav = TeamNavigator(control, start_margin_m=0., **({'wall_clock': clock} if clock else {}))
    nav.reset(NavigatorInit(state.episode_id, robot, ObservationConfig(), NavigationConfig()))
    return nav


class AlternativeApproachTests(unittest.TestCase):
    def test_generated_alternatives_keep_identity_and_refresh_after_new_obstacles(self):
        grid = boundary_map()
        robot = request().robot_config
        observation = scene_frame(x=1., y=1.)
        mapping = MappingConfig(width_cells=18, height_cells=14, resolution_m=.2)
        candidates = build_candidates(grid, observation, robot, mapping, ObservationConfig(), {})
        frontiers, memory, history, counter = assign_identities(candidates, {}, {}, 0, 0., mapping)
        self.assertTrue(frontiers[0].alternative_approach_poses)
        self.assertEqual(frontiers[0].alternative_approach_poses, candidates[0].alternative_approach_poses)
        self.assertEqual(candidates, build_candidates(grid, observation, robot, mapping, ObservationConfig(), {}))
        old = frontiers[0].approach_pose
        cells = [list(row) for row in grid.data]
        cells[int(old.y_m/grid.resolution_m)][int(old.x_m/grid.resolution_m)] = 100
        updated = replace(grid, revision=2, data=tuple(map(tuple, cells)))
        candidates = build_candidates(updated, observation, robot, mapping, ObservationConfig(), {})
        refreshed, _, _, _ = assign_identities(candidates, memory, history, counter, 1., mapping)
        self.assertEqual(refreshed[0].frontier_id, frontiers[0].frontier_id)
        costmap = Costmap.from_robot(updated, robot)
        for frontier in refreshed:
            for pose in (frontier.approach_pose,) + frontier.alternative_approach_poses:
                self.assertNotEqual(pose, old)
                self.assertTrue(costmap.pose_valid(pose))

    def test_disconnected_preferred_pose_resolves_to_first_reachable_alternative(self):
        state, robot = disconnected()
        nav = navigator(state, robot)
        before = state.frontiers[0]
        self.assertFalse(planner.plan(nav._costmap_for(state.occupancy_map), state.robot_pose,
                                      before.approach_pose).success)
        evaluation = nav.evaluate_candidates(CandidateEvaluationRequest(state.episode_id, state))
        assessment = evaluation.assessments[0]
        self.assertTrue(assessment.reachable)
        self.assertEqual(assessment.approach_pose, before.alternative_approach_poses[0])
        validate_evaluation(state, evaluation)
        frame = scene_frame(x=.65, y=.65)
        resolved = resolve_approaches(state, evaluation, frame, MappingConfig(), ObservationConfig())
        self.assertEqual(state.frontiers[0], before)
        self.assertEqual(resolved.frontiers[0].approach_pose, assessment.approach_pose)
        self.assertEqual(resolved.frontiers[0].information_gain_m2, estimate_gain(
            state.occupancy_map, assessment.approach_pose, frame, MappingConfig(), ObservationConfig()))
        self.assertAlmostEqual(resolved.frontiers[0].distance_from_robot_m, .2)
        for name in (PolicyName.NEAREST, PolicyName.INFORMATION_GAIN, PolicyName.RANDOM):
            policy = BaselinePolicy(name)
            policy.reset(state.episode_id, 0, PolicyConfig())
            selected = decide(policy, resolved, build_context(resolved, evaluation))
            self.assertEqual(selected.subgoal.approach_pose, assessment.approach_pose)
        with self.assertRaisesRegex(ContractError, 'assessed approach'):
            decide(policy, state, build_context(state, evaluation))

    def test_preferred_pose_wins_when_reachable(self):
        state, robot = disconnected()
        frontier = state.frontiers[0]
        state = replace(state, frontiers=(replace(frontier, approach_pose=Pose(.65, .65, 0., 0.)),))
        result = navigator(state, robot).evaluate_candidates(CandidateEvaluationRequest(state.episode_id, state))
        self.assertEqual(result.assessments[0].approach_pose, state.frontiers[0].approach_pose)

    def test_all_unreachable_and_all_invalid_are_distinct(self):
        state, robot = disconnected()
        frontier = replace(state.frontiers[0], alternative_approach_poses=(Pose(2.25, .65, 0., 0.),))
        for valid in (True, False):
            candidate = frontier if valid else replace(frontier, approach_pose=Pose(0., 0., 0., 0.),
                                                       alternative_approach_poses=(Pose(.1, .1, 0., 0.),))
            current = replace(state, frontiers=(candidate,))
            result = navigator(current, robot).evaluate_candidates(CandidateEvaluationRequest(current.episode_id, current))
            assessment = result.assessments[0]
            self.assertFalse(assessment.reachable)
            self.assertEqual(assessment.failure_reason, FailureReason.NO_PATH if valid else FailureReason.INVALID_GOAL)
            self.assertIsNone(assessment.approach_pose)

    def test_partial_distance_field_is_inconclusive(self):
        state, robot = disconnected()
        nav = navigator(state, robot)
        with patch('frontier_search.navigation.navigator.planner.distance_field', return_value=({}, False)):
            assessment = nav.evaluate_candidates(CandidateEvaluationRequest(state.episode_id, state)).assessments[0]
        self.assertIsNone(assessment.reachable)
        self.assertEqual(assessment.failure_reason, FailureReason.TIMEOUT)

    def test_partial_field_can_still_prove_an_alternative_reachable(self):
        state, robot = disconnected()
        nav = navigator(state, robot)
        with patch('frontier_search.navigation.navigator.planner.distance_field',
                   return_value=({(8, 6): .2}, False)):
            assessment = nav.evaluate_candidates(CandidateEvaluationRequest(state.episode_id, state)).assessments[0]
        self.assertTrue(assessment.reachable)
        self.assertEqual(assessment.approach_pose, state.frontiers[0].alternative_approach_poses[0])

    def test_resolved_gain_reaches_information_gain_policy_and_llm_payload(self):
        state, robot = disconnected()
        second = replace(state.frontiers[0], frontier_id='F2', approach_pose=Pose(.85, .65, 0., 0.),
                         alternative_approach_poses=(), information_gain_m2=5.)
        state = replace(state, frontiers=state.frontiers + (second,))
        evaluation = navigator(state, robot).evaluate_candidates(CandidateEvaluationRequest(state.episode_id, state))
        with patch('frontier_search.perception.frontiers.estimate_gain', return_value=1.) as gain:
            resolved = resolve_approaches(state, evaluation, scene_frame(), MappingConfig(), ObservationConfig())
        self.assertEqual(gain.call_count, 1)  # Recompute only the pose B changed.
        context = build_context(resolved, evaluation)
        policy = BaselinePolicy(PolicyName.INFORMATION_GAIN)
        policy.reset(state.episode_id, 0, PolicyConfig())
        self.assertEqual(decide(policy, resolved, context).subgoal.frontier_id, 'F2')
        class Client:
            def select_id(self, payload, **kwargs):
                self.payload = payload
                return 'F1'
        client = Client()
        policy = LLMGuidedPolicy(client)
        policy.reset(state.episode_id, 0, PolicyConfig(llm_model='local-test'))
        selection = decide(policy, resolved, context)
        candidate = next(f for f in client.payload['candidates'] if f['frontier_id'] == 'F1')
        self.assertEqual(candidate['information_gain_m2'], 1.)
        self.assertEqual(selection.subgoal.approach_pose, evaluation.assessments[0].approach_pose)

    def test_assessment_cannot_invent_a_pose_or_reuse_a_stale_revision(self):
        state, robot = disconnected()
        evaluation = navigator(state, robot).evaluate_candidates(CandidateEvaluationRequest(state.episode_id, state))
        invented = replace(evaluation.assessments[0], approach_pose=Pose(.65, .75, 0., 0.))
        with self.assertRaisesRegex(ContractError, 'unoffered'):
            validate_evaluation(state, replace(evaluation, assessments=(invented,)))
        with self.assertRaisesRegex(ContractError, 'stale assessment'):
            validate_evaluation(state, replace(evaluation, state_revision=0))

    def test_expired_assessment_is_inconclusive(self):
        state, robot = disconnected()
        now = [0.]
        nav = navigator(state, robot, clock=lambda: now[0])
        def expired(*args, **kwargs):
            now[0] = 10.
            return {}, True
        with patch('frontier_search.navigation.navigator.planner.distance_field', side_effect=expired):
            assessment = nav.evaluate_candidates(CandidateEvaluationRequest(state.episode_id, state)).assessments[0]
        self.assertIsNone(assessment.reachable)
        self.assertEqual(assessment.failure_reason, FailureReason.TIMEOUT)

    def test_map_change_invalidates_the_previously_selected_pose(self):
        state, robot = disconnected()
        nav = navigator(state, robot)
        evaluation = nav.evaluate_candidates(CandidateEvaluationRequest(state.episode_id, state))
        frame = scene_frame(x=.65, y=.65)
        resolved = resolve_approaches(state, evaluation, frame, MappingConfig(), ObservationConfig())
        policy = BaselinePolicy(PolicyName.NEAREST)
        policy.reset(state.episode_id, 0, PolicyConfig())
        goal = decide(policy, resolved, build_context(resolved, evaluation)).subgoal
        accepted = nav.start(NavigationRequest(state.episode_id, 'g', state.occupancy_map, state.robot_pose, goal))
        self.assertTrue(accepted.planner_result.success)
        cells = [list(row) for row in state.occupancy_map.data]
        cells[8][6] = 100
        updated = replace(state.occupancy_map, revision=2, data=tuple(map(tuple, cells)))
        nav.update_map(state.episode_id, 'g', updated)
        feedback = nav.poll(state.episode_id, 'g')
        self.assertEqual(feedback.status, NavigationStatus.FAILED)
        self.assertEqual(feedback.failure_reason, FailureReason.INVALID_GOAL)

    def test_mapper_resolution_rejects_stale_snapshot_and_is_read_only(self):
        mapper = RGBDMapper()
        mapper.reset(request())
        first = mapper.update(scene_frame())
        # Legacy assessments with no selected alternative leave geometry unchanged.
        evaluation = CandidateEvaluation(first.episode_id, first.revision, first.robot_pose,
            tuple(CandidateAssessment(f.frontier_id, False, None, FailureReason.NO_PATH) for f in first.frontiers))
        self.assertEqual(mapper.resolve_approaches(first, evaluation), first)
        self.assertIs(mapper._state.latest_state, first)
        mapper.update(scene_frame(sequence=2))
        with self.assertRaisesRegex(ContractError, 'latest mapper snapshot'):
            mapper.resolve_approaches(first, evaluation)

    def test_runner_dispatches_the_assessed_alternative_and_logs_its_gain(self):
        manifest, runner, log = make_runner()
        original = runner.mapper.update
        latest = [None]
        def with_alternatives(observation):
            latest[0] = observation
            state = original(observation)
            return replace(state, frontiers=tuple(replace(f, approach_pose=Pose(0., 0., 0., 0.),
                alternative_approach_poses=(f.approach_pose,), information_gain_m2=999.) for f in state.frontiers))
        runner.mapper.update = with_alternatives
        runner.mapper.resolve_approaches = lambda state, evaluation: resolve_approaches(
            state, evaluation, latest[0], manifest.mapping, manifest.observation)
        self.assertEqual(runner.run(manifest), TerminationReason.NO_FRONTIERS)
        dispatches = [e['request'].subgoal for e in log.events if e['event'] == 'dispatch']
        decisions = [e for e in log.events if e['event'] == 'decision' and e['selection'].subgoal]
        self.assertEqual(len(dispatches), 2)
        for dispatch, decision in zip(dispatches, decisions):
            assessment = decision['context'].evaluation.assessments[0]
            self.assertEqual(dispatch.approach_pose, assessment.approach_pose)
            self.assertEqual(dispatch.approach_pose, decision['state'].frontiers[0].approach_pose)
            self.assertNotEqual(decision['state'].frontiers[0].information_gain_m2, 999.)
        self.assertEqual(runner.summary['collisions'], 0)

    def test_f000220_selects_a_reachable_offered_pose(self):
        fixture = json.loads((Path(__file__).resolve().parents[1]/'fixtures/shared_round_frontier.json').read_text())
        data = tuple(tuple(v for v, count in row for _ in range(count)) for row in fixture['rows_rle'])
        grid = OccupancyGrid(data=data, **fixture['grid'])
        robot_data = dict(fixture['robot'])
        robot_data['footprint_base'] = tuple(Point2(**v) for v in robot_data['footprint_base'])
        robot = shared_round_robot(RobotConfig(**robot_data))
        pose = Pose(**fixture['robot_pose'])
        angle = pose.yaw_rad + fixture['camera_yaw_offset_rad']
        observation = SimpleNamespace(robot_pose=pose, camera_intrinsics=CameraIntrinsics(640, 480, 312.77, 312.77, 319.5, 239.5),
            camera_to_world=SimpleNamespace(matrix=((0., 0., cos(angle), pose.x_m),
                (0., 0., sin(angle), pose.y_m), (0., 0., 0., 0.), (0., 0., 0., 1.))))
        support = tuple(GridCell(**v) for v in fixture['frontier']['support_cells'])
        mapping = MappingConfig(**fixture['mapping'])
        poses = tuple(_approaches(grid, support, _clearance(grid), robot, observation, mapping))
        frontier = Frontier('F000220', (), support, poses[0], None, 0., 0., alternative_approach_poses=poses[1:])
        state = world(grid, pose, (frontier,))
        nav = navigator(state, robot)
        costmap = nav._costmap_for(grid)
        self.assertFalse(planner.plan(costmap, pose, poses[0]).success)
        assessment = nav.evaluate_candidates(CandidateEvaluationRequest(state.episode_id, state))
        selected = assessment.assessments[0]
        self.assertTrue(selected.reachable)
        self.assertNotEqual(selected.approach_pose, poses[0])
        self.assertTrue(costmap.pose_valid(selected.approach_pose))
        self.assertTrue(planner.plan(costmap, pose, selected.approach_pose).success)
        resolved = resolve_approaches(state, assessment, observation, mapping, ObservationConfig())
        policy = BaselinePolicy(PolicyName.INFORMATION_GAIN)
        policy.reset(state.episode_id, 0, PolicyConfig())
        choice = decide(policy, resolved, build_context(resolved, assessment))
        self.assertEqual(choice.subgoal.approach_pose, selected.approach_pose)
        self.assertAlmostEqual(resolved.frontiers[0].information_gain_m2,
            estimate_gain(grid, selected.approach_pose, observation, mapping, ObservationConfig()))


if __name__ == '__main__':
    unittest.main()
