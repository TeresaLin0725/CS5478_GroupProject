"""Cross-owner seams; authored maps are NOT an RGB-D mapping episode."""

from dataclasses import replace
import unittest

from frontier_search.config import MappingConfig, NavigationConfig, ObservationConfig, PolicyConfig
from frontier_search.contracts import (
    CandidateEvaluationRequest, GridCell, NavigationRequest, NavigationStatus,
    NavigatorInit, Point3, PolicyName, Pose,
)
from frontier_search.fixtures.loader import load_navigation_fixture, load_policy_fixture
from frontier_search.navigation.dev_robot import PLACEHOLDER_ROBOT
from frontier_search.navigation.navigator import TeamNavigator
from frontier_search.perception.occupancy import mark_obstacles
from frontier_search.perception.projection import ProjectedDepthPoint
from frontier_search.policies.baselines import BaselinePolicy
from frontier_search.runner.episode import build_context, decide
from frontier_search.simulation.mock_adapter import MockSimulator
from frontier_search.validation import ContractError


class W2IntegrationTests(unittest.TestCase):
    def setUp(self):
        fixture = load_navigation_fixture('open_space')
        template = load_policy_fixture().state
        # Both candidates are inside footprint-safe free space. Their identities
        # and information gains are authored, not produced by A's pending update.
        frontiers = tuple(replace(f, approach_pose=pose, support_cells=(cell,), region_id=None)
                          for f, pose, cell in zip(template.frontiers,
                              (Pose(2.05, 1.05, 0, 0), fixture.goal_pose),
                              (GridCell(10, 20), GridCell(10, 30))))
        self.state = replace(template, timestamp_s=0.0, robot_pose=fixture.start_pose,
                             occupancy_map=fixture.grid, frontiers=frontiers,
                             semantic_objects=(), regions=())
        self.sim = MockSimulator(fixture.grid, PLACEHOLDER_ROBOT)
        self.sim.reset(self.state.episode_id, 'mock', fixture.start_pose, 0)
        self.nav = TeamNavigator(self.sim)
        self.nav.reset(NavigatorInit(self.state.episode_id, PLACEHOLDER_ROBOT,
                                    ObservationConfig(), NavigationConfig()))
        self.policy = BaselinePolicy(PolicyName.NEAREST)
        self.policy.reset(self.state.episode_id, 0, PolicyConfig())

    def tearDown(self):
        self.sim.close()

    def assess(self, state):
        return self.nav.evaluate_candidates(CandidateEvaluationRequest(state.episode_id, state))

    def test_b_evaluation_c_selection_b_execution(self):
        self.sim.pause()
        evaluation = self.assess(self.state)
        context = build_context(self.state, evaluation)
        selection = decide(self.policy, self.state, context)
        self.assertEqual(selection.subgoal.frontier_id, 'F1')
        self.sim.resume()
        feedback = self.nav.start(NavigationRequest(self.state.episode_id, 'w2-goal',
            self.state.occupancy_map, self.sim.current_pose(), selection.subgoal))
        for _ in range(1000):
            if feedback.status in (NavigationStatus.REACHED, NavigationStatus.FAILED,
                                    NavigationStatus.CANCELLED):
                break
            self.sim.step()
            self.nav.on_step()
            feedback = self.nav.poll(self.state.episode_id, 'w2-goal')
        self.assertEqual(feedback.status, NavigationStatus.REACHED)
        self.assertEqual(self.sim.collisions, 0)
        self.assertGreater(self.sim.executed_distance_m, 0)

    def test_a_obstacle_revision_changes_b_feasibility_and_c_choice(self):
        old_evaluation = self.assess(self.state)
        point = ProjectedDepthPoint(0, 0, 1.0, Point3(2.05, 1.15, 0.3))
        grid = mark_obstacles(self.state.occupancy_map, (point,), self.state.robot_pose,
                              PLACEHOLDER_ROBOT, MappingConfig())
        state = replace(self.state, revision=grid.revision, occupancy_map=grid,
                        explored_map=replace(self.state.explored_map, revision=grid.revision))
        self.assertEqual(self.state.occupancy_map.data[11][20], 0)
        self.assertEqual(grid.data[11][20], 100)
        with self.assertRaises(ContractError):
            build_context(state, old_evaluation)
        evaluation = self.assess(state)
        context = build_context(state, evaluation)
        self.assertNotIn('F1', context.eligible_frontier_ids)
        self.assertEqual(decide(self.policy, state, context).subgoal.frontier_id, 'F2')

    def test_b_rejects_authored_candidates_without_footprint_clearance(self):
        # The original C policy fixture authors reachability. Real B assessment
        # must override it: its frontier centres sit too close to unknown cells.
        state = load_policy_fixture().state
        evaluation = self.assess(state)
        context = build_context(state, evaluation)
        self.assertEqual(context.eligible_frontier_ids, ())
        self.assertEqual(decide(self.policy, state, context).stop_reason, 'no_eligible_frontiers')


if __name__ == '__main__':
    unittest.main()
