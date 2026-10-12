"""B evaluates candidates -> C's LLM-guided policy chooses -> B runs A* to that choice.

The LLM client is a local stub (no network, no paid API); it only decides WHICH frontier.
"""

from dataclasses import replace
import unittest

from frontier_search.config import MappingConfig, NavigationConfig, ObservationConfig, PolicyConfig
from frontier_search.contracts import (CandidateEvaluationRequest, GridCell, NavigationRequest,
                                       NavigationStatus, NavigatorInit, Point3, Pose)
from frontier_search.fixtures.loader import load_navigation_fixture, load_policy_fixture
from frontier_search.navigation.dev_robot import PLACEHOLDER_ROBOT
from frontier_search.navigation.locobot_robot import LOCOBOT_DEV_ROBOT
from frontier_search.navigation.navigator import TeamNavigator
from frontier_search.perception.occupancy import mark_obstacles
from frontier_search.perception.projection import ProjectedDepthPoint
from frontier_search.policies.llm import LLMGuidedPolicy
from frontier_search.runner.episode import build_context, decide
from frontier_search.simulation.mock_adapter import MockSimulator

DONE = (NavigationStatus.REACHED, NavigationStatus.FAILED, NavigationStatus.CANCELLED)


class StubLLM:
    def __init__(self, answer):
        self.answer, self.payloads = answer, []

    def select_id(self, payload, **kwargs):
        self.payloads.append(payload)
        return self.answer


class LLMToBPipelineTests(unittest.TestCase):
    ROBOT = PLACEHOLDER_ROBOT

    def setUp(self):
        fixture = load_navigation_fixture('open_space')
        template = load_policy_fixture().state
        frontiers = tuple(replace(f, approach_pose=pose, support_cells=(cell,), region_id=None)
                          for f, pose, cell in zip(template.frontiers,
                              (Pose(2.05, 1.05, 0, 0), fixture.goal_pose),
                              (GridCell(10, 20), GridCell(10, 30))))
        self.state = replace(template, timestamp_s=0.0, robot_pose=fixture.start_pose,
                             occupancy_map=fixture.grid, frontiers=frontiers,
                             semantic_objects=(), regions=())
        self.sim = MockSimulator(fixture.grid, self.ROBOT)
        self.sim.reset(self.state.episode_id, 'mock', fixture.start_pose, 0)
        self.addCleanup(self.sim.close)
        self.nav = TeamNavigator(self.sim)
        self.nav.reset(NavigatorInit(self.state.episode_id, self.ROBOT,
                                    ObservationConfig(), NavigationConfig()))

    def choose(self, answer, state=None):
        state = state or self.state
        client = StubLLM(answer)
        policy = LLMGuidedPolicy(client)
        policy.reset(state.episode_id, 0, PolicyConfig(llm_model='stub'))
        evaluation = self.nav.evaluate_candidates(CandidateEvaluationRequest(state.episode_id, state))
        context = build_context(state, evaluation)
        return decide(policy, state, context), client, evaluation, state

    def execute(self, selection, state):
        feedback = self.nav.start(NavigationRequest(state.episode_id, 'g', state.occupancy_map,
                                                    self.sim.current_pose(), selection.subgoal))
        for _ in range(2000):
            if feedback.status in DONE:
                break
            self.sim.step(); self.nav.on_step(); feedback = self.nav.poll(state.episode_id, 'g')
        return feedback

    def test_llm_choice_that_differs_from_nearest_is_what_b_drives_to(self):
        selection, client, evaluation, state = self.choose('F2')   # F1 is the nearer frontier
        self.assertEqual(selection.subgoal.frontier_id, 'F2')
        # The LLM saw only B's reachable candidates, with B's path costs.
        costs = {a.frontier_id: a.path_cost_m for a in evaluation.assessments if a.reachable}
        seen = {c['frontier_id']: c['path_cost_m'] for c in client.payloads[0]['candidates']}
        self.assertEqual(seen, costs)
        feedback = self.execute(selection, state)
        self.assertEqual(feedback.status, NavigationStatus.REACHED)
        self.assertEqual(self.sim.collisions, 0)
        goal = selection.subgoal.approach_pose
        pose = self.sim.current_pose()
        self.assertLessEqual(((pose.x_m - goal.x_m) ** 2 + (pose.y_m - goal.y_m) ** 2) ** .5, .1 + 1e-6)

    def test_invented_llm_id_falls_back_to_nearest_and_b_still_executes(self):
        selection, _, _, state = self.choose('invented')
        self.assertEqual(selection.subgoal.frontier_id, 'F1')
        self.assertEqual(self.execute(selection, state).status, NavigationStatus.REACHED)

    def test_frontier_b_marks_unreachable_is_hidden_from_llm_and_cannot_be_chosen(self):
        point = ProjectedDepthPoint(0, 0, 1.0, Point3(2.05, 1.15, 0.3))   # blocks F1, as in the W2 test
        grid = mark_obstacles(self.state.occupancy_map, (point,), self.state.robot_pose,
                              self.ROBOT, MappingConfig())
        state = replace(self.state, revision=grid.revision, occupancy_map=grid,
                        explored_map=replace(self.state.explored_map, revision=grid.revision))
        selection, client, evaluation, state = self.choose('F1', state)   # LLM asks for the blocked one
        self.assertEqual([c['frontier_id'] for c in client.payloads[0]['candidates']], ['F2'])
        self.assertEqual(selection.subgoal.frontier_id, 'F2')              # fallback, never F1
        self.assertEqual(self.execute(selection, state).status, NavigationStatus.REACHED)


class LocobotLLMToBPipelineTests(LLMToBPipelineTests):
    ROBOT = LOCOBOT_DEV_ROBOT


if __name__ == '__main__':
    unittest.main()
