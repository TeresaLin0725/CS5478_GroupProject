"""C's real three-room case and policies, with B's REAL evaluation, A* search and execution.

C's own synthetic runner executes with a point-robot "Mock B". Here C's decision (Nearest, or the
LLM policy's authored replay answer, no network/API) is handed to B's TeamNavigator on the
MockSimulator. Arrival scans stay C's scripted mock A; this is not a real RGB-D mapper.
"""

from dataclasses import replace
import unittest

from frontier_search.config import NavigationConfig, ObservationConfig, PolicyConfig
from frontier_search.contracts import (CandidateEvaluationRequest, FailureReason, NavigationRequest, NavigationStatus,
                                       NavigatorInit, OccupancyGrid, PlannerAttempt, PolicyName)
from frontier_search.fixtures.three_rooms import GRID, arrival_scan, initial_state
from frontier_search.navigation.dev_robot import PLACEHOLDER_ROBOT
from frontier_search.navigation.locobot_robot import LOCOBOT_DEV_ROBOT
from frontier_search.navigation.navigator import TeamNavigator
from frontier_search.policies.baselines import BaselinePolicy
from frontier_search.policies.llm import LLMGuidedPolicy
from frontier_search.policies.provider import ReplayClient
from frontier_search.runner.episode import build_context, decide
from frontier_search.simulation.mock_adapter import MockSimulator

DONE = (NavigationStatus.REACHED, NavigationStatus.FAILED, NavigationStatus.CANCELLED)


class CDecisionToBSearchTests(unittest.TestCase):
    ROBOT = PLACEHOLDER_ROBOT
    NAV = NavigationConfig()

    def setUp(self):
        self.state = initial_state('c-to-b')
        # Physical truth for the mock simulator: walls solid, rooms and corridor free.
        truth = OccupancyGrid('world', 1, 1.0, 0.0, 0.0, 13, 5,
                              tuple(tuple(100 if ch == '#' else 0 for ch in line) for line in GRID))
        self.sim = MockSimulator(truth, self.ROBOT)
        self.sim.reset(self.state.episode_id, 'three_rooms', self.state.robot_pose, 0)
        self.addCleanup(self.sim.close)
        self.nav = TeamNavigator(self.sim)
        self.nav.reset(NavigatorInit(self.state.episode_id, self.ROBOT, ObservationConfig(), self.NAV))

    def policy(self, name):
        if name == 'llm':
            policy = LLMGuidedPolicy(ReplayClient())
            config = PolicyConfig(llm_model='authored-replay')
        else:
            policy, config = BaselinePolicy(PolicyName.NEAREST), PolicyConfig()
        policy.reset(self.state.episode_id, 0, config)
        return policy

    def cycle(self, state, policy, previous=None):
        """One C decision on B's real assessment, then B's search/execution to that goal."""
        evaluation = self.nav.evaluate_candidates(CandidateEvaluationRequest(state.episode_id, state))
        context = build_context(state, evaluation, previous_attempt=previous)
        selection = decide(policy, state, context)
        if selection.subgoal is None:
            return selection, None, evaluation
        goal_id = 'g%d' % state.revision
        feedback = self.nav.start(NavigationRequest(state.episode_id, goal_id, state.occupancy_map,
                                                    self.sim.current_pose(), selection.subgoal))
        for _ in range(4000):
            if feedback.status in DONE:
                break
            self.sim.step(); self.nav.on_step(); feedback = self.nav.poll(state.episode_id, goal_id)
        return selection, feedback, evaluation

    def test_c_llm_decision_is_executed_by_b(self):
        selection, feedback, evaluation = self.cycle(self.state, self.policy('llm'))
        self.assertEqual(selection.subgoal.frontier_id, 'F3')            # C's authored LLM answer
        self.assertTrue(all(a.reachable for a in evaluation.assessments))  # B judged all three reachable
        self.assertEqual(feedback.status, NavigationStatus.REACHED)
        self.assertEqual(self.sim.collisions, 0)
        goal = selection.subgoal.approach_pose
        pose = self.sim.current_pose()
        self.assertLessEqual(((pose.x_m - goal.x_m) ** 2 + (pose.y_m - goal.y_m) ** 2) ** .5, .1 + 1e-6)
        self.assertGreater(self.sim.executed_distance_m, 8.5)           # ~9 m along the corridor

    def test_nearest_runs_three_decision_cycles_through_b(self):
        policy, state, previous, order = self.policy('nearest'), self.state, None, []
        for _ in range(3):
            selection, feedback, _ = self.cycle(state, policy, previous)
            order.append(selection.subgoal.frontier_id)
            self.assertEqual(feedback.status, NavigationStatus.REACHED)
            previous = PlannerAttempt(selection.subgoal, feedback.planner_result, 'reached', None)
            state, _ = arrival_scan(state, selection.subgoal.frontier_id)   # C's scripted mock A
            state = replace(state, robot_pose=self.sim.current_pose())      # B's true pose
        self.assertEqual(order, ['F1', 'F2', 'F3'])
        self.assertEqual(self.sim.collisions, 0)


class LocobotCDecisionToBSearchTests(CDecisionToBSearchTests):
    ROBOT = LOCOBOT_DEV_ROBOT
    # 9 m at 0.1 m/s needs ~90 s; the shared default navigation deadline is 30 s (not edited here).
    NAV = NavigationConfig(navigation_deadline_s=120.0)

    def test_default_30s_deadline_times_out_a_slow_locobot_without_calling_it_unreachable(self):
        self.nav.reset(NavigatorInit(self.state.episode_id, self.ROBOT, ObservationConfig(),
                                     NavigationConfig()))
        selection, feedback, _ = self.cycle(self.state, self.policy('llm'))
        self.assertEqual(feedback.status, NavigationStatus.FAILED)
        self.assertEqual(feedback.failure_reason, FailureReason.TIMEOUT)
        self.assertTrue(feedback.planner_result.success)                 # B had found a path
        self.assertAlmostEqual(self.sim.executed_distance_m, 30 * 0.1, delta=0.1)
        self.assertEqual(self.sim.collisions, 0)


if __name__ == '__main__':
    unittest.main()
