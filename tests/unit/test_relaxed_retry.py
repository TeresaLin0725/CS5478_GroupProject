"""Opt-in bounded relaxed retry for a dispatched goal after a strict NO_PATH / INVALID_GOAL (B-local).

Default (None) is unchanged: a strict failure is final. Candidate evaluation always stays strict, so reachability
assessment is identical for every policy; only a goal the runner dispatches anyway gets one relaxed plan.
"""

from dataclasses import replace
import unittest

from frontier_search.config import NavigationConfig, ObservationConfig
from frontier_search.contracts import (CandidateEvaluationRequest, FailureReason, NavigationRequest,
                                       NavigationStatus, NavigatorInit, PolicyName, Pose, Subgoal)
from frontier_search.fixtures.loader import load_policy_fixture
from frontier_search.navigation.locobot_robot import LOCOBOT_DEV_ROBOT as ROBOT
from frontier_search.navigation.navigator import RelaxedRetry, TeamNavigator
from frontier_search.simulation.mock_adapter import MockSimulator
from tests.unit.test_planning_radius import DONE, world

START, GOAL = Pose(0.9, 2.0, 0, 0), Pose(4.5, 2.0, 0, 0)


def navigator(test, grid, relaxed):
    sim = MockSimulator(grid, ROBOT)
    test.addCleanup(sim.close)
    sim.reset('x', 'm', START, 0)
    nav = TeamNavigator(sim, relaxed_retry=relaxed)
    nav.reset(NavigatorInit('x', ROBOT, ObservationConfig(), NavigationConfig(navigation_deadline_s=600)))
    return sim, nav


def dispatch(grid, nav, goal=GOAL):
    return nav.start(NavigationRequest('x', 'g', grid, START, Subgoal('F1', goal, 1, PolicyName.NEAREST)))


class RelaxedRetryTests(unittest.TestCase):
    def test_default_strict_failure_is_final(self):
        grid, _ = world(5)  # 0.5 m gap: strict circumscribed disc plus margin needs 0.7 m
        _, nav = navigator(self, grid, None)
        feedback = dispatch(grid, nav)
        self.assertEqual((feedback.status, feedback.failure_reason), (NavigationStatus.FAILED, FailureReason.NO_PATH))

    def test_relaxed_retry_plans_and_drives_through_the_narrow_gap(self):
        grid, _ = world(5)
        sim, nav = navigator(self, grid, RelaxedRetry())
        feedback = dispatch(grid, nav)
        self.assertEqual(feedback.status, NavigationStatus.ACCEPTED)
        self.assertTrue(any(kind == 'relaxed_retry' for kind, _ in nav.events))
        for _ in range(14000):
            if feedback.status in DONE:
                break
            sim.step(); nav.on_step(); feedback = nav.poll('x', 'g')
        self.assertEqual(feedback.status, NavigationStatus.REACHED)
        self.assertEqual(sim.collisions, 0)

    def test_evaluation_stays_strict_with_relaxed_retry_enabled(self):
        grid, _ = world(5)
        _, nav = navigator(self, grid, RelaxedRetry())
        state = self._state(grid)
        evaluation = nav.evaluate_candidates(CandidateEvaluationRequest('x', state))
        self.assertIs(evaluation.assessments[0].reachable, False)

    def test_a_wall_with_no_gap_keeps_the_strict_reason(self):
        grid, _ = world(0)
        _, nav = navigator(self, grid, RelaxedRetry())
        feedback = dispatch(grid, nav)
        self.assertEqual((feedback.status, feedback.failure_reason), (NavigationStatus.FAILED, FailureReason.NO_PATH))
        self.assertTrue(any(kind == 'relaxed_retry' for kind, _ in nav.events))

    def test_reachable_goal_never_uses_the_relaxed_plan(self):
        grid, _ = world(10)
        _, nav = navigator(self, grid, RelaxedRetry())
        feedback = dispatch(grid, nav)
        self.assertEqual(feedback.status, NavigationStatus.ACCEPTED)
        self.assertFalse(any(kind == 'relaxed_retry' for kind, _ in nav.events))

    def test_invalid_parameters_are_rejected(self):
        with self.assertRaises(ValueError):
            RelaxedRetry(margin_m=-0.01)
        with self.assertRaises(ValueError):
            RelaxedRetry(planning_radius_m=0.0)

    @staticmethod
    def _state(grid):
        state = load_policy_fixture('selection').state
        frontier = replace(state.frontiers[0], approach_pose=GOAL)
        return replace(state, episode_id='x', occupancy_map=grid, robot_pose=START, frontiers=(frontier,))


if __name__ == '__main__':
    unittest.main()
