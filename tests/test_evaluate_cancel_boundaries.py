"""P3 (evaluation boundary) and P4 (cancelling an unknown goal id)."""

from dataclasses import replace
import unittest

from frontier_search.config import NavigationConfig, ObservationConfig
from frontier_search.contracts import (CandidateEvaluationRequest, FailureReason, GridCell,
                                       NavigationRequest, NavigationStatus, NavigatorInit,
                                       PolicyName, Subgoal)
from frontier_search.fixtures.loader import load_navigation_fixture, load_policy_fixture
from frontier_search.navigation.dev_robot import PLACEHOLDER_ROBOT
from frontier_search.navigation.navigator import NavigatorBoundaryError, TeamNavigator
from frontier_search.simulation.mock_adapter import MockSimulator

FIXTURE = load_navigation_fixture('open_space')
TEMPLATE = load_policy_fixture().state
DONE = (NavigationStatus.REACHED, NavigationStatus.FAILED, NavigationStatus.CANCELLED)


def state(revision, episode_id='ep'):
    frontier = replace(TEMPLATE.frontiers[0], approach_pose=FIXTURE.goal_pose,
                       support_cells=(GridCell(10, 30),), region_id=None)
    return replace(TEMPLATE, episode_id=episode_id, revision=revision, timestamp_s=0.0,
                   robot_pose=FIXTURE.start_pose, occupancy_map=replace(FIXTURE.grid, revision=revision),
                   explored_map=replace(TEMPLATE.explored_map, revision=revision),
                   frontiers=(frontier,), semantic_objects=(), regions=())


class BoundaryTests(unittest.TestCase):
    def make(self):
        sim = MockSimulator(FIXTURE.grid, PLACEHOLDER_ROBOT)
        self.addCleanup(sim.close)
        sim.reset('ep', 'mock', FIXTURE.start_pose, 0)
        nav = TeamNavigator(sim)
        nav.reset(NavigatorInit('ep', PLACEHOLDER_ROBOT, ObservationConfig(), NavigationConfig()))
        return sim, nav

    def request(self, goal_id, revision):
        return NavigationRequest('ep', goal_id, replace(FIXTURE.grid, revision=revision), FIXTURE.start_pose,
                                 Subgoal('F1', FIXTURE.goal_pose, revision, PolicyName.NEAREST))

    # --- P3 -----------------------------------------------------------------

    def test_evaluation_records_the_newest_map_offered_and_start_rejects_older_ones(self):
        # Intentional (docs/B notes): "read-only" means no motion and no touching the active operation.
        _, plain = self.make()
        self.assertEqual(plain.start(self.request('a', 3)).status, NavigationStatus.ACCEPTED)
        _, evaluated = self.make()
        evaluated.evaluate_candidates(CandidateEvaluationRequest('ep', state(5)))
        stale = evaluated.start(self.request('a', 3))
        self.assertEqual((stale.status, stale.failure_reason), (NavigationStatus.FAILED, FailureReason.INVALID_GOAL))

    def test_evaluation_never_moves_the_robot_or_touches_the_active_operation(self):
        sim, nav = self.make()
        nav.start(self.request('g', 1))
        sim.step(); nav.on_step()
        before = (sim._v, sim._w, sim.current_pose(), nav.poll('ep', 'g').status)
        nav.evaluate_candidates(CandidateEvaluationRequest('ep', state(1)))
        self.assertEqual((sim._v, sim._w, sim.current_pose(), nav.poll('ep', 'g').status), before)

    def test_request_and_state_episode_must_match(self):
        _, nav = self.make()
        self.assertTrue(nav.evaluate_candidates(CandidateEvaluationRequest('ep', state(1))).assessments)
        with self.assertRaisesRegex(NavigatorBoundaryError, 'episode'):
            nav.evaluate_candidates(CandidateEvaluationRequest('ep', state(1, episode_id='SOMEONE-ELSE')))

    # --- P4 -----------------------------------------------------------------

    def test_cancelling_an_unknown_goal_does_not_disturb_the_running_operation(self):
        sim, nav = self.make()
        nav.start(self.request('real', 1))
        for _ in range(5):
            sim.step(); nav.on_step()
        moving = (sim._v, sim._w)
        self.assertGreater(moving[0], 0.0)
        ghost = nav.cancel('ep', 'ghost', 'typo')
        self.assertEqual(ghost.status, NavigationStatus.CANCELLED)
        self.assertEqual((sim._v, sim._w), moving)                       # not stopped, not even for a tick
        self.assertEqual(nav.poll('ep', 'real').status, NavigationStatus.EXECUTING)
        feedback = nav.poll('ep', 'real')
        for _ in range(3000):
            if feedback.status in DONE:
                break
            sim.step(); nav.on_step(); feedback = nav.poll('ep', 'real')
        self.assertEqual(feedback.status, NavigationStatus.REACHED)       # the real goal still completes

    def test_cancel_is_idempotent_and_the_cancelled_id_stays_consumed(self):
        sim, nav = self.make()
        nav.start(self.request('real', 1))
        first = nav.cancel('ep', 'ghost', 'x')
        self.assertEqual(nav.cancel('ep', 'ghost', 'again'), first)
        with self.assertRaisesRegex(NavigatorBoundaryError, 'already used'):
            nav.start(self.request('ghost', 1))

    def test_cancelling_before_any_plan_exists_still_latches_and_stops(self):
        sim, nav = self.make()
        sim.set_velocity(0.3, 0.0)
        result = nav.cancel('ep', 'never-started', 'x')                  # nothing is running: documented case
        self.assertEqual(result.status, NavigationStatus.CANCELLED)
        self.assertEqual((sim._v, sim._w), (0.0, 0.0))

    def test_cancelling_the_running_operation_still_stops_it(self):
        sim, nav = self.make()
        nav.start(self.request('real', 1))
        for _ in range(5):
            sim.step(); nav.on_step()
        self.assertEqual(nav.cancel('ep', 'real', 'x').status, NavigationStatus.CANCELLED)
        self.assertEqual((sim._v, sim._w), (0.0, 0.0))


if __name__ == '__main__':
    unittest.main()
