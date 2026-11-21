"""P2: B's TIMEOUT must beat the runner's own operation deadline, on every time step.

The runner cancels an operation when ``sim_time - started >= limit`` (checked at the top of its loop),
which ends the whole episode as ``search_limit``. B's TIMEOUT instead feeds C's recovery (cooldown,
retry). B therefore has to time out on the same step the limit is reached, i.e. with ``>=``, and a
goal reached on that very step must still count as REACHED.
"""

from dataclasses import replace
import unittest

try:
    from tests.integration.test_igibson_adapter import FakeBackend
except ImportError:  # run from inside tests/
    from tests.integration.test_igibson_adapter import FakeBackend

from frontier_search.config import NavigationConfig, ObservationConfig
from frontier_search.contracts import (FailureReason, NavigationRequest, NavigationStatus,
                                       NavigatorInit, PolicyName, Pose, Subgoal)
from frontier_search.fixtures.loader import load_navigation_fixture
from frontier_search.fixtures.w2_episode import ScriptedMapper, build_mock_runner, mock_manifest
from frontier_search.navigation.dev_robot import PLACEHOLDER_ROBOT
from frontier_search.navigation.navigator import TeamNavigator
from frontier_search.policies.baselines import BaselinePolicy
from frontier_search.runner.episode import EpisodeRunner
from frontier_search.simulation.igibson_adapter import AdapterConfig, IGibsonAdapter
from frontier_search.simulation.mock_adapter import MockSimulator


class Log:
    def __init__(self):
        self.events = []

    def write(self, event, episode, **payload):
        self.events.append((event, payload))

    def runner_killed_operation(self):
        return any(event == 'operation_timeout' for event, _ in self.events)

    def navigate_failures(self):
        return [p['feedback'].failure_reason for e, p in self.events
                if e == 'operation_feedback' and p['kind'] == 'navigate' and p['feedback'].failure_reason]


def too_slow(manifest, deadline_s):
    """A robot that cannot reach the goal before the navigation deadline."""
    return replace(manifest, robot=replace(manifest.robot, max_linear_speed_mps=0.01),
                   navigation=replace(manifest.navigation, navigation_deadline_s=deadline_s))


class TimeoutAlignmentTests(unittest.TestCase):
    def run_adapter(self, deadline_s):
        manifest = too_slow(mock_manifest('progress'), deadline_s)
        adapter = IGibsonAdapter(manifest.robot, manifest.mapping, AdapterConfig(), FakeBackend)
        log = Log()
        runner = EpisodeRunner(adapter, ScriptedMapper('progress'), TeamNavigator(adapter),
                               BaselinePolicy(PolicyName.NEAREST), None, logger=log)
        return runner.run(manifest), log

    def test_b_times_out_first_for_the_adapters_one_thirtieth_second_step(self):
        # Deadlines that are exact multiples of the 1/30 s step were the failing cases (incl. the default 30 s).
        for deadline in (3.0, 4.5, 6.0, 30.0):
            with self.subTest(deadline_s=deadline):
                reason, log = self.run_adapter(deadline)
                self.assertFalse(log.runner_killed_operation())        # runner's backstop never fired
                self.assertNotEqual(reason.value, 'search_limit')      # episode not ended by the race
                self.assertIn(FailureReason.TIMEOUT, log.navigate_failures())   # B reported it to recovery

    def test_b_still_times_out_first_on_the_mock_simulators_step(self):
        for deadline in (3.0, 3.05, 5.0):
            with self.subTest(deadline_s=deadline):
                manifest = too_slow(mock_manifest('progress'), deadline)
                log = Log()
                reason = build_mock_runner(manifest, log).run(manifest)
                self.assertFalse(log.runner_killed_operation())
                self.assertIn(FailureReason.TIMEOUT, log.navigate_failures())

    def test_goal_reached_on_the_boundary_step_is_reached_not_timeout(self):
        fixture = load_navigation_fixture('open_space')

        def run(limit):
            sim = MockSimulator(fixture.grid, PLACEHOLDER_ROBOT)
            self.addCleanup(sim.close)
            sim.reset('x', 'm', fixture.start_pose, 0)
            nav = TeamNavigator(sim)
            nav.reset(NavigatorInit('x', PLACEHOLDER_ROBOT, ObservationConfig(),
                                    NavigationConfig(navigation_deadline_s=limit)))
            feedback = nav.start(NavigationRequest('x', 'g', fixture.grid, fixture.start_pose,
                                                   Subgoal('F1', fixture.goal_pose, 1, PolicyName.NEAREST)))
            for _ in range(5000):
                if feedback.status in (NavigationStatus.REACHED, NavigationStatus.FAILED):
                    break
                sim.step(); nav.on_step(); feedback = nav.poll('x', 'g')
            return feedback, sim.sim_time_s()

        reached, arrival_time = run(300.0)
        self.assertEqual(reached.status, NavigationStatus.REACHED)
        # Limit just below the arrival time: the arrival step is the first step at/after the limit.
        boundary, _ = run(arrival_time - 1e-9)
        self.assertEqual(boundary.status, NavigationStatus.REACHED)
        # And a limit clearly earlier still times out.
        early, _ = run(arrival_time / 2)
        self.assertEqual((early.status, early.failure_reason), (NavigationStatus.FAILED, FailureReason.TIMEOUT))


if __name__ == '__main__':
    unittest.main()
