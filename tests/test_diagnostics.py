"""Read-only failure diagnostics: conservative planning vs a genuinely blocked frontier."""
from dataclasses import replace
import unittest

from frontier_search.config import NavigationConfig, ObservationConfig
from frontier_search.contracts import CandidateEvaluationRequest, NavigatorInit, OccupancyGrid, Pose
from frontier_search.fixtures.loader import load_policy_fixture
from frontier_search.navigation.diagnostics import _clearance_to_blocked_m, diagnose_goals
from frontier_search.navigation.locobot_robot import LOCOBOT_DEV_ROBOT as ROBOT
from frontier_search.navigation.navigator import TeamNavigator
from frontier_search.simulation.mock_adapter import MockSimulator
from tests.test_epsilon_retry import probes, walled_runner
from tests.test_planning_radius import world

START, GOAL = Pose(0.9, 2.0, 0, 0), Pose(4.5, 2.0, 0, 0)


def diagnose(gap):
    grid, _ = world(gap)
    return diagnose_goals(grid, ROBOT, START, {'F': GOAL})['F']


class DiagnosticsTests(unittest.TestCase):
    def test_clearance_uses_cell_edges_and_corners_without_overestimating(self):
        cells = [[0] * 5 for _ in range(5)]
        cells[2][2] = 100
        grid = OccupancyGrid('world', 1, .1, 0., 0., 5, 5, tuple(map(tuple, cells)))
        # The nearest obstacle point is its corner (.2, .2), not its centre minus half a cell.
        self.assertAlmostEqual(_clearance_to_blocked_m(grid, Pose(.19, .19, 0., 0.)), .01 * 2**.5)
        self.assertAlmostEqual(_clearance_to_blocked_m(grid, Pose(.19, .25, 0., 0.)), .01)
        self.assertEqual(_clearance_to_blocked_m(grid, Pose(.25, .25, 0., 0.)), 0.)

    def test_wide_gap_is_simply_reachable(self):
        report = diagnose(10)
        self.assertEqual(report['cause'], 'reachable')
        self.assertIsNone(report['slack_m'])

    def test_narrow_gap_is_conservative_planning_with_positive_slack(self):
        report = diagnose(5)  # 0.5 m gap; strict circumscribed disc plus margin needs about 0.55 m
        self.assertEqual(report['cause'], 'conservative_planning')
        self.assertGreater(report['slack_m'], 0.0)
        self.assertLess(report['min_reachable_clearance_m'], report['strict_clearance_m'])
        self.assertGreaterEqual(report['min_reachable_clearance_m'], report['inscribed_radius_m'])

    def test_wall_without_a_gap_is_blocked(self):
        report = diagnose(0)
        self.assertEqual(report['cause'], 'blocked')
        self.assertIsNone(report['min_reachable_clearance_m'])

    def test_goal_outside_the_map_is_reported(self):
        grid, _ = world(10)
        report = diagnose_goals(grid, ROBOT, START, {'F': Pose(99.0, 2.0, 0, 0)})['F']
        self.assertEqual(report['cause'], 'goal_out_of_bounds')
        self.assertIsNone(report['goal_clearance_m'])

    def test_navigator_method_matches_and_stays_read_only(self):
        grid, _ = world(5)
        state = load_policy_fixture('selection').state
        frontier = replace(state.frontiers[0], approach_pose=GOAL)
        state = replace(state, episode_id='x', occupancy_map=grid, robot_pose=START, frontiers=(frontier,))
        sim = MockSimulator(grid, ROBOT)
        self.addCleanup(sim.close)
        sim.reset('x', 'm', START, 0)
        nav = TeamNavigator(sim)
        nav.reset(NavigatorInit('x', ROBOT, ObservationConfig(), NavigationConfig()))
        events_before = list(nav.events)
        report = nav.diagnose_candidates(CandidateEvaluationRequest('x', state))
        self.assertEqual(report[frontier.frontier_id]['cause'], 'conservative_planning')
        self.assertEqual(nav.events, events_before)  # diagnosis adds no event or operation
        self.assertIsNone(nav._active)

    def test_stuck_episode_emits_diagnostics_event(self):
        manifest, runner, log = walled_runner(gap_cells=5, robot=ROBOT)
        runner.run(manifest)
        events = [e for e in log.events if e['event'] == 'frontier_diagnostics']
        self.assertTrue(events)
        self.assertEqual(next(iter(events[0]['frontiers'].values()))['cause'], 'conservative_planning')
        self.assertEqual(probes(log), [])


if __name__ == '__main__':
    unittest.main()
