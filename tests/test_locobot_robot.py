"""B's navigator with the Locobot development robot (A's recording settings)."""

import unittest

from frontier_search.config import NavigationConfig, ObservationConfig
from frontier_search.contracts import (FailureReason, NavigationRequest, NavigationStatus,
                                       NavigatorInit, PolicyName, Subgoal)
from frontier_search.fixtures.loader import load_navigation_fixture
from frontier_search.navigation.costmap import circumradius
from frontier_search.navigation.locobot_robot import LOCOBOT_DEV_ROBOT as ROBOT
from frontier_search.navigation.navigator import TeamNavigator
from frontier_search.simulation.mock_adapter import MockSimulator

DONE = (NavigationStatus.REACHED, NavigationStatus.FAILED, NavigationStatus.CANCELLED)


class LocobotRobotTests(unittest.TestCase):
    def run_fixture(self, name):
        f = load_navigation_fixture(name)
        sim = MockSimulator(f.grid, ROBOT)
        self.addCleanup(sim.close)
        sim.reset('x', 'mock', f.start_pose, 0)
        nav = TeamNavigator(sim)
        nav.reset(NavigatorInit('x', ROBOT, ObservationConfig(), NavigationConfig()))
        fb = nav.start(NavigationRequest('x', 'g', f.grid, f.start_pose,
                                         Subgoal('F1', f.goal_pose, f.grid.revision, PolicyName.NEAREST)))
        first = fb
        for _ in range(20000):
            if fb.status in DONE:
                break
            sim.step(); nav.on_step(); fb = nav.poll('x', 'g')
        return first, fb, sim

    def test_open_space_reaches_goal_without_collision(self):
        _, fb, sim = self.run_fixture('open_space')
        self.assertEqual(fb.status, NavigationStatus.REACHED)
        self.assertEqual(sim.collisions, 0)

    def test_half_metre_gap_is_rejected_by_conservative_disc_planning(self):
        # The rectangle (0.362 m wide + 2*0.02 m margin = 0.402 m) would fit the 0.5 m doorway,
        # but planning uses the circumscribed disc (about 0.256 m radius), which does not.
        self.assertLess(2 * 0.181 + 2 * 0.02, 0.5)
        self.assertGreater(2 * (circumradius(ROBOT.footprint_base) + ROBOT.clearance_margin_m), 0.5)
        first, fb, _ = self.run_fixture('wall_detour')
        self.assertEqual(first.status, NavigationStatus.FAILED)
        self.assertEqual(first.failure_reason, FailureReason.NO_PATH)


if __name__ == '__main__':
    unittest.main()
