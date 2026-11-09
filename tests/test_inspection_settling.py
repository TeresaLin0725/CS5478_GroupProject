"""Inspection dwell hysteresis leaves room for drift without relaxing safety."""
from dataclasses import replace
import unittest

from frontier_search.config import ObservationConfig
from frontier_search.contracts import ObservationRequest, Pose, NavigationStatus, FailureReason
from frontier_search.navigation.dev_robot import PLACEHOLDER_ROBOT
from frontier_search.simulation.mock_adapter import MockSimulator
from test_stuck_fixes import room, make_nav, run, JitterSimulator


class InspectionSettlingTests(unittest.TestCase):
    def setup_scan(self, ratio=.5, routine='inspection'):
        grid, pose = room(), Pose(1.55, 1.55, 0., .04)
        sim = MockSimulator(grid, PLACEHOLDER_ROBOT)
        cfg = replace(ObservationConfig(), inspection_headings_rad=(0.,),
                      arrival_headings_rad=(0.,), inspection_hold_s=4.,
                      inspection_heading_settle_ratio=ratio)
        nav = make_nav(self, sim, pose, cfg)
        nav.start_observation(ObservationRequest('x', 'o', grid, pose, routine, 0.))
        return sim, nav

    def test_entry_is_tighter_but_small_drift_keeps_original_hold(self):
        sim, nav = self.setup_scan()
        nav.on_step()
        self.assertIsNone(nav._active.dwell_until)  # .04 is inside outer, outside inner
        sim._pose = replace(sim._pose, yaw_rad=.02)
        nav.on_step()
        deadline = nav._active.dwell_until
        self.assertIsNotNone(deadline)
        sim._pose = replace(sim._pose, yaw_rad=.04)
        sim._t = deadline - .1
        nav.on_step()
        self.assertEqual(nav._active.dwell_until, deadline)
        sim._t = deadline
        nav.on_step()
        self.assertEqual(nav.poll('x', 'o').status, NavigationStatus.REACHED)
        self.assertEqual(len(nav.capture_marks('o')), 1)

    def test_outer_tolerance_violation_requires_tight_reacquisition(self):
        sim, nav = self.setup_scan()
        sim._pose = replace(sim._pose, yaw_rad=.02)
        nav.on_step()
        sim._pose = replace(sim._pose, yaw_rad=.06)
        nav.on_step()
        self.assertIsNone(nav._active.dwell_until)
        self.assertEqual(nav.capture_marks('o'), ())
        sim._pose = replace(sim._pose, yaw_rad=.04)
        nav.on_step()
        self.assertIsNone(nav._active.dwell_until)

    def test_default_and_other_scans_keep_original_entry_tolerance(self):
        for ratio, routine in [(1., 'inspection'), (.5, 'arrival')]:
            with self.subTest(ratio=ratio, routine=routine):
                sim, nav = self.setup_scan(ratio, routine)
                nav.on_step()
                self.assertIsNotNone(nav._active.dwell_until)

    def test_unsettled_inspection_still_fails_before_deadline(self):
        grid, start = room(), Pose(1.55, 1.55, 0., 0.)
        sim = JitterSimulator(grid, PLACEHOLDER_ROBOT, target_yaw=.5)
        cfg = replace(ObservationConfig(), inspection_headings_rad=(.5,),
                      inspection_heading_settle_ratio=.5, inspection_hold_s=4.,
                      operation_deadline_s=400.)
        nav = make_nav(self, sim, start, cfg)
        nav.start_observation(ObservationRequest('x', 'o', grid, start, 'inspection', 0.))
        feedback = run(sim, nav, 'o', 30.)
        self.assertEqual((feedback.status, feedback.failure_reason),
                         (NavigationStatus.FAILED, FailureReason.BLOCKED))
        self.assertLess(feedback.timestamp_s, 15.)
        self.assertEqual(nav.capture_marks('o'), ())

    def test_ratio_validation(self):
        for value in [0., -.5, 1.1, float('nan'), float('inf'), True, '0.5']:
            with self.subTest(value=value), self.assertRaises(ValueError):
                ObservationConfig(inspection_heading_settle_ratio=value)


if __name__ == '__main__':
    unittest.main()
