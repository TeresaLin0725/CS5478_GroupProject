"""B-side robustness to localisation error (mock simulator). See tests/pose_noise_study.py for the experiments."""
import unittest

from frontier_search.contracts import FailureReason, NavigationStatus
from tests.pose_noise_study import (evaluation_under_noise, map_registration_error, stale_pose_gate)


class StalePoseGateTests(unittest.TestCase):
    def test_small_pose_disagreement_is_accepted(self):
        for offset in (0.0, 0.03):
            self.assertEqual(stale_pose_gate(offset)[0], NavigationStatus.ACCEPTED)

    def test_disagreement_above_the_limit_is_refused_as_invalid_goal(self):
        # Finding: with a real localisation error above pose_freshness_m (5 cm) every dispatch is refused, so the
        # default gate is only compatible with the ground-truth-pose assumption.
        for offset in (0.10, 0.20):
            self.assertEqual(stale_pose_gate(offset), (NavigationStatus.FAILED, FailureReason.INVALID_GOAL))


class EvaluationNoiseTests(unittest.TestCase):
    def test_verdict_is_stable_up_to_10_cm_of_start_noise(self):
        for sigma in (0.02, 0.05, 0.10):
            result = evaluation_under_noise(sigma, sigma)
            self.assertEqual((result['flip_rate'], result['unusable_rate']), (0.0, 0.0))
            self.assertLess(result['mean_cost_error_m'], 0.2)

    def test_large_noise_near_a_wall_makes_the_start_unusable_sometimes(self):
        result = evaluation_under_noise(0.30, 0.30)
        self.assertGreater(result['unusable_rate'], 0.0)
        self.assertEqual(result['flip_rate'], result['unusable_rate'])  # flips come only from an unusable start

    def test_study_is_reproducible_for_a_seed(self):
        self.assertEqual(evaluation_under_noise(0.2, 0.1, trials=15), evaluation_under_noise(0.2, 0.1, trials=15))


class MapRegistrationTests(unittest.TestCase):
    def test_a_map_shift_within_the_gap_slack_is_tolerated(self):
        # 0.6 m gap, 0.38 m wide robot: about 0.1 m of slack per side.
        for rows in (0, 1, -1):
            result = map_registration_error(rows)
            self.assertEqual((result['status'], result['collisions']), ('reached', 0), result)

    def test_a_shift_beyond_the_slack_ends_blocked_after_contact(self):
        for rows in (2, -2, 4):
            result = map_registration_error(rows)
            self.assertEqual((result['status'], result['reason']), ('failed', 'blocked'), result)
            self.assertGreater(result['collisions'], 0)

    def test_a_wider_gap_tolerates_a_larger_shift(self):
        self.assertEqual(map_registration_error(2, gap_cells=10)['status'], 'reached')


if __name__ == '__main__':
    unittest.main()
