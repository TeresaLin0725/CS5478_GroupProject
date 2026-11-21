"""Production mapper confirmation and frozen terminal evidence regressions."""

from dataclasses import FrozenInstanceError, replace
import unittest
from unittest.mock import Mock, patch

from frontier_search.config import MappingConfig
from frontier_search.contracts import Point3
from frontier_search.immutable import require_immutable
from frontier_search.perception.detection import RGBDetection
from frontier_search.perception.mapper import RGBDMapper
from frontier_search.perception.claims import CONFIRMATION_TIME_TOLERANCE_S
from frontier_search.fixtures.w2_episode import mock_manifest
from frontier_search.runner.episode import EpisodeRunner, TerminationReason
from frontier_search.validation import ContractError
from tests.unit.test_mapper_update import request, scene_frame


class TargetClaimTests(unittest.TestCase):
    def test_retained_scan_hypothesis_cannot_confirm_with_stale_evidence(self):
        self.mapper.reset(request(confidence_threshold=.5))
        self.assertIsNone(self.update(1, time=70.333).target_claim)
        state = self.update(2, time=84.1, detections=())
        self.assertTrue(state.semantic_objects)
        self.assertIsNone(state.target_claim)
        # Reacquisition needs two fresh frames, not the pre-scan observation.
        self.assertIsNone(self.update(3, time=84.5).target_claim)
        self.assertIsNotNone(self.update(4, time=85.).target_claim)

    def test_repeated_weak_detections_remain_tracks_without_claim(self):
        self.mapper.reset(request(confidence_threshold=.5))
        weak = replace(self.detection, confidence=.54)
        for sequence in range(1, 5):
            state = self.update(sequence, detections=(weak,))
            self.assertIsNone(state.target_claim)
        self.assertEqual(state.semantic_objects[0].confirmation_count, 4)
        # One stronger frame cannot reuse weak evidence to make a claim.
        self.assertIsNone(self.update(5).target_claim)
        claim = self.update(6).target_claim
        self.assertIsNotNone(claim)
        bundle = self.mapper.get_claim_evidence('mapping-test', claim.claim_id)
        self.assertEqual([d.capture_sequence for d in bundle.detections], [5, 6])

    def test_claim_threshold_is_validated(self):
        for value in (-.1, 1.1, float('nan'), True):
            with self.subTest(value=value), self.assertRaises(ValueError):
                MappingConfig(claim_confidence_threshold=value)

    def setUp(self):
        self.detector = Mock()
        self.detection = RGBDetection('cup', .9, (28, 28, 33, 33))
        self.mapper = RGBDMapper(self.detector)
        self.mapper.reset(request())

    def update(self, sequence, time=None, position=Point3(1., 0., .5), detections=None):
        self.detector.detect.return_value = (self.detection,) if detections is None else detections
        observation = scene_frame(sequence=sequence)
        if time is not None:
            observation = replace(observation, timestamp_s=time)
        with patch('frontier_search.perception.tracking.localize_detection', return_value=position):
            return self.mapper.update(observation)

    def test_confirm_two_frames_and_return_exact_immutable_evidence(self):
        self.assertIsNone(self.update(1).target_claim)
        state = self.update(2)
        claim = state.target_claim
        self.assertEqual(claim.claim_id, 'claim-000001')
        self.assertEqual(claim.track_id, state.semantic_objects[0].track_id)
        self.assertEqual(claim.timestamp_s, 2.)
        bundle = self.mapper.get_claim_evidence('mapping-test', claim.claim_id)
        self.assertEqual(tuple(d.observation_id for d in bundle.detections), claim.evidence_observation_ids)
        self.assertEqual(tuple(d.capture_sequence for d in bundle.detections), (1, 2))
        self.assertTrue(all(d.position_world is not None and d.class_name == 'cup' for d in bundle.detections))
        require_immutable(bundle)
        with self.assertRaises(FrozenInstanceError):
            bundle.claim_id = 'changed'

    def test_window_boundary_is_inclusive_and_stale_history_cannot_confirm(self):
        self.update(1, time=0.)
        self.assertIsNotNone(self.update(2, time=2.).target_claim)
        self.mapper.reset(request())
        self.update(1, time=0.)
        state = self.update(2, time=2.001)
        self.assertEqual(state.semantic_objects[0].confirmation_count, 2)
        self.assertIsNone(state.target_claim)
        claim = self.update(3, time=3.).target_claim
        bundle = self.mapper.get_claim_evidence('mapping-test', claim.claim_id)
        self.assertEqual([d.capture_sequence for d in bundle.detections], [2, 3])

    def test_configured_frame_count_uses_only_recent_required_evidence(self):
        self.mapper.reset(request(confirmation_frames=3, confirmation_window_s=5.))
        self.update(1)
        self.assertIsNone(self.update(2).target_claim)
        self.assertEqual(len(self.update(3).target_claim.evidence_observation_ids), 3)

    def test_fractional_timestamp_boundary_tolerates_roundoff_but_not_stale_frames(self):
        self.assertGreater(8.3 - 6.3, 2.)  # Reproduces the original missed claim.
        self.update(1, time=6.3)
        self.assertIsNotNone(self.update(2, time=8.3).target_claim)
        self.mapper.reset(request())
        self.update(1, time=6.3)
        outside = 8.3 + 10 * CONFIRMATION_TIME_TOLERANCE_S
        self.assertIsNone(self.update(2, time=outside).target_claim)

    def test_distinct_sequences_with_equal_timestamps_are_valid(self):
        self.update(10, time=1.)
        self.assertIsNotNone(self.update(20, time=1.).target_claim)

    def test_duplicate_sequence_rejected_before_inference_and_cannot_confirm(self):
        self.update(1)
        calls = self.detector.detect.call_count
        with self.assertRaises(ContractError):
            self.update(1)
        self.assertEqual(self.detector.detect.call_count, calls)
        self.assertIsNone(self.mapper._state.target_claim)
        self.assertEqual(self.mapper._state.tracks['track-000001'].confirmation_count, 1)

    def test_duplicate_boxes_do_not_confirm_on_one_frame(self):
        state = self.update(1, detections=(self.detection, self.detection,
                                         replace(self.detection, confidence=.95)))
        self.assertEqual(len(state.semantic_objects), 1)
        self.assertIsNone(state.target_claim)

    def test_other_class_low_confidence_and_unknown_depth_do_not_confirm(self):
        for detection, position in ((replace(self.detection, class_name='chair'), Point3(1., 0., .5)),
                                   (replace(self.detection, confidence=.79), Point3(1., 0., .5)),
                                   (self.detection, None)):
            with self.subTest(detection=detection, position=position):
                self.mapper.reset(request())
                self.update(1, position=position, detections=(detection,))
                self.assertIsNone(self.update(2, position=position, detections=(detection,)).target_claim)

    def test_different_positions_and_tracks_cannot_pool_evidence(self):
        self.update(1, position=Point3(1., 0., .5))
        self.assertIsNone(self.update(2, position=Point3(2., 0., .5)).target_claim)

    def test_pairwise_gate_rejects_opposite_sides_of_track_anchor(self):
        self.mapper.reset(request(confirmation_frames=3, confirmation_window_s=5.))
        self.update(1, position=Point3(1., 0., .5))
        self.update(2, position=Point3(1.2, 0., .5))
        state = self.update(3, position=Point3(.8, 0., .5))
        self.assertEqual(len(state.semantic_objects), 1)
        self.assertEqual(state.semantic_objects[0].confirmation_count, 3)
        self.assertIsNone(state.target_claim)

    def test_missing_detection_never_supplies_a_confirmation_frame(self):
        self.update(1)
        self.assertIsNone(self.update(2, detections=()).target_claim)
        self.assertIsNotNone(self.update(3).target_claim)

    def test_claim_and_evidence_latch_after_new_objects_misses_and_window_expiry(self):
        self.update(1)
        claim = self.update(2).target_claim
        bundle = self.mapper.get_claim_evidence('mapping-test', claim.claim_id)
        state = self.update(3, time=100., position=Point3(3., 0., .5))
        self.assertIs(state.target_claim, claim)
        self.assertIs(self.update(4, time=101., detections=()).target_claim, claim)
        self.assertIs(self.mapper.get_claim_evidence('mapping-test', claim.claim_id), bundle)
        self.assertEqual(self.mapper._state.entity_counters['claim'], 1)
        self.assertEqual(len(bundle.detections), 2)

    def test_simultaneous_targets_choose_stable_track_id(self):
        detections = (self.detection, replace(self.detection, bbox_xyxy_px=(40, 40, 45, 45)))
        def locate(d, *_):
            return Point3(1. if d.bbox_xyxy_px[0] == 28 else 2., 0., .5)
        with patch('frontier_search.perception.tracking.localize_detection', side_effect=locate):
            self.detector.detect.return_value = detections
            self.mapper.update(scene_frame(sequence=1))
            self.detector.detect.return_value = tuple(reversed(detections))
            state = self.mapper.update(scene_frame(sequence=2))
        self.assertEqual(state.target_claim.track_id, 'track-000001')

    def test_late_failure_does_not_publish_claim_or_consume_counter(self):
        self.update(1)
        previous = self.mapper._state
        with patch('frontier_search.perception.mapper.build_candidates', side_effect=RuntimeError('late failure')):
            with self.assertRaises(RuntimeError):
                self.update(2)
        self.assertIs(self.mapper._state, previous)
        self.assertIsNone(previous.target_claim)
        self.assertIsNone(previous.claim_evidence)
        self.assertEqual(previous.entity_counters['claim'], 0)
        self.assertEqual(self.update(2).target_claim.claim_id, 'claim-000001')

    def test_reset_and_evidence_identity_errors_are_explicit(self):
        for mapper in (RGBDMapper(), self.mapper):
            with self.assertRaises(ContractError):
                mapper.get_claim_evidence('mapping-test', 'claim-000001')
        self.update(1)
        claim = self.update(2).target_claim
        with self.assertRaisesRegex(ContractError, 'episode mismatch'):
            self.mapper.get_claim_evidence('other-episode', claim.claim_id)
        with self.assertRaisesRegex(ContractError, 'unknown claim'):
            self.mapper.get_claim_evidence('mapping-test', 'other-claim')
        self.mapper.reset(request())
        self.assertIsNone(self.mapper._state.claim_evidence)
        self.assertIsNone(self.update(1).target_claim)
        with self.assertRaises(ContractError):
            self.mapper.get_claim_evidence('mapping-test', claim.claim_id)

    def test_invalid_confirmation_config_is_rejected(self):
        for frames in (True, 1, 2.5, float('nan'), '2'):
            with self.subTest(frames=frames), self.assertRaises(ValueError):
                MappingConfig(confirmation_frames=frames)
        for value in (True, float('nan'), float('inf'), '1'):
            with self.subTest(window=value), self.assertRaises(ValueError):
                MappingConfig(confirmation_window_s=value)
            with self.subTest(confidence=value), self.assertRaises(ValueError):
                MappingConfig(confidence_threshold=value)


class ClaimManifestTests(unittest.TestCase):
    def test_incompatible_counts_rejected_before_simulator_initialization(self):
        manifest = mock_manifest()
        manifest = replace(manifest, enable_target_evaluation=True,
                           evaluation=replace(manifest.evaluation, required_evidence_frames=3))
        with self.assertRaisesRegex(ContractError, 'confirmation_frames must be at least'):
            manifest.validate()
        simulator, mapper, evaluator = Mock(), Mock(), Mock()
        runner = EpisodeRunner(simulator, mapper, Mock(), Mock(), evaluator=evaluator)
        self.assertEqual(runner.run(manifest), TerminationReason.INITIALIZATION_FAILED)
        simulator.reset.assert_not_called()
        mapper.reset.assert_not_called()
        evaluator.evaluate.assert_not_called()

    def test_equal_or_stricter_confirmation_counts_are_valid(self):
        base = mock_manifest()
        for confirmation, required in ((2, 2), (3, 3), (3, 2)):
            with self.subTest(confirmation=confirmation, required=required):
                replace(base, enable_target_evaluation=True,
                        mapping=replace(base.mapping, confirmation_frames=confirmation),
                        evaluation=replace(base.evaluation, required_evidence_frames=required)).validate()

    def test_disabled_evaluation_does_not_require_count_compatibility(self):
        base = mock_manifest()
        replace(base, enable_target_evaluation=False,
                evaluation=replace(base.evaluation, required_evidence_frames=3)).validate()

    def test_evaluator_count_requires_integer_at_least_two(self):
        base = mock_manifest()
        for value in (True, 1, 2.5, '3'):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, 'required evidence frames'):
                replace(base, evaluation=replace(base.evaluation, required_evidence_frames=value)).validate()
        with self.assertRaisesRegex(ValueError, 'required evidence frames'):
            replace(base, evaluation=replace(base.evaluation, required_evidence_frames=float('nan'))).validate()
