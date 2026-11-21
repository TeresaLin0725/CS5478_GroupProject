"""Independent truth matching and production-mapper terminal scoring seams."""

from dataclasses import replace
import unittest
from unittest.mock import Mock

from frontier_search.config import EvaluationConfig
from frontier_search.contracts import EvaluationRequest, Point3
from frontier_search.evaluation.retained import GroundTruthFrame, RetainedFrames, VisibleInstance
from frontier_search.evaluation.verifier import RetainedTargetEvaluator
from frontier_search.runner.episode import EpisodeRunner, TerminationReason
from frontier_search.fixtures.w2_episode import mock_manifest
from frontier_search.validation import ContractError
from tests.unit import test_target_claims as claim_fixtures


class TargetEvaluatorTests(unittest.TestCase):
    def setUp(self):
        fixture = claim_fixtures.TargetClaimTests()
        fixture.setUp()
        fixture.update(1)
        self.state = fixture.update(2)
        self.mapper = fixture.mapper
        self.request = EvaluationRequest('mapping-test', self.state.target_claim,
            self.mapper.get_claim_evidence('mapping-test', self.state.target_claim.claim_id))
        # Authored independent truth, never passed to the detector/mapper.
        self.truth = VisibleInstance('instance-cup', 'cup', .5, (28, 28, 33, 33), Point3(1., 0., .5))
        self.frames = tuple(GroundTruthFrame('mapping-test', i, float(i), 61, 61, (self.truth,)) for i in (1, 2))

    def evaluator(self, frames=None, **config):
        store = RetainedFrames()
        for frame in self.frames if frames is None else frames:
            store.retain(frame)
        evaluator = RetainedTargetEvaluator(store, 'cup')
        evaluator.reset('mapping-test', EvaluationConfig(**config))
        return evaluator

    def test_true_claim_matches_one_visible_instance_and_is_idempotent(self):
        evaluator = self.evaluator()
        result = evaluator.evaluate(self.request)
        self.assertTrue(result.valid_target)
        self.assertEqual(result.matched_instance_id, 'instance-cup')
        self.assertIs(evaluator.evaluate(self.request), result)

    def test_wrong_class_visibility_box_or_position_is_false(self):
        alternatives = (replace(self.truth, class_name='chair'),
                        replace(self.truth, visibility=.09),
                        replace(self.truth, bbox_xyxy_px=(40, 40, 45, 45)),
                        replace(self.truth, position_world=Point3(3., 0., .5)))
        for truth in alternatives:
            frames = tuple(replace(frame, instances=(truth,)) for frame in self.frames)
            with self.subTest(truth=truth):
                result = self.evaluator(frames).evaluate(self.request)
                self.assertFalse(result.valid_target)
                self.assertIsNone(result.matched_instance_id)

    def test_target_absent_from_one_frame_and_switched_instances_are_false(self):
        for instances in ((), (replace(self.truth, instance_id='another-cup'),)):
            frames = (self.frames[0], replace(self.frames[1], instances=instances))
            self.assertFalse(self.evaluator(frames).evaluate(self.request).valid_target)

    def test_presence_elsewhere_cannot_validate_claim(self):
        distant = replace(self.truth, bbox_xyxy_px=(40, 40, 45, 45), position_world=Point3(2., 0., .5))
        frames = tuple(replace(frame, instances=(distant,)) for frame in self.frames)
        self.assertFalse(self.evaluator(frames).evaluate(self.request).valid_target)

    def test_thresholds_include_exact_boundary(self):
        truth = replace(self.truth, visibility=.1, position_world=Point3(1.25, 0., .5))
        frames = tuple(replace(frame, instances=(truth,)) for frame in self.frames)
        self.assertTrue(self.evaluator(frames, minimum_bbox_iou=1.).evaluate(self.request).valid_target)

    def test_insufficient_distinct_frames_is_false(self):
        result = self.evaluator(required_evidence_frames=3).evaluate(self.request)
        self.assertFalse(result.valid_target)
        self.assertEqual(result.reason, 'insufficient_evidence_frames')

    def test_missing_truth_is_error_even_if_another_frame_is_negative(self):
        evaluator = self.evaluator((replace(self.frames[0], instances=()),))
        with self.assertRaises(LookupError):
            evaluator.evaluate(self.request)

    def test_wrong_capture_identity_timestamp_and_missing_reference_are_errors(self):
        for frame in (replace(self.frames[0], episode_id='other'),
                      replace(self.frames[0], capture_sequence=99),
                      replace(self.frames[0], timestamp_s=1.01),
                      replace(self.frames[0], instances=(replace(self.truth, position_world=None),))):
            source = Mock()
            source.get_frame.return_value = frame
            evaluator = RetainedTargetEvaluator(source, 'cup')
            evaluator.reset('mapping-test', EvaluationConfig())
            with self.subTest(frame=frame), self.assertRaises(ContractError):
                evaluator.evaluate(self.request)

    def test_malformed_claims_do_not_score(self):
        bundle = self.request.evidence
        cases = (
            replace(self.request, episode_id='other'),
            replace(self.request, claim=replace(self.request.claim, target_class='chair')),
            replace(self.request, evidence=replace(bundle, claim_id='other')),
            replace(self.request, evidence=replace(bundle, detections=(bundle.detections[0],))),
            replace(self.request, evidence=replace(bundle, detections=(bundle.detections[0],
                replace(bundle.detections[1], capture_sequence=1)))),
            replace(self.request, evidence=replace(bundle, detections=(bundle.detections[0],
                replace(bundle.detections[1], position_world=None)))),
        )
        for request in cases:
            with self.subTest(request=request), self.assertRaises(ContractError):
                self.evaluator().evaluate(request)

    def test_retention_is_write_once_and_episode_scoped(self):
        store = RetainedFrames()
        store.retain(self.frames[0])
        with self.assertRaises(ContractError):
            store.retain(self.frames[0])
        with self.assertRaises(LookupError):
            store.get_frame('other', 1)
        with self.assertRaises(ContractError):
            store.retain(replace(self.frames[1], instances=(self.truth, self.truth)))

    def test_terminal_latch_and_reset(self):
        evaluator = self.evaluator()
        evaluator.evaluate(self.request)
        changed = replace(self.request, claim=replace(self.request.claim, claim_id='claim-2'),
                          evidence=replace(self.request.evidence, claim_id='claim-2'))
        with self.assertRaisesRegex(ContractError, 'already completed'):
            evaluator.evaluate(changed)
        evaluator.reset('mapping-test', EvaluationConfig())
        self.assertTrue(evaluator.evaluate(changed).valid_target)
        with self.assertRaises(ContractError):
            RetainedTargetEvaluator(RetainedFrames(), 'cup').evaluate(self.request)

    def test_runner_stops_cancels_and_scores_production_mapper_claim(self):
        for valid in (True, False):
            frames = self.frames if valid else tuple(replace(f, instances=()) for f in self.frames)
            simulator, navigator, policy = Mock(), Mock(), Mock()
            runner = EpisodeRunner(simulator, self.mapper, navigator, policy, evaluator=self.evaluator(frames))
            runner.manifest = replace(mock_manifest(), episode_id='mapping-test', enable_target_evaluation=True)
            runner.state, runner.active = self.state, 'active-goal'
            runner._call = lambda name, fn, *args, **kwargs: fn(*args)
            runner._emit = Mock()
            result = runner._score()
            self.assertEqual(result, TerminationReason.SUCCESS if valid else TerminationReason.FALSE_TARGET_CLAIM)
            simulator.stop.assert_called_once()
            navigator.cancel.assert_called_once_with('mapping-test', 'active-goal', 'target_claim')
            policy.select.assert_not_called()
            self.assertIsNone(runner.active)

    def test_config_rejects_invalid_scoring_parameters(self):
        for field, values in (('minimum_visibility', (0, 1.1, True)),
                              ('minimum_bbox_iou', (0, -1, float('nan'))),
                              ('localization_tolerance_m', (0, float('inf'))),
                              ('deadline_s', (-1, True)), ('required_evidence_frames', (1, 2.5))):
            for value in values:
                with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                    EvaluationConfig(**{field: value})
