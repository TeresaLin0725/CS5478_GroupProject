"""Independent terminal claim scoring using capture-bound ground-truth records."""

from math import hypot

from ..config import EvaluationConfig
from ..contracts import (DetectionEvidence, EvidenceBundle, EvaluationRequest,
                         EvaluationResult, Point3, TargetClaim)
from ..immutable import require_immutable
from ..validation import require, is_finite_number
from .retained import GroundTruthSource, validate_box, validate_frame


def _iou(a, b):
    overlap = max(0, min(a[2], b[2]) - max(a[0], b[0])) * max(0, min(a[3], b[3]) - max(a[1], b[1]))
    area_a, area_b = (a[2] - a[0]) * (a[3] - a[1]), (b[2] - b[0]) * (b[3] - b[1])
    return overlap / (area_a + area_b - overlap)


class RetainedTargetEvaluator:
    """TargetEvaluator implementation; construct with the manifest's target class.

    Every cited frame must match the same visible instance by exact class,
    visibility, box IoU AND distance to an independent surface reference.
    Missing/malformed truth raises an error so the runner records evaluation_error.
    A valid negative returns False so the runner records false_target_claim.
    """
    def __init__(self, source: GroundTruthSource, target_class: str):
        require(isinstance(target_class, str) and bool(target_class.strip()), 'missing evaluator target')
        self._source = source
        self._target_class = target_class
        self._episode_id = None
        self._config = None
        self._request = None
        self._result = None

    def reset(self, episode_id: str, config: EvaluationConfig) -> None:
        require(isinstance(episode_id, str) and bool(episode_id.strip()), 'missing evaluation episode')
        require(isinstance(config, EvaluationConfig), 'missing evaluation config')
        config.__post_init__()
        self._episode_id, self._config = episode_id, config
        self._request = self._result = None

    def evaluate(self, request: EvaluationRequest) -> EvaluationResult:
        require(self._episode_id is not None, 'evaluator must be reset')
        require(isinstance(request, EvaluationRequest), 'invalid evaluation request')
        require_immutable(request)
        claim, bundle = request.claim, request.evidence
        require(isinstance(claim, TargetClaim) and isinstance(bundle, EvidenceBundle)
                and all(isinstance(item, DetectionEvidence) for item in bundle.detections),
                'invalid claim or evidence records')
        require(request.episode_id == bundle.episode_id == self._episode_id, 'evaluation episode mismatch')
        require(bool(claim.claim_id) and claim.claim_id == bundle.claim_id, 'evaluation claim mismatch')
        require(claim.target_class == self._target_class, 'evaluation target mismatch')
        require(is_finite_number(claim.timestamp_s) and claim.timestamp_s >= 0, 'invalid claim timestamp')
        require(bool(claim.track_id), 'missing claimed track')
        ids = tuple(item.observation_id for item in bundle.detections)
        require(bool(ids) and all(ids) and len(set(ids)) == len(ids)
                and len(set(claim.evidence_observation_ids)) == len(claim.evidence_observation_ids)
                and set(ids) == set(claim.evidence_observation_ids), 'claim evidence IDs mismatch')
        sequences = tuple(item.capture_sequence for item in bundle.detections)
        require(all(type(seq) is int and seq >= 0 for seq in sequences)
                and len(set(sequences)) == len(sequences), 'claim needs distinct capture sequences')
        for item in bundle.detections:
            require(item.track_id == claim.track_id and item.class_name == claim.target_class,
                    'claim evidence track/class mismatch')
            require(is_finite_number(item.timestamp_s) and 0 <= item.timestamp_s <= claim.timestamp_s,
                    'invalid evidence timestamp')
            require(is_finite_number(item.confidence) and 0 <= item.confidence <= 1, 'invalid evidence confidence')
            require(isinstance(item.position_world, Point3)
                    and all(is_finite_number(value) for value in (item.position_world.x_m,
                        item.position_world.y_m, item.position_world.z_m)), 'claim evidence must be localized')
        if self._request is not None:
            require(request == self._request, 'terminal evaluation already completed for another request')
            return self._result
        cfg = self._config
        common = None
        # Fetch/validate every claimed capture before returning a negative, so
        # missing records cannot be silently reclassified as false detections.
        for item in bundle.detections:
            frame = self._source.get_frame(request.episode_id, item.capture_sequence)
            validate_frame(frame)
            require(frame.episode_id == request.episode_id and frame.capture_sequence == item.capture_sequence,
                    'ground-truth capture identity mismatch')
            require(abs(frame.timestamp_s - item.timestamp_s) <= 1e-9, 'ground-truth timestamp mismatch')
            validate_box(item.bbox_xyxy_px, frame.width, frame.height)
            matches = set()
            for truth in frame.instances:
                if truth.class_name != self._target_class or truth.visibility < cfg.minimum_visibility:
                    continue
                require(truth.position_world is not None, 'visible target lacks surface reference')
                p, q = item.position_world, truth.position_world
                distance = hypot(hypot(p.x_m - q.x_m, p.y_m - q.y_m), p.z_m - q.z_m)
                if (_iou(item.bbox_xyxy_px, truth.bbox_xyxy_px) >= cfg.minimum_bbox_iou
                        and distance <= cfg.localization_tolerance_m):
                    matches.add(truth.instance_id)
            common = matches if common is None else common & matches
        enough = len(sequences) >= cfg.required_evidence_frames
        valid = enough and bool(common)
        result = EvaluationResult(request.episode_id, claim.claim_id, valid,
            min(common) if valid else None,
            'visible_instance_matched' if valid else 'insufficient_evidence_frames' if not enough
            else 'no_consistent_visible_instance')
        self._request, self._result = request, result
        return result
