"""Confirm an observed target without access to evaluator or simulator truth."""

from itertools import combinations
from math import hypot
from typing import Dict, Optional

from ..config import MappingConfig
from ..contracts import DetectionEvidence, ObjectTrack, TargetClaim


# Absolute one-nanosecond slack at the upper boundary absorbs timestamp
# subtraction roundoff. Future timestamps remain ineligible (no lower slack).
CONFIRMATION_TIME_TOLERANCE_S = 1e-9


def confirm_target(tracks: Dict[str, ObjectTrack], evidence: Dict[str, DetectionEvidence],
                   target_class: str, sequence: int, timestamp_s: float,
                   cfg: MappingConfig, claim_counter: int, eligible_ids=None) -> Optional[TargetClaim]:
    """Use the most recent N distinct accepted frames within the closed window.

    Require a fresh observation on this update and pairwise spatial consistency
    among the selected measurements. Stable track ID breaks simultaneous ties.
    Lifetime confirmation_count and smoothed track confidence are not sufficient.
    """
    for track_id in sorted(tracks):
        track = tracks[track_id]
        if track.class_name != target_class or track.position_world is None:
            continue
        by_sequence = {}
        for observation_id in track.observation_ids:
            item = evidence[observation_id]
            if eligible_ids is not None and observation_id not in eligible_ids:
                continue
            if (item.track_id == track_id and item.class_name == target_class
                    and item.position_world is not None
                    and item.confidence >= max(cfg.confidence_threshold,
                                               cfg.claim_confidence_threshold)
                    and 0 <= timestamp_s - item.timestamp_s <= (
                        cfg.confirmation_window_s + CONFIRMATION_TIME_TOLERANCE_S)
                    and item.capture_sequence <= sequence):
                by_sequence[item.capture_sequence] = item
        samples = sorted(by_sequence.values(), key=lambda item: item.capture_sequence)
        samples = samples[-cfg.confirmation_frames:]
        if len(samples) < cfg.confirmation_frames or samples[-1].capture_sequence != sequence:
            continue
        if eligible_ids is not None and len({s.timestamp_s for s in samples}) < cfg.confirmation_frames:
            continue
        consistent = True
        for a, b in combinations(samples, 2):
            p, q = a.position_world, b.position_world
            if hypot(hypot(p.x_m - q.x_m, p.y_m - q.y_m), p.z_m - q.z_m) > cfg.object_match_distance_m:
                consistent = False
                break
        if consistent:
            return TargetClaim('claim-%06d' % (claim_counter + 1), track_id,
                target_class, timestamp_s, tuple(item.observation_id for item in samples))
    return None
