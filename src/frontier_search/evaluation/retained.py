"""Evaluation-only retained capture records. Never pass these to perception/policies."""

from dataclasses import dataclass
from typing import Optional, Protocol, Tuple

from ..contracts import Point3
from ..immutable import ImmutableRecord
from ..validation import require, is_finite_number


@dataclass(frozen=True)
class VisibleInstance(ImmutableRecord):
    instance_id: str
    class_name: str
    # Fraction of the instance's projected silhouette visible in this capture.
    visibility: float
    bbox_xyxy_px: Tuple[int, int, int, int]
    # Independently measured visible-surface reference, not object centre.
    position_world: Optional[Point3]


@dataclass(frozen=True)
class GroundTruthFrame(ImmutableRecord):
    episode_id: str
    capture_sequence: int
    timestamp_s: float
    width: int
    height: int
    instances: Tuple[VisibleInstance, ...]


def validate_box(box, width, height):
    require(isinstance(box, tuple) and len(box) == 4 and all(type(x) is int for x in box),
            'bounding box requires four integer coordinates')
    x1, y1, x2, y2 = box
    require(0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height, 'invalid bounding box bounds')


def validate_frame(frame):
    require(isinstance(frame, GroundTruthFrame), 'missing ground-truth frame')
    require(isinstance(frame.episode_id, str) and bool(frame.episode_id.strip()), 'missing ground-truth episode')
    require(type(frame.capture_sequence) is int and frame.capture_sequence >= 0, 'invalid capture sequence')
    require(is_finite_number(frame.timestamp_s) and frame.timestamp_s >= 0, 'invalid capture timestamp')
    require(type(frame.width) is int and type(frame.height) is int and min(frame.width, frame.height) > 0,
            'invalid capture dimensions')
    ids = set()
    for item in frame.instances:
        require(isinstance(item, VisibleInstance), 'invalid visible instance')
        require(isinstance(item.instance_id, str) and bool(item.instance_id.strip())
                and item.instance_id not in ids, 'missing or duplicate instance ID')
        ids.add(item.instance_id)
        require(isinstance(item.class_name, str) and bool(item.class_name.strip()), 'missing instance class')
        require(is_finite_number(item.visibility) and 0 <= item.visibility <= 1, 'invalid visibility')
        validate_box(item.bbox_xyxy_px, frame.width, frame.height)
        require(item.position_world is None or isinstance(item.position_world, Point3)
                and all(is_finite_number(v) for v in (item.position_world.x_m, item.position_world.y_m,
                                                     item.position_world.z_m)), 'invalid surface position')


class GroundTruthSource(Protocol):
    def get_frame(self, episode_id: str, capture_sequence: int) -> GroundTruthFrame: ...


class RetainedFrames:
    """Append-only episode/sequence store populated by the capture adapter.

    Retains all frames until the owner discards the store, including through
    terminal scoring. Missing records are infrastructure errors, not negatives.
    """
    def __init__(self):
        self._frames = {}

    def retain(self, frame: GroundTruthFrame) -> None:
        validate_frame(frame)
        key = (frame.episode_id, frame.capture_sequence)
        require(key not in self._frames, 'ground-truth capture already retained')
        self._frames[key] = frame

    def get_frame(self, episode_id: str, capture_sequence: int) -> GroundTruthFrame:
        try:
            return self._frames[(episode_id, capture_sequence)]
        except KeyError as exc:
            raise LookupError('ground-truth capture unavailable') from exc
