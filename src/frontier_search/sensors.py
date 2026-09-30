"""Owned immutable sensor values; actual encoding must be frozen by adapter B."""


from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class ImageBuffer:
    width: int
    height: int
    encoding: str
    data: bytes


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: Tuple[float, ...] = ()


@dataclass(frozen=True)
class Transform:
    """Row-major 4x4 camera-to-world matrix; column vectors, metres."""

    matrix: Tuple[Tuple[float, ...], ...]
