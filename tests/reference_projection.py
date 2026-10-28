# Frozen projector from 254279b; independent oracle for sampling optimization.
"""Project aligned axial depth into world coordinates using capture-time calibration."""

from dataclasses import dataclass
from math import isfinite
from struct import iter_unpack
from typing import Iterator, Tuple

from frontier_search.contracts import Point3, RGBDObservation
from frontier_search.immutable import ImmutableRecord
from frontier_search.validation import require
from frontier_search.perception.observations import validate_depth_calibration


@dataclass(frozen=True)
class ProjectedDepthPoint(ImmutableRecord):
    """A visible surface point with its source pixel and axial depth."""

    u: int
    v: int
    depth_m: float
    position_world: Point3


# Convert valid depth pixels into world-space surface points with pixel correspondence.
def iter_depth_world(observation: RGBDObservation, stride: int = 1) -> Iterator[Tuple[int, int, float, float, float, float]]:
    """Stream (u, v, depth, world x, y, z) without allocating per-pixel records.

    Pixel centres are integer (u, v); camera axes are right/down/forward. The
    supplied camera-to-world transform includes the camera mount and robot pose.
    RGB and robot_pose are not used. Zero, negative and nonfinite depths are
    omitted; an entirely invalid depth image yields no points.
    """
    validate_depth_calibration(observation)
    camera = observation.camera_intrinsics
    matrix = observation.camera_to_world.matrix
    a, b, c = matrix[:3]
    for index, (depth_m,) in enumerate(iter_unpack("<f", observation.depth_image.data)):
        if not isfinite(depth_m) or depth_m <= 0:
            continue
        v, u = divmod(index, camera.width)
        if v % stride or u % stride:
            continue
        x = (u - camera.cx) * depth_m / camera.fx
        y = (v - camera.cy) * depth_m / camera.fy
        wx = a[0] * x + a[1] * y + a[2] * depth_m + a[3]
        wy = b[0] * x + b[1] * y + b[2] * depth_m + b[3]
        wz = c[0] * x + c[1] * y + c[2] * depth_m + c[3]
        require(isfinite(wx) and isfinite(wy) and isfinite(wz), "projection overflowed world coordinates")
        yield u, v, depth_m, wx, wy, wz


def project_depth_to_world(observation: RGBDObservation) -> Tuple[ProjectedDepthPoint, ...]:
    """Return immutable surface records in image order, using the streaming projector."""
    return tuple(ProjectedDepthPoint(u, v, depth, Point3(x, y, z))
                 for u, v, depth, x, y, z in iter_depth_world(observation))
