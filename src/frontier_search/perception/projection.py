"""Project aligned axial depth into world coordinates using capture-time calibration."""

from dataclasses import dataclass
from math import isfinite
from struct import iter_unpack
from typing import Tuple

from ..contracts import Point3, RGBDObservation
from ..immutable import ImmutableRecord
from ..validation import require
from .observations import validate_depth_calibration


@dataclass(frozen=True)
class ProjectedDepthPoint(ImmutableRecord):
    """A visible surface point with its source pixel and axial depth."""

    u: int
    v: int
    depth_m: float
    position_world: Point3


# Convert valid depth pixels into world-space surface points with pixel correspondence.
def project_depth_to_world(observation: RGBDObservation) -> Tuple[ProjectedDepthPoint, ...]:
    """Accept depth32f_le axial metres and return points in image row order.

    Pixel centres are integer (u, v); camera axes are right/down/forward. The
    supplied camera-to-world transform includes the camera mount and robot pose.
    RGB and robot_pose are not used. Zero, negative and nonfinite depths are
    omitted; an entirely invalid depth image returns an empty tuple.
    """
    validate_depth_calibration(observation)
    camera = observation.camera_intrinsics
    matrix = observation.camera_to_world.matrix
    points = []
    for index, (depth_m,) in enumerate(iter_unpack("<f", observation.depth_image.data)):
        if not isfinite(depth_m) or depth_m <= 0:
            continue
        v, u = divmod(index, camera.width)
        x = (u - camera.cx) * depth_m / camera.fx
        y = (v - camera.cy) * depth_m / camera.fy
        world = tuple(row[0] * x + row[1] * y + row[2] * depth_m + row[3]
                      for row in matrix[:3])
        require(all(isfinite(value) for value in world), "projection overflowed world coordinates")
        points.append(ProjectedDepthPoint(u, v, depth_m, Point3(*world)))
    return tuple(points)
