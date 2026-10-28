"""Project aligned axial depth into world coordinates using capture-time calibration."""

from dataclasses import dataclass
from functools import lru_cache
from math import isfinite
from struct import Struct
from typing import Iterator, Tuple

try:
    import numpy as np
except ImportError:
    np = None

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


@lru_cache(maxsize=8)
def _sample_axes(camera, stride):
    """Cache sampled pixels and offsets, without changing multiply/divide order."""
    us = tuple(range(0, camera.width, stride))
    vs = tuple(range(0, camera.height, stride))
    return us, vs, tuple(u - camera.cx for u in us), tuple(v - camera.cy for v in vs)


@lru_cache(maxsize=8)
def _numpy_sample_axes(camera, stride):
    us, vs, dx, dy = _sample_axes(camera, stride)
    axes = (np.asarray(us, dtype=np.int64), np.asarray(vs, dtype=np.int64),
            np.asarray(dx, dtype=np.float64), np.asarray(dy, dtype=np.float64))
    for axis in axes:
        axis.setflags(write=False)
    return axes


def _iter_depth_world_scalar(observation, stride):
    camera = observation.camera_intrinsics
    a, b, c = observation.camera_to_world.matrix[:3]
    us, vs, dx, dy = _sample_axes(camera, stride)
    unpack = Struct('<f').unpack_from
    data = observation.depth_image.data
    for v, y_offset in zip(vs, dy):
        row_offset = v * camera.width
        for u, x_offset in zip(us, dx):
            depth = unpack(data, 4 * (row_offset + u))[0]
            if not isfinite(depth) or depth <= 0:
                continue
            x = x_offset * depth / camera.fx
            y = y_offset * depth / camera.fy
            wx = a[0] * x + a[1] * y + a[2] * depth + a[3]
            wy = b[0] * x + b[1] * y + b[2] * depth + b[3]
            wz = c[0] * x + c[1] * y + c[2] * depth + c[3]
            require(isfinite(wx) and isfinite(wy) and isfinite(wz), "projection overflowed world coordinates")
            yield u, v, depth, wx, wy, wz


def _iter_depth_world_numpy(observation, stride):
    camera = observation.camera_intrinsics
    a, b, c = observation.camera_to_world.matrix[:3]
    us, vs, dx, dy = _numpy_sample_axes(camera, stride)
    # This is a strided view of the original immutable bytes, not a full image scan.
    sampled = np.frombuffer(observation.depth_image.data, dtype='<f4').reshape(
        camera.height, camera.width)[::stride, ::stride]
    # Bound temporary arrays/Python lists even when stride=1 on large recordings.
    for first in range(0, len(vs), 64):
        block = sampled[first:first + 64]
        with np.errstate(all='ignore'):
            valid = np.isfinite(block) & (block > 0)
        depth = block[valid].astype(np.float64)
        shape = block.shape
        u = np.broadcast_to(us, shape)[valid]
        v = np.broadcast_to(vs[first:first + 64, None], shape)[valid]
        # Separate ufunc operations preserve the scalar's left-to-right order;
        # do not normalize offsets first, use matmul, float32 arithmetic or FMA.
        with np.errstate(all='ignore'):
            x = np.broadcast_to(dx, shape)[valid] * depth / camera.fx
            y = np.broadcast_to(dy[first:first + 64, None], shape)[valid] * depth / camera.fy
            wx = a[0] * x + a[1] * y + a[2] * depth + a[3]
            wy = b[0] * x + b[1] * y + b[2] * depth + b[3]
            wz = c[0] * x + c[1] * y + c[2] * depth + c[3]
        points = zip(u.tolist(), v.tolist(), depth.tolist(), wx.tolist(), wy.tolist(), wz.tolist())
        if np.all(np.isfinite(wx) & np.isfinite(wy) & np.isfinite(wz)):
            yield from points
        else:
            # Match streaming failure behavior: earlier finite points are yielded
            # before the first overflowing point raises ContractError.
            for point in points:
                require(all(isfinite(value) for value in point[3:]), "projection overflowed world coordinates")
                yield point


def iter_depth_world(observation: RGBDObservation, stride: int = 1) -> Iterator[Tuple[int, int, float, float, float, float]]:
    """Stream (u, v, depth, world x, y, z), in sampled image order.

    Pixel centres are integer (u, v); camera axes are right/down/forward. The
    supplied camera-to-world transform includes the camera mount and robot pose.
    RGB and robot_pose are not used. Zero, negative and nonfinite depths are
    omitted; an entirely invalid depth image yields no points.
    """
    validate_depth_calibration(observation)
    require(type(stride) is int and stride > 0, "depth stride must be a positive integer")
    if np is None:
        yield from _iter_depth_world_scalar(observation, stride)
    else:
        yield from _iter_depth_world_numpy(observation, stride)


def project_depth_to_world(observation: RGBDObservation) -> Tuple[ProjectedDepthPoint, ...]:
    """Return immutable surface records in image order, using the streaming projector."""
    return tuple(ProjectedDepthPoint(u, v, depth, Point3(x, y, z))
                 for u, v, depth, x, y, z in iter_depth_world(observation))
