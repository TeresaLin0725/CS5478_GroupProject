"""Owned sensor values and RGB-D conventions for recordings and test fixtures."""


from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class SensorConventions:
    """Describe image encoding and camera geometry for a sensor source."""

    rgb_encoding: str
    depth_encoding: str
    depth_units: str
    depth_measurement: str
    invalid_depth_value: float
    camera_axes: Tuple[str, str, str]


# Recording conventions verified with the local iGibson capture utility.
# RGB: interleaved unsigned 8-bit R, G, B bytes (3 bytes per pixel).
# Depth: little-endian IEEE-754 float32 bytes (struct format '<f', 4 bytes per pixel).
# Both buffers: tightly packed row-major, top row first, no padding; aligned pixels.
# Pixel centres use integer (u, v) coordinates; u increases right, v increases down.
# Captures use an ideal pinhole camera with distortion=(). Depth is axial, not ray range.
# Zero denotes invalid depth; decoders also reject negative/nonfinite depth.
RGBD_SENSOR_CONVENTIONS = SensorConventions(
    rgb_encoding="rgb8",
    depth_encoding="depth32f_le",
    depth_units="metres",
    depth_measurement="camera_z",
    invalid_depth_value=0.0,
    camera_axes=("right", "down", "forward"),  # +x, +y, +z
)

# Preserve the fixture name for existing callers.
SYNTHETIC_SENSOR_CONVENTIONS = RGBD_SENSOR_CONVENTIONS


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
