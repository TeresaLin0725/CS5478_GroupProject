"""Labelled analytic RGB-D development data; scene truth stays in this renderer."""

from math import cos, sin
from struct import pack

from ..contracts import Pose, RGBDObservation
from ..sensors import CameraIntrinsics, ImageBuffer, Transform


def render_frame(sequence=1, x=0.0, y=0.0, yaw=0.0, boxes=(), floor_z=0.0,
                 width=61, height=61, pitch_down=0.0, episode_id='synthetic-rgbd'):
    """Render a flat floor, x=4 wall and optional AABBs into axial RGB-D.

    This is an analytic image generator, not simulated robot execution. Depth
    uses the existing camera conventions. RGB is neutral grey; no semantic
    detector or target success is implied by these fixtures.
    """
    fx, fy = 40.0 * width / 61, 40.0 * height / 61
    cx, cy = (width - 1) / 2, (height - 1) / 2
    camera = (x, y, floor_z + 0.6)
    cp, sp, cyaw, syaw = cos(pitch_down), sin(pitch_down), cos(yaw), sin(yaw)
    matrix = ((syaw, -sp * cyaw, cp * cyaw, x),
              (-cyaw, -sp * syaw, cp * syaw, y),
              (0., -cp, -sp, camera[2]), (0., 0., 0., 1.))
    depths = []
    for v in range(height):
        for u in range(width):
            right, down = (u - cx) / fx, (v - cy) / fy
            direction = tuple(row[0] * right + row[1] * down + row[2] for row in matrix[:3])
            hits = []
            if direction[2] < 0:
                hits.append((floor_z - camera[2]) / direction[2])
            if direction[0] > 0:
                hits.append((4.0 - x) / direction[0])
            for lower, upper in boxes:
                enter, leave = 0.0, float('inf')
                for origin, d, low, high in zip(camera, direction, lower, upper):
                    if d == 0:
                        if not low <= origin <= high:
                            leave = -1.0
                    else:
                        near, far = sorted(((low - origin) / d, (high - origin) / d))
                        enter, leave = max(enter, near), min(leave, far)
                if 0 < enter <= leave:
                    hits.append(enter)
            depths.append(min(hits) if hits else 0.0)
    return RGBDObservation(
        episode_id, sequence, float(sequence),
        ImageBuffer(width, height, 'rgb8', bytes([128]) * width * height * 3),
        ImageBuffer(width, height, 'depth32f_le', pack('<%df' % len(depths), *depths)),
        CameraIntrinsics(width, height, fx, fy, cx, cy), Transform(matrix), Pose(x, y, floor_z, yaw))
