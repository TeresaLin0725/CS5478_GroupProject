"""Small known-geometry observations used only by unit tests."""

from struct import pack

from frontier_search.contracts import Pose, RGBDObservation
from frontier_search.sensors import (
    CameraIntrinsics, ImageBuffer, SYNTHETIC_SENSOR_CONVENTIONS, Transform,
)


# Provide a known wall observation for checking perception geometry.
def load_wall_observation() -> RGBDObservation:
    """Return a 5x3 view of a flat wall at 2 m axial depth.

    The camera sits at world (1, 2, 1), facing +x, above a robot at (1, 2, 0).
    Camera right maps to world -y, down to -z, and forward to +x. All visible
    wall points therefore lie on world x=3. With fx=fy=2 and centre (2, 1),
    expected pixel-to-world results include:
        (u=2, v=1) -> (3, 2, 1)  centre
        (u=0, v=0) -> (3, 4, 2)  top-left
        (u=4, v=2) -> (3, 0, 0)  bottom-right

    RGB is uniform grey. Depth is packed little-endian float32, with no invalid
    pixels. Use dataclasses.replace to vary capture metadata in later tests.
    """
    convention = SYNTHETIC_SENSOR_CONVENTIONS
    width, height = 5, 3
    color = ImageBuffer(width, height, convention.rgb_encoding,
                        bytes((128, 128, 128)) * (width * height))
    depth = ImageBuffer(width, height, convention.depth_encoding,
                        pack("<%df" % (width * height), *([2.0] * (width * height))))
    intrinsics = CameraIntrinsics(width, height, fx=2.0, fy=2.0, cx=2.0, cy=1.0)
    camera_to_world = Transform((
        (0.0, 0.0, 1.0, 1.0),
        (-1.0, 0.0, 0.0, 2.0),
        (0.0, -1.0, 0.0, 1.0),
        (0.0, 0.0, 0.0, 1.0),
    ))
    return RGBDObservation(
        episode_id="synthetic-wall", sequence=1, timestamp_s=0.0,
        color_image=color, depth_image=depth, camera_intrinsics=intrinsics,
        camera_to_world=camera_to_world, robot_pose=Pose(1.0, 2.0, 0.0, 0.0),
    )
