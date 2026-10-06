"""Convert iGibson renderer frames to the shared RGB-D convention. Used for offline mapper development and evaluation of target detection and tracking."""

from typing import Tuple

from ..contracts import Pose, RGBDObservation
from ..perception.observations import validate_rgbd_observation
from ..sensors import CameraIntrinsics, ImageBuffer, Transform


# Convert a rendered capture and verify its camera calibration.
def convert_igibson_frames(rgb_frame, xyz_frame, intrinsics, view_matrix,
                           robot_pose: Pose, episode_id: str, sequence: int,
                           timestamp_s: float) -> Tuple[RGBDObservation, float]:
    import numpy as np

    depth = -xyz_frame[:, :, 2].copy()
    valid = np.isfinite(xyz_frame[:, :, :3]).all(axis=2) & (depth > 0)
    if not valid.any() or not np.isfinite(rgb_frame[:, :, :3]).all():
        raise RuntimeError("capture contains no valid depth or nonfinite RGB")
    depth[~valid] = 0
    height, width = depth.shape
    k = intrinsics
    # Renderer intrinsics reference image edges; our contract uses integer pixel centres.
    fx, fy = float(k[0, 0]), float(k[1, 1])
    cx, cy = float(k[0, 2] - 0.5), float(k[1, 2] - 0.5)
    u, v = np.meshgrid(np.arange(width), np.arange(height))
    error = max(float(np.max(np.abs(((u - cx) * depth / fx - xyz_frame[:, :, 0])[valid]))),
                float(np.max(np.abs(((v - cy) * depth / fy + xyz_frame[:, :, 1])[valid]))))
    if error > 0.001:
        raise RuntimeError("camera calibration disagrees with rendered XYZ: %.6f m" % error)

    # OpenGL uses right/up/back; mapper camera coordinates use right/down/forward.
    camera_to_world = np.linalg.inv(view_matrix.astype(np.float64)).dot(np.diag([1., -1., -1., 1.]))
    rgb = np.rint(np.clip(rgb_frame[:, :, :3], 0, 1) * 255).astype(np.uint8)
    observation = RGBDObservation(
        episode_id=episode_id, sequence=sequence,
        timestamp_s=timestamp_s,
        color_image=ImageBuffer(width, height, "rgb8", rgb.tobytes(order="C")),
        depth_image=ImageBuffer(width, height, "depth32f_le", depth.astype("<f4").tobytes(order="C")),
        camera_intrinsics=CameraIntrinsics(width, height, fx, fy, cx, cy),
        camera_to_world=Transform(tuple(tuple(float(x) for x in row) for row in camera_to_world)),
        robot_pose=robot_pose,
    )
    validate_rgbd_observation(observation)
    return observation, error
