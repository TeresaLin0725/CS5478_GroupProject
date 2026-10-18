"""Validate capture records at the perception input boundary."""


from ..contracts import Pose, RGBDObservation
from ..immutable import require_immutable
from ..sensors import CameraIntrinsics, ImageBuffer, Transform, RGBD_SENSOR_CONVENTIONS
from ..validation import ContractError, is_finite_number, require



# Check that depth and calibration describe a supported pinhole camera capture.
def validate_depth_calibration(observation: RGBDObservation) -> None:
    require(isinstance(observation, RGBDObservation), "projection requires RGBDObservation")
    depth = observation.depth_image
    camera = observation.camera_intrinsics
    transform = observation.camera_to_world
    require(isinstance(depth, ImageBuffer), "depth requires ImageBuffer")
    require(isinstance(camera, CameraIntrinsics), "missing camera intrinsics")
    require(isinstance(transform, Transform), "missing camera-to-world transform")
    require(depth.encoding == "depth32f_le", "projection requires depth32f_le axial metres")
    require(type(depth.width) is int and type(depth.height) is int
            and depth.width > 0 and depth.height > 0, "depth dimensions must be positive integers")
    require(type(camera.width) is int and type(camera.height) is int
            and (camera.width, camera.height) == (depth.width, depth.height),
            "depth and calibration dimensions differ")
    require(isinstance(depth.data, bytes) and len(depth.data) == depth.width * depth.height * 4,
            "depth buffer must contain one float32 per pixel")
    require(all(is_finite_number(x) for x in (camera.fx, camera.fy, camera.cx, camera.cy))
            and camera.fx > 0 and camera.fy > 0, "invalid pinhole intrinsics")
    require(isinstance(camera.distortion, tuple)
            and all(is_finite_number(x) and x == 0 for x in camera.distortion),
            "nonzero distortion requires image rectification before projection")

    matrix = transform.matrix
    require(isinstance(matrix, tuple) and len(matrix) == 4
            and all(isinstance(row, tuple) and len(row) == 4 for row in matrix),
            "camera-to-world transform must be 4x4")
    require(all(is_finite_number(x) for row in matrix for x in row),
            "camera-to-world transform must be finite")
    require(all(abs(a - b) <= 1e-6 for a, b in zip(matrix[3], (0, 0, 0, 1))),
            "invalid homogeneous transform bottom row")
    # A rigid transform preserves scale and handedness; tolerate float32 rounding.
    require(all(abs(sum(matrix[i][k] * matrix[j][k] for k in range(3))
                    - (1 if i == j else 0)) <= 1e-5
                for i in range(3) for j in range(3)),
            "camera rotation must be orthonormal")
    a, b, c = matrix[:3]
    determinant = (a[0] * (b[1] * c[2] - b[2] * c[1])
                   - a[1] * (b[0] * c[2] - b[2] * c[0])
                   + a[2] * (b[0] * c[1] - b[1] * c[0]))
    require(abs(determinant - 1) <= 1e-5, "camera rotation must preserve handedness")


# Check a complete RGB-D observation before it enters perception or recording.
def validate_rgbd_observation(observation: RGBDObservation) -> None:
    validate_depth_calibration(observation)
    try:
        require_immutable(observation)
    except (TypeError, ValueError) as exc:
        raise ContractError("invalid RGB-D observation: %s" % exc) from exc
    require(isinstance(observation.episode_id, str) and bool(observation.episode_id.strip()),
            "missing observation episode ID")
    require(type(observation.sequence) is int and observation.sequence >= 0,
            "observation sequence must be a nonnegative integer")
    require(is_finite_number(observation.timestamp_s) and observation.timestamp_s >= 0,
            "observation timestamp must be finite and nonnegative")
    color, depth = observation.color_image, observation.depth_image
    require(isinstance(color, ImageBuffer), "color requires ImageBuffer")
    require(type(color.width) is int and type(color.height) is int
            and (color.width, color.height) == (depth.width, depth.height),
            "RGB and depth dimensions differ")
    require(color.encoding == RGBD_SENSOR_CONVENTIONS.rgb_encoding,
            "color encoding must be rgb8")
    require(isinstance(color.data, bytes) and len(color.data) == color.width * color.height * 3,
            "RGB buffer must contain three bytes per pixel")
    pose = observation.robot_pose
    require(isinstance(pose, Pose) and all(is_finite_number(x) for x in
            (pose.x_m, pose.y_m, pose.z_m, pose.yaw_rad)), "robot pose must be finite")
    motion = (observation.odometry_linear_speed_mps, observation.odometry_angular_speed_radps)
    require(all(value is None for value in motion) or all(
        is_finite_number(value) and value >= 0 for value in motion),
        "control-step odometry speeds must both be absent or finite nonnegative magnitudes")
