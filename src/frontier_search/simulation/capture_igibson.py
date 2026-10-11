"""Capture a real scene using the existing standalone iGibson project. Used for offline mapper development and evaluation of target detection and tracking.

Run with its igibson environment and this repository's src on PYTHONPATH.
This is a perception development utility, not the integrated simulator adapter.
"""

import argparse
import importlib.util
import hashlib
import json
from pathlib import Path
import random

from ..contracts import Pose
from ..fixtures.recording import save_rgbd_recording
from .rgbd import convert_igibson_frames


DEFAULT_CONFIG = Path(__file__).resolve().parents[3] / "configs" / "perception.locobot.json"


def load_capture_config(path):
    """Keep the selected robot and native camera convention explicit."""
    config = json.loads(Path(path).read_text(encoding="utf-8"))
    if (config.get("robot") != "Locobot" or config.get("camera_link") != "eyes"
            or config.get("camera_pose") != "native_link"):
        raise ValueError("perception capture requires Locobot's native eyes camera")
    for key in ("image_width", "image_height"):
        if type(config.get(key)) is not int or config[key] <= 0:
            raise ValueError("%s must be a positive integer" % key)
    fov = config.get("vertical_fov_deg")
    if type(fov) not in (int, float) or not 0 < fov < 180:
        raise ValueError("vertical_fov_deg must lie between 0 and 180")
    if config.get("hide_robot") is not False:
        raise ValueError("native perception capture must retain robot visibility")
    return config


# Capture and save an iGibson observation for offline perception development.
def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference-project", type=Path,
                        default=Path("D:/Projects/CS5478_RoboticProject"))
    parser.add_argument("--scene", default="Rs_int")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("output directory already exists; choose a new recording directory")
    try:
        config = load_capture_config(args.config)
    except (OSError, ValueError, TypeError, AttributeError) as exc:
        parser.error(str(exc))

    # Reuse the reference project's tested robot setup and spawn helper.
    spec = importlib.util.spec_from_file_location(
        "reference_simulator_test", str(args.reference_project / "src" / "simulator_test.py"))
    demo = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(demo)
    import igibson
    import numpy as np
    import pybullet as p
    from igibson.robots.locobot import Locobot

    random.seed(args.seed)
    np.random.seed(args.seed)
    simulator = demo.Simulator(
        mode="headless", image_width=config["image_width"], image_height=config["image_height"],
        vertical_fov=config["vertical_fov_deg"],
        physics_timestep=1.0 / 120.0, render_timestep=demo.TIMESTEP,
        rendering_settings=demo.MeshRendererSettings(optimized=True, hide_robot=False),
    )
    try:
        print("Loading scene:", args.scene, flush=True)
        scene = demo.InteractiveIndoorScene(args.scene, build_graph=False,
                                            should_open_all_doors=True, include_robots=False)
        simulator.import_scene(scene)
        # Hidden traversability is used only by the reference spawn sampler.
        scene.load_trav_map(str(Path(scene.scene_dir) / "layout"))
        robot = Locobot(action_type="continuous", action_normalize=False,
                              controller_config={"base": {
                                  "name": "DifferentialDriveController",
                                  "command_input_limits": None, "command_output_limits": None,
                              }})
        simulator.import_object(robot)
        demo.spawn_robot(simulator, scene, robot)
        robot.apply_action(np.zeros(2))
        simulator.step()

        # Capture all modalities and calibration at one stationary simulation state.
        renderer = simulator.renderer
        rgb_frame, xyz_frame = renderer.render_robot_cameras(modes=("rgb", "3d"))
        position = robot.get_position()
        yaw = p.getEulerFromQuaternion(robot.get_orientation())[2]
        observation, error = convert_igibson_frames(
            rgb_frame, xyz_frame, renderer.get_intrinsics(), renderer.V,
            Pose(*(float(x) for x in position), yaw_rad=float(yaw)),
            "igibson-%s-seed-%d" % (args.scene, args.seed), 1,
            float(simulator.frame_count * simulator.render_timestep),
        )
        height, width = observation.depth_image.height, observation.depth_image.width
        rgb = np.frombuffer(observation.color_image.data, dtype=np.uint8).reshape(height, width, 3)
        depth = np.frombuffer(observation.depth_image.data, dtype="<f4").reshape(height, width)
        valid = depth > 0
        save_rgbd_recording(args.output, observation, {
            "kind": "igibson_capture", "scene_id": args.scene, "seed": args.seed,
            "robot": config["robot"], "camera_link": config["camera_link"],
            "camera_pose": config["camera_pose"], "capture_config": config,
            "robot_model_sha256": hashlib.sha256(Path(robot.model_file).read_bytes()).hexdigest(),
            "camera_position_world_m": [float(v) for v in robot.eyes.get_position()],
            "camera_orientation_world_xyzw": [float(v) for v in robot.eyes.get_orientation()],
            "reference_project": str(args.reference_project.resolve()),
            "igibson_package": str(Path(igibson.__file__).parent),
            "depth_units": "metres", "depth_measurement": "camera_z",
            "camera_axes": ["right", "down", "forward"],
            "xyz_projection_max_error_m": error,
        })
        # Convenient previews; depth data above is saved without clipping.
        demo.cv2.imwrite(str(args.output / "rgb.png"), rgb[:, :, ::-1])
        preview = (np.clip(depth / 5.0, 0, 1) * 255).astype(np.uint8)
        demo.cv2.imwrite(str(args.output / "depth_preview.png"), preview)
        print("Saved:", args.output.resolve(), flush=True)
        print("Valid depth pixels: %d/%d; range %.3f..%.3f m; XYZ error %.8f m" % (
            valid.sum(), valid.size, depth[valid].min(), depth[valid].max(), error), flush=True)
    finally:
        simulator.disconnect()


if __name__ == "__main__":
    main()
