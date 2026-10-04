"""Capture a driven TurtleBot approach with paired RGB-D target and absent views. 
    Used for offline mapper development and evaluation of target detection and tracking."""

import argparse
import json
import math
from pathlib import Path
import random

from render_furnished_probes import bounds


# Preserve a calibrated robot-camera frame for offline mapper development.
def save_frame(simulator, robot, output, episode, sequence, source):
    import cv2
    import numpy as np
    import pybullet as p
    from frontier_search.contracts import Pose
    from frontier_search.fixtures.recording import save_rgbd_recording
    from frontier_search.simulation.rgbd import convert_igibson_frames

    renderer = simulator.renderer
    rgb_frame, xyz, seg = renderer.render_robot_cameras(modes=("rgb", "3d", "ins_seg"))
    position = robot.get_position()
    yaw = p.getEulerFromQuaternion(robot.get_orientation())[2]
    observation, error = convert_igibson_frames(
        rgb_frame, xyz, renderer.get_intrinsics(), renderer.V,
        Pose(*(float(x) for x in position), yaw_rad=float(yaw)),
        episode, sequence, float(simulator.frame_count * simulator.render_timestep),
    )
    rgb = np.frombuffer(observation.color_image.data, dtype=np.uint8).reshape(
        observation.color_image.height, observation.color_image.width, 3)
    save_rgbd_recording(output, observation, dict(source, xyz_projection_max_error_m=error))
    cv2.imwrite(str(output / "rgb.png"), rgb[:, :, ::-1])
    return seg, error, [row[3] for row in observation.camera_to_world.matrix[:3]]


# Reject robot contacts above the floor while allowing wheel support.
def furniture_contacts(robot, bodies, floor, p):
    return [contact for body in robot.get_body_ids() for other in bodies
            for contact in p.getClosestPoints(body, other, distance=0.0)
            if contact[6][2] > floor + 0.03]


# Drive a clear approach and capture paired target conditions at identical poses.
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "masks").mkdir()
    temporary = args.output / "scene_instances"
    temporary.mkdir()
    import cv2
    import numpy as np
    import pybullet as p
    from igibson.objects.articulated_object import URDFObject
    from igibson.render.mesh_renderer.mesh_renderer_settings import MeshRendererSettings
    from igibson.robots.turtlebot import Turtlebot
    from igibson.scenes.igibson_indoor_scene import InteractiveIndoorScene
    from igibson.simulator import Simulator
    from igibson.utils.assets_utils import get_ig_model_path, get_ig_avg_category_specs
    from igibson.utils.constants import MAX_INSTANCE_COUNT

    random.seed(0)
    np.random.seed(0)
    simulator = Simulator(mode="headless", image_width=640, image_height=480, vertical_fov=45,
                          physics_timestep=1.0 / 120.0, render_timestep=1.0 / 30.0,
                          rendering_settings=MeshRendererSettings(optimized=True))
    try:
        scene = InteractiveIndoorScene("Rs_int", build_graph=False, should_open_all_doors=True, include_robots=False)
        simulator.import_scene(scene)
        if scene.objects_by_category.get("cup") or scene.objects_by_category.get("mug"):
            raise RuntimeError("Scene already contains target category objects")
        table = scene.objects_by_category["coffee_table"][0]
        low, high = bounds(table, p, np)
        centre = (low + high) / 2
        hit = p.rayTest([centre[0], centre[1], high[2] + 0.1], [centre[0], centre[1], low[2]])[0]
        if hit[0] not in table.get_body_ids():
            raise RuntimeError("No clear table support")
        support = np.array(hit[3])
        specs = get_ig_avg_category_specs()
        targets = []
        for category, model, angle in (("cup", "cup_000", 0), ("mug", "08_0", 90)):
            path = Path(get_ig_model_path(category, model))
            obj = URDFObject(str(path / (model + ".urdf")), name="approach_" + category,
                             category=category, model_path=str(path), avg_obj_dims=specs.get(category),
                             fit_avg_dim_volume=True, fixed_base=True, texture_randomization=False,
                             scene_instance_folder=str(temporary.resolve()))
            simulator.import_object(obj)
            quaternion = p.getQuaternionFromEuler([0, 0, math.radians(angle)])
            obj.set_position_orientation([0., 0., 0.], quaternion)
            low, high = bounds(obj, p, np)
            position = support - [(low[0] + high[0]) / 2, (low[1] + high[1]) / 2, low[2]]
            position[2] += 0.002
            obj.set_position([100., 100., 100.])
            targets.append((category, model, angle, obj, position, quaternion))

        robot = Turtlebot(action_type="continuous", action_normalize=False,
                          controller_config={"base": {"name": "DifferentialDriveController",
                                                      "command_input_limits": None, "command_output_limits": None}})
        simulator.import_object(robot)
        robot.reset()
        floor = float(scene.floor_heights[0])
        bodies = [p.getBodyUniqueId(i) for i in range(p.getNumBodies())
                  if p.getBodyUniqueId(i) not in robot.get_body_ids()]
        robot.set_position_orientation([0., 0., 0.], [0., 0., 0., 1.])
        bottom, _ = bounds(robot, p, np)
        base_z = floor - bottom[2] + 0.015
        selected = None
        # Find a straight route using actual robot geometry before detector inference.
        for angle in np.linspace(-math.pi / 8, 2 * math.pi - math.pi / 8, 72, endpoint=False):
            direction = np.array([math.cos(angle), math.sin(angle)])
            yaw = math.atan2(-direction[1], -direction[0])
            orientation = p.getQuaternionFromEuler([0., 0., yaw])
            clear = True
            for distance in np.linspace(1.5, 0.65, 35):
                xy = support[:2] + direction * distance
                robot.set_position_orientation([xy[0], xy[1], base_z], orientation)
                robot.keep_still()
                if furniture_contacts(robot, bodies, floor, p):
                    clear = False
                    break
                ground = p.rayTest([xy[0], xy[1], floor + 0.005], [xy[0], xy[1], floor - 0.05])[0]
                if ground[0] < 0:
                    clear = False
                    break
            if clear:
                selected = direction, orientation
                break
        if selected is None:
            raise RuntimeError("No clear straight approach from 1.5 m to 0.65 m")
        direction, orientation = selected
        start = support[:2] + direction * 1.5
        robot.set_position_orientation([start[0], start[1], base_z], orientation)
        robot.keep_still()
        for _ in range(30):
            robot.apply_action(np.zeros(2))
            simulator.step()
        # Calibrate steering direction for the installed robot model.
        yaw_before = p.getEulerFromQuaternion(robot.get_orientation())[2]
        for _ in range(15):
            robot.apply_action(np.array([0.0, 0.2]))
            simulator.step()
        yaw_after = p.getEulerFromQuaternion(robot.get_orientation())[2]
        response = math.atan2(math.sin(yaw_after - yaw_before), math.cos(yaw_after - yaw_before))
        if abs(response) < 0.01:
            raise RuntimeError("Robot steering calibration produced insufficient rotation")
        steering_sign = float(np.sign(response))
        robot.set_position_orientation([start[0], start[1], base_z], orientation)
        robot.keep_still()
        for _ in range(30):
            robot.apply_action(np.zeros(2))
            simulator.step()
        rows, trajectory = [], []
        stop_reason = "step_limit"
        for step in range(401):
            position = robot.get_position()
            distance = float(np.linalg.norm(position[:2] - support[:2]))
            if furniture_contacts(robot, bodies, floor, p):
                stop_reason = "furniture_contact"
                break
            if step % 20 == 0 or distance <= 0.70:
                frame = len(trajectory)
                trajectory.append({"frame": frame, "step": step, "robot_position": position.tolist(),
                                   "horizontal_table_distance_m": distance,
                                   "timestamp_s": float(simulator.frame_count * simulator.render_timestep)})
                for target in [None] + targets:
                    name = "frame_%02d_%s" % (frame, "absent" if target is None else target[0])
                    if target is not None:
                        category, model, angle, obj, target_position, quaternion = target
                        obj.set_position_orientation(target_position, quaternion)
                    simulator.sync(force_sync=True)
                    recording = args.output / "recordings" / name
                    seg, error, camera = save_frame(simulator, robot, recording,
                        "approach-" + ("absent" if target is None else target[0]), frame + 1,
                        {"kind": "igibson_robot_approach", "scene": "Rs_int", "robot": "Turtlebot"})
                    image_name = name + ".png"
                    (args.output / image_name).write_bytes((recording / "rgb.png").read_bytes())
                    mask = np.zeros(seg.shape[:2], dtype=bool)
                    if target is not None:
                        ids = np.rint(seg[:, :, 0] * MAX_INSTANCE_COUNT).astype(np.int32)
                        mask = np.isin(simulator.renderer.get_pb_ids_for_instance_ids(ids), obj.get_body_ids())
                        obj.set_position([100., 100., 100.])
                        simulator.sync(force_sync=True)
                    ys, xs = np.where(mask)
                    cv2.imwrite(str(args.output / "masks" / image_name), mask.astype(np.uint8) * 255)
                    rows.append({"image": image_name, "frame": frame, "target_present": target is not None,
                                 "target_visible_pixels": int(mask.sum()), "camera_position": camera,
                                 "horizontal_table_distance_m": distance,
                                 "asset_category": target[0] if target else None,
                                 "asset_model": target[1] if target else None,
                                 "yaw_deg": target[2] if target else None,
                                 "recording": str(recording.relative_to(args.output)),
                                 "xyz_projection_max_error_m": error,
                                 "visible_bbox_xyxy_px": [int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1] if xs.size else None})
                print("Frame %d: table distance %.3f m" % (frame, distance), flush=True)
            if distance <= 0.70:
                stop_reason = "reached_approach_limit"
                break
            desired = math.atan2(support[1] - position[1], support[0] - position[0])
            yaw = p.getEulerFromQuaternion(robot.get_orientation())[2]
            yaw_error = math.atan2(math.sin(desired - yaw), math.cos(desired - yaw))
            robot.apply_action(np.array([0.10, steering_sign * np.clip(2 * yaw_error, -0.4, 0.4)]))
            simulator.step()
        robot.apply_action(np.zeros(2))
        robot.keep_still()
        (args.output / "manifest.json").write_text(json.dumps({
            "kind": "robot_camera_approach", "scene": "Rs_int", "image_size": [640, 480],
            "vertical_fov_deg": 45, "support_position": support.tolist(), "stop_reason": stop_reason,
            "steering_calibration": {"positive_command_yaw_delta_rad": response, "command_sign": steering_sign},
            "trajectory": trajectory, "images": rows,
            "limitations": "One local straight approach selected using simulator geometry; no global planner. Fixed target placement; paired conditions rendered without advancing physics. Masks and scene geometry used for evaluation/setup only. No tracking or target-claim verification.",
        }, indent=2), encoding="utf-8")
        print("Saved %d images; %s" % (len(rows), stop_reason), flush=True)
    finally:
        simulator.disconnect()


if __name__ == "__main__":
    main()
