"""Render controlled iGibson cup assets for detector screening, not search episodes. used for offline evaluation of target detection and tracking."""

import argparse
import json
from pathlib import Path


# Create target-present and target-absent views with known asset identity and distance.
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--width", type=int, default=320)
    parser.add_argument("--height", type=int, default=240)
    args = parser.parse_args()
    if args.width <= 0 or args.height <= 0:
        parser.error("image dimensions must be positive")
    args.output.mkdir(parents=True, exist_ok=False)
    temporary = args.output / "scene_instances"
    temporary.mkdir()
    import cv2
    import numpy as np
    import pybullet as p
    from igibson.objects.articulated_object import URDFObject
    from igibson.render.mesh_renderer.mesh_renderer_settings import MeshRendererSettings
    from igibson.scenes.empty_scene import EmptyScene
    from igibson.simulator import Simulator
    from igibson.utils.assets_utils import get_ig_model_path, get_ig_avg_category_specs

    rows = []
    # Keep camera geometry fixed while varying the native render resolution.
    simulator = Simulator(mode="headless", image_width=args.width, image_height=args.height, vertical_fov=45,
                          rendering_settings=MeshRendererSettings(optimized=True))
    try:
        simulator.import_scene(EmptyScene(floor_plane_rgba=[0.6, 0.6, 0.6, 1.]))
        category_specs = get_ig_avg_category_specs()
        objects = []
        # The optimized renderer requires every object before its first render.
        for category, model in (("mug", "08_0"), ("cup", "cup_000")):
            model_path = get_ig_model_path(category, model)
            obj = URDFObject(str(Path(model_path) / (model + ".urdf")), name="probe_" + category,
                             category=category, model_path=model_path,
                             avg_obj_dims=category_specs.get(category), fit_avg_dim_volume=True,
                             fixed_base=True, texture_randomization=False,
                             scene_instance_folder=str(temporary.resolve()))
            simulator.import_object(obj)
            obj.set_position([100., 100., 100.])
            objects.append((category, model, obj))
        simulator.sync(force_sync=True)
        simulator.renderer.set_camera(np.array([0., 0., 0.4]), np.array([1., 0., 0.4]), np.array([0., 0., 1.]))
        background = simulator.renderer.render(modes=("rgb",))[0]
        cv2.imwrite(str(args.output / "absent.png"), np.rint(np.clip(background[:, :, :3], 0, 1) * 255).astype(np.uint8)[:, :, ::-1])
        rows.append({"image": "absent.png", "target_present": False})
        for category, model, obj in objects:
            for angle in (0, 90):
                quaternion = p.getQuaternionFromEuler([0, 0, angle * np.pi / 180])
                obj.set_position_orientation([0., 0., 0.], quaternion)
                boxes = [p.getAABB(body, link) for body in obj.get_body_ids()
                         for link in range(-1, p.getNumJoints(body))]
                low = np.min([box[0] for box in boxes], axis=0)
                high = np.max([box[1] for box in boxes], axis=0)
                centre = (low + high) / 2
                for distance in (0.5, 1.0, 1.5):
                    obj.set_position_orientation(np.array([distance, 0., 0.4]) - centre, quaternion)
                    simulator.sync(force_sync=True)
                    rgb = simulator.renderer.render(modes=("rgb",))[0][:, :, :3]
                    name = "%s_%s_%ddeg_%.1fm.png" % (category, model, angle, distance)
                    cv2.imwrite(str(args.output / name), np.rint(np.clip(rgb, 0, 1) * 255).astype(np.uint8)[:, :, ::-1])
                    rows.append({"image": name, "target_present": True, "target_label": "cup",
                                 "asset_category": category, "asset_model": model, "yaw_deg": angle,
                                 "centre_distance_m": distance, "asset_bounds_m": (high - low).tolist()})
            obj.set_position([100., 100., 100.])
            simulator.sync(force_sync=True)
        (args.output / "manifest.json").write_text(json.dumps({
            "kind": "controlled_asset_probes", "image_size": [args.width, args.height], "vertical_fov_deg": 45,
            "camera_position": [0, 0, 0.4], "images": rows,
            "limitations": "Isolated assets centred in a fixed camera; suspended for appearance tests. No clutter, robot motion, navigation or success verification. Mug-to-cup is a proposed development label mapping.",
        }, indent=2), encoding="utf-8")
        print("Rendered %d diagnostic images to %s" % (len(rows), args.output), flush=True)
    finally:
        simulator.disconnect()


if __name__ == "__main__":
    main()
