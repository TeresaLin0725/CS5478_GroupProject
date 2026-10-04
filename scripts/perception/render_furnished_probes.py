"""Render paired cup-present and cup-absent views in a furnished iGibson scene. Used for offline evaluation of target detection and tracking."""

import argparse
import json
from pathlib import Path
import random


# Measure the world-space extent of an articulated object.
def bounds(obj, physics, np):
    boxes = [physics.getAABB(body, link) for body in obj.get_body_ids()
             for link in range(-1, physics.getNumJoints(body))]
    return np.min([box[0] for box in boxes], axis=0), np.max([box[1] for box in boxes], axis=0)


# Capture furnished backgrounds and labelled target views from fixed floor viewpoints.
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=0)
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
    from igibson.scenes.igibson_indoor_scene import InteractiveIndoorScene
    from igibson.simulator import Simulator
    from igibson.utils.assets_utils import get_ig_model_path, get_ig_avg_category_specs
    from igibson.utils.constants import MAX_INSTANCE_COUNT

    random.seed(args.seed)
    np.random.seed(args.seed)
    simulator = Simulator(mode="headless", image_width=640, image_height=480,
                          vertical_fov=45, rendering_settings=MeshRendererSettings(optimized=True))
    try:
        scene = InteractiveIndoorScene("Rs_int", build_graph=False,
                                       should_open_all_doors=True, include_robots=False)
        simulator.import_scene(scene)
        if scene.objects_by_category.get("cup") or scene.objects_by_category.get("mug"):
            raise RuntimeError("Negative controls require a scene without existing cups or mugs")
        scene.load_trav_map(str(Path(scene.scene_dir) / "layout"))
        table = scene.objects_by_category["coffee_table"][0]
        table_low, table_high = bounds(table, p, np)
        centre = (table_low + table_high) / 2
        # Check the actual support surface at the table centre.
        hit = p.rayTest([centre[0], centre[1], table_high[2] + 0.1],
                        [centre[0], centre[1], table_low[2]])[0]
        if hit[0] not in table.get_body_ids():
            raise RuntimeError("Table centre has no unobstructed support surface")
        support = np.array(hit[3])
        objects = []
        specs = get_ig_avg_category_specs()
        for category, model in (("cup", "cup_000"), ("mug", "08_0")):
            path = Path(get_ig_model_path(category, model))
            obj = URDFObject(str(path / (model + ".urdf")), name="probe_" + category,
                             category=category, model_path=str(path), avg_obj_dims=specs.get(category),
                             fit_avg_dim_volume=True, fixed_base=True, texture_randomization=False,
                             scene_instance_folder=str(temporary.resolve()))
            simulator.import_object(obj)
            obj.set_position([100., 100., 100.])
            objects.append((category, model, obj))

        candidates = []
        # Reject furniture intersections around the camera and approximate robot body.
        shape = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.18, height=0.4)
        proxy = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=shape,
                                 basePosition=[100., 100., 100.])
        scene_bodies = [p.getBodyUniqueId(i) for i in range(p.getNumBodies())
                        if p.getBodyUniqueId(i) != proxy]
        try:
            for _ in range(3000):
                _, point = scene.get_random_point(floor=0)
                distance = np.linalg.norm(point[:2] - support[:2])
                if 0.6 <= distance <= 2.0:
                    point = np.array(point, dtype=float)
                    p.resetBasePositionAndOrientation(proxy, point + [0., 0., 0.22], [0., 0., 0., 1.])
                    if any(p.getClosestPoints(proxy, body, distance=0.0) for body in scene_bodies):
                        continue
                    point[2] += 0.4
                    candidates.append(point)
        finally:
            p.removeBody(proxy)
        if not candidates:
            raise RuntimeError("No floor-map viewpoints near the coffee table")
        # Spread the views around the table without inspecting detector outputs.
        viewpoints = [min(candidates, key=lambda x: np.linalg.norm(x[:2] - support[:2]))]
        for _ in range(5):
            candidate = max(candidates, key=lambda x: min(np.linalg.norm(x[:2] - y[:2]) for y in viewpoints))
            if min(np.linalg.norm(candidate[:2] - y[:2]) for y in viewpoints) < 0.25:
                break
            viewpoints.append(candidate)

        rows = []
        for view_index, camera in enumerate(viewpoints):
            simulator.sync(force_sync=True)
            renderer = simulator.renderer
            renderer.set_camera(camera, np.array([support[0], support[1], camera[2]]), np.array([0., 0., 1.]))
            rgb = renderer.render(modes=("rgb",))[0]
            name = "view_%02d_absent.png" % view_index
            cv2.imwrite(str(args.output / name), np.rint(np.clip(rgb[:, :, :3], 0, 1) * 255).astype(np.uint8)[:, :, ::-1])
            rows.append({"image": name, "view": view_index, "target_present": False,
                         "camera_position": camera.tolist(), "target_visible_pixels": 0})
            for category, model, obj in objects:
                for angle in (0, 90):
                    quaternion = p.getQuaternionFromEuler([0, 0, np.deg2rad(angle)])
                    obj.set_position_orientation([0., 0., 0.], quaternion)
                    low, high = bounds(obj, p, np)
                    position = support - np.array([(low[0] + high[0]) / 2, (low[1] + high[1]) / 2, low[2]])
                    position[2] += 0.002
                    obj.set_position_orientation(position, quaternion)
                    simulator.sync(force_sync=True)
                    rgb, seg = renderer.render(modes=("rgb", "ins_seg"))
                    ids = np.rint(seg[:, :, 0] * MAX_INSTANCE_COUNT).astype(np.int32)
                    mask = np.isin(renderer.get_pb_ids_for_instance_ids(ids), obj.get_body_ids())
                    ys, xs = np.where(mask)
                    name = "view_%02d_%s_%s_%ddeg.png" % (view_index, category, model, angle)
                    cv2.imwrite(str(args.output / name), np.rint(np.clip(rgb[:, :, :3], 0, 1) * 255).astype(np.uint8)[:, :, ::-1])
                    cv2.imwrite(str(args.output / "masks" / name), mask.astype(np.uint8) * 255)
                    rows.append({"image": name, "view": view_index, "target_present": True,
                                 "target_label": "cup", "asset_category": category, "asset_model": model,
                                 "yaw_deg": angle, "camera_position": camera.tolist(),
                                 "object_position": position.tolist(), "target_visible_pixels": int(mask.sum()),
                                 "visible_bbox_xyxy_px": [int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1] if xs.size else None})
                    obj.set_position([100., 100., 100.])
                    simulator.sync(force_sync=True)
            print("Captured furnished view %d" % view_index, flush=True)
        (args.output / "manifest.json").write_text(json.dumps({
            "kind": "furnished_scene_probes", "scene": "Rs_int", "seed": args.seed,
            "image_size": [640, 480], "vertical_fov_deg": 45, "camera_height_above_floor_m": 0.4,
            "support_object": table.name, "support_position": support.tolist(), "images": rows,
            "viewpoint_clearance_proxy": {"radius_m": 0.18, "height_m": 0.4, "bottom_above_floor_m": 0.02},
            "limitations": "Fixed horizontal diagnostic camera on sampled traversable-map points; cylinder clearance checked, actual robot geometry and route reachability not verified. Assets fixed 2 mm above table support; no settling or search episode. Simulator masks are evaluation labels only, never detector input.",
        }, indent=2), encoding="utf-8")
        print("Saved %d images to %s" % (len(rows), args.output), flush=True)
    finally:
        simulator.disconnect()


if __name__ == "__main__":
    main()
