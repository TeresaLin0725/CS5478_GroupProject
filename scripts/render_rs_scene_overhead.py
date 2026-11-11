"""Render an actual top-down RGB image of a frozen Rs benchmark scene at t=0.

This is a post-run visualization. It does not replay or modify search episodes.
The dynamic cart can move later, so its appearance here is the initial state.
"""
import argparse
import json
from math import radians, tan
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-root', type=Path, required=True)
    parser.add_argument('--job-id', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--isolated-objects', action='store_true',
                        help='Render only the inserted bowl and challenge furniture as transparent RGBA')
    args = parser.parse_args()
    root = args.run_root.resolve()
    protocol = json.loads((root / 'protocol.json').read_text(encoding='utf-8'))
    job = next(j for j in protocol['jobs'] if j['job_id'] == args.job_id)
    target = protocol['targets_evaluator_only'][job['target_location']]
    pose = job['start_pose']

    import numpy as np
    from PIL import Image
    from frontier_search.simulation.igibson_rs import IGibsonWorld

    width, height = 990, 1230
    fov_deg = 45.0
    eye_z = 11.0
    cx, cy = -1.1, -.3
    world = IGibsonWorld(start=(pose['x_m'], pose['y_m'], pose['yaw_rad']),
                        target_position=tuple(target['position_world']),
                        target_model=target['model'], profile=job['profile'],
                        scene_layout=protocol['scene_layout'],
                        image_width=width, image_height=height)
    try:
        world.sim.sync(force_sync=True)
        renderer = world.sim.renderer
        renderer.set_fov(fov_deg)
        renderer.set_camera(np.array([cx, cy, eye_z]),
                            np.array([cx, cy, 0.0]), np.array([0.0, 1.0, 0.0]))
        if args.isolated_objects:
            keep = list(world.target.renderer_instances)
            for record in world.furniture:
                keep.extend(record['object'].renderer_instances)
            hidden = [inst for inst in renderer.instances if inst not in keep]
            rgb, seg = renderer.render(modes=('rgb', 'ins_seg'), hidden=hidden)
            ids = np.clip(np.rint(seg[:, :, 0] * 1024).astype(int), 0, 1023)
            bodies = {world.target_body}
            for record in world.furniture:
                bodies.update(record['object'].get_body_ids())
            mask = np.isin(renderer.instance_id_to_pb_id[ids], list(bodies))
            image = np.dstack(((np.clip(rgb[:, :, :3], 0, 1) * 255).astype(np.uint8),
                               mask.astype(np.uint8) * 255))
        else:
            rgb = renderer.render(modes=('rgb',), hidden=world.robot.renderer_instances)[0]
            image = (np.clip(rgb[:, :, :3], 0, 1) * 255).astype(np.uint8)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(image).save(args.output)
        half_y = eye_z * tan(radians(fov_deg) / 2)
        half_x = half_y * width / height
        metadata = {'job_id': args.job_id, 'profile': job['profile'],
                    'target_location': job['target_location'], 'start_id': job['start_id'],
                    'scene_layout': protocol['scene_layout'],
                    'source': ('Actual iGibson overhead RGB and instance segmentation of inserted objects, '
                               't=0; scanned scene and robot hidden.' if args.isolated_objects else
                               'Actual iGibson RGB renderer, overhead camera, t=0; robot hidden.'),
                    'dynamic_cart_note': 'The cart can move during an episode; this image shows its initial position.',
                    'image_size_px': [width, height], 'camera_eye_xyz_m': [cx, cy, eye_z],
                    'camera_target_xyz_m': [cx, cy, 0.0], 'camera_up_xyz': [0.0, 1.0, 0.0],
                    'vertical_fov_deg': fov_deg,
                    'floor_extent_xy_m': [cx-half_x, cx+half_x, cy-half_y, cy+half_y],
                    'furniture': [{k: v for k, v in record.items() if k != 'object'}
                                  for record in world.furniture],
                    'cart_route': (None if world.cart_motion is None else
                                   {'start': world.cart_motion.start, 'end': world.cart_motion.end})}
        args.output.with_suffix('.json').write_text(
            json.dumps(metadata, indent=2), encoding='utf-8')
        print(args.output)
    finally:
        world.close()


if __name__ == '__main__':
    main()
