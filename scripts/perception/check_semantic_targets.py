"""Place installed kitchen targets in Rs and screen the native camera + RGB detector.

This is a stationary diagnostic, not a navigation episode. Masks are used only
for offline scoring and are never passed to the detector.
"""
import argparse
from dataclasses import asdict
import json
from math import atan2, pi
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src'))


def iou(a, b):
    if a is None or b is None:
        return 0.
    overlap = max(0, min(a[2], b[2])-max(a[0], b[0])) * max(0, min(a[3], b[3])-max(a[1], b[1]))
    union = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - overlap
    return overlap / union if union else 0.


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--models', nargs='+', choices=('024_bowl', '011_banana'), default=['024_bowl', '011_banana'])
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ.setdefault('YOLO_CONFIG_DIR', str(ROOT / 'data/yolo-config'))

    import numpy as np
    from PIL import Image, ImageDraw
    import torch
    from frontier_search.simulation.igibson_rs import IGibsonWorld, DATA_ROOT
    from frontier_search.perception.detection import TiledYOLODetector
    from frontier_search.sensors import ImageBuffer

    torch.set_num_threads(4)
    asset_root = DATA_ROOT / 'robot_assets/assets/models/ycb'
    report = dict(scene='Rs', profile='apartment',
                  installed_ycb_models=sorted(p.name for p in asset_root.iterdir() if p.is_dir()),
                  toothbrush_assets=list(str(p.relative_to(DATA_ROOT)) for p in DATA_ROOT.rglob('*toothbrush*')),
                  confidence_threshold=.5, camera='unmodified native Locobot eyes, vertical FOV 75 degrees',
                  limitations='Stationary diagnostic poses are teleported, not driven. Pose collision checks do not prove route reachability or search success. No LLM calls. Scoring masks are private offline labels.',
                  targets=[])
    tiles = []
    for model in args.models:
        label = {'024_bowl': 'bowl', '011_banana': 'banana'}[model]
        world = IGibsonWorld(profile='apartment', target_model=model, target_position=(100., 100., 100.))
        try:
            p, renderer = world.p, world.sim.renderer
            world.target.set_position_orientation((0., 0., 0.), (0., 0., 0., 1.))
            low, high = map(np.array, p.getAABB(world.target_body))
            # Use a flat counter beside the stove, already used by the mug task.
            # Check a 3x3 footprint, not just a single centre ray.
            half = (high-low)[:2]/2
            world.target.set_position_orientation((100., 100., 100.), (0., 0., 0., 1.))
            support_checks = []
            # The bowl is wider than the old mug. Move inward if any footprint
            # sample misses the counter; choose using geometry before detection.
            for x, y in ((-1.4, 2.95), (-1.35, 2.95), (-1.3, 2.95), (-1.25, 2.95)):
                origins = [[x+dx, y+dy, 1.25] for dx in (-half[0], 0., half[0]) for dy in (-half[1], 0., half[1])]
                hits = p.rayTestBatch(origins, [[a, b, .5] for a, b, _ in origins])
                support = [float(h[3][2]) for h in hits]
                supported = all(h[0] == world.scene.mesh_body_id for h in hits)
                support_checks.append(dict(x=x, y=y, all_hit=supported, heights_m=support))
                if supported and max(support)-min(support) <= .015:
                    break
            else:
                raise RuntimeError('No fully supported flat counter placement for ' + model)
            # AABBs include Bullet's collision margin; leave 2 mm above the highest hit.
            position = [float(x-(low[0]+high[0])/2), float(y-(low[1]+high[1])/2), float(max(support)-low[2]+.002)]
            world.target.set_position_orientation(position, (0., 0., 0., 1.))
            placement = dict(model=model, target_class=label, room='kitchen',
                             position_m=position, orientation_xyzw=[0., 0., 0., 1.],
                             collision_extent_m=(high-low).tolist(), support_heights_m=support,
                             support_candidates_checked=support_checks,
                             all_support_rays_hit_scan=supported, support_height_spread_m=max(support)-min(support),
                             static_body=True, views=[])
            detector = TiledYOLODetector(str(ROOT/'weights/yolo11/yolo11s.pt'), target_class=label,
                                        minimum_confidence=.5, tile_every_frames=1,
                                        device='0' if torch.cuda.is_available() else 'cpu')
            placement['detector_supports_class'] = label in detector._model.names.values()
            placement['detector_supports_toothbrush'] = 'toothbrush' in detector._model.names.values()
            viewpoints = [('start', .8, -2.4, pi/2),
                          ('kitchen_entry', .4, 1.75, atan2(y-1.75, x-.4)),
                          ('kitchen_mid', -.35, 2.15, atan2(y-2.15, x+.35)),
                          ('kitchen_near', -1.05, 2.15, atan2(y-2.15, x+1.05))]
            for name, rx, ry, yaw in viewpoints:
                world.robot.set_position_orientation([rx, ry, .05], p.getQuaternionFromEuler([0., 0., yaw]))
                world.robot.apply_action(np.zeros(2))
                p.performCollisionDetection()
                collisions = []
                for body in world.robot_bodies:
                    for other in [world.scene.mesh_body_id, world.target_body]:
                        collisions.extend(c for c in p.getClosestPoints(body, other, 0.)
                                          if c[8] < -.002 and c[5][2] > .06)
                world.sim.sync(force_sync=True)
                renderer.set_fov(75.)
                rgb, seg = renderer.render_robot_cameras(modes=('rgb', 'ins_seg'))
                rgb8 = np.rint(np.clip(rgb[:, :, :3], 0., 1.)*255).astype(np.uint8)
                ids = np.rint(seg[:, :, 0]*1024).astype(int).clip(0, 1023)
                mask = renderer.instance_id_to_pb_id[ids] == world.target_body
                rows, cols = np.nonzero(mask)
                bbox = [int(cols.min()), int(rows.min()), int(cols.max()+1), int(rows.max()+1)] if len(rows) else None
                detections = detector.detect(ImageBuffer(640, 480, 'rgb8', rgb8.tobytes()))
                matches = [d for d in detections if d.class_name == label and iou(d.bbox_xyxy_px, bbox) >= .5]
                filename = '%s_%s.png' % (model, name)
                Image.fromarray(rgb8).save(args.output/filename)
                # Same camera without the inserted target: check background false positives.
                world.target.set_position_orientation((100., 100., 100.), (0., 0., 0., 1.))
                world.sim.sync(force_sync=True)
                absent = renderer.render_robot_cameras(modes=('rgb',))[0]
                absent8 = np.rint(np.clip(absent[:, :, :3], 0., 1.)*255).astype(np.uint8)
                negative = detector.detect(ImageBuffer(640, 480, 'rgb8', absent8.tobytes()))
                Image.fromarray(absent8).save(args.output/('%s_%s_absent.png' % (model, name)))
                world.target.set_position_orientation(position, (0., 0., 0., 1.))
                row = dict(name=name, robot_pose_m_rad=[rx, ry, yaw], robot_pose_penetrations=len(collisions),
                           horizontal_target_distance_m=float(np.hypot(rx-x, ry-y)),
                           image=filename, target_pixels=int(mask.sum()), target_bbox=bbox,
                           matched_target_detections=[asdict(d) for d in matches],
                           all_detections=[asdict(d) for d in detections],
                           absent_target_detections=[asdict(d) for d in negative if d.class_name == label])
                placement['views'].append(row)
                tile = Image.new('RGB', (640, 515), 'white')
                tile.paste(Image.fromarray(rgb8), (0, 35))
                draw = ImageDraw.Draw(tile)
                draw.text((8, 8), '%s / %s | %d px | matched %d | collisions %d' % (label, name, mask.sum(), len(matches), len(collisions)), fill='black')
                if bbox:
                    draw.rectangle((bbox[0], bbox[1]+35, bbox[2], bbox[3]+35), outline='lime', width=2)
                tiles.append(tile)
                print(model, name, 'pixels', int(mask.sum()), 'matched', [round(d.confidence, 3) for d in matches], 'penetrations', len(collisions), flush=True)
            report['targets'].append(placement)
        finally:
            world.close()
    sheet = Image.new('RGB', (640*4, 515*len(args.models)), 'white')
    for index, tile in enumerate(tiles):
        sheet.paste(tile, (index % 4*640, index // 4*515))
    sheet.save(args.output/'contact-sheet.png')
    (args.output/'report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print('Saved:', args.output.resolve(), flush=True)


if __name__ == '__main__':
    main()
