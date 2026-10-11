"""Measure native Locobot camera visibility in controlled furniture fixtures.

Run with the installed iGibson Python environment. Uses its unmodified URDF,
PyBullet TinyRenderer, and the same +X forward/+Z up convention as iGibson's
render_single_robot_camera. Segmentation measures visibility, not detection.
"""

import argparse
import hashlib
import json
from math import asin, degrees
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('choose a new output directory')

    import igibson
    import numpy as np
    import pybullet as p
    from PIL import Image, ImageDraw

    model = Path(igibson.assets_path) / 'models/locobot/locobot.urdf'
    args.output.mkdir(parents=True)
    p.connect(p.DIRECT)
    try:
        p.setGravity(0, 0, -9.81)
        p.createMultiBody(0, p.createCollisionShape(p.GEOM_PLANE))
        robot = p.loadURDF(str(model), [0, 0, 0.05])
        for _ in range(480):
            p.stepSimulation()
        links = {p.getJointInfo(robot, i)[12].decode(): i
                 for i in range(p.getNumJoints(robot))}
        eye, orientation = p.getLinkState(robot, links['eyes'],
                                         computeForwardKinematics=True)[4:6]
        rotation = np.array(p.getMatrixFromQuaternion(orientation)).reshape(3, 3)
        forward, up = rotation[:, 0], rotation[:, 2]
        view = p.computeViewMatrix(eye, np.array(eye) + forward, up)
        projection = p.computeProjectionMatrixFOV(60, 320 / 240, 0.05, 5)

        def box(center, half, color):
            collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
            visual = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=color)
            return p.createMultiBody(0, collision, visual, center)

        def capture(target):
            frame = p.getCameraImage(320, 240, view, projection,
                                     renderer=p.ER_TINY_RENDERER,
                                     flags=p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX)
            rgb = np.array(frame[2], dtype=np.uint8).reshape(240, 320, 4)[:, :, :3]
            ids = np.array(frame[4]).reshape(240, 320)
            mask = (ids >= 0) & ((ids & ((1 << 24) - 1)) == target)
            return Image.fromarray(rgb), mask

        cases = []
        for surface, height in [('table', 0.78), ('counter', 0.88)]:
            for placement, x in [('front', 0.90), ('back', 1.30)]:
                for size, object_height in [('small', 0.05), ('tall', 0.13)]:
                    cases.append((surface, height, placement, x, size, object_height))
        for height in (0.60, 0.80, 1.20, 1.50):
            cases.append(('open_shelf', height, 'back', 1.30, 'small', 0.05))

        rows, tiles = [], []
        for surface, height, placement, x, size, object_height in cases:
            furniture = [box([1.1, 0, height - 0.025], [0.35, 0.40, 0.025],
                             [0.55, 0.35, 0.18, 1])]
            if surface == 'counter':
                furniture.append(box([1.1, 0, (height - 0.05) / 2],
                                     [0.35, 0.40, (height - 0.05) / 2],
                                     [0.45, 0.30, 0.18, 1]))
            if surface == 'open_shelf':
                # Open front, backing and sides, plus a board 30 cm above.
                furniture.extend([
                    box([1.44, 0, height + 0.15], [0.01, 0.4, 0.15], [0.5, 0.3, 0.2, 1]),
                    box([1.1, -0.39, height + 0.15], [0.35, 0.01, 0.15], [0.5, 0.3, 0.2, 1]),
                    box([1.1, 0.39, height + 0.15], [0.35, 0.01, 0.15], [0.5, 0.3, 0.2, 1]),
                    box([1.1, 0, height + 0.30], [0.35, 0.4, 0.025], [0.5, 0.3, 0.2, 1])])
            target = box([x, 0, height + object_height / 2],
                         [0.025, 0.025, object_height / 2], [0.9, 0.1, 0.1, 1])
            image, mask = capture(target)
            for body in furniture:
                p.removeBody(body)
            _, reference = capture(target)
            visible, reference_pixels = int(mask.sum()), int(reference.sum())
            name = '%s-%.2f-%s-%s' % (surface, height, placement, size)
            row = dict(case=name, surface_height_m=height, target_height_m=object_height,
                       target_position_m=[x, 0, height + object_height / 2],
                       visible_pixels=visible, unobstructed_pixels=reference_pixels,
                       visible_fraction=visible / reference_pixels if reference_pixels else None,
                       diagnosis=('outside_native_view_or_robot_occluded' if not reference_pixels
                                  else 'furniture_occluded' if not visible else 'partially_visible'
                                  if visible < reference_pixels else 'visible'),
                       image=name + '.png')
            rows.append(row)
            image.save(args.output / row['image'])
            tile = Image.new('RGB', (320, 280), '#eeeeee')
            tile.paste(image, (0, 40))
            draw = ImageDraw.Draw(tile)
            draw.text((5, 3), name, fill='black')
            draw.text((5, 19), '%d px | %s' % (visible, row['diagnosis']), fill='black')
            tiles.append(tile)
            p.removeBody(target)

        report = dict(source='controlled_geometry_native_locobot_camera',
                      renderer='PyBullet TinyRenderer; not an apartment-scene trial',
                      model=str(model), model_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
                      camera_position_m=list(eye), camera_orientation_xyzw=list(orientation),
                      camera_pitch_deg=degrees(asin(float(forward[2]))),
                      vertical_fov_deg=60, image_size=[320, 240],
                      robot_hidden=False, camera_pose_overridden=False,
                      scope='Single stationary pose; segmentation visibility only, not YOLO accuracy. '
                            'Fixture heights and placements are authored. No navigation feasibility claim.',
                      cases=rows)
        (args.output / 'report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
        sheet = Image.new('RGB', (320 * 3, 280 * 4), 'white')
        for i, tile in enumerate(tiles):
            sheet.paste(tile, (i % 3 * 320, i // 3 * 280))
        sheet.save(args.output / 'contact-sheet.png')
        print('Camera height %.3f m; pitch %.2f degrees' % (eye[2], report['camera_pitch_deg']))
        for row in rows:
            print('%s: %d px, %s' % (row['case'], row['visible_pixels'], row['diagnosis']))
        print('Report:', args.output.resolve())
    finally:
        p.disconnect()


if __name__ == '__main__':
    main()
