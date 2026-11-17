"""Preflight semantic-navigation targets before any policy episode is allowed to run."""
import argparse
from dataclasses import asdict, replace
import json
from math import atan2, pi
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def atomic(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')
    temporary.replace(path)


def iou(a, b):
    overlap = max(0, min(a[2], b[2]) - max(a[0], b[0])) * max(0, min(a[3], b[3]) - max(a[1], b[1]))
    area_a = (a[2] - a[0]) * (a[3] - a[1])
    area_b = (b[2] - b[0]) * (b[3] - b[1])
    return overlap / max(1, area_a + area_b - overlap)


def target_spec(task):
    from frontier_search.simulation.integrated_rs import RsTargetSpec
    target = task['target']
    return RsTargetSpec(target['class_name'], target['model'], tuple(target['position_world']),
                        target.get('yaw_rad', 0.), target['instance_id'],
                        target.get('roll_rad', 0.), target.get('pitch_rad', 0.)).validate()


def set_robot_pose(sim, pose):
    backend = sim._backend
    world = backend.world
    body = next(iter(world.robot_bodies))
    world.p.resetBasePositionAndOrientation(
        body, [pose.x_m, pose.y_m, .01], world.p.getQuaternionFromEuler([0., 0., pose.yaw_rad]))
    world.p.resetBaseVelocity(body, [0., 0., 0.], [0., 0., 0.])
    world.stop(15)


def oracle_grid(sim, manifest):
    from frontier_search.contracts import OccupancyGrid
    from frontier_search.fixtures.clearance import clearance_binding, footprint_cells
    probe, cfg = sim._backend.scene_probe(manifest.mapping), manifest.mapping
    band = clearance_binding(manifest, footprint_cells(manifest))
    rows = []
    for row in range(cfg.height_cells):
        values = []
        for column in range(cfg.width_cells):
            x = cfg.origin_x_m + column * cfg.resolution_m
            y = cfg.origin_y_m + row * cfg.resolution_m
            clear = probe.column_clear(x, y, x + cfg.resolution_m, y + cfg.resolution_m,
                                       band['required_z_min_m'], band['required_z_max_m'])
            supported = probe.floor_supported(x, y, x + cfg.resolution_m, y + cfg.resolution_m)
            values.append(0 if clear and supported else 100)
        rows.append(tuple(values))
    return OccupancyGrid('world', 1, cfg.resolution_m, cfg.origin_x_m, cfg.origin_y_m,
                         cfg.width_cells, cfg.height_cells, tuple(rows))


def matched_detection(detector, observation, truth):
    detections = detector.detect(observation.color_image)
    matches = [item for item in detections if item.class_name == truth.class_name]
    best = max((iou(item.bbox_xyxy_px, truth.bbox_xyxy_px) for item in matches), default=0.)
    confidence = max((item.confidence for item in matches
                      if iou(item.bbox_xyxy_px, truth.bbox_xyxy_px) == best), default=None)
    return detections, best, confidence


def preflight_task(task, suite, detector, output, shared_grid=None):
    import numpy as np
    from PIL import Image
    from frontier_search.contracts import Pose
    from frontier_search.evaluation.retained import RetainedFrames
    from frontier_search.navigation.costmap import Costmap
    from frontier_search.navigation.lattice import LatticeSearch
    from frontier_search.runner.integrated import shared_round_manifest_for
    from frontier_search.simulation.igibson_adapter import AdapterConfig
    from frontier_search.simulation.integrated_rs import RetainingAdapter

    spec = target_spec(task)
    start = Pose(suite['start_pose'][0], suite['start_pose'][1], 0., suite['start_pose'][2])
    manifest = replace(shared_round_manifest_for('preflight-' + task['task_id']),
                       target_class=spec.class_name, start_pose=start,
                       environment_seed=suite['environment_seed'])
    store = RetainedFrames()
    sim = RetainingAdapter(manifest.robot, manifest.mapping,
        AdapterConfig(scene_id='Rs', settle_steps=0, image_width=640, image_height=480,
                      vertical_fov_deg=75.), store, suite['profile'], spec)
    result = dict(task_id=task['task_id'], passed=False, target=asdict(spec),
                  target_region=task['target_region'], start_pose=asdict(start),
                  scope='Private geometry, target truth and detector preflight; never passed to a policy.')
    try:
        sim.reset(manifest.episode_id, 'Rs', start, manifest.environment_seed)
        result['spawn_clearance_cells'] = len(sim.verify_spawn_clearance())
        result['start_area'] = asdict(sim.verify_start_area(manifest, 'semantic-target-preflight'))
        grid = shared_grid or oracle_grid(sim, manifest)
        costmap = Costmap.from_robot(grid, manifest.robot)
        costmap.prepare()
        view = Pose(task['view_pose'][0], task['view_pose'][1], 0., task['view_pose'][2])
        result['view_pose'] = asdict(view)
        result['view_pose_valid'] = costmap.pose_valid(view)
        route = LatticeSearch(grid, manifest.robot).solve(start, {'view': view}, 30.)['view']
        result['view_route'] = asdict(route)

        initial = []
        for yaw in (0., pi / 2., pi, -pi / 2.):
            pose = Pose(start.x_m, start.y_m, 0., yaw)
            set_robot_pose(sim, pose)
            observation = sim.capture()
            frame = store.get_frame(manifest.episode_id, observation.sequence)
            target_visible = any(item.class_name == spec.class_name and item.visibility >= .1
                                 for item in frame.instances)
            detections = detector.detect(observation.color_image)
            initial.append(dict(yaw_rad=yaw, truth_visible=target_visible,
                detected_target=any(item.class_name == spec.class_name for item in detections)))
        result['initial_views'] = initial

        close_checks = []
        yaw_offsets = task.get('view_yaw_offsets', [-.05, 0., .05])
        for delta in yaw_offsets:
            pose = replace(view, yaw_rad=view.yaw_rad + delta)
            set_robot_pose(sim, pose)
            observation = sim.capture()
            frame = store.get_frame(manifest.episode_id, observation.sequence)
            truths = [item for item in frame.instances if item.class_name == spec.class_name]
            if not truths:
                close_checks.append(dict(yaw_rad=pose.yaw_rad, truth_visible=False,
                                         best_iou=0., confidence=None))
                continue
            truth = truths[0]
            _, best_iou, confidence = matched_detection(detector, observation, truth)
            close_checks.append(dict(yaw_rad=pose.yaw_rad, truth_visible=True,
                truth=asdict(truth), best_iou=best_iou, confidence=confidence))
            if len(close_checks) == 1:
                rgb = np.frombuffer(observation.color_image.data, dtype=np.uint8).reshape(480, 640, 3)
                Image.fromarray(rgb).save(output / (task['task_id'] + '.png'))
        result['close_views'] = close_checks
        start_area_ok = (bool(result['start_area']['cells']) and not result['start_area']['rejected']
                         and result['start_area']['verification'].get('all_cells_clear') is True
                         and result['start_area']['verification'].get('floor_support_verified') is True)
        matched_close_views = sum(item['truth_visible']
            and item['best_iou'] >= manifest.evaluation.minimum_bbox_iou for item in close_checks)
        result['matched_close_views'] = matched_close_views
        result['passed'] = bool(start_area_ok and result['view_pose_valid'] and route.success
            and not any(item['detected_target'] for item in initial)
            and matched_close_views >= 1)
        if not result['passed']:
            result['error'] = 'Target failed initial-detection, reachability, or close-view YOLO/IoU gate.'
        return result, grid
    except Exception as exc:
        result['error'] = type(exc).__name__ + ': ' + str(exc)
        return result, shared_grid
    finally:
        sim.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT / 'configs/semantic-navigation-smoke.json')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--task-id', action='append', default=[],
                        help='Run only this task id; repeat to select multiple tasks.')
    args = parser.parse_args()
    suite = json.loads(args.config.read_text(encoding='utf-8'))
    args.output.mkdir(parents=True, exist_ok=True)

    import torch
    from frontier_search.runner.integrated import DisplayDetector
    from frontier_search.sensors import ImageBuffer
    torch.set_num_threads(4)
    device = '0' if torch.cuda.is_available() else 'cpu'
    detector = DisplayDetector(str(ROOT / 'weights/yolo11/yolo11s.pt'), target_class='cup',
                               tile_every_frames=3, minimum_confidence=.50, device=device)
    detector.detect(ImageBuffer(640, 480, 'rgb8', bytes(640 * 480 * 3)))
    selected = [task for task in suite['tasks']
                if not args.task_id or task['task_id'] in set(args.task_id)]
    missing = sorted(set(args.task_id) - {task['task_id'] for task in selected})
    if missing:
        parser.error('unknown task id(s): ' + ', '.join(missing))
    results, grid = [], None
    for task in selected:
        detector.target_class = task['target']['class_name']
        result, grid = preflight_task(task, suite, detector, args.output, grid)
        results.append(result)
        atomic(args.output / (task['task_id'] + '.json'), result)
        print(task['task_id'], 'PASS' if result['passed'] else 'FAIL', result.get('error', ''), flush=True)
    report = dict(schema_version='1.0', passed=all(item['passed'] for item in results),
                  config=str(args.config.resolve()), device=device,
                  selected_task_ids=[task['task_id'] for task in selected], results=results)
    report_name = ('preflight.json' if not args.task_id else
                   'preflight-' + '-'.join(args.task_id) + '.json')
    atomic(args.output / report_name, report)
    return int(not report['passed'])


if __name__ == '__main__':
    raise SystemExit(main())
