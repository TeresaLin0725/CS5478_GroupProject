"""Continuous native-camera recording; scripted motion is not policy execution."""

from dataclasses import asdict, replace
import hashlib
import json
from math import hypot, pi
from pathlib import Path

from ..contracts import Pose
from ..fixtures.recording import save_rgbd_recording
from .rgbd import convert_igibson_frames


DEFAULT_PLAN = ((0.0, 0.4, 1.0), (0.0, -0.4, 2.0), (0.0, 0.4, 1.0),
                (0.1, 0.0, 1.0), (0.0, 0.4, 1.0), (0.0, -0.4, 2.0))


def export_development_setup(root, robot, pose, episode, args):
    """Conservative measured geometry and cell-volume spawn check, for replay only.

    This local development manifest is not the agreed team navigation config.
    Hidden collision geometry is used only to verify the initial seed mask.
    """
    import pybullet as p
    from ..config import MappingConfig
    from ..contracts import Point2, RobotConfig
    from ..fixtures.sequence import footprint_cells
    from ..runner.manifest import EpisodeManifest
    from ..navigation.geometry import circle_footprint

    own = set(robot.get_body_ids())
    # A separate client avoids altering/teleporting the recorded robot. Measure
    # its aligned collision AABBs directly rather than rotating already enlarged
    # world AABBs back into the base frame. Preserve its current joint positions.
    client = p.connect(p.DIRECT)
    corners, relative_top = [], 0.0
    try:
        copy = p.loadURDF(robot.model_file, [0, 0, 0], useFixedBase=True, physicsClientId=client)
        original = next(iter(own))
        for joint in range(p.getNumJoints(original)):
            p.resetJointState(copy, joint, p.getJointState(original, joint)[0], physicsClientId=client)
        for link in range(-1, p.getNumJoints(copy, physicsClientId=client)):
            low, high = p.getAABB(copy, link, physicsClientId=client)
            relative_top = max(relative_top, high[2])
            corners.extend((x, y) for x in (low[0], high[0]) for y in (low[1], high[1]))
    finally:
        p.disconnect(client)
    top = pose.z_m + relative_top
    xmin, xmax = min(c[0] for c in corners), max(c[0] for c in corners)
    ymin, ymax = min(c[1] for c in corners), max(c[1] for c in corners)
    robot_config = RobotConfig(tuple(Point2(x, y) for x, y in
        ((xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax))),
        0.0, top-pose.z_m, 0.02, 'differential_drive_development_recording', True,
        0.1, 0.4, 0.05, 0.1)
    mapping = MappingConfig(resolution_m=0.1, width_cells=100, height_cells=100,
                            origin_x_m=-5, origin_y_m=-5, base_elevation_margin_m=0.005)
    manifest = EpisodeManifest(episode, args.scene, 'igibson', 'cup', pose, robot_config,
                               mapping=mapping, environment_seed=args.seed)
    manifest.validate()
    cells = footprint_cells(manifest)
    rotation_cells = footprint_cells(replace(manifest, robot=replace(robot_config,
        footprint_base=circle_footprint(max(hypot(v.x_m, v.y_m) for v in robot_config.footprint_base)))))
    # Collision columns start above the configured floor band, match the mapper's
    # conservative upper envelope, and include full cell squares (not centres).
    low_z = max(pose.z_m, mapping.floor_height_m + mapping.floor_tolerance_m)
    high_z = top + mapping.base_elevation_margin_m
    shape = p.createCollisionShape(p.GEOM_BOX,
        halfExtents=[mapping.resolution_m/2, mapping.resolution_m/2, (high_z-low_z)/2])
    probe = p.createMultiBody(0, shape, basePosition=[0, 0, (high_z+low_z)/2])
    obstacles = [p.getBodyUniqueId(i) for i in range(p.getNumBodies())
                 if p.getBodyUniqueId(i) not in own | {probe}]
    blocked = []
    try:
        for cell in sorted(set(cells) | set(rotation_cells), key=lambda c: (c.row, c.column)):
            p.resetBasePositionAndOrientation(probe, [mapping.origin_x_m+(cell.column+.5)*mapping.resolution_m,
                mapping.origin_y_m+(cell.row+.5)*mapping.resolution_m, (high_z+low_z)/2], [0, 0, 0, 1])
            if any(p.getClosestPoints(probe, other, distance=0) for other in obstacles):
                blocked.append(asdict(cell))
    finally:
        p.removeBody(probe)
    (root/'development-manifest.json').write_text(json.dumps(asdict(manifest), indent=2), encoding='utf-8')
    initial_blocked = [c for c in blocked if c in [asdict(cell) for cell in cells]]
    result = dict(cells=[asdict(c) for c in cells],
        provenance='Development spawn check: every initial footprint-plus-0.02m-margin grid cell '
                   'tested as a full column against all nonrobot PyBullet collision bodies; '
                   'vertical band %.6f..%.6f m; no extra free-space seed. Not B live-adapter acceptance.'
                   % (low_z, high_z), blocked_cells=initial_blocked, verified=not initial_blocked,
        rotation_blocked_cells=blocked, rotation_clearance_verified=not blocked,
        rotation_check_scope='Circumscribed footprint disc plus margin, setup check only; extra cells never seed mapping')
    (root/'development-clearance-check.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    if not initial_blocked:
        (root/'development-clearance.json').write_text(json.dumps(
            {k: result[k] for k in ('cells', 'provenance')}, indent=2), encoding='utf-8')
    return result


def record_sequence(simulator, robot, scene, demo, config, args):
    """Save a continuous session and retain raw frames even if mapping rejects them."""
    import numpy as np
    import pybullet as p

    root = args.output
    root.mkdir(parents=True, exist_ok=False)
    episode = 'locobot-%s-seed-%d-sequence' % (args.scene, args.seed)
    frames, poses, commands = [], [], []
    report = dict(status='running', episode_id=episode, scene_id=args.scene,
                  capture_config=config, mode='scripted_recording_not_policy_execution',
                  frames=frames, commands=commands, spawn_clearance_verified=False)
    report['scan_mode'] = 'full_base_revolution' if args.full_scan else 'short_move_scan'
    model_hash = hashlib.sha256(Path(robot.model_file).read_bytes()).hexdigest()

    def summary():
        if poses:
            report['base_z_range_m'] = [min(pose.z_m for pose in poses), max(pose.z_m for pose in poses)]
            report['base_z_span_m'] = report['base_z_range_m'][1] - report['base_z_range_m'][0]
            report['recorded_translation_m'] = sum(hypot(b.x_m-a.x_m, b.y_m-a.y_m)
                                                 for a, b in zip(poses, poses[1:]))
            report['start_pose'] = asdict(poses[0])
            report['absolute_recorded_yaw_travel_rad'] = sum(abs((b.yaw_rad-a.yaw_rad+pi) % (2*pi)-pi)
                                                           for a, b in zip(poses, poses[1:]))
            report['signed_recorded_yaw_travel_rad'] = sum((b.yaw_rad-a.yaw_rad+pi) % (2*pi)-pi
                                                         for a, b in zip(poses, poses[1:]))
            report['flat_floor_sequence_compatible'] = all(abs(pose.z_m-poses[0].z_m) <= 1e-6
                                                          for pose in poses)
            report['development_base_margin_m'] = 0.005
            report['development_base_margin_compatible'] = all(abs(pose.z_m-poses[0].z_m) <= 0.005 + 1e-6
                                                              for pose in poses)
        (root / 'capture-report.json').write_text(json.dumps(report, indent=2, allow_nan=False), encoding='utf-8')

    def capture():
        rgb, xyz = simulator.renderer.render_robot_cameras(modes=('rgb', '3d'))
        position = robot.get_position()
        pose = Pose(*(float(v) for v in position),
                    yaw_rad=float(p.getEulerFromQuaternion(robot.get_orientation())[2]))
        observation, error = convert_igibson_frames(rgb, xyz, simulator.renderer.get_intrinsics(),
            simulator.renderer.V, pose, episode, len(frames)+1,
            float(simulator.frame_count * simulator.render_timestep))
        name = 'frames/%06d' % observation.sequence
        save_rgbd_recording(root / name, observation, dict(kind='igibson_capture',
            scene_id=args.scene, robot='Locobot', camera_pose='native_link', camera_link='eyes',
            capture_config=config, robot_model_sha256=model_hash,
            camera_position_world_m=[float(v) for v in robot.eyes.get_position()],
            camera_orientation_world_xyzw=[float(v) for v in robot.eyes.get_orientation()],
            xyz_projection_max_error_m=error, trajectory='scripted_not_policy_executed'))
        demo.cv2.imwrite(str(root / name / 'rgb.png'),
                        np.frombuffer(observation.color_image.data, dtype=np.uint8).reshape(
                            config['image_height'], config['image_width'], 3)[:, :, ::-1])
        frames.append(dict(path=name, sequence=observation.sequence,
                           timestamp_s=observation.timestamp_s, pose=asdict(pose),
                           xyz_projection_max_error_m=error))
        poses.append(pose)
        summary()
        print('Captured frame %d, base z %.8f m' % (observation.sequence, pose.z_m), flush=True)

    bodies = set(robot.get_body_ids())

    def obstacle_contact():
        # Abort on force-bearing contact above the floor band. This is a local
        # recording guard, not B's collision-free navigation/clearance proof.
        z = float(robot.get_position()[2])
        return any(contact[2] not in bodies and contact[6][2] > z + 0.06 and contact[9] > 0.1
                   for body in bodies for contact in p.getContactPoints(bodyA=body))

    try:
        capture()
        clearance = export_development_setup(root, robot, poses[0], episode, args)
        report['spawn_clearance_verified'] = clearance['verified']
        report['rotation_clearance_verified'] = clearance['rotation_clearance_verified']
        report['setup_scope'] = 'local measured conservative geometry; pending B/C configuration review'
        summary()
        if args.full_scan and not clearance['rotation_clearance_verified']:
            raise RuntimeError('full scan requires clear rotation volume; choose another spawn seed')
        plan = ((0.0, -0.4, pi / (4 * 0.4)),) * 8 if args.full_scan else DEFAULT_PLAN
        for linear, angular, duration in plan:
            commands.append(dict(linear_mps=linear, angular_command_radps=angular, duration_s=duration))
            previous_yaw = float(p.getEulerFromQuaternion(robot.get_orientation())[2])
            turned = 0.0
            maximum_duration = duration + 2.0 if args.full_scan else duration
            commands[-1]['maximum_duration_s'] = maximum_duration
            for step in range(round(maximum_duration / simulator.render_timestep)):
                robot.apply_action(np.array([linear, angular]))
                simulator.step()
                commands[-1]['steps_executed'] = step + 1
                if obstacle_contact():
                    raise RuntimeError('recording aborted on obstacle contact; inspect saved frames')
                if args.full_scan:
                    yaw = float(p.getEulerFromQuaternion(robot.get_orientation())[2])
                    turned += (yaw-previous_yaw+pi) % (2*pi)-pi
                    previous_yaw = yaw
                    if turned >= pi/4:
                        break
            if args.full_scan and turned < pi/4:
                raise RuntimeError('full-scan segment failed to reach measured 45-degree heading within deadline')
            robot.apply_action(np.zeros(2))
            for _ in range(12):
                simulator.step()
                if obstacle_contact():
                    raise RuntimeError('recording aborted on obstacle contact during settling')
            capture()
        if args.full_scan and abs(report['signed_recorded_yaw_travel_rad']) < 2*pi-0.15:
            raise RuntimeError('measured rotation did not complete a full scan within 0.15 rad tolerance')
        report['status'] = 'complete'
    except Exception as exc:
        report['status'] = 'failed'
        report['error'] = str(exc)
        raise
    finally:
        robot.apply_action(np.zeros(2))
        summary()
    print('Saved %d continuous frames: %s' % (len(frames), root.resolve()), flush=True)
