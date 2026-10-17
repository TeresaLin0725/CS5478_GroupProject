"""iGibson 2.2 Gibson `Rs` scan: Locobot, RGB-D occupancy mapping, frontiers.

Real iGibson assets and renderer: the Rs apartment is a static photogrammetry
scan, the robot is iGibson's differential-drive Locobot moved only by wheel
velocity commands, and the map is fused from rendered depth images.

Explicit simplifications, recorded in every log:
- Gibson static scans carry no object or room semantics. Furniture classes are
  hand-annotated boxes (LANDMARKS) reported only when the rendered depth image
  shows them within range: an annotated-landmark / visible-class oracle, not YOLO.
- The target is a real YCB object inserted into the scene as a static body; it is
  detected from rendered instance segmentation and localised from depth pixels.
- Region polygons are hand-drawn room boxes (a region prior); room types are
  never given to the policy.
- The camera is Locobot's `eyes` link; its head tilt is set to -25 degrees while
  driving and swept to -25 and 0 degrees at each scan heading.
"""
import heapq
import os
from collections import deque
from math import atan2, cos, hypot, pi, radians, sin, tan
from pathlib import Path

import numpy as np

from ..contracts import (CandidateAssessment, CandidateEvaluation, ExploredGrid,
    FailureReason, Frontier, GridCell, ObjectTrack, OccupancyGrid, Point2, Point3,
    Pose, RegionSummary, TargetClaim, WorldState)


DATA_ROOT = Path(__file__).resolve().parents[3] / 'data' / 'igibson-data'
SCENE_ID = 'Rs'

# Map covering the Rs apartment, world frame (x east, y as in iGibson maps).
RES = 0.1
ORIGIN_X, ORIGIN_Y = -4.5, -4.5
WIDTH, HEIGHT = 70, 85

IMAGE_W, IMAGE_H = 640, 480
VERTICAL_FOV_DEG = 60.0
CAMERA_PITCH_RAD = radians(-25.0)     # head tilt for floor/obstacle mapping
LEVEL_PITCH_RAD = 0.0                  # second tilt in scans: counter/table tops
SENSOR_RANGE_M = 4.0
FLOOR_MAX_Z = 0.06
OBSTACLE_MIN_Z = 0.08
OCCUPIED_MIN_HITS = 3
CONTROL_DT = 1.0 / 30.0
PHYSICS_DT = 1.0 / 120.0
MAX_LINEAR_MPS = 0.35
MAX_ANGULAR_RADPS = 1.2
INITIAL_CLEARANCE_M = 0.6

START_POSE = (0.8, -2.4, pi / 2)
TARGET_CLASS = 'cracker_box'
TARGET_MODEL = '003_cracker_box'
# Flat kitchen counter spot beside the stove (ray-cast top 0.880 m, flat within
# 15 mm); centre = top + half box height. Not visible from the start pose.
TARGET_POSITION = (-1.4, 2.95, 0.945)

# Hand-annotated from the Rs floor plan and checked against rendered views:
# (class, centre xyz, half extents xyz). Never passed to the policy directly.
LANDMARKS = (
    ('sofa', (-1.5, -1.7, 0.35), (0.45, 0.9, 0.3)),
    ('coffee_table', (-0.48, -1.25, 0.38), (0.3, 0.6, 0.05)),
    ('dining_table', (1.5, 0.4, 0.74), (0.35, 0.45, 0.04)),
    ('desk', (0.15, -3.8, 0.74), (0.7, 0.25, 0.04)),
    ('television', (1.6, -1.5, 1.0), (0.15, 0.4, 0.25)),
    ('bed', (-3.1, -0.8, 0.6), (0.8, 1.05, 0.05)),
    ('stove', (-1.6, 2.1, 0.9), (0.3, 0.3, 0.05)),
    ('sink', (-0.8, 3.12, 0.9), (0.4, 0.25, 0.05)),
    ('refrigerator', (0.1, 3.05, 1.2), (0.35, 0.35, 0.5)),
    ('toilet', (-3.52, 3.08, 0.4), (0.2, 0.3, 0.15)),
    ('shower', (-3.45, 2.15, 1.0), (0.4, 0.4, 0.5)),
)

# Hand-drawn room boxes (x0, y0, x1, y1): allowed region prior, unlabeled.
REGIONS = (
    ('R1', (-2.05, -4.2, 1.95, 1.5)),
    ('R2', (-4.1, -2.2, -2.05, 1.5)),
    ('R3', (-4.1, 1.5, -2.1, 3.6)),
    ('R4', (-2.1, 1.5, 1.95, 3.6)),
)


def configure_paths(data_root=DATA_ROOT):
    """Point iGibson at project-local assets; must run before importing igibson."""
    os.environ.setdefault('GIBSON_ASSETS_PATH', str(Path(data_root) / 'robot_assets' / 'assets'))
    os.environ.setdefault('GIBSON_DATASET_PATH', str(Path(data_root) / 'g_dataset'))
    os.environ.setdefault('IG_LOG_LEVEL', 'WARNING')


def wrap(angle):
    return (angle + pi) % (2 * pi) - pi


def world_to_cell(x, y):
    return int((y - ORIGIN_Y) // RES), int((x - ORIGIN_X) // RES)


def cell_center(row, col):
    return ORIGIN_X + (col + 0.5) * RES, ORIGIN_Y + (row + 0.5) * RES


def region_of(x, y):
    for region_id, (x0, y0, x1, y1) in REGIONS:
        if x0 <= x < x1 and y0 <= y < y1:
            return region_id
    return None


class IGibsonWorld:
    """Owns the iGibson simulator; ground truth here is for motion and scoring only."""

    def __init__(self, data_root=DATA_ROOT, start=START_POSE, target_position=TARGET_POSITION,
                 profile='apartment', target_model=TARGET_MODEL, image_width=IMAGE_W, image_height=IMAGE_H):
        configure_paths(data_root)
        import pybullet as p
        from igibson.objects.ycb_object import YCBObject
        from igibson.render.mesh_renderer.mesh_renderer_settings import MeshRendererSettings
        from igibson.robots.locobot import Locobot
        from igibson.scenes.gibson_indoor_scene import StaticIndoorScene
        from igibson.simulator import Simulator
        self.p = p
        self.sim = Simulator(mode='headless', image_width=image_width, image_height=image_height,
                             vertical_fov=VERTICAL_FOV_DEG, physics_timestep=PHYSICS_DT,
                             render_timestep=CONTROL_DT,
                             rendering_settings=MeshRendererSettings(enable_shadow=False, msaa=False,
                                                                     hide_robot=False))
        # No traversability graph: the planner uses only the sensed map.
        self.scene = StaticIndoorScene(SCENE_ID, build_graph=False)
        self.sim.import_scene(self.scene)
        self.robot = Locobot(action_type='continuous', action_normalize=True,
                             rendering_params={'use_pbr': False, 'use_pbr_mapping': False},
                             controller_config={'base': {'name': 'DifferentialDriveController'}})
        self.sim.import_object(self.robot)
        # Non-PBR shading: PBR environment maps are not in the public assets and
        # PBR objects would render black (segmentation is unaffected).
        self.target = YCBObject(target_model, rendering_params={'use_pbr': False, 'use_pbr_mapping': False})
        self.sim.import_object(self.target)
        self.target_body = self.target.get_body_ids()[0]
        # Static body: dynamic YCB boxes slide off the uneven scanned counter into the sink.
        p.changeDynamics(self.target_body, -1, mass=0)
        self.target.set_position_orientation(list(target_position), [0, 0, 0, 1])
        self.robot_bodies = set(self.robot.get_body_ids())
        self.floor_bodies = set(self.scene.floor_body_ids)
        from .indoor_challenges import import_furniture
        self.profile = profile
        self.furniture = import_furniture(self, profile, Path(data_root).parent / 'demo-props')
        self.environment_events = []
        self.cart_started_s = None
        # Measured: the spherical casters snag on millimetre lips in the scanned
        # floor mesh (e.g. the kitchen threshold). They ride on iGibson's flat
        # floor plane only; wheels and body keep full collision with the scan.
        for body in self.robot_bodies:
            for link in range(self.p.getNumJoints(body)):
                if 'caster' in self.p.getJointInfo(body, link)[12].decode():
                    self.p.setCollisionFilterPair(body, self.scene.mesh_body_id, link, -1, 0)
        limits = self.robot._controllers['base'].command_output_limits
        self.max_command = (float(limits[1][0]), float(limits[1][1]))
        x, y, yaw = start
        self.robot.set_position_orientation([x, y, 0.05], p.getQuaternionFromEuler([0, 0, yaw]))
        self.robot.reset()
        for _ in range(20):
            self.robot.apply_action(np.zeros(2))
            self.sim.step()
        self.robot_radius, self.robot_height = self._footprint()
        self.steps, self.distance, self.collision_steps = 0, 0.0, 0
        pose = self.pose()
        self.trajectory = [(pose.x_m, pose.y_m)]
        self.last_rgb = np.zeros((IMAGE_H, IMAGE_W, 3), dtype=np.uint8)
        self.last_target_bbox = None
        self.step_observer = None
        self.spectator_eye = None
        self.spectator_target = None
        self.spectator_offset = 0
        self.spectator_last_s = 0.0
        self.camera_pitch = CAMERA_PITCH_RAD

    def _footprint(self):
        """Half-width of the robot's overall AABB about its base (round base, arm stowed)."""
        pose = self.pose()
        radius, top = 0.0, 0.0
        for body in self.robot_bodies:
            for link in range(-1, self.p.getNumJoints(body)):
                low, high = self.p.getAABB(body, link)
                top = max(top, high[2])
                radius = max(radius, abs(low[0] - pose.x_m), abs(high[0] - pose.x_m),
                             abs(low[1] - pose.y_m), abs(high[1] - pose.y_m))
        return radius, top

    @property
    def sim_time(self):
        return self.steps * CONTROL_DT

    def pose(self):
        x, y, _ = self.robot.get_position()
        return Pose(float(x), float(y), 0.0, float(self.robot.get_rpy()[2]))

    def drive(self, linear, angular):
        """One 1/30 s control step of wheel commands; returns obstacle contact."""
        before = self.pose()
        self._update_challenge(before)
        # Measured: iGibson's Locobot wheel ordering makes a positive angular
        # command turn clockwise, so negate to keep yaw counter-clockwise positive.
        action = np.array([np.clip(linear / self.max_command[0], -1, 1),
                           np.clip(-angular / self.max_command[1], -1, 1)])
        self.robot.apply_action(action)
        self.sim.step()
        self.steps += 1
        after = self.pose()
        self.distance += hypot(after.x_m - before.x_m, after.y_m - before.y_m)
        if self.steps % 6 == 0:
            self.trajectory.append((after.x_m, after.y_m))
        touching = False
        for body in self.robot_bodies:
            for contact in self.p.getContactPoints(bodyA=body):
                if (contact[2] not in self.robot_bodies and contact[2] not in self.floor_bodies
                        and contact[5][2] > 0.04 and contact[9] > 0.01):
                    touching = True
        self.collision_steps += int(touching)
        if self.step_observer is not None:
            self.step_observer()
        return touching

    def _update_challenge(self, pose):
        if self.profile != 'dynamic':
            return
        if self.cart_started_s is None and self.sim_time > 16 and pose.y_m > -1.4 and pose.x_m > .1:
            self.cart_started_s = self.sim_time
            self.environment_events.append({'time_s': self.sim_time, 'event': 'cart_crossing',
                                            'text': '推车开始横穿客厅通道'})
        if self.cart_started_s is None:
            return
        elapsed = self.sim_time - self.cart_started_s
        # The cart follows its own timed motion. It is never injected into A's map.
        fraction = min(1.0, elapsed / 3.0) if elapsed < 11 else max(0.0, 1 - (elapsed - 11) / 3.0)
        cart = next(record for record in self.furniture if record['kind'] == 'cart')
        cart['object'].set_position_orientation([1.40 - .70 * fraction, -.10, 0], [0, 0, 0, 1])

    def spectator(self):
        """Third-person iGibson render with the entire physical robot visible."""
        pose = self.pose()
        renderer = self.sim.renderer
        # Stay inside the scanned ceiling. Fall back to a closer camera when a
        # wall occludes the robot; spectator ray checks never reach the mapper.
        body = next(iter(self.robot_bodies))
        target = np.array([pose.x_m, pose.y_m, .36])
        eye = None
        offsets = ((-.85, -.75, 1.60), (.80, -.75, 1.60), (-.75, .70, 1.60), (.70, .70, 1.60), (.25, -.25, 1.35))
        # Keep the current side while it remains clear. The old first-fit
        # selection switched sides as soon as an earlier offset became clear.
        order = [self.spectator_offset] + [i for i in range(len(offsets)) if i != self.spectator_offset]
        for index in order:
            offset = offsets[index]
            candidate = np.array([pose.x_m + offset[0], pose.y_m + offset[1], offset[2]])
            points = [[pose.x_m+dx, pose.y_m+dy, z] for dx,dy,z in
                      ((0,0,.12), (.12,0,.32), (-.12,0,.32), (0,0,self.robot_height-.04))]
            hits = self.p.rayTestBatch([candidate.tolist()] * len(points), points)
            if all(hit[0] == body or hit[0] == -1 or hit[2] > .94 for hit in hits):
                eye = candidate
                self.spectator_offset = index
                break
        if eye is None:
            eye = np.array([pose.x_m + .25, pose.y_m - .25, 1.35])
        dt = max(0.0, self.sim_time - self.spectator_last_s)
        if self.spectator_eye is None:
            self.spectator_eye, self.spectator_target = eye.copy(), target.copy()
        elif dt > 0:
            # A bounded translation also softens a necessary occlusion change.
            delta = eye - self.spectator_eye
            length = np.linalg.norm(delta)
            self.spectator_eye += delta * min(1.0, dt * 1.8 / max(length, 1e-9))
            self.spectator_target += (target-self.spectator_target) * min(1.0, dt * 8)
        self.spectator_last_s = self.sim_time
        renderer.set_fov(72)
        renderer.set_camera(self.spectator_eye, self.spectator_target, np.array([0, 0, 1]))
        rgb = renderer.render(modes=('rgb',), hidden=[])[0]
        return (np.clip(rgb[:, :, :3], 0, 1) * 255).astype(np.uint8)

    def stop(self, steps=3):
        for _ in range(steps):
            self.drive(0.0, 0.0)

    def capture(self, pitch=CAMERA_PITCH_RAD):
        """Render RGB, camera-frame points and instance ids from the tilted eyes camera."""
        pose = self.pose()
        self.camera_pitch = pitch
        eye = np.array(self.robot.eyes.get_position(), dtype=float)
        yaw = pose.yaw_rad
        forward = np.array([cos(yaw) * cos(pitch), sin(yaw) * cos(pitch), sin(pitch)])
        up = np.array([-cos(yaw) * sin(pitch), -sin(yaw) * sin(pitch), cos(pitch)])
        renderer = self.sim.renderer
        renderer.set_fov(VERTICAL_FOV_DEG)
        renderer.set_camera(eye, eye + forward, up)
        rgb, points, instances = renderer.render(modes=('rgb', '3d', 'ins_seg'),
                                                 hidden=self.robot.renderer_instances)
        self.last_rgb = (np.clip(rgb[:, :, :3], 0, 1) * 255).astype(np.uint8)
        ids = np.round(instances[:, :, 0] * 1024).astype(int)
        body_ids = renderer.instance_id_to_pb_id[np.clip(ids, 0, 1023)]
        rows, cols = np.nonzero(body_ids == self.target_body)
        # Display only: outline of the target's rendered segmentation in this frame.
        self.last_target_bbox = (int(cols.min()), int(rows.min()), int(cols.max()), int(rows.max())) if len(rows) else None
        return {'eye': eye, 'view': np.array(renderer.V, dtype=float), 'points_camera': points[:, :, :3],
                'body_ids': body_ids, 'pose': pose, 'time_s': self.sim_time}

    def preview_camera(self):
        """Live RGB preview only; no extra mapper observations or evidence."""
        pose, pitch = self.pose(), self.camera_pitch
        eye = np.array(self.robot.eyes.get_position(), dtype=float)
        forward = np.array([cos(pose.yaw_rad)*cos(pitch),sin(pose.yaw_rad)*cos(pitch),sin(pitch)])
        up = np.array([-cos(pose.yaw_rad)*sin(pitch),-sin(pose.yaw_rad)*sin(pitch),cos(pitch)])
        renderer = self.sim.renderer
        renderer.set_fov(VERTICAL_FOV_DEG)
        renderer.set_camera(eye,eye+forward,up)
        rgb = renderer.render(modes=('rgb',),hidden=self.robot.renderer_instances)[0]
        return (np.clip(rgb[:,:,:3],0,1)*255).astype(np.uint8)

    def target_position(self):
        return tuple(float(v) for v in self.target.get_position())

    def close(self):
        self.sim.disconnect()


class RGBDMapper:
    """A-module stand-in: fuses depth into an occupancy grid, tracks and frontiers."""

    def __init__(self, world, episode_id, target_class=TARGET_CLASS, close_distance_m=None):
        self.world, self.episode_id, self.target_class = world, episode_id, target_class
        self.close_distance_m = close_distance_m
        self.free_hits = np.zeros((HEIGHT, WIDTH), dtype=np.int32)
        self.occ_hits = np.zeros((HEIGHT, WIDTH), dtype=np.int32)
        self.revision, self.sequence = 0, 0
        self.tracks, self.claim = {}, None
        self.target_evidence = []  # (observation id, estimated xyz): evaluator audit only
        self.target_observations = {}
        self.nearby_evidence = []
        self.frontier_history = []
        self.next_id = 1
        focal = (IMAGE_H / 2.0) / tan(radians(VERTICAL_FOV_DEG) / 2.0)
        self.intrinsics = (focal, focal, IMAGE_W / 2.0, IMAGE_H / 2.0)
        pose = world.pose()
        self.initial_clearance = self._disc(pose.x_m, pose.y_m, INITIAL_CLEARANCE_M)
        for row, col in self.initial_clearance:
            self.free_hits[row, col] += 1

    @staticmethod
    def _disc(x, y, radius):
        cells = []
        row0, col0 = world_to_cell(x, y)
        span = int(radius / RES) + 1
        for row in range(row0 - span, row0 + span + 1):
            for col in range(col0 - span, col0 + span + 1):
                cx, cy = cell_center(row, col)
                if 0 <= row < HEIGHT and 0 <= col < WIDTH and hypot(cx - x, cy - y) <= radius:
                    cells.append((row, col))
        return cells

    @property
    def cells(self):
        grid = np.full((HEIGHT, WIDTH), -1, dtype=int)
        grid[self.free_hits > 0] = 0
        grid[self.occ_hits >= OCCUPIED_MIN_HITS] = 100
        return grid

    def mark_footprint(self):
        """Cells under the physical robot body are free by construction."""
        pose = self.world.pose()
        for row, col in self._disc(pose.x_m, pose.y_m, self.world.robot_radius * 0.8):
            self.free_hits[row, col] += 1

    def _add(self, grid, xs, ys):
        rows = np.floor((ys - ORIGIN_Y) / RES).astype(int)
        cols = np.floor((xs - ORIGIN_X) / RES).astype(int)
        ok = (rows >= 0) & (rows < HEIGHT) & (cols >= 0) & (cols < WIDTH)
        np.add.at(grid, (rows[ok], cols[ok]), 1)

    def integrate(self, frame):
        camera = frame['points_camera']
        depth = -camera[:, :, 2]
        valid = (depth > 0.15) & (depth < SENSOR_RANGE_M)
        homogeneous = np.concatenate([camera, np.ones(camera.shape[:2] + (1,))], axis=2)
        world = homogeneous.reshape(-1, 4) @ np.linalg.inv(frame['view']).T
        world = world[:, :3].reshape(camera.shape)
        z = world[:, :, 2]
        obstacle = valid & (z > OBSTACLE_MIN_Z) & (z < self.world.robot_height + 0.05)
        self._add(self.occ_hits, world[:, :, 0][obstacle], world[:, :, 1][obstacle])
        # Free space only from rays whose endpoint is visible floor.
        floor = valid & (z < FLOOR_MAX_Z)
        floor[::, 1::2] = False
        floor[1::2, ::] = False
        ends = world[floor][:, :2]
        if len(ends):
            start = frame['eye'][:2]
            offsets = ends - start
            lengths = np.hypot(offsets[:, 0], offsets[:, 1])
            steps = np.arange(0.0, SENSOR_RANGE_M + RES, RES / 2)
            samples = start + offsets[:, None, :] * (steps[None, :, None] / np.maximum(lengths, 1e-6)[:, None, None])
            inside = steps[None, :] <= lengths[:, None]
            self._add(self.free_hits, samples[:, :, 0][inside], samples[:, :, 1][inside])
        self.sequence += 1
        self._detect(frame, depth, valid, world)

    def _project(self, frame, point):
        camera = frame['view'] @ np.array([point[0], point[1], point[2], 1.0])
        forward = -camera[2]
        if forward <= 0.1:
            return None
        fx, fy, cx, cy = self.intrinsics
        u, v = cx + fx * camera[0] / forward, cy - fy * camera[1] / forward
        if 0 <= u < IMAGE_W and 0 <= v < IMAGE_H:
            return int(v), int(u), forward
        return None

    def _record(self, class_name, position, stamp):
        track_id = 'track-' + class_name
        obs_id = 'capture-%d-%s' % (self.sequence, class_name)
        prior = self.tracks.get(track_id)
        ids = prior.observation_ids + (obs_id,) if prior else (obs_id,)
        self.tracks[track_id] = ObjectTrack(track_id, class_name, 1.0, Point3(*position),
            prior.first_seen_s if prior else stamp, stamp, ids, len(ids))
        return track_id, obs_id, ids

    def _detect(self, frame, depth, valid, world):
        stamp = frame['time_s']
        for class_name, centre, half in LANDMARKS:
            seen = 0
            for sx in (-0.6, 0.0, 0.6):
                for sy in (-0.6, 0.0, 0.6):
                    point = (centre[0] + sx * half[0], centre[1] + sy * half[1], centre[2] + half[2])
                    hit = self._project(frame, point)
                    if hit is None or hit[2] > SENSOR_RANGE_M:
                        continue
                    row, col, expected = hit
                    if valid[row, col] and abs(depth[row, col] - expected) < 0.3:
                        seen += 1
            if seen >= 2:
                # Landmark oracle: class and annotated centre, gated by rendered depth visibility.
                self._record(class_name, centre, stamp)
        mask = (frame['body_ids'] == self.world.target_body) & valid
        if int(mask.sum()) >= 40:
            estimate = tuple(float(v) for v in world[mask].mean(axis=0))
            track_id, obs_id, ids = self._record(self.target_class, estimate, stamp)
            self.target_evidence.append((obs_id, estimate))
            nearby = self.close_distance_m is None or hypot(estimate[0] - frame['pose'].x_m,
                                                           estimate[1] - frame['pose'].y_m) <= self.close_distance_m
            linear, angular = self.world.p.getBaseVelocity(next(iter(self.world.robot_bodies)))
            stopped = hypot(linear[0], linear[1]) < .03 and abs(angular[2]) < .06
            self.target_observations[obs_id] = {'pose': frame['pose'], 'time_s': stamp, 'stopped': stopped}
            if self.close_distance_m is None:
                if len(ids) >= 2 and self.claim is None:
                    self.claim = TargetClaim('claim-1', track_id, self.target_class, stamp, ids[-2:])
            elif nearby and stopped:
                if not self.nearby_evidence or self.target_observations[self.nearby_evidence[-1]]['time_s'] != stamp:
                    self.nearby_evidence.append(obs_id)
                if len(self.nearby_evidence) >= 2 and self.claim is None:
                    self.claim = TargetClaim('claim-1', track_id, self.target_class, stamp, tuple(self.nearby_evidence[-2:]))

    def observe(self, pitch=CAMERA_PITCH_RAD):
        self.integrate(self.world.capture(pitch))
        self.revision += 1

    def traversable(self):
        from scipy import ndimage
        grid = self.cells
        free = grid == 0
        to_occupied = ndimage.distance_transform_edt(grid != 100) * RES
        to_unknown = ndimage.distance_transform_edt(grid != -1) * RES
        clearance = self.world.robot_radius + 0.05
        return free & (to_occupied > clearance) & (to_unknown > RES * 1.5)

    def plan(self, goal_cell=None):
        """8-connected Dijkstra over the inflated known-free grid from the live pose."""
        pose = self.world.pose()
        start = world_to_cell(pose.x_m, pose.y_m)
        safe = self.traversable()
        cost, parent = {start: 0.0}, {start: None}
        queue = [(0.0, start)]
        moves = [(dr, dc, hypot(dr, dc) * RES) for dr in (-1, 0, 1) for dc in (-1, 0, 1) if dr or dc]
        while queue:
            distance, cell = heapq.heappop(queue)
            if distance > cost[cell] or cell == goal_cell:
                if cell == goal_cell:
                    break
                continue
            for dr, dc, step in moves:
                other = (cell[0] + dr, cell[1] + dc)
                if not (0 <= other[0] < HEIGHT and 0 <= other[1] < WIDTH) or not safe[other]:
                    continue
                if dr and dc and not (safe[cell[0] + dr, cell[1]] and safe[cell[0], cell[1] + dc]):
                    continue
                if distance + step < cost.get(other, float('inf')):
                    cost[other], parent[other] = distance + step, cell
                    heapq.heappush(queue, (distance + step, other))
        return cost, parent, safe

    @staticmethod
    def route(parent, goal):
        cells, cell = [], goal
        while cell is not None:
            cells.append(cell)
            cell = parent[cell]
        return [cell_center(*c) for c in reversed(cells)]

    def snapshot(self):
        grid = self.cells
        pose = self.world.pose()
        cost, parent, _ = self.plan()
        unknown = grid == -1
        support = set()
        for row in range(1, HEIGHT - 1):
            for col in range(1, WIDTH - 1):
                if grid[row, col] == 0 and (unknown[row - 1, col] or unknown[row + 1, col]
                                            or unknown[row, col - 1] or unknown[row, col + 1]):
                    support.add((row, col))
        clusters = []
        while support:
            seed = min(support)
            region = region_of(*cell_center(*seed))
            support.remove(seed)
            queue, cluster = deque([seed]), [seed]
            while queue:
                row, col = queue.popleft()
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        cell = (row + dr, col + dc)
                        if cell in support and region_of(*cell_center(*cell)) == region:
                            support.remove(cell)
                            queue.append(cell)
                            cluster.append(cell)
            if len(cluster) >= 4 and region is not None:
                clusters.append((region, cluster))
        frontiers, routes, history, assessments, used = [], {}, [], [], set()
        for region, cluster in clusters:
            centre = np.mean(cluster, axis=0)
            cx, cy = cell_center(centre[0], centre[1])
            # A cluttered boundary can wrap around furniture. Its centroid may
            # lie behind an obstacle although its observed edge is reachable.
            # Assess approach distance to actual support, not that centroid.
            from scipy import ndimage
            edge = np.ones(grid.shape, dtype=bool)
            for cell in cluster:
                edge[cell] = False
            distance_to_edge = ndimage.distance_transform_edt(edge) * RES
            near = [c for c in cost if distance_to_edge[c] <= 1.2
                    and region_of(*cell_center(*c)) == region
                    and hypot(cell_center(*c)[0] - pose.x_m, cell_center(*c)[1] - pose.y_m) >= .35]
            goal = min(near, key=lambda c: (distance_to_edge[c], cost[c], c)) if near else None
            gx, gy = cell_center(*goal) if goal else (cx, cy)
            if goal and hypot(gx - pose.x_m, gy - pose.y_m) < 0.3:
                goal = None  # already here; the arrival scan did not resolve it
            rows, cols = np.ogrid[:HEIGHT, :WIDTH]
            window = (rows - centre[0]) ** 2 + (cols - centre[1]) ** 2 <= (1.0 / RES) ** 2
            gain = float(np.count_nonzero(unknown & window)) * RES * RES
            cells = set(cluster)
            matched = [(fid, len(cells & old), hypot(ox - cx, oy - cy))
                       for fid, old, (ox, oy) in self.frontier_history if fid not in used]
            matched = [m for m in matched if m[1] > 0 or m[2] < 0.5]
            if matched:
                fid = min(matched, key=lambda m: (-m[1], m[2], m[0]))[0]
            else:
                fid = 'F%d' % self.next_id
                self.next_id += 1
            used.add(fid)
            history.append((fid, cells, (cx, cy)))
            yaw = atan2(cy - gy, cx - gx) if goal else pose.yaw_rad
            frontiers.append(Frontier(fid, (), tuple(GridCell(int(r), int(c)) for r, c in sorted(cluster)),
                                      Pose(gx, gy, 0.0, yaw), region, gain, hypot(gx - pose.x_m, gy - pose.y_m)))
            if goal:
                routes[fid] = self.route(parent, goal)
                assessments.append(CandidateAssessment(fid, True, round(cost[goal], 3), None))
            else:
                assessments.append(CandidateAssessment(fid, False, None, FailureReason.NO_PATH))
        self.frontier_history = history
        track = self.tracks.get('track-' + self.target_class)
        if track is not None and self.close_distance_m is not None and self.claim is None:
            tx, ty = track.position_world.x_m, track.position_world.y_m
            options = [c for c in cost if grid[c] == 0
                       and hypot(cell_center(*c)[0] - tx, cell_center(*c)[1] - ty) <= self.close_distance_m - .20]
            if options:
                goal = min(options, key=lambda c: (cost[c], c))
                gx, gy = cell_center(*goal)
                fid = 'approach-' + track.track_id
                frontiers.append(Frontier(fid, (), (GridCell(*goal),),
                    Pose(gx, gy, 0.0, atan2(ty - gy, tx - gx)), region_of(gx, gy), 0.0,
                    hypot(gx - pose.x_m, gy - pose.y_m), 'target_approach', track.track_id))
                routes[fid] = self.route(parent, goal)
                assessments.append(CandidateAssessment(fid, True, round(cost[goal], 3), None))
        regions = []
        for region_id, (x0, y0, x1, y1) in REGIONS:
            r0, c0 = world_to_cell(x0, y0)
            r1, c1 = world_to_cell(x1, y1)
            part = grid[max(r0, 0):r1, max(c0, 0):c1]
            observed = tuple(sorted({t.class_name for t in self.tracks.values()
                                     if region_of(t.position_world.x_m, t.position_world.y_m) == region_id}))
            regions.append(RegionSummary(region_id, (Point2(x0, y0), Point2(x1, y0), Point2(x1, y1), Point2(x0, y1)),
                                         'igibson_room_prior', float(np.mean(part != -1)), observed, None, None))
        occupancy = OccupancyGrid('world', self.revision, RES, ORIGIN_X, ORIGIN_Y, WIDTH, HEIGHT,
                                  tuple(tuple(int(v) for v in row) for row in grid))
        explored = ExploredGrid('world', self.revision, RES, ORIGIN_X, ORIGIN_Y, WIDTH, HEIGHT,
                                tuple(tuple(bool(v != -1) for v in row) for row in grid))
        state = WorldState('1.0', self.episode_id, self.revision, self.world.sim_time, self.sequence,
                           self.target_class, pose, occupancy, explored, tuple(self.tracks.values()),
                           tuple(regions), tuple(frontiers), self.claim)
        evaluation = CandidateEvaluation(self.episode_id, self.revision, pose, tuple(assessments))
        return state, evaluation, routes
