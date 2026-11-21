"""Native Gibson Rs scene, Locobot physics and rendering backend.

The integrated pipeline supplies the public RGB-D mapper and YOLO detector.
Target instance geometry and segmentation remain private to evaluation and
visualization; they are not a substitute for detector observations.
"""
import os
from math import cos, hypot, pi, radians, sin
from pathlib import Path
import numpy as np
from ..contracts import Pose

DATA_ROOT = Path(__file__).resolve().parents[3] / 'data' / 'igibson-data'

SCENE_ID = 'Rs'

IMAGE_W, IMAGE_H = 640, 480

VERTICAL_FOV_DEG = 60.0

CAMERA_PITCH_RAD = radians(-25.0)

CONTROL_DT = 1.0 / 30.0

PHYSICS_DT = 1.0 / 120.0

START_POSE = (0.8, -2.4, pi / 2)

TARGET_MODEL = '003_cracker_box'

TARGET_POSITION = (-1.4, 2.95, 0.945)

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

class IGibsonWorld:
    """Owns the iGibson simulator; ground truth here is for motion and scoring only."""

    def __init__(self, data_root=DATA_ROOT, start=START_POSE, target_position=TARGET_POSITION,
                 profile='apartment', target_model=TARGET_MODEL, image_width=IMAGE_W, image_height=IMAGE_H,
                 scene_layout='legacy', target_orientation=(0, 0, 0, 1)):
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
        self.target.set_position_orientation(list(target_position), list(target_orientation))
        self.robot_bodies = set(self.robot.get_body_ids())
        self.floor_bodies = set(self.scene.floor_body_ids)
        from .indoor_challenges import import_furniture
        self.profile = profile
        self.furniture = import_furniture(self, profile, Path(data_root).parent / 'demo-props',scene_layout)
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
        self.cart_motion = None
        self.cart_blocked = False
        self.cart_finished = False
        if self.profile == 'dynamic':
            from .cart_motion import CartMotion
            cart = next(record for record in self.furniture if record['kind'] == 'cart')
            try:
                self.cart_motion = CartMotion(self.p, cart['object'])
            except Exception:
                self.close()
                raise
            cart['position'] = list(self.cart_motion.start)
            self.environment_events.append({'time_s': 0.0, 'event': 'cart_route_verified',
                                            'start': list(self.cart_motion.start),
                                            'end': list(self.cart_motion.end)})

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
        if self.profile != 'dynamic' or self.cart_blocked or self.cart_finished:
            return
        # The collision-free fallback initially occupies the narrow aisle.
        # Move on the scene clock, before the robot can be trapped behind it,
        # then park beyond the entrance. Original viable routes retain their loop.
        one_way = self.cart_motion.start[0] < 1.4
        ready = one_way or (pose.y_m > -1.4 and pose.x_m > .1)
        if self.cart_started_s is None and self.sim_time > 16 and ready:
            self.cart_started_s = self.sim_time
            self.environment_events.append({'time_s': self.sim_time, 'event': 'cart_crossing',
                                            'text': 'Cart started crossing the living-room passage'})
        if self.cart_started_s is None:
            return
        elapsed = self.sim_time - self.cart_started_s
        # The cart follows its own timed motion. It is never injected into A's map.
        travel_s = 3.0 * hypot(self.cart_motion.end[0] - self.cart_motion.start[0],
                               self.cart_motion.end[1] - self.cart_motion.start[1]) / self.cart_motion.TRAVEL_M
        fraction = min(1.0, elapsed / travel_s) if one_way else (
            min(1.0, elapsed / 3.0) if elapsed < 11 else max(0.0, 1 - (elapsed - 11) / 3.0))
        if not self.cart_motion.move(fraction):
            self.cart_blocked = True
            self.environment_events.append({'time_s': self.sim_time, 'event': 'cart_blocked',
                                            'text': 'Cart swept volume is occupied, stop movement to avoid penetration'})
        elif one_way and fraction >= 1.0:
            self.cart_finished = True
            self.environment_events.append({'time_s': self.sim_time, 'event': 'cart_parked',
                                            'text': 'Cart parked beside the passage, keeping the passage open'})

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
        # Some public Locobot meshes have a zero-valued diffuse material and
        # appear as silhouettes. A slate presentation finish reveals their
        # geometry in this spectator pass only. Restore every value before
        # any sensor/evaluator render can run; physics and RGB-D are untouched.
        materials = {}
        for instance in self.robot.renderer_instances:
            for obj in instance.objects:
                for vao in obj.VAO_ids:
                    material = renderer.material_idx_to_material_instance_mapping[renderer.shape_material_idx[vao]]
                    if not material.is_texture() and max(material.kd[:3]) < .15:
                        materials[id(material)] = (material, material.kd)
        lightpos, lightview = renderer.lightpos, renderer.lightV
        try:
            for material, _ in materials.values():
                material.kd = [.24, .28, .34]
            renderer.set_light_position_direction([pose.x_m+1, pose.y_m-1.1, 3],
                                                  [pose.x_m, pose.y_m, .4])
            rgb = renderer.render(modes=('rgb',), hidden=[])[0]
        finally:
            for material, original in materials.values():
                material.kd = original
            renderer.lightpos, renderer.lightV = lightpos, lightview
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
