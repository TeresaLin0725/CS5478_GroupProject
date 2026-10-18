"""Owner B: iGibson simulator adapter for the Locobot (native fixed ``eyes`` camera).

STATUS: written on a machine WITHOUT iGibson/PyBullet. The logic in ``IGibsonAdapter`` (clock,
stepping, velocity limits, contact counting, spawn clearance, pose checks, angular-sign calibration)
is unit-tested against a fake backend. ``IGibsonBackend`` is the thin layer that touches iGibson and
PyBullet; it only uses calls already used by A's ``capture_igibson.py``/``record_sequence.py`` and
C's ``igibson_rs.py``, but it has NEVER been run against the real simulator. Run it on the team's
iGibson machine and expect to fix small API details before trusting any result.

Robotics background
-------------------
* The runner owns time: ``step()`` is the only call that advances the physics, ``capture()`` and
  ``current_pose()`` are read-only. The episode clock is the number of control steps since the
  robot settled times the render timestep, so it starts at 0.
* Differential-drive (unicycle) command: ``set_velocity(v, w)`` is a body linear speed and a yaw
  rate (counter-clockwise positive, the project convention). iGibson's Locobot wheel ordering makes
  a positive command turn clockwise in C's measurements, so the adapter applies a configurable
  ``angular_command_sign`` and offers ``calibrate_angular_sign()`` to MEASURE it instead of assuming.
* The camera is rigidly mounted: viewpoint changes come only from base motion, and each capture
  returns the actual capture-time camera-to-world transform (A's ``convert_igibson_frames``).
* Spawn clearance is a setup check against scene geometry (permitted by the interface freeze).
  Only the footprint-plus-margin cells are returned; the scene's free space is never exposed.
* ``collisions`` counts CONTROL STEPS with force-bearing non-floor contact, not distinct collision
  events; report it with that definition.

Annotations stay Python 3.8 compatible; iGibson/PyBullet/NumPy are imported lazily.
"""

from dataclasses import dataclass, replace
from math import hypot, pi
from typing import Callable, List, Optional, Tuple

from ..config import MappingConfig
from ..contracts import GridCell, OccupancyGrid, Pose, RGBDObservation, RobotConfig
from ..navigation.geometry import cells_overlapping_footprint
from .rgbd import convert_igibson_frames
from .start_area_verifier import CallbackSceneProbe, SceneProbe, StartAreaResult, verify_start_area


@dataclass(frozen=True)
class AdapterConfig:
    scene_id: str = "Rs_int"
    image_width: int = 320            # perception.locobot.json
    image_height: int = 240
    vertical_fov_deg: float = 60.0
    physics_dt_s: float = 1.0 / 120.0
    control_dt_s: float = 1.0 / 30.0  # one adapter step() = one render/control step
    settle_steps: int = 20
    # Positive angular command turns Locobot clockwise in C's measurement; UNVALIDATED here.
    angular_command_sign: float = -1.0
    # Reset must leave the robot where requested: A requires the first capture pose to equal the
    # manifest start pose, and poses are never rewritten.
    start_xy_tolerance_m: float = 0.01
    start_yaw_tolerance_rad: float = 0.05
    start_z_tolerance_m: float = 0.005
    contact_force_n: float = 0.1       # A's recording guard
    contact_height_above_base_m: float = 0.06  # ignore wheel/floor contacts near the base plane

    def __post_init__(self) -> None:
        if self.angular_command_sign not in (1.0, -1.0):
            raise ValueError("angular_command_sign must be +1 or -1")
        if self.control_dt_s <= 0 or self.physics_dt_s <= 0 or self.settle_steps < 0:
            raise ValueError("timesteps must be positive and settle_steps nonnegative")


class IGibsonBackendProtocol:
    """What the adapter needs from the physics/renderer (implemented by IGibsonBackend or a fake)."""

    snapshot_id = ""

    def open(self, scene_id: str, seed: int, start_pose: Pose) -> None: ...
    def pose(self) -> Pose: ...
    def apply_action(self, linear: float, angular_command: float) -> None: ...
    def step(self) -> None: ...
    def render(self): ...          # -> (rgb, xyz, intrinsics 3x3, view matrix 4x4)
    def in_contact(self, base_z: float, min_height_m: float, min_force_n: float) -> bool: ...
    def scene_probe(self, mapping: MappingConfig) -> SceneProbe: ...
    def close(self) -> None: ...


class IGibsonAdapter:
    def __init__(self, robot: RobotConfig, mapping: MappingConfig, config: Optional[AdapterConfig] = None,
                 backend_factory: Optional[Callable[[AdapterConfig], IGibsonBackendProtocol]] = None) -> None:
        self._robot, self._mapping = robot, mapping
        self._config = config or AdapterConfig()
        self._factory = backend_factory or (lambda cfg: IGibsonBackend(cfg))
        self._backend = None  # type: Optional[IGibsonBackendProtocol]
        self._episode_id = ""
        self._v = self._w = 0.0
        self._paused = self._closed = False
        self._steps = 0
        self._sequence = 0
        self.collisions = 0
        self.contact_this_step = False
        self.executed_distance_m = 0.0
        self.settled_pose = None  # type: Optional[Pose]
        self._odometry = (None, None)

    # --- SimulatorAdapter ---------------------------------------------------

    def reset(self, episode_id: str, scene_id: str, start_pose: Pose, seed: int) -> None:
        """Load the scene, place the Locobot, let it settle, then zero the episode clock.

        Raises ValueError if the settled pose is not the requested one: the manifest's start pose
        must equal the first capture pose, so use ``settled_pose`` to regenerate the manifest.
        """
        if self._backend is not None:
            self._backend.close()
        self._backend = self._factory(self._config)
        self._backend.open(scene_id or self._config.scene_id, seed, start_pose)
        for _ in range(self._config.settle_steps):
            self._backend.apply_action(0.0, 0.0)
            self._backend.step()
        self.settled_pose = self._backend.pose()
        c, s = self._config, self.settled_pose
        if (hypot(s.x_m - start_pose.x_m, s.y_m - start_pose.y_m) > c.start_xy_tolerance_m
                or abs((s.yaw_rad - start_pose.yaw_rad + pi) % (2 * pi) - pi) > c.start_yaw_tolerance_rad
                or abs(s.z_m - start_pose.z_m) > c.start_z_tolerance_m):
            raise ValueError("robot did not settle at the requested start pose; settled at %r" % (s,))
        self._episode_id = episode_id
        self._v = self._w = 0.0
        self._paused = self._closed = False
        self._steps = self._sequence = 0
        self.collisions, self.executed_distance_m = 0, 0.0
        self.contact_this_step = False

        self._odometry = (None, None)

    def verify_spawn_clearance(self) -> Tuple[GridCell, ...]:
        """Footprint-plus-margin cells at the settled pose, each tested as a full collision column.

        Rejects (ValueError) if any cell is blocked. Returns only those cells, in A's grid indices.
        """
        self._require_open()
        probe = self._backend.scene_probe(self._mapping)
        pose, m, r = self.settled_pose, self._mapping, self._robot
        grid = OccupancyGrid('world', 0, m.resolution_m, m.origin_x_m, m.origin_y_m,
                             m.width_cells, m.height_cells, ())
        cells = tuple(GridCell(row, col) for row, col in cells_overlapping_footprint(
            grid, r.footprint_base, pose, r.clearance_margin_m))
        z_min = max(m.floor_height_m + m.floor_tolerance_m, pose.z_m + r.collision_z_min_m - m.base_elevation_margin_m)
        z_max = pose.z_m + r.collision_z_max_m + m.base_elevation_margin_m
        blocked = []  # type: List[GridCell]
        for cell in cells:
            if not (0 <= cell.row < m.height_cells and 0 <= cell.column < m.width_cells):
                blocked.append(cell)
                continue
            x0, y0 = m.origin_x_m + cell.column * m.resolution_m, m.origin_y_m + cell.row * m.resolution_m
            if not probe.column_clear(x0, y0, x0 + m.resolution_m, y0 + m.resolution_m, z_min, z_max):
                blocked.append(cell)
        if not cells or blocked:
            raise ValueError("spawn footprint clearance cannot be verified; blocked cells: %s"
                             % [(c.row, c.column) for c in blocked])
        return cells

    def verify_start_area(self, manifest, verifier_id: str) -> StartAreaResult:
        """Opt-in larger verified starting area (verified_start_area.md) from this scene snapshot."""
        self._require_open()
        return verify_start_area(manifest, self._backend.scene_probe(self._mapping), verifier_id)

    def capture(self) -> RGBDObservation:
        """RGB-D at the current state with the actual pose and capture-time camera transform. No stepping."""
        self._require_open()
        rgb, xyz, intrinsics, view = self._backend.render()
        self._sequence += 1
        observation, _ = convert_igibson_frames(rgb, xyz, intrinsics, view, self._backend.pose(),
                                                self._episode_id, self._sequence, self.sim_time_s())
        return replace(observation, odometry_linear_speed_mps=self._odometry[0],
                       odometry_angular_speed_radps=self._odometry[1])

    def current_pose(self) -> Pose:
        self._require_open()
        return self._backend.pose()

    def step(self) -> None:
        """Advance exactly one control step with the stored velocity command."""
        self._require_open()
        if self._paused:
            raise RuntimeError("step() called while paused")
        before = self._backend.pose()
        self._backend.apply_action(self._v, self._config.angular_command_sign * self._w)
        self._backend.step()
        self._steps += 1
        after = self._backend.pose()
        dt = self._config.control_dt_s
        self._odometry = (hypot(after.x_m-before.x_m, after.y_m-before.y_m)/dt,
                          abs((after.yaw_rad-before.yaw_rad+pi) % (2*pi)-pi)/dt)
        self.executed_distance_m += hypot(after.x_m - before.x_m, after.y_m - before.y_m)
        self.contact_this_step = self._backend.in_contact(after.z_m,
            self._config.contact_height_above_base_m, self._config.contact_force_n)
        if self.contact_this_step:
            self.collisions += 1

    def stop(self) -> None:
        self._v = self._w = 0.0
        if self._backend is not None and not self._closed:
            self._backend.apply_action(0.0, 0.0)

    def pause(self) -> None:
        self._paused = True

    def resume(self) -> None:
        self._paused = False

    def close(self) -> None:
        if self._backend is not None and not self._closed:
            self.stop()
            self._backend.close()
        self._closed = True

    # --- ControlInterface (B's local seam, navigation/navigator.py) ---------

    def sim_time_s(self) -> float:
        return self._steps * self._config.control_dt_s

    def set_velocity(self, linear_mps: float, angular_radps: float) -> None:
        r = self._robot
        self._v = max(-r.max_linear_speed_mps, min(r.max_linear_speed_mps, linear_mps))
        self._w = max(-r.max_angular_speed_radps, min(r.max_angular_speed_radps, angular_radps))

    # --- validation helpers -------------------------------------------------

    def calibrate_angular_sign(self, command_radps: float = 0.4, steps: int = 30) -> float:
        """MEASURE which command sign yields counter-clockwise yaw, then adopt it.

        Turns in place with a positive command and compares the signed yaw change. Run it while
        stopped and clear of obstacles; it changes the robot's heading, so call it BEFORE the
        episode's ``reset`` (or reset again afterwards). Returns the sign now in use.
        """
        self._require_open()
        start = self._backend.pose().yaw_rad
        for _ in range(steps):
            self._backend.apply_action(0.0, command_radps)
            self._backend.step()
        self._backend.apply_action(0.0, 0.0)
        turned = (self._backend.pose().yaw_rad - start + pi) % (2 * pi) - pi
        if abs(turned) < 0.02:
            raise RuntimeError("calibration turn too small to determine the command sign")
        self._config = replace(self._config, angular_command_sign=1.0 if turned > 0 else -1.0)
        return self._config.angular_command_sign

    def _require_open(self) -> None:
        if self._backend is None or self._closed:
            raise RuntimeError("adapter is not reset or is closed")


class IGibsonBackend(IGibsonBackendProtocol):
    """The ONLY code that touches iGibson/PyBullet. Unverified against the real simulator.

    Calls mirror A's ``capture_igibson.py``/``record_sequence.py`` (Locobot, native camera, raw
    m/s and rad/s commands with no normalisation) and C's ``igibson_rs.py``.
    """

    def __init__(self, config: AdapterConfig) -> None:
        self._config = config
        self._cache = {}

    def open(self, scene_id: str, seed: int, start_pose: Pose) -> None:
        import hashlib
        import random
        from pathlib import Path
        import numpy as np
        import pybullet as p
        from igibson.render.mesh_renderer.mesh_renderer_settings import MeshRendererSettings
        from igibson.robots.locobot import Locobot
        from igibson.scenes.igibson_indoor_scene import InteractiveIndoorScene
        from igibson.simulator import Simulator

        self._np, self._p = np, p
        random.seed(seed)
        np.random.seed(seed)
        c = self._config
        self._sim = Simulator(mode="headless", image_width=c.image_width, image_height=c.image_height,
                              vertical_fov=c.vertical_fov_deg, physics_timestep=c.physics_dt_s,
                              render_timestep=c.control_dt_s,
                              rendering_settings=MeshRendererSettings(optimized=True, hide_robot=False))
        scene = InteractiveIndoorScene(scene_id, build_graph=False, should_open_all_doors=True,
                                       include_robots=False)
        self._sim.import_scene(scene)
        self._scene = scene
        # Raw m/s and rad/s: no command limits, exactly as A's recorder configures the controller.
        self._robot = Locobot(action_type="continuous", action_normalize=False,
                              controller_config={"base": {"name": "DifferentialDriveController",
                                                          "command_input_limits": None,
                                                          "command_output_limits": None}})
        self._sim.import_object(self._robot)
        self._bodies = set(self._robot.get_body_ids())
        self._robot.set_position_orientation([start_pose.x_m, start_pose.y_m, start_pose.z_m],
                                             p.getQuaternionFromEuler([0, 0, start_pose.yaw_rad]))
        self._robot.reset()
        model_hash = hashlib.sha256(Path(self._robot.model_file).read_bytes()).hexdigest()[:12]
        self.snapshot_id = "igibson:%s:seed%d:locobot-urdf-%s:at-reset" % (scene_id, seed, model_hash)

    def pose(self) -> Pose:
        x, y, z = (float(v) for v in self._robot.get_position())
        yaw = float(self._p.getEulerFromQuaternion(self._robot.get_orientation())[2])
        return Pose(x, y, z, yaw)

    def apply_action(self, linear: float, angular_command: float) -> None:
        self._robot.apply_action(self._np.array([linear, angular_command]))

    def step(self) -> None:
        self._sim.step()

    def render(self):
        renderer = self._sim.renderer
        rgb, xyz = renderer.render_robot_cameras(modes=("rgb", "3d"))
        return rgb, xyz, renderer.get_intrinsics(), renderer.V

    def in_contact(self, base_z: float, min_height_m: float, min_force_n: float) -> bool:
        p = self._p
        return any(contact[2] not in self._bodies and contact[6][2] > base_z + min_height_m
                   and contact[9] > min_force_n
                   for body in self._bodies for contact in p.getContactPoints(bodyA=body))

    def scene_probe(self, mapping: MappingConfig) -> SceneProbe:
        p = self._p
        obstacles_of = lambda extra=(): [p.getBodyUniqueId(i) for i in range(p.getNumBodies())
                                         if p.getBodyUniqueId(i) not in self._bodies | set(extra)]

        def column_blocked(x0, y0, x1, y1, z0, z1):
            # Full cell-square column against all non-robot collision bodies (A's development check).
            half = [(x1 - x0) / 2, (y1 - y0) / 2, (z1 - z0) / 2]
            shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
            probe = p.createMultiBody(0, shape, basePosition=[(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2])
            try:
                return any(p.getClosestPoints(probe, other, distance=0) for other in obstacles_of([probe]))
            finally:
                p.removeBody(probe)

        def floor_heights(points):
            top, bottom = mapping.floor_height_m + 0.15, mapping.floor_height_m - 0.15
            heights = []
            # These backends import robot links in Bullet's default group 1.
            # Exclude the robot from rays rather than repeatedly casting from
            # inside its convex links, which can hit the same link indefinitely.
            # This setup-only query never advances physics; restore every link.
            links = [(body,link) for body in self._bodies
                     for link in range(-1,p.getNumJoints(body))]
            try:
                for body,link in links:
                    p.setCollisionFilterGroupMask(body,link,2,-1)
                hits = p.rayTestBatch([[x,y,top] for x,y in points],
                                     [[x,y,bottom] for x,y in points],collisionFilterMask=1)
                heights = [hit[3][2] if hit[0] != -1 else None for hit in hits]
            finally:
                for body,link in links:
                    p.setCollisionFilterGroupMask(body,link,1,-1)
            return heights

        return CallbackSceneProbe(column_blocked, floor_heights, nominal_floor_z=mapping.floor_height_m,
                                  snapshot_id=self.snapshot_id,
                                  description="PyBullet closest-point box column and downward ray floor support "
                                              "against all non-robot bodies (unverified on real iGibson)")

    def close(self) -> None:
        self._sim.disconnect()
