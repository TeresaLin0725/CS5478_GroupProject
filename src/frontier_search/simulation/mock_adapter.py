"""Owner B: deterministic in-memory simulator for developing and testing B without iGibson.

Robotics background
-------------------
* Unicycle (differential-drive) kinematics: the robot state is (x, y, yaw) and
  the inputs are linear speed v and angular speed w:
  ``x' = v cos(yaw)``, ``y' = v sin(yaw)``, ``yaw' = w``. Each ``step`` integrates
  these over a fixed time step ``dt`` (midpoint heading, which is exact for
  constant v and w over the step).
* Ground truth vs. belief: this mock owns a *true* occupancy grid used only for
  physical collision. B's planner works from the (possibly partial) map passed in
  by the caller, so tests can exercise the difference between what is known
  and what is real. The ground truth is never exposed to robot modules.
* Physical collision is checked with the exact footprint and *no* planning
  margin; the clearance margin is a planning safety buffer, not physics.

The mock implements both ``SimulatorAdapter`` (simulation/api.py) and B's local
``ControlInterface`` (navigation/navigator.py). It is development scaffolding,
not an observation model: camera images are placeholders.

Only the standard library is used; annotations stay Python 3.8 compatible.
"""

from math import cos, hypot, sin
from typing import Optional, Tuple

from ..contracts import GridCell, OccupancyGrid, Pose, RGBDObservation, RobotConfig
from ..navigation.geometry import (FREE, cell_value, cells_overlapping_footprint,
                                   footprint_collides)
from ..sensors import CameraIntrinsics, ImageBuffer, Transform


class MockSimulator:
    def __init__(self, true_grid: OccupancyGrid, robot: RobotConfig, dt_s: float = 0.05) -> None:
        if dt_s <= 0:
            raise ValueError("dt_s must be positive")
        self._grid = true_grid           # ground truth, used only for physical collision
        self._robot = robot
        self._dt = dt_s
        self._episode_id = ""
        self._pose = Pose(0.0, 0.0, 0.0, 0.0)
        self._t = 0.0
        self._v = 0.0
        self._w = 0.0
        self._sequence = 0
        self._paused = False
        self._closed = False
        self._seed = 0
        self.collisions = 0
        self.executed_distance_m = 0.0   # actual travelled distance (not planned cost)

    # --- SimulatorAdapter ---------------------------------------------------

    def reset(self, episode_id: str, scene_id: str, start_pose: Pose, seed: int) -> None:
        """Place the robot. ``scene_id`` is ignored: the scene is the true grid."""
        self._episode_id, self._seed = episode_id, seed
        self._pose, self._t, self._v, self._w = start_pose, 0.0, 0.0, 0.0
        self._sequence, self._paused, self._closed = 0, False, False
        self.collisions, self.executed_distance_m = 0, 0.0

    def verify_spawn_clearance(self) -> Tuple[GridCell, ...]:
        """Cells under the footprint at the start pose, if all are truly free.

        Rejects initialization (ValueError) when clearance cannot be verified,
        as the contract requires.
        """
        cells = tuple(cells_overlapping_footprint(self._grid, self._robot.footprint_base,
                                                  self._pose, self._robot.clearance_margin_m))
        if not cells or any(cell_value(self._grid, r, c) != FREE for r, c in cells):
            raise ValueError("spawn footprint clearance cannot be verified")
        return tuple(GridCell(r, c) for r, c in cells)

    def capture(self) -> RGBDObservation:
        """Placeholder RGB-D frame stamped with the current sequence, time and pose."""
        self._require_open()
        self._sequence += 1
        pose = self._pose
        c, s = cos(pose.yaw_rad), sin(pose.yaw_rad)
        matrix = ((c, -s, 0.0, pose.x_m), (s, c, 0.0, pose.y_m),
                  (0.0, 0.0, 1.0, pose.z_m), (0.0, 0.0, 0.0, 1.0))
        blank = ImageBuffer(1, 1, "mock", b"")
        return RGBDObservation(self._episode_id, self._sequence, self._t, blank, blank,
                               CameraIntrinsics(1, 1, 1.0, 1.0, 0.5, 0.5), Transform(matrix), pose)

    def current_pose(self) -> Pose:
        return self._pose

    def step(self) -> None:
        """Advance one time step. The runner owns stepping; stepping while paused is a bug."""
        self._require_open()
        if self._paused:
            raise RuntimeError("step() called while paused")
        p = self._pose
        yaw = p.yaw_rad + self._w * self._dt
        mid = p.yaw_rad + 0.5 * self._w * self._dt
        moved = Pose(p.x_m + self._v * cos(mid) * self._dt,
                     p.y_m + self._v * sin(mid) * self._dt, p.z_m, yaw)
        if not footprint_collides(self._grid, self._robot.footprint_base, moved, 0.0):
            self.executed_distance_m += hypot(moved.x_m - p.x_m, moved.y_m - p.y_m)
            self._pose = moved
        else:
            self.collisions += 1
            turned = Pose(p.x_m, p.y_m, p.z_m, yaw)  # translation blocked; allow turning
            if not footprint_collides(self._grid, self._robot.footprint_base, turned, 0.0):
                self._pose = turned
        self._t += self._dt

    def stop(self) -> None:
        self._v = self._w = 0.0

    def pause(self) -> None:
        self._paused = True

    def resume(self) -> None:
        self._paused = False

    def close(self) -> None:
        self.stop()
        self._closed = True

    # --- ControlInterface (B's local seam, navigation/navigator.py) ---------

    def sim_time_s(self) -> float:
        return self._t

    def set_velocity(self, linear_mps: float, angular_radps: float) -> None:
        r = self._robot
        self._v = max(-r.max_linear_speed_mps, min(r.max_linear_speed_mps, linear_mps))
        self._w = max(-r.max_angular_speed_radps, min(r.max_angular_speed_radps, angular_radps))

    def _require_open(self) -> None:
        if self._closed:
            raise RuntimeError("simulator is closed")
