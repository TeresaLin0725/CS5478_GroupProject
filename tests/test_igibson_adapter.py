"""B's real-adapter LOGIC against a fake backend (no iGibson/PyBullet; NumPy only).

This does not test the iGibson/PyBullet calls in ``IGibsonBackend``; those need the team's machine.
"""

from dataclasses import replace
from math import cos, sin
import unittest

try:
    import numpy as np
except ImportError:
    np = None

from frontier_search.config import NavigationConfig, ObservationConfig
from frontier_search.contracts import (NavigationRequest, NavigationStatus, NavigatorInit, PolicyName,
                                       Pose, Subgoal)
from frontier_search.fixtures.clearance import validate_clearance
from frontier_search.fixtures.w2_episode import mock_manifest
from frontier_search.navigation.dev_robot import PLACEHOLDER_ROBOT
from frontier_search.navigation.navigator import TeamNavigator
from frontier_search.simulation.igibson_adapter import AdapterConfig, IGibsonAdapter, IGibsonBackendProtocol
from frontier_search.simulation.start_area_verifier import CallbackSceneProbe

START = Pose(1.05, 1.55, 0.0, 0.0)
DT = 1.0 / 30.0


class FakeBackend(IGibsonBackendProtocol):
    """Unicycle physics with boxes (x0, y0, x1, y1, z0, z1) and floor holes (x0, y0, x1, y1)."""
    snapshot_id = "fake-snapshot"
    cw_positive = True     # like Locobot in C's measurement: positive command turns clockwise
    boxes = ()
    holes = ()
    log = []

    def __init__(self, config):
        self.config, self.closed = config, False
        self.v = self.w = 0.0

    def open(self, scene_id, seed, start_pose):
        self.scene, self.seed = scene_id, seed
        self._pose = Pose(start_pose.x_m, start_pose.y_m, start_pose.z_m, start_pose.yaw_rad)

    def pose(self):
        return self._pose

    def apply_action(self, linear, angular_command):
        self.v, self.w = linear, angular_command
        FakeBackend.log.append((linear, angular_command))

    def step(self):
        p = self._pose
        rate = -self.w if self.cw_positive else self.w
        yaw = p.yaw_rad + rate * DT
        mid = p.yaw_rad + 0.5 * rate * DT
        self._pose = Pose(p.x_m + self.v * cos(mid) * DT, p.y_m + self.v * sin(mid) * DT,
                          p.z_m, yaw)

    def render(self):
        rgb = np.full((2, 3, 4), 0.5, dtype=np.float32)
        xyz = np.array([[[-1, 0.5, -2], [0, 0.5, -2], [1, 0.5, -2]],
                        [[-1, -0.5, -2], [0, -0.5, -2], [1, -0.5, -2]]], dtype=np.float32)
        return rgb, xyz, np.array([[2., 0, 1.5], [0, 2., 1.], [0, 0, 1.]]), np.eye(4)

    def in_contact(self, base_z, min_height_m, min_force_n):
        p = self._pose
        return any(x0 <= p.x_m <= x1 and y0 <= p.y_m <= y1 for x0, y0, x1, y1, z0, z1 in self.boxes)

    def scene_probe(self, mapping):
        def blocked(x0, y0, x1, y1, z0, z1):
            return any(bx0 < x1 and bx1 > x0 and by0 < y1 and by1 > y0 and bz0 < z1 and bz1 > z0
                       for bx0, by0, bx1, by1, bz0, bz1 in self.boxes)

        def heights(points):
            return [None if any(x0 <= x <= x1 and y0 <= y <= y1 for x0, y0, x1, y1 in self.holes) else 0.0
                    for x, y in points]
        return CallbackSceneProbe(blocked, heights, snapshot_id=self.snapshot_id, description="fake")

    def close(self):
        self.closed = True



@unittest.skipIf(np is None, "capture conversion requires NumPy")
class IGibsonAdapterTests(unittest.TestCase):
    backend_cls = FakeBackend

    def make(self, **backend_attrs):
        cls = type("B", (FakeBackend,), backend_attrs)
        self.backends = []

        def factory(config):
            backend = cls(config)
            self.backends.append(backend)
            return backend
        manifest = mock_manifest()
        adapter = IGibsonAdapter(PLACEHOLDER_ROBOT, manifest.mapping, AdapterConfig(), factory)
        self.addCleanup(adapter.close)
        return adapter

    def reset(self, adapter, pose=START):
        adapter.reset("ep", "Rs_int", pose, 7)

    def test_reset_zeroes_clock_and_capture_is_read_only(self):
        adapter = self.make()
        self.reset(adapter)
        self.assertEqual(adapter.sim_time_s(), 0.0)
        first, second = adapter.capture(), adapter.capture()
        self.assertEqual((first.sequence, second.sequence), (1, 2))
        self.assertEqual(adapter.sim_time_s(), 0.0)           # capturing never advances time
        self.assertEqual(first.robot_pose, adapter.settled_pose)
        adapter.step()
        self.assertAlmostEqual(adapter.sim_time_s(), DT)
        self.assertAlmostEqual(adapter.capture().timestamp_s, DT)

    def test_pause_blocks_step_but_not_capture_and_resume_restores(self):
        adapter = self.make()
        self.reset(adapter)
        adapter.pause()
        with self.assertRaises(RuntimeError):
            adapter.step()
        adapter.capture()
        adapter.resume()
        adapter.step()

    def test_velocity_is_clamped_and_yaw_is_counter_clockwise_positive(self):
        adapter = self.make()                                 # fake turns CW for positive commands
        self.reset(adapter)
        adapter.set_velocity(10.0, 10.0)
        adapter.step()
        self.assertEqual(self.backends[-1].v, PLACEHOLDER_ROBOT.max_linear_speed_mps)
        self.assertEqual(self.backends[-1].w, -PLACEHOLDER_ROBOT.max_angular_speed_radps)  # sign applied
        before = adapter.current_pose().yaw_rad
        for _ in range(10):
            adapter.step()
        self.assertGreater(adapter.current_pose().yaw_rad, before)   # +w => CCW in world

    def test_calibrate_sign_measures_both_conventions(self):
        for cw, expected in ((True, -1.0), (False, 1.0)):
            adapter = self.make(cw_positive=cw)
            adapter.reset("ep", "Rs_int", START, 1)
            self.assertEqual(adapter.calibrate_angular_sign(), expected)
            self.reset(adapter)
            adapter.set_velocity(0.0, 0.4)
            for _ in range(15):
                adapter.step()
            self.assertGreater(adapter.current_pose().yaw_rad, 0.0)

    def test_reset_rejects_robot_that_did_not_settle_at_requested_pose(self):
        class Drift(FakeBackend):
            def step(self):
                super().step()
                p = self._pose
                self._pose = Pose(p.x_m + 0.01, p.y_m, p.z_m, p.yaw_rad)   # slides while settling
        adapter = IGibsonAdapter(PLACEHOLDER_ROBOT, mock_manifest().mapping, AdapterConfig(), Drift)
        with self.assertRaisesRegex(ValueError, "did not settle"):
            adapter.reset("ep", "Rs_int", START, 0)

    def test_spawn_clearance_returns_only_footprint_cells_or_rejects(self):
        adapter = self.make()
        self.reset(adapter)
        cells = adapter.verify_spawn_clearance()
        from frontier_search.fixtures.clearance import footprint_cells
        self.assertEqual(set(cells), set(footprint_cells(mock_manifest())))
        blocked = self.make(boxes=((1.0, 1.5, 1.1, 1.6, 0.0, 0.5),))      # a box under the robot
        self.reset(blocked)
        with self.assertRaisesRegex(ValueError, "cannot be verified"):
            blocked.verify_spawn_clearance()

    def test_spawn_clearance_ignores_objects_above_or_below_the_robot_height_band(self):
        high = self.make(boxes=((1.0, 1.5, 1.1, 1.6, 1.2, 2.0),))          # above the 1.0 m collision top
        self.reset(high)
        self.assertTrue(high.verify_spawn_clearance())

    def test_spawn_clearance_rejects_an_object_in_the_upper_robot_height_band(self):
        # Overhead bar at 0.6-0.9 m: above the floor band but inside the 1.0 m collision height.
        bar = self.make(boxes=((1.0, 1.5, 1.1, 1.6, 0.6, 0.9),))
        self.reset(bar)
        with self.assertRaisesRegex(ValueError, "cannot be verified"):
            bar.verify_spawn_clearance()

    def test_verified_start_area_through_the_adapter_is_accepted_by_a_guards(self):
        adapter = self.make(holes=((1.45, 1.35, 1.55, 1.45),))             # a hole in the floor
        self.reset(adapter)
        manifest = replace(mock_manifest(), initial_clearance_mode="verified_start_area",
                           initial_clearance_radius_m=1.0)
        result = adapter.verify_start_area(manifest, "adapter-test")
        validate_clearance(manifest, result.cells, result.verification)
        self.assertEqual(result.verification["scene_snapshot_id"], "fake-snapshot")
        self.assertIn(("floor not supported/traversable"), [reason for _, reason in result.rejected])

    def test_contact_steps_and_distance_are_counted(self):
        adapter = self.make(boxes=((1.20, 1.4, 1.5, 1.7, 0.0, 0.5),))
        self.reset(adapter)
        adapter.set_velocity(0.5, 0.0)
        for _ in range(60):
            adapter.step()
        self.assertGreater(adapter.collisions, 0)
        self.assertGreater(adapter.executed_distance_m, 0.5)

    def test_close_zeroes_motion_and_blocks_further_use(self):
        adapter = self.make()
        self.reset(adapter)
        adapter.set_velocity(0.3, 0.0)
        adapter.close()
        self.assertTrue(self.backends[-1].closed)
        self.assertEqual(self.backends[-1].v, 0.0)
        with self.assertRaises(RuntimeError):
            adapter.step()

    def test_team_navigator_drives_the_real_adapter_class_to_a_goal(self):
        adapter = self.make()
        self.reset(adapter)
        navigator = TeamNavigator(adapter)
        navigator.reset(NavigatorInit("ep", PLACEHOLDER_ROBOT, ObservationConfig(), NavigationConfig()))
        from frontier_search.fixtures.loader import load_navigation_fixture
        fixture = load_navigation_fixture("open_space")
        goal = fixture.goal_pose
        feedback = navigator.start(NavigationRequest("ep", "g", fixture.grid, adapter.current_pose(),
                                                     Subgoal("F1", goal, fixture.grid.revision, PolicyName.NEAREST)))
        for _ in range(4000):
            if feedback.status in (NavigationStatus.REACHED, NavigationStatus.FAILED, NavigationStatus.CANCELLED):
                break
            adapter.step()
            navigator.on_step()
            feedback = navigator.poll("ep", "g")
        self.assertEqual(feedback.status, NavigationStatus.REACHED)


if __name__ == "__main__":
    unittest.main()
