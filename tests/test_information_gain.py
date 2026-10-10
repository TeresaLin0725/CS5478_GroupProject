"""Gain depends only on observed geometry, calibration and the fixed sweep."""

from dataclasses import replace
from math import pi
import unittest

from frontier_search.config import MappingConfig, ObservationConfig
from frontier_search.contracts import Pose
from frontier_search.perception.gain import _ray_cells, estimate_gain
from test_frontiers import grid
from test_mapper_update import scene_frame


class InformationGainTests(unittest.TestCase):
    def setUp(self):
        self.grid = grid([[0 if c < 8 else -1 for c in range(16)] for _ in range(16)], 1.)
        self.cfg = MappingConfig(gain_range_m=5., depth_max_m=6., gain_ray_step_rad=.05)
        self.pose = Pose(5.5, 8.5, 0., 0.)
        self.frame = scene_frame()
        self.routine = ObservationConfig(arrival_headings_rad=(0.,))

    def gain(self, **changes):
        return estimate_gain(changes.get('grid', self.grid), changes.get('pose', self.pose),
                             changes.get('frame', self.frame), changes.get('cfg', self.cfg),
                             changes.get('routine', self.routine))

    def test_heading_range_and_fov_change_predicted_area(self):
        front = self.gain()
        self.assertGreater(front, 0.)
        self.assertEqual(self.gain(pose=replace(self.pose, yaw_rad=pi)), 0.)
        self.assertEqual(self.gain(cfg=replace(self.cfg, gain_range_m=1.)), 0.)
        narrow = replace(self.frame, camera_intrinsics=replace(self.frame.camera_intrinsics, fx=1000.))
        self.assertLess(self.gain(frame=narrow), front)
        self.assertLessEqual(front, sum(v == -1 for row in self.grid.data for v in row))

    def test_known_wall_occludes_and_unknown_does_not(self):
        data = [list(row) for row in self.grid.data]
        for row in data:
            row[7] = 100
        self.assertEqual(self.gain(grid=grid(data, 1.)), 0.)
        # Geometry hidden behind the same wall cannot change predicted gain.
        for row in data:
            row[8:] = [100] * 8
        self.assertEqual(self.gain(grid=grid(data, 1.)), 0.)
        self.assertGreater(self.gain(), 0.)

    def test_sweep_counts_unknown_cells_once_and_includes_turns(self):
        initial = self.gain()
        self.assertEqual(self.gain(routine=replace(self.routine, arrival_headings_rad=(0., 0., 0.))), initial)
        sweep = replace(self.routine, arrival_headings_rad=(-.5, .5, 0.))
        self.assertGreaterEqual(self.gain(routine=sweep), initial)
        # Intermediate headings of a large turn see the right-hand unknown area,
        # even though the base starts pointing away and finishes pointing left.
        reverse = replace(self.pose, yaw_rad=pi)
        turn = replace(self.routine, arrival_headings_rad=(pi,))
        self.assertGreater(self.gain(pose=reverse, routine=turn), 0.)

    def test_camera_mount_offset_affects_range_from_approach_pose(self):
        shifted = scene_frame(x=1.)
        shifted = replace(shifted, robot_pose=self.frame.robot_pose)
        self.assertGreater(self.gain(frame=shifted), self.gain())

    def test_all_known_grid_has_zero_gain(self):
        self.assertEqual(self.gain(grid=grid([[0] * 16 for _ in range(16)], 1.)), 0.)

    def test_corner_side_cells_block_before_diagonal_cell(self):
        cells = list(_ray_cells(1.5, 1.5, pi / 4, 2., self.grid))
        self.assertEqual(cells[:4], [(1, 1), (1, 2), (2, 1), (2, 2)])

    def test_gain_uses_square_metres(self):
        result = self.gain()
        scaled = replace(self.grid, resolution_m=.5)
        pose = replace(self.pose, x_m=self.pose.x_m / 2, y_m=self.pose.y_m / 2)
        cfg = replace(self.cfg, gain_range_m=self.cfg.gain_range_m / 2)
        self.assertAlmostEqual(self.gain(grid=scaled, pose=pose, cfg=cfg), result / 4)


if __name__ == '__main__':
    unittest.main()
