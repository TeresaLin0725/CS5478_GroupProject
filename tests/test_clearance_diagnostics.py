"""Geometric checks for interpreting native-camera mapping failures."""

from dataclasses import replace
import unittest

from frontier_search.contracts import OccupancyGrid, Point2, Pose
from frontier_search.navigation.costmap import Costmap
from frontier_search.navigation.planner import goal_is_valid
from frontier_search.runner.diagnose_clearance import missing_bins, nearest_visible_floor
from rgbd_support import load_wall_observation


class ClearanceDiagnosticTests(unittest.TestCase):
    def test_missing_height_bins_include_partial_top_without_roundoff_bin(self):
        self.assertEqual(missing_bins(3, 0.03, 0.18, 0.05), [2])
        self.assertEqual(missing_bins(7, 0.03, 0.18000000000000002, 0.05), [])
        self.assertEqual(missing_bins(7, 0.03, 0.19, 0.05), [3])

    def test_bottom_edge_floor_distance_is_camera_relative(self):
        frame = load_wall_observation()
        self.assertAlmostEqual(nearest_visible_floor(frame, 0), 2.0)
        self.assertIsNone(nearest_visible_floor(frame, 1.0))
        horizontal_bottom = replace(frame, camera_intrinsics=replace(frame.camera_intrinsics, cy=2.0))
        self.assertIsNone(nearest_visible_floor(horizontal_bottom, 0))

    def test_exact_footprint_can_fit_while_planning_disc_does_not(self):
        data = tuple(tuple(0 if 2 <= r <= 6 and 2 <= c <= 6 else -1
                           for c in range(9)) for r in range(9))
        grid = OccupancyGrid('world', 1, 0.1, 0, 0, 9, 9, data)
        footprint = tuple(Point2(x, y) for x, y in
                          ((-.181, -.181), (.181, -.181), (.181, .181), (-.181, .181)))
        costmap = Costmap(grid, footprint, .02)
        goal = Pose(.45, .45, 0, 0)
        self.assertTrue(costmap.pose_valid(goal))
        self.assertFalse(goal_is_valid(costmap, goal))


if __name__ == '__main__':
    unittest.main()
