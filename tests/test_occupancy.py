"""Check grid boundaries and conservative robot-height obstacle marking."""

from dataclasses import replace
import unittest

from frontier_search.config import MappingConfig
from frontier_search.contracts import GridCell, OccupancyGrid, Point2, Point3, Pose, RobotConfig
from frontier_search.perception.occupancy import mark_obstacles, world_to_grid
from frontier_search.perception.projection import ProjectedDepthPoint
from frontier_search.validation import ContractError


def evidence(x, y, z, depth=2.):
    return ProjectedDepthPoint(0, 0, depth, Point3(x, y, z))


class OccupancyTests(unittest.TestCase):
    def setUp(self):
        self.cfg = MappingConfig(resolution_m=0.5, width_cells=4, height_cells=3,
                                 origin_x_m=-1., origin_y_m=-0.5)
        self.grid = OccupancyGrid("world", 0, 0.5, -1., -0.5, 4, 3,
                                  ((-1, -1, -1, -1), (-1, 0, -1, -1), (-1, -1, -1, 100)))
        self.pose = Pose(0., 0., 0.1, 0.)
        self.robot = RobotConfig(
            (Point2(-0.1, -0.1), Point2(0.1, -0.1), Point2(0.1, 0.1), Point2(-0.1, 0.1)),
            0., 0.4, 0.02, "differential_drive", True, 0.3, 1., 0.05, 0.1)

    def mark(self, *points):
        return mark_obstacles(self.grid, points, self.pose, self.robot, self.cfg)

    def test_world_axes_centres_and_half_open_bounds(self):
        for x, y, expected in (
            (-1., -0.5, GridCell(0, 0)), (-0.75, -0.25, GridCell(0, 0)),
            (-0.5, 0., GridCell(1, 1)), (0.999, 0.999, GridCell(2, 3)),
            (-1.001, 0., None), (0., -0.501, None), (1., 0., None), (0., 1., None),
        ):
            with self.subTest(x=x, y=y):
                self.assertEqual(world_to_grid(self.grid, x, y), expected)

    def test_only_obstacle_endpoints_marked_without_inflation_or_clearing(self):
        result = self.mark(evidence(-0.75, -0.25, 0.2))
        self.assertEqual(result.data, ((100, -1, -1, -1), (-1, 0, -1, -1), (-1, -1, -1, 100)))
        self.assertEqual(result.revision, 1)
        self.assertEqual(self.grid.data[0][0], -1)
        self.assertEqual(self.grid.revision, 0)

    def test_floor_out_of_bounds_and_high_surfaces_remain_unknown(self):
        result = self.mark(evidence(-0.75, -0.25, 0.), evidence(-0.25, -0.25, 0.8),
                           evidence(10., 10., 0.3), evidence(-0.75, 0.25, 0.05))
        self.assertEqual(result.data, self.grid.data)

    def test_height_is_relative_to_robot_base_and_bounds_are_inclusive(self):
        result = self.mark(evidence(-0.75, -0.25, 0.1), evidence(-0.25, -0.25, 0.5),
                           evidence(0.25, -0.25, 0.501))
        self.assertEqual(result.data[0], (100, 100, -1, -1))

    def test_configured_floor_band(self):
        cfg = replace(self.cfg, floor_height_m=1., floor_tolerance_m=0.05)
        pose = replace(self.pose, z_m=1.)
        result = mark_obstacles(self.grid, (evidence(-0.75, -0.25, 1.),
                                           evidence(-0.25, -0.25, 1.02),
                                           evidence(0.25, -0.25, 1.2)), pose, self.robot, cfg)
        self.assertEqual(result.data[0], (-1, -1, 100, -1))

    def test_depth_range_and_duplicate_evidence(self):
        result = self.mark(evidence(-0.75, -0.25, 0.2, 0.09), evidence(-0.25, -0.25, 0.2, 5.1),
                           evidence(0.25, -0.25, 0.2, 0.1), evidence(0.75, -0.25, 0.2, 5.0),
                           evidence(0.25, -0.25, 0.2, 0.1))
        self.assertEqual(result.data[0], (-1, -1, 100, 100))

    def test_obstacle_overrides_spawn_clearance_and_empty_evidence_preserves_map(self):
        result = self.mark(evidence(-0.25, 0.25, 0.3))
        self.assertEqual(result.data[1][1], 100)
        unchanged = mark_obstacles(result, (), self.pose, self.robot, self.cfg)
        self.assertEqual(unchanged.data, result.data)
        self.assertEqual(unchanged.revision, 2)

    def test_invalid_geometry_and_collision_interval(self):
        with self.assertRaises(ContractError):
            world_to_grid(self.grid, float("nan"), 0.)
        for grid in (replace(self.grid, frame_id="camera"), replace(self.grid, resolution_m=0),
                     replace(self.grid, data=((0,),)), replace(self.grid, revision=-1),
                     replace(self.grid, origin_x_m=5.)):
            with self.subTest(grid=grid):
                with self.assertRaises(ContractError):
                    mark_obstacles(grid, (), self.pose, self.robot, self.cfg)
        with self.assertRaises(ContractError):
            mark_obstacles(self.grid, (), self.pose,
                           replace(self.robot, collision_z_max_m=0.), self.cfg)

    def test_invalid_mapping_parameters_rejected(self):
        for changes in ({"floor_height_m": float("nan")}, {"floor_tolerance_m": -0.1},
                        {"depth_min_m": 0.}, {"depth_max_m": 0.05}, {"depth_max_m": float("inf")}):
            with self.subTest(changes=changes):
                with self.assertRaises(ValueError):
                    replace(self.cfg, **changes)


if __name__ == "__main__":
    unittest.main()
