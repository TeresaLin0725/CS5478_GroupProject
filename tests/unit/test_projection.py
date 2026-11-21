"""Analytical geometry and malformed-input checks for depth projection."""

from dataclasses import FrozenInstanceError, replace
from struct import pack
import unittest

from frontier_search.contracts import Point3, Pose
from tests.rgbd_support import load_wall_observation
from frontier_search.immutable import require_immutable
from frontier_search.perception.projection import project_depth_to_world
from frontier_search.sensors import Transform
from frontier_search.validation import ContractError


IDENTITY = Transform(((1., 0., 0., 0.), (0., 1., 0., 0.),
                      (0., 0., 1., 0.), (0., 0., 0., 1.)))


class DepthProjectionTests(unittest.TestCase):
    def setUp(self):
        self.observation = load_wall_observation()

    def test_known_world_points_and_entire_wall_plane(self):
        points = project_depth_to_world(self.observation)
        self.assertEqual(len(points), 15)
        self.assertEqual(points[0].position_world, Point3(3, 4, 2))
        self.assertEqual(points[7].position_world, Point3(3, 2, 1))
        self.assertEqual(points[-1].position_world, Point3(3, 0, 0))
        self.assertEqual([(p.u, p.v) for p in points], [(u, v) for v in range(3) for u in range(5)])
        self.assertTrue(all(p.position_world.x_m == 3 and p.depth_m == 2 for p in points))

    def test_identity_transform_exposes_camera_coordinates(self):
        points = project_depth_to_world(replace(self.observation, camera_to_world=IDENTITY))
        self.assertEqual(points[0].position_world, Point3(-2, -1, 2))
        self.assertEqual(points[7].position_world, Point3(0, 0, 2))
        self.assertEqual(points[-1].position_world, Point3(2, 1, 2))

    def test_translation_and_quarter_turn(self):
        transform = Transform(((0., -1., 0., 10.), (1., 0., 0., 20.),
                               (0., 0., 1., 30.), (0., 0., 0., 1.)))
        points = project_depth_to_world(replace(self.observation, camera_to_world=transform))
        self.assertEqual(points[0].position_world, Point3(11, 18, 32))
        self.assertEqual(points[7].position_world, Point3(10, 20, 32))

    def test_unequal_focal_lengths_and_varying_axial_depth(self):
        camera = replace(self.observation.camera_intrinsics, fx=4., fy=2., cx=1., cy=0.)
        depth = replace(self.observation.depth_image, data=pack("<15f", *range(1, 16)))
        points = project_depth_to_world(replace(self.observation, depth_image=depth,
                                               camera_intrinsics=camera, camera_to_world=IDENTITY))
        self.assertEqual(points[0].position_world, Point3(-0.25, 0, 1))
        self.assertEqual(points[7].position_world, Point3(2, 4, 8))

    def test_invalid_pixels_skipped_without_losing_correspondence(self):
        values = [0., -1., float("nan"), float("inf"), float("-inf")] + [2.] * 10
        depth = replace(self.observation.depth_image, data=pack("<15f", *values))
        points = project_depth_to_world(replace(self.observation, depth_image=depth))
        self.assertEqual(len(points), 10)
        self.assertEqual((points[0].u, points[0].v), (0, 1))
        self.assertEqual(points[0].position_world, Point3(3, 4, 1))
        empty_depth = replace(depth, data=pack("<15f", *([0.] * 15)))
        self.assertEqual(project_depth_to_world(replace(self.observation, depth_image=empty_depth)), ())

    def test_rgb_and_robot_pose_do_not_override_capture_transform(self):
        expected = project_depth_to_world(self.observation)
        changed = replace(self.observation, robot_pose=Pose(50, 60, 70, 1.5),
                          color_image=replace(self.observation.color_image, data=b""))
        self.assertEqual(project_depth_to_world(changed), expected)

    def test_input_unchanged_and_output_immutable(self):
        points = project_depth_to_world(self.observation)
        self.assertEqual(self.observation, load_wall_observation())
        require_immutable(points)
        with self.assertRaises(FrozenInstanceError):
            points[0].u = 3

    def test_bad_depth_and_intrinsics_rejected(self):
        observation = self.observation
        depth, camera = observation.depth_image, observation.camera_intrinsics
        bad = [replace(observation, depth_image=replace(depth, encoding="mock")),
               replace(observation, depth_image=replace(depth, data=depth.data[:-1])),
               replace(observation, depth_image=replace(depth, data=depth.data + b"1234")),
               replace(observation, depth_image=replace(depth, width=0)),
               replace(observation, depth_image=replace(depth, height=3.0)),
               replace(observation, camera_intrinsics=replace(camera, width=4)),
               replace(observation, camera_intrinsics=replace(camera, fx=0)),
               replace(observation, camera_intrinsics=replace(camera, fy=-1)),
               replace(observation, camera_intrinsics=replace(camera, distortion=(0.1,)))]
        for frame in bad:
            with self.subTest(frame=frame):
                with self.assertRaises(ContractError):
                    project_depth_to_world(frame)
        zero_distortion = replace(observation, camera_intrinsics=replace(camera, distortion=(0.,) * 5))
        self.assertEqual(project_depth_to_world(zero_distortion), project_depth_to_world(observation))

    def test_nonrigid_and_malformed_transforms_rejected(self):
        matrices = [
            ((1., 0., 0.),) * 3,
            ((2., 0., 0., 0.), (0., 1., 0., 0.), (0., 0., 1., 0.), (0., 0., 0., 1.)),
            ((-1., 0., 0., 0.), (0., 1., 0., 0.), (0., 0., 1., 0.), (0., 0., 0., 1.)),
            ((1., 0., 0., 0.), (0., 1., 0., 0.), (0., 0., 1., 0.), (0., 0., 1., 1.)),
        ]
        for matrix in matrices:
            with self.subTest(matrix=matrix):
                with self.assertRaises(ContractError):
                    project_depth_to_world(replace(self.observation, camera_to_world=Transform(matrix)))

    def test_nonfinite_calibration_rejected_defensively(self):
        # Contract constructors normally reject nonfinite values already.
        for field in ("fx", "cx"):
            observation = load_wall_observation()
            object.__setattr__(observation.camera_intrinsics, field, float("nan"))
            with self.assertRaises(ContractError):
                project_depth_to_world(observation)


if __name__ == "__main__":
    unittest.main()
