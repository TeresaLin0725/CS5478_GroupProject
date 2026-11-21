"""Check synthetic capture encoding and its analytically known wall geometry."""

from dataclasses import FrozenInstanceError
from struct import unpack
import unittest

from tests.rgbd_support import load_wall_observation
from frontier_search.immutable import require_immutable


class RGBDFixtureTests(unittest.TestCase):
    def test_aligned_buffers_and_known_depth(self):
        observation = load_wall_observation()
        rgb, depth = observation.color_image, observation.depth_image
        camera = observation.camera_intrinsics
        self.assertEqual((rgb.width, rgb.height), (5, 3))
        self.assertEqual((depth.width, depth.height), (5, 3))
        self.assertEqual((camera.width, camera.height), (5, 3))
        self.assertEqual(rgb.encoding, "rgb8")
        self.assertEqual(depth.encoding, "depth32f_le")
        self.assertEqual(rgb.data, bytes((128, 128, 128)) * 15)
        self.assertEqual(len(depth.data), 60)
        self.assertEqual(unpack("<15f", depth.data), (2.0,) * 15)
        self.assertEqual((camera.fx, camera.fy, camera.cx, camera.cy), (2, 2, 2, 1))
        self.assertEqual(camera.distortion, ())

    def test_camera_mount_and_wall_reference_points(self):
        observation = load_wall_observation()
        matrix = observation.camera_to_world.matrix
        self.assertEqual(tuple(row[3] for row in matrix), (1, 2, 1, 1))
        self.assertEqual(observation.robot_pose.z_m, 0)
        self.assertEqual(observation.robot_pose.yaw_rad, 0)
        # Analytically derived camera points check the fixture's mounting transform.
        for camera_point, expected_world in (
            ((0, 0, 2, 1), (3, 2, 1, 1)),
            ((-2, -1, 2, 1), (3, 4, 2, 1)),
            ((2, 1, 2, 1), (3, 0, 0, 1)),
        ):
            with self.subTest(camera_point=camera_point):
                world = tuple(sum(a * b for a, b in zip(row, camera_point)) for row in matrix)
                self.assertEqual(world, expected_world)

    def test_capture_is_deterministic_and_immutable(self):
        observation = load_wall_observation()
        self.assertEqual(observation, load_wall_observation())
        self.assertEqual(observation.episode_id, "synthetic-wall")
        self.assertEqual(observation.sequence, 1)
        self.assertEqual(observation.timestamp_s, 0)
        require_immutable(observation)
        with self.assertRaises(FrozenInstanceError):
            observation.sequence = 2


if __name__ == "__main__":
    unittest.main()
