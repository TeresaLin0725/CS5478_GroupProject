"""Camera-conversion checks; NumPy is optional outside the capture environment."""

import unittest

try:
    import numpy as np
except ImportError:
    np = None

from frontier_search.contracts import Pose
from frontier_search.simulation.rgbd import convert_igibson_frames


@unittest.skipIf(np is None, "renderer conversion requires optional NumPy")
class IGibsonRGBDTests(unittest.TestCase):
    def setUp(self):
        self.rgb = np.full((2, 3, 4), 0.5, dtype=np.float32)
        self.xyz = np.array([[[-1, 0.5, -2], [0, 0.5, -2], [1, 0.5, -2]],
                             [[-1, -0.5, -2], [0, -0.5, -2], [1, -0.5, -2]]], dtype=np.float32)
        self.k = np.array([[2., 0, 1.5], [0, 2., 1.], [0, 0, 1.]])

    def convert(self):
        return convert_igibson_frames(self.rgb, self.xyz, self.k, np.eye(4),
                                      Pose(4, 5, 0.1, 0), "capture", 3, 1.5)

    def test_pixel_centres_axes_and_owned_buffers(self):
        observation, error = self.convert()
        self.assertEqual(error, 0)
        self.assertEqual(observation.color_image.data, bytes([128]) * 18)
        self.assertEqual(observation.camera_intrinsics.cx, 1)
        self.assertEqual(observation.camera_intrinsics.cy, 0.5)
        self.assertEqual(observation.camera_to_world.matrix,
                         ((1., 0., 0., 0.), (0., -1., 0., 0.),
                          (0., 0., -1., 0.), (0., 0., 0., 1.)))
        self.rgb[:] = 0
        self.xyz[:] = 0
        self.assertEqual(observation.color_image.data, bytes([128]) * 18)
        self.assertEqual(np.frombuffer(observation.depth_image.data, dtype="<f4").tolist(), [2.] * 6)
        self.assertEqual((observation.sequence, observation.timestamp_s), (3, 1.5))

    def test_invalid_depth_is_zeroed(self):
        self.xyz[0, 0, 2] = float("nan")
        observation, _ = self.convert()
        self.assertEqual(np.frombuffer(observation.depth_image.data, dtype="<f4").tolist(), [0.] + [2.] * 5)

    def test_disagreeing_calibration_rejected(self):
        self.k[0, 2] += 0.1
        with self.assertRaises(RuntimeError):
            self.convert()

    def test_empty_depth_and_nonfinite_rgb_rejected(self):
        self.xyz[:, :, 2] = 0
        with self.assertRaises(RuntimeError):
            self.convert()
        self.xyz[:, :, 2] = -2
        self.rgb[0, 0, 0] = float("nan")
        with self.assertRaises(RuntimeError):
            self.convert()


if __name__ == "__main__":
    unittest.main()
