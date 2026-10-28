"""Sampling and batch transforms must preserve the old projector's exact output."""

from dataclasses import replace
from random import Random
from struct import pack
import unittest
from unittest.mock import patch

from frontier_search.fixtures.analytic_rgbd import render_frame
from frontier_search.perception import projection
from frontier_search.validation import ContractError
from reference_projection import iter_depth_world as reference_project


class ProjectionEquivalenceTests(unittest.TestCase):
    def assert_points_identical(self, expected, actual):
        self.assertEqual(len(expected), len(actual))
        for old, new in zip(expected, actual):
            self.assertEqual(old[:2], new[:2])
            self.assertTrue(all(type(value) is int for value in new[:2]))
            self.assertTrue(all(type(value) is float for value in new[2:]))
            self.assertEqual([value.hex() for value in old[2:]], [value.hex() for value in new[2:]])

    def test_exact_coordinates_for_numpy_and_scalar_with_calibration_changes(self):
        rng = Random(5478)
        for width, height in ((1, 1), (5, 3), (37, 29), (17, 267)):
            frame = render_frame(width=width, height=height, x=.123, y=-.345, yaw=.137, pitch_down=.281)
            for calibration in (frame.camera_intrinsics,
                                replace(frame.camera_intrinsics, fx=13.123, fy=27.891, cx=-.25, cy=.33)):
                values = [rng.choice((0., -0., -1., 1e-45, 1.17549435e-38, 3.4e38,
                                      float('nan'), float('inf'), float('-inf'),
                                      rng.uniform(.01, 1000.))) for _ in range(width * height)]
                frame = replace(frame, camera_intrinsics=calibration,
                                depth_image=replace(frame.depth_image, data=pack('<%df' % len(values), *values)))
                for stride in (1, 2, 4, 9, 1000):
                    expected = list(reference_project(frame, stride))
                    with self.subTest(width=width, height=height, stride=stride):
                        self.assert_points_identical(expected, list(projection.iter_depth_world(frame, stride)))
                        with patch.object(projection, 'np', None):
                            self.assert_points_identical(expected, list(projection.iter_depth_world(frame, stride)))

    def test_stride_is_validated_before_sampling(self):
        frame = render_frame(width=3, height=3)
        for stride in (0, -1, True, 1.5):
            with self.subTest(stride=stride), self.assertRaisesRegex(ContractError, 'stride'):
                list(projection.iter_depth_world(frame, stride))

    def test_overflow_raises_after_the_same_finite_prefix(self):
        frame = render_frame(width=5, height=1)
        frame = replace(frame, camera_intrinsics=replace(frame.camera_intrinsics, fx=1e-300, cx=0.),
                        depth_image=replace(frame.depth_image, data=pack('<5f', 1., 3e38, 1., 1., 1.)))
        reference = reference_project(frame)
        first = next(reference)
        with self.assertRaisesRegex(ContractError, 'overflowed'):
            next(reference)
        for fallback in (False, True):
            with self.subTest(fallback=fallback), patch.object(projection, 'np', None if fallback else projection.np):
                actual = projection.iter_depth_world(frame)
                self.assert_points_identical([first], [next(actual)])
                with self.assertRaisesRegex(ContractError, 'overflowed'):
                    next(actual)

    @unittest.skipIf(projection.np is None, 'NumPy acceleration is optional')
    def test_signaling_nan_is_ignored_without_changing_numpy_error_policy(self):
        frame = render_frame(width=5, height=3)
        bits = (0x7f800001, 0x7fc00001, 0x7f800000, 0xff800000, 0x40000000) * 3
        frame = replace(frame, depth_image=replace(frame.depth_image,
                        data=b''.join(pack('<I', value) for value in bits)))
        with projection.np.errstate(all='raise'):
            policy = projection.np.geterr()
            for stride in (1, 2, 4):
                self.assert_points_identical(list(reference_project(frame, stride)),
                                             list(projection.iter_depth_world(frame, stride)))
            self.assertEqual(projection.np.geterr(), policy)

    @unittest.skipIf(projection.np is None, 'NumPy acceleration is optional')
    def test_cached_axes_are_bounded_and_read_only(self):
        projection._numpy_sample_axes.cache_clear()
        for index in range(12):
            frame = render_frame(width=3, height=3)
            camera = replace(frame.camera_intrinsics, cx=float(index))
            for axis in projection._numpy_sample_axes(camera, 2):
                self.assertFalse(axis.flags.writeable)
        self.assertEqual(projection._numpy_sample_axes.cache_info().currsize, 8)
