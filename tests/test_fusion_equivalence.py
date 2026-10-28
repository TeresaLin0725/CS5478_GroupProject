"""Differential checks against the frozen pre-optimization fusion algorithm."""

from copy import deepcopy
from dataclasses import replace
from random import Random
from struct import pack
import unittest

from frontier_search.config import MappingConfig
from frontier_search.fixtures.analytic_rgbd import render_frame
from frontier_search.perception.fusion import _ray_voxels, fuse_observation
from reference_fusion import _ray_voxels as reference_ray, fuse_observation as reference_fuse


class FusionEquivalenceTests(unittest.TestCase):
    def test_random_and_boundary_rays_match_reference_in_order(self):
        rng = Random(5478)
        for index in range(5000):
            bounds = rng.choice(((3., 4., 2.4), (7., 11., 19.99999999999), (2., 3., .02)))
            def coordinate(upper):
                if rng.random() < .6:
                    return rng.choice((0., upper, 1., -1.)) + rng.choice((0., 1e-13, -1e-13, 1e-10, -1e-10))
                return rng.uniform(-2., upper + 2.)
            start = tuple(coordinate(b) for b in bounds)
            end = start if index % 17 == 0 else tuple(coordinate(b) for b in bounds)
            if index % 7 == 0:
                end = (start[0], end[1], end[2])
            with self.subTest(index=index, start=start, end=end, bounds=bounds):
                self.assertEqual(list(_ray_voxels(start, end, bounds)),
                                 list(reference_ray(start, end, bounds)))

    def test_multiframe_fusion_matches_all_reference_buffers(self):
        box = (((1.1, -.4, 0.), (1.5, .4, 1.2)),)
        frames = [render_frame(sequence=i + 1, width=31, height=23,
                               x=.1 if i == 5 else 0., yaw=.2 if i == 5 else 0.,
                               boxes=box if i in (0, 3) else ()) for i in range(6)]
        # Invalid depth and an unobserved frame must not falsely clear obstacles.
        invalid = replace(frames[-1].depth_image,
                          data=pack('<%df' % (31 * 23), *([float('nan')] * (31 * 23))))
        frames.append(replace(frames[-1], sequence=7, depth_image=invalid))
        for dynamic in (0, 2):
            for stride in (1, 4):
                cfg = MappingConfig(resolution_m=.2, width_cells=25, height_cells=20,
                                    origin_x_m=-.5, origin_y_m=-2., depth_pixel_stride=stride,
                                    dynamic_clear_frames=dynamic)
                buffers = [[[-1] * 25 for _ in range(20)], [[False] * 25 for _ in range(20)],
                           [[0] * 25 for _ in range(20)], [[False] * 25 for _ in range(20)]]
                counts = [[0] * 25 for _ in range(20)] if dynamic else None
                expected, expected_counts = deepcopy(buffers), deepcopy(counts)
                for index, frame in enumerate(frames):
                    kwargs = dict(increment_clear_count=index != 2)
                    fuse_observation(frame, cfg, (.04, .923), *buffers, clear_counts=counts, **kwargs)
                    reference_fuse(frame, cfg, (.04, .923), *expected,
                                   clear_counts=expected_counts, **kwargs)
                    with self.subTest(dynamic=dynamic, stride=stride, frame=index):
                        self.assertEqual(buffers, expected)
                        self.assertEqual(counts, expected_counts)

    def test_masks_above_machine_word_size_remain_exact(self):
        frame = render_frame(width=9, height=7)
        cfg = MappingConfig(resolution_m=.2, width_cells=20, height_cells=10,
                            origin_y_m=-1., clearance_bin_height_m=.001, dynamic_clear_frames=2)
        buffers = [[[-1] * 20 for _ in range(10)], [[False] * 20 for _ in range(10)],
                   [[0] * 20 for _ in range(10)], [[False] * 20 for _ in range(10)]]
        counts = [[0] * 20 for _ in range(10)]
        expected, expected_counts = deepcopy(buffers), deepcopy(counts)
        fuse_observation(frame, cfg, (.04, .923), *buffers, clear_counts=counts)
        reference_fuse(frame, cfg, (.04, .923), *expected, clear_counts=expected_counts)
        self.assertTrue(any(mask.bit_length() > 64 for row in buffers[2] for mask in row))
        self.assertEqual(buffers, expected)
        self.assertEqual(counts, expected_counts)
