"""Denoising effectiveness, boundary preservation and map-fusion integration."""
from dataclasses import replace
import unittest
from unittest.mock import patch
import numpy as np

from frontier_search.config import MappingConfig
from frontier_search.fixtures.analytic_rgbd import render_frame
from frontier_search.perception.depth_filter import filter_mapping_depth
from frontier_search.perception.fusion import fuse_observation


def observation(values):
    height, width = values.shape
    obs = render_frame(width=width, height=height)
    return replace(obs, depth_image=replace(obs.depth_image, data=values.astype('<f4').tobytes()))


def depths(obs):
    return np.frombuffer(obs.depth_image.data, dtype='<f4').reshape(obs.depth_image.height, obs.depth_image.width)


class DepthFilterTests(unittest.TestCase):
    def test_off_is_exact_identity(self):
        obs = observation(np.ones((9, 9)))
        self.assertIs(filter_mapping_depth(obs, MappingConfig()), obs)

    def test_planar_noise_reduced_without_mutation_or_metadata_changes(self):
        values = 2 + np.random.default_rng(59).normal(0, .01, (100, 100))
        obs = observation(values)
        before = obs.depth_image.data
        result = filter_mapping_depth(obs, MappingConfig(depth_filter_kernel_size=5))
        self.assertLess(np.std(depths(result)[3:-3, 3:-3] - 2), .0035)
        self.assertEqual(obs.depth_image.data, before)
        self.assertEqual(replace(result, depth_image=obs.depth_image), obs)

    def test_depth_step_and_thin_obstacle_preserved(self):
        values = np.full((20, 20), 2., dtype='<f4')
        values[:, :8] = 1.
        values[:, 14] = .5
        obs = observation(values)
        result = filter_mapping_depth(obs, MappingConfig(depth_filter_kernel_size=5))
        np.testing.assert_array_equal(depths(result), values)

    def test_invalid_centres_preserved_and_sparse_evidence_not_filled(self):
        values = np.zeros((9, 9), dtype='<f4')
        values[4] = [np.nan, np.inf, -1., .01, 2., 6., 0., 0., 0.]
        obs = observation(values)
        result = filter_mapping_depth(obs, MappingConfig(depth_filter_kernel_size=5))
        self.assertEqual(result.depth_image.data, obs.depth_image.data)

    def test_invalid_neighbours_do_not_bias_valid_measurements(self):
        values = np.full((9, 9), 2., dtype='<f4')
        values[3:5, 3:5] = 0
        values[4, 4] = 2.03
        result = filter_mapping_depth(observation(values), MappingConfig(depth_filter_kernel_size=5))
        self.assertAlmostEqual(depths(result)[4, 4], 2.)
        self.assertEqual(depths(result)[3, 3], 0.)

    def test_border_does_not_fabricate_majority_support(self):
        values = np.full((7, 7), 2., dtype='<f4')
        values[0, 0] = 2.03
        obs = observation(values)
        self.assertEqual(depths(filter_mapping_depth(obs, MappingConfig(depth_filter_kernel_size=5)))[0, 0], depths(obs)[0, 0])

    def test_mapping_fusion_consumes_filtered_depth(self):
        values = np.full((9, 9), 2., dtype='<f4')
        values[4, 4] = 2.03
        obs = observation(values)
        cfg = MappingConfig(width_cells=2, height_cells=2, depth_filter_kernel_size=5)
        with patch('frontier_search.perception.fusion.iter_depth_world', return_value=iter(())) as projection:
            fuse_observation(obs, cfg, (.1, .5), [[-1]*2 for _ in range(2)],
                [[False]*2 for _ in range(2)], [[0]*2 for _ in range(2)], [[False]*2 for _ in range(2)])
        self.assertEqual(depths(projection.call_args[0][0])[4, 4], 2.)
        self.assertNotEqual(depths(obs)[4, 4], 2.)

    def test_configuration_validation(self):
        for value in (-1, 1, 2, 7, True, 3.):
            with self.assertRaises(ValueError):
                MappingConfig(depth_filter_kernel_size=value)
        for value in (0, -.1, float('nan'), float('inf'), True):
            with self.assertRaises(ValueError):
                MappingConfig(depth_filter_max_delta_m=value)

    def test_strided_filter_matches_full_resolution_at_projected_pixels(self):
        values = 2 + np.random.default_rng(9).normal(0, .01, (139, 77))
        obs = observation(values)
        cfg = MappingConfig(depth_filter_kernel_size=5)
        full = depths(filter_mapping_depth(obs, cfg))
        sparse = depths(filter_mapping_depth(obs, replace(cfg, depth_pixel_stride=4)))
        np.testing.assert_array_equal(full[::4, ::4], sparse[::4, ::4])
        untouched = np.ones(values.shape, dtype=bool)
        untouched[::4, ::4] = False
        np.testing.assert_array_equal(sparse[untouched], depths(obs)[untouched])
