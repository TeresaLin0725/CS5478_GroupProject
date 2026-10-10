"""Known ray intersections for the mapping evidence traversal."""

import unittest

from frontier_search.perception.fusion import _ray_voxels


class MappingRayTests(unittest.TestCase):
    def test_entry_from_outside_and_reverse(self):
        cells = list(_ray_voxels((-1., 0.5, 0.5), (4., 0.5, 0.5), (3, 2, 2)))
        self.assertEqual(cells, [(0, 0, 0), (1, 0, 0), (2, 0, 0)])
        self.assertEqual(list(_ray_voxels((4., 0.5, 0.5), (-1., 0.5, 0.5), (3, 2, 2))),
                         list(reversed(cells)))

    def test_diagonal_corner_touches_are_not_evidence(self):
        self.assertEqual(list(_ray_voxels((0., 0., 0.), (3., 3., 3.), (3, 3, 3))),
                         [(0, 0, 0), (1, 1, 1), (2, 2, 2)])

    def test_truncated_top_bin_and_negative_direction_at_boundary(self):
        self.assertEqual(list(_ray_voxels((0.5, 0.5, 4.), (0.5, 0.5, -1.), (1, 1, 2.4))),
                         [(0, 0, 2), (0, 0, 1), (0, 0, 0)])
        self.assertEqual(list(_ray_voxels((2., 0.5, 0.5), (1., 0.5, 0.5), (3, 1, 1))),
                         [(1, 0, 0)])

    def test_missing_map_and_zero_length_rays(self):
        self.assertEqual(list(_ray_voxels((-1., -1., 0.), (-2., -2., 0.), (2, 2, 2))), [])
        self.assertEqual(list(_ray_voxels((1., 1., 1.), (1., 1., 1.), (2, 2, 2))), [])
        self.assertEqual(list(_ray_voxels((0., 2., 0.), (1., 2., 1.), (2, 2, 2))), [])


if __name__ == '__main__':
    unittest.main()
