"""Collision fast path must equal the original full cell-overlap calculation."""
from math import pi
from random import Random
import unittest

from frontier_search.contracts import OccupancyGrid, Point2, Pose
from frontier_search.navigation.geometry import cells_overlapping_footprint, cell_value, footprint_collides
from frontier_search.navigation.locobot_robot import round_footprint


class CollisionEquivalenceTests(unittest.TestCase):
    def test_matches_full_overlap_for_rotations_margins_edges_and_unknown(self):
        rng = Random(20261101)
        shapes = (round_footprint(), tuple(Point2(x,y) for x,y in
                  ((-.3,-.1),(.3,-.1),(.3,.1),(-.3,.1))))
        for fill in ('free', 'mixed', 'occupied'):
            data = tuple(tuple(0 if fill == 'free' else 100 if fill == 'occupied'
                               else rng.choice((0,0,0,0,-1,100)) for _ in range(18)) for _ in range(16))
            grid = OccupancyGrid('world', 1, .1, -.8, -.7, 18, 16, data)
            for footprint in shapes:
                poses = [Pose(rng.uniform(-1.,1.2),rng.uniform(-.9,1.1),0.,rng.uniform(-pi,pi))
                         for _ in range(120)]
                # Include exact cell-edge and margin contacts, not just random poses.
                poses += [Pose(.2,.2,0.,yaw) for yaw in (0.,pi/2,pi/4)]
                for pose in poses:
                    for margin in (0.,.01,.05):
                        expected = any(cell_value(grid,r,c) != 0 for r,c in
                            cells_overlapping_footprint(grid,footprint,pose,margin))
                        self.assertEqual(footprint_collides(grid,footprint,pose,margin),expected)


if __name__ == '__main__':unittest.main()
