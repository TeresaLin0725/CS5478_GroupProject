"""The faster collision check must preserve complete probe outputs and ranking."""
from dataclasses import replace
from math import pi
from random import Random
from types import SimpleNamespace
import unittest

from frontier_search.contracts import OccupancyGrid,Point2,Pose
from frontier_search.navigation.geometry import footprint_collides
from frontier_search.perception.mapper import _navigation_probes
from frontier_search.runner.integrated import manifest_for
from frontier_search.navigation.locobot_robot import shared_round_robot
from tests.reference_probes import navigation_probes


class ProbeEquivalenceTests(unittest.TestCase):
    def test_full_outputs_match_reference_on_coarse_and_fine_maps(self):
        round_robot=shared_round_robot(manifest_for('probe-test').robot)
        rectangle=replace(round_robot,footprint_base=tuple(Point2(x,y) for x,y in
            ((-.25,-.12),(.25,-.12),(.25,.12),(-.25,.12))),clearance_margin_m=.03)
        rng=Random(5478)
        for resolution in (.1,.05):
            w,h=round(3/resolution),round(2.6/resolution)
            for mode in ('free','sparse','wall'):
                data=tuple(tuple(0 if mode=='free' else
                    rng.choice((-1,100)) if mode=='sparse' and rng.random()<.015 else
                    100 if mode=='wall' and c==w//3 else 0 for c in range(w)) for r in range(h))
                grid=OccupancyGrid('world',1,resolution,-1.5,-1.3,w,h,data)
                for pose in (Pose(.031,-.019,.007,.3),Pose(-1.3,-1.1,.007,pi)):
                    obs=SimpleNamespace(robot_pose=pose)
                    for robot in (round_robot,rectangle):
                        with self.subTest(resolution=resolution,mode=mode,pose=pose,robot=robot):
                            expected=navigation_probes(grid,obs,robot)
                            actual=_navigation_probes(grid,obs,robot)
                            self.assertEqual(actual,expected)
                            for frontier in actual:
                                self.assertFalse(footprint_collides(grid,robot.footprint_base,
                                    frontier.approach_pose,robot.clearance_margin_m))


if __name__=='__main__':unittest.main()
