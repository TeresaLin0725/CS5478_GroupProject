"""Coverage recovery stays on sensed free space and respects prior views."""
from dataclasses import replace
from math import hypot
import unittest
from unittest.mock import patch

from frontier_search.config import MappingConfig
from frontier_search.contracts import OccupancyGrid
from frontier_search.fixtures.analytic_rgbd import render_frame
from frontier_search.navigation.geometry import footprint_collides
from frontier_search.perception.mapper import RGBDMapper, _navigation_probes
from test_mapper_update import request, scene_frame


class ViewpointTests(unittest.TestCase):
    def setUp(self):
        from frontier_search.runner.integrated import shared_round_manifest_for
        self.robot=shared_round_manifest_for('views').robot
        self.cfg=MappingConfig(unvisited_recovery_viewpoints=True)
        self.obs=render_frame(x=2.,y=2.)
        self.grid=OccupancyGrid('world',1,.1,0.,0.,50,50,tuple((0,)*50 for _ in range(50)))

    def probes(self,visited=()):
        return _navigation_probes(self.grid,self.obs,self.robot,self.cfg,visited)

    def test_deterministic_bounded_valid_and_farther_than_local_ring(self):
        probes=self.probes()
        self.assertTrue(probes)
        self.assertLessEqual(len(probes),8)
        self.assertEqual(probes,self.probes())
        visited=tuple((f.approach_pose.x_m,f.approach_pose.y_m) for f in probes)
        farther=self.probes(visited)
        self.assertTrue(any(f.distance_from_robot_m>1.05 for f in farther))
        for f in probes+farther:
            self.assertEqual(f.kind,'navigation_probe')
            self.assertGreaterEqual(f.distance_from_robot_m,.6)
            self.assertLessEqual(f.distance_from_robot_m,3.)
            self.assertFalse(footprint_collides(self.grid,self.robot.footprint_base,f.approach_pose,self.robot.clearance_margin_m))

    def test_visited_view_neighborhood_is_not_repeated(self):
        first=self.probes()
        visited=[(f.approach_pose.x_m,f.approach_pose.y_m) for f in first]
        for f in self.probes(visited):
            self.assertTrue(all(hypot(f.approach_pose.x_m-x,f.approach_pose.y_m-y)>=.6 for x,y in visited))

    def test_unknown_and_occupied_are_never_overridden(self):
        for value in (-1,100):
            grid=replace(self.grid,data=tuple((value,)*50 for _ in range(50)))
            self.assertEqual(_navigation_probes(grid,self.obs,self.robot,self.cfg,()),())

    def test_complete_view_history_exhausts_fallback(self):
        visited=[((c+.5)*.1,(r+.5)*.1) for r in range(50) for c in range(50)]
        self.assertEqual(self.probes(visited),())

    def test_disabled_retains_legacy_candidates(self):
        cfg=replace(self.cfg,unvisited_recovery_viewpoints=False)
        self.assertEqual(_navigation_probes(self.grid,self.obs,self.robot),
            _navigation_probes(self.grid,self.obs,self.robot,cfg,[(0.,0.)]))

    def test_configuration_validation(self):
        with self.assertRaises(ValueError):MappingConfig(unvisited_recovery_viewpoints=1)
        for value in (0.,-1.,float('nan'),float('inf'),True):
            with self.assertRaises(ValueError):MappingConfig(recovery_viewpoint_spacing_m=value)
            with self.assertRaises(ValueError):MappingConfig(recovery_viewpoint_range_m=value)
        with self.assertRaises(ValueError):MappingConfig(recovery_viewpoint_spacing_m=4.)

    def test_mapper_history_is_episode_local_and_failed_frame_is_atomic(self):
        mapper=RGBDMapper()
        init=request(unvisited_recovery_viewpoints=True,dynamic_clear_frames=2)
        mapper.reset(init)
        mapper.update(scene_frame())
        before=dict(mapper._state.viewpoint_positions)
        self.assertTrue(before)
        with patch('frontier_search.perception.mapper.build_candidates',side_effect=RuntimeError('rejected')):
            with self.assertRaises(RuntimeError):
                mapper.update(scene_frame(sequence=2,x=.8))
        self.assertEqual(mapper._state.viewpoint_positions,before)
        mapper.update(scene_frame(sequence=2,x=.8))
        self.assertGreater(len(mapper._state.viewpoint_positions),len(before))
        mapper.reset(init)
        self.assertEqual(mapper._state.viewpoint_positions,{})
