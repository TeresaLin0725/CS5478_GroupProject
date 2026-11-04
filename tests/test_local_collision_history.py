"""Evidence survives window eviction; fresh hazards and reset rules still win."""
from copy import deepcopy
from dataclasses import replace
from pathlib import Path
import importlib.util
from math import atan2
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path[:0]=[str(Path(__file__).resolve().parents[1]/'scripts')]
from frontier_search.config import MappingConfig
from frontier_search.fixtures.analytic_rgbd import render_frame
from frontier_search.experimental.collision_refinement.rolling_local_map import RollingLocalMap
from frontier_search.experimental.collision_refinement import LocalCollisionMapper
from frontier_search.navigation.geometry import world_to_cell
from frontier_search.perception.fusion import _finalize_free_space
from test_local_collision_window import frame
from test_mapper_update import request, scene_frame


@unittest.skipUnless(importlib.util.find_spec('numpy'), 'NumPy required for experimental local fusion')
class HistoryTests(unittest.TestCase):
    def subject(self, enabled=True):
        req=SimpleNamespace(episode_id='synthetic-rgbd',mapping_config=MappingConfig(
            width_cells=70,height_cells=85,origin_x_m=-4.5,origin_y_m=-4.5,dynamic_clear_frames=2),
            initial_clearance_cells=())
        return RollingLocalMap(req,retain_history=enabled)

    def seed(self, local, occupancy=0, mask=(1<<18)-1, floor_seen=True, count=0):
        local.update(frame(),(.04,.923),1)
        cell=world_to_cell(local.grid,.475,1.175)
        for name,value in dict(occupancy=occupancy,coverage=True,free_masks=mask,
                floor_seen=floor_seen,clear_counts=count).items():
            local.buffers[name][cell.row][cell.column]=value

    def leave(self, local):
        local.update(frame(2,-3.),(.04,.923),2)

    def back(self, local, sequence=3):
        local.update(frame(sequence),(.04,.923),sequence)
        return world_to_cell(local.grid,.475,1.175)

    def test_complete_observed_free_column_survives_leave_and_return(self):
        local=self.subject();self.seed(local);snapshot=local.grid
        self.leave(local);self.assertGreater(len(local._history),0)
        cell=self.back(local)
        self.assertEqual(local.grid.data[cell.row][cell.column],0)
        self.assertEqual(local.buffers['free_masks'][cell.row][cell.column],(1<<18)-1)
        self.assertTrue(local.buffers['floor_seen'][cell.row][cell.column])
        self.assertEqual(local.history_restored_cells,1)
        self.assertEqual(len(local._history),0)
        self.assertIsNot(snapshot,local.grid)

    def test_disabled_history_preserves_old_unknown_on_return(self):
        local=self.subject(False);self.seed(local);self.leave(local);cell=self.back(local)
        self.assertEqual(local.grid.data[cell.row][cell.column],-1)
        self.assertEqual(local._history,{})

    def test_partial_evidence_and_unseen_cells_remain_unknown(self):
        local=self.subject();self.seed(local,-1,(1<<17)-1)
        self.leave(local);cell=self.back(local)
        self.assertEqual(local.grid.data[cell.row][cell.column],-1)
        self.assertEqual(local.buffers['free_masks'][cell.row][cell.column],(1<<17)-1)
        unseen=world_to_cell(local.grid,.525,1.175)
        self.assertEqual(local.grid.data[unseen.row][unseen.column],-1)

    def test_new_obstacle_wins_over_restored_free_evidence(self):
        local=self.subject();self.seed(local);self.leave(local)
        obstacle=render_frame(sequence=3,width=9,height=9,yaw=atan2(1.175,.475),
            boxes=(((.455,1.155,0.),(.495,1.195,1.)),))
        local.update(obstacle,(.04,.923),3)
        cell=world_to_cell(local.grid,.475,1.175)
        self.assertEqual(local.grid.data[cell.row][cell.column],100)

    def test_consecutive_clearing_credit_cannot_cross_window_absence(self):
        local=self.subject();self.seed(local,100,count=1);self.leave(local)
        def clear(observation,cfg,band,occupancy,coverage,masks,floors,clear_counts=None,increment_clear_count=True):
            c=int((.475-cfg.origin_x_m)/cfg.resolution_m);r=int((1.175-cfg.origin_y_m)/cfg.resolution_m)
            fresh=[[0]*cfg.width_cells for _ in range(cfg.height_cells)];fresh[r][c]=(1<<18)-1
            _finalize_free_space(cfg,occupancy,masks,floors,clear_counts,increment_clear_count,
                fresh,{(r,c)},set(),(1<<18)-1)
        with patch('frontier_search.experimental.collision_refinement.rolling_local_map.fuse_observation',side_effect=clear):
            cell=self.back(local)
            self.assertEqual(local.grid.data[cell.row][cell.column],100)
            self.assertEqual(local.buffers['clear_counts'][cell.row][cell.column],1)
            self.back(local,4)
        self.assertEqual(local.grid.data[cell.row][cell.column],0)

    def test_failed_return_does_not_consume_history_or_change_published_state(self):
        local=self.subject();self.seed(local);self.leave(local)
        history=local._history;old_grid=local.grid;before=deepcopy(history)
        with patch('frontier_search.experimental.collision_refinement.rolling_local_map.fuse_observation',side_effect=ValueError('reject')):
            with self.assertRaises(ValueError):self.back(local)
        self.assertIs(local.grid,old_grid);self.assertIs(local._history,history)
        self.assertEqual(local._history,before)
        self.assertEqual(local.history_restored_cells,0)
        cell=self.back(local);self.assertEqual(local.grid.data[cell.row][cell.column],0)

    def test_history_rejects_changed_height_or_grid_geometry(self):
        local=self.subject();self.seed(local);self.leave(local);history=deepcopy(local._history)
        with self.assertRaisesRegex(ValueError,'geometry'):
            local.update(frame(3),(.04,.924),3)
        local.base=replace(local.base,clearance_bin_height_m=.025)
        with self.assertRaisesRegex(ValueError,'geometry'):self.back(local)
        self.assertEqual(local._history,history)

    def test_cache_is_bounded_by_map_and_tracks_only_evicted_evidence(self):
        local=self.subject();self.seed(local)
        for seq,x in enumerate((-3.,0.,3.,0.,-3.,0.,12.,0.),2):
            local.update(frame(seq,x),(.04,.923),seq)
            self.assertLessEqual(len(local._history),local.base.width_cells*local.base.height_cells*local.factor**2)
            for r,c in local._history:
                self.assertTrue(0<=r<local.base.height_cells*local.factor)
                self.assertTrue(0<=c<local.base.width_cells*local.factor)
                self.assertFalse(local.origin[1]*local.factor<=r<(local.origin[1]*local.factor+local.width) and
                    local.origin[0]*local.factor<=c<(local.origin[0]*local.factor+local.width))

    def test_mapper_reset_discards_previous_episode_history(self):
        init=request();init=replace(init,mapping_config=replace(init.mapping_config,resolution_m=.1,width_cells=50,height_cells=40))
        mapper=LocalCollisionMapper(retain_history=True);mapper.reset(init)
        mapper.update(scene_frame());old=mapper._local
        old._history[(1,1)]=(0,True,7,True)
        mapper.reset(replace(init,episode_id='new-episode'))
        self.assertIsNot(mapper._local,old);self.assertEqual(mapper._local._history,{})
        self.assertEqual(mapper._local.history_restored_cells,0)

    def test_mapper_rolls_back_history_if_patch_creation_fails_after_fusion(self):
        init=request();init=replace(init,mapping_config=replace(init.mapping_config,resolution_m=.1,width_cells=50,height_cells=40))
        mapper=LocalCollisionMapper(retain_history=True);mapper.reset(init);mapper.update(scene_frame())
        old_state,old_patch=mapper._state,mapper.collision_patch
        old_history,old_grid=mapper._local._history,mapper._local.grid
        old_shifts=mapper._local.shifts
        with patch('frontier_search.experimental.collision_refinement.bridge.LocalCollisionPatch',side_effect=ValueError('patch rejected')):
            with self.assertRaises(ValueError):mapper.update(scene_frame(sequence=2,x=3.))
        self.assertIs(mapper._state,old_state);self.assertIs(mapper.collision_patch,old_patch)
        self.assertIs(mapper._local._history,old_history);self.assertIs(mapper._local.grid,old_grid)
        self.assertEqual(mapper._local.shifts,old_shifts)
        mapper.update(scene_frame(sequence=2,x=3.))
        self.assertGreater(len(mapper._local._history),0)

    def test_invalid_switches_rejected_in_mapper_and_benchmark(self):
        from benchmark_episode import local_collision_switches
        self.assertEqual(local_collision_switches({}),(False,False))
        self.assertEqual(local_collision_switches(dict(local_collision_refinement=True,local_collision_retain_history=True)),(True,True))
        for value in (0,1,'true',None):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):LocalCollisionMapper(retain_history=value)
                with self.assertRaises(ValueError):local_collision_switches(dict(local_collision_retain_history=value))
        with self.assertRaises(ValueError):local_collision_switches(dict(local_collision_retain_history=True))


if __name__=='__main__':unittest.main()
