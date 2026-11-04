"""Independent evidence release, rejection, cache/expiry and executed safety checks."""
from dataclasses import replace
from math import pi
from types import SimpleNamespace
import unittest
import importlib.util
from unittest.mock import patch as mock_patch

from frontier_search.config import MappingConfig, NavigationConfig, ObservationConfig
from frontier_search.contracts import (OccupancyGrid, Pose, RobotConfig, NavigatorInit,
    NavigationRequest, NavigationStatus, PolicyName, Subgoal, FailureReason)
from frontier_search.experimental.collision_refinement import (LocalCollisionPatch,
    LocalCollisionNavigator, LocalCollisionMapper)
from frontier_search.experimental.collision_refinement.costmap import RefinedCostmap
from frontier_search.fixtures.loader import load_policy_fixture
from frontier_search.navigation.geometry import circle_footprint
from frontier_search.navigation.costmap import Costmap
from frontier_search.navigation.planner import path_valid, plan
from frontier_search.simulation.mock_adapter import MockSimulator
from test_mapper_update import request, scene_frame


def subject():
    robot=RobotConfig(circle_footprint(.02),0.,1.,0.,'differential_drive',True,.3,1.,.01,.1)
    cfg=MappingConfig(resolution_m=.1,width_cells=10,height_cells=10)
    rows=[[0]*10 for _ in range(10)];rows[5][5]=100
    grid=OccupancyGrid('world',1,.1,0.,0.,10,10,tuple(map(tuple,rows)))
    pose=Pose(.58,.58,0.,0.)
    state=replace(load_policy_fixture().state,episode_id='patch',revision=1,timestamp_s=0.,
        last_processed_sequence=1,occupancy_map=grid,robot_pose=pose,frontiers=())
    cells=[[0]*6 for _ in range(6)];cells[2][2]=100
    fine=OccupancyGrid('world',1,.05,.4,.4,6,6,tuple(map(tuple,cells)))
    evidence=LocalCollisionPatch(state,fine,robot,cfg,(.03,1.),0.)
    return robot,cfg,pose,grid,evidence


class LocalCollisionPatchTests(unittest.TestCase):
    def setUp(self):
        self.robot,self.cfg,self.pose,self.grid,self.patch=subject()
        self.current=self.patch;self.clock=0.
        self.control=SimpleNamespace(current_pose=lambda:self.pose,sim_time_s=lambda:self.clock,set_velocity=lambda *args:None)
        self.nav=LocalCollisionNavigator(self.control,patch_provider=lambda:self.current,
            mapping_config=self.cfg,start_margin_m=0.)
        self.nav.reset(NavigatorInit('patch',self.robot,ObservationConfig(),NavigationConfig()))

    def map(self):return self.nav._costmap_for(self.grid)

    def set_child(self, value):
        rows=[list(r) for r in self.patch.grid.data];rows[3][3]=value
        self.current=replace(self.patch,grid=replace(self.patch.grid,data=tuple(map(tuple,rows))))

    def test_refines_only_intersecting_free_children_and_preserves_wall(self):
        self.assertFalse(Costmap.from_robot(self.grid,self.robot).start_pose_valid(self.pose))
        cm=self.map()
        self.assertIsInstance(cm,RefinedCostmap)
        self.assertTrue(cm.start_pose_valid(self.pose))
        self.assertTrue(cm.pose_valid(self.pose))
        self.assertTrue(cm.full_rotation_valid(self.pose))
        self.assertTrue(cm.first_move_valid(self.pose,.65,.65))
        self.assertTrue(cm.segment_valid(self.pose,Pose(.85,.65,0.,0.)))
        self.assertEqual(self.current.grid.data[2][2],100)
        self.assertEqual(self.grid.data[5][5],100)
        self.assertFalse(cm.pose_valid(Pose(.525,.525,0.,0.)))

    def test_unknown_or_occupied_child_rejects_start_turn_and_segment(self):
        for value in (-1,100):
            self.set_child(value);cm=self.map()
            with self.subTest(value=value):
                self.assertFalse(cm.start_pose_valid(self.pose))
                self.assertFalse(cm.rotation_valid(self.pose,pi))
                self.assertFalse(cm.segment_valid(self.pose,Pose(.85,.65,0.,0.)))
                self.assertFalse(cm.first_move_valid(self.pose,.65,.65))

    def test_fresh_fine_obstacle_rejects_previously_free_coarse_space(self):
        self.grid=replace(self.grid,data=tuple((0,)*10 for _ in range(10)))
        self.patch=replace(self.patch,state=replace(self.patch.state,occupancy_map=self.grid))
        self.set_child(100)
        self.assertFalse(self.map().pose_valid(self.pose))

    def test_coarse_unknown_and_map_boundary_cannot_be_released(self):
        rows=[list(r) for r in self.grid.data];rows[5][5]=-1
        self.grid=replace(self.grid,data=tuple(map(tuple,rows)))
        self.current=replace(self.patch,state=replace(self.patch.state,occupancy_map=self.grid))
        self.assertFalse(self.map().pose_valid(self.pose))
        self.assertFalse(self.map().pose_valid(Pose(-.01,.5,0.,0.)))

    def test_missing_or_expired_patch_invalidates_cached_release(self):
        cm=self.map();self.assertTrue(cm.pose_valid(self.pose))
        cm.cell_traversable(6,6)  # populate a planning cache before invalidation
        self.clock=.5
        self.assertFalse(cm.pose_valid(self.pose))
        self.assertNotIsInstance(self.map(),RefinedCostmap)
        self.clock=0.;self.current=None
        self.assertFalse(cm.pose_valid(self.pose))
        self.assertNotIsInstance(self.map(),RefinedCostmap)

    def test_replacing_patch_invalidates_pose_and_cell_cache(self):
        old=self.map();self.assertTrue(old.pose_valid(self.pose))
        self.set_child(100)
        new=self.map();self.assertIsNot(old,new)
        self.assertFalse(old.pose_valid(self.pose))
        self.assertFalse(new.pose_valid(self.pose))

    def test_metadata_mismatch_falls_back_to_original_rejection(self):
        bad=(replace(self.patch,version='other'),
            replace(self.patch,state=replace(self.patch.state,episode_id='other')),
            replace(self.patch,state=replace(self.patch.state,timestamp_s=1.)),
            replace(self.patch,state=replace(self.patch.state,occupancy_map=replace(self.grid))),
            replace(self.patch,grid=replace(self.patch.grid,revision=0)),
            replace(self.patch,grid=replace(self.patch.grid,origin_x_m=.401)),
            replace(self.patch,collision_z=(.03,.9)),replace(self.patch,base_z_m=.01),
            replace(self.patch,robot=replace(self.robot,collision_z_max_m=.9)),
            replace(self.patch,mapping=replace(self.cfg,depth_pixel_stride=2)))
        for p in bad:
            self.current=p
            with self.subTest(p=p):
                self.assertNotIsInstance(self.map(),RefinedCostmap)
                self.assertFalse(self.map().pose_valid(self.pose))

    def test_coarse_planning_nodes_and_fine_segment_checks_agree(self):
        cm=self.map();goal=Pose(.85,.65,0.,0.)
        result=plan(cm,self.pose,goal)
        self.assertTrue(result.success)
        self.assertTrue(path_valid(cm,result.path))
        self.assertIs(cm.grid,self.grid)
        self.assertEqual(cm.grid.resolution_m,.1)
        self.assertEqual(cm.sample_step_m,.025)

    def test_patch_context_is_immutable(self):
        from dataclasses import FrozenInstanceError
        with self.assertRaises(FrozenInstanceError):self.patch.base_z_m=2.

    def test_opt_in_does_not_change_ordinary_costmap(self):
        self.assertFalse(Costmap.from_robot(self.grid,self.robot).pose_valid(self.pose))
        self.assertTrue(self.map().pose_valid(self.pose))


class LocalCollisionExecutionTests(unittest.TestCase):
    def setup_nav(self, max_age=10.):
        robot,cfg,pose,coarse,evidence=subject()
        true_cells=[[0]*20 for _ in range(20)];true_cells[10][10]=100
        truth=OccupancyGrid('world',1,.05,0.,0.,20,20,tuple(map(tuple,true_cells)))
        sim=MockSimulator(truth,robot);self.addCleanup(sim.close)
        sim.reset('patch','mock',pose,0)
        self.current=evidence
        nav=LocalCollisionNavigator(sim,patch_provider=lambda:self.current,mapping_config=cfg,start_margin_m=0.)
        nav.reset(NavigatorInit('patch',robot,ObservationConfig(),NavigationConfig(map_freshness_s=max_age)))
        req=NavigationRequest('patch','go',coarse,pose,Subgoal('F',Pose(.85,.65,0.,0.),1,PolicyName.NEAREST))
        return sim,nav,req

    def test_drives_from_refined_start_without_mock_contacts(self):
        sim,nav,req=self.setup_nav()
        f=nav.start(req)
        for _ in range(180):
            if f.status in (NavigationStatus.REACHED,NavigationStatus.FAILED):break
            sim.step();nav.on_step();f=nav.poll('patch','go')
        self.assertEqual(f.status,NavigationStatus.REACHED)
        self.assertEqual(sim.collisions,0)

    def test_expiry_during_execution_stops_a_cached_operation(self):
        sim,nav,req=self.setup_nav(.4)
        self.assertEqual(nav.start(req).status,NavigationStatus.ACCEPTED)
        sim._t=.5;nav.on_step()
        self.assertEqual(nav.poll('patch','go').failure_reason,FailureReason.BLOCKED)
        self.assertEqual((sim._v,sim._w),(0.,0.))

    def test_replaced_evidence_stops_even_without_new_coarse_revision(self):
        sim,nav,req=self.setup_nav()
        self.assertEqual(nav.start(req).status,NavigationStatus.ACCEPTED)
        rows=[list(r) for r in self.current.grid.data];rows[3][3]=100
        self.current=replace(self.current,grid=replace(self.current.grid,data=tuple(map(tuple,rows))))
        nav.on_step()
        self.assertEqual(nav.poll('patch','go').failure_reason,FailureReason.BLOCKED)
        self.assertEqual((sim._v,sim._w),(0.,0.))

    def test_new_fine_obstacle_is_applied_to_an_operation_started_without_patch(self):
        sim,nav,req=self.setup_nav()
        original=self.current
        self.current=None
        sim._pose=Pose(.68,.58,0.,0.)
        req=replace(req,start_pose=sim._pose)
        self.assertEqual(nav.start(req).status,NavigationStatus.ACCEPTED)
        rows=[list(r) for r in original.grid.data];rows[3][5]=100
        self.current=replace(original,grid=replace(original.grid,data=tuple(map(tuple,rows))))
        nav.on_step()
        self.assertEqual(nav.poll('patch','go').failure_reason,FailureReason.BLOCKED)
        self.assertEqual((sim._v,sim._w),(0.,0.))


@unittest.skipUnless(importlib.util.find_spec('numpy'), 'NumPy required for experimental local fusion')
class LocalCollisionMapperTests(unittest.TestCase):
    def test_global_outputs_equal_and_failed_local_fusion_rolls_back(self):
        from frontier_search.perception.mapper import RGBDMapper
        init=request()
        init=replace(init,mapping_config=replace(init.mapping_config,resolution_m=.1,width_cells=50,height_cells=40))
        coarse=RGBDMapper();fine=LocalCollisionMapper()
        coarse.reset(init);fine.reset(init)
        frame=scene_frame()
        self.assertEqual(coarse.update(frame),fine.update(frame))
        old_state,old_patch,old_buffers=fine._state,fine.collision_patch,fine._local.buffers
        with mock_patch.object(fine._local,'update',side_effect=ValueError('fusion rejected')):
            with self.assertRaises(ValueError):fine.update(scene_frame(sequence=2))
        self.assertIs(fine._state,old_state)
        self.assertIs(fine.collision_patch,old_patch)
        self.assertIs(fine._local.buffers,old_buffers)
        self.assertEqual(coarse.update(scene_frame(sequence=2)),fine.update(scene_frame(sequence=2)))


if __name__=='__main__':unittest.main()
