"""Goal-position acquisition survives safety-cache invalidation, never unsafe turns."""
from dataclasses import replace
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from frontier_search.config import MappingConfig
from frontier_search.contracts import (OccupancyGrid, Pose, NavigatorInit, NavigationRequest,
    Subgoal, PolicyName, NavigationStatus, FailureReason)
from frontier_search.experimental.collision_refinement import LocalCollisionNavigator, LocalCollisionPatch
from frontier_search.fixtures.loader import load_policy_fixture
from frontier_search.runner.integrated import shared_round_manifest_for
from test_local_collision_patch import subject


class GoalHysteresisTests(unittest.TestCase):
    def setUp(self):
        self.manifest=shared_round_manifest_for('hold','nearest')
        self.mapping=MappingConfig(resolution_m=.1,origin_x_m=-1.,origin_y_m=-1.,width_cells=20,height_cells=20)
        self.grid=OccupancyGrid('world',1,.1,-1.,-1.,20,20,tuple((0,)*20 for _ in range(20)))
        self.pose=Pose(.02,0.,0.,1.);self.clock=0.;self.command=None;self.current=None
        def command(v,w):self.command=(v,w)
        self.control=SimpleNamespace(current_pose=lambda:self.pose,sim_time_s=lambda:self.clock,set_velocity=command)
        self.nav=LocalCollisionNavigator(self.control,patch_provider=lambda:self.current,mapping_config=self.mapping,start_margin_m=0.)
        self.nav.reset(NavigatorInit('hold',self.manifest.robot,self.manifest.observation,self.manifest.navigation))
        self.goal=Pose(0.,0.,0.,0.)
        request=NavigationRequest('hold','g',self.grid,self.pose,Subgoal('F',self.goal,1,PolicyName.NEAREST))
        self.assertEqual(self.nav.start(request).status,NavigationStatus.ACCEPTED)
        self.nav._active.index=len(self.nav._active.path)-1
        self.nav._active.turn_ok='goal'  # previously acquired, then drifted within public tolerance

    def test_safe_map_update_preserves_position_and_revalidates_goal_turn(self):
        self.nav.update_map('hold','g',replace(self.grid,revision=2))
        cm=self.nav._active.costmap
        with patch.object(cm,'rotation_valid',wraps=cm.rotation_valid) as rotate:
            self.nav.on_step()
        self.assertGreater(rotate.call_count,0)
        self.assertEqual(self.command[0],0.)
        self.assertLess(self.command[1],0.)  # turn toward goal yaw, not back toward its position

    def test_repeated_safe_updates_keep_hysteresis_and_finish_inside_public_tolerance(self):
        for revision,yaw in ((2,.9),(3,.6),(4,.3)):
            self.pose=replace(self.pose,yaw_rad=yaw)
            self.nav.update_map('hold','g',replace(self.grid,revision=revision));self.nav.on_step()
            self.assertEqual(self.command[0],0.)
            self.assertLess(self.command[1],0.)
        self.pose=replace(self.pose,yaw_rad=.05)
        self.nav.update_map('hold','g',replace(self.grid,revision=5));self.nav.on_step()
        self.assertEqual(self.nav.poll('hold','g').status,NavigationStatus.REACHED)

    def test_drift_outside_public_tolerance_reacquires_position(self):
        self.nav.update_map('hold','g',replace(self.grid,revision=2))
        self.pose=replace(self.pose,x_m=.04)
        self.nav.on_step()
        self.assertNotEqual(self.nav.poll('hold','g').status,NavigationStatus.REACHED)
        self.assertGreater(self.command[1],0.)

    def test_new_turn_hazard_cannot_reuse_old_goal_proof(self):
        self.nav.update_map('hold','g',replace(self.grid,revision=2))
        with patch.object(self.nav._active.costmap,'rotation_valid',return_value=False):
            self.nav.on_step()
        self.assertEqual(self.nav.poll('hold','g').failure_reason,FailureReason.BLOCKED)
        self.assertEqual(self.command,(0.,0.))

    def test_real_new_occupancy_stops_before_any_goal_turn(self):
        rows=[list(r) for r in self.grid.data];rows[10][10]=100
        self.nav.update_map('hold','g',replace(self.grid,revision=2,data=tuple(map(tuple,rows))))
        self.assertEqual(self.nav.poll('hold','g').status,NavigationStatus.FAILED)
        self.assertEqual(self.command,(0.,0.))

    def test_new_patch_same_coarse_revision_rechecks_turn_without_losing_position(self):
        state=replace(load_policy_fixture().state,episode_id='hold',revision=1,timestamp_s=0.,
            last_processed_sequence=1,robot_pose=self.pose,occupancy_map=self.grid)
        fine=OccupancyGrid('world',1,.05,-1.,-1.,40,40,tuple((0,)*40 for _ in range(40)))
        self.current=LocalCollisionPatch(state,fine,self.manifest.robot,self.mapping,(.03,.908),0.)
        # Adopt one patch, acquire goal yaw control, then replace the immutable evidence.
        self.nav.on_step();self.nav._active.turn_ok='goal'
        self.current=replace(self.current,grid=replace(fine))
        cm=self.nav._costmap_for(self.grid)
        with patch.object(cm,'rotation_valid',wraps=cm.rotation_valid) as rotate:self.nav.on_step()
        self.assertGreater(rotate.call_count,0)
        self.assertEqual(self.command[0],0.)
        self.assertLess(self.command[1],0.)

    def test_replan_discards_old_acquisition(self):
        self.nav.update_map('hold','g',replace(self.grid,revision=2));self.nav.on_step()
        self.assertTrue(getattr(self.nav._active,'_local_goal_position_acquired',False))
        self.nav._replan(self.nav._active,self.pose)
        self.assertFalse(getattr(self.nav._active,'_local_goal_position_acquired',False))
        self.assertIsNone(self.nav._active.turn_ok)

    def test_old_map_revision_does_not_invalidate_valid_turn_proof(self):
        cm=self.nav._active.costmap
        self.nav.update_map('hold','g',replace(self.grid,revision=0))
        with patch.object(cm,'rotation_valid',wraps=cm.rotation_valid) as rotate:self.nav.on_step()
        self.assertEqual(rotate.call_count,0)
        self.assertLess(self.command[1],0.)

    def test_expired_patch_cannot_preserve_goal_hold_inside_coarse_obstacle(self):
        robot,cfg,pose,coarse,evidence=subject()
        rows=[list(r) for r in coarse.data];rows[6][6]=100
        coarse=replace(coarse,data=tuple(map(tuple,rows)))
        evidence=replace(evidence,state=replace(evidence.state,occupancy_map=coarse))
        now=[0.];live=[pose];commands=[]
        control=SimpleNamespace(current_pose=lambda:live[0],sim_time_s=lambda:now[0],
            set_velocity=lambda v,w:commands.append((v,w)))
        nav=LocalCollisionNavigator(control,patch_provider=lambda:evidence,mapping_config=cfg,start_margin_m=0.)
        nav.reset(NavigatorInit('patch',robot,self.manifest.observation,self.manifest.navigation))
        request=NavigationRequest('patch','g',coarse,pose,Subgoal('F',Pose(.65,.65,0.,0.),1,PolicyName.NEAREST))
        self.assertEqual(nav.start(request).status,NavigationStatus.ACCEPTED)
        live[0]=Pose(.644,.65,0.,1.);nav._active.index=len(nav._active.path)-1
        nav._active.turn_ok='goal';now[0]=1.
        nav.on_step()
        self.assertEqual(nav.poll('patch','g').failure_reason,FailureReason.BLOCKED)
        self.assertEqual(commands[-1],(0.,0.))


if __name__=='__main__':unittest.main()
