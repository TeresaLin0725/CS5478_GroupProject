"""A's bounded release authority; B consumes it consistently without new policies."""
from dataclasses import replace
from types import SimpleNamespace
import importlib.util
import unittest

from frontier_search.contracts import GridCell, Pose, NavigatorInit
from frontier_search.config import NavigationConfig, ObservationConfig
from frontier_search.experimental.collision_refinement import LocalCollisionMapper, LocalCollisionNavigator
from frontier_search.experimental.collision_refinement.patch import current_conflict_cells
from frontier_search.experimental.collision_refinement.costmap import RefinedCostmap
from frontier_search.navigation.planner import plan
from test_local_collision_patch import subject
from test_mapper_update import request, scene_frame


class CurrentConflictScopeTests(unittest.TestCase):
    def setUp(self):
        self.robot,self.cfg,self.pose,self.grid,self.patch=subject()
        rows=[list(r) for r in self.grid.data];rows[4][4]=100
        self.grid=replace(self.grid,data=tuple(map(tuple,rows)))
        self.state=replace(self.patch.state,occupancy_map=self.grid)
        self.patch=replace(self.patch,state=self.state,
            release_cells=current_conflict_cells(self.state,self.robot))
        self.current=self.patch
        self.control=SimpleNamespace(current_pose=lambda:self.pose,sim_time_s=lambda:0.,set_velocity=lambda *args:None)
        self.nav=LocalCollisionNavigator(self.control,patch_provider=lambda:self.current,
            mapping_config=self.cfg,release_scope='current_conflicts',start_margin_m=0.)
        self.nav.reset(NavigatorInit('patch',self.robot,ObservationConfig(),NavigationConfig()))

    def cm(self):return self.nav._costmap_for(self.grid)

    def test_selects_only_current_occupied_conflict_not_distant_obstacle(self):
        self.assertEqual(self.patch.release_cells,(GridCell(5,5),))
        self.assertEqual(self.grid.data[4][4],100)
        cm=self.cm()
        self.assertTrue(cm.pose_valid(self.pose))
        self.assertFalse(cm.pose_valid(Pose(.45,.45,0.,0.)))
        self.assertNotIn((4,4),cm._rectangles)
        # The old full-window mode would release that distant coarse obstacle.
        full=RefinedCostmap(cm.coarse,replace(self.patch,release_cells=None),lambda:True)
        self.assertTrue(full.pose_valid(Pose(.45,.45,0.,0.)))

    def test_normally_colliding_margin_is_included(self):
        pose=Pose(.625,.58,0.,0.)
        robot=replace(self.robot,clearance_margin_m=.01)
        state=replace(self.state,robot_pose=pose)
        self.assertEqual(current_conflict_cells(state,self.robot),())
        self.assertEqual(current_conflict_cells(state,robot),(GridCell(5,5),))

    def test_unknown_is_not_a_release_candidate_and_still_blocks(self):
        rows=[list(r) for r in self.grid.data];rows[5][5]=-1
        self.grid=replace(self.grid,data=tuple(map(tuple,rows)))
        self.state=replace(self.state,occupancy_map=self.grid)
        self.current=replace(self.patch,state=self.state,release_cells=current_conflict_cells(self.state,self.robot))
        self.assertEqual(self.current.release_cells,())
        self.assertFalse(self.cm().pose_valid(self.pose))

    def test_unknown_and_occupied_children_still_block_authorized_cell(self):
        for value in (-1,100):
            rows=[list(r) for r in self.patch.grid.data];rows[3][3]=value
            self.current=replace(self.patch,grid=replace(self.patch.grid,data=tuple(map(tuple,rows))))
            with self.subTest(value=value):
                cm=self.cm()
                self.assertFalse(cm.start_pose_valid(self.pose))
                self.assertFalse(cm.rotation_valid(self.pose,1.))
                self.assertFalse(cm.segment_valid(self.pose,Pose(.85,.65,0.,0.)))

    def test_empty_authority_still_blocks_new_fine_obstacle_in_coarse_free_space(self):
        rows=[list(r) for r in self.grid.data];rows[5][5]=0
        self.grid=replace(self.grid,data=tuple(map(tuple,rows)))
        self.state=replace(self.state,occupancy_map=self.grid)
        fine=[list(r) for r in self.patch.grid.data];fine[3][3]=100
        self.current=replace(self.patch,state=self.state,grid=replace(self.patch.grid,data=tuple(map(tuple,fine))),release_cells=())
        self.assertFalse(self.cm().pose_valid(self.pose))

    def test_unjustified_or_stale_authority_falls_back_to_coarse_rejection(self):
        variants=((),(GridCell(4,4),GridCell(5,5)),(GridCell(5,5),GridCell(5,5)),
                  (GridCell(-1,5),),None)
        for cells in variants:
            self.current=replace(self.patch,release_cells=cells)
            with self.subTest(cells=cells):
                self.assertNotIsInstance(self.cm(),RefinedCostmap)
                self.assertFalse(self.cm().pose_valid(self.pose))
        # The old authorization does not follow a robot that moved elsewhere.
        self.current=replace(self.patch,state=replace(self.state,robot_pose=Pose(.75,.75,0.,0.)))
        self.assertNotIsInstance(self.cm(),RefinedCostmap)

    def test_planning_and_execution_share_same_authority(self):
        cm=self.cm()
        self.assertTrue(plan(cm,self.pose,Pose(.85,.65,0.,0.)).success)
        self.assertFalse(plan(cm,self.pose,Pose(.45,.45,0.,0.)).success)
        self.assertFalse(cm.cell_traversable(4,4))
        self.assertFalse(cm.first_move_valid(self.pose,.45,.45))
        self.assertTrue(cm.first_move_valid(self.pose,.65,.65))

    def test_scope_mismatch_and_typo_are_rejected(self):
        for cls,args,options in ((LocalCollisionMapper,(),{}),(LocalCollisionNavigator,(self.control,),
                dict(patch_provider=lambda:self.patch,mapping_config=self.cfg))):
            with self.assertRaises(ValueError):cls(*args,release_scope='typo',**options)
        nav=LocalCollisionNavigator(self.control,patch_provider=lambda:self.patch,mapping_config=self.cfg)
        nav.reset(NavigatorInit('patch',self.robot,ObservationConfig(),NavigationConfig()))
        self.assertNotIsInstance(nav._costmap_for(self.grid),RefinedCostmap)


@unittest.skipUnless(importlib.util.find_spec('numpy'),'NumPy required for local fusion')
class CurrentConflictMapperTests(unittest.TestCase):
    def test_scope_only_changes_authority_not_global_or_local_evidence(self):
        init=request()
        init=replace(init,mapping_config=replace(init.mapping_config,resolution_m=.1,width_cells=50,height_cells=40))
        full=LocalCollisionMapper();scoped=LocalCollisionMapper(release_scope='current_conflicts')
        full.reset(init);scoped.reset(init)
        for sequence,x in ((1,0.),(2,.1),(3,.6)):
            frame=scene_frame(sequence=sequence,x=x)
            a,b=full.update(frame),scoped.update(frame)
            self.assertEqual(a,b)
            self.assertEqual(full.collision_patch.grid,scoped.collision_patch.grid)
            self.assertEqual(full._local.buffers,scoped._local.buffers)
            self.assertIsNone(full.collision_patch.release_cells)
            self.assertEqual(scoped.collision_patch.release_cells,current_conflict_cells(b,init.robot_config))


if __name__=='__main__':unittest.main()
