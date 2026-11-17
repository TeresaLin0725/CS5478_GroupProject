"""Pose recovery must leave existing unknown overlap without opening new space."""
from dataclasses import replace
import unittest

from frontier_search.config import NavigationConfig, ObservationConfig
from frontier_search.contracts import OccupancyGrid, Pose, NavigatorInit, ObservationRequest, NavigationStatus
from frontier_search.navigation.costmap import Costmap
from frontier_search.navigation.locobot_robot import LOCOBOT_ROUND_ROBOT
from frontier_search.navigation.navigator import TeamNavigator
from frontier_search.navigation.pose_recovery import choose_pose_egress, egress_path_valid
from frontier_search.simulation.mock_adapter import MockSimulator


class PoseRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.robot=replace(LOCOBOT_ROUND_ROBOT,clearance_margin_m=.01)
        cells=[[0]*40 for _ in range(40)]
        cells[15][5]=-1
        self.grid=OccupancyGrid('world',1,.1,0.,0.,40,40,tuple(map(tuple,cells)))
        # Recorded failure geometry translated into an authored grid.
        self.pose=Pose(.7769496442,1.656866961,0.,.0659682675)

    def costmap(self, grid=None):
        return Costmap.from_robot(grid or self.grid,self.robot,start_margin_m=0.)

    def navigator(self, routine='recovery'):
        physical=replace(self.grid,data=tuple(tuple(0 if v==-1 else v for v in row) for row in self.grid.data))
        sim=MockSimulator(physical,self.robot)
        sim.reset('E','mock',self.pose,0)
        nav=TeamNavigator(sim,start_margin_m=0.)
        nav.reset(NavigatorInit('E',self.robot,ObservationConfig(recovery_headings_rad=(0.,),dwell_s=0.),NavigationConfig()))
        req=ObservationRequest('E','scan',self.grid,self.pose,routine,self.pose.yaw_rad)
        return sim,nav,req

    def test_unknown_overlap_can_exit_without_mutating_map(self):
        cm=self.costmap()
        self.assertFalse(cm.start_pose_valid(self.pose))
        escape=choose_pose_egress(cm,self.pose)
        self.assertIsNotNone(escape)
        self.assertTrue(egress_path_valid(cm,self.pose,escape))
        self.assertTrue(cm.full_rotation_valid(escape.goal))
        self.assertEqual(self.grid.data[15][5],-1)

    def test_occupied_overlap_is_never_eligible(self):
        cells=[list(row) for row in self.grid.data];cells[15][5]=100
        self.assertIsNone(choose_pose_egress(self.costmap(replace(self.grid,data=tuple(map(tuple,cells)))),self.pose))

    def test_new_unknown_or_occupied_cell_invalidates_escape(self):
        escape=choose_pose_egress(self.costmap(),self.pose)
        for value in (-1,100):
            cells=[list(row) for row in self.grid.data];cells[16][10]=value
            changed=replace(self.grid,revision=2,data=tuple(map(tuple,cells)))
            self.assertFalse(egress_path_valid(self.costmap(changed),self.pose,escape))

    def test_drifting_deeper_into_original_unknown_is_rejected(self):
        escape=choose_pose_egress(self.costmap(),self.pose)
        deeper=replace(self.pose,x_m=self.pose.x_m-.02)
        self.assertFalse(egress_path_valid(self.costmap(),deeper,escape))

    def test_recovery_moves_then_completes_scan_with_zero_contacts(self):
        sim,nav,req=self.navigator()
        self.addCleanup(sim.close)
        self.assertEqual(nav.start_observation(req).status,NavigationStatus.ACCEPTED)
        for _ in range(180):
            sim.step();nav.on_step()
            feedback=nav.poll('E','scan')
            if feedback.status in (NavigationStatus.REACHED,NavigationStatus.FAILED):break
        self.assertEqual(feedback.status,NavigationStatus.REACHED)
        self.assertEqual(sim.collisions,0)
        self.assertLess(sim.executed_distance_m,.20)
        self.assertTrue(self.costmap().start_pose_valid(sim.current_pose()))
        self.assertTrue(any('completed' in text for kind,text in nav.events if kind=='pose_egress'))

    def test_arrival_cannot_silently_enable_egress(self):
        sim,nav,req=self.navigator('arrival');self.addCleanup(sim.close)
        self.assertEqual(nav.start_observation(req).status,NavigationStatus.FAILED)
        self.assertEqual(sim.executed_distance_m,0.)

    def test_changed_map_stops_active_egress(self):
        sim,nav,req=self.navigator();self.addCleanup(sim.close)
        nav.start_observation(req)
        cells=[list(row) for row in self.grid.data];cells[15][5]=100
        nav.update_map('E','scan',replace(self.grid,revision=2,data=tuple(map(tuple,cells))))
        self.assertEqual(nav.poll('E','scan').status,NavigationStatus.FAILED)
        self.assertEqual((sim._v,sim._w),(0.,0.))

    def test_contact_feedback_prevents_starting_egress(self):
        sim,nav,req=self.navigator();self.addCleanup(sim.close)
        sim.contact_this_step=True
        self.assertEqual(nav.start_observation(req).status,NavigationStatus.FAILED)

    def test_egress_can_be_cancelled_and_stops_motors(self):
        sim,nav,req=self.navigator();self.addCleanup(sim.close)
        nav.start_observation(req);sim.step();nav.on_step()
        self.assertNotEqual(sim._v,0.)
        self.assertEqual(nav.cancel('E','scan','user_cancel').status,NavigationStatus.CANCELLED)
        self.assertEqual((sim._v,sim._w),(0.,0.))

    def test_egress_timeout_stops_even_if_pose_keeps_jittering(self):
        sim,nav,req=self.navigator();self.addCleanup(sim.close)
        nav.start_observation(req)
        sim._t=6.1
        nav.on_step()
        self.assertEqual(nav.poll('E','scan').status,NavigationStatus.FAILED)
        self.assertEqual((sim._v,sim._w),(0.,0.))


if __name__=='__main__':unittest.main()
