"""Acceptance regressions for close-object search and full-body crowded motion."""
from dataclasses import replace
import importlib.util
from math import pi
import unittest

HAS_DEPS=all(importlib.util.find_spec(m) for m in ('numpy',))


@unittest.skipUnless(HAS_DEPS,'optional numpy not installed')
class CloseObjectSearchTests(unittest.TestCase):
    def test_diagonal_observed_boundary_survives_frontier_size_filter(self):
        from frontier_search.config import MappingConfig
        from frontier_search.contracts import OccupancyGrid
        from frontier_search.perception.frontiers import extract_clusters
        # A range fan at a doorway leaves a staircase of free/unknown cells.
        # Four-connectivity splits it into tiny pieces and loses the doorway.
        cells=[[-1]*12 for _ in range(12)]
        for r in range(1,10):
            for c in range(1,r+1):cells[r][c]=0
        grid=OccupancyGrid('world',1,.15,0,0,12,12,tuple(map(tuple,cells)))
        clusters=extract_clusters(grid,MappingConfig(frontier_connectivity=8,frontier_min_cells=3))
        self.assertTrue(any(all(any(c.row==i and c.column==i for c in cluster)
                                for i in range(2,9)) for cluster in clusters))

    def grid(self,gap=7):
        from frontier_search.contracts import OccupancyGrid
        cells=[[0]*80 for _ in range(60)]
        for row in (20,21+gap):
            for col in range(20,70): cells[row][col]=100
        for r in range(60): cells[r][0]=cells[r][-1]=100
        for c in range(80): cells[0][c]=cells[-1][c]=100
        return OccupancyGrid('world',1,.1,0,0,80,60,tuple(map(tuple,cells)))

    def test_real_rectangle_can_drive_straight_but_cannot_rotate_in_same_aisle(self):
        from frontier_search.contracts import Pose,FailureReason
        from frontier_search.navigation.lattice import LatticeSearch
        from frontier_search.contracts import Point2, RobotConfig
        ROBOT = RobotConfig(tuple(Point2(x,y) for x,y in ((-.34,-.22),(.34,-.22),(.34,.22),(-.34,.22))),
                            0.,1.04,.035,'ideal_differential_drive_rectangular',True,.6,1.,.04,.04)
        search=LatticeSearch(self.grid(),ROBOT)
        results=search.solve(Pose(1.45,2.45,0,0),
            {'straight':Pose(5.05,2.45,0,0),'turn':Pose(5.05,2.45,0,pi/2)},5)
        self.assertTrue(results['straight'].success)
        self.assertAlmostEqual(results['straight'].path_cost_m,3.6,places=4)
        self.assertEqual(results['turn'].failure_reason,FailureReason.INVALID_GOAL)
        self.assertTrue(all(search.costmap.pose_valid(p) for p in results['straight'].path))

    def test_point_gap_does_not_let_extended_body_through(self):
        from frontier_search.contracts import Pose,FailureReason
        from frontier_search.navigation.lattice import LatticeSearch
        from frontier_search.contracts import Point2, RobotConfig
        ROBOT = RobotConfig(tuple(Point2(x,y) for x,y in ((-.34,-.22),(.34,-.22),(.34,.22),(-.34,.22))),
                            0.,1.04,.035,'ideal_differential_drive_rectangular',True,.6,1.,.04,.04)
        grid=self.grid(3)
        self.assertEqual(grid.data[22][50],0) # point robot has a free centre
        answer=LatticeSearch(grid,ROBOT).solve(Pose(1.45,2.25,0,0),{'gap':Pose(5.05,2.25,0,0)},5)['gap']
        self.assertEqual(answer.failure_reason,FailureReason.INVALID_GOAL)

    def test_reverse_exit_does_not_require_a_dangerous_180_degree_turn(self):
        from frontier_search.contracts import Pose
        from frontier_search.navigation.lattice import LatticeSearch
        from frontier_search.contracts import Point2, RobotConfig
        ROBOT = RobotConfig(tuple(Point2(x,y) for x,y in ((-.34,-.22),(.34,-.22),(.34,.22),(-.34,.22))),
                            0.,1.04,.035,'ideal_differential_drive_rectangular',True,.6,1.,.04,.04)
        answer=LatticeSearch(self.grid(),ROBOT).solve(Pose(5.05,2.45,0,0),{'exit':Pose(1.45,2.45,0,0)},5)['exit']
        self.assertTrue(answer.success)
        self.assertTrue(any(b.x_m<a.x_m-.05 and abs(a.yaw_rad)<.01 for a,b in zip(answer.path,answer.path[1:])))

    def test_safe_turn_endpoints_do_not_imply_a_safe_intermediate_sweep(self):
        from frontier_search.contracts import Pose
        from frontier_search.navigation.lattice import LatticeSearch
        from frontier_search.contracts import Point2, RobotConfig
        ROBOT = RobotConfig(tuple(Point2(x,y) for x,y in ((-.34,-.22),(.34,-.22),(.34,.22),(-.34,.22))),
                            0.,1.04,.035,'ideal_differential_drive_rectangular',True,.6,1.,.04,.04)
        costmap=LatticeSearch(self.grid(),ROBOT).costmap
        pose=Pose(5.05,2.45,0,0)
        self.assertTrue(costmap.pose_valid(pose))
        self.assertTrue(costmap.pose_valid(Pose(5.05,2.45,0,pi)))
        self.assertFalse(costmap.rotation_valid(pose,pi))



    def test_independent_verifier_rejects_visible_but_distant_target(self):
        from frontier_search.contracts import (Pose,Point3,TargetClaim,DetectionEvidence,EvidenceBundle,EvaluationRequest)
        from frontier_search.evaluation.proximity import ProximityEvaluator
        class World:
            target_position=(2.,0.,.82)
            visible_evidence={1:50,2:60}
            _v=_w=0.
            def current_pose(self):return Pose(0,0,0,0)
        detections=tuple(DetectionEvidence('o%d'%i,i,i*.3,'cup','mug',.9,(1,1,20,20),Point3(2,0,.82)) for i in (1,2))
        claim=TargetClaim('c','cup','mug',1.,('o1','o2'))
        result=ProximityEvaluator(World()).evaluate(EvaluationRequest('e',claim,EvidenceBundle('e','c',detections)))
        self.assertFalse(result.valid_target)

    def test_known_target_approach_is_prioritized_and_keeps_its_type(self):
        from frontier_search.contracts import ObjectTrack,Point3,PolicyName
        from frontier_search.config import PolicyConfig
        from frontier_search.fixtures.loader import load_policy_fixture
        from frontier_search.policies.baselines import BaselinePolicy
        from frontier_search.runner.episode import build_context,decide
        f=load_policy_fixture()
        track=ObjectTrack('cup','mug',.9,Point3(3,3,1),0,1,('a','b'),2)
        second=replace(f.state.frontiers[1],kind='target_approach',target_track_id='cup',information_gain_m2=0)
        state=replace(f.state,target_class='mug',semantic_objects=(track,),frontiers=(f.state.frontiers[0],second)+f.state.frontiers[2:])
        p=BaselinePolicy(PolicyName.NEAREST);p.reset(state.episode_id,0,PolicyConfig())
        result=decide(p,state,build_context(state,f.context.evaluation))
        self.assertEqual(result.subgoal.frontier_id,second.frontier_id)
        self.assertEqual(result.subgoal.kind,'target_approach')

    def test_local_recovery_does_not_poison_a_remote_opportunity(self):
        from frontier_search.config import RecoveryConfig
        from frontier_search.fixtures.loader import load_policy_fixture
        from frontier_search.runner.recovery import SpatialRecovery
        from frontier_search.contracts import Pose
        fixture=load_policy_fixture();state=fixture.state;f=state.frontiers[0]
        recovery=SpatialRecovery(RecoveryConfig(opportunity_attempt_cap=1),locality_radius_m=1.2)
        recovery.reset(state.episode_id);recovery.record_dispatch(state,f.frontier_id)
        shifted=replace(f,approach_pose=Pose(f.approach_pose.x_m+3,f.approach_pose.y_m,0,0))
        old={e.frontier_id for e in recovery.exclusions(state,fixture.context.evaluation)}
        remote=replace(state,frontiers=(shifted,)+state.frontiers[1:])
        new={e.frontier_id for e in recovery.exclusions(remote,fixture.context.evaluation)}
        self.assertIn(f.frontier_id,old)
        self.assertNotIn(f.frontier_id,new)
        self.assertIn(f.frontier_id,{e.frontier_id for e in recovery.exclusions(state,fixture.context.evaluation)})





if __name__=='__main__':unittest.main()
