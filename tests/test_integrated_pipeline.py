"""Cross-module regressions for native integration and conservative dynamic updates."""
from dataclasses import replace
import unittest
from unittest.mock import patch,Mock

from frontier_search.contracts import Point3
from frontier_search.perception.mapper import RGBDMapper
from frontier_search.perception.detection import RGBDetection
from frontier_search.perception.fusion import fuse_observation
from frontier_search.config import MappingConfig
from test_mapper_update import request,scene_frame


class InspectionTests(unittest.TestCase):
    def test_metric_size_prior_rejects_large_furniture_shaped_like_a_cup(self):
        self.mapper.reset(request(inspection_distance_m=.9,target_max_extent_m=.25))
        self.detector.detect.return_value=(RGBDetection('cup',.9,(0,0,60,60)),)
        with patch('frontier_search.perception.mapper.localize_detection',return_value=Point3(.7,0,.5)):
            state=self.mapper.update(scene_frame())
        self.assertFalse(state.semantic_objects)

    def setUp(self):
        self.detector=Mock()
        self.detector.detect.return_value=(RGBDetection('cup',.95,(28,28,33,33)),)
        self.mapper=RGBDMapper(self.detector)
        self.mapper.reset(request(inspection_distance_m=.9))

    def update(self,seq,time,distance=.7,x=0.,odometry=None):
        frame=replace(scene_frame(sequence=seq),timestamp_s=time)
        frame=replace(frame,robot_pose=replace(frame.robot_pose,x_m=x))
        if odometry is not None:
            frame=replace(frame,odometry_linear_speed_mps=odometry[0],odometry_angular_speed_radps=odometry[1])
        with patch('frontier_search.perception.tracking.localize_detection',return_value=Point3(distance,0,.5)):
            return self.mapper.update(frame)

    def test_far_visual_confirmation_cannot_terminate(self):
        for i in range(1,5):self.assertIsNone(self.update(i,float(i),distance=2.).target_claim)

    def test_moving_capture_and_duplicate_time_cannot_confirm(self):
        self.update(1,1.)
        self.update(2,2.,x=.3)
        self.assertIsNone(self.update(3,2.,x=.3).target_claim)
        self.assertIsNone(self.update(4,3.,x=.3).target_claim)
        self.assertIsNotNone(self.update(5,4.,x=.3).target_claim)

    def test_close_stationary_distinct_frames_confirm(self):
        self.update(1,1.)
        self.assertIsNone(self.update(2,2.).target_claim)
        state=self.update(3,3.)
        self.assertIsNotNone(state.target_claim)
        self.assertEqual([d.capture_sequence for d in self.mapper.get_claim_evidence(
            state.episode_id,state.target_claim.claim_id).detections],[2,3])

    def test_control_step_turn_is_rejected_even_when_camera_poses_look_stationary(self):
        self.update(1,1.,odometry=(0.,0.))
        self.assertIsNone(self.update(2,2.,odometry=(0.,.12)).target_claim)
        self.assertIsNone(self.update(3,3.,odometry=(0.,0.)).target_claim)
        state=self.update(4,4.,odometry=(0.,0.))
        self.assertIsNotNone(state.target_claim)
        self.assertEqual([d.capture_sequence for d in self.mapper.get_claim_evidence(
            state.episode_id,state.target_claim.claim_id).detections],[3,4])


class PhysicalScanTests(unittest.TestCase):
    def test_arrival_braking_drift_preserves_yaw_acquisition_but_not_false_success(self):
        from math import pi
        from test_navigation_b import make_navigator,goal_of
        from frontier_search.fixtures.loader import load_navigation_fixture
        from frontier_search.contracts import NavigationRequest,NavigationStatus
        fixture=load_navigation_fixture('open_space')
        goal=replace(goal_of(fixture),approach_pose=replace(fixture.goal_pose,yaw_rad=pi/2))
        sim,nav=make_navigator(fixture)
        nav._robot=replace(nav._robot,position_tolerance_m=.03)
        nav.start(NavigationRequest('E','g',fixture.grid,fixture.start_pose,goal))
        nav._active.index=len(nav._active.path)-1
        # Enter the position capture band, then replay native-like braking drift.
        sim._pose=replace(goal.approach_pose,x_m=goal.approach_pose.x_m-.014,yaw_rad=0.)
        nav.on_step()
        self.assertEqual(sim._v,0.)
        self.assertGreater(sim._w,0.)
        sim._pose=replace(sim._pose,x_m=goal.approach_pose.x_m-.025)
        nav.on_step()
        self.assertEqual(sim._v,0.)
        self.assertGreater(sim._w,0.)
        # Drift beyond the declared tolerance must cause correction, not success.
        sim._pose=replace(goal.approach_pose,x_m=goal.approach_pose.x_m-.035)
        nav.on_step()
        self.assertNotEqual(nav.poll('E','g').status,NavigationStatus.REACHED)
        sim._pose=replace(goal.approach_pose,x_m=goal.approach_pose.x_m-.01)
        nav.on_step()
        self.assertEqual(nav.poll('E','g').status,NavigationStatus.REACHED)
        self.assertEqual((sim._v,sim._w),(0.,0.))

    def test_physical_contact_stops_and_reports_blocked_on_the_same_tick(self):
        from test_navigation_b import make_navigator,goal_of
        from frontier_search.fixtures.loader import load_navigation_fixture
        from frontier_search.contracts import NavigationRequest,NavigationStatus,FailureReason
        fixture=load_navigation_fixture('open_space')
        sim,nav=make_navigator(fixture)
        nav.start(NavigationRequest('E','g',fixture.grid,fixture.start_pose,goal_of(fixture)))
        sim.contact_this_step=True
        nav.on_step()
        feedback=nav.poll('E','g')
        self.assertEqual((feedback.status,feedback.failure_reason),
                         (NavigationStatus.FAILED,FailureReason.BLOCKED))
        pose=sim.current_pose();sim.step();self.assertEqual(sim.current_pose(),pose)

    def test_inertial_drift_restarts_dwell_without_a_false_scan_mark(self):
        from test_navigation_b import make_navigator
        from frontier_search.fixtures.loader import load_navigation_fixture
        from frontier_search.contracts import Pose,ObservationRequest
        fixture=load_navigation_fixture('open_space')
        start=Pose(2.05,2.05,0.,0.)
        sim,nav=make_navigator(fixture,start=start)
        nav._obs_cfg=replace(nav._obs_cfg,arrival_headings_rad=(0.,),dwell_s=.5)
        nav.start_observation(ObservationRequest('E','scan',fixture.grid,start,'arrival',0.))
        nav.on_step()
        op=nav._active
        self.assertIsNotNone(op.dwell_until)
        original_pose=sim.current_pose
        sim.current_pose=lambda:replace(original_pose(),yaw_rad=.12)
        nav._tick_observation(op,1.)
        self.assertIsNone(op.dwell_until)
        self.assertEqual(nav.capture_marks('scan'),())


class DetectorTileTests(unittest.TestCase):
    def test_fixed_rgb_tiles_translate_boxes_and_suppress_duplicates(self):
        from frontier_search.perception.detection import TiledYOLODetector,YOLODetector
        from frontier_search.sensors import ImageBuffer
        detector=TiledYOLODetector.__new__(TiledYOLODetector)
        detector.target_class='cup'
        def infer(self,image):
            cup=RGBDetection('cup',.9,(1,1,3,3))
            return (cup,RGBDetection('chair' if image.width==12 else 'dog',.8,(0,0,2,2)))
        with patch.object(YOLODetector,'detect',infer):
            detections=detector.detect(ImageBuffer(12,8,'rgb8',bytes(12*8*3)))
        self.assertEqual(sum(d.class_name=='cup' for d in detections),6)
        self.assertEqual(sum(d.bbox_xyxy_px==(1,1,3,3) for d in detections),1)
        self.assertIn('chair',{d.class_name for d in detections})
        self.assertNotIn('dog',{d.class_name for d in detections})

    def test_tile_decimation_never_reuses_boxes_from_an_old_image(self):
        from frontier_search.perception.detection import TiledYOLODetector,YOLODetector
        from frontier_search.sensors import ImageBuffer
        detector=TiledYOLODetector.__new__(TiledYOLODetector)
        detector.target_class='cup';detector.tile_every_frames=3
        def infer(self,image):
            return (RGBDetection('cup',.9,(1,1,3,3)),) if image.width<12 else ()
        with patch.object(YOLODetector,'detect',infer):
            image=ImageBuffer(12,8,'rgb8',bytes(12*8*3))
            self.assertTrue(detector.detect(image))
            self.assertEqual(detector.detect(image),())
            self.assertEqual(detector.detect(image),())
            self.assertTrue(detector.detect(image))


class RecoveryViewpointTests(unittest.TestCase):
    def test_one_localized_view_can_propose_inspection_but_cannot_claim_success(self):
        from frontier_search.perception.mapper import _target_approaches
        from frontier_search.contracts import OccupancyGrid,ObjectTrack
        req=request(inspection_distance_m=.9)
        cfg=req.mapping_config
        grid=OccupancyGrid('world',1,cfg.resolution_m,cfg.origin_x_m,cfg.origin_y_m,
            cfg.width_cells,cfg.height_cells,tuple((0,)*cfg.width_cells for _ in range(cfg.height_cells)))
        track=ObjectTrack('tentative','cup',.9,Point3(2.,0.,.9),1.,1.,('single',),1)
        proposals=_target_approaches(grid,scene_frame(),{'tentative':track},'cup',cfg,req.robot_config)
        self.assertTrue(proposals)
        self.assertTrue(all(p.kind=='target_approach' and p.target_track_id=='tentative' for p in proposals))
        after_scan=replace(scene_frame(),timestamp_s=7.)
        self.assertTrue(_target_approaches(grid,after_scan,{'tentative':track},'cup',cfg,req.robot_config))
        old=replace(scene_frame(),timestamp_s=20.)
        self.assertFalse(_target_approaches(grid,old,{'tentative':track},'cup',cfg,req.robot_config))
        confirmed=replace(track,confirmation_count=2)
        self.assertTrue(_target_approaches(grid,old,{'tentative':confirmed},'cup',cfg,req.robot_config))

    def test_alternate_known_free_viewpoints_are_only_eligible_during_recovery(self):
        from frontier_search.fixtures.loader import load_policy_fixture
        from frontier_search.runner.episode import build_context
        from frontier_search.contracts import FailureReason
        f=load_policy_fixture()
        probes=replace(f.state.frontiers[1],kind='navigation_probe')
        state=replace(f.state,frontiers=(f.state.frontiers[0],probes)+f.state.frontiers[2:])
        evaluation=replace(f.context.evaluation,assessments=tuple(replace(a,reachable=True,
            path_cost_m=1.,failure_reason=None) for a in f.context.evaluation.assessments))
        context=build_context(state,evaluation)
        self.assertNotIn(probes.frontier_id,context.eligible_frontier_ids)
        blocked=replace(evaluation,assessments=tuple(a if a.frontier_id==probes.frontier_id else
            replace(a,reachable=False,path_cost_m=None,failure_reason=FailureReason.NO_PATH)
            for a in evaluation.assessments))
        self.assertEqual(build_context(state,blocked).eligible_frontier_ids,(probes.frontier_id,))


class IndependentCloseEvaluationTests(unittest.TestCase):
    def evaluate(self,distance=0.7,speed=0.,times=(1.,2.)):
        from types import SimpleNamespace
        from frontier_search.evaluation.retained import RetainedFrames,GroundTruthFrame,VisibleInstance
        from frontier_search.simulation.integrated_rs import CloseRetainedEvaluator
        from frontier_search.config import EvaluationConfig
        from frontier_search.contracts import Pose,DetectionEvidence,TargetClaim,EvidenceBundle,EvaluationRequest
        store=RetainedFrames();motions={};items=[]
        for seq,time in enumerate(times,1):
            store.retain(GroundTruthFrame('e',seq,time,100,100,
                (VisibleInstance('real-mug','cup',1.,(10,10,30,40),Point3(distance,0,.9)),)))
            motions[seq]=dict(pose=Pose(0,0,0,0),linear_mps=speed,angular_radps=0.,command_stopped=True)
            items.append(DetectionEvidence('d%d'%seq,seq,time,'t','cup',.95,(10,10,30,40),Point3(distance,0,.9)))
        adapter=SimpleNamespace(capture_motion=motions,_v=0.,_w=0.)
        evaluator=CloseRetainedEvaluator(store,adapter)
        evaluator.reset('e',EvaluationConfig())
        claim=TargetClaim('c','t','cup',max(times),tuple(d.observation_id for d in items))
        return evaluator.evaluate(EvaluationRequest('e',claim,EvidenceBundle('e','c',tuple(items))))

    def test_near_stationary_independent_frames_pass(self):
        self.assertTrue(self.evaluate().valid_target)

    def test_w3_visible_identity_is_insufficient_when_far_moving_or_same_time(self):
        for kwargs in (dict(distance=2.),dict(speed=.1),dict(times=(1.,1.))):
            with self.subTest(kwargs=kwargs):self.assertFalse(self.evaluate(**kwargs).valid_target)


class DynamicClearingTests(unittest.TestCase):
    def setUp(self):
        self.cfg=MappingConfig(resolution_m=1.,width_cells=3,height_cells=1,
            floor_tolerance_m=.03,clearance_bin_height_m=.5,dynamic_clear_frames=2)
        self.occ=[[0,100,-1]];self.cov=[[False]*3];self.masks=[[0]*3]
        self.floors=[[False]*3];self.counts=[[0]*3]

    def fuse(self,points,voxels):
        with patch('frontier_search.perception.fusion.iter_depth_world',return_value=iter(points)),patch(
            'frontier_search.perception.fusion._ray_voxels',return_value=iter(voxels)):
            fuse_observation(scene_frame(),self.cfg,(0.,1.),self.occ,self.cov,self.masks,self.floors,self.counts)

    def test_clear_requires_two_complete_current_floor_and_column_frames(self):
        floor=(0,0,2.,1.5,.5,0.)
        self.fuse([floor],[(1,0,0)])
        self.assertEqual(self.occ[0][1],100)
        self.fuse([floor],[(1,0,0),(1,0,1)])
        self.assertEqual(self.occ[0][1],100)
        self.fuse([floor],[(1,0,0),(1,0,1)])
        self.assertEqual(self.occ[0][1],0)

    def test_any_current_obstacle_hit_wins_over_old_free_masks(self):
        floor=(0,0,2.,1.5,.5,0.);hit=(0,0,2.,1.5,.5,.4)
        for _ in range(3):self.fuse([floor,hit],[(1,0,0),(1,0,1)])
        self.assertEqual(self.occ[0][1],100)

    def test_unobserved_frame_breaks_clearing_streak(self):
        floor=(0,0,2.,1.5,.5,0.)
        self.fuse([floor],[(1,0,0),(1,0,1)])
        self.fuse([],[])
        self.fuse([floor],[(1,0,0),(1,0,1)])
        self.assertEqual(self.occ[0][1],100)

    def test_same_timestamp_cannot_increment_clearing_evidence(self):
        floor=(0,0,2.,1.5,.5,0.)
        self.fuse([floor],[(1,0,0),(1,0,1)])
        with patch('frontier_search.perception.fusion.iter_depth_world',return_value=iter([floor])),patch(
            'frontier_search.perception.fusion._ray_voxels',return_value=iter([(1,0,0),(1,0,1)])):
            fuse_observation(scene_frame(),self.cfg,(0.,1.),self.occ,self.cov,self.masks,self.floors,
                             self.counts,increment_clear_count=False)
        self.assertEqual(self.occ[0][1],100)
        self.assertEqual(self.counts[0][1],1)
        self.fuse([floor],[(1,0,0),(1,0,1)])
        self.assertEqual(self.occ[0][1],0)
