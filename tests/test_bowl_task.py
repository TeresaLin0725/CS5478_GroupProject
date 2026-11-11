"""Bowl identity, full initial scan, and recording isolation from the mug baseline."""
from dataclasses import replace
import json
from math import pi
from types import SimpleNamespace
import unittest

from frontier_search.config import EvaluationConfig
from frontier_search.contracts import (DetectionEvidence,EvidenceBundle,EvaluationRequest,
    Point3,Pose,TargetClaim)
from frontier_search.evaluation.retained import GroundTruthFrame,RetainedFrames,VisibleInstance
from frontier_search.runner.integrated import (recording_validation_matches,runtime_configuration,
    shared_round_manifest_for,task_manifest_for)
from frontier_search.simulation.integrated_rs import CloseRetainedEvaluator,RetainingAdapter
from frontier_search.simulation.igibson_adapter import AdapterConfig
from frontier_search.simulation.rs_tasks import KITCHEN_BOWL,LEGACY_CUP
from frontier_search.validation import ContractError


class BowlTaskTests(unittest.TestCase):
    def test_fixed_start_preserves_the_original_full_initial_scan(self):
        bowl=task_manifest_for('fixed','nearest','kitchen_bowl')
        bowl.validate()
        self.assertEqual(bowl.target_class,'bowl')
        self.assertEqual(bowl.start_pose,Pose(.8,-2.4,0.,pi/2))
        self.assertTrue(bowl.observation.allow_full_rotation)
        self.assertEqual(bowl.observation,replace(shared_round_manifest_for('fixed').observation,
                                               inspection_heading_settle_ratio=.5))

    def test_legacy_task_keeps_the_baseline_and_rejects_unknown_task(self):
        self.assertEqual(task_manifest_for('legacy'),shared_round_manifest_for('legacy'))
        with self.assertRaises(ValueError):
            task_manifest_for('bad',task='unknown')

    def test_simulation_factory_carries_bowl_without_exposing_position_to_manifest(self):
        manifest=task_manifest_for('bowl',task='kitchen_bowl')
        adapter=RetainingAdapter(manifest.robot,manifest.mapping,AdapterConfig(),RetainedFrames(),
                                 'apartment',KITCHEN_BOWL)
        backend=adapter._factory(AdapterConfig())
        self.assertEqual(backend.target_spec,KITCHEN_BOWL)
        self.assertFalse(hasattr(manifest,'target_position'))

    def test_recording_gate_binds_target_start_and_observation(self):
        manifest=task_manifest_for('bowl','nearest','kitchen_bowl')
        base=runtime_configuration(manifest,'apartment',None,KITCHEN_BOWL)
        certificate=dict(passed=True,source_fingerprint='source',profile='apartment',policy='nearest',
                         runtime_configuration=json.loads(json.dumps(base)))
        for actual in (
            runtime_configuration(manifest,'apartment',None,LEGACY_CUP),
            runtime_configuration(replace(manifest,start_pose=replace(manifest.start_pose,yaw_rad=0.)),
                                  'apartment',None,KITCHEN_BOWL),
            runtime_configuration(replace(manifest,observation=replace(manifest.observation,allow_full_rotation=False)),
                                  'apartment',None,KITCHEN_BOWL)):
            self.assertFalse(recording_validation_matches(certificate,'source','apartment','nearest',actual))

    def test_independent_evaluator_accepts_bowl_and_rejects_wrong_class(self):
        source=RetainedFrames()
        motions={}
        evidence=[]
        for seq in (1,2):
            position=Point3(.7,0.,.9)
            source.retain(GroundTruthFrame('bowl',seq,float(seq),100,100,
                (VisibleInstance('real-bowl','bowl',1.,(10,10,30,40),position),)))
            motions[seq]=dict(pose=Pose(0.,0.,0.,0.),linear_mps=0.,angular_radps=0.)
            evidence.append(DetectionEvidence('d%d'%seq,seq,float(seq),'track','bowl',.9,(10,10,30,40),position))
        adapter=SimpleNamespace(capture_motion=motions,_v=0.,_w=0.)
        claim=TargetClaim('claim','track','bowl',2.,('d1','d2'))
        request=EvaluationRequest('bowl',claim,EvidenceBundle('bowl','claim',tuple(evidence)))
        evaluator=CloseRetainedEvaluator(source,adapter,target_class='bowl')
        evaluator.reset('bowl',EvaluationConfig())
        self.assertTrue(evaluator.evaluate(request).valid_target)
        legacy=CloseRetainedEvaluator(source,adapter)
        legacy.reset('bowl',EvaluationConfig())
        with self.assertRaises(ContractError):
            legacy.evaluate(request)


if __name__=='__main__':
    unittest.main()
