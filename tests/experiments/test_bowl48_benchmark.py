"""Keep targets, starts, noise pairing and preflight routing correct in bowl48."""
from contextlib import redirect_stdout
from dataclasses import asdict
from io import StringIO
import json
from math import pi
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts'))
import run_paired_benchmark as benchmark
from benchmark_episode import target_for_protocol,preflight_manifest,episode
from benchmark_protocol import bowl48_task_specs,validate_pairing
from frontier_search.config import SensorNoiseConfig
from frontier_search.runner import integrated
from frontier_search.runner.manifest import load_manifest
from frontier_search.simulation.rs_tasks import KITCHEN_BOWL,BATHROOM_BOWL
from frontier_search.simulation.indoor_challenges import PROFILES,profile_props
from frontier_search.simulation.integrated_rs import RetainingAdapter
from frontier_search.simulation.igibson_adapter import AdapterConfig
from frontier_search.evaluation.retained import RetainedFrames


class Bowl48Tests(unittest.TestCase):
    def test_removed_experiment_modes_fail_before_starting_a_native_episode(self):
        for key in ('local_collision_refinement', 'local_collision_retain_history'):
            with self.subTest(feature=key), self.assertRaisesRegex(ValueError, 'not supported'):
                episode({key: True}, {}, Path('unused-output'))

    def test_accessible_layout_preserves_legacy_and_moves_only_doorway_chair(self):
        for profile in ('apartment','clutter','dynamic'):
            original=PROFILES[profile]['props']
            self.assertEqual(profile_props(profile),original)
            expected=tuple((kind,(-3.35,.85,0) if kind=='chair' and pos==(-2.7,1.,0) else pos,yaw)
                           for kind,pos,yaw in original)
            self.assertEqual(profile_props(profile,'bowl48_accessible'),expected)
            self.assertEqual(PROFILES[profile]['props'],original)
        with self.assertRaises(ValueError):profile_props('clutter','unknown')

    def test_adapter_factory_binds_layout_and_target(self):
        m=benchmark.benchmark_manifest('kitchen_bowl')
        for target in (KITCHEN_BOWL,BATHROOM_BOWL):
            adapter=RetainingAdapter(m.robot,m.mapping,AdapterConfig(),RetainedFrames(),
                                     'clutter',target,'bowl48_accessible')
            backend=adapter._factory(AdapterConfig())
            self.assertEqual(backend.scene_layout,'bowl48_accessible')
            self.assertEqual(backend.target_spec,target)
        adapter=RetainingAdapter(m.robot,m.mapping,AdapterConfig(),RetainedFrames())
        self.assertEqual(adapter._factory(AdapterConfig()).scene_layout,'legacy')

    def test_complete_pairing_kitchen_first_and_no_seed_variation(self):
        jobs=bowl48_task_specs()
        self.assertEqual(len(jobs),48)
        self.assertEqual(validate_pairing(jobs),12)
        self.assertEqual({j['target_location'] for j in jobs[:24]},{'kitchen'})
        self.assertEqual({j['target_location'] for j in jobs[24:]},{'bathroom'})
        self.assertEqual({j['environment_seed'] for j in jobs},{17})
        self.assertEqual({j['perception_seed'] for j in jobs},{10017})
        self.assertEqual({j['policy_seed'] for j in jobs},{20017})
        self.assertEqual({j['start_id'] for j in jobs},{'A','B'})
        self.assertEqual({(j['start_pose']['x_m'],j['start_pose']['y_m'],j['start_pose']['yaw_rad']) for j in jobs},
                         {(.8,-2.4,pi/2),(-1.5,.6,pi)})

    def test_freeze_seals_per_location_target_and_per_start_preflight(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);(root/'src').mkdir();(root/'.env').write_text('')
            weight=root/'weights/yolo11/yolo11s.pt';weight.parent.mkdir(parents=True);weight.write_bytes(b'fixture')
            asset=root/'data/igibson-data/robot_assets/assets/models/ycb/024_bowl/textured_simple.obj'
            asset.parent.mkdir(parents=True);asset.write_text('fixture')
            cert=root/'certificate.json';cert.write_text(json.dumps(dict(returncode=0,source_unchanged=True,source_fingerprint='fixture')))
            out=root/'suite'
            with patch.object(benchmark,'ROOT',root),patch.object(benchmark,'HARNESSES',()), \
                 patch.object(integrated,'source_fingerprint',return_value='fixture'), \
                 patch('frontier_search.policies.provider.load_local_env'),redirect_stdout(StringIO()):
                benchmark.freeze(out,2,sensor_noise=SensorNoiseConfig(True,.01,.01,123),
                    task='kitchen_bowl',tests_certificate=cert,design='bowl48')
            protocol=json.loads((out/'protocol.json').read_text(encoding='utf-8'))
            self.assertEqual(protocol['scene_layout'],'bowl48_accessible')
            self.assertEqual(len(protocol['preflight_cases']),12)
            for case in protocol['preflight_cases']:
                job=next(j for j in protocol['jobs'] if j['job_id']==case['job_id'])
                resolved=preflight_manifest(protocol,case['case_id'])
                self.assertEqual(asdict(resolved.start_pose),job['start_pose'])
                self.assertEqual(resolved.sensor_noise.seed,140)
            for job in protocol['jobs']:
                expected=KITCHEN_BOWL if job['target_location']=='kitchen' else BATHROOM_BOWL
                self.assertEqual(target_for_protocol(protocol,job),expected)
                manifest=load_manifest(out/job['manifest_path'])
                self.assertEqual(manifest.target_class,'bowl')
                self.assertEqual(manifest.mapping.depth_filter_kernel_size,5)
                self.assertTrue(manifest.mapping.unvisited_recovery_viewpoints)
                self.assertEqual(manifest.observation.inspection_heading_settle_ratio,.5)
                self.assertEqual(manifest.sensor_noise.seed,140)
                self.assertTrue(manifest.sensor_noise.enabled)
                self.assertNotIn('target_position',asdict(manifest))
            selected=protocol['preflight_cases'][-1]
            job=next(j for j in protocol['jobs'] if j['job_id']==selected['job_id'])
            (out/job['manifest_path']).write_text('{}')
            with self.assertRaisesRegex(ValueError,'manifest changed'):
                preflight_manifest(protocol,selected['case_id'])


if __name__=='__main__':unittest.main()
