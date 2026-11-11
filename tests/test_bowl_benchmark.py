"""The 36 bowl tasks share the restored start, target, and paired noise streams."""
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

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import run_paired_benchmark as benchmark
from benchmark_episode import target_for_protocol
from benchmark_protocol import task_specs,validate_pairing
from frontier_search.config import SensorNoiseConfig
from frontier_search.runner import integrated
from frontier_search.runner.manifest import load_manifest
from frontier_search.simulation.rs_tasks import KITCHEN_BOWL,LEGACY_CUP


class BowlBenchmarkTests(unittest.TestCase):
    def test_bowl_matches_validated_noisy_configuration_except_target(self):
        actual=benchmark.benchmark_manifest('kitchen_bowl')
        self.assertEqual(actual.mapping.depth_filter_kernel_size,5)
        self.assertEqual(actual.mapping.depth_filter_max_delta_m,.08)
        self.assertTrue(actual.mapping.unvisited_recovery_viewpoints)
        self.assertEqual(actual.observation.inspection_heading_settle_ratio,.5)

    def test_all_36_tasks_share_the_original_pose(self):
        pose=dict(x_m=.8,y_m=-2.4,z_m=0.,yaw_rad=pi/2)
        jobs=task_specs(pose)
        self.assertEqual(len(jobs),36)
        self.assertEqual(validate_pairing(jobs),9)
        self.assertTrue(all(j['start_pose']==pose for j in jobs))
        self.assertEqual({j['environment_seed'] for j in jobs},{17,29,43})
        self.assertEqual(len({j['start_pose']['yaw_rad'] for j in task_specs()}),3)

    def test_target_resolution_preserves_legacy_and_bowl_identity(self):
        self.assertEqual(target_for_protocol({}),LEGACY_CUP)
        for target in (LEGACY_CUP,KITCHEN_BOWL):
            protocol=dict(target_evaluator_only=dict(class_name=target.class_name,model=target.model,
                position_world=list(target.position),instance_id=target.instance_id))
            self.assertEqual(target_for_protocol(protocol),target)

    def test_freeze_uses_bowl_assets_and_the_requested_regression_certificate(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            (root/'src').mkdir()
            (root/'.env').write_text('')
            weights=root/'weights/yolo11/yolo11s.pt'
            weights.parent.mkdir(parents=True)
            weights.write_bytes(b'fixture')
            bowl=root/'data/igibson-data/robot_assets/assets/models/ycb/024_bowl/textured_simple.obj'
            bowl.parent.mkdir(parents=True)
            bowl.write_text('fixture')
            cert=root/'new-regression.json'
            cert.write_text(json.dumps(dict(returncode=0,source_unchanged=True,source_fingerprint='fixture')))
            out=root/'suite'
            noise=SensorNoiseConfig(True,.01,.01,123)
            with patch.object(benchmark,'ROOT',root),patch.object(benchmark,'HARNESSES',()), \
                 patch.object(integrated,'source_fingerprint',return_value='fixture'), \
                 patch('frontier_search.policies.provider.load_local_env'),redirect_stdout(StringIO()):
                benchmark.freeze(out,1,sensor_noise=noise,task='kitchen_bowl',tests_certificate=cert)
            protocol=json.loads((out/'protocol.json').read_text())
            self.assertEqual(target_for_protocol(protocol),KITCHEN_BOWL)
            self.assertTrue(protocol['fixed_start_heading'])
            self.assertEqual({x['yaw_rad'] for x in protocol['seed_heading_blocks']},{pi/2})
            self.assertTrue(any('/024_bowl/' in key for key in protocol['asset_hashes']))
            for job in protocol['jobs']:
                manifest=load_manifest(out/job['manifest_path'])
                self.assertEqual(manifest.target_class,'bowl')
                self.assertEqual(asdict(manifest.start_pose),dict(x_m=.8,y_m=-2.4,z_m=0.,yaw_rad=pi/2))
                self.assertEqual(manifest.sensor_noise.seed,123+job['environment_seed'])
                self.assertTrue(manifest.sensor_noise.enabled)


if __name__=='__main__':
    unittest.main()
