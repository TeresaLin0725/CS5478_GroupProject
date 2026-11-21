"""Noise distribution, integration, frozen settings, and evaluator isolation."""
import argparse
from contextlib import redirect_stdout
from dataclasses import asdict, replace
import io
import json
from pathlib import Path
import random
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts'))
from frontier_search.config import MappingConfig, SensorNoiseConfig
from frontier_search.fixtures.analytic_rgbd import render_frame
from frontier_search.runner import integrated
from frontier_search.runner.manifest import load_manifest
from frontier_search.simulation.sensor_noise import DepthNoise, add_noise_arguments, noise_from_arguments
from frontier_search.simulation.igibson_adapter import AdapterConfig, IGibsonAdapter
from frontier_search.simulation.integrated_rs import RetainingAdapter
from tests.integration.test_w2_runner import make_runner
import run_paired_benchmark as benchmark


def observation(width=100, height=100):
    obs = render_frame(width=width, height=height)
    return replace(obs, depth_image=replace(obs.depth_image,
                   data=np.full((height, width), 2., dtype='<f4').tobytes()))


def depths(obs):
    return np.frombuffer(obs.depth_image.data, dtype='<f4')


class DepthNoiseTests(unittest.TestCase):
    def test_disabled_is_identity_and_does_not_touch_global_rng(self):
        obs = observation()
        for enabled in (False, True):
            noise = DepthNoise(SensorNoiseConfig(enabled=enabled), MappingConfig())
            py_state, np_state = random.getstate(), np.random.get_state()
            result = noise.apply(obs)
            self.assertEqual(py_state, random.getstate())
            current = np.random.get_state()
            np.testing.assert_array_equal(np_state[1], current[1])
            self.assertEqual(np_state[2:], current[2:])
            if not enabled:
                self.assertIs(result, obs)
                self.assertEqual(noise.stats['captures'], 0)

    def test_seed_sequence_repeatability_and_policy_independence(self):
        obs = observation()
        cfg = SensorNoiseConfig(enabled=True, seed=71)
        noise = DepthNoise(cfg, MappingConfig())
        first = noise.apply(obs)
        self.assertEqual(first, DepthNoise(cfg, MappingConfig()).apply(obs))
        self.assertEqual(first, noise.apply(obs))
        other_policy = noise.apply(replace(obs, episode_id='another-policy'))
        self.assertEqual(first.depth_image, other_policy.depth_image)
        self.assertNotEqual(first.depth_image, noise.apply(replace(obs, sequence=2)).depth_image)
        self.assertNotEqual(first.depth_image,
                            DepthNoise(replace(cfg, seed=72), MappingConfig()).apply(obs).depth_image)
        self.assertEqual(replace(first, depth_image=obs.depth_image), obs)
        self.assertTrue(np.all(depths(obs) == 2.))

    def test_gaussian_distribution_matches_requested_level(self):
        obs = observation(400, 250)
        cfg = SensorNoiseConfig(enabled=True, depth_std_m=.03, depth_dropout_prob=0)
        delta = depths(DepthNoise(cfg, MappingConfig()).apply(obs)) - depths(obs)
        self.assertLess(abs(float(delta.mean())), .0005)
        self.assertAlmostEqual(float(delta.std()), .03, delta=.0005)

    def test_dropout_rate_and_ablation_streams(self):
        obs = observation(400, 250)
        cfg = SensorNoiseConfig(enabled=True, depth_std_m=0, depth_dropout_prob=.05)
        only = DepthNoise(cfg, MappingConfig()).apply(obs)
        both = DepthNoise(replace(cfg, depth_std_m=.01), MappingConfig()).apply(obs)
        np.testing.assert_array_equal(depths(only) == 0, depths(both) == 0)
        self.assertAlmostEqual(float(np.mean(depths(only) == 0)), .05, delta=.003)
        self.assertTrue(np.all(depths(only)[depths(only) != 0] == 2.))
        gaussian = DepthNoise(replace(cfg, depth_std_m=.01, depth_dropout_prob=0), MappingConfig()).apply(obs)
        kept = depths(both) != 0
        np.testing.assert_array_equal(depths(both)[kept], depths(gaussian)[kept])

    def test_invalid_inputs_are_preserved_and_new_outliers_become_invalid(self):
        obs = observation(8, 1)
        values = np.array([0., -1., np.nan, np.inf, .01, 9., 1., 2.], dtype='<f4')
        obs = replace(obs, depth_image=replace(obs.depth_image, data=values.tobytes()))
        cfg = SensorNoiseConfig(enabled=True, depth_std_m=0, depth_dropout_prob=1)
        result = DepthNoise(cfg, MappingConfig()).apply(obs)
        self.assertEqual(result.depth_image.data[:24], obs.depth_image.data[:24])
        np.testing.assert_array_equal(depths(result)[6:], [0., 0.])
        noise = DepthNoise(replace(cfg, depth_std_m=100., depth_dropout_prob=0), MappingConfig())
        result = noise.apply(observation())
        self.assertGreater(noise.stats['out_of_range_pixels'], 9000)
        self.assertTrue(np.all((depths(result) == 0) | ((depths(result) >= .1) & (depths(result) <= 5.))))

    def test_zero_settings_identity_and_stats(self):
        obs = observation()
        noise = DepthNoise(SensorNoiseConfig(enabled=True, depth_std_m=0, depth_dropout_prob=0), MappingConfig())
        self.assertIs(noise.apply(obs), obs)
        self.assertEqual(noise.stats, dict(captures=1, valid_pixels=10000, dropout_pixels=0,
                                         out_of_range_pixels=0, modified_pixels=0))

    def test_native_retained_truth_and_motion_are_clean(self):
        manifest = integrated.shared_round_manifest_for('native-noise')
        obs = observation()
        store = Mock()
        adapter = RetainingAdapter(manifest.robot, manifest.mapping, AdapterConfig(), store)
        backend = Mock()
        backend._bodies = (1,)
        backend._p.getBaseVelocity.return_value = ((0., 0., 0.), (0., 0., 0.))
        backend.truth.return_value = 'clean-truth'
        adapter._backend = backend
        with patch.object(IGibsonAdapter, 'capture', return_value=obs):
            captured = adapter.capture()
        noisy = DepthNoise(SensorNoiseConfig(enabled=True, depth_std_m=0, depth_dropout_prob=1),
                           manifest.mapping).apply(captured)
        backend.truth.assert_called_once_with(obs)
        store.retain.assert_called_once_with('clean-truth')
        self.assertEqual(adapter.capture_motion[obs.sequence]['pose'], obs.robot_pose)
        self.assertTrue(np.all(depths(noisy) == 0))
        self.assertTrue(np.all(depths(captured) == 2.))


class NoiseConfigurationTests(unittest.TestCase):
    def parse(self, argv):
        parser = argparse.ArgumentParser()
        add_noise_arguments(parser)
        return noise_from_arguments(parser.parse_args(argv))

    def test_cli_defaults_switches_and_independent_components(self):
        self.assertEqual(self.parse([]), SensorNoiseConfig())
        self.assertTrue(self.parse(['--noise']).enabled)
        self.assertFalse(self.parse(['--depth-noise-std-m', '.03']).enabled)
        self.assertEqual(self.parse(['--noise', '--depth-noise-std-m', '0',
                                    '--depth-dropout-prob', '.05', '--noise-seed', '12']),
                         SensorNoiseConfig(True, 0., .05, 12))

    def test_json_overrides_and_explicit_disable(self):
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / 'noise.json'
            path.write_text(json.dumps(dict(enabled=True, depth_std_m=.03, seed=9)), encoding='utf-8')
            self.assertEqual(self.parse(['--noise-config', str(path), '--no-noise', '--noise-seed', '10']),
                             SensorNoiseConfig(False, .03, .01, 10))
            for invalid in ({'depth_st_m': .03}, [], {'enabled': 'true'}, {'seed': None}):
                path.write_text(json.dumps(invalid), encoding='utf-8')
                with self.assertRaises(ValueError):
                    self.parse(['--noise-config', str(path)])

    def test_invalid_configuration_rejected_even_when_off(self):
        bad = [dict(enabled=1), dict(seed=True), dict(seed=-1), dict(seed=2**32),
               dict(seed=1.5), dict(depth_std_m=True), dict(depth_std_m=-.01),
               dict(depth_std_m=float('inf')), dict(depth_std_m=float('nan')),
               dict(depth_dropout_prob=True), dict(depth_dropout_prob=-.1),
               dict(depth_dropout_prob=1.1), dict(depth_dropout_prob=float('nan'))]
        for fields in bad:
            with self.subTest(fields=fields), self.assertRaises(ValueError):
                SensorNoiseConfig(**fields)

    def test_manifest_roundtrip_and_legacy_default(self):
        manifest = replace(integrated.shared_round_manifest_for('roundtrip'),
                           sensor_noise=SensorNoiseConfig(True, .03, .05, 123))
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / 'manifest.json'
            data = asdict(manifest)
            path.write_text(json.dumps(data), encoding='utf-8')
            self.assertEqual(load_manifest(path), manifest)
            del data['sensor_noise']
            path.write_text(json.dumps(data), encoding='utf-8')
            self.assertEqual(load_manifest(path).sensor_noise, SensorNoiseConfig())

    def test_recording_gate_rejects_any_noise_setting_change(self):
        manifest = integrated.shared_round_manifest_for('gate')
        config = integrated.runtime_configuration(manifest, 'dynamic', None)
        report = dict(passed=True, source_fingerprint='source', profile='dynamic',
                      policy=manifest.policy_name, runtime_configuration=json.loads(json.dumps(config)))
        self.assertTrue(integrated.recording_validation_matches(report, 'source', 'dynamic', manifest.policy_name, config))
        for changed in (dict(enabled=True), dict(depth_std_m=.02), dict(depth_dropout_prob=.02), dict(seed=1)):
            actual = integrated.runtime_configuration(replace(manifest,
                sensor_noise=replace(manifest.sensor_noise, **changed)), 'dynamic', None)
            self.assertFalse(integrated.recording_validation_matches(report, 'source', 'dynamic', manifest.policy_name, actual))
        del report['runtime_configuration']['sensor_noise']
        self.assertFalse(integrated.recording_validation_matches(report, 'source', 'dynamic', manifest.policy_name, config))

    def test_frozen_run_rejects_noise_overrides_before_launch(self):
        with patch.object(sys, 'argv', ['benchmark', 'run', '--output', 'unused', '--noise']), \
                patch.object(benchmark, 'run') as run, redirect_stdout(io.StringIO()), \
                patch('sys.stderr', new_callable=io.StringIO):
            with self.assertRaises(SystemExit) as error:
                benchmark.main()
        self.assertEqual(error.exception.code, 2)
        run.assert_not_called()

    def test_direct_entry_applies_cli_to_manifest_before_startup(self):
        class Configured(Exception):
            pass
        with patch.object(sys, 'argv', ['integrated', '--output', 'unused', '--noise',
                                       '--depth-noise-std-m', '.03', '--noise-seed', '19']), \
                patch.object(integrated, 'source_fingerprint', return_value='fixture'), \
                patch.object(integrated, 'load_local_env'), \
                patch.object(integrated, 'runtime_configuration', side_effect=Configured) as runtime:
            with self.assertRaises(Configured):
                integrated.main()
        self.assertEqual(runtime.call_args.args[0].sensor_noise, SensorNoiseConfig(True, .03, .01, 19))

    def test_result_integrity_rejects_noise_mismatch_and_exports_settings(self):
        cfg = asdict(SensorNoiseConfig(True, .03, .05, 17))
        with tempfile.TemporaryDirectory() as td:
            out = Path(td)
            job = benchmark.task_specs()[0]
            job.update(sensor_noise=cfg, manifest_path='manifest.json')
            manifest = dict(sensor_noise=cfg)
            (out / 'manifest.json').write_text(json.dumps(manifest), encoding='utf-8')
            folder = out / 'episodes' / job['job_id']
            folder.mkdir(parents=True)
            (folder / 'events.jsonl').write_text(json.dumps(dict(event='episode_start', manifest=manifest)), encoding='utf-8')
            summary = dict(termination='search_limit', source_fingerprint='fixture', source_unchanged=True,
                           sensor_noise=cfg, sensor_noise_stats={'captures': 1})
            protocol = dict(engine_fingerprint='fixture', base_manifest={'recovery': {'minimum_progress_m2': .1}})
            (folder / 'summary.json').write_text(json.dumps(summary), encoding='utf-8')
            row = benchmark.result_row(out, job, protocol)
            self.assertTrue(row['integrity_checks']['sensor_noise_configuration'])
            self.assertEqual(row['termination'], 'search_limit')
            self.assertEqual(row['noise_seed'], 17)
            self.assertEqual(row['depth_noise_std_m'], .03)
            summary['sensor_noise'] = dict(cfg, enabled=False)
            (folder / 'summary.json').write_text(json.dumps(summary), encoding='utf-8')
            self.assertEqual(benchmark.result_row(out, job, protocol)['termination'], 'integrity_error')

    def test_freeze_persists_paired_noise_settings_and_labels_report(self):
        cfg = SensorNoiseConfig(True, .03, .05, 1000)
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            (root / 'src').mkdir()
            (root / '.env').write_text('', encoding='utf-8')
            weights = root / 'weights/yolo11/yolo11s.pt'
            weights.parent.mkdir(parents=True)
            weights.write_bytes(b'fixture')
            cert = root / 'report/evidence/demo/checks-multiview/tests.json'
            cert.parent.mkdir(parents=True)
            cert.write_text(json.dumps(dict(returncode=0, source_unchanged=True, source_fingerprint='fixture')), encoding='utf-8')
            out = root / 'suite'
            with patch.object(benchmark, 'ROOT', root), patch.object(benchmark, 'HARNESSES', ()), \
                    patch.object(integrated, 'source_fingerprint', return_value='fixture'), \
                    patch('frontier_search.policies.provider.load_local_env'), redirect_stdout(io.StringIO()):
                benchmark.freeze(out, 1, sensor_noise=cfg)
            protocol = json.loads((out / 'protocol.json').read_text())
            self.assertEqual(protocol['sensor_noise'], asdict(cfg))
            for job in protocol['jobs']:
                expected = replace(cfg, seed=1000 + job['environment_seed'])
                self.assertEqual(load_manifest(out / job['manifest_path']).sensor_noise, expected)
                self.assertEqual(job['sensor_noise'], asdict(expected))
            self.assertEqual(benchmark.validate_pairing(protocol['jobs']), 9)
            self.assertIn('Sensor noise: ON', (out / 'report/index.html').read_text(encoding='utf-8'))
            self.assertEqual(json.loads((out / 'results.json').read_text())['sensor_noise'], asdict(cfg))
            self.assertFalse((root / 'configs/experiment.rs-20261018.json').exists())


class RunnerNoiseTests(unittest.TestCase):
    def test_runner_passes_noisy_copy_and_logs_configuration_and_counts(self):
        for enabled in (False, True):
            manifest, runner, logger = make_runner()
            manifest = replace(manifest, sensor_noise=SensorNoiseConfig(enabled, 0., 1., 91))
            raw, received = [], []
            capture, update = runner.simulator.capture, runner.mapper.update

            def capture_valid():
                obs = capture()
                obs = replace(obs, depth_image=replace(obs.depth_image, data=np.array([2.], dtype='<f4').tobytes()))
                raw.append(obs)
                return obs

            def observe(obs):
                received.append(obs)
                return update(obs)

            runner.simulator.capture = capture_valid
            runner.mapper.update = observe
            runner.cancelled = lambda: bool(received)
            runner.run(manifest)
            self.assertEqual(len(received), 1)
            self.assertEqual(depths(raw[0])[0], 2.)
            self.assertEqual(depths(received[0])[0], 0. if enabled else 2.)
            if not enabled:
                self.assertIs(received[0], raw[0])
            self.assertEqual(runner.summary['sensor_noise'], asdict(manifest.sensor_noise))
            self.assertEqual(runner.summary['sensor_noise_stats']['dropout_pixels'], int(enabled))
            self.assertEqual(next(e for e in logger.events if e['event']=='episode_start')['manifest'], manifest)


if __name__ == '__main__':
    unittest.main()
