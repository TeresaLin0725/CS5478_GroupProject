"""Portable package integrity and offline A/B/C replay acceptance."""

from dataclasses import replace
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

from frontier_search.contracts import GridCell
from frontier_search.experiments.logging import dumps
from frontier_search.fixtures.sequence import load_sequence, write_sequence, validate_sequence
from frontier_search.runner.replay import AssessmentControl, generate_synthetic, pack_recordings, replay
from frontier_search.validation import ContractError


CONFIG = Path(__file__).resolve().parents[2] / 'configs' / 'w2.rgbd-synthetic.json'


class RGBDSequenceTests(unittest.TestCase):
    def test_configured_base_margin_preserves_real_pose_and_rejects_excess(self):
        data = load_sequence(self.dataset)
        frames = list(data.observations)
        frames[1] = replace(frames[1], robot_pose=replace(frames[1].robot_pose, z_m=0.003))
        with self.assertRaisesRegex(ContractError, 'base elevation'):
            validate_sequence(data.manifest, frames, data.initial_clearance_cells, data.clearance_provenance)
        manifest = replace(data.manifest, mapping=replace(data.manifest.mapping, base_elevation_margin_m=0.005))
        validate_sequence(manifest, frames, data.initial_clearance_cells, data.clearance_provenance)
        self.assertEqual(frames[1].robot_pose.z_m, 0.003)
        frames[1] = replace(frames[1], robot_pose=replace(frames[1].robot_pose, z_m=0.006))
        with self.assertRaisesRegex(ContractError, 'base elevation'):
            validate_sequence(manifest, frames, data.initial_clearance_cells, data.clearance_provenance)

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.dataset = self.root / 'dataset'
        generate_synthetic(self.dataset, CONFIG)

    def index(self):
        return json.loads((self.dataset / 'sequence.json').read_text(encoding='utf-8'))

    def write_index(self, index):
        (self.dataset / 'sequence.json').write_text(dumps(index), encoding='utf-8')

    def test_generation_roundtrip_motion_scan_and_no_overwrite(self):
        first = load_sequence(self.dataset)
        self.assertEqual(first.manifest.source, 'synthetic')
        self.assertEqual(len(first.observations), 7)
        self.assertGreater(len({frame.robot_pose.x_m for frame in first.observations}), 1)
        self.assertGreater(len({frame.robot_pose.yaw_rad for frame in first.observations}), 1)
        # The permitted seed is only the footprint mask, not the broad free patch
        # used in earlier hand-authored cross-module fixtures.
        self.assertLess(len(first.initial_clearance_cells), 10)
        other = self.root / 'other'
        generate_synthetic(other, CONFIG)
        self.assertEqual(load_sequence(other), first)
        with self.assertRaises(FileExistsError):
            generate_synthetic(self.dataset, CONFIG)
        self.assertEqual(load_sequence(self.dataset), first)

    def test_full_replay_repeat_hashes_assessments_and_artifacts(self):
        output = self.root / 'replay'
        report = replay(self.dataset, output)
        self.assertTrue(report['repeat_verified'])
        self.assertEqual(report['status'], 'complete')
        self.assertEqual(report['mode'], 'offline_rgbd_replay')
        self.assertGreater(report['final_known_cells'], report['first_known_cells'])
        self.assertGreater(report['final_explored_cells'], report['first_explored_cells'])
        self.assertGreater(report['reachable_assessments'], 0)
        self.assertGreater(report['rejected_assessments'], 0)
        self.assertAlmostEqual(report['recorded_translation_m'], .4)
        events = [json.loads(line) for line in (output / 'events.jsonl').read_text(encoding='utf-8').splitlines()]
        self.assertEqual(len(events), 7)
        self.assertGreater(len({tuple(f['frontier_id'] for f in e['state']['frontiers']) for e in events}), 1)
        for event in events:
            self.assertEqual(event['source'], 'synthetic')
            self.assertEqual(event['state']['revision'], event['assessment']['state_revision'])
            for decision in event['decisions'].values():
                if decision['subgoal'] is not None:
                    self.assertIn(decision['subgoal']['frontier_id'], event['eligible_frontier_ids'])
            self.assertIsNone(event['state']['target_claim'])
        for path in (output / 'frames').glob('*.svg'):
            ET.parse(path)
        self.assertIn('decisions are not executed', (output / 'report.html').read_text(encoding='utf-8'))
        second = replay(self.dataset, self.root / 'repeat')
        self.assertEqual(second, report)
        self.assertEqual((output / 'events.jsonl').read_bytes(), (self.root / 'repeat/events.jsonl').read_bytes())
        with self.assertRaises(FileExistsError):
            replay(self.dataset, output)

    def test_tampered_frame_and_manifest_fail_before_output(self):
        path = self.dataset / 'frames/000000/depth.bin'
        original = path.read_bytes()
        path.write_bytes(original[:-1])
        with self.assertRaisesRegex(ContractError, 'checksum mismatch'):
            replay(self.dataset, self.root / 'bad')
        self.assertFalse((self.root / 'bad').exists())
        path.write_bytes(original)
        manifest = self.dataset / 'manifest.json'
        manifest.write_text(manifest.read_text(encoding='utf-8') + ' ', encoding='utf-8')
        with self.assertRaisesRegex(ContractError, 'manifest checksum'):
            load_sequence(self.dataset)

    def test_reordered_duplicate_and_escaping_paths_are_rejected(self):
        original = self.index()
        swapped = dict(original, frames=list(reversed(original['frames'])))
        self.write_index(swapped)
        with self.assertRaisesRegex(ContractError, 'increasing sequence'):
            load_sequence(self.dataset)
        self.write_index(dict(original, frames=[original['frames'][0]] * 2))
        with self.assertRaisesRegex(ContractError, 'duplicate frame'):
            load_sequence(self.dataset)
        escaped = dict(original['frames'][0], path='../outside')
        self.write_index(dict(original, frames=[escaped]))
        with self.assertRaisesRegex(ContractError, 'escapes package'):
            load_sequence(self.dataset)

    def test_extra_spawn_clearance_cannot_reveal_free_map(self):
        sequence = load_sequence(self.dataset)
        destination = self.root / 'extra-clearance'
        with self.assertRaisesRegex(ContractError, 'initial footprint'):
            write_sequence(destination, sequence.manifest, sequence.observations,
                           sequence.initial_clearance_cells + (GridCell(0, 0),), 'invalid broad seed')
        self.assertFalse(destination.exists())

    def test_capture_metadata_mismatch_is_rejected(self):
        sequence = load_sequence(self.dataset)
        first = sequence.observations[0]
        variants = (
            (), (replace(first, episode_id='other'),),
            (replace(first, robot_pose=replace(first.robot_pose, x_m=first.robot_pose.x_m + .1)),),
            (first, replace(sequence.observations[1], camera_intrinsics=replace(first.camera_intrinsics, fx=99.))),
        )
        for index, frames in enumerate(variants):
            with self.subTest(index=index), self.assertRaises(ContractError):
                write_sequence(self.root / ('bad-%d' % index), sequence.manifest, frames,
                               sequence.initial_clearance_cells, sequence.clearance_provenance)

    def test_pack_existing_frame_format_preserves_source_metadata(self):
        sequence = load_sequence(self.dataset)
        clearance = self.root / 'clearance.json'
        clearance.write_text(dumps(dict(cells=sequence.initial_clearance_cells,
                                        provenance=sequence.clearance_provenance)), encoding='utf-8')
        paths = tuple(self.dataset / frame['path'] for frame in self.index()['frames'])
        destination = self.root / 'packed'
        pack_recordings(destination, self.dataset / 'manifest.json', paths, clearance)
        packed = load_sequence(destination)
        self.assertEqual(packed.observations, sequence.observations)
        original_meta = json.loads((paths[0] / 'observation.json').read_text(encoding='utf-8'))
        packed_meta = json.loads((destination / 'frames/000000/observation.json').read_text(encoding='utf-8'))
        self.assertEqual(original_meta['source'], packed_meta['source'])

    def test_pipeline_failure_is_reported_without_success_artifacts(self):
        output = self.root / 'failed'
        with patch('frontier_search.runner.replay._assess', side_effect=RuntimeError('injected mapper failure')):
            with self.assertRaises(RuntimeError):
                replay(self.dataset, output)
        report = json.loads((output / 'report.json').read_text(encoding='utf-8'))
        self.assertEqual(report['status'], 'failed')
        self.assertFalse(report['repeat_verified'])
        self.assertFalse((output / 'report.html').exists())

    def test_repeat_mismatch_fails_explicitly_and_actuation_is_forbidden(self):
        sequence = load_sequence(self.dataset)
        with self.assertRaises(RuntimeError):
            AssessmentControl(sequence.observations[0]).set_velocity(0., 0.)
        output = self.root / 'nondeterministic'
        with patch('frontier_search.runner.replay._assess', side_effect=[[{'a': 1}], [{'a': 2}]]):
            with self.assertRaisesRegex(ContractError, 'repeat replay differs'):
                replay(self.dataset, output)
        self.assertEqual(json.loads((output / 'report.json').read_text(encoding='utf-8'))['status'], 'failed')

    def test_cli_reports_invalid_input_with_nonzero_status(self):
        result = subprocess.run([sys.executable, '-m', 'frontier_search.runner.replay', 'run',
                                 '--dataset', str(self.root / 'missing'), '--output', str(self.root / 'out')],
                                capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('replay error', result.stderr)


if __name__ == '__main__':
    unittest.main()
