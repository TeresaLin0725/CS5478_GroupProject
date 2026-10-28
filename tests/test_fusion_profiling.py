"""The diagnostic must preserve and compare private, sequential fusion evidence."""

from copy import deepcopy
from dataclasses import replace
import gzip
import json
from pathlib import Path
import tempfile
import unittest

from frontier_search.fixtures.sequence import load_sequence, mapper_init
from frontier_search.perception.mapper import RGBDMapper
from frontier_search.perception.profiling import (
    BUFFER_NAMES, compare_snapshots, read_baseline, replay_fusion, run_diagnostic, state_transitions,
)
from frontier_search.runner.replay import generate_synthetic


CONFIG = Path(__file__).resolve().parents[1] / 'configs' / 'w2.rgbd-synthetic.json'


class FusionProfilingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory()
        cls.root = Path(cls.temporary.name)
        generate_synthetic(cls.root / 'dataset', CONFIG)
        cls.sequence = load_sequence(cls.root / 'dataset')

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def test_diagnostic_buffers_equal_direct_mapper_each_frame(self):
        # Enable the real clearing path, including the same-timestamp gate.
        sequence = replace(self.sequence, manifest=replace(self.sequence.manifest,
            mapping=replace(self.sequence.manifest.mapping, dynamic_clear_frames=2)))
        frames = sequence.observations[:3]
        frames = (frames[0], frames[1], replace(frames[2], timestamp_s=frames[1].timestamp_s))
        records, snapshots = replay_fusion(sequence, frames, capture=True)
        direct = RGBDMapper()
        direct.reset(mapper_init(sequence.manifest, sequence.initial_clearance_cells))
        expected = []
        for frame in frames:
            direct.update(frame)
            expected.append(dict(sequence=frame.sequence,
                                 buffers={name: deepcopy(getattr(direct._state, name))
                                          for name in BUFFER_NAMES}))
        self.assertTrue(compare_snapshots(expected, snapshots)['equal'])
        self.assertEqual([record['sequence'] for record in records], [frame.sequence for frame in frames])

    def test_archive_repeat_profile_and_baseline_comparison(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            report, equal = run_diagnostic(self.sequence, root / 'before', repeats=2,
                                           warmups=0, max_frames=2)
            self.assertTrue(equal)
            self.assertEqual(report['unprofiled_summary']['count'], 4)
            self.assertEqual(len(list((root / 'before' / 'profiles').glob('*.prof'))), 2)
            self.assertGreater(report['profiled_frames'][0]['stages']['ray_traversal_s'], 0)
            self.assertGreater(report['profiled_frames'][0]['stages']['map_finalization_s'], 0)
            baseline_path = root / 'before' / 'baseline.json.gz'
            archive = read_baseline(baseline_path)
            self.assertEqual(len(archive['snapshots']), 2)
            report, equal = run_diagnostic(self.sequence, root / 'after', repeats=1,
                                           warmups=0, max_frames=2, baseline=baseline_path)
            self.assertTrue(equal)
            self.assertTrue(report['baseline_comparison']['equal'])
            with self.assertRaises(FileExistsError):
                run_diagnostic(self.sequence, root / 'after', warmups=0, max_frames=2)
            with self.assertRaisesRegex(ValueError, 'inputs/configuration differ'):
                run_diagnostic(self.sequence, root / 'wrong-input', warmups=0,
                               max_frames=1, baseline=baseline_path)
            self.assertFalse((root / 'wrong-input').exists())
            # A hidden evidence change must fail even if the occupancy map agrees.
            archive['snapshots'][0]['buffers']['clear_counts'][0][0] += 1
            bad_path = root / 'tampered.json.gz'
            with gzip.open(str(bad_path), 'wt', encoding='utf-8') as stream:
                json.dump(archive, stream)
            report, equal = run_diagnostic(self.sequence, root / 'bad', repeats=1,
                                           warmups=0, max_frames=2, baseline=bad_path)
            self.assertFalse(equal)
            diff = report['baseline_comparison']['differences'][0]
            self.assertEqual(diff['buffer'], 'clear_counts')
            self.assertEqual(diff['changed_cells'], 1)

    def test_diff_catches_every_buffer(self):
        _, original = replay_fusion(self.sequence, self.sequence.observations[:1], capture=True)
        for name in BUFFER_NAMES:
            with self.subTest(buffer=name):
                changed = deepcopy(original)
                cell = changed[0]['buffers'][name][0][0]
                changed[0]['buffers'][name][0][0] = not cell if type(cell) is bool else cell + 1
                difference = compare_snapshots(original, changed)
                self.assertFalse(difference['equal'])
                self.assertEqual(difference['differences'][0]['buffer'], name)

    def test_fixed_640_fixture_exercises_clearing_and_timestamp_gate(self):
        import importlib.util
        path = CONFIG.parents[1] / 'scripts' / 'make_fusion_fixture.py'
        spec = importlib.util.spec_from_file_location('make_fusion_fixture', str(path))
        generator = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(generator)
        with tempfile.TemporaryDirectory() as folder:
            dataset = Path(folder) / 'fixture'
            generator.make_fixture(dataset)
            with self.assertRaises(FileExistsError):
                generator.make_fixture(dataset)
            sequence = load_sequence(dataset)
            self.assertEqual(sequence.manifest.mapping.depth_pixel_stride, 4)
            self.assertEqual(sequence.observations[0].depth_image.width, 640)
            _, snapshots = replay_fusion(sequence, sequence.observations, capture=True)
            transitions = state_transitions(snapshots)
            occupied = transitions[0]['occupied_cells']
            self.assertGreater(occupied, 0)
            self.assertGreater(transitions[1]['pending_clear_cells'], 0)
            self.assertEqual(snapshots[1]['buffers']['clear_counts'], snapshots[2]['buffers']['clear_counts'])
            self.assertEqual(transitions[2]['obstacle_to_free_cells'], 0)
            self.assertEqual(transitions[3]['pending_clear_cells'], 0)
            self.assertEqual(transitions[4]['obstacle_to_free_cells'], 0)
            self.assertEqual(transitions[5]['obstacle_to_free_cells'], occupied)
