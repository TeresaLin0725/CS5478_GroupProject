from dataclasses import replace
from pathlib import Path
import json
import tempfile
import unittest

from frontier_search.contracts import GridCell
from frontier_search.experiments.logging import dumps
from frontier_search.fixtures.clearance import clearance_binding, validate_clearance
from frontier_search.fixtures.sequence import load_sequence, mapper_init, write_sequence
from frontier_search.perception.mapper import RGBDMapper
from frontier_search.runner.replay import generate_synthetic, replay, pack_recordings
from frontier_search.validation import ContractError

CONFIG = Path(__file__).resolve().parents[1] / 'configs/w2.rgbd-synthetic.json'


class VerifiedStartAreaTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        generate_synthetic(self.root / 'original', CONFIG)
        self.data = load_sequence(self.root / 'original')
        self.manifest = replace(self.data.manifest, initial_clearance_mode='verified_start_area',
                                initial_clearance_radius_m=1.0)
        # This analytic fixture has flat floor and its wall is at x=4, outside this rectangle.
        self.cells = tuple(GridCell(r, c) for r in range(7, 13) for c in range(1, 6))
        binding = clearance_binding(self.manifest, self.cells)
        self.verification = dict(context_sha256=binding['context_sha256'], cells_sha256=binding['cells_sha256'],
            checked_z_min_m=binding['required_z_min_m'], checked_z_max_m=binding['required_z_max_m'],
            all_cells_clear=True, floor_support_verified=True, verifier='analytic test fixture',
            verification_method='Flat plane z=0; only wall x=4 lies outside all supplied cells.',
            scene_snapshot_id='synthetic-test-only')

    def test_default_rejects_expansion_and_area_requires_verification(self):
        self.assertEqual(self.data.manifest.initial_clearance_mode, 'footprint')
        with self.assertRaises(ContractError):
            mapper_init(self.data.manifest, self.cells)
        with self.assertRaises(ContractError):
            mapper_init(self.manifest, self.cells)
        validate_clearance(self.manifest, self.cells, self.verification)

    def test_binding_cannot_be_reused_for_different_context_or_cells(self):
        for manifest in (replace(self.manifest, scene_id='changed'),
                         replace(self.manifest, initial_clearance_radius_m=1.1),
                         replace(self.manifest, robot=replace(self.manifest.robot, collision_z_max_m=.5))):
            with self.subTest(manifest=manifest), self.assertRaisesRegex(ContractError, 'does not match'):
                validate_clearance(manifest, self.cells, self.verification)
        with self.assertRaisesRegex(ContractError, 'does not match'):
            validate_clearance(self.manifest, self.cells[1:], self.verification)
        validate_clearance(replace(self.manifest, policy_name='random'), self.cells, self.verification)

    def test_rejects_bad_attestations_and_height_bounds(self):
        variants = ({'all_cells_clear': False}, {'floor_support_verified': False}, {'verifier': ''},
                    {'checked_z_min_m': .2}, {'checked_z_max_m': .3}, {'checked_z_max_m': float('nan')})
        for change in variants:
            with self.subTest(change=change), self.assertRaises(ContractError):
                validate_clearance(self.manifest, self.cells, dict(self.verification, **change))

    def test_rejects_cells_outside_radius_map_duplicates_missing_footprint_and_islands(self):
        variants = (self.cells + (self.cells[0],), self.cells + (GridCell(-1, 1),),
                    self.cells + (GridCell(10, 8),), (GridCell(10, 3),),
                    self.data.initial_clearance_cells + (GridCell(7, 3),))
        for cells in variants:
            with self.subTest(cells=cells), self.assertRaises(ContractError):
                validate_clearance(self.manifest, cells, self.verification)
        # Centre fits radius .65 but a rectangle corner exceeds it.
        with self.assertRaisesRegex(ContractError, 'entire clearance cell'):
            validate_clearance(replace(self.manifest, initial_clearance_radius_m=.65), self.cells, self.verification)

    def test_roundtrip_pack_replay_labels_and_coverage(self):
        destination = self.root / 'area'
        write_sequence(destination, self.manifest, self.data.observations, self.cells,
                       'analytic fixture only', verification=self.verification)
        loaded = load_sequence(destination)
        self.assertEqual(loaded.clearance_verification, self.verification)
        self.assertEqual(json.loads((destination / 'sequence.json').read_text())['format_version'], 2)
        report = replay(destination, self.root / 'report')
        self.assertTrue(report['repeat_verified'])
        self.assertEqual(report['initial_clearance_mode'], 'verified_start_area')
        self.assertEqual(report['initial_clearance_cells'], len(self.cells))
        self.assertIn('verified_start_area', (self.root / 'report/report.html').read_text())
        baseline, expanded = RGBDMapper(), RGBDMapper()
        baseline.reset(mapper_init(self.data.manifest, self.data.initial_clearance_cells))
        expanded.reset(mapper_init(self.manifest, self.cells, self.verification))
        for frame in self.data.observations:
            a, b = baseline.update(frame), expanded.update(frame)
            self.assertEqual(a.explored_map, b.explored_map)
        clearance = self.root / 'clearance.json'
        clearance.write_text(dumps(dict(cells=self.cells, provenance='analytic fixture', verification=self.verification)))
        frames = tuple(sorted((destination / 'frames').iterdir()))
        pack_recordings(self.root / 'packed', destination / 'manifest.json', frames, clearance)
        self.assertEqual(load_sequence(self.root / 'packed').clearance_verification, self.verification)
        index = json.loads((destination / 'sequence.json').read_text())
        index['clearance_verification']['floor_support_verified'] = False
        (destination / 'sequence.json').write_text(dumps(index))
        with self.assertRaises(ContractError):
            replay(destination, self.root / 'bad')
        self.assertFalse((self.root / 'bad').exists())

    def test_mode_radius_validation(self):
        for mode, radius in (('footprint', 1), ('verified_start_area', 0), ('unknown', 1),
                             ('verified_start_area', float('inf'))):
            with self.subTest(mode=mode, radius=radius), self.assertRaises(ValueError):
                replace(self.manifest, initial_clearance_mode=mode, initial_clearance_radius_m=radius).validate()


if __name__ == '__main__':
    unittest.main()
