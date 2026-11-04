"""B verifier output -> A's `replay pack` -> load -> `replay run`, on A's analytic floor/wall fixture.

The mock probe mirrors that fixture's truth (flat floor, wall at x=4) so B's record is consistent
with the recorded frames. This checks the file hand-off, not real Locobot clearance.
"""

from dataclasses import replace
from pathlib import Path
import json
import tempfile
import unittest

from frontier_search.contracts import GridCell, OccupancyGrid
from frontier_search.experiments.logging import dumps
from frontier_search.fixtures.sequence import load_sequence
from frontier_search.runner.manifest import load_manifest
from frontier_search.navigation.locobot_robot import shared_round_robot
from frontier_search.runner.replay import generate_synthetic, pack_recordings, replay
from frontier_search.simulation.start_area_verifier import MockSceneProbe, verify_start_area
from frontier_search.validation import ContractError

CONFIG = Path(__file__).resolve().parents[1] / 'configs/w2.rgbd-synthetic.json'


class StartAreaRoundTripTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        generate_synthetic(self.root / 'recorded', CONFIG)       # stands in for B's raw frames
        self.frames = tuple(sorted((self.root / 'recorded' / 'frames').iterdir()))
        # Verified-area manifest on disk, as B would hand it to A.
        raw = json.loads(CONFIG.read_text())
        raw.update(initial_clearance_mode='verified_start_area', initial_clearance_radius_m=1.0)
        self.manifest_path = self.root / 'manifest.json'
        self.manifest_path.write_text(json.dumps(raw))
        self.manifest = load_manifest(self.manifest_path)
        m = self.manifest.mapping
        wall_col = int(round((4.0 - m.origin_x_m) / m.resolution_m))
        truth = OccupancyGrid('world', 0, m.resolution_m, m.origin_x_m, m.origin_y_m,
                              m.width_cells, m.height_cells,
                              tuple(tuple(100 if c >= wall_col else 0 for c in range(m.width_cells))
                                    for _ in range(m.height_cells)))
        self.truth = truth
        self.result = verify_start_area(self.manifest, MockSceneProbe(truth, snapshot_id='analytic-wall-x4'),
                                        'B mock verifier')

    def write_clearance(self, record=None):
        path = self.root / 'clearance.json'
        path.write_text(dumps(record or self.result.clearance_record()))
        return path

    def test_exported_record_packs_loads_and_replays(self):
        self.assertGreater(len(self.result.cells), len(self.load_footprint()))
        package = self.root / 'package'
        pack_recordings(package, self.manifest_path, self.frames, self.write_clearance())
        data = load_sequence(package)
        self.assertEqual(set(data.initial_clearance_cells), set(self.result.cells))
        self.assertEqual(data.clearance_verification, self.result.verification)
        self.assertEqual(data.manifest.initial_clearance_mode, 'verified_start_area')
        report = replay(package, self.root / 'report')
        self.assertTrue(report['repeat_verified'])
        self.assertEqual(report['initial_clearance_cells'], len(self.result.cells))

    def test_tampered_exports_are_rejected_by_pack(self):
        record = json.loads(dumps(self.result.clearance_record()))
        cells_dropped = dict(record, cells=record['cells'][1:])
        flag_off = dict(record, verification=dict(record['verification'], floor_support_verified=False))
        no_verification = {k: v for k, v in record.items() if k != 'verification'}
        for name, bad in (('cells', cells_dropped), ('flag', flag_off), ('missing', no_verification)):
            with self.subTest(bad=name), self.assertRaises(ContractError):
                pack_recordings(self.root / ('bad-' + name), self.manifest_path, self.frames,
                                self.write_clearance(bad))

    def test_record_for_a_different_radius_is_rejected(self):
        raw = json.loads(self.manifest_path.read_text())
        raw['initial_clearance_radius_m'] = 1.2
        other = self.root / 'other.json'
        other.write_text(json.dumps(raw))
        with self.assertRaises(ContractError):
            pack_recordings(self.root / 'bad-radius', other, self.frames, self.write_clearance())

    def test_round_geometry_requires_fresh_attestation_and_preserves_old_package(self):
        square_bytes = self.manifest_path.read_bytes()
        rounded = replace(self.manifest, robot=shared_round_robot(self.manifest.robot))
        round_path = self.root / 'round-manifest.json'
        round_path.write_text(dumps(rounded), encoding='utf-8')
        with self.assertRaisesRegex(ContractError, 'does not match'):
            pack_recordings(self.root / 'bad-round', round_path, self.frames, self.write_clearance())
        fresh = verify_start_area(rounded, MockSceneProbe(self.truth), 'round verifier')
        fresh_path = self.root / 'round-clearance.json'
        fresh_path.write_text(dumps(fresh.clearance_record()), encoding='utf-8')
        package = self.root / 'round-package'
        pack_recordings(package, round_path, self.frames, fresh_path)
        data = load_sequence(package)
        self.assertEqual(data.manifest.robot, rounded.robot)
        self.assertEqual(data.clearance_verification, fresh.verification)
        self.assertEqual(self.manifest_path.read_bytes(), square_bytes)

    def load_footprint(self):
        from frontier_search.fixtures.clearance import footprint_cells
        return footprint_cells(self.manifest)


if __name__ == '__main__':
    unittest.main()
