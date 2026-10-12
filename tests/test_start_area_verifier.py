"""B's verified-starting-area verifier against the mock scene (not a physics check)."""

from dataclasses import replace
import unittest

from frontier_search.contracts import GridCell, OccupancyGrid
from frontier_search.fixtures.clearance import footprint_cells, validate_clearance
from frontier_search.fixtures.sequence import mapper_init
from frontier_search.fixtures.w2_episode import mock_manifest
from frontier_search.simulation.start_area_verifier import MockSceneProbe, verify_start_area

# Mock manifest: 40x40 grid at 0.1 m, start (1.05, 1.55) => start cell row 15, column 10.
MANIFEST = replace(mock_manifest(), initial_clearance_mode='verified_start_area',
                   initial_clearance_radius_m=1.0)


def grid(walls=()):
    data = [[0] * 40 for _ in range(40)]
    for r, c in walls:
        data[r][c] = 100
    return OccupancyGrid('world', 0, .1, 0, 0, 40, 40, tuple(tuple(row) for row in data))


class StartAreaVerifierTests(unittest.TestCase):
    def verify(self, **probe):
        walls = probe.pop('walls', ())
        return verify_start_area(MANIFEST, MockSceneProbe(grid(walls), **probe), 'mock-verifier')

    def test_open_floor_gives_connected_area_accepted_by_a_guards(self):
        result = self.verify()
        cells = set(result.cells)
        self.assertTrue(set(footprint_cells(MANIFEST)) <= cells)
        self.assertGreater(len(cells), len(footprint_cells(MANIFEST)))
        self.assertEqual(result.rejected, ())
        self.assertEqual(result.verification['checked_z_min_m'], .03)
        self.assertEqual(result.verification['checked_z_max_m'], 1.0)
        validate_clearance(MANIFEST, result.cells, result.verification)
        mapper_init(MANIFEST, result.cells, result.verification)
        self.assertEqual(set(result.clearance_record()), {'cells', 'provenance', 'verification'})

    def test_every_cell_square_lies_inside_the_radius_not_just_its_centre(self):
        c = MANIFEST.mapping
        for cell in self.verify().cells:
            for x in (cell.column, cell.column + 1):
                for y in (cell.row, cell.row + 1):
                    self.assertLessEqual(((x * c.resolution_m - 1.05) ** 2 + (y * c.resolution_m - 1.55) ** 2) ** .5,
                                         1.0 + 1e-9)

    def test_wall_in_radius_is_excluded_not_filled_in(self):
        result = self.verify(walls=((15, 16),))
        self.assertNotIn(GridCell(15, 16), result.cells)
        self.assertIn((GridCell(15, 16), 'collision column blocked'), result.rejected)

    def test_object_outside_robot_height_band_does_not_block_but_in_band_does(self):
        # Shelf above the 1.0 m collision top and a 1 cm floor-level lip below the 0.03 m floor band.
        high = self.verify(walls=((15, 16),), obstacle_z={(15, 16): (1.2, 2.0)})
        low = self.verify(walls=((15, 17),), obstacle_z={(15, 17): (0.0, 0.01)})
        self.assertIn(GridCell(15, 16), high.cells)
        self.assertIn(GridCell(15, 17), low.cells)
        mid = self.verify(walls=((15, 16),), obstacle_z={(15, 16): (0.4, 0.8)})
        self.assertNotIn(GridCell(15, 16), mid.cells)

    def test_hole_and_step_fail_floor_support_but_small_drift_passes(self):
        result = self.verify(floor_z={(15, 16): None, (15, 17): 0.1, (15, 18): 0.015})
        reasons = dict(result.rejected)
        self.assertEqual(reasons[GridCell(15, 16)], 'floor not supported/traversable')
        self.assertEqual(reasons[GridCell(15, 17)], 'floor not supported/traversable')
        self.assertIn(GridCell(15, 18), result.cells)

    def test_closed_off_pocket_is_dropped_as_disconnected(self):
        ring = tuple((r, c) for r in (17, 21) for c in range(13, 18)) \
            + tuple((r, c) for c in (13, 17) for r in range(17, 22))
        result = self.verify(walls=ring)
        self.assertNotIn(GridCell(19, 15), result.cells)   # free, but enclosed
        self.assertIn((GridCell(19, 15), 'not connected to the initial footprint'), result.rejected)
        validate_clearance(MANIFEST, result.cells, result.verification)

    def test_blocked_footprint_rejects_initialisation(self):
        with self.assertRaisesRegex(ValueError, 'cannot be verified'):
            self.verify(walls=((15, 10),))
        with self.assertRaisesRegex(ValueError, 'cannot be verified'):
            self.verify(floor_z={(15, 10): None})

    def test_requires_verified_mode_and_verifier_id(self):
        probe = MockSceneProbe(grid())
        with self.assertRaises(ValueError):
            verify_start_area(mock_manifest(), probe, 'v')
        with self.assertRaises(ValueError):
            verify_start_area(MANIFEST, probe, ' ')

    def test_record_is_bound_to_this_context_and_cells(self):
        result = self.verify()
        other = replace(MANIFEST, scene_id='other')
        with self.assertRaises(Exception):
            validate_clearance(other, result.cells, result.verification)
        with self.assertRaises(Exception):
            validate_clearance(MANIFEST, result.cells[1:], result.verification)


if __name__ == '__main__':
    unittest.main()
