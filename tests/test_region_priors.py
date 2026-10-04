"""Room-prior loading and mapper initialization without simulator dependencies."""

from dataclasses import replace
import json
from pathlib import Path
import tempfile
import unittest

from frontier_search.immutable import require_immutable
from frontier_search.perception.mapper import RGBDMapper
from frontier_search.perception.priors import load_region_priors
from frontier_search.validation import ContractError
from test_perception_reset import make_request


class RegionPriorTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.path = Path(self.temporary.name) / "rooms.json"
        self.region = {"region_id": "R1", "boundary_world": [[-1, 2], [1, 2], [1, 4], [-1, 4]]}

    def load(self, rooms, oracle=False, **fields):
        document = dict(schema_version="1.0", frame_id="world", regions=rooms)
        document.update(fields)
        self.path.write_text(json.dumps(document), encoding="utf-8")
        return load_region_priors(self.path, oracle_room_labels=oracle)

    def test_loaded_priors_reset_without_revealing_free_space(self):
        priors = self.load([self.region])
        require_immutable(priors)
        mapper = RGBDMapper()
        mapper.reset(replace(make_request(), region_priors=priors, initial_clearance_cells=()))
        self.assertEqual(mapper._state.request.region_priors, priors)
        self.assertTrue(all(x == -1 for row in mapper._state.occupancy for x in row))
        self.assertTrue(all(not x for row in mapper._state.coverage for x in row))
        self.assertIsNone(mapper._state.latest_state)
        self.region["boundary_world"][0][0] = 99
        self.assertEqual(priors[0].boundary_world[0].x_m, -1)

    def test_empty_rooms_and_reversed_or_closed_polygons(self):
        self.assertEqual(self.load([]), ())
        vertices = self.region["boundary_world"]
        for boundary in (list(reversed(vertices)), vertices + [vertices[0]]):
            self.assertEqual(len(self.load([dict(self.region, boundary_world=boundary)])), 1)

    def test_semantic_labels_require_explicit_oracle_authorization(self):
        room = dict(self.region, room_type="kitchen")
        with self.assertRaises(ContractError):
            self.load([room])
        self.assertEqual(self.load([room], oracle=True)[0].room_type, "kitchen")
        with self.assertRaises(ContractError):
            self.load([room], oracle=1)

    def test_malformed_rooms_and_duplicate_ids_are_rejected(self):
        invalid = [
            [self.region, self.region], [dict(self.region, region_id=" ")],
            [dict(self.region, region_id=[])], [dict(self.region, room_type=[])],
            [dict(self.region, boundary_world=[])],
            [dict(self.region, boundary_world=[[0, 0], [1, 1], [2, 2]])],
            [dict(self.region, boundary_world=[[0, 0], [1, 0], [0, 0]])],
            [dict(self.region, boundary_world=[[False, 0], [1, 0], [0, 1]])],
            [dict(self.region, boundary_world=[[0, 0], [1, 0], [0, float("nan")]])],
            [dict(self.region, boundary_world=[[0, 0], [1, 0], [0, 10 ** 400]])],
            [dict(self.region, objects=[{"class": "cup"}])], [None],
        ]
        for rooms in invalid:
            with self.subTest(rooms=rooms), self.assertRaises(ContractError):
                self.load(rooms)

    def test_wrong_schema_frame_and_hidden_root_fields_are_rejected(self):
        for fields in ({"schema_version": "2.0"}, {"frame_id": "camera"},
                       {"occupancy": [[0]]}, {"regions": {}}):
            with self.subTest(fields=fields), self.assertRaises(ContractError):
                self.load([self.region], **fields)

    def test_unreadable_and_malformed_documents_raise_contract_error(self):
        with self.assertRaises(ContractError):
            load_region_priors(self.path)
        for text in ("{bad json", "[]", "null"):
            self.path.write_text(text, encoding="utf-8")
            with self.subTest(text=text), self.assertRaises(ContractError):
                load_region_priors(self.path)


if __name__ == "__main__":
    unittest.main()
