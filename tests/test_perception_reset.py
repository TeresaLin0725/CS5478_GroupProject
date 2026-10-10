"""Episode reset checks without simulator, camera decoding or detection."""

from dataclasses import replace
from random import Random, getstate
import unittest

from frontier_search.config import MappingConfig, ObservationConfig
from frontier_search.contracts import GridCell, MapperInit, Point2, RegionPrior, RobotConfig
from frontier_search.fixtures.loader import load_policy_fixture
from frontier_search.perception.mapper import RGBDMapper
from frontier_search.validation import ContractError


def make_request():
    robot = RobotConfig(
        footprint_base=(Point2(-0.1, -0.1), Point2(0.1, -0.1),
                        Point2(0.1, 0.1), Point2(-0.1, 0.1)),
        collision_z_min_m=0.0, collision_z_max_m=0.4, clearance_margin_m=0.02,
        motion_model="differential_drive", can_turn_in_place=True,
        max_linear_speed_mps=0.3, max_angular_speed_radps=1.0,
        position_tolerance_m=0.05, yaw_tolerance_rad=0.1,
    )
    region = RegionPrior("R1", (Point2(-1, 2), Point2(1, 2),
                                Point2(1, 4), Point2(-1, 4)), None)
    return MapperInit(
        "episode-1", "cup", 7,
        MappingConfig(width_cells=4, height_cells=3, origin_x_m=-1.0, origin_y_m=2.0),
        ObservationConfig(), robot, (region,), (GridCell(0, 0), GridCell(2, 3)), False,
    )


class MapperResetTests(unittest.TestCase):
    def setUp(self):
        self.request = make_request()
        self.mapper = RGBDMapper()

    def test_constructor_and_reset_publish_no_state(self):
        self.assertIsNone(self.mapper._state)
        self.assertIsNone(self.mapper.reset(self.request))
        state = self.mapper._state
        self.assertEqual(state.request, self.request)
        self.assertEqual(state.revision, 0)
        self.assertIsNone(state.last_processed_sequence)
        self.assertIsNone(state.last_timestamp_s)
        self.assertIsNone(state.latest_state)

    def test_only_clearance_seeds_free_space_and_no_coverage(self):
        self.mapper.reset(self.request)
        state = self.mapper._state
        self.assertEqual(state.occupancy, [[0, -1, -1, -1], [-1, -1, -1, -1], [-1, -1, -1, 0]])
        self.assertEqual(state.coverage, [[False] * 4 for _ in range(3)])
        self.assertEqual(state.request.region_priors, self.request.region_priors)
        # Room polygons alone, including ones larger than the grid, imply no free cells.
        self.mapper.reset(replace(self.request, initial_clearance_cells=()))
        self.assertTrue(all(v == -1 for row in self.mapper._state.occupancy for v in row))

    def test_buffers_have_independent_rows_and_mapper_instances(self):
        self.mapper.reset(self.request)
        other = RGBDMapper()
        other.reset(self.request)
        self.mapper._state.occupancy[0][1] = 100
        self.mapper._state.coverage[0][1] = True
        self.assertEqual(self.mapper._state.occupancy[1][1], -1)
        self.assertFalse(self.mapper._state.coverage[1][1])
        self.assertEqual(other._state.occupancy[0][1], -1)
        self.assertFalse(other._state.coverage[0][1])

    def test_reset_discards_all_episode_state(self):
        self.mapper.reset(self.request)
        old = self.mapper._state
        fixture = load_policy_fixture("target_claim").state
        old.occupancy[1][1] = 100
        old.coverage[1][1] = True
        old.tracks["old"] = fixture.semantic_objects[0]
        old.frontiers["old"] = fixture.frontiers[0]
        old.frontier_history["old"] = ("parent",)
        old.evidence["old"] = object()  # Sentinel for retained evidence storage.
        old.target_claim = fixture.target_claim
        for key in old.entity_counters:
            old.entity_counters[key] = 10
        old.revision, old.last_processed_sequence, old.last_timestamp_s = 5, 12, 3.5
        old.latest_state = fixture
        old.rng.random()
        new_request = replace(self.request, episode_id="episode-2", target_class="chair",
                              mapping_config=MappingConfig(width_cells=2, height_cells=2),
                              initial_clearance_cells=(), region_priors=(), seed=42)
        self.mapper.reset(new_request)
        state = self.mapper._state
        self.assertIsNot(state, old)
        self.assertEqual(state.request, new_request)
        self.assertEqual(state.occupancy, [[-1, -1], [-1, -1]])
        self.assertEqual(state.coverage, [[False, False], [False, False]])
        for name in ("tracks", "frontiers", "frontier_history", "evidence"):
            self.assertEqual(getattr(state, name), {})
        self.assertTrue(all(v == 0 for v in state.entity_counters.values()))
        self.assertEqual(state.revision, 0)
        for name in ("target_claim", "last_processed_sequence", "last_timestamp_s", "latest_state"):
            self.assertIsNone(getattr(state, name))
        self.assertEqual(state.rng.random(), Random(42).random())

    def test_private_rng_is_reproducible_without_global_side_effects(self):
        global_state = getstate()
        self.mapper.reset(self.request)
        first = [self.mapper._state.rng.random() for _ in range(4)]
        self.mapper.reset(self.request)
        self.assertEqual(first, [self.mapper._state.rng.random() for _ in range(4)])
        self.assertEqual(getstate(), global_state)

    def test_invalid_requests_leave_initialized_episode_untouched(self):
        region = self.request.region_priors[0]
        cfg = self.request.mapping_config
        invalid = [
            replace(self.request, episode_id=""),
            replace(self.request, target_class="  "),
            replace(self.request, seed=True),
            replace(self.request, oracle_room_labels=1),
            replace(self.request, mapping_config=replace(cfg, width_cells=2.5)),
            replace(self.request, mapping_config=replace(cfg, height_cells=True)),
            replace(self.request, mapping_config=replace(cfg, resolution_m=True)),
            replace(self.request, initial_clearance_cells=(GridCell(-1, 0),)),
            replace(self.request, initial_clearance_cells=(GridCell(3, 0),)),
            replace(self.request, initial_clearance_cells=(GridCell(0, 4),)),
            replace(self.request, initial_clearance_cells=(GridCell(0.5, 0),)),
            replace(self.request, initial_clearance_cells=(GridCell(True, 0),)),
            replace(self.request, region_priors=(region, region)),
            replace(self.request, region_priors=(replace(region, region_id=""),)),
            replace(self.request, region_priors=(replace(region, boundary_world=()),)),
            replace(self.request, region_priors=(replace(region, boundary_world=(
                Point2(0, 0), Point2(1, 0), Point2(0, 0))),)),
            replace(self.request, region_priors=(replace(region, boundary_world=(
                Point2(0, 0), Point2(1, 1), Point2(2, 2))),)),
            replace(self.request, region_priors=(replace(region, room_type="kitchen"),)),
        ]
        self.mapper.reset(self.request)
        previous = self.mapper._state
        previous.occupancy[1][1] = 100
        previous.tracks["saved"] = object()
        rng_state = previous.rng.getstate()
        for request in invalid:
            with self.subTest(request=request):
                with self.assertRaises(ContractError):
                    self.mapper.reset(request)
                self.assertIs(self.mapper._state, previous)
                self.assertEqual(previous.occupancy[1][1], 100)
                self.assertIn("saved", previous.tracks)
                self.assertEqual(previous.rng.getstate(), rng_state)

    def test_nonfinite_and_invalid_geometry_rejected_at_reset_boundary(self):
        # Normal constructors already reject several of these. Bypass them only
        # here to exercise reset's defensive checks on malformed external inputs.
        for field, value in (("resolution_m", 0), ("resolution_m", 10 ** 400), ("width_cells", 0),
                             ("height_cells", -1), ("origin_x_m", float("nan")),
                             ("origin_y_m", float("inf"))):
            with self.subTest(field=field):
                request = make_request()
                object.__setattr__(request.mapping_config, field, value)
                with self.assertRaises(ContractError):
                    self.mapper.reset(request)
                self.assertIsNone(self.mapper._state)
        request = make_request()
        object.__setattr__(request.region_priors[0].boundary_world[0], "x_m", float("nan"))
        with self.assertRaises(ContractError):
            self.mapper.reset(request)

    def test_oracle_labels_and_polygon_winding(self):
        region = replace(self.request.region_priors[0], room_type="kitchen")
        for vertices in (region.boundary_world, tuple(reversed(region.boundary_world)),
                         region.boundary_world + (region.boundary_world[0],)):
            request = replace(self.request, oracle_room_labels=True,
                              region_priors=(replace(region, boundary_world=vertices),))
            self.mapper.reset(request)
            self.assertEqual(self.mapper._state.request.region_priors[0].room_type, "kitchen")

    def test_invalid_updates_and_unimplemented_evidence_fail_explicitly(self):
        for initialized in (False, True):
            if initialized:
                self.mapper.reset(self.request)
            with self.assertRaises(ContractError):
                self.mapper.update(None)
            with self.assertRaises(NotImplementedError):
                self.mapper.get_claim_evidence("episode-1", "claim-1")


if __name__ == "__main__":
    unittest.main()
