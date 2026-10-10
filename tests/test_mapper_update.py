"""Geometric regression scenes rendered analytically, without simulator truth inputs."""

from dataclasses import FrozenInstanceError, replace
from functools import partial
from pathlib import Path
from struct import pack
import tempfile
import unittest
from unittest.mock import patch

from frontier_search.config import MappingConfig
from frontier_search.contracts import GridCell, Point2, RegionPrior
from frontier_search.fixtures.analytic_rgbd import render_frame
from frontier_search.fixtures.recording import load_rgbd_recording, save_rgbd_recording
from frontier_search.immutable import require_immutable
from frontier_search.perception.mapper import RGBDMapper
from frontier_search.perception.priors import rasterize_region
from frontier_search.validation import ContractError, validate_state
from test_perception_reset import make_request


scene_frame = partial(render_frame, episode_id="mapping-test")


def request(**mapping):
    cfg = MappingConfig(resolution_m=0.2, width_cells=25, height_cells=20,
                        origin_y_m=-2., **mapping)
    room = RegionPrior('R1', (Point2(0, -2), Point2(5, -2), Point2(5, 2), Point2(0, 2)), None)
    return replace(make_request(), episode_id='mapping-test', mapping_config=cfg,
                   region_priors=(room,), initial_clearance_cells=(GridCell(10, 0),))


class MapperUpdateTests(unittest.TestCase):
    def setUp(self):
        self.mapper = RGBDMapper()
        self.request = request()
        self.mapper.reset(self.request)

    def test_floor_wall_free_space_and_unobserved_cells(self):
        state = self.mapper.update(scene_frame())
        self.assertEqual(state.occupancy_map.data[10][20], 100)  # x=4 wall
        self.assertEqual(state.occupancy_map.data[10][7], 0)     # visible floor/column at x=1.5
        self.assertEqual(state.occupancy_map.data[10][23], -1)   # behind wall
        self.assertEqual(state.occupancy_map.data[0][0], -1)     # outside camera view
        self.assertFalse(state.explored_map.data[10][0])         # seed alone isn't coverage
        self.assertFalse(state.explored_map.data[10][23])
        self.assertTrue(state.explored_map.data[10][7])
        self.assertEqual(state.regions[0].explored_fraction,
                         sum(sum(row) for row in state.explored_map.data) / 500)
        validate_state(state)

    def test_invalid_depth_produces_no_new_evidence(self):
        frame = scene_frame()
        for depth in (0., -1., float('nan'), float('inf'), 0.05, 6.):
            with self.subTest(depth=depth):
                self.mapper.reset(self.request)
                invalid = replace(frame, depth_image=replace(frame.depth_image,
                    data=pack('<f', depth) * (61 * 61)))
                state = self.mapper.update(invalid)
                self.assertEqual(sum(v == 0 for row in state.occupancy_map.data for v in row), 1)
                self.assertFalse(any(any(row) for row in state.explored_map.data))

    def test_low_obstacle_overrides_clearance_without_inflation(self):
        box = ((1.4, -0.21, 0.), (1.8, 0.21, 0.18))
        seed = replace(self.request, initial_clearance_cells=(GridCell(10, 7),))
        self.mapper.reset(seed)
        state = self.mapper.update(scene_frame(boxes=(box,)))
        # Float32 projection can place the front face on either side of the x=1.4 boundary.
        self.assertIn(100, state.occupancy_map.data[10][6:9])
        self.assertNotEqual(state.occupancy_map.data[10][8], 0)
        self.assertNotEqual(state.occupancy_map.data[7][7], 100)

    def test_tabletop_is_not_floor_or_under_table_clearance(self):
        table = ((1.2, -0.4, 0.3), (2.0, 0.4, 0.35))
        state = self.mapper.update(scene_frame(boxes=(table,)))
        self.assertNotEqual(state.occupancy_map.data[10][8], 0)
        # A shorter robot ignores a tabletop above its envelope, but still lacks
        # observed floor and full-height free evidence under that tabletop.
        self.mapper.reset(replace(self.request, robot_config=replace(
            self.request.robot_config, collision_z_max_m=0.2)))
        short = self.mapper.update(scene_frame(boxes=(table,)))
        self.assertEqual(short.occupancy_map.data[10][8], -1)
        self.assertTrue(short.explored_map.data[10][8])

    def test_rays_above_robot_do_not_clear_space(self):
        frame = scene_frame()
        # Only the top image row contains valid depth, all above the robot.
        high = replace(frame, depth_image=replace(frame.depth_image,
            data=pack('<61f', *([4.] * 61)) + pack('<f', 0.) * (60 * 61)))
        state = self.mapper.update(high)
        self.assertEqual(sum(v == 0 for row in state.occupancy_map.data for v in row), 1)
        self.assertFalse(any(v == 100 for row in state.occupancy_map.data for v in row))

    def test_floor_endpoints_without_full_height_evidence_remain_unknown(self):
        frame = scene_frame()
        floor_only = replace(frame, depth_image=replace(frame.depth_image,
            data=pack('<f', 0.) * (60 * 61) + frame.depth_image.data[-61 * 4:]))
        state = self.mapper.update(floor_only)
        self.assertTrue(any(any(row) for row in self.mapper._state.floor_seen))
        self.assertTrue(any(any(row) for row in state.explored_map.data))
        self.assertEqual(sum(v == 0 for row in state.occupancy_map.data for v in row), 1)

    def test_collision_lower_bound_is_configurable(self):
        box = ((1.4, -0.21, 0.), (1.8, 0.21, 0.18))
        self.mapper.reset(replace(self.request, robot_config=replace(
            self.request.robot_config, collision_z_min_m=0.25)))
        state = self.mapper.update(scene_frame(boxes=(box,)))
        self.assertNotIn(100, state.occupancy_map.data[10][6:9])

    def test_snapshot_metadata_immutability_and_repeated_views(self):
        first = self.mapper.update(scene_frame())
        second = self.mapper.update(scene_frame(sequence=2))
        self.assertEqual(first.occupancy_map.data, second.occupancy_map.data)
        self.assertEqual(first.explored_map.data, second.explored_map.data)
        self.assertEqual((first.revision, second.revision), (1, 2))
        self.assertEqual(second.occupancy_map.revision, second.explored_map.revision)
        self.assertEqual(second.last_processed_sequence, 2)
        self.assertEqual(second.timestamp_s, 2.)
        self.assertEqual(second.robot_pose, scene_frame(sequence=2).robot_pose)
        self.assertEqual(second.target_class, self.request.target_class)
        self.assertEqual((second.semantic_objects, second.target_claim), ((), None))
        self.assertEqual(second.frontiers, ())
        self.assertEqual(first.frontiers, second.frontiers)
        require_immutable(second)
        with self.assertRaises(FrozenInstanceError):
            first.revision = 99
        before = first.occupancy_map.data
        self.mapper.update(scene_frame(sequence=3, boxes=(((1.4, -0.2, 0.), (1.8, 0.2, 0.3)),)))
        self.assertEqual(first.occupancy_map.data, before)
        self.assertIsNot(first.occupancy_map.data, second.occupancy_map.data)

    def test_movement_and_rotation_add_coverage_and_replay_identically(self):
        frames = (scene_frame(), scene_frame(2, x=0.4, yaw=0.5), scene_frame(3, x=0.4, yaw=-0.5))
        states = [self.mapper.update(frame) for frame in frames]
        counts = [sum(sum(row) for row in state.explored_map.data) for state in states]
        self.assertGreater(counts[1], counts[0])
        self.assertGreater(counts[2], counts[1])
        replay = RGBDMapper()
        replay.reset(self.request)
        with tempfile.TemporaryDirectory() as directory:
            for frame, expected in zip(frames, states):
                path = Path(directory) / str(frame.sequence)
                save_rgbd_recording(path, frame, {'source': 'synthetic'})
                self.assertEqual(replay.update(load_rgbd_recording(path)), expected)

    def test_obstacles_persist_until_reset_even_after_clear_view(self):
        first = self.mapper.update(scene_frame(boxes=(((1.4, -0.2, 0.), (1.8, 0.2, 0.3)),)))
        second = self.mapper.update(scene_frame(sequence=2))
        for r, row in enumerate(first.occupancy_map.data):
            for c, value in enumerate(row):
                if value == 100:
                    self.assertEqual(second.occupancy_map.data[r][c], 100)
        self.mapper.reset(self.request)
        self.assertEqual(self.mapper.update(scene_frame()).occupancy_map.data[10][7], 0)

    def test_rejected_frames_are_atomic_and_do_not_consume_sequences(self):
        self.mapper.update(scene_frame())
        valid = scene_frame(sequence=2)
        invalid = (replace(valid, episode_id='other'), replace(valid, sequence=1),
                   replace(valid, timestamp_s=0.),
                   replace(valid, camera_intrinsics=replace(valid.camera_intrinsics, fx=41.)),
                   replace(valid, robot_pose=replace(valid.robot_pose, z_m=0.1)),
                   replace(valid, depth_image=replace(valid.depth_image, data=b'')))
        previous = self.mapper._state
        for frame in invalid:
            with self.subTest(frame=frame.sequence):
                with self.assertRaises(ContractError):
                    self.mapper.update(frame)
                self.assertIs(self.mapper._state, previous)
        self.assertEqual(self.mapper.update(valid).revision, 2)

    def test_floor_height_and_robot_base_elevation_translate_together(self):
        ground = self.mapper.update(scene_frame())
        self.mapper.reset(request(floor_height_m=1.0))
        raised = self.mapper.update(scene_frame(floor_z=1.0))
        self.assertEqual(ground.occupancy_map.data, raised.occupancy_map.data)
        self.assertEqual(ground.explored_map.data, raised.explored_map.data)

    def test_failure_after_partial_computation_is_atomic(self):
        first = self.mapper.update(scene_frame())
        previous = self.mapper._state
        def fail(obs, cfg, collision, occupancy, coverage, masks, floors):
            occupancy[0][0] = 100
            coverage[0][0] = floors[0][0] = True
            masks[0][0] = 1
            raise RuntimeError('injected fusion failure')
        with patch('frontier_search.perception.mapper.fuse_observation', side_effect=fail):
            with self.assertRaisesRegex(RuntimeError, 'injected fusion failure'):
                self.mapper.update(scene_frame(sequence=2))
        self.assertIs(self.mapper._state, previous)
        self.assertEqual(previous.occupancy[0][0], first.occupancy_map.data[0][0])
        self.assertFalse(previous.coverage[0][0])
        self.assertFalse(previous.floor_seen[0][0])
        self.assertEqual(previous.free_masks[0][0], 0)
        self.assertEqual(self.mapper.update(scene_frame(sequence=2)).revision, 2)

    def test_sequence_gaps_and_equal_timestamps_are_accepted(self):
        self.mapper.update(scene_frame())
        state = self.mapper.update(replace(scene_frame(sequence=5), timestamp_s=1.))
        self.assertEqual((state.last_processed_sequence, state.timestamp_s), (5, 1.))

    def test_reset_discards_fused_evidence_and_allows_new_calibration(self):
        self.mapper.update(scene_frame())
        self.mapper.reset(self.request)
        self.assertFalse(any(any(row) for row in self.mapper._state.free_masks))
        self.assertFalse(any(any(row) for row in self.mapper._state.floor_seen))
        frame = scene_frame(width=31, height=31)
        self.assertEqual(self.mapper.update(frame).revision, 1)

    def test_mapping_evidence_configuration_is_validated(self):
        for values in ({'clearance_bin_height_m': 0.}, {'clearance_bin_height_m': float('nan')},
                       {'clearance_bin_height_m': True}, {'depth_surface_margin_m': -0.1},
                       {'depth_surface_margin_m': 0.1}, {'depth_surface_margin_m': float('inf')}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                request(**values)
        for req in (request(clearance_bin_height_m=1e-8),
                    replace(self.request, robot_config=replace(self.request.robot_config,
                                                               collision_z_max_m=0.02))):
            self.mapper.reset(req)
            with self.assertRaises(ContractError):
                self.mapper.update(scene_frame())
            self.assertIsNone(self.mapper._state.latest_state)

    def test_room_rasterization_boundary_winding_and_empty_denominator(self):
        region = RegionPrior('tiny', (Point2(0., 0.), Point2(0.01, 0.),
                                     Point2(0.01, 0.01), Point2(0., 0.01)), None)
        self.mapper.reset(replace(self.request, region_priors=(region,)))
        self.assertIsNone(self.mapper.update(scene_frame()).regions[0].explored_fraction)
        region = RegionPrior('edge', (Point2(0.1, 0.1), Point2(0.3, 0.1),
                                     Point2(0.3, 0.3), Point2(0.1, 0.3)), None)
        expected = rasterize_region(region, self.request.mapping_config)
        self.assertEqual(expected, (GridCell(10, 0), GridCell(10, 1), GridCell(11, 0), GridCell(11, 1)))
        for boundary in (tuple(reversed(region.boundary_world)),
                         region.boundary_world + (region.boundary_world[0],)):
            self.assertEqual(rasterize_region(replace(region, boundary_world=boundary),
                                              self.request.mapping_config), expected)

    def test_out_of_map_rooms_are_rejected_before_fusing(self):
        region = self.request.region_priors[0]
        outside = replace(region, boundary_world=tuple(Point2(p.x_m + 10, p.y_m)
                                                       for p in region.boundary_world))
        self.mapper.reset(replace(self.request, region_priors=(outside,)))
        previous = self.mapper._state
        with self.assertRaisesRegex(ContractError, 'outside configured map'):
            self.mapper.update(scene_frame())
        self.assertIs(previous, self.mapper._state)

    def test_oracle_room_labels_require_explicit_mode(self):
        region = replace(self.request.region_priors[0], room_type='kitchen')
        self.mapper.reset(replace(self.request, region_priors=(region,), oracle_room_labels=True))
        state = self.mapper.update(scene_frame())
        self.assertEqual(state.regions[0].room_type_source, 'simulator_annotation')
        validate_state(state, oracle_room_labels=True)


if __name__ == '__main__':
    unittest.main()
