"""Shared benchmark geometry, initialization, and recorded frontier regression."""
from dataclasses import asdict, replace
from collections import deque
import hashlib
import json
from math import cos, hypot, sin
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts'))
from benchmark_episode import navigation_options, preflight_manifest, validate_shared_geometry
from run_paired_benchmark import benchmark_manifest
from frontier_search.config import MappingConfig
from frontier_search.contracts import GridCell, NavigatorInit, OccupancyGrid, Point2, Pose, RobotConfig
from frontier_search.fixtures.clearance import footprint_cells, validate_clearance
from frontier_search.fixtures.sequence import mapper_init
from frontier_search.fixtures.w2_episode import mock_manifest
from frontier_search.navigation.costmap import Costmap
from frontier_search.navigation.locobot_robot import ROUND_FOOTPRINT_OVERRIDE_NAV_OPTIONS, shared_round_robot
from frontier_search.navigation.navigator import TeamNavigator
from frontier_search.navigation.planner import plan
from frontier_search.perception.frontiers import _approach, _clearance, _oriented_pose
from frontier_search.perception.mapper import RGBDMapper
from frontier_search.runner.integrated import manifest_for
from frontier_search.simulation.igibson_adapter import AdapterConfig, IGibsonAdapter
from tests.integration.test_igibson_adapter import FakeBackend, np


class SharedRoundGeometryTests(unittest.TestCase):
    def test_new_manifest_preserves_every_other_robot_setting(self):
        old = manifest_for('template')
        new = benchmark_manifest()
        self.assertEqual(len(new.robot.footprint_base), 16)
        for point in new.robot.footprint_base:
            self.assertAlmostEqual(hypot(point.x_m, point.y_m), .19)
        self.assertEqual(new.robot.clearance_margin_m, .01)
        self.assertEqual(replace(new.robot, footprint_base=old.robot.footprint_base,
                                 clearance_margin_m=old.robot.clearance_margin_m), old.robot)
        self.assertEqual(new.mapping, old.mapping)
        self.assertEqual(new.policy, old.policy)

    def test_presets_preserve_legacy_behavior_and_reject_mislabeled_geometry(self):
        self.assertEqual(navigation_options({}), {})
        self.assertEqual(navigation_options({'navigation_preset': 'legacy'}), {})
        self.assertEqual(navigation_options({'navigation_preset': 'round_footprint_override'}),
                         ROUND_FOOTPRINT_OVERRIDE_NAV_OPTIONS)
        protocol = {'navigation_preset': 'shared_round'}
        self.assertEqual(navigation_options(protocol), {'start_margin_m': 0.})
        validate_shared_geometry(protocol, benchmark_manifest())
        with self.assertRaises(ValueError):
            validate_shared_geometry(protocol, manifest_for('legacy'))
        with self.assertRaises(ValueError):
            navigation_options({'navigation_preset': 'typo'})

    def test_preflight_uses_sealed_manifest_geometry(self):
        manifest = benchmark_manifest()
        with tempfile.TemporaryDirectory() as td:
            path = Path(td) / 'manifest.json'
            path.write_text(json.dumps(asdict(manifest)), encoding='utf-8')
            protocol = dict(navigation_preset='shared_round', run_root=td, jobs=[dict(
                profile='dynamic', manifest_path=path.name,
                manifest_sha256=hashlib.sha256(path.read_bytes()).hexdigest())])
            resolved = preflight_manifest(protocol, 'dynamic')
            self.assertEqual(resolved, replace(manifest, episode_id='oracle-dynamic'))
            path.write_text('{}', encoding='utf-8')
            with self.assertRaisesRegex(ValueError, 'manifest changed'):
                preflight_manifest(protocol, 'dynamic')

    @unittest.skipIf(np is None, 'NumPy required for fake adapter')
    def test_spawn_mapper_and_navigator_use_identical_geometry(self):
        original = mock_manifest()
        manifest = replace(original, robot=shared_round_robot(original.robot))
        adapter = IGibsonAdapter(manifest.robot, manifest.mapping, AdapterConfig(), FakeBackend)
        self.addCleanup(adapter.close)
        adapter.reset(manifest.episode_id, 'Rs_int', manifest.start_pose, manifest.environment_seed)
        cells = adapter.verify_spawn_clearance()
        self.assertEqual(set(cells), set(footprint_cells(manifest)))
        validate_clearance(manifest, cells)
        mapper = RGBDMapper()
        mapper.reset(mapper_init(manifest, cells))
        navigator = TeamNavigator(adapter, **navigation_options({'navigation_preset': 'shared_round'}))
        navigator.reset(NavigatorInit(manifest.episode_id, manifest.robot,
                                      manifest.observation, manifest.navigation))
        self.assertEqual(mapper._state.request.robot_config, navigator._robot)
        grid = OccupancyGrid('world', 1, .1, 0., 0., 40, 40,
                             tuple((0,) * 40 for _ in range(40)))
        costmap = navigator._costmap_for(grid)
        self.assertEqual(costmap.footprint_base, manifest.robot.footprint_base)
        self.assertEqual(costmap.margin_m, manifest.robot.clearance_margin_m)
        self.assertEqual(costmap.start_margin_m, 0.)
        area_manifest = replace(manifest, initial_clearance_mode='verified_start_area',
                                initial_clearance_radius_m=.65)
        verified = adapter.verify_start_area(area_manifest, 'shared-round-regression')
        validate_clearance(area_manifest, verified.cells, verified.verification)
        mapper.reset(mapper_init(area_manifest, verified.cells, verified.verification))
        self.assertEqual(mapper._state.request.robot_config, navigator._robot)
        blocked_backend = type('BlockedBackend', (FakeBackend,),
                               {'boxes': ((1., 1.5, 1.1, 1.6, 0., .5),)})
        blocked = IGibsonAdapter(manifest.robot, manifest.mapping, AdapterConfig(), blocked_backend)
        self.addCleanup(blocked.close)
        blocked.reset(manifest.episode_id, 'Rs_int', manifest.start_pose, manifest.environment_seed)
        with self.assertRaisesRegex(ValueError, 'cannot be verified'):
            blocked.verify_spawn_clearance()

    def test_recorded_f000220_gets_a_safe_approach_and_has_reachable_alternatives(self):
        fixture = json.loads((Path(__file__).resolve().parents[1] / 'fixtures/shared_round_frontier.json').read_text())
        data = tuple(tuple(v for v, count in row for _ in range(count)) for row in fixture['rows_rle'])
        grid = OccupancyGrid(data=data, **fixture['grid'])
        robot_data = dict(fixture['robot'])
        robot_data['footprint_base'] = tuple(Point2(**v) for v in robot_data['footprint_base'])
        old_robot = RobotConfig(**robot_data)
        robot = shared_round_robot(old_robot)
        pose = Pose(**fixture['robot_pose'])
        angle = pose.yaw_rad + fixture['camera_yaw_offset_rad']
        observation = SimpleNamespace(robot_pose=pose, camera_to_world=SimpleNamespace(
            matrix=((0., 0., cos(angle), 0.), (0., 0., sin(angle), 0.),
                    (0., 0., 0., 0.), (0., 0., 0., 1.))))
        support = tuple(GridCell(**v) for v in fixture['frontier']['support_cells'])
        config = MappingConfig(**fixture['mapping'])
        clearance = _clearance(grid)
        self.assertIsNone(_approach(grid, support, clearance, old_robot, observation, config))
        approach = _approach(grid, support, clearance, robot, observation, config)
        self.assertIsNotNone(approach)
        costmap = Costmap.from_robot(grid, robot, start_margin_m=0.)
        self.assertTrue(costmap.pose_valid(approach))
        # Geometry alignment alone does not make A's chosen pose reachable.
        # Check the available approaches separately; B still owns path planning.
        distances = {(cell.row, cell.column): 0 for cell in support}
        pending = deque(distances)
        maximum = int(config.frontier_approach_search_m / grid.resolution_m)
        while pending:
            r, c = pending.popleft()
            if distances[r, c] >= maximum:
                continue
            for dr, dc in ((-1, 0), (0, -1), (0, 1), (1, 0)):
                cell = r + dr, c + dc
                if (0 <= cell[0] < grid.height_cells and 0 <= cell[1] < grid.width_cells
                        and grid.data[cell[0]][cell[1]] == 0 and cell not in distances):
                    distances[cell] = distances[r, c] + 1
                    pending.append(cell)
        middle_r = sum(cell.row for cell in support) / len(support)
        middle_c = sum(cell.column for cell in support) / len(support)
        alternatives = (_oriented_pose(grid, support, middle_r, middle_c, r, c, observation)
                        for r, c in sorted(distances))
        self.assertTrue(any(costmap.pose_valid(candidate) and plan(costmap, pose, candidate).success
                            for candidate in alternatives))
        row = int((approach.y_m - grid.origin_y_m) / grid.resolution_m)
        column = int((approach.x_m - grid.origin_x_m) / grid.resolution_m)
        changed = [list(values) for values in grid.data]
        changed[row][column] = 100
        updated = replace(grid, revision=grid.revision + 1, data=tuple(map(tuple, changed)))
        self.assertFalse(Costmap.from_robot(updated, robot).pose_valid(approach))
        alternative = _approach(updated, support, _clearance(updated), robot, observation, config)
        if alternative is not None:
            self.assertTrue(Costmap.from_robot(updated, robot).pose_valid(alternative))


if __name__ == '__main__':
    unittest.main()
