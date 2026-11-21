"""B's configuration records must describe its actual costmaps without changing behaviour."""
from dataclasses import replace
import json
from math import cos, hypot, pi
import unittest

from frontier_search.config import NavigationConfig, ObservationConfig
from frontier_search.contracts import NavigatorInit
from frontier_search.fixtures.loader import load_navigation_fixture
from frontier_search.navigation.costmap import Costmap
from frontier_search.navigation.locobot_robot import (
    LOCOBOT_DEV_ROBOT, LOCOBOT_ROUND_RADIUS_M, LOCOBOT_ROUND_VERTEX_RADIUS_M,
    ROUND_FOOTPRINT_OVERRIDE_NAV_OPTIONS, round_footprint,
)
from frontier_search.navigation.navigator import (
    NavigatorBoundaryError, NearObstacleLimit, RelaxedRetry, TeamNavigator,
)


class NavigationConfigurationTests(unittest.TestCase):
    def navigator(self, **options):
        nav = TeamNavigator(None, **options)
        request = NavigatorInit('config-check', LOCOBOT_DEV_ROBOT, ObservationConfig(), NavigationConfig())
        nav.reset(request)
        return nav, request

    def test_snapshot_requires_initialization(self):
        with self.assertRaises(NavigatorBoundaryError):
            TeamNavigator(None).effective_configuration()

    def test_round_radius_means_vertex_radius_and_existing_geometry_is_unchanged(self):
        self.assertEqual(LOCOBOT_ROUND_VERTEX_RADIUS_M, .19)
        self.assertEqual(LOCOBOT_ROUND_RADIUS_M, LOCOBOT_ROUND_VERTEX_RADIUS_M)
        polygon = round_footprint()
        self.assertEqual(len(polygon), 16)
        self.assertTrue(all(abs(hypot(p.x_m, p.y_m)-.19) < 1e-12 for p in polygon))
        low, high = Costmap.planning_radius_bounds(polygon)
        self.assertAlmostEqual(low, .19*cos(pi/16))
        self.assertAlmostEqual(high, .19)

    def test_default_uses_shared_square_and_resolves_default_margins(self):
        nav, request = self.navigator()
        config = nav.effective_configuration()
        self.assertFalse(config['footprint_overridden'])
        self.assertEqual(config['shared_robot_config'], config['effective_robot_config'])
        strict = config['strict_costmap']
        self.assertAlmostEqual(strict['planning_radius_m'], hypot(.181, .181))
        self.assertEqual(strict['planning_margin_m'], request.robot_config.clearance_margin_m)
        self.assertEqual(strict['start_margin_m'], strict['planning_margin_m'])
        self.assertIsNone(config['relaxed_costmap'])

    def assert_matches_costmap(self, record, costmap):
        self.assertEqual(record['planning_radius_m'], costmap.planning_radius_m)
        self.assertEqual(record['planning_margin_m'], costmap.margin_m)
        self.assertEqual(record['start_margin_m'], costmap.start_margin_m)
        self.assertEqual(record['planning_disc_vertex_radius_m'], costmap._disc_radius_m)
        self.assertEqual(record['planning_disc_polygon'],
                         [dict(x_m=p.x_m, y_m=p.y_m) for p in costmap._disc])

    def test_round_override_records_actual_geometry_without_changing_shared_robot(self):
        nav, request = self.navigator(**ROUND_FOOTPRINT_OVERRIDE_NAV_OPTIONS)
        config = nav.effective_configuration()
        self.assertTrue(config['footprint_overridden'])
        self.assertEqual(len(request.robot_config.footprint_base), 4)
        self.assertEqual(len(config['shared_robot_config']['footprint_base']), 4)
        self.assertEqual(len(config['effective_robot_config']['footprint_base']), 16)
        self.assertEqual(request.robot_config, LOCOBOT_DEV_ROBOT)
        strict = config['strict_costmap']
        self.assertAlmostEqual(strict['planning_radius_m'], .19)
        self.assertAlmostEqual(strict['planning_disc_vertex_radius_m'], .19/cos(pi/16))
        self.assertEqual((strict['planning_margin_m'], strict['start_margin_m']), (.01, 0.))
        self.assert_matches_costmap(strict, nav._costmap_for(load_navigation_fixture().grid))

    def test_explicit_settings_and_inherited_start_margin_match_real_costmap(self):
        nav, _ = self.navigator(planning_radius_m=.22, planning_margin_m=.03,
                                linear_gain=1.8, smooth_paths=True, near_obstacle_limit=NearObstacleLimit())
        config = nav.effective_configuration()
        strict = config['strict_costmap']
        self.assertEqual((strict['planning_radius_m'], strict['planning_margin_m'], strict['start_margin_m']),
                         (.22, .03, .03))
        self.assertEqual(config['parameter_sources']['start_margin'], 'planning_margin')
        self.assertEqual(config['controller']['linear_gain_per_s'], 1.8)
        self.assertTrue(config['controller']['path_smoothing'])
        self.assertIsNotNone(config['controller']['near_obstacle_limit'])
        self.assert_matches_costmap(strict, nav._costmap_for(load_navigation_fixture().grid))

    def test_relaxed_retry_is_recorded_separately_and_matches_actual_costmap(self):
        for retry in (RelaxedRetry(), RelaxedRetry(planning_radius_m=.188, margin_m=.005, start_margin_m=.002)):
            with self.subTest(retry=retry):
                nav, _ = self.navigator(relaxed_retry=retry, **ROUND_FOOTPRINT_OVERRIDE_NAV_OPTIONS)
                config = nav.effective_configuration()
                grid = load_navigation_fixture().grid
                self.assert_matches_costmap(config['strict_costmap'], nav._costmap_for(grid))
                self.assert_matches_costmap(config['relaxed_costmap'], nav._costmap_for(grid, relaxed=True))
                self.assertEqual(config, nav.effective_configuration())

    def test_reset_event_is_json_serializable_and_emitted_once_per_episode(self):
        nav, request = self.navigator(**ROUND_FOOTPRINT_OVERRIDE_NAV_OPTIONS)
        for episode in ('config-check', 'second-episode'):
            if episode != 'config-check':
                nav.reset(replace(request, episode_id=episode))
            records = [json.loads(detail) for kind, detail in nav.events if kind == 'navigation_configuration']
            self.assertEqual(len(records), 1)
            self.assertEqual(records[0], json.loads(json.dumps(nav.effective_configuration())))
            # Same envelope used by both native entry points' summary.json.
            summary = json.loads(json.dumps({'navigator_events': nav.events}))
            self.assertEqual(json.loads(summary['navigator_events'][0][1]), records[0])

    def test_snapshot_is_detached_and_reading_it_does_not_mutate_navigation(self):
        nav, _ = self.navigator(**ROUND_FOOTPRINT_OVERRIDE_NAV_OPTIONS)
        before_events = list(nav.events)
        config = nav.effective_configuration()
        config['effective_robot_config']['footprint_base'][0]['x_m'] = 99.
        config['strict_costmap']['planning_margin_m'] = 99.
        fresh = nav.effective_configuration()
        self.assertEqual(fresh['strict_costmap']['planning_margin_m'], .01)
        self.assertLess(fresh['effective_robot_config']['footprint_base'][0]['x_m'], 1.)
        self.assertEqual(nav.events, before_events)
        self.assertIsNone(nav._active)
        self.assertEqual(nav._ops, {})


if __name__ == '__main__':
    unittest.main()
