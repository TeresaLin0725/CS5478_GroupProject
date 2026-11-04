"""New native entry points and every A candidate builder share the same geometry."""
from dataclasses import replace
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import run_simple_baselines
from frontier_search.contracts import NavigatorInit
from frontier_search.navigation.geometry import footprint_collides
from frontier_search.navigation.navigator import TeamNavigator
from frontier_search.perception import mapper as mapping
from frontier_search.perception.detection import RGBDetection
from frontier_search.runner import integrated
from frontier_search.runner.manifest import load_manifest
from test_mapper_update import request, scene_frame


class SharedGeometryHandoffTests(unittest.TestCase):
    def test_new_factory_preserves_legacy_settings_except_geometry(self):
        old = integrated.manifest_for('same', 'random')
        new = integrated.shared_round_manifest_for('same', 'random')
        self.assertEqual(len(old.robot.footprint_base), 4)
        self.assertEqual(len(new.robot.footprint_base), 16)
        self.assertEqual(new.robot.clearance_margin_m, .01)
        self.assertEqual(replace(new, robot=old.robot), old)
        self.assertEqual(integrated.manifest_for('same', 'random'), old)

    def test_direct_entry_passes_shared_robot_without_b_only_override(self):
        class AtNavigationConstruction(Exception):
            pass

        fake_torch = SimpleNamespace(set_num_threads=Mock(), __version__='test',
            version=SimpleNamespace(cuda=None), cuda=SimpleNamespace(is_available=lambda: False))
        detector = Mock()
        with tempfile.TemporaryDirectory() as td, \
                patch.dict(sys.modules, {'torch': fake_torch}), \
                patch.object(integrated, 'load_local_env'), \
                patch.object(integrated, 'source_fingerprint', return_value='test'), \
                patch.object(integrated, 'DisplayDetector', return_value=detector), \
                patch.object(integrated, 'RetainingAdapter') as adapter, \
                patch.object(integrated, 'TeamNavigator', side_effect=AtNavigationConstruction) as nav, \
                patch.object(sys, 'argv', ['integrated', '--output', td, '--policy', 'nearest']):
            with self.assertRaises(AtNavigationConstruction):
                integrated.main()
            manifest = load_manifest(Path(td) / 'manifest.json')
            self.assertEqual(len(manifest.robot.footprint_base), 16)
            self.assertEqual(adapter.call_args.args[:2], (manifest.robot, manifest.mapping))
            options = nav.call_args.kwargs
            self.assertNotIn('footprint_override', options)
            actual = TeamNavigator(None, **options)
            actual.reset(NavigatorInit(manifest.episode_id, manifest.robot,
                                       manifest.observation, manifest.navigation))
            config = actual.effective_configuration()
            self.assertEqual(config['shared_robot_config'], config['effective_robot_config'])
            self.assertEqual(config['strict_costmap']['planning_margin_m'], .01)
            self.assertEqual(config['strict_costmap']['start_margin_m'], 0.)
            self.assertIsNone(config['relaxed_costmap'])

    def test_simple_suite_freezes_shared_geometry_for_all_three_policies(self):
        with tempfile.TemporaryDirectory() as td, \
                patch('frontier_search.policies.provider.load_local_env'), \
                patch.object(integrated, 'source_fingerprint', return_value='test'), \
                patch.object(run_simple_baselines.shutil, 'copytree'), \
                patch.object(run_simple_baselines.shutil, 'copy2'), \
                patch.object(run_simple_baselines.subprocess, 'check_output', return_value='test\n'):
            protocol = run_simple_baselines.freeze(Path(td))
            self.assertEqual(protocol['navigation_preset'], 'shared_round')
            for job in protocol['jobs']:
                manifest = load_manifest(Path(td) / job['manifest_path'])
                self.assertEqual(len(manifest.robot.footprint_base), 16)
                self.assertEqual(manifest.robot.clearance_margin_m, .01)
                self.assertEqual(manifest.policy_name, job['policy'])
                self.assertEqual(manifest.recovery.epsilon_retry, 0.)
                self.assertFalse(manifest.recovery.stuck_retry)

    def test_mapper_uses_shared_robot_for_all_three_candidate_kinds(self):
        from frontier_search.navigation.locobot_robot import shared_round_robot
        init = request(inspection_distance_m=.9, dynamic_clear_frames=2)
        init = replace(init, robot_config=shared_round_robot(init.robot_config))
        detector = Mock(detect=Mock(return_value=(RGBDetection('cup', .9, (28, 28, 33, 33)),)))
        mapper = mapping.RGBDMapper(detector)
        mapper.reset(init)
        with patch.object(mapping, 'build_candidates', wraps=mapping.build_candidates) as ordinary, \
                patch.object(mapping, '_target_approaches', wraps=mapping._target_approaches) as target, \
                patch.object(mapping, '_navigation_probes', wraps=mapping._navigation_probes) as probe:
            state = mapper.update(scene_frame(x=.7))
        for call, index in ((ordinary, 2), (target, 5), (probe, 2)):
            self.assertIs(call.call_args.args[index], init.robot_config)
        self.assertEqual({f.kind for f in state.frontiers},
                         {'frontier', 'target_approach', 'navigation_probe'})
        for frontier in state.frontiers:
            # Target proposals use geometry for ranking; B still validates their complete footprint.
            if frontier.kind != 'target_approach':
                for pose in (frontier.approach_pose,) + frontier.alternative_approach_poses:
                    self.assertFalse(footprint_collides(state.occupancy_map,
                        init.robot_config.footprint_base, pose, init.robot_config.clearance_margin_m))
        legacy = mapping.RGBDMapper(detector)
        legacy.reset(replace(init, robot_config=request().robot_config))
        before = legacy.update(scene_frame(x=.7))
        self.assertEqual(state.occupancy_map, before.occupancy_map)
        self.assertEqual(state.explored_map, before.explored_map)
        self.assertEqual(state.semantic_objects, before.semantic_objects)


if __name__ == '__main__':
    unittest.main()
