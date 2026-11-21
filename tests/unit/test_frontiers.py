"""Frontier geometry, identities and owner-boundary regressions."""

from dataclasses import replace
from math import floor, isfinite, pi
import unittest
from unittest.mock import patch

from frontier_search.config import MappingConfig, NavigationConfig, ObservationConfig, PolicyConfig, RecoveryConfig
from frontier_search.contracts import (
    CandidateEvaluation, CandidateEvaluationRequest, GridCell, NavigatorInit,
    OccupancyGrid, PolicyName, Pose,
)
from frontier_search.fixtures.loader import load_policy_fixture
from frontier_search.navigation.costmap import Costmap
from frontier_search.navigation.navigator import TeamNavigator
from frontier_search.perception.frontiers import (
    FrontierCandidate, assign_identities, build_candidates, extract_clusters,
)
from frontier_search.perception.mapper import RGBDMapper
from frontier_search.policies.baselines import BaselinePolicy
from frontier_search.runner.episode import build_context, decide
from frontier_search.runner.recovery import SpatialRecovery
from frontier_search.simulation.mock_adapter import MockSimulator
from frontier_search.validation import validate_state
from tests.unit.test_mapper_update import request, scene_frame


def grid(data, resolution=0.2):
    return OccupancyGrid('world', 1, resolution, 0., 0., len(data[0]), len(data),
                         tuple(tuple(row) for row in data))


def boundary_map():
    data = [[100] * 18 for _ in range(14)]
    for r in range(2, 12):
        for c in range(2, 16):
            data[r][c] = 0 if c < 9 else -1
    return grid(data)


def candidate(cells):
    return FrontierCandidate(tuple(GridCell(r, c) for r, c in sorted(cells)),
                             Pose(1., 1., 0., 0.), None, 1., 1.)


class FrontierGeometryTests(unittest.TestCase):
    def setUp(self):
        self.grid = boundary_map()
        self.mapping = MappingConfig(width_cells=18, height_cells=14, resolution_m=.2)
        self.robot = request().robot_config
        self.frame = scene_frame(x=1., y=1.)

    def candidates(self, **kwargs):
        return build_candidates(kwargs.get('grid', self.grid), kwargs.get('frame', self.frame),
            self.robot, kwargs.get('mapping', self.mapping), ObservationConfig(), kwargs.get('regions', {}))

    def test_boundary_support_interior_pose_and_direction(self):
        clusters = extract_clusters(self.grid, self.mapping)
        self.assertEqual(clusters, (tuple(GridCell(r, 8) for r in range(2, 12)),))
        frontier, = self.candidates()
        p = frontier.approach_pose
        r, c = floor(p.y_m / .2), floor(p.x_m / .2)
        self.assertEqual(self.grid.data[r][c], 0)
        self.assertLess(c, 8)  # enough space for a body, not just its centre
        self.assertAlmostEqual(p.yaw_rad, 0.)
        self.assertTrue(Costmap.from_robot(self.grid, self.robot).pose_valid(p))
        self.assertGreater(frontier.information_gain_m2, 0.)
        self.assertEqual(frontier.support_cells, clusters[0])
        self.assertEqual(self.grid, boundary_map())

    def test_outside_grid_is_not_an_unknown_frontier(self):
        self.assertEqual(extract_clusters(grid([[0] * 4 for _ in range(4)]), self.mapping), ())
        self.assertEqual(extract_clusters(grid([[100, -1], [-1, 100]]), self.mapping), ())

    def test_clustering_connectivity_size_filter_and_deterministic_order(self):
        data = [[-1] * 6 for _ in range(6)]
        data[1][1] = data[2][2] = 0
        cfg = replace(self.mapping, frontier_min_cells=1)
        separate = ((GridCell(1, 1),), (GridCell(2, 2),))
        self.assertEqual(extract_clusters(grid(data), cfg), separate)
        self.assertEqual(extract_clusters(grid(data), replace(cfg, frontier_connectivity=8)),
                         ((GridCell(1, 1), GridCell(2, 2)),))
        self.assertEqual(extract_clusters(grid(data), self.mapping), ())

    def test_regions_use_support_majority_and_stable_tie_breaking(self):
        cells = tuple(GridCell(r, 8) for r in range(2, 12))
        f, = self.candidates(regions={'R2': cells[:5], 'R1': cells[5:]})
        self.assertEqual(f.region_id, 'R1')
        f, = self.candidates(regions={'R2': cells[:6], 'R1': cells[6:]})
        self.assertEqual(f.region_id, 'R2')

    def test_mounted_camera_is_oriented_toward_unknown(self):
        # Keep robot yaw zero while the camera optical axis points +world y.
        frame = replace(scene_frame(x=1., y=1., yaw=pi / 2), robot_pose=self.frame.robot_pose)
        f, = self.candidates(frame=frame)
        self.assertAlmostEqual(f.approach_pose.yaw_rad, -pi / 2)
        self.assertGreater(f.information_gain_m2, 0.)

    def test_equivalent_yaw_does_not_change_approach_geometry(self):
        frame = replace(self.frame, robot_pose=replace(self.frame.robot_pose, yaw_rad=2 * pi))
        self.assertEqual(self.candidates(frame=frame), self.candidates())

    def test_thin_free_strip_does_not_publish_colliding_goal(self):
        data = [[100] * 8 for _ in range(8)]
        for r in range(1, 7):
            data[r][3], data[r][4] = 0, -1
        narrow = grid(data)
        self.assertTrue(extract_clusters(narrow, self.mapping))
        self.assertEqual(self.candidates(grid=narrow), ())

    def test_new_obstacle_reselects_approach_on_updated_map(self):
        original, = self.candidates()
        data = [list(row) for row in self.grid.data]
        p = original.approach_pose
        data[floor(p.y_m / .2)][floor(p.x_m / .2)] = 100
        updated = replace(self.grid, revision=self.grid.revision + 1,
                          data=tuple(tuple(row) for row in data))
        candidates = self.candidates(grid=updated)
        self.assertTrue(candidates)
        for candidate in candidates:
            self.assertNotEqual(candidate.approach_pose, p)
            self.assertTrue(Costmap.from_robot(updated, self.robot).pose_valid(candidate.approach_pose))

    def test_all_frontier_settings_reject_invalid_values(self):
        for changes in ({'frontier_min_cells': 0}, {'frontier_min_cells': True},
                        {'frontier_connectivity': 6}, {'frontier_approach_search_m': float('nan')},
                        {'frontier_match_tolerance_cells': -1}, {'frontier_match_tolerance_cells': 9},
                        {'frontier_match_fraction': 0}, {'frontier_match_fraction': 1.1},
                        {'frontier_history_s': -1}, {'gain_range_m': 0}, {'gain_ray_step_rad': 0}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                replace(self.mapping, **changes)


class FrontierIdentityTests(unittest.TestCase):
    def setUp(self):
        self.cfg = MappingConfig(frontier_match_tolerance_cells=0, frontier_history_s=3.)
        self.whole = candidate((5, c) for c in range(2, 11))
        self.left = candidate((5, c) for c in range(2, 5))
        self.right = candidate((5, c) for c in range(8, 11))
        self.memory, self.lineage, self.counter = {}, {}, 0

    def update(self, candidates, time):
        frontiers, memory, lineage, counter = assign_identities(tuple(candidates), self.memory,
            self.lineage, self.counter, time, self.cfg)
        self.memory, self.lineage, self.counter = memory, lineage, counter
        return frontiers

    def test_same_and_shifted_cluster_preserve_id(self):
        first, = self.update((self.whole,), 1.)
        repeat, = self.update((self.whole,), 100.)
        self.assertEqual(first.frontier_id, repeat.frontier_id)  # active IDs do not expire
        shifted, = self.update((candidate((5, c) for c in range(3, 12)),), 101.)
        self.assertEqual(first.frontier_id, shifted.frontier_id)

    def test_split_merge_and_retired_parent_does_not_return(self):
        first, = self.update((self.whole,), 1.)
        children = self.update((self.right, self.left), 2.)
        self.assertEqual(len(children), 2)
        self.assertTrue(all(f.parent_frontier_ids == (first.frontier_id,) for f in children))
        self.assertNotIn(first.frontier_id, self.memory)
        merged, = self.update((self.whole,), 3.)
        self.assertEqual(merged.parent_frontier_ids, tuple(f.frontier_id for f in children))
        self.assertNotEqual(first.frontier_id, merged.frontier_id)
        repeat, = self.update((self.whole,), 4.)
        self.assertEqual(merged, repeat)
        self.assertIn(first.frontier_id, self.lineage)

    def test_disappearance_reappearance_and_expiration(self):
        first, = self.update((self.whole,), 1.)
        self.assertEqual(self.update((), 2.), ())
        reappeared, = self.update((self.whole,), 3.)
        self.assertEqual(first.frontier_id, reappeared.frontier_id)
        self.update((), 4.)
        later, = self.update((self.whole,), 10.)
        self.assertNotEqual(first.frontier_id, later.frontier_id)
        self.assertEqual(later.parent_frontier_ids, ())
        self.assertIn(first.frontier_id, self.lineage)

    def test_unrelated_regions_never_reuse_ids(self):
        first, = self.update((self.whole,), 1.)
        unrelated, = self.update((candidate(((30, 30), (30, 31))),), 2.)
        self.assertNotEqual(first.frontier_id, unrelated.frontier_id)
        self.assertEqual(unrelated.parent_frontier_ids, ())

    def test_spatial_tolerance_recovers_a_nonoverlapping_shift(self):
        self.cfg = replace(self.cfg, frontier_match_tolerance_cells=1)
        first, = self.update((self.whole,), 1.)
        shifted, = self.update((candidate((6, c) for c in range(2, 11)),), 2.)
        self.assertEqual(first.frontier_id, shifted.frontier_id)

    def test_nearby_exact_matches_do_not_churn_and_input_order_does_not_matter(self):
        self.cfg = replace(self.cfg, frontier_match_tolerance_cells=1)
        a, b = candidate(((2, 2),)), candidate(((3, 3),))
        first = self.update((b, a), 1.)
        repeated = self.update((a, b), 2.)
        self.assertEqual(first, repeated)
        fresh = assign_identities((a, b), {}, {}, 0, 1., self.cfg)[0]
        self.assertEqual(first, fresh)

    def test_history_inputs_are_not_mutated(self):
        self.update((self.whole,), 1.)
        old_memory, old_lineage = dict(self.memory), dict(self.lineage)
        assign_identities((self.left, self.right), self.memory, self.lineage, self.counter, 2., self.cfg)
        self.assertEqual(old_memory, self.memory)
        self.assertEqual(old_lineage, self.lineage)

    def test_actual_lineage_preserves_c_recovery_cap(self):
        first, = self.update((self.whole,), 1.)
        state = replace(load_policy_fixture().state, frontiers=(first,))
        recovery = SpatialRecovery(RecoveryConfig(opportunity_attempt_cap=1))
        recovery.reset(state.episode_id)
        recovery.record_dispatch(state, first.frontier_id)
        children = self.update((self.left, self.right), 2.)
        state = replace(state, frontiers=children)
        evaluation = CandidateEvaluation(state.episode_id, state.revision, state.robot_pose, ())
        exclusions = recovery.exclusions(state, evaluation)
        self.assertEqual({e.frontier_id for e in exclusions}, {f.frontier_id for f in children})
        self.assertTrue(all(e.retry_condition == 'exhausted' and e.attempt_count == 1 for e in exclusions))
        self.update((), 3.)
        merged, = self.update((self.whole,), 4.)
        state = replace(state, frontiers=(merged,))
        self.assertEqual(recovery.exclusions(state, evaluation)[0].retry_condition, 'exhausted')

    def test_extracted_map_boundary_splits_and_merges(self):
        known = boundary_map()
        frame, init = scene_frame(), request()
        cfg = replace(self.cfg, resolution_m=.2)
        def publish(observed, time):
            return self.update(build_candidates(observed, frame, init.robot_config,
                                               cfg, init.observation_config, {}), time)
        parent, = publish(known, 1.)
        data = [list(row) for row in known.data]
        data[6][8] = 100
        children = publish(grid(data), 2.)
        self.assertEqual(len(children), 2)
        self.assertTrue(all(f.parent_frontier_ids == (parent.frontier_id,) for f in children))
        merged, = publish(known, 3.)
        self.assertEqual(merged.parent_frontier_ids, tuple(f.frontier_id for f in children))


class MapperFrontierIntegrationTests(unittest.TestCase):
    def test_rgbd_a_to_real_b_assessment_and_c_selection(self):
        init = request()
        # Explicit synthetic spawn prior: all cells under the development robot
        # are clear, including the cells behind the starting camera.
        init = replace(init, initial_clearance_cells=tuple(GridCell(r, c)
                       for r in range(8, 12) for c in range(2, 8)))
        mapper = RGBDMapper()
        mapper.reset(init)
        state = mapper.update(scene_frame(x=.7))
        validate_state(state)
        self.assertTrue(state.frontiers)
        simulator = MockSimulator(state.occupancy_map, init.robot_config)
        simulator.reset(state.episode_id, 'synthetic_rgbd', state.robot_pose, 0)
        self.addCleanup(simulator.close)
        nav = TeamNavigator(simulator)
        nav.reset(NavigatorInit(state.episode_id, init.robot_config, init.observation_config, NavigationConfig()))
        simulator.pause()
        evaluation = nav.evaluate_candidates(CandidateEvaluationRequest(state.episode_id, state))
        context = build_context(state, evaluation)
        self.assertTrue(context.eligible_frontier_ids)
        rejected = {a.frontier_id for a in evaluation.assessments if a.reachable is False}
        costmap = Costmap.from_robot(state.occupancy_map, init.robot_config)
        self.assertTrue(all(costmap.pose_valid(f.approach_pose) for f in state.frontiers))
        self.assertFalse(rejected.intersection(context.eligible_frontier_ids))
        policy = BaselinePolicy(PolicyName.NEAREST)
        policy.reset(state.episode_id, 0, PolicyConfig())
        result = decide(policy, state, context)
        self.assertIn(result.subgoal.frontier_id, context.eligible_frontier_ids)
        chosen = next(f for f in state.frontiers if f.frontier_id == result.subgoal.frontier_id)
        self.assertEqual(result.subgoal.approach_pose, chosen.approach_pose)
        self.assertEqual(result.subgoal.selected_from_revision, state.revision)
        self.assertTrue(all(isfinite(f.information_gain_m2) and f.information_gain_m2 >= 0
                            for f in state.frontiers))
        policy = BaselinePolicy(PolicyName.INFORMATION_GAIN)
        policy.reset(state.episode_id, 0, PolicyConfig())
        gain_selection = decide(policy, state, context)
        gains = {f.frontier_id: f.information_gain_m2 for f in state.frontiers}
        expected = min((a for a in evaluation.assessments if a.frontier_id in context.eligible_frontier_ids),
                       key=lambda a: (-gains[a.frontier_id] / (a.path_cost_m + .1), a.frontier_id))
        self.assertEqual(gain_selection.subgoal.frontier_id, expected.frontier_id)

    def test_failure_after_identity_allocation_does_not_consume_ids(self):
        mapper = RGBDMapper()
        mapper.reset(request())
        previous = mapper._state
        with patch('frontier_search.perception.mapper.validate_state', side_effect=ValueError('reject')):
            with self.assertRaises(ValueError):
                mapper.update(scene_frame())
        self.assertIs(mapper._state, previous)
        self.assertEqual(previous.entity_counters['frontier'], 0)
        first = mapper.update(scene_frame())
        mapper.reset(request())
        self.assertEqual(mapper.update(scene_frame()), first)
        self.assertTrue(first.frontiers)


if __name__ == '__main__':
    unittest.main()
