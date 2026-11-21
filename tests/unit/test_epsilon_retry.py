"""C-side recovery from over-conservative exclusions: epsilon_retry and stuck_retry (both opt-in, default off).

The mapper below puts a wall between the robot and the only frontier, so B assesses it NO_PATH (excluded until the
map changes, which a trapped robot never produces). Probing the excluded frontier is bounded by the retry caps.
"""
from dataclasses import replace
from types import SimpleNamespace
import unittest

from frontier_search.contracts import FailureReason, Frontier, FrontierExclusion, GridCell, Pose
from frontier_search.navigation.locobot_robot import LOCOBOT_DEV_ROBOT
from frontier_search.navigation.navigator import RelaxedRetry, TeamNavigator
from frontier_search.runner.episode import TerminationReason
from frontier_search.runner.recovery import SpatialRecovery, probe_candidates
from tests.integration.test_w2_runner import make_runner


def walled_runner(gap_cells=0, relaxed=False, robot=None, **recovery):
    manifest, runner, log = make_runner(**({'robot': robot} if robot else {}))
    manifest = replace(manifest, recovery=replace(manifest.recovery, **recovery))
    update = runner.mapper.update

    def walled(obs):
        state = update(obs)
        cells = [list(row) for row in state.occupancy_map.data]
        for r in range(5, 35):
            cells[r][16] = 100
        low = 20 - gap_cells // 2
        for r in range(low, low + gap_cells):
            cells[r][16] = 0
        return replace(state, occupancy_map=replace(state.occupancy_map, data=tuple(map(tuple, cells))))
    runner.mapper.update = walled
    if relaxed:
        runner.navigator = TeamNavigator(runner.simulator, relaxed_retry=RelaxedRetry())
    return manifest, runner, log


def probes(log):
    return [e for e in log.events if e['event'] == 'epsilon_probe']


class ProbeCandidateTests(unittest.TestCase):
    def test_nearest_first_and_exhausted_or_probe_kind_skipped(self):
        far = Frontier('F9', (), (GridCell(1, 1),), Pose(3, 1.5, 0, 0), None, 1.0, 3.0)
        near = Frontier('F2', (), (GridCell(1, 2),), Pose(1, 1.5, 0, 0), None, 1.0, 1.0)
        done = Frontier('F3', (), (GridCell(1, 3),), Pose(1, 1.6, 0, 0), None, 1.0, 0.5)
        alt = Frontier('F4', (), (GridCell(1, 4),), Pose(1, 1.7, 0, 0), None, 1.0, 0.1, kind='navigation_probe')
        state = SimpleNamespace(frontiers=(far, near, done, alt))
        exclusions = tuple(FrontierExclusion(f.frontier_id, 'no_path', 1, cond, None)
                           for f, cond in ((far, 'evidence_change'), (near, 'evidence_change'),
                                           (done, 'exhausted'), (alt, 'cooldown')))
        recovery = SimpleNamespace(can_probe=lambda state, frontier: True)
        self.assertEqual([f.frontier_id for f in probe_candidates(state, exclusions, recovery)], ['F2', 'F9'])

    def test_cooldown_and_unknown_restrictions_are_not_probe_candidates(self):
        f = Frontier('F1', (), (GridCell(1, 1),), Pose(1, 1, 0, 0), None, 1., 1.)
        state = SimpleNamespace(frontiers=(f,))
        recovery = SimpleNamespace(can_probe=lambda state, frontier: True)
        for reason, condition in (('blocked', 'cooldown'), ('timeout', 'cooldown'),
                                  ('no_progress', 'cooldown_or_change'), ('no_path', 'cooldown'),
                                  ('blocked', 'evidence_change')):
            with self.subTest(reason=reason, condition=condition):
                e = FrontierExclusion('F1', reason, 0, condition, 999.)
                self.assertEqual(probe_candidates(state, (e,), recovery), [])

    def test_recovery_without_full_restriction_check_cannot_probe(self):
        f = Frontier('F1', (), (GridCell(1, 1),), Pose(1, 1, 0, 0), None, 1., 1.)
        e = FrontierExclusion('F1', 'no_path', 0, 'evidence_change', None)
        self.assertEqual(probe_candidates(SimpleNamespace(frontiers=(f,)), (e,), object()), [])


class EpisodeProbeTests(unittest.TestCase):
    def test_default_ends_with_no_feasible_frontiers_and_never_probes(self):
        manifest, runner, log = walled_runner()
        self.assertEqual(runner.run(manifest), TerminationReason.NO_FEASIBLE_FRONTIERS)
        self.assertEqual(probes(log), [])

    def test_stuck_retry_probes_the_excluded_frontier_and_stays_bounded(self):
        manifest, runner, log = walled_runner(stuck_retry=True)
        self.assertEqual(runner.run(manifest), TerminationReason.NO_FEASIBLE_FRONTIERS)
        found = probes(log)
        self.assertTrue(found and all(e['mode'] == 'stuck' for e in found))
        self.assertLessEqual(runner.summary['attempts'], manifest.recovery.opportunity_attempt_cap)
        actions = [e for e in log.events if e['event'] == 'action_selected']
        dispatches = [e for e in log.events if e['event'] == 'dispatch']
        self.assertEqual(len(actions), len(dispatches))
        self.assertTrue(all(e['action_source'] == 'stuck_retry' for e in actions))
        self.assertEqual([e['selection'].subgoal for e in actions],
                         [e['request'].subgoal for e in dispatches])
        replans = [e for e in log.events if e['event'] == 'strategy_replan']
        self.assertTrue(replans)
        self.assertTrue(all(e['action_source'] == 'stuck_retry' for e in replans))

    def test_stuck_probe_plus_relaxed_retry_escapes_a_gap_the_strict_costmap_rejects(self):
        # Locobot square footprint: strict needs about 0.55 m, the 0.5 m gap only opens for the relaxed plan.
        moved = {}
        for name, relaxed, stuck in (('strict', False, False), ('probe_only', False, True), ('probe_relaxed', True, True)):
            manifest, runner, _ = walled_runner(gap_cells=5, relaxed=relaxed, robot=LOCOBOT_DEV_ROBOT,
                                                stuck_retry=stuck)
            runner.run(manifest)
            moved[name] = runner.summary['actual_distance_m']
            self.assertEqual(runner.summary['collisions'], 0)
        self.assertEqual((moved['strict'], moved['probe_only']), (0.0, 0.0))
        self.assertGreater(moved['probe_relaxed'], 1.0)

    def test_epsilon_does_not_probe_when_nothing_is_excluded(self):
        manifest, runner, log = make_runner()
        manifest = replace(manifest, recovery=replace(manifest.recovery, epsilon_retry=1.0))
        runner.run(manifest)
        self.assertEqual(probes(log), [])  # nothing excluded in a clean run: nothing to probe

    def mixed_choice(self, restriction, policy='nearest'):
        manifest, runner, log = make_runner(policy_name=policy)
        manifest = replace(manifest, recovery=replace(manifest.recovery, epsilon_retry=1.))
        update = runner.mapper.update
        def two_choices(obs):
            state = update(obs)
            f = state.frontiers[0]
            other = replace(f, frontier_id='OTHER', support_cells=(GridCell(8, 8),),
                            approach_pose=Pose(1.5, 1.55, 0., 0.), distance_from_robot_m=.45)
            return replace(state, frontiers=(f, other))
        runner.mapper.update = two_choices
        class RestrictedRecovery(SpatialRecovery):
            def record_evaluation(self, state, evaluation):
                if restriction == 'blocked':
                    self.record_observation_failure(state, state, 'F1')
                elif restriction == 'no_progress':
                    self.record_progress(state, state, 'F1')
                elif restriction == 'timeout':
                    timed = tuple(replace(a, reachable=None, path_cost_m=None, failure_reason=FailureReason.TIMEOUT)
                                  if a.frontier_id == 'F1' else a for a in evaluation.assessments)
                    super().record_evaluation(state, replace(evaluation, assessments=timed))
                geometry = tuple(replace(a, reachable=False, path_cost_m=None, failure_reason=FailureReason.NO_PATH)
                                 if a.frontier_id == 'F1' else a for a in evaluation.assessments)
                super().record_evaluation(state, replace(evaluation, assessments=geometry))
        runner.recovery = RestrictedRecovery(manifest.recovery)
        runner.cancelled = lambda: any(e['event'] == 'dispatch' for e in log.events)
        self.assertEqual(runner.run(manifest), TerminationReason.EXTERNALLY_CANCELLED)
        return log

    def test_geometry_probe_can_replace_a_policy_choice_for_each_policy(self):
        for policy in ('random', 'nearest', 'information_gain', 'llm_guided'):
            with self.subTest(policy=policy):
                log = self.mixed_choice('no_path', policy)
                decision = next(e for e in log.events if e['event'] == 'decision')
                action = next(e for e in log.events if e['event'] == 'action_selected')
                dispatch = next(e for e in log.events if e['event'] == 'dispatch')
                self.assertEqual(decision['selection'].subgoal.frontier_id, 'OTHER')
                self.assertEqual(action['policy_selection'], decision['selection'])
                self.assertEqual(action['action_source'], 'epsilon_retry')
                self.assertEqual(action['selection'].subgoal, dispatch['request'].subgoal)
                self.assertEqual(dispatch['request'].subgoal.frontier_id, 'F1')

    def test_epsilon_does_not_override_mixed_active_cooldowns(self):
        for restriction in ('blocked', 'timeout', 'no_progress'):
            with self.subTest(restriction=restriction):
                log = self.mixed_choice(restriction)
                self.assertEqual(probes(log), [])
                action = next(e for e in log.events if e['event'] == 'action_selected')
                self.assertEqual(action['action_source'], 'policy')
                self.assertEqual(action['selection'].subgoal.frontier_id, 'OTHER')

    def test_geometry_probe_waits_for_existing_blockage_cooldown(self):
        for options in ({'stuck_retry': True}, {'epsilon_retry': 1.}):
            with self.subTest(options=options):
                manifest, runner, log = walled_runner(cooldown_s=5., **options)
                class CooldownRecovery(SpatialRecovery):
                    def reset(self, episode_id):
                        super().reset(episode_id)
                        self.seeded = False
                    def record_evaluation(self, state, evaluation):
                        if not self.seeded:
                            self.record_observation_failure(state, state, state.frontiers[0].frontier_id)
                            self.seeded = True
                        super().record_evaluation(state, evaluation)
                runner.recovery = CooldownRecovery(manifest.recovery)
                self.assertEqual(runner.run(manifest), TerminationReason.NO_FEASIBLE_FRONTIERS)
                wait = next(e for e in log.events if e['event'] == 'cooldown_wait')
                dispatch = next(e for e in log.events if e['event'] == 'dispatch')
                self.assertGreaterEqual(dispatch['simulated_time_s'], wait['until_s'])
                self.assertTrue(probes(log))

    def test_epsilon_draws_are_reproducible_for_a_seed(self):
        runs = []
        for _ in range(2):
            manifest, runner, log = walled_runner(epsilon_retry=0.5, stuck_retry=True)
            runner.run(manifest)
            runs.append([(e['mode'], e['frontier_id']) for e in probes(log)])
        self.assertEqual(runs[0], runs[1])

    def test_invalid_probabilities_are_rejected(self):
        manifest, _, _ = make_runner()
        for bad in (-0.1, 1.5):
            with self.assertRaises(Exception):
                replace(manifest, recovery=replace(manifest.recovery, epsilon_retry=bad)).__post_init__()


if __name__ == '__main__':
    unittest.main()
