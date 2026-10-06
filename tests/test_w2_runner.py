"""Lifecycle acceptance tests using C's authored mapper and B's actual navigator."""
from dataclasses import asdict, replace
import json
from pathlib import Path
import tempfile
import time
import unittest

from frontier_search.config import EpisodeLimits
from frontier_search.contracts import (CandidateAssessment, FailureReason, NavigationStatus,
    ObjectTrack, Point3, TargetClaim, EvidenceBundle, EvaluationResult, DetectionEvidence)
from frontier_search.fixtures.w2_episode import build_mock_runner, mock_manifest
from frontier_search.policies.baselines import BaselinePolicy
from frontier_search.runner.episode import TerminationReason
from frontier_search.runner.manifest import load_manifest
from frontier_search.runner.supervisor import run_supervised


class MemoryLogger:
    def __init__(self):
        self.events = []

    def write(self, event, episode_id, **payload):
        self.events.append(dict(event=event, episode_id=episode_id, **payload))


def hanging_factory(manifest, logger=None):
    runner = build_mock_runner(manifest, logger)
    def hung_update(obs):
        time.sleep(60)  # parent must kill this process, not wait for completion
    runner.mapper.update = hung_update
    return runner


def make_runner(scenario='progress', **overrides):
    manifest = mock_manifest(scenario)
    observation = replace(manifest.observation, initial_headings_rad=(0.,),
                          arrival_headings_rad=(0.,), recovery_headings_rad=(0.,), dwell_s=0.)
    manifest = replace(manifest, observation=observation, **overrides)
    logger = MemoryLogger()
    runner = build_mock_runner(manifest, logger)
    return manifest, runner, logger


class RunnerTests(unittest.TestCase):
    def test_two_dispatches_continuous_mapping_distance_and_terminal(self):
        m, runner, log = make_runner()
        self.assertEqual(runner.run(m), TerminationReason.NO_FRONTIERS)
        self.assertEqual(runner.summary['attempts'], 2)
        self.assertEqual(runner.summary['collisions'], 0)
        self.assertGreater(runner.summary['actual_distance_m'], 1.7)
        self.assertLess(runner.summary['actual_distance_m'], 2.)
        events = log.events
        self.assertEqual(sum(e['event'] == 'episode_terminal' for e in events), 1)
        self.assertEqual(events[-1]['event'], 'episode_terminal')
        self.assertEqual(sum(e['event'] == 'dispatch' for e in events), 2)
        self.assertTrue(all(e['new_known_area_m2'] > 0 for e in events if e['event'] == 'progress'))
        initial_capture = next(i for i, e in enumerate(events) if e['event'] == 'capture_processed')
        initial_motion = next(i for i, e in enumerate(events) if e['event'] == 'observation_start')
        self.assertLess(initial_capture, initial_motion)
        self.assertAlmostEqual(runner.summary['actual_distance_m'], runner.simulator.executed_distance_m)
        self.assertTrue(runner.simulator._closed)

    def test_decisions_are_paused_and_scan_samples_have_distinct_sequences(self):
        m, runner, log = make_runner()
        m = replace(m, observation=replace(m.observation, processed_samples_per_heading=2))
        select = runner.policy.select
        def checked_select(state, context):
            self.assertTrue(runner.simulator._paused)
            self.assertEqual(state.robot_pose, runner.simulator.current_pose())
            self.assertEqual(state.revision, context.evaluation.state_revision)
            return select(state, context)
        runner.policy.select = checked_select
        self.assertEqual(runner.run(m), TerminationReason.NO_FRONTIERS)
        barriers = [e for e in log.events if e['event'] == 'scan_barrier']
        self.assertGreaterEqual(len(barriers), 3)
        self.assertTrue(all(len(set(e['sequences'])) == 2 for e in barriers))
        sequences = [e['sequence'] for e in log.events if e['event'] == 'capture_processed']
        self.assertEqual(sequences, sorted(set(sequences)))

    def test_no_frontiers_gets_one_recovery_scan(self):
        m, runner, log = make_runner('no_frontiers')
        self.assertEqual(runner.run(m), TerminationReason.NO_FRONTIERS)
        routines = [e['request'].routine for e in log.events if e['event'] == 'observation_start']
        self.assertEqual(routines, ['initial', 'recovery'])
        self.assertEqual(runner.summary['attempts'], 0)

    def test_no_progress_is_bounded(self):
        m, runner, log = make_runner('no_progress')
        self.assertEqual(runner.run(m), TerminationReason.NO_FEASIBLE_FRONTIERS)
        self.assertEqual(runner.summary['attempts'], m.recovery.no_progress_cap)
        self.assertTrue(all(e['new_known_area_m2'] == 0 for e in log.events if e['event'] == 'progress'))

    def test_hidden_blockage_preserves_successful_plan_and_caps_retries(self):
        m, runner, log = make_runner('blocked')
        self.assertEqual(runner.run(m), TerminationReason.NO_FEASIBLE_FRONTIERS)
        failures = [e['feedback'] for e in log.events if e['event'] == 'operation_terminal'
                    and e['kind'] == 'navigate']
        self.assertEqual(len(failures), m.recovery.navigation_retry_cap)
        self.assertTrue(all(f.status == NavigationStatus.FAILED and f.planner_result.success
                            and f.failure_reason == FailureReason.BLOCKED for f in failures))

    def test_step_and_attempt_budgets(self):
        for limits in (EpisodeLimits(max_steps=2), EpisodeLimits(max_attempts=1)):
            with self.subTest(limits=limits):
                m, runner, log = make_runner(limits=limits)
                self.assertEqual(runner.run(m), TerminationReason.SEARCH_LIMIT)
                self.assertLessEqual(runner.summary['steps'], limits.max_steps)
                self.assertLessEqual(runner.summary['attempts'], limits.max_attempts)
                self.assertTrue(runner.simulator._closed)

    def test_external_cancel_stops_motion(self):
        m, runner, log = make_runner()
        runner.cancelled = lambda: runner.steps >= 8
        self.assertEqual(runner.run(m), TerminationReason.EXTERNALLY_CANCELLED)
        self.assertEqual((runner.simulator._v, runner.simulator._w), (0, 0))
        self.assertTrue(runner.simulator._closed)

    def test_mapper_old_sequence_and_changed_geometry_are_rejected(self):
        for mode in ('old_sequence', 'geometry', 'revision', 'target', 'parent'):
            with self.subTest(mode=mode):
                m, runner, log = make_runner()
                update = runner.mapper.update
                def bad_update(obs):
                    state = update(obs)
                    if obs.sequence < 2:
                        return state
                    if mode == 'old_sequence':
                        return replace(state, last_processed_sequence=obs.sequence-1)
                    if mode == 'geometry':
                        return replace(state, occupancy_map=replace(state.occupancy_map, origin_x_m=1.),
                                       explored_map=replace(state.explored_map, origin_x_m=1.))
                    if mode == 'revision':
                        return replace(state, revision=1, occupancy_map=replace(state.occupancy_map, revision=1),
                                       explored_map=replace(state.explored_map, revision=1))
                    if mode == 'parent':
                        return replace(state, frontiers=(replace(state.frontiers[0], parent_frontier_ids=('future',)),))
                    return replace(state, target_class='other')
                runner.mapper.update = bad_update
                self.assertEqual(runner.run(m), TerminationReason.PIPELINE_ERROR)
                self.assertEqual(runner.summary['attempts'], 0)

    def test_old_b_evaluation_cannot_dispatch(self):
        m, runner, log = make_runner()
        evaluate = runner.navigator.evaluate_candidates
        runner.navigator.evaluate_candidates = lambda request: replace(evaluate(request), state_revision=0)
        self.assertEqual(runner.run(m), TerminationReason.PIPELINE_ERROR)
        self.assertFalse(any(e['event'] == 'dispatch' for e in log.events))

    def test_drift_after_selection_reselects_without_dispatch(self):
        m, runner, log = make_runner(limits=EpisodeLimits(max_decisions=2))
        select = runner.policy.select
        def drifting(state, context):
            result = select(state, context)
            runner.simulator._pose = replace(runner.simulator._pose, yaw_rad=runner.simulator._pose.yaw_rad+.2)
            return result
        runner.policy.select = drifting
        self.assertEqual(runner.run(m), TerminationReason.SEARCH_LIMIT)
        self.assertEqual(runner.summary['attempts'], 0)
        self.assertEqual(sum(e['event'] == 'stale_dispatch' for e in log.events), 2)

    def test_evaluation_timeout_is_inconclusive_and_bounded(self):
        m, runner, log = make_runner()
        evaluate = runner.navigator.evaluate_candidates
        def timeout(request):
            value = evaluate(request)
            return replace(value, assessments=tuple(CandidateAssessment(f.frontier_id, None, None,
                FailureReason.TIMEOUT) for f in request.state.frontiers))
        runner.navigator.evaluate_candidates = timeout
        self.assertEqual(runner.run(m), TerminationReason.NO_FEASIBLE_FRONTIERS)
        self.assertEqual(runner.summary['attempts'], 0)
        decisions = [e for e in log.events if e['event'] == 'decision']
        self.assertTrue(all(a.reachable is None for e in decisions for a in e['context'].evaluation.assessments))

    def test_missing_scan_marks_do_not_count_as_completed_observation(self):
        m, runner, log = make_runner()
        runner.navigator.capture_marks = lambda goal_id: ()
        self.assertEqual(runner.run(m), TerminationReason.PIPELINE_ERROR)
        self.assertEqual(runner.summary['decisions'], 0)

    def test_spawn_rejection_and_cleanup_error_visible(self):
        m, runner, log = make_runner()
        runner.simulator.verify_spawn_clearance = lambda: ()
        self.assertEqual(runner.run(m), TerminationReason.INITIALIZATION_FAILED)
        self.assertTrue(runner.simulator._closed)
        m, runner, log = make_runner('no_frontiers')
        close = runner.simulator.close
        def broken_close():
            close()
            raise RuntimeError('cleanup failed')
        runner.simulator.close = broken_close
        self.assertEqual(runner.run(m), TerminationReason.PIPELINE_ERROR)
        self.assertTrue(any(e['event'] == 'cleanup_error' for e in log.events))

    def test_llm_unconfigured_fallback_is_logged(self):
        m, runner, log = make_runner(policy_name='llm_guided')
        self.assertEqual(runner.run(m), TerminationReason.NO_FRONTIERS)
        calls = [e['record'] for e in log.events if e['event'] == 'llm_call']
        self.assertEqual(len(calls), 2)
        self.assertTrue(all(c['fallback_reason'] == 'unconfigured_client' for c in calls))

    def test_reusing_runner_resets_episode_history(self):
        m, runner, log = make_runner('no_frontiers')
        self.assertEqual(runner.run(m), TerminationReason.NO_FRONTIERS)
        self.assertEqual(runner.run(replace(m, episode_id='next')), TerminationReason.NO_FRONTIERS)
        self.assertEqual(runner.recovery.episode_id, 'next')

    def test_claim_preempts_policy_scores_once_and_never_resumes_on_false_claim(self):
        for valid, expected in ((True, TerminationReason.SUCCESS), (False, TerminationReason.FALSE_TARGET_CLAIM),
                                (None, TerminationReason.EVALUATION_ERROR)):
            with self.subTest(valid=valid):
                m, runner, log = make_runner(enable_target_evaluation=True)
                update = runner.mapper.update
                def claiming(obs):
                    state = update(obs)
                    if obs.sequence < 3:
                        return state
                    ids = ('e1', 'e2')
                    obj = ObjectTrack('track', 'cup', .99, Point3(1, 1, .5), 0., obs.timestamp_s, ids, 2)
                    return replace(state, semantic_objects=(obj,),
                        target_claim=TargetClaim('claim', 'track', 'cup', obs.timestamp_s, ids))
                runner.mapper.update = claiming
                runner.mapper.get_claim_evidence = lambda episode, claim: EvidenceBundle(episode, claim,
                    tuple(DetectionEvidence('e%d' % seq, seq, 0., 'track', 'cup', .99,
                          (0, 0, 1, 1), Point3(1, 1, .5)) for seq in (1, 2)))
                class Evaluator:
                    calls = 0
                    def reset(self, episode_id, config):
                        pass
                    def evaluate(self, request):
                        self.calls += 1
                        if valid is None:
                            raise RuntimeError('verifier failed')
                        return EvaluationResult(request.episode_id, request.claim.claim_id, valid,
                                                'hidden-instance' if valid else None, 'fixture')
                evaluator = Evaluator()
                runner._evaluator = evaluator
                self.assertEqual(runner.run(m), expected)
                self.assertEqual(evaluator.calls, 1)
                self.assertEqual(runner.summary['decisions'], 0)
                self.assertEqual((runner.simulator._v, runner.simulator._w), (0, 0))
                self.assertFalse(any(e['event'] == 'decision' for e in log.events))

    def test_simulator_clock_violation_and_wrong_feedback_are_errors(self):
        for mode in ('clock', 'feedback'):
            with self.subTest(mode=mode):
                m, runner, log = make_runner()
                if mode == 'clock':
                    runner.simulator.step = lambda: None
                else:
                    poll = runner.navigator.poll
                    runner.navigator.poll = lambda episode, goal: replace(poll(episode, goal), goal_id='wrong')
                self.assertEqual(runner.run(m), TerminationReason.PIPELINE_ERROR)

    def test_runner_operation_deadline_cancels_navigation(self):
        m, runner, log = make_runner()
        m = replace(m, navigation=replace(m.navigation, navigation_deadline_s=.1))
        tick = runner.navigator.on_step
        # A faulty navigator keeps returning ACCEPTED and enforces no deadline.
        def stalled_tick():
            if runner.navigator._active is not None and runner.navigator._active.kind == 'navigate':
                return
            tick()
        runner.navigator.on_step = stalled_tick
        self.assertEqual(runner.run(m), TerminationReason.SEARCH_LIMIT)
        self.assertTrue(any(e['event'] == 'operation_timeout' for e in log.events))
        self.assertTrue(runner.simulator._closed)

    def test_failed_arrival_observation_cannot_repeat_unboundedly(self):
        m, runner, log = make_runner('no_progress')
        start = runner.navigator.start_observation
        def fail_arrival(request):
            feedback = start(request)
            if request.routine != 'initial':
                feedback = runner.navigator.cancel(request.episode_id, request.goal_id, 'test_failure')
                return replace(feedback, status=NavigationStatus.FAILED, failure_reason=FailureReason.BLOCKED)
            return feedback
        runner.navigator.start_observation = fail_arrival
        self.assertEqual(runner.run(m), TerminationReason.NO_FEASIBLE_FRONTIERS)
        self.assertLessEqual(runner.summary['attempts'], m.recovery.navigation_retry_cap)


class ManifestAndSupervisorTests(unittest.TestCase):
    def test_json_roundtrip_and_invalid_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'manifest.json'
            value = mock_manifest()
            path.write_text(json.dumps(asdict(value)), encoding='utf-8')
            self.assertEqual(load_manifest(path), value)
            data = asdict(value)
            data['typo_field'] = True
            path.write_text(json.dumps(data), encoding='utf-8')
            with self.assertRaises(ValueError):
                load_manifest(path)
        for m in (replace(value, module_deadline_s=0), replace(value, capture_every_steps=0),
                  replace(value, observation=replace(value.observation, arrival_headings_rad=()))):
            with self.assertRaises(ValueError):
                m.validate()

    def test_supervised_run_writes_one_terminal_and_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'run.jsonl'
            result = run_supervised(mock_manifest('no_frontiers'), path)
            self.assertEqual(result['termination'], 'no_frontiers')
            events = [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()]
            self.assertEqual(sum(e['event'] == 'episode_terminal' for e in events), 1)
            self.assertEqual(events[-1]['event'], 'episode_terminal')
            with self.assertRaises(FileExistsError):
                run_supervised(mock_manifest(), path)

    def test_hung_mapper_is_killed_and_failure_logged(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest = replace(mock_manifest(), module_deadline_s=.8)
            started = time.monotonic()
            result = run_supervised(manifest, Path(directory)/'hang.jsonl',
                                    'test_w2_runner:hanging_factory')
            self.assertEqual(result['termination'], 'pipeline_error')
            self.assertEqual(result['detail'], 'module_timeout:mapper.update')
            self.assertLess(time.monotonic()-started, 5.)

    def test_hard_wall_budget_covers_hanging_modules(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest = replace(mock_manifest(), limits=EpisodeLimits(max_wall_time_s=.8))
            result = run_supervised(manifest, Path(directory)/'wall.jsonl',
                                    'test_w2_runner:hanging_factory')
            self.assertEqual(result['termination'], 'search_limit')
            self.assertEqual(result['detail'], 'hard_wall_deadline')


if __name__ == '__main__':
    unittest.main()
