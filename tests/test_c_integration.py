from dataclasses import replace
import json
import os
from pathlib import Path
import subprocess
import tempfile
from time import monotonic
import unittest
from unittest.mock import patch

from frontier_search.config import PolicyConfig
from frontier_search.contracts import PolicyName
from frontier_search.experiments.logging import dumps
from frontier_search.fixtures.three_rooms import initial_state, assess, FixtureEvaluator
from frontier_search.policies.baselines import BaselinePolicy
from frontier_search.policies.llm import LLMGuidedPolicy
from frontier_search.policies.provider import GeminiCompatibleClient, parse_response, load_local_env
from frontier_search.runner.episode import build_context, decide
from frontier_search.runner.synthetic import run_case


class IntegrationTests(unittest.TestCase):
    def test_both_synthetic_cases(self):
        for name, ids in [('nearest', ['F1', 'F2', 'F3']), ('llm_guided', ['F3'])]:
            summary, events = run_case(name)
            self.assertEqual(summary['termination'], 'success')
            selections = [e['selection'].subgoal.frontier_id for e in events
                          if e['event'] == 'fixture_decision' and e['selection'].subgoal]
            self.assertEqual(selections, ids)
            self.assertEqual(summary['executed_fixture_distance_m'], 9.0)
            states = [e['state'] for e in events if e['event'] == 'fixture_arrival_scan']
            self.assertEqual([s.revision for s in states], list(range(2, len(states) + 2)))
            self.assertEqual(len([e for e in events if e['event'] == 'fixture_terminal']), 1)
            json.loads(dumps(events))

    def test_false_claim_is_terminal_not_success(self):
        summary, events = run_case('llm_guided', evaluator=FixtureEvaluator(False))
        self.assertEqual(summary['termination'], 'false_target_claim')
        self.assertEqual(summary['replay_calls'], 1)
        self.assertEqual(events[-2]['event'], 'fixture_evaluation')

    def test_invalid_llm_falls_back_and_no_truth_in_payload(self):
        class Client:
            def select_id(self, payload, **kwargs):
                serialized = dumps(payload)
                for forbidden in ('occupancy_map', 'synthetic-target', 'valid_target', 'evidence'):
                    self_test.assertNotIn(forbidden, serialized)
                self_test.assertTrue(all(r['room_type'] is None for r in payload['regions']))
                return 'invented'
        self_test = self
        summary, _ = run_case('llm_guided', client=Client())
        self.assertEqual(summary['termination'], 'success')
        # The final decision has one eligible frontier and now bypasses the model.
        self.assertEqual(summary['fallbacks'], 2)

    def test_evaluator_error_is_not_success(self):
        class BrokenEvaluator:
            def evaluate(self, state, evidence):
                raise RuntimeError('test')
        summary, _ = run_case('llm_guided', evaluator=BrokenEvaluator())
        self.assertEqual(summary['termination'], 'evaluation_error')

    def test_path_cost_changes_choice_and_unknown_blocks(self):
        state = initial_state('test')
        grid = [list(row) for row in state.occupancy_map.data]
        grid[1][4] = -1
        blocked = replace(state, occupancy_map=replace(state.occupancy_map,
                          data=tuple(tuple(row) for row in grid)))
        evaluations = assess(blocked)
        self.assertEqual([a.reachable for a in evaluations.assessments], [True, False, False])
        context = build_context(state, assess(state))
        costs = tuple(replace(a, path_cost_m=20.0) if a.frontier_id == 'F1' else a
                      for a in context.evaluation.assessments)
        context = replace(context, evaluation=replace(context.evaluation, assessments=costs))
        policy = BaselinePolicy(PolicyName.NEAREST)
        policy.reset('test', 0, PolicyConfig())
        self.assertEqual(decide(policy, state, context).subgoal.frontier_id, 'F2')

    def test_fenced_json_and_reject_prose(self):
        self.assertEqual(parse_response('```json\n{"frontier_id":"F3","reason":"clues"}\n```')['frontier_id'], 'F3')
        ranked = parse_response('{"frontier_id":"F3","reason":"clues",'
                                '"ranked_frontier_ids":["F3","F2"],"confidence":0.8}')
        self.assertEqual(ranked['ranked_frontier_ids'], ['F3', 'F2'])
        for raw in ('F3', 'text {"frontier_id":"F3","reason":"clues"}', '{"frontier_id": 3}'):
            with self.assertRaises(ValueError):
                parse_response(raw)
        for raw in ('{"frontier_id":"F3","reason":"x","ranked_frontier_ids":["F2"]}',
                    '{"frontier_id":"F3","reason":"x","confidence":2}'):
            with self.assertRaises(ValueError):
                parse_response(raw)

    def test_provider_serialization_usage_and_no_key_logging(self):
        with patch.dict(os.environ, {'LLM_API_KEY': 'test-secret'}):
            client = GeminiCompatibleClient()
        response = {'raw_response': '{"frontier_id":"F3","reason":"clues"}',
                    'usage': {'total_tokens': 12}, 'response_model': 'test-model'}
        with patch('frontier_search.policies.provider.subprocess.run') as run:
            run.return_value = subprocess.CompletedProcess([], 0, json.dumps(response), '')
            summary, events = run_case('llm_guided', backend='live', client=client, model='test-model')
            self.assertEqual(summary['llm_calls'], 1)
            log = next(e['record'] for e in events if e['event'] == 'fixture_llm_call')
            self.assertEqual(log['usage']['total_tokens'], 12)
            self.assertNotIn('test-secret', dumps(events))
            self.assertNotIn('test-secret', str(run.call_args.args))

    def test_timeout_maps_to_nearest_and_expired_deadline_skips_worker(self):
        with patch.dict(os.environ, {'LLM_API_KEY': 'test-secret'}):
            client = GeminiCompatibleClient()
        with patch('frontier_search.policies.provider.subprocess.run') as run:
            with self.assertRaises(TimeoutError):
                client.select_id({}, model='test', reasoning=None, deadline_monotonic=monotonic()-1)
            run.assert_not_called()
            run.side_effect = subprocess.TimeoutExpired('worker', 0.01)
            state = initial_state('test')
            policy = LLMGuidedPolicy(client)
            policy.reset('test', 0, PolicyConfig(llm_model='test'))
            result = decide(policy, state, build_context(state, assess(state)))
            self.assertEqual(result.subgoal.frontier_id, 'F1')
            self.assertEqual(policy.call_records[-1]['fallback_reason'], 'timeout')

    def test_env_is_scoped_and_process_values_win(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / '.env'
            path.write_text('LLM_MODEL=file\nLLM_API_KEY=fixture\nUNRELATED=value\n', encoding='utf-8')
            with patch.dict(os.environ, {'LLM_MODEL': 'process'}, clear=True):
                load_local_env(path)
                self.assertEqual(os.environ['LLM_MODEL'], 'process')
                self.assertEqual(os.environ['LLM_API_KEY'], 'fixture')
                self.assertNotIn('UNRELATED', os.environ)


if __name__ == '__main__':
    unittest.main()
