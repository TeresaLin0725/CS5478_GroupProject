"""Focused contract and policy checks without simulator or hosted-model calls."""


from dataclasses import replace
import json
from pathlib import Path
import tempfile
import unittest

from frontier_search.config import PolicyConfig
from frontier_search.contracts import (
    CandidateAssessment, FailureReason, PolicyName, SelectionResult, Subgoal,
)
from frontier_search.experiments.logging import JsonlLogger
from frontier_search.fixtures.loader import load_navigation_fixture, load_policy_fixture
from frontier_search.policies.baselines import BaselinePolicy
from frontier_search.policies.llm import LLMGuidedPolicy
from frontier_search.runner.episode import decide
from frontier_search.validation import ContractError, validate_context, validate_selection, validate_state
from typing import Union


class StubClient:
    def __init__(self, response: Union[str, Exception]) -> None:
        self.response = response
        self.payload = None

    def select_id(self, payload: dict, **kwargs: object) -> str:
        self.payload = payload
        if isinstance(self.response, Exception):
            raise self.response
        return self.response


class ScaffoldTests(unittest.TestCase):
    def setUp(self) -> None:
        self.fixture = load_policy_fixture()

    def select(self, name: PolicyName, fixture=None):
        fixture = fixture or self.fixture
        policy = BaselinePolicy(name)
        policy.reset(fixture.state.episode_id, 7, PolicyConfig())
        return decide(policy, fixture.state, fixture.context)

    def test_documented_policy_choices(self) -> None:
        self.assertEqual(self.select(PolicyName.NEAREST).subgoal.frontier_id, 'F1')
        self.assertEqual(self.select(PolicyName.INFORMATION_GAIN).subgoal.frontier_id, 'F2')

    def test_nearest_uses_path_cost_not_euclidean_distance(self) -> None:
        fixture = self.fixture
        changed = tuple(replace(a, path_cost_m=10.0) if a.frontier_id == 'F1' else a
                        for a in fixture.context.evaluation.assessments)
        context = replace(fixture.context, evaluation=replace(fixture.context.evaluation, assessments=changed))
        self.assertEqual(self.select(PolicyName.NEAREST, replace(fixture, context=context)).subgoal.frontier_id, 'F2')

    def test_random_is_seeded_and_only_eligible(self) -> None:
        policies = [BaselinePolicy(PolicyName.RANDOM) for _ in range(2)]
        for policy in policies:
            policy.reset(self.fixture.state.episode_id, 11, PolicyConfig())
        choices = [[policy.select(self.fixture.state, self.fixture.context).subgoal.frontier_id
                    for _ in range(30)] for policy in policies]
        self.assertEqual(choices[0], choices[1])
        self.assertEqual(set(choices[0]), {'F1', 'F2'})

    def test_ties_use_frontier_id_independent_of_input_order(self) -> None:
        fixture = self.fixture
        assessments = tuple(replace(a, path_cost_m=2.0) if a.reachable else a
                            for a in reversed(fixture.context.evaluation.assessments))
        context = replace(fixture.context, eligible_frontier_ids=('F2', 'F1'),
                          evaluation=replace(fixture.context.evaluation, assessments=assessments))
        self.assertEqual(self.select(PolicyName.NEAREST, replace(fixture, context=context)).subgoal.frontier_id, 'F1')

    def test_stop_requests_and_exclusions(self) -> None:
        for name, reason in (('no_frontiers', 'no_frontiers'), ('all_excluded', 'no_eligible_frontiers'),
                             ('target_claim', 'target_claimed')):
            with self.subTest(name=name):
                self.assertEqual(self.select(PolicyName.NEAREST, load_policy_fixture(name)).stop_reason, reason)
        for name in ('inconclusive', 'prior_failure'):
            self.assertEqual(self.select(PolicyName.NEAREST, load_policy_fixture(name)).subgoal.frontier_id, 'F2')

    def test_mutable_snapshot_and_nonfinite_numbers_rejected(self) -> None:
        with self.assertRaises(TypeError):
            replace(self.fixture.state, frontiers=list(self.fixture.state.frontiers))
        with self.assertRaises(ValueError):
            replace(self.fixture.state.frontiers[0], information_gain_m2=float('nan'))

    def test_hidden_standard_room_labels_rejected(self) -> None:
        state = self.fixture.state
        region = replace(state.regions[0], room_type='kitchen', room_type_source='simulator_annotation')
        state = replace(state, regions=(region,) + state.regions[1:])
        with self.assertRaises(ContractError):
            validate_state(state)
        validate_state(state, oracle_room_labels=True)

    def test_stale_or_ineligible_assessment_rejected(self) -> None:
        state, context = self.fixture.state, self.fixture.context
        for invalid in (
            replace(context, evaluation=replace(context.evaluation, state_revision=0)),
            replace(context, eligible_frontier_ids=('F1', 'F2', 'F3')),
            replace(context, evaluation=replace(context.evaluation, assessments=(
                CandidateAssessment('F1', None, None, FailureReason.NO_PATH),
            ) + context.evaluation.assessments[1:])),
        ):
            with self.assertRaises(ContractError):
                validate_context(state, invalid)

    def test_invented_pose_and_ignored_claim_rejected(self) -> None:
        state, context = self.fixture.state, self.fixture.context
        frontier = state.frontiers[0]
        result = SelectionResult(state.episode_id, Subgoal('F1', replace(frontier.approach_pose, yaw_rad=0.2),
                                                         state.revision, PolicyName.NEAREST), None, None)
        with self.assertRaises(ContractError):
            validate_selection(state, context, result)
        claimed = load_policy_fixture('target_claim')
        result = replace(result, subgoal=replace(result.subgoal, approach_pose=frontier.approach_pose))
        with self.assertRaises(ContractError):
            validate_selection(claimed.state, claimed.context, result)

    def test_llm_eligibility_timeout_and_call_cap(self) -> None:
        for response, selected, fallback in (
            ('F2', 'F2', None), ('F3', 'F1', 'invalid_or_ineligible_id'),
            ('{"frontier_id": "F2"}', 'F1', 'invalid_or_ineligible_id'),
            (TimeoutError(), 'F1', 'timeout'),
        ):
            with self.subTest(response=response):
                client = StubClient(response)
                policy = LLMGuidedPolicy(client)
                policy.reset(self.fixture.state.episode_id, 0, PolicyConfig(llm_model='stub-model', llm_call_cap=1))
                result = decide(policy, self.fixture.state, self.fixture.context)
                self.assertEqual(result.subgoal.frontier_id, selected)
                self.assertEqual(policy.call_records[0]['fallback_reason'], fallback)
                self.assertNotIn('occupancy_map', client.payload)
                self.assertEqual({c['frontier_id'] for c in client.payload['candidates']}, {'F1', 'F2'})
                decide(policy, self.fixture.state, self.fixture.context)
                self.assertEqual(policy.call_records[-1]['fallback_reason'], 'call_cap')

    def test_navigation_grids_preserve_unknown_and_newer_blockage(self) -> None:
        fixture = load_navigation_fixture('blocked_update')
        self.assertEqual(fixture.grid.data[0][0], -1)
        self.assertEqual(fixture.grid.data[20][20], 0)
        self.assertEqual(fixture.updated_grid.data[20][20], 100)
        self.assertGreater(fixture.updated_grid.revision, fixture.grid.revision)
        self.assertEqual(load_navigation_fixture('unknown_barrier').grid.data[20][20], -1)
        self.assertEqual(load_navigation_fixture('narrow_doorway').grid.data[19][20], 100)

    def test_jsonl_serializes_contracts_and_preserves_existing_logs(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'events.jsonl'
            with JsonlLogger(path) as logger:
                logger.write('selection', self.fixture.state.episode_id,
                             result=self.select(PolicyName.NEAREST))
            record = json.loads(path.read_text(encoding='utf-8'))
            self.assertEqual(record['result']['subgoal']['policy'], 'nearest')
            with self.assertRaises(FileExistsError):
                JsonlLogger(path)

    def test_epsilon_must_be_positive_and_finite(self) -> None:
        for epsilon in (0, -1, float('inf'), float('nan')):
            with self.assertRaises(ValueError):
                PolicyConfig(epsilon_m=epsilon)


if __name__ == '__main__':
    unittest.main()
