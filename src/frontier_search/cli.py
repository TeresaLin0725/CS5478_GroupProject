"""Synthetic decision demo; requires neither iGibson nor a hosted LLM."""

import argparse
from pathlib import Path
from typing import List

from .config import PolicyConfig
from .contracts import PolicyName
from .experiments.logging import JsonlLogger, dumps
from .fixtures.loader import load_policy_fixture
from .policies.baselines import BaselinePolicy
from .policies.llm import LLMGuidedPolicy
from .runner.episode import build_context, decide


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--policy', choices=[p.value for p in PolicyName], default='nearest')
    # Avoid contextual tuple inference against argparse's Iterable protocol in Pylance.
    fixture_choices: List[str] = [
        'selection', 'no_frontiers', 'all_excluded', 'target_claim', 'inconclusive', 'prior_failure',
    ]
    parser.add_argument('--fixture', default='selection', choices=fixture_choices)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--log', type=Path, help='New JSONL output path (existing files are preserved)')
    args = parser.parse_args()
    fixture = load_policy_fixture(args.fixture)
    name = PolicyName(args.policy)
    policy = LLMGuidedPolicy() if name == PolicyName.LLM_GUIDED else BaselinePolicy(name)
    policy.reset(fixture.state.episode_id, args.seed, PolicyConfig())
    context = build_context(fixture.state, fixture.context.evaluation,
                            fixture.context.exclusions, fixture.context.previous_attempt)
    result = decide(policy, fixture.state, context)
    output = {'mode': 'synthetic_decision_demo', 'fixture': fixture.metadata,
              'seed': args.seed, 'eligible_frontier_ids': context.eligible_frontier_ids,
              'selection': result}
    if args.log:
        with JsonlLogger(args.log) as logger:
            logger.write('fixture_decision', fixture.state.episode_id, **output)
            if isinstance(policy, LLMGuidedPolicy):
                for record in policy.call_records:
                    logger.write('fixture_llm_call', fixture.state.episode_id, record=record)
    print(dumps(output))
