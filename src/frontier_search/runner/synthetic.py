"""C integration case using contract 1.0. Not the production EpisodeRunner."""
import argparse
from pathlib import Path
import os
from math import hypot
from time import monotonic

from ..config import PolicyConfig
from ..contracts import PlannerAttempt, PolicyName
from ..experiments.logging import JsonlLogger, dumps
from ..fixtures.three_rooms import initial_state, assess, plan_grid_path, arrival_scan, FixtureEvaluator
from ..policies.baselines import BaselinePolicy
from ..policies.llm import LLMGuidedPolicy
from ..policies.provider import GeminiCompatibleClient, ReplayClient, load_local_env
from .episode import build_context, decide


def run_case(name, backend='replay', client=None, model=None, logger=None,
             evaluator=None, max_decisions=20, deadline_s=10.0):
    state = initial_state('three-rooms-' + name)
    policy_name = PolicyName(name)
    llm = policy_name == PolicyName.LLM_GUIDED
    if llm:
        client = client or (ReplayClient() if backend == 'replay' else GeminiCompatibleClient())
        policy = LLMGuidedPolicy(client)
    else:
        policy = BaselinePolicy(policy_name)
    config = PolicyConfig(llm_model=model or ('authored-replay' if backend == 'replay' else None),
                          llm_deadline_s=deadline_s)
    if llm and backend == 'live' and not model:
        raise ValueError('Live case requires LLM_MODEL')
    policy.reset(state.episode_id, 0, config)
    evaluator = evaluator or FixtureEvaluator()
    previous, evidence = None, None
    events = []
    def emit(kind, **payload):
        event = {'event': kind, 'episode_id': state.episode_id, **payload}
        events.append(event)
        if logger:
            logger.write(kind, state.episode_id, **payload)
    status, actual_distance, decisions = 'search_limit', 0.0, 0
    started = monotonic()
    emit('fixture_start', source='synthetic', backend=backend if llm else 'none', config=config)
    try:
        for _ in range(max_decisions):
            evaluation = assess(state)
            context = build_context(state, evaluation, previous_attempt=previous)
            selection = decide(policy, state, context)
            emit('fixture_decision', state=state, context=context, selection=selection)
            if selection.subgoal is None:
                status = selection.stop_reason
                break
            decisions += 1
            if llm:
                emit('fixture_llm_call', record=policy.call_records[-1])
            plan = plan_grid_path(state, selection.subgoal.approach_pose)
            if not plan.success:
                # This static case cannot change between assessment and dispatch.
                raise RuntimeError('Mock B disagreed with its candidate assessment')
            previous = PlannerAttempt(selection.subgoal, plan, 'reached', None)
            # Mock B moves along each path segment; track executed fixture positions.
            positions = plan.path
            actual_distance += sum(hypot(b.x_m-a.x_m, b.y_m-a.y_m)
                                   for a, b in zip(positions, positions[1:]))
            emit('fixture_navigation', attempt=previous, executed_poses=positions)
            state, evidence = arrival_scan(state, selection.subgoal.frontier_id)
            emit('fixture_arrival_scan', state=state)
            if state.target_claim is not None:
                # Terminal decision prevents further model calls. Evaluator output
                # is only logged and never passed to C's policy/context.
                stop = decide(policy, state, build_context(state, assess(state), previous_attempt=previous))
                emit('fixture_stop', selection=stop, evidence=evidence)
                try:
                    result = evaluator.evaluate(state, evidence)
                except Exception as exc:
                    emit('fixture_error', error_type=type(exc).__name__)
                    status = 'evaluation_error'
                    break
                emit('fixture_evaluation', result=result)
                status = 'success' if result.valid_target else 'false_target_claim'
                break
    except Exception as exc:
        status = 'pipeline_error'
        emit('fixture_error', error_type=type(exc).__name__)
    records = policy.call_records if llm else []
    summary = {'source': 'synthetic', 'policy': name, 'backend': backend if llm else 'none',
        'termination': status, 'decisions': decisions, 'executed_fixture_distance_m': actual_distance,
        'llm_calls': policy.calls if llm and backend == 'live' else 0,
        'replay_calls': policy.calls if llm and backend == 'replay' else 0,
        'fallbacks': sum(r['fallback_selected_action'] for r in records),
        'wall_time_s': monotonic() - started}
    emit('fixture_terminal', **summary)
    return summary, events


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--policy', choices=['nearest', 'llm_guided', 'both'], default='both')
    parser.add_argument('--backend', choices=['replay', 'live'], default='replay')
    parser.add_argument('--env-file', type=Path, default=Path('.env'))
    parser.add_argument('--log', type=Path, required=True, help='New JSONL path; never overwritten')
    args = parser.parse_args()
    load_local_env(args.env_file)
    names = ['nearest', 'llm_guided'] if args.policy == 'both' else [args.policy]
    failed = False
    with JsonlLogger(args.log) as logger:
        for name in names:
            summary, _ = run_case(name, args.backend, model=os.environ.get('LLM_MODEL')
                                  if args.backend == 'live' else None, logger=logger)
            print(dumps(summary))
            failed |= summary['termination'] != 'success'
            # A fallback run is valid, but cannot pass the live-provider smoke gate.
            failed |= args.backend == 'live' and summary['fallbacks'] > 0
    return int(failed)


if __name__ == '__main__':
    raise SystemExit(main())
