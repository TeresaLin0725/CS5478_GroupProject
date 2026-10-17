"""Symbolic LLM adapter seam with auditable Nearest fallback.

No provider is bundled. A real client MUST enforce transport cancellation at the
absolute deadline. The full runner must independently supervise hung modules;
this synchronous scaffold cannot interrupt a client that ignores its contract.
"""


from dataclasses import asdict
from time import monotonic
from typing import Callable, List, Optional, Protocol, Tuple

from ..config import PolicyConfig
from ..contracts import PolicyName, SelectionContext, WorldState
from .baselines import BaselinePolicy, nearest_id, nearest_target_approach


class LLMClient(Protocol):
    def select_id(self, payload: dict, *, model: str, reasoning: Optional[str],
                  deadline_monotonic: float) -> str: ...


class LLMGuidedPolicy(BaselinePolicy):
    def __init__(self, client: Optional[LLMClient] = None,
                 log: Optional[Callable[[dict], None]] = None) -> None:
        super().__init__(PolicyName.NEAREST)
        self.name = PolicyName.LLM_GUIDED
        self.client = client
        self.log = log
        self.call_records: List[dict] = []

    def reset(self, episode_id: str, seed: int, config: PolicyConfig) -> None:
        super().reset(episode_id, seed, config)
        self.calls = 0
        self.call_records.clear()

    def _choose(self, state: WorldState, context: SelectionContext) -> Tuple[str, str]:
        target = nearest_target_approach(state, context)
        if target is not None:
            return target, 'approach observed target; visibility alone is not completion'
        costs = {a.frontier_id: a.path_cost_m for a in context.evaluation.assessments}
        payload = {
            'prompt': self.config.llm_prompt,
            'target_class': state.target_class,
            'observed_objects': [asdict(obj) for obj in state.semantic_objects],
            'regions': [{'region_id': r.region_id, 'room_type': r.room_type,
                         'explored_fraction': r.explored_fraction,
                         'observed_object_classes': r.observed_object_classes} for r in state.regions],
            'candidates': [{'frontier_id': f.frontier_id, 'region_id': f.region_id,
                            'information_gain_m2': f.information_gain_m2,
                            'path_cost_m': costs[f.frontier_id]} for f in sorted(
                                state.frontiers, key=lambda f: f.frontier_id)
                           if f.frontier_id in context.eligible_frontier_ids],
            'exclusions': [asdict(e) for e in context.exclusions],
            'previous_attempt': asdict(context.previous_attempt) if context.previous_attempt else None,
        }
        started = monotonic()
        response = None
        fallback = None
        provider_record = None
        if self.client is None or not self.config.llm_model:
            fallback = 'unconfigured_client'
        elif self.calls >= self.config.llm_call_cap:
            fallback = 'call_cap'
        else:
            self.calls += 1
            deadline = started + self.config.llm_deadline_s
            try:
                response = self.client.select_id(payload, model=self.config.llm_model,
                                                reasoning=self.config.llm_reasoning,
                                                deadline_monotonic=deadline)
                if monotonic() > deadline:
                    fallback = 'timeout'
                elif not isinstance(response, str) or response not in context.eligible_frontier_ids:
                    fallback = 'invalid_or_ineligible_id'
            except TimeoutError:
                fallback = 'timeout'
            except Exception as error:
                fallback = f'client_error:{type(error).__name__}'
            provider_record = getattr(self.client, 'last_record', None)
        record = {'episode_id': state.episode_id, 'state_revision': state.revision,
                  'model': self.config.llm_model, 'reasoning': self.config.llm_reasoning,
                  'input': payload, 'response': response, 'latency_s': monotonic() - started,
                  'fallback_reason': fallback, 'fallback_selected_action': fallback is not None,
                  'usage': provider_record.get('usage') if provider_record else None,
                  'cost': provider_record.get('cost') if provider_record else None,
                  'provider': provider_record}
        self.call_records.append(record)
        if self.log:
            self.log(record)
        rationale = provider_record.get('reason', 'LLM selection') if provider_record else 'LLM selection'
        return (nearest_id(context), f'nearest fallback: {fallback}') if fallback else (response, rationale)
