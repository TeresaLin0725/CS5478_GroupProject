"""Symbolic LLM adapter seam with auditable Nearest fallback.

No provider is bundled. A real client MUST enforce transport cancellation at the
absolute deadline. The full runner must independently supervise hung modules;
this synchronous scaffold cannot interrupt a client that ignores its contract.
"""


import json
from time import monotonic
from typing import Callable, Dict, List, Optional, Protocol, Tuple

from ..config import PolicyConfig
from ..contracts import PolicyName, SelectionContext, WorldState
from ..validation import require
from .api import PolicyFeedback
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
        self.feedback_records: List[PolicyFeedback] = []

    def reset(self, episode_id: str, seed: int, config: PolicyConfig) -> None:
        super().reset(episode_id, seed, config)
        self.calls = 0
        self.call_records.clear()
        self.feedback_records.clear()
        self.last_candidate_ranking = ()  # type: Tuple[str, ...]
        self._cache_key = None
        self._cache_choice = None
        self._cache_rationale = None
        self._cache_ranking = ()  # type: Tuple[str, ...]

    def observe_feedback(self, feedback: PolicyFeedback) -> None:
        require(feedback.episode_id == self.episode_id, "policy feedback episode mismatch")
        if self.config.llm_feedback_window == 0:
            self.feedback_records.clear()
            return
        self.feedback_records.append(feedback)
        del self.feedback_records[:-self.config.llm_feedback_window]

    @staticmethod
    def _pose(pose) -> dict:
        return {'x_m': pose.x_m, 'y_m': pose.y_m, 'z_m': pose.z_m,
                'yaw_rad': pose.yaw_rad}

    @staticmethod
    def _point(point) -> dict:
        return {'x_m': point.x_m, 'y_m': point.y_m, 'z_m': point.z_m}

    @staticmethod
    def _previous_attempt(context: SelectionContext) -> Optional[dict]:
        attempt = context.previous_attempt
        if attempt is None:
            return None
        result = attempt.result
        return {
            'frontier_id': attempt.subgoal.frontier_id,
            'frontier_kind': attempt.subgoal.kind,
            'selected_from_revision': attempt.subgoal.selected_from_revision,
            'execution_status': attempt.execution_status,
            'failure_reason': attempt.failure_reason.value if attempt.failure_reason else None,
            'planner_success': result.success if result else None,
            'path_cost_m': result.path_cost_m if result else None,
            # A full waypoint list is large and has no high-level semantic value.
            'path_waypoint_count': len(result.path) if result and result.path else 0,
        }

    @staticmethod
    def _feedback(feedback: PolicyFeedback) -> dict:
        return {
            'decision_number': feedback.decision_number,
            'frontier_id': feedback.frontier_id,
            'frontier_kind': feedback.frontier_kind,
            'region_id': feedback.region_id,
            'action_source': feedback.action_source,
            'selected_from_revision': feedback.selected_from_revision,
            'completed_revision': feedback.completed_revision,
            'timestamp_s': feedback.timestamp_s,
            'navigation_status': feedback.navigation_status.value,
            'failure_reason': feedback.failure_reason.value if feedback.failure_reason else None,
            'path_cost_m': feedback.path_cost_m,
            'new_known_area_m2': feedback.new_known_area_m2,
            'observation_status': (feedback.observation_status.value
                                   if feedback.observation_status else None),
        }

    def _payload(self, state: WorldState, context: SelectionContext) -> dict:
        assessments = {a.frontier_id: a for a in context.evaluation.assessments}
        exclusions = {e.frontier_id: e for e in context.exclusions}
        regions = {region.region_id: region for region in state.regions}
        eligible = set(context.eligible_frontier_ids)
        candidates = []
        for frontier in sorted(state.frontiers, key=lambda item: item.frontier_id):
            if frontier.frontier_id not in eligible:
                continue
            assessment = assessments[frontier.frontier_id]
            region = regions.get(frontier.region_id)
            candidates.append({
                'frontier_id': frontier.frontier_id,
                'kind': frontier.kind,
                'region_id': frontier.region_id,
                'information_gain_m2': frontier.information_gain_m2,
                'path_cost_m': assessment.path_cost_m,
                'gain_per_path_m': (frontier.information_gain_m2 /
                                    (assessment.path_cost_m + self.config.epsilon_m)),
                'euclidean_distance_m': frontier.distance_from_robot_m,
                'approach_pose': self._pose(frontier.approach_pose),
                'alternative_approach_count': len(frontier.alternative_approach_poses),
                'parent_frontier_ids': frontier.parent_frontier_ids,
                'region_explored_fraction': region.explored_fraction if region else None,
                'region_observed_object_classes': (region.observed_object_classes if region else ()),
            })
        tracks = sorted(state.semantic_objects,
                        key=lambda item: (-item.confidence, -item.last_seen_s,
                                          item.class_name, item.track_id))
        observed = [{
            'track_id': track.track_id,
            'class_name': track.class_name,
            'confidence': track.confidence,
            'position_world': (self._point(track.position_world)
                               if track.position_world is not None else None),
            'last_seen_s': track.last_seen_s,
            'confirmation_count': track.confirmation_count,
        } for track in tracks[:self.config.llm_max_observed_objects]]
        grid = state.occupancy_map
        total_cells = grid.width_cells * grid.height_cells
        known_cells = sum(value != -1 for row in grid.data for value in row)
        explored_cells = sum(sum(row) for row in state.explored_map.data)
        low_level = []
        for assessment in sorted(context.evaluation.assessments,
                                 key=lambda item: item.frontier_id):
            exclusion = exclusions.get(assessment.frontier_id)
            low_level.append({
                'frontier_id': assessment.frontier_id,
                'eligible': assessment.frontier_id in eligible,
                'reachable': assessment.reachable,
                'path_cost_m': assessment.path_cost_m,
                'failure_reason': (assessment.failure_reason.value
                                   if assessment.failure_reason else None),
                'exclusion_reason': exclusion.reason if exclusion else None,
                'attempt_count': exclusion.attempt_count if exclusion else 0,
                'retry_condition': exclusion.retry_condition if exclusion else None,
                'retry_after_s': exclusion.retry_after_s if exclusion else None,
            })
        return {
            'prompt': self.config.llm_prompt,
            'target_class': state.target_class,
            'decision': {
                'state_revision': state.revision,
                'timestamp_s': state.timestamp_s,
                'robot_pose': self._pose(state.robot_pose),
                'known_fraction': known_cells / total_cells if total_cells else 0.0,
                'explored_fraction': explored_cells / total_cells if total_cells else 0.0,
                'frontier_count': len(state.frontiers),
                'eligible_frontier_count': len(context.eligible_frontier_ids),
            },
            'observed_objects': observed,
            'observed_object_count': len(state.semantic_objects),
            'regions': [{
                'region_id': region.region_id,
                'room_type': region.room_type,
                'explored_fraction': region.explored_fraction,
                'observed_object_classes': region.observed_object_classes,
            } for region in sorted(state.regions, key=lambda item: item.region_id)],
            'candidates': candidates,
            'eligible_frontier_ids': tuple(sorted(context.eligible_frontier_ids)),
            'low_level_feedback': low_level,
            'exclusions': [{
                'frontier_id': item.frontier_id,
                'reason': item.reason,
                'attempt_count': item.attempt_count,
                'retry_condition': item.retry_condition,
                'retry_after_s': item.retry_after_s,
            } for item in sorted(context.exclusions, key=lambda entry: entry.frontier_id)],
            'recent_execution_feedback': [self._feedback(item) for item in self.feedback_records],
            'previous_attempt': self._previous_attempt(context),
            'constraints': {
                'select_only_eligible': True,
                'target_claim_is_terminal': True,
                'hidden_map_and_evaluator_unavailable': True,
            },
        }

    @staticmethod
    def _material_key(payload: dict) -> str:
        # Revisions and timestamps advance on every capture. Cache only when all
        # material observations, candidates, B assessments and feedback match.
        decision = {key: value for key, value in payload['decision'].items()
                    if key not in ('state_revision', 'timestamp_s')}
        material = dict(payload)
        material['decision'] = decision
        return json.dumps(material, sort_keys=True, separators=(',', ':'), ensure_ascii=True)

    def _choose(self, state: WorldState, context: SelectionContext) -> Tuple[str, str]:
        target = nearest_target_approach(state, context)
        if target is not None:
            self.last_candidate_ranking = (target,)
            return target, 'approach observed target; visibility alone is not completion'
        if self.config.llm_bypass_single_candidate and len(context.eligible_frontier_ids) == 1:
            choice = context.eligible_frontier_ids[0]
            self.last_candidate_ranking = (choice,)
            return choice, 'single eligible frontier; hosted model call bypassed'
        payload = self._payload(state, context)
        cache_key = self._material_key(payload)
        if (self.config.llm_cache_identical_context and cache_key == self._cache_key
                and self._cache_choice in context.eligible_frontier_ids):
            self.last_candidate_ranking = tuple(
                item for item in self._cache_ranking if item in context.eligible_frontier_ids)
            return self._cache_choice, 'cached identical-context LLM selection: ' + self._cache_rationale
        started = monotonic()
        response = None
        fallback = None
        provider_record = None
        model_called = False
        if self.client is None or not self.config.llm_model:
            fallback = 'unconfigured_client'
        elif self.calls >= self.config.llm_call_cap:
            fallback = 'call_cap'
        else:
            self.calls += 1
            model_called = True
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
            if not isinstance(provider_record, dict):
                provider_record = None
        ranking = []  # type: List[str]
        if provider_record and isinstance(provider_record.get('answer'), dict):
            supplied = provider_record['answer'].get('ranked_frontier_ids', ())
            if isinstance(supplied, list):
                ranking = [item for item in supplied if item in context.eligible_frontier_ids]
        if response in context.eligible_frontier_ids:
            ranking = [response] + [item for item in ranking if item != response]
        if fallback:
            selected = nearest_id(context)
            self.last_candidate_ranking = (selected,)
        else:
            selected = response
            self.last_candidate_ranking = tuple(ranking or (response,))
        record = {'episode_id': state.episode_id, 'state_revision': state.revision,
                  'model': self.config.llm_model, 'reasoning': self.config.llm_reasoning,
                  'input': payload, 'response': response, 'latency_s': monotonic() - started,
                  'model_called': model_called,
                  'validated_ranking': self.last_candidate_ranking,
                  'fallback_reason': fallback, 'fallback_selected_action': fallback is not None,
                  'usage': provider_record.get('usage') if provider_record else None,
                  'cost': provider_record.get('cost') if provider_record else None,
                  'provider': provider_record}
        self.call_records.append(record)
        if self.log:
            self.log(record)
        rationale = provider_record.get('reason', 'LLM selection') if provider_record else 'LLM selection'
        if not fallback and self.config.llm_cache_identical_context:
            self._cache_key = cache_key
            self._cache_choice = selected
            self._cache_rationale = rationale
            self._cache_ranking = self.last_candidate_ranking
        return (selected, f'nearest fallback: {fallback}') if fallback else (selected, rationale)
