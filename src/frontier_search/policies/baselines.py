"""Runnable deterministic baselines on authored or real assessed state."""


from random import Random
from typing import Optional, Tuple

from ..config import PolicyConfig
from ..contracts import PolicyName, SelectionContext, SelectionResult, Subgoal, WorldState
from ..validation import require, validate_context


def nearest_id(context: SelectionContext) -> str:
    costs = {a.frontier_id: a.path_cost_m for a in context.evaluation.assessments}
    return min(context.eligible_frontier_ids, key=lambda fid: (costs[fid], fid))


class BaselinePolicy:
    def __init__(self, name: PolicyName) -> None:
        require(name in (PolicyName.RANDOM, PolicyName.NEAREST, PolicyName.INFORMATION_GAIN),
                "use LLMGuidedPolicy for llm_guided")
        self.name = name
        self.episode_id: Optional[str] = None

    def reset(self, episode_id: str, seed: int, config: PolicyConfig) -> None:
        self.episode_id = episode_id
        self.config = config
        self.rng = Random(seed)  # Dedicated policy stream; never global RNG.

    def select(self, state: WorldState, context: SelectionContext) -> SelectionResult:
        require(self.episode_id == state.episode_id, "policy must be reset for this episode")
        validate_context(state, context)
        stop = ('target_claimed' if state.target_claim else 'no_frontiers' if not state.frontiers
                else 'no_eligible_frontiers' if not context.eligible_frontier_ids else None)
        if stop:
            return SelectionResult(state.episode_id, None, stop, None)
        fid, rationale = self._choose(state, context)
        frontier = next(f for f in state.frontiers if f.frontier_id == fid)
        return SelectionResult(state.episode_id,
                               Subgoal(fid, frontier.approach_pose, state.revision, self.name),
                               None, rationale)

    def _choose(self, state: WorldState, context: SelectionContext) -> Tuple[str, str]:
        eligible = sorted(context.eligible_frontier_ids)
        if self.name == PolicyName.RANDOM:
            return self.rng.choice(eligible), 'uniform eligible choice'
        if self.name == PolicyName.NEAREST:
            return nearest_id(context), 'minimum assessed path length'
        gains = {f.frontier_id: f.information_gain_m2 for f in state.frontiers}
        costs = {a.frontier_id: a.path_cost_m for a in context.evaluation.assessments}
        fid = min(eligible, key=lambda f: (-gains[f] / (costs[f] + self.config.epsilon_m), f))
        return fid, 'maximum information gain / (path cost + epsilon)'
