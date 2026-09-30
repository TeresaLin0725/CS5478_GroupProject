"""Owner C: dependency wiring and the shared, runnable decision boundary.

The simulator episode loop deliberately fails until lifecycle safeguards exist.
The fixture CLI exercises decide() only; it does not report experiment success.
"""


from enum import Enum
from typing import Optional, Tuple

from ..contracts import (
    CandidateEvaluation, FrontierExclusion, PlannerAttempt, SelectionContext,
    SelectionResult, WorldState,
)
from ..evaluation.api import TargetEvaluator
from ..navigation.api import Navigator
from ..perception.api import PerceptionMapper
from ..policies.api import SearchPolicy
from ..simulation.api import SimulatorAdapter
from ..validation import validate_context, validate_selection, validate_state
from .recovery import RecoveryManager


class TerminationReason(str, Enum):
    SUCCESS = 'success'
    FALSE_TARGET_CLAIM = 'false_target_claim'
    NO_FRONTIERS = 'no_frontiers'
    NO_FEASIBLE_FRONTIERS = 'no_feasible_frontiers'
    SEARCH_LIMIT = 'search_limit'
    INITIALIZATION_FAILED = 'initialization_failed'
    PIPELINE_ERROR = 'pipeline_error'
    EVALUATION_ERROR = 'evaluation_error'
    EXTERNALLY_CANCELLED = 'externally_cancelled'


def build_context(state: WorldState, evaluation: CandidateEvaluation,
                  exclusions: Tuple[FrontierExclusion, ...] = (),
                  previous_attempt: Optional[PlannerAttempt] = None) -> SelectionContext:
    excluded = {e.frontier_id for e in exclusions}
    eligible = tuple(sorted(a.frontier_id for a in evaluation.assessments
                            if a.reachable is True and a.frontier_id not in excluded))
    context = SelectionContext(state.episode_id, evaluation, eligible, exclusions, previous_attempt)
    validate_context(state, context)
    return context


def decide(policy: SearchPolicy, state: WorldState, context: SelectionContext,
           *, oracle_room_labels: bool = False) -> SelectionResult:
    """Call only while stopped/paused with a fresh state and matching assessment."""
    validate_state(state, oracle_room_labels=oracle_room_labels)
    validate_context(state, context)
    result = policy.select(state, context)
    validate_selection(state, context, result)
    return result


class EpisodeRunner:
    def __init__(self, simulator: SimulatorAdapter, mapper: PerceptionMapper,
                 navigator: Navigator, policy: SearchPolicy, evaluator: TargetEvaluator,
                 recovery: RecoveryManager) -> None:
        self.simulator = simulator
        self.mapper = mapper
        self.navigator = navigator
        self.policy = policy
        self._evaluator = evaluator  # Never forwarded to the policy.
        self.recovery = recovery

    def run(self, manifest: object) -> TerminationReason:
        raise NotImplementedError(
            'Implement Interface Freeze section 5: verified spawn/reset, initial capture/scan, '
            'processed-frame barriers, paused decisions, stale dispatch revalidation, '
            'continuous perception and claim preemption, arrival scans, bounded spatial '
            'recovery, independent deadlines/budgets, isolated scoring and terminal logging.'
        )
