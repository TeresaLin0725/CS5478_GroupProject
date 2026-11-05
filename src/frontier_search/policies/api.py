"""Owner C: policy selection plus bounded A/B execution feedback."""
from dataclasses import dataclass
from typing import Optional, Protocol


from ..config import PolicyConfig
from ..contracts import FailureReason, NavigationStatus, SelectionContext, SelectionResult, WorldState


@dataclass(frozen=True)
class PolicyFeedback:
    """Observed outcome delivered to C after dispatch; never simulator truth.

    B supplies navigation status/path cost, while A supplies post-arrival map
    progress. The policy cannot use an evaluator or hidden occupancy data.
    """
    episode_id: str
    decision_number: int
    frontier_id: str
    frontier_kind: str
    region_id: Optional[str]
    action_source: str
    selected_from_revision: int
    completed_revision: int
    timestamp_s: float
    navigation_status: NavigationStatus
    failure_reason: Optional[FailureReason]
    path_cost_m: Optional[float]
    new_known_area_m2: Optional[float]
    observation_status: Optional[NavigationStatus]


class SearchPolicy(Protocol):
    def reset(self, episode_id: str, seed: int, config: PolicyConfig) -> None: ...
    def select(self, state: WorldState, context: SelectionContext) -> SelectionResult: ...
    def observe_feedback(self, feedback: PolicyFeedback) -> None: ...
