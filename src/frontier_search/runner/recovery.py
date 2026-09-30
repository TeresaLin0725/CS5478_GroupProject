"""Owner C: recovery interface. Persistent spatial/lineage tracking is pending."""
from typing import Protocol, Tuple


from ..contracts import CandidateEvaluation, FrontierExclusion, PlannerAttempt, WorldState


class RecoveryManager(Protocol):
    def reset(self, episode_id: str) -> None: ...
    def record_attempt(self, state_at_dispatch: WorldState, attempt: PlannerAttempt) -> None: ...
    def record_progress(self, before: WorldState, after: WorldState, frontier_id: str) -> None: ...
    def exclusions(self, state: WorldState, evaluation: CandidateEvaluation) -> Tuple[FrontierExclusion, ...]: ...


# Implementation must retain overlapping support and lineage through disappearance,
# transfer cumulative caps on split/merge, and distinguish local changed evidence
# from unrelated revision bumps. Do not implement a frontier-ID-only blacklist.
