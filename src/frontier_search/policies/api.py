"""Owner C: select only from the runner's shared eligible candidates."""
from typing import Protocol


from ..config import PolicyConfig
from ..contracts import SelectionContext, SelectionResult, WorldState


class SearchPolicy(Protocol):
    def reset(self, episode_id: str, seed: int, config: PolicyConfig) -> None: ...
    def select(self, state: WorldState, context: SelectionContext) -> SelectionResult: ...
