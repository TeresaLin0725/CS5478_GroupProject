"""Owner A: isolated terminal scoring. Do not inject this into policy modules."""
from typing import Protocol


from ..config import EvaluationConfig
from ..contracts import EvaluationRequest, EvaluationResult


class TargetEvaluator(Protocol):
    def reset(self, episode_id: str, config: EvaluationConfig) -> None: ...
    def evaluate(self, request: EvaluationRequest) -> EvaluationResult: ...
