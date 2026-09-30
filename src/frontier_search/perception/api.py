"""Owner A: RGB-D mapping, tracking, frontier identity and claim evidence."""
from typing import Protocol


from ..contracts import EvidenceBundle, MapperInit, RGBDObservation, WorldState


class PerceptionMapper(Protocol):
    def reset(self, request: MapperInit) -> None: ...
    def update(self, observation: RGBDObservation) -> WorldState: ...
    def get_claim_evidence(self, episode_id: str, claim_id: str) -> EvidenceBundle: ...
