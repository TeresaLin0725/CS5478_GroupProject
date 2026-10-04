"""Owner A: episode initialization for the RGB-D mapper.

Private mutable buffers hold working data; published WorldState snapshots must
be independently owned and immutable.
"""

from dataclasses import dataclass, field
from random import Random
from typing import Dict, List, Optional, Tuple

from ..config import MappingConfig, ObservationConfig
from ..contracts import (
    DetectionEvidence, EvidenceBundle, Frontier, GridCell, MapperInit, ObjectTrack,
    RGBDObservation, RobotConfig, TargetClaim, WorldState,
)
from ..immutable import require_immutable
from ..validation import ContractError, is_finite_number, require
from .priors import validate_region_priors


@dataclass
class _EpisodeState:
    """Hold one episode's private maps, tracks, evidence and counters.

    RGBDMapper replaces this entire container on each successful reset.
    """

    request: MapperInit
    occupancy: List[List[int]]
    coverage: List[List[bool]]
    rng: Random
    tracks: Dict[str, ObjectTrack] = field(default_factory=dict)
    frontiers: Dict[str, Frontier] = field(default_factory=dict)
    frontier_history: Dict[str, Tuple[str, ...]] = field(default_factory=dict)
    evidence: Dict[str, DetectionEvidence] = field(default_factory=dict)
    target_claim: Optional[TargetClaim] = None
    entity_counters: Dict[str, int] = field(default_factory=lambda: {
        "track": 0, "frontier": 0, "observation": 0, "claim": 0,
    })
    revision: int = 0
    last_processed_sequence: Optional[int] = None
    last_timestamp_s: Optional[float] = None
    latest_state: Optional[WorldState] = None



def _nonempty_text(value: object) -> bool:
    return isinstance(value, str) and bool(value.strip())


# Check that configuration and priors are valid for a new mapping episode.
def _validate_init(request: MapperInit) -> None:
    require(isinstance(request, MapperInit), "reset requires MapperInit")
    try:
        require_immutable(request)
    except (TypeError, ValueError) as exc:
        raise ContractError("invalid mapper initialization: %s" % exc) from exc

    require(_nonempty_text(request.episode_id), "missing episode ID")
    require(_nonempty_text(request.target_class), "missing target class")
    require(type(request.seed) is int, "seed must be an integer")
    require(type(request.oracle_room_labels) is bool, "oracle_room_labels must be boolean")
    require(isinstance(request.mapping_config, MappingConfig), "missing mapping configuration")
    require(isinstance(request.observation_config, ObservationConfig), "missing observation configuration")
    require(isinstance(request.robot_config, RobotConfig), "missing robot configuration")
    cfg = request.mapping_config
    require(type(cfg.width_cells) is int and cfg.width_cells > 0,
            "grid width must be a positive integer")
    require(type(cfg.height_cells) is int and cfg.height_cells > 0,
            "grid height must be a positive integer")
    require(is_finite_number(cfg.resolution_m) and cfg.resolution_m > 0,
            "grid resolution must be finite and positive")
    require(is_finite_number(cfg.origin_x_m) and is_finite_number(cfg.origin_y_m),
            "grid origin must be finite")

    require(isinstance(request.initial_clearance_cells, tuple), "clearance cells must be a tuple")
    for cell in request.initial_clearance_cells:
        require(isinstance(cell, GridCell), "clearance requires GridCell records")
        require(type(cell.row) is int and type(cell.column) is int,
                "clearance indices must be integers")
        require(0 <= cell.row < cfg.height_cells and 0 <= cell.column < cfg.width_cells,
                "clearance cell outside grid")

    validate_region_priors(request.region_priors, request.oracle_room_labels)


class RGBDMapper:
    """Manage perception lifecycle through the PerceptionMapper API."""

    def __init__(self) -> None:
        self._state = None  # type: Optional[_EpisodeState]

    # Prepare the mapper for a new episode and clear previous episode data.
    def reset(self, request: MapperInit) -> None:
        """Validate and replace the episode, without publishing an observation.

        Room geometry never seeds free space. Only the supplied verified spawn
        cells are free, and they do not count as geometric observation coverage.
        Failed validation or allocation leaves the previous episode untouched.
        """
        _validate_init(request)
        cfg = request.mapping_config
        occupancy = [[-1 for _ in range(cfg.width_cells)] for _ in range(cfg.height_cells)]
        coverage = [[False for _ in range(cfg.width_cells)] for _ in range(cfg.height_cells)]
        for cell in request.initial_clearance_cells:
            occupancy[cell.row][cell.column] = 0
        new_state = _EpisodeState(request, occupancy, coverage, Random(request.seed))
        self._state = new_state

    def update(self, observation: RGBDObservation) -> WorldState:
        # TODO: Process RGB-D into maps, tracks and frontiers; publish an immutable WorldState.
        raise NotImplementedError("RGB-D processing is not implemented yet")

    def get_claim_evidence(self, episode_id: str, claim_id: str) -> EvidenceBundle:
        # TODO: Validate episode/claim IDs and return retained evidence for scoring.
        raise NotImplementedError("target claim evidence retrieval is not implemented yet")
