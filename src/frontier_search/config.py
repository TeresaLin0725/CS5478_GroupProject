"""Configuration records. Defaults are synthetic development values, not a freeze.

Before real integration, extend these records with the selected sensor, detector,
motion-model and algorithm settings listed in interface_freeze.md section 8.
"""


from dataclasses import dataclass
from math import isfinite
from typing import Optional, Tuple


@dataclass(frozen=True)
class MappingConfig:
    resolution_m: float = 0.1
    width_cells: int = 40
    height_cells: int = 40
    origin_x_m: float = 0.0
    origin_y_m: float = 0.0
    confirmation_frames: int = 2
    confirmation_window_s: float = 2.0
    confidence_threshold: float = 0.8

    def __post_init__(self) -> None:
        if not isfinite(self.resolution_m) or self.resolution_m <= 0:
            raise ValueError("resolution_m must be finite and positive")
        if min(self.width_cells, self.height_cells) <= 0:
            raise ValueError("grid dimensions must be positive")
        if self.confirmation_frames < 2:
            raise ValueError("target confirmation requires at least two distinct frames")
        if not 0 <= self.confidence_threshold <= 1:
            raise ValueError("confidence_threshold must lie in [0, 1]")
        if not isfinite(self.confirmation_window_s) or self.confirmation_window_s <= 0:
            raise ValueError("confirmation window must be finite and positive")


@dataclass(frozen=True)
class ObservationConfig:
    initial_headings_rad: Tuple[float, ...] = (0.0, 1.57, -1.57, 0.0)
    arrival_headings_rad: Tuple[float, ...] = (-0.5, 0.5, 0.0)
    recovery_headings_rad: Tuple[float, ...] = (-0.5, 0.5, 0.0)
    heading_tolerance_rad: float = 0.05
    dwell_s: float = 0.2
    processed_samples_per_heading: int = 1
    allow_full_rotation: bool = False
    operation_deadline_s: float = 20.0
    drain_deadline_s: float = 5.0


@dataclass(frozen=True)
class RecoveryConfig:
    cooldown_s: float = 5.0
    evaluation_retry_cap: int = 2
    navigation_retry_cap: int = 2
    opportunity_attempt_cap: int = 4
    no_progress_cap: int = 2
    progress_radius_m: float = 1.0
    minimum_progress_m2: float = 0.1
    max_cycles_without_navigation: int = 2


@dataclass(frozen=True)
class PolicyConfig:
    epsilon_m: float = 0.1
    llm_model: Optional[str] = None
    llm_reasoning: Optional[str] = None
    llm_deadline_s: float = 5.0
    llm_call_cap: int = 10
    llm_prompt: str = "Return exactly one eligible frontier ID."

    def __post_init__(self) -> None:
        if not isfinite(self.epsilon_m) or self.epsilon_m <= 0:
            raise ValueError("epsilon_m must be finite and positive")
        if not isfinite(self.llm_deadline_s) or self.llm_deadline_s <= 0:
            raise ValueError("LLM deadline must be finite and positive")
        if self.llm_call_cap < 0:
            raise ValueError("LLM call cap must be nonnegative")


@dataclass(frozen=True)
class NavigationConfig:
    batch_deadline_s: float = 2.0
    navigation_deadline_s: float = 30.0
    max_replans: int = 3
    pose_freshness_m: float = 0.05
    map_freshness_s: float = 0.2


@dataclass(frozen=True)
class EvaluationConfig:
    minimum_visibility: float = 0.1
    minimum_bbox_iou: float = 0.5
    localization_tolerance_m: float = 0.25
    required_evidence_frames: int = 2
    deadline_s: float = 5.0


@dataclass(frozen=True)
class EpisodeLimits:
    max_attempts: int = 20
    max_steps: int = 10000
    max_simulated_time_s: float = 300.0
    max_wall_time_s: float = 600.0
    max_decisions: int = 50
