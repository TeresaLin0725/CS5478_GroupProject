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
    floor_height_m: float = 0.0       # world z; explicit flat-floor assumption
    floor_tolerance_m: float = 0.03
    depth_min_m: float = 0.1         # axial camera depth
    depth_max_m: float = 5.0
    clearance_bin_height_m: float = 0.05  # vertical free-space evidence resolution
    depth_surface_margin_m: float = 0.02  # stop carving before measured surfaces
    base_elevation_margin_m: float = 0.0  # expand fixed world collision envelope for bounded base drift
    frontier_min_cells: int = 2
    frontier_connectivity: int = 4
    frontier_approach_search_m: float = 1.0
    frontier_match_tolerance_cells: int = 1
    frontier_match_fraction: float = 0.25
    frontier_history_s: float = 30.0
    gain_range_m: float = 3.0
    gain_ray_step_rad: float = 0.05
    object_match_distance_m: float = 0.35
    depth_pixel_stride: int = 1
    frontier_cluster_span_m: float = 0.0
    dynamic_clear_frames: int = 0
    inspection_distance_m: Optional[float] = None
    inspection_linear_speed_mps: float = 0.03
    inspection_angular_speed_radps: float = 0.08
    target_max_extent_m: Optional[float] = None
    # Keep a hypothesis across the roughly 13-second arrival scan, with slack.
    # This does not extend the independent terminal confirmation window.
    tentative_target_memory_s: float = 30.0
    # Detection hypotheses may be weak; terminal evidence must be stronger.
    claim_confidence_threshold: float = 0.65

    def __post_init__(self) -> None:
        if (type(self.claim_confidence_threshold) not in (int, float)
                or not isfinite(self.claim_confidence_threshold)
                or not 0 <= self.claim_confidence_threshold <= 1):
            raise ValueError('claim_confidence_threshold must lie in [0, 1]')
        if type(self.depth_pixel_stride) is not int or self.depth_pixel_stride < 1:
            raise ValueError('depth stride must be a positive integer')
        if not isfinite(self.tentative_target_memory_s) or self.tentative_target_memory_s <= 0:
            raise ValueError('tentative target memory must be positive')
        if type(self.dynamic_clear_frames) is not int or self.dynamic_clear_frames < 0:
            raise ValueError('dynamic clear frames must be a nonnegative integer')
        if not isfinite(self.frontier_cluster_span_m) or self.frontier_cluster_span_m < 0:
            raise ValueError('frontier span must be nonnegative')
        for value in (self.inspection_linear_speed_mps, self.inspection_angular_speed_radps):
            if not isfinite(value) or value <= 0:
                raise ValueError('inspection speeds must be positive')
        if self.inspection_distance_m is not None and (
                not isfinite(self.inspection_distance_m) or self.inspection_distance_m <= 0):
            raise ValueError('inspection distance must be positive')
        if self.target_max_extent_m is not None and (
                not isfinite(self.target_max_extent_m) or self.target_max_extent_m <= 0):
            raise ValueError('target extent must be positive')
        if (type(self.object_match_distance_m) not in (int, float)
                or not isfinite(self.object_match_distance_m) or self.object_match_distance_m <= 0):
            raise ValueError("object match distance must be finite and positive")
        if not isfinite(self.resolution_m) or self.resolution_m <= 0:
            raise ValueError("resolution_m must be finite and positive")
        if min(self.width_cells, self.height_cells) <= 0:
            raise ValueError("grid dimensions must be positive")
        if type(self.confirmation_frames) is not int or self.confirmation_frames < 2:
            raise ValueError("target confirmation requires at least two distinct frames")
        if (type(self.confidence_threshold) not in (int, float)
                or not isfinite(self.confidence_threshold) or not 0 <= self.confidence_threshold <= 1):
            raise ValueError("confidence_threshold must lie in [0, 1]")
        if (type(self.confirmation_window_s) not in (int, float)
                or not isfinite(self.confirmation_window_s) or self.confirmation_window_s <= 0):
            raise ValueError("confirmation window must be finite and positive")
        if not isfinite(self.floor_height_m):
            raise ValueError("floor height must be finite")
        if not isfinite(self.floor_tolerance_m) or self.floor_tolerance_m < 0:
            raise ValueError("floor tolerance must be finite and nonnegative")
        if not (isfinite(self.depth_min_m) and isfinite(self.depth_max_m)
                and 0 < self.depth_min_m < self.depth_max_m):
            raise ValueError("depth limits must be finite and satisfy 0 < min < max")
        if (type(self.clearance_bin_height_m) not in (int, float)
                or not isfinite(self.clearance_bin_height_m) or self.clearance_bin_height_m <= 0):
            raise ValueError("clearance bin height must be finite and positive")
        if (type(self.depth_surface_margin_m) not in (int, float)
                or not isfinite(self.depth_surface_margin_m)
                or not 0 <= self.depth_surface_margin_m < self.depth_min_m):
            raise ValueError("surface margin must be nonnegative and smaller than minimum depth")
        if (type(self.base_elevation_margin_m) not in (int, float)
                or not isfinite(self.base_elevation_margin_m) or self.base_elevation_margin_m < 0):
            raise ValueError("base elevation margin must be finite and nonnegative")
        if type(self.frontier_min_cells) is not int or self.frontier_min_cells < 1:
            raise ValueError("frontier minimum must be a positive integer")
        if type(self.frontier_connectivity) is not int or self.frontier_connectivity not in (4, 8):
            raise ValueError("frontier connectivity must be 4 or 8")
        if (type(self.frontier_match_tolerance_cells) is not int
                or not 0 <= self.frontier_match_tolerance_cells <= 8):
            raise ValueError("frontier matching tolerance must be an integer from 0 to 8 cells")
        for value in (self.frontier_approach_search_m, self.gain_range_m):
            if type(value) not in (int, float) or not isfinite(value) or value <= 0:
                raise ValueError("frontier search and gain ranges must be finite and positive")
        if (type(self.frontier_match_fraction) not in (int, float)
                or not isfinite(self.frontier_match_fraction) or not 0 < self.frontier_match_fraction <= 1):
            raise ValueError("frontier matching fraction must be in (0, 1]")
        if (type(self.frontier_history_s) not in (int, float)
                or not isfinite(self.frontier_history_s) or self.frontier_history_s < 0):
            raise ValueError("frontier history window must be finite and nonnegative")
        if (type(self.gain_ray_step_rad) not in (int, float)
                or not isfinite(self.gain_ray_step_rad) or not 0.001 <= self.gain_ray_step_rad <= 1.0):
            raise ValueError("gain ray step must be in [0.001, 1] radians")


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
    # Optional stopped observation after reaching an observed target approach.
    # Physics and regular RGB-D captures continue; this does not assert success.
    inspection_hold_s: float = 0.0
    inspection_headings_rad: Tuple[float, ...] = (-0.15, 0.0, 0.15)


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

    def __post_init__(self) -> None:
        for name in ('minimum_visibility', 'minimum_bbox_iou'):
            value = getattr(self, name)
            if type(value) not in (int, float) or not isfinite(value) or not 0 < value <= 1:
                raise ValueError('%s must be finite and in (0, 1]' % name)
        if type(self.required_evidence_frames) is not int or self.required_evidence_frames < 2:
            raise ValueError('required evidence frames must be an integer of at least two')
        for name in ('localization_tolerance_m', 'deadline_s'):
            value = getattr(self, name)
            if type(value) not in (int, float) or not isfinite(value) or value <= 0:
                raise ValueError('%s must be finite and positive' % name)


@dataclass(frozen=True)
class EpisodeLimits:
    max_attempts: int = 20
    max_steps: int = 10000
    max_simulated_time_s: float = 300.0
    max_wall_time_s: float = 600.0
    max_decisions: int = 50
