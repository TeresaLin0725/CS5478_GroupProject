"""Configuration records. Defaults are synthetic development values, not a freeze.

Before real integration, extend these records with the selected sensor, detector,
motion-model and algorithm settings listed in interface_freeze.md section 8.
"""


from dataclasses import dataclass
from math import isfinite
from typing import Optional, Tuple


@dataclass(frozen=True)
class SensorNoiseConfig:
    """Opt-in depth corruption; experimental levels, not a calibrated sensor model."""
    enabled: bool = False
    depth_std_m: float = 0.01
    depth_dropout_prob: float = 0.01
    seed: int = 0

    def __post_init__(self) -> None:
        if type(self.enabled) is not bool:
            raise ValueError('sensor noise enabled must be a bool')
        if (type(self.depth_std_m) not in (int, float)
                or not isfinite(self.depth_std_m) or self.depth_std_m < 0):
            raise ValueError('depth_std_m must be finite and nonnegative')
        if (type(self.depth_dropout_prob) not in (int, float)
                or not isfinite(self.depth_dropout_prob) or not 0 <= self.depth_dropout_prob <= 1):
            raise ValueError('depth_dropout_prob must be between 0 and 1')
        if type(self.seed) is not int or not 0 <= self.seed < 2**32:
            raise ValueError('noise seed must be an integer in [0, 2**32)')


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
    # Opt-in spatial denoising for map fusion, independent of simulated noise.
    depth_filter_kernel_size: int = 0
    depth_filter_max_delta_m: float = 0.08
    # Optional fallback viewpoints outside the legacy local probe ring.
    unvisited_recovery_viewpoints: bool = False
    recovery_viewpoint_spacing_m: float = 0.6
    recovery_viewpoint_range_m: float = 3.0
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
        if type(self.unvisited_recovery_viewpoints) is not bool:
            raise ValueError('unvisited recovery viewpoints must be bool')
        for value in (self.recovery_viewpoint_spacing_m, self.recovery_viewpoint_range_m):
            if type(value) not in (int, float) or not isfinite(value) or value <= 0:
                raise ValueError('recovery viewpoint distances must be finite and positive')
        if self.recovery_viewpoint_range_m < self.recovery_viewpoint_spacing_m:
            raise ValueError('recovery viewpoint range must cover the spacing')
        if type(self.depth_filter_kernel_size) is not int or self.depth_filter_kernel_size not in (0, 3, 5):
            raise ValueError('depth filter kernel must be 0 (off), 3 or 5')
        if (type(self.depth_filter_max_delta_m) not in (int, float)
                or not isfinite(self.depth_filter_max_delta_m) or self.depth_filter_max_delta_m <= 0):
            raise ValueError('depth filter range threshold must be finite and positive')
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
    # 1 preserves the original entry tolerance; smaller values leave drift room.
    inspection_heading_settle_ratio: float = 1.0

    def __post_init__(self) -> None:
        value = self.inspection_heading_settle_ratio
        if type(value) not in (int, float) or not isfinite(value) or not 0 < value <= 1:
            raise ValueError('inspection_heading_settle_ratio must be finite and in (0, 1]')


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
    # Opt-in recovery from over-conservative exclusions (all defaults keep behaviour unchanged).
    # epsilon_retry: per decision, with this probability probe a geometry-excluded frontier instead of the policy's choice
    #   (epsilon-greedy, Sutton and Barto: exploit the policy, rarely try what the model rejected).
    # stuck_retry: when nothing is eligible and the usual recovery scan and cooldown wait are spent, dispatch one
    #   geometry-excluded frontier deterministically instead of ending with no_feasible_frontiers (a trapped robot produces no
    #   new evidence, so evidence_change exclusions would otherwise never lift).
    # Both require every active restriction to be no_path/invalid_goal; cooldowns and hard caps are never waived.
    epsilon_retry: float = 0.0
    stuck_retry: bool = False


@dataclass(frozen=True)
class PolicyConfig:
    epsilon_m: float = 0.1
    llm_model: Optional[str] = None
    llm_reasoning: Optional[str] = None
    llm_deadline_s: float = 5.0
    llm_call_cap: int = 10
    llm_prompt: str = (
        "Choose the best eligible frontier for finding the target. Use semantic evidence, "
        "information gain, assessed path cost, exclusions, and recent execution feedback."
    )
    # C keeps hosted-model calls for decisions where semantics can change the action.
    # Target approaches and a single eligible frontier are deterministic and safe to bypass.
    llm_bypass_single_candidate: bool = True
    # Reuse only an exact material decision context; revisions/timestamps alone do not
    # invalidate a choice, while candidates, costs, observations and feedback do.
    llm_cache_identical_context: bool = True
    # Bound prompt growth independently of episode length.
    llm_feedback_window: int = 8
    llm_max_observed_objects: int = 24
    # System prompt variant; 'v1' is the original wording, kept byte-identical.
    llm_prompt_version: str = 'v1'

    def __post_init__(self) -> None:
        if self.llm_prompt_version not in ('v1', 'v2'):
            raise ValueError("llm_prompt_version must be 'v1' or 'v2'")
        if not isfinite(self.epsilon_m) or self.epsilon_m <= 0:
            raise ValueError("epsilon_m must be finite and positive")
        if not isfinite(self.llm_deadline_s) or self.llm_deadline_s <= 0:
            raise ValueError("LLM deadline must be finite and positive")
        if type(self.llm_call_cap) is not int or self.llm_call_cap < 0:
            raise ValueError("LLM call cap must be nonnegative")
        if type(self.llm_feedback_window) is not int or self.llm_feedback_window < 0:
            raise ValueError("LLM feedback window must be a nonnegative integer")
        if type(self.llm_max_observed_objects) is not int or self.llm_max_observed_objects < 1:
            raise ValueError("LLM observed-object cap must be a positive integer")
        if type(self.llm_bypass_single_candidate) is not bool:
            raise ValueError("LLM single-candidate bypass must be a bool")
        if type(self.llm_cache_identical_context) is not bool:
            raise ValueError("LLM context caching must be a bool")


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
