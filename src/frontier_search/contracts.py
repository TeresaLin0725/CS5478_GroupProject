"""Shared contract 1.0 records from Documentation/english/interface_freeze.md.

Collections use tuples for deeply immutable owned snapshots. Behavioral boundary
checks live in validation.py; these types contain no evaluator capability.
"""

from __future__ import annotations


from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple

from .config import (MappingConfig, ObservationConfig, PolicyConfig,
                     NavigationConfig, EvaluationConfig)
from .immutable import ImmutableRecord
from .sensors import ImageBuffer, CameraIntrinsics, Transform

@dataclass(frozen=True)
class Point2(ImmutableRecord):
    x_m: float
    y_m: float


@dataclass(frozen=True)
class Point3(ImmutableRecord):
    x_m: float
    y_m: float
    z_m: float


@dataclass(frozen=True)
class Pose(ImmutableRecord):
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float


@dataclass(frozen=True)
class GridCell(ImmutableRecord):
    row: int
    column: int


@dataclass(frozen=True)
class RobotConfig(ImmutableRecord):
    footprint_base: Tuple[Point2, ...]  # polygon in robot-base coordinates
    collision_z_min_m: float         # relative to base
    collision_z_max_m: float
    clearance_margin_m: float
    motion_model: str
    can_turn_in_place: bool
    max_linear_speed_mps: float
    max_angular_speed_radps: float
    position_tolerance_m: float
    yaw_tolerance_rad: float


@dataclass(frozen=True)
class OccupancyGrid(ImmutableRecord):
    frame_id: str                    # "world"
    revision: int
    resolution_m: float
    origin_x_m: float
    origin_y_m: float
    width_cells: int
    height_cells: int
    data: Tuple[Tuple[int, ...], ...]    # -1, 0, 100


@dataclass(frozen=True)
class ExploredGrid(ImmutableRecord):
    frame_id: str                    # "world"
    revision: int
    resolution_m: float
    origin_x_m: float
    origin_y_m: float
    width_cells: int
    height_cells: int
    data: Tuple[Tuple[bool, ...], ...]   # accepted geometric sensor evidence


@dataclass(frozen=True)
class ObjectTrack(ImmutableRecord):
    track_id: str
    class_name: str
    confidence: float
    position_world: Optional[Point3]
    first_seen_s: float
    last_seen_s: float
    observation_ids: Tuple[str, ...]
    confirmation_count: int          # distinct accepted capture sequences


@dataclass(frozen=True)
class DetectionEvidence(ImmutableRecord):
    observation_id: str
    capture_sequence: int
    timestamp_s: float
    track_id: str
    class_name: str
    confidence: float
    bbox_xyxy_px: Tuple[int, int, int, int]
    position_world: Optional[Point3]


@dataclass(frozen=True)
class TargetClaim(ImmutableRecord):
    claim_id: str
    track_id: str
    target_class: str
    timestamp_s: float
    evidence_observation_ids: Tuple[str, ...]


@dataclass(frozen=True)
class EvidenceBundle(ImmutableRecord):
    episode_id: str
    claim_id: str
    detections: Tuple[DetectionEvidence, ...]


@dataclass(frozen=True)
class RegionPrior(ImmutableRecord):
    region_id: str
    boundary_world: Tuple[Point2, ...]  # complete polygon, world x/y
    room_type: Optional[str]            # only in oracle-label condition


@dataclass(frozen=True)
class RegionSummary(ImmutableRecord):
    region_id: str
    boundary_world: Tuple[Point2, ...]
    boundary_source: str             # "igibson_room_prior"
    explored_fraction: Optional[float]
    observed_object_classes: Tuple[str, ...]
    room_type: Optional[str]
    room_type_source: Optional[str]     # "simulator_annotation" in oracle condition


@dataclass(frozen=True)
class Frontier(ImmutableRecord):
    frontier_id: str
    parent_frontier_ids: Tuple[str, ...]
    support_cells: Tuple[GridCell, ...]
    approach_pose: Pose               # heading toward unknown space
    region_id: Optional[str]
    information_gain_m2: float
    distance_from_robot_m: float      # Euclidean diagnostic only


@dataclass(frozen=True)
class WorldState(ImmutableRecord):
    schema_version: str              # "1.0"
    episode_id: str
    revision: int
    timestamp_s: float
    last_processed_sequence: int
    target_class: str
    robot_pose: Pose
    occupancy_map: OccupancyGrid
    explored_map: ExploredGrid
    semantic_objects: Tuple[ObjectTrack, ...]
    regions: Tuple[RegionSummary, ...]
    frontiers: Tuple[Frontier, ...]
    target_claim: Optional[TargetClaim]


class FailureReason(Enum):
    NO_PATH = "no_path"
    BLOCKED = "blocked"
    TIMEOUT = "timeout"
    INVALID_GOAL = "invalid_goal"


@dataclass(frozen=True)
class CandidateAssessment(ImmutableRecord):
    frontier_id: str
    reachable: Optional[bool]           # None: inconclusive
    path_cost_m: Optional[float]
    failure_reason: Optional[FailureReason]


@dataclass(frozen=True)
class CandidateEvaluationRequest(ImmutableRecord):
    episode_id: str
    state: WorldState


@dataclass(frozen=True)
class CandidateEvaluation(ImmutableRecord):
    episode_id: str
    state_revision: int
    start_pose: Pose
    assessments: Tuple[CandidateAssessment, ...]


@dataclass(frozen=True)
class PlannerResult(ImmutableRecord):
    success: bool
    path: Optional[Tuple[Pose, ...]]
    path_cost_m: Optional[float]
    failure_reason: Optional[FailureReason]


@dataclass(frozen=True)
class MapperInit(ImmutableRecord):
    episode_id: str
    target_class: str
    seed: int
    mapping_config: MappingConfig
    observation_config: ObservationConfig
    robot_config: RobotConfig
    region_priors: Tuple[RegionPrior, ...]
    initial_clearance_cells: Tuple[GridCell, ...]
    oracle_room_labels: bool


@dataclass(frozen=True)
class RGBDObservation(ImmutableRecord):
    episode_id: str
    sequence: int
    timestamp_s: float
    color_image: ImageBuffer
    depth_image: ImageBuffer
    camera_intrinsics: CameraIntrinsics
    camera_to_world: Transform
    robot_pose: Pose


class PolicyName(Enum):
    RANDOM = "random"
    NEAREST = "nearest"
    INFORMATION_GAIN = "information_gain"
    LLM_GUIDED = "llm_guided"


@dataclass(frozen=True)
class Subgoal(ImmutableRecord):
    frontier_id: str
    approach_pose: Pose
    selected_from_revision: int
    policy: PolicyName


@dataclass(frozen=True)
class PlannerAttempt(ImmutableRecord):
    subgoal: Subgoal
    result: Optional[PlannerResult]    # may stop before a plan exists
    execution_status: str          # reached, failed, cancelled
    failure_reason: Optional[FailureReason]


@dataclass(frozen=True)
class FrontierExclusion(ImmutableRecord):
    frontier_id: str
    reason: str                    # failure enum value or "no_progress"
    attempt_count: int
    retry_condition: str           # cooldown, evidence_change, cooldown_or_change, exhausted
    retry_after_s: Optional[float]    # simulation time, only for cooldown conditions


@dataclass(frozen=True)
class SelectionContext(ImmutableRecord):
    episode_id: str
    evaluation: CandidateEvaluation
    eligible_frontier_ids: Tuple[str, ...]
    exclusions: Tuple[FrontierExclusion, ...]
    previous_attempt: Optional[PlannerAttempt]


@dataclass(frozen=True)
class SelectionResult(ImmutableRecord):
    episode_id: str
    subgoal: Optional[Subgoal]
    stop_reason: Optional[str]        # target_claimed, no_frontiers, no_eligible_frontiers
    rationale: Optional[str]


class NavigationStatus(Enum):
    ACCEPTED = "accepted"
    EXECUTING = "executing"
    REACHED = "reached"             # operation completed
    FAILED = "failed"
    CANCELLED = "cancelled"


@dataclass(frozen=True)
class NavigatorInit(ImmutableRecord):
    episode_id: str
    robot_config: RobotConfig
    observation_config: ObservationConfig
    navigation_config: NavigationConfig


@dataclass(frozen=True)
class NavigationRequest(ImmutableRecord):
    episode_id: str
    goal_id: str
    occupancy_map: OccupancyGrid
    start_pose: Pose                 # live pose at dispatch
    subgoal: Subgoal


@dataclass(frozen=True)
class ObservationRequest(ImmutableRecord):
    episode_id: str
    goal_id: str
    occupancy_map: OccupancyGrid
    start_pose: Pose
    routine: str                     # initial, arrival, recovery
    heading_rad: float               # reference heading for configured sweep


@dataclass(frozen=True)
class NavigationFeedback(ImmutableRecord):
    episode_id: str
    goal_id: str
    timestamp_s: float
    status: NavigationStatus
    robot_pose: Pose
    distance_remaining_m: Optional[float]
    planner_result: Optional[PlannerResult]
    failure_reason: Optional[FailureReason]


@dataclass(frozen=True)
class EvaluationRequest(ImmutableRecord):
    episode_id: str
    claim: TargetClaim
    evidence: EvidenceBundle


@dataclass(frozen=True)
class EvaluationResult(ImmutableRecord):
    episode_id: str
    claim_id: str
    valid_target: bool
    matched_instance_id: Optional[str]
    reason: str
