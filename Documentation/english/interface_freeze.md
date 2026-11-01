# Interface Freeze

The current close-object 3D demo opts into the
[1.1 close-object extension](close_object_search.md): typed observed-target
approach goals, full-body motion and independently verified arrival at the cup.
The initial 1.0 definition below remains the contract for legacy fixtures.

The current native adapter also supplies the backward-compatible optional
`RGBDObservation.odometry_linear_speed_mps` and
`odometry_angular_speed_radps` magnitudes. Both are absent, or both are finite
and nonnegative. They are derived from the latest 30 Hz **known robot poses**,
not from hidden scene state. See the close-object extension for their use in
the shared stillness gate; the lower-rate camera interval can alias base motion.

This document defines the initial behavioral contract, version **1.0**, for [the implementation plan](robotics_project_detailed_plan.md). Python-like signatures describe data and ownership, not a required serialization library. Numerical configuration is recorded in the experiment manifest before integration.

## 1. Module Boundaries and Ownership

| Module | Input | Output | Owner |
|---|---|---|---|
| PerceptionMapper | Episode priors/configuration, RGB-D and poses | Immutable WorldState and claim evidence | A |
| Navigator | Robot configuration, map/pose, candidate poses or motion request | CandidateEvaluation and NavigationFeedback | B |
| SearchPolicy | WorldState, SelectionContext, policy configuration | SelectionResult | C |
| Runner | Episode manifest and module APIs | Lifecycle, common recovery, logs and termination | C |
| Simulator adapter | Controls and simulator state | Captures, verified spawn clearance, isolated evaluator records | B |
| TargetEvaluator | Terminal claim, evidence and hidden scene records | EvaluationResult for logs only | A |

C owns integration; A and B supply module fixtures/checks. The evaluator is a separate capability, not a member of WorldState.

## 2. Shared Conventions and Priors

- **Scope:** One floor, static scene, visual target-class detection. No approach-distance requirement, manipulation, or SLAM initially.
- **World:** Right-handed, gravity-aligned, fixed per episode. x/y horizontal metres, z height, yaw radians about +z, positive counter-clockwise viewed from above.
- **Grid:** Origin is the lower-left corner of cell (row=0, column=0). Rows increase with world y; columns with x. Cell centres are `origin + (index + 0.5) * resolution_m`. Bounds/resolution stay fixed per episode; out-of-bounds is non-traversable. Occupancy and explored grids share geometry/revision.
- **Occupancy:** -1 unknown, 0 known free, 100 occupied. Unknown is not free for collision checking. A publishes uninflated evidence; B derives its costmap.
- **Time:** Simulation timestamps are nondecreasing per episode. Capture sequence increases for every frame, including skipped frames; processed sequences strictly increase. Measure wall time separately with a monotonic clock.
- **Identity:** IDs are unique per entity type/episode and never reused within an episode. Nested records inherit episode identity; public request/response envelopes carry episode_id.
- **Snapshots:** WorldState revisions strictly increase. Snapshots are deeply immutable: frozen dataclasses alone do not freeze nested lists/arrays. Use read-only owned buffers or equivalent lifetime-safe snapshots.
- **Values:** Unknown uses None, not zero. Numbers are finite; confidence/fractions lie in [0,1]. Empty candidates are valid observations, not API errors.
- **Allowed priors:** Complete room polygons, simulator robot pose, calibrated capture-time camera transforms, and verified initial footprint clearance. Room geometry reveals unseen layout, though not obstacle occupancy. This is a known-room-layout/known-pose experiment.
- **Spawn clearance:** The adapter verifies a collision-free spawn and supplies only the initial footprint-clearance mask. A may seed these cells as free. No additional ground-truth free space is exposed. Reject initialization if clearance cannot be verified.
- **Optional experimental extension:** `verified_start_area` permits a bounded, connected initial mask only with B's scene-bound clearance and floor-support attestation. Footprint-only remains the default; see [schema, validation and ownership](verified_start_area.md). All later occupancy still comes from observations.
- **Hidden data:** Full occupancy, object locations/instance IDs, and semantic room labels are excluded from robot modules. Labels may be enabled only in a named oracle-label condition shared across policies. Use opaque room IDs, not semantic simulator names. The evaluator alone receives hidden matching data.

## 3. Core Data Contracts

### 3.1 Geometry and configuration

```python
@dataclass(frozen=True)
class Point2:
    x_m: float
    y_m: float

@dataclass(frozen=True)
class Point3:
    x_m: float
    y_m: float
    z_m: float

@dataclass(frozen=True)
class Pose:
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float

@dataclass(frozen=True)
class GridCell:
    row: int
    column: int

@dataclass(frozen=True)
class RobotConfig:
    footprint_base: Sequence[Point2]  # polygon in robot-base coordinates
    collision_z_min_m: float         # relative to base
    collision_z_max_m: float
    clearance_margin_m: float
    motion_model: str
    can_turn_in_place: bool
    max_linear_speed_mps: float
    max_angular_speed_radps: float
    position_tolerance_m: float
    yaw_tolerance_rad: float
```

Pose uses world coordinates unless stated otherwise. Motion-model-specific constraints (for example acceleration or minimum turning radius) are additional required configuration when applicable. Do not silently assume differential drive.

MappingConfig, ObservationConfig, RecoveryConfig, PolicyConfig, NavigationConfig, and EvaluationConfig are validated configuration records, not hidden globals. Required settings appear in section 8. ImageBuffer, CameraIntrinsics, and Transform are adapter types whose encodings/conventions must also be fixed there.

### 3.2 Occupancy and geometric coverage

```python
@dataclass(frozen=True)
class OccupancyGrid:
    frame_id: str                    # "world"
    revision: int
    resolution_m: float
    origin_x_m: float
    origin_y_m: float
    width_cells: int
    height_cells: int
    data: Sequence[Sequence[int]]    # -1, 0, 100

@dataclass(frozen=True)
class ExploredGrid:
    frame_id: str                    # "world"
    revision: int
    resolution_m: float
    origin_x_m: float
    origin_y_m: float
    width_cells: int
    height_cells: int
    data: Sequence[Sequence[bool]]   # accepted geometric sensor evidence
```

Dimensions match the buffers. Except for the spawn seed, occupancy comes from robot observations. A uses a documented depth/floor/height model: a ray above the collision envelope or an observed tabletop does not prove clearance at robot height. Invalid depth stays unknown unless other accepted evidence exists.

ExploredGrid represents geometric observation, not exhaustive object inspection. Spawn clearance alone does not mark explored cells. Both grids carry the current state revision even if one buffer's contents did not change.

### 3.3 Object tracks and target evidence

```python
@dataclass(frozen=True)
class ObjectTrack:
    track_id: str
    class_name: str
    confidence: float
    position_world: Point3 | None
    first_seen_s: float
    last_seen_s: float
    observation_ids: Sequence[str]
    confirmation_count: int          # distinct accepted capture sequences

@dataclass(frozen=True)
class DetectionEvidence:
    observation_id: str
    capture_sequence: int
    timestamp_s: float
    track_id: str
    class_name: str
    confidence: float
    bbox_xyxy_px: tuple[int, int, int, int]
    position_world: Point3 | None

@dataclass(frozen=True)
class TargetClaim:
    claim_id: str
    track_id: str
    target_class: str
    timestamp_s: float
    evidence_observation_ids: Sequence[str]

@dataclass(frozen=True)
class EvidenceBundle:
    episode_id: str
    claim_id: str
    detections: Sequence[DetectionEvidence]
```

A owns association and confirmation. Depth-insufficient tracks may remain semantic evidence, but initial target confirmation requires a localized, spatially consistent track across at least two distinct frames. Freeze confidence, temporal window, and association thresholds. Reprocessing a frame cannot increase confirmation_count.

The adapter retains RGB-D/calibration and evaluator-only ground-truth records keyed by episode/capture sequence. Retention covers the confirmation window and preserves claim evidence through scoring. Class names use an explicit detector-to-target label mapping.

A emits at most one terminal claim per episode. It requests a stop, not experimental success. C cannot modify claims. Detailed evidence is retrieved by the runner for scoring, not supplied to C.

### 3.4 Regions

```python
@dataclass(frozen=True)
class RegionPrior:
    region_id: str
    boundary_world: Sequence[Point2]  # complete polygon, world x/y
    room_type: str | None            # only in oracle-label condition

@dataclass(frozen=True)
class RegionSummary:
    region_id: str
    boundary_world: Sequence[Point2]
    boundary_source: str             # "igibson_room_prior"
    explored_fraction: float | None
    observed_object_classes: Sequence[str]
    room_type: str | None
    room_type_source: str | None     # "simulator_annotation" in oracle condition
```

Complete boundaries are required. explored_fraction is True cells divided by all cell centres inside the polygon under a fixed rasterization rule; zero denominator yields None. Validate that the grid contains the intended polygons. This is geometric coverage, not target-absence confidence.

Standard room_type/source are both None; the oracle-label condition fills both. C may infer likely types without editing A's state. Unknown object positions must not default to association with the robot's current room.

### 3.5 Frontiers and identity

```python
@dataclass(frozen=True)
class Frontier:
    frontier_id: str
    parent_frontier_ids: Sequence[str]
    support_cells: Sequence[GridCell]
    approach_pose: Pose               # heading toward unknown space
    region_id: str | None
    information_gain_m2: float
    distance_from_robot_m: float      # Euclidean diagnostic only
    alternative_approach_poses: Sequence[Pose] = ()  # ranked after approach_pose (1.2)
```

A deterministically clusters known-free cells adjacent to unknown cells. Approach positions are known-free; B verifies full-footprint validity/reachability. Gain is nonnegative predicted unknown area visible during the arrival sweep using camera range/FOV, deduplicated cells, and known occlusion. It is not hidden-map ground-truth gain.

Match clusters by overlap within fixed spatial tolerance, with deterministic tie-breaking. A one-to-one match preserves ID. Splits/merges retire affected IDs and give children all matched parent IDs. Persist lineage/spatial history through temporary disappearances for the configured recovery window. Parent IDs reference earlier entities, never descendants.

Runner exclusions/attempt counts follow lineage and spatial overlap. Relabeling cannot reset retry history; new IDs alone are not changed evidence. Keep compact attempt counts and exhausted-opportunity records for the entire episode even when detailed cluster history expires. Fixed grid geometry permits support-cell comparison across revisions.

### 3.6 World state

```python
@dataclass(frozen=True)
class WorldState:
    schema_version: str              # "1.0"
    episode_id: str
    revision: int
    timestamp_s: float
    last_processed_sequence: int
    target_class: str
    robot_pose: Pose
    occupancy_map: OccupancyGrid
    explored_map: ExploredGrid
    semantic_objects: Sequence[ObjectTrack]
    regions: Sequence[RegionSummary]
    frontiers: Sequence[Frontier]
    target_claim: TargetClaim | None
```

Each update is a complete snapshot with matching map revisions. Tracks may retain older observations; all collections represent accumulated state as of the same cutoff. robot_pose is the last processed capture pose, not necessarily the live control pose. At decision boundaries the runner obtains a fresh processed observation and current adapter pose.

### 3.7 Candidate evaluation and planning

```python
class FailureReason(Enum):
    NO_PATH = "no_path"
    BLOCKED = "blocked"
    TIMEOUT = "timeout"
    INVALID_GOAL = "invalid_goal"

@dataclass(frozen=True)
class CandidateAssessment:
    frontier_id: str
    reachable: bool | None           # None: inconclusive
    path_cost_m: float | None
    failure_reason: FailureReason | None
    approach_pose: Pose | None = None  # exact reachable pose selected by B (1.2)

@dataclass(frozen=True)
class CandidateEvaluationRequest:
    episode_id: str
    state: WorldState

@dataclass(frozen=True)
class CandidateEvaluation:
    episode_id: str
    state_revision: int
    start_pose: Pose
    assessments: Sequence[CandidateAssessment]

@dataclass(frozen=True)
class PlannerResult:
    success: bool
    path: Sequence[Pose] | None
    path_cost_m: float | None
    failure_reason: FailureReason | None
```

Evaluation returns one assessment per input frontier using the state's map/pose and execution motion model. True requires finite nonnegative cost and no failure reason. False requires NO_PATH or INVALID_GOAL and no cost. Inconclusive evaluation uses None/TIMEOUT/no cost, not proven unreachability. Evaluation is read-only and bounded by a batch deadline; a distance field may amortize work.

Successful PlannerResult contains a path, finite nonnegative cost, and no reason. Failed planning has a reason and no path/cost. Planning success does not imply execution success. Detailed evidence can be an optional later field.

## 4. Public APIs

### 4.1 Initialization and A

```python
@dataclass(frozen=True)
class MapperInit:
    episode_id: str
    target_class: str
    seed: int
    mapping_config: MappingConfig
    observation_config: ObservationConfig
    robot_config: RobotConfig
    region_priors: Sequence[RegionPrior]
    initial_clearance_cells: Sequence[GridCell]
    oracle_room_labels: bool

@dataclass(frozen=True)
class RGBDObservation:
    episode_id: str
    sequence: int
    timestamp_s: float
    color_image: ImageBuffer
    depth_image: ImageBuffer
    camera_intrinsics: CameraIntrinsics
    camera_to_world: Transform
    robot_pose: Pose

class PerceptionMapper:
    def reset(self, request: MapperInit) -> None: ...
    def update(self, observation: RGBDObservation) -> WorldState: ...
    def resolve_approaches(self, state: WorldState, evaluation: CandidateEvaluation) -> WorldState: ...
    def get_claim_evidence(self, episode_id: str, claim_id: str) -> EvidenceBundle: ...
```

Reset clears maps, tracks, frontiers, counters, and claims. The first state comes from a real observation after reset; errors cannot produce valid-looking empty updates. Target class stays fixed per episode.

A preserves capture sequence/time and validates encodings/transforms. It cannot use hidden scene data to infer occupancy, objects, or claims; permitted room, pose, calibration, and spawn-clearance priors are explicit exceptions. Oracle room labels require configuration authorization and source marking.

### 4.2 C and selection context

```python
class PolicyName(Enum):
    RANDOM = "random"
    NEAREST = "nearest"
    INFORMATION_GAIN = "information_gain"
    LLM_GUIDED = "llm_guided"

@dataclass(frozen=True)
class Subgoal:
    frontier_id: str
    approach_pose: Pose
    selected_from_revision: int
    policy: PolicyName

@dataclass(frozen=True)
class PlannerAttempt:
    subgoal: Subgoal
    result: PlannerResult | None    # may stop before a plan exists
    execution_status: str          # reached, failed, cancelled
    failure_reason: FailureReason | None

@dataclass(frozen=True)
class FrontierExclusion:
    frontier_id: str
    reason: str                    # failure enum value or "no_progress"
    attempt_count: int
    retry_condition: str           # cooldown, evidence_change, cooldown_or_change, exhausted
    retry_after_s: float | None    # simulation time, only for cooldown conditions

@dataclass(frozen=True)
class SelectionContext:
    episode_id: str
    evaluation: CandidateEvaluation
    eligible_frontier_ids: Sequence[str]
    exclusions: Sequence[FrontierExclusion]
    previous_attempt: PlannerAttempt | None

@dataclass(frozen=True)
class SelectionResult:
    episode_id: str
    subgoal: Subgoal | None
    stop_reason: str | None        # target_claimed, no_frontiers, no_eligible_frontiers
    rationale: str | None

class SearchPolicy:
    def reset(self, episode_id: str, seed: int, config: PolicyConfig) -> None: ...
    def select(self, state: WorldState, context: SelectionContext) -> SelectionResult: ...
```

- Evaluation matches state episode/revision/pose. Eligible IDs are exactly reachable=True assessments minus shared runner exclusions. No policy-specific candidate filtering/truncation in the baseline.
- Exclusions distinguish cooldown, evidence-change, either condition, and exhausted retry budgets. retry_after_s is set only when cooldown is an allowed trigger; exhausted opportunities never become eligible again that episode. Reassessment is required after any unlock.
- C selects only an eligible frontier in state.frontiers. Subgoal ID/full pose, including yaw, match A's candidate; selected revision matches the assessed state.
- Exactly one of subgoal/stop_reason is set. A claim takes precedence as target_claimed. No frontiers returns no_frontiers; no eligible candidates returns no_eligible_frontiers. These request runner action, not immediate exhaustion.
- Random is uniform; Nearest minimizes path_cost_m; Information Gain maximizes gain divided by path_cost_m plus fixed positive epsilon_m. Ties use stable ID order. All policies receive the same context structure.
- The LLM receives observed semantics, geometric coverage, gain, costs, and shared recovery context. Boundaries exist in WorldState but may be omitted from the compact prompt. Missing labels/coverage remain unknown. Hidden state, images, and evaluator outcomes are excluded.
- The adapter validates the returned ID and constructs Subgoal. Invalid/timeout output triggers deterministic Nearest fallback within the decision deadline and logs a reason. No unbounded retry loop.

### 4.3 B: navigation and observation motion

```python
class NavigationStatus(Enum):
    ACCEPTED = "accepted"
    EXECUTING = "executing"
    REACHED = "reached"             # operation completed
    FAILED = "failed"
    CANCELLED = "cancelled"

@dataclass(frozen=True)
class NavigatorInit:
    episode_id: str
    robot_config: RobotConfig
    observation_config: ObservationConfig
    navigation_config: NavigationConfig

@dataclass(frozen=True)
class NavigationRequest:
    episode_id: str
    goal_id: str
    occupancy_map: OccupancyGrid
    start_pose: Pose                 # live pose at dispatch
    subgoal: Subgoal

@dataclass(frozen=True)
class ObservationRequest:
    episode_id: str
    goal_id: str
    occupancy_map: OccupancyGrid
    start_pose: Pose
    routine: str                     # initial, arrival, recovery
    heading_rad: float               # reference heading for configured sweep

@dataclass(frozen=True)
class NavigationFeedback:
    episode_id: str
    goal_id: str
    timestamp_s: float
    status: NavigationStatus
    robot_pose: Pose
    distance_remaining_m: float | None
    planner_result: PlannerResult | None
    failure_reason: FailureReason | None

class Navigator:
    def reset(self, request: NavigatorInit) -> None: ...
    def evaluate_candidates(self, request: CandidateEvaluationRequest) -> CandidateEvaluation: ...
    def start(self, request: NavigationRequest) -> NavigationFeedback: ...
    def start_observation(self, request: ObservationRequest) -> NavigationFeedback: ...
    def update_map(self, episode_id: str, goal_id: str, occupancy_map: OccupancyGrid) -> None: ...
    def poll(self, episode_id: str, goal_id: str) -> NavigationFeedback: ...
    def cancel(self, episode_id: str, goal_id: str, reason: str) -> NavigationFeedback: ...
```

B is bound to the simulator/control adapter. The runner advances simulation; B's controller executes at the configured cadence on those steps. Poll reads status without advancing simulation. Allow at most one active navigation/observation operation; overlapping starts raise a documented boundary exception. Reset requires the prior operation to be stopped and clears episode state.

The **runner**, not SearchPolicy, calls `start_observation` at the prescribed initialization, arrival, and recovery points. B turns the robot with its fixed camera; A processes the resulting observations. For example: C selects F3 → B navigates to F3 → runner requests the arrival scan → B executes the scan while A observes → runner waits for fresh state and refreshes candidate assessments → C selects again. A target claim or stopping condition can interrupt this sequence. This shared routine keeps the initial four-policy comparison focused on frontier selection; policy-selected scanning is a later extension.

- start validates ID, map/pose freshness, and full-footprint validity; it returns ACCEPTED or immediate FAILED. Accepted work can fail/cancel before EXECUTING. One terminal outcome is latched; later polls return that outcome, not a new terminal event.
- B executes only the selected goal. Check footprint clearance over segments/rotations, unknown/out-of-bounds, motion constraints, and corner cutting. Arrival requires both position and yaw tolerances.
- Accept newer maps only for the same episode and grid geometry. Recheck remaining paths and replan to the same goal if needed. Stop unsafe motion before replanning; never select another frontier.
- Publish PlannerResult as planning completes and retain it in feedback. A successful plan can be followed by execution FAILED with its own failure_reason. After a completed replan feedback carries the newest result; log each change.
- FAILED requires FailureReason; other statuses have failure_reason=None. PlannerResult can remain None on pre-plan validation failure/cancellation or observation motion. Reached navigation requires a successful plan; observation need not have one.
- cancel stops motion and is idempotent, returning CANCELLED unless already terminal. Cancellation does not count as a failed frontier.
- Observation routines use fixed configuration and checked motion. Full rotations require capability and clearance. Unsafe routines fail rather than rotating blindly; implicit recovery cannot be unbounded.
- Observation REACHED means motion/dwell completed. The runner separately enforces A's processed-frame barrier before selecting again. Observation time and motion count against budgets.

### 4.4 Evaluation-only scoring

```python
@dataclass(frozen=True)
class EvaluationRequest:
    episode_id: str
    claim: TargetClaim
    evidence: EvidenceBundle

@dataclass(frozen=True)
class EvaluationResult:
    episode_id: str
    claim_id: str
    valid_target: bool
    matched_instance_id: str | None
    reason: str

class TargetEvaluator:
    # Constructed with isolated access to retained scene/frame records.
    def reset(self, episode_id: str, config: EvaluationConfig) -> None: ...
    def evaluate(self, request: EvaluationRequest) -> EvaluationResult: ...
```

Freeze visible-instance/class matching, pixel-overlap or localization tolerances, minimum visibility, and required evidence frames in the manifest. Require the requested class in the claimed observations, not simply somewhere in the scene. Results and simulator IDs go only to terminal logs. Verifier errors yield evaluation_error, never automatic success.

## 5. Runner Control Flow and Recovery

### 5.1 Decision and motion lifecycle

1. Validate the manifest; verify spawn clearance; reset A, B, C, and evaluator. Capture an initial frame before observation motion so B has a map.
2. Execute the fixed initial observation routine while advancing simulation, processing RGB-D, and checking claims/limits. Unsafe motion yields a typed failure and only bounded recovery.
3. At a decision boundary, stop the robot, drain observations through the frame barrier, and pause simulation. Obtain state and B's evaluation, apply recovery exclusions, and call C with matching revisions.
4. Validate selection. If map/pose changes before dispatch, reassess the candidate. Refresh/reselect if identity or approach pose changed. A global revision bump alone does not invalidate an unchanged candidate, but B revalidates its path against the current map. Log assessment/selection/dispatch revisions.
5. Resume simulation and start B. Continue A updates, forward maps, and poll. The runner independently enforces step, simulated-time, wall-clock, operation, and decision limits, including a hanging LLM call.
6. On navigation REACHED, retain PlannerAttempt, execute arrival observation around the required heading, and wait for fresh processed frames. On FAILED, record reason and apply recovery. On CANCELLED, log cause without marking a failed frontier.
7. A target claim preempts motion/decisions. Cancel B, freeze evidence, and end as success or false_target_claim after scoring. Never return evaluator results to C.
8. Limits cancel motion and end as search_limit. Measure actual executed distance; planned cost is not executed distance.

**Frame barrier:** At each configured scan heading, capture the required samples after heading/dwell criteria are satisfied. A must process designated frames even if it ordinarily subsamples. After motion completion, process a frame at or after the terminal simulation timestamp. Enforce a wall-clock drain deadline; failed/missing perception is pipeline_error, not successful observation or fabricated empty state.

### 5.2 Failure and no-progress policy

The runner maintains history by lineage and spatial support and projects exclusions into SelectionContext.

- NO_PATH/INVALID_GOAL suppress until relevant evidence changes: traversability in the failed route/connected component, approach geometry, or robot reachability component. Without detailed route evidence, conservatively compare changes in the robot's reachable component and candidate neighborhood. Object-label changes or unrelated revision bumps do not qualify.
- BLOCKED/TIMEOUT allow reassessment after simulation-time cooldown within retry caps. Evaluation timeout cannot become permanent NO_PATH. Evaluation retries are also bounded.
- Split/merge descendants inherit overlapping exclusions and cumulative attempts. Retained spatial history prevents temporary disappearance/reappearance from resetting caps.
- Freeze a local observation neighborhood at frontier dispatch. After arrival observation, count cells that changed from unknown to known there. Below threshold is no progress. Suppress that opportunity until cooldown or relevant evidence change; reaching its no-progress cap suppresses it for the episode.
- Relevant evidence may unlock geometric suppression but cannot reset hard per-opportunity or per-episode attempt caps.
- With no eligible candidates, perform at most one recovery observation cycle before the final exhaustion decision. Wait for the earliest pending cooldown while safely stopped and observing, within budgets, then reassess. End as no_frontiers or no_feasible_frontiers when no retry remains. A new navigation attempt permits another bounded recovery cycle; loops without navigation are also globally bounded.

Failed arrival observations enter bounded recovery; do not immediately reselect the reached position without new evidence. Initialization that cannot produce a usable state after bounded recovery ends as initialization_failed.

Final reasons: success, false_target_claim, no_frontiers, no_feasible_frontiers, search_limit, initialization_failed, pipeline_error, evaluation_error, externally_cancelled. Only success counts as success. Infrastructure errors remain visible and are reported separately, not silently discarded.

## 6. Experiment and Logging Contract

- Use paired scene/start/target configurations and repeated seeds. Isolate RNG streams so extra policy draws cannot change scene generation. Validate target presence and reachable observable viewpoints offline through evaluator-only access.
- Freeze detector weights/labels, simulator/assets, geometry, mapping, cadence, candidates/assessment, scans, recovery, budgets, and code revision or source snapshot hash.
- Pause simulation during assessment/selection for every policy. Record simulated time and wall time separately; LLM latency remains part of wall time. Limits are runner-owned.
- Log state/assessment revisions, features, eligible IDs/exclusions, selected pose, operation events, freshness, claims/evidence, verifier result, actual distance, collisions, replans, and terminal reason.
- For LLM calls log exact model/configuration, prompt/input, response, latency, usage/cost when available, fallback reason, and whether fallback selected the action. Preserve symbolic input for audit; a seed does not guarantee hosted-model replay.
- Report success/false-claim rates over all episodes, paired comparisons with uncertainty intervals, termination counts, and efficiency. Label success-conditioned distance/time and sample count. Report operational errors explicitly.
- Same candidates means identical generation/feasibility/recovery rules for a given state, not identical evolving lists across different trajectories.

## 7. Versioning, Validation, and Required Checks

Contract 1.0 is the first implementation target. Future breaking field/behavior changes require a major increment; optional additions may use a minor increment. Reject unsupported major versions and document boundary exceptions. API failures cannot masquerade as zero poses, empty maps, or arbitrary frontiers.

Validate episode identity, grid geometry/dimensions/values, finite values/ranges, sequence/revisions, deep immutability, ID/lineage references, and selection/assessment coherence at relevant boundaries.

Before main experiments, implement focused checks for:

- Hidden occupancy, object instances, and standard-run room labels absent from policy state; only declared priors allowed.
- Footprint clearance, unknown blocking, narrow passages, diagonal corners, and safe rotations.
- Equal eligible inputs on fixed state, path-cost use, rejection of invented/stale subgoals.
- Identity through split/merge/disappearance, changed-evidence and timeout retries, global caps.
- One terminal outcome, pre-plan cancellation, execution failure after successful planning.
- Processed-frame barriers and no-progress suppression after arrival.
- Distinct-frame confirmation, true/false scoring, no evaluator feedback into search.
- LLM invalid/timeout fallback and bounded termination for stalled modules.
- Paired manifest replay, actual-distance accounting, complete failed-episode logs.

## 8. Configuration to Fix Before Integration

Record values and units in one validated manifest. All policies share these unless a named ablation changes them.

| Configuration | Required decisions |
|---|---|
| Sensor/transform | RGB encoding, depth units/invalid values, image size, intrinsics/distortion, transform convention, range/FOV, capture/processing cadence |
| RobotConfig | Footprint, base/camera frames, collision height, clearance, motion model/limits, arrival tolerances |
| MappingConfig | Fixed grid, depth/floor/height/evidence model, coverage rasterization, frontier adjacency/clustering, identity overlap/tolerance/history window, approach heading, gain ray model; target label mapping, confidence, at least two distinct frames, confirmation window, localization/association tolerances, evidence retention |
| ObservationConfig | Initial/arrival/recovery heading sequences, heading tolerance, dwell, processed samples, rotation eligibility, operation/drain deadlines |
| RecoveryConfig | Relevant-change tolerance, cooldowns, evaluation/navigation retry caps, progress neighborhood/threshold, no-progress cap, hard opportunity/episode attempt caps |
| PolicyConfig | Policy/seed, positive epsilon_m, tie rule; LLM model/reasoning settings, prompt, call/deadline limits and Nearest fallback |
| NavigationConfig | Batch planning deadline, navigation deadline, same-goal replan bounds, pose/map freshness tolerances |
| Episode | Scene/target/start/seeds, priors, step/simulated-time/wall-time/decision limits, fixed decision-time pause |
| EvaluationConfig | Visible-target matching thresholds, supporting frames, evaluator deadline, success definition, reporting/uncertainty procedure |

The team records these choices once robot, scenes, and detector are selected and freezes them for comparable experiments.

For mock inputs, iGibson development helpers, and the transition to real module outputs, see [Mock Data and Independent Development](development_fixtures.md).
