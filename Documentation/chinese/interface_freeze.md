# 接口约定

本文定义[实施计划](robotics_project_detailed_plan.md)的初始行为约定，版本为 **1.0**。类似 Python 的签名用于描述数据及归属，不强制使用特定序列化库。数值配置应在集成前记录到实验配置清单中。

## 1. 模块边界与归属

| 模块 | 输入 | 输出 | 负责人 |
|---|---|---|---|
| PerceptionMapper | 回合先验与配置、RGB-D 及位姿 | 不可变 WorldState 和声明证据 | A |
| Navigator | 机器人配置、地图与位姿、候选位姿或运动请求 | CandidateEvaluation 和 NavigationFeedback | B |
| SearchPolicy | WorldState、SelectionContext、策略配置 | SelectionResult | C |
| Runner | 回合配置清单及模块 API | 生命周期、共用恢复、日志与终止 | C |
| 仿真器适配器 | 控制指令与仿真器状态 | 采集数据、已验证的出生位置净空、隔离的评估器记录 | B |
| TargetEvaluator | 终止搜索的目标声明、证据及隐藏场景记录 | 仅用于日志的 EvaluationResult | A |

C 负责集成；A 和 B 提供模块测试数据与检查。评估器是独立能力，不是 WorldState 的成员。

## 2. 共用约定与先验

- **范围：**单楼层、静态场景、通过视觉检测目标类别。初期不要求接近距离、物体操作或 SLAM。
- **世界坐标：**右手系，与重力方向对齐，每回合固定。x/y 为水平坐标，单位米；z 为高度；yaw 为绕 +z 轴的弧度，从上方看逆时针为正。
- **栅格：**原点为栅格 (row=0, column=0) 的左下角。行随世界 y 增加，列随 x 增加。单元中心为 `origin + (index + 0.5) * resolution_m`。每回合内边界和分辨率固定；越界区域不可通行。占据栅格与已探索栅格共用几何参数和修订号。
- **占据值：**-1 表示未知，0 表示已知空闲，100 表示被占据。碰撞检查不能把未知当作空闲。A 发布未膨胀的证据；B 构建自己的代价地图。
- **时间：**每回合内仿真时间戳非递减。每个采集帧的序列号均增加，包括被跳过的帧；已处理序列号严格递增。实际时间使用单调时钟单独测量。
- **身份：**ID 在每种实体类型及每回合内唯一，回合内不可复用。嵌套记录继承回合身份；公共请求与响应的外层结构携带 episode_id。
- **快照：**WorldState 修订号严格递增。快照须深度不可变：仅将 dataclass 设为 frozen，不能冻结嵌套列表或数组。使用由快照持有的只读缓冲区，或具有等效生命周期安全性的快照机制。
- **取值：**未知使用 None，不使用零。数值须有限；置信度和比例位于 [0,1]。空候选集合是合法观察，不是 API 错误。
- **允许的先验：**完整房间多边形、仿真器机器人位姿、经标定的采集时相机变换，以及已验证的初始占地净空。房间几何会揭示未观察布局，但不揭示障碍物占据情况。这是已知房间布局及已知位姿实验。
- **出生位置净空：**适配器验证无碰撞的出生位置，仅提供初始占地范围净空掩码。A 可以用这些单元初始化空闲区域。不暴露额外真值空闲空间。无法验证净空时拒绝初始化。
- **隐藏数据：**完整占据地图、物体位置与实例 ID、语义房间标签不得进入机器人模块。标签只能在有明确名称且所有策略共用的真值标签条件下启用。使用不透明房间 ID，不使用具有语义的仿真器名称。只有评估器接收隐藏的匹配数据。

## 3. 核心数据约定

### 3.1 几何与配置

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

除非另有说明，Pose 使用世界坐标。运动模型特有的约束（例如加速度或最小转弯半径）在适用时也是必需配置。不要默认假设机器人采用差速驱动。

MappingConfig、ObservationConfig、RecoveryConfig、PolicyConfig、NavigationConfig 和 EvaluationConfig 是经过验证的配置记录，不是隐藏全局变量。必需设置见第 8 节。ImageBuffer、CameraIntrinsics 和 Transform 是适配器类型，其编码与约定也必须在那里固定。

### 3.2 占据与几何覆盖

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

维度必须与缓冲区一致。除出生位置种子外，占据状态来自机器人观察。A 使用有明确说明的深度、地面及高度模型：从碰撞包络上方经过的射线，或被观察到的桌面，都不能证明机器人高度范围内有足够净空。除非存在其他有效证据，无效深度保持未知。

ExploredGrid 表示几何观察，不表示穷尽式物体检查。仅出生位置净空不能将单元标记为已探索。即使某个缓冲区内容未变，两张栅格也都携带当前状态修订号。

### 3.3 物体轨迹与目标证据

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

A 负责关联与确认。深度不足的轨迹可保留为语义证据，但初始目标确认要求至少两个不同帧中的轨迹已定位且空间一致。固定置信度、时间窗口和关联阈值。重复处理一帧不能增加 confirmation_count。

适配器按回合与采集序列号保留 RGB-D、标定数据，以及仅评估器可用的真值记录。保留期覆盖确认窗口，并将声明证据保留到评分完成。类别名称使用明确的检测器到目标标签映射。

A 每回合最多发出一个终止搜索的声明。它请求停止，不代表实验成功。C 不能修改声明。详细证据由运行器取出用于评分，不提供给 C。

### 3.4 区域

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

必须提供完整边界。按照固定栅格化规则，explored_fraction 等于多边形内值为 True 的单元数除以单元中心位于该多边形内的全部单元数；分母为零时返回 None。验证栅格包含预期多边形。这是几何覆盖比例，不是目标不存在的置信度。

标准条件下 room_type/source 均为 None；真值标签条件下才填充两者。C 可以推断可能类型，但不修改 A 的状态。物体位置未知时，不能默认将其关联到机器人当前房间。

### 3.5 前沿与身份

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

A 对与未知单元相邻的已知空闲单元进行确定性聚类。接近位置处于已知空闲区；B 验证完整占地轮廓的有效性及可达性。信息增益是到达扫描期间预计可见的未知面积，为非负数，依据相机量程、视场、去重单元和已知遮挡计算。它不是根据隐藏地图得到的真值增益。

在固定空间容差内按重叠匹配前沿簇，并采用确定性平局规则。一对一匹配保留 ID。分裂或合并时停用受影响 ID，并为子簇记录所有匹配的父级 ID。在配置的恢复窗口内，临时消失的簇仍保留谱系与空间历史。父级 ID 引用更早的实体，绝不能引用后代。

运行器的排除状态和尝试计数沿谱系及空间重叠传递。重新标记不能重置重试历史；新 ID 本身不是证据变化。即使详细簇历史过期，也要在整个回合内保留简要尝试计数及已耗尽机会的记录。固定栅格几何允许跨修订号比较构成前沿的单元。

### 3.6 世界状态

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

每次更新都是完整快照，地图修订号一致。轨迹可以保留较早观察；所有集合表示截至同一时刻的累计状态。robot_pose 是最后处理帧采集时的位姿，不一定是实时控制位姿。在决策边界，运行器获取新处理的观察和适配器当前位姿。

### 3.7 候选评估与规划

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

评估使用状态中的地图与位姿，以及执行所用运动模型，对每个输入前沿返回一个评估。True 要求有限且非负的代价，并且没有失败原因。False 要求 NO_PATH 或 INVALID_GOAL，且无代价。无法确定的评估使用 None/TIMEOUT/无代价，不表示已证明不可达。评估只读，并受批量截止时间限制；可利用距离场摊薄开销。

成功的 PlannerResult 包含路径、有限非负代价且无失败原因。规划失败时提供原因，不提供路径或代价。规划成功不意味着执行成功。详细证据可作为后续可选字段。

## 4. 公共 API

### 4.1 初始化与 A

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

Reset 清空地图、轨迹、前沿、计数器和声明。第一个状态来自重置后的真实观察；出错时不能生成看似有效的空更新。目标类别在回合内固定。

A 保留采集序列号与时间，并验证编码和变换。不得使用隐藏场景数据推断占据、物体或声明；允许的房间、位姿、标定和出生位置净空先验是明确例外。真值房间标签需要配置授权和来源标记。

### 4.2 C 与选择上下文

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

- 评估必须与状态的回合、修订号和位姿匹配。合格 ID 恰好等于 reachable=True 的评估集合减去运行器的共用排除项。基础版本不允许策略专属候选筛选或截断。
- 排除项区分冷却期、证据变化、二者之一，以及重试预算耗尽。只有允许冷却期触发时才设置 retry_after_s；耗尽的机会在本回合不再恢复资格。任何解除排除后都必须重新评估。
- C 只选择 state.frontiers 中的合格前沿。Subgoal 的 ID 和完整位姿（包括 yaw）必须与 A 的候选一致；选择修订号与被评估状态一致。
- subgoal/stop_reason 必须且只能设置一个。声明优先，返回 target_claimed。没有前沿返回 no_frontiers；没有合格候选返回 no_eligible_frontiers。这些值请求运行器采取动作，不表示立即判定搜索耗尽。
- Random 均匀采样；Nearest 最小化 path_cost_m；Information Gain 最大化增益除以 path_cost_m 与固定正数 epsilon_m 之和。平局按稳定 ID 顺序处理。所有策略接收相同上下文结构。
- LLM 接收已观察语义、几何覆盖、增益、代价和共用恢复上下文。WorldState 中存在边界，但紧凑提示词可以省略。缺失标签或覆盖值保持未知。不提供隐藏状态、图像或评估器结果。
- 适配器验证返回 ID 并构造 Subgoal。无效或超时输出在决策截止时间内触发确定性 Nearest 回退，并记录原因。不得无限重试。

### 4.3 B：导航与观察运动

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

B 与仿真器/控制适配器绑定。运行器推进仿真；B 的控制器在这些步中按配置频率执行。Poll 读取状态，不推进仿真。同时最多允许一个导航或观察操作；重叠启动应抛出已定义的边界异常。Reset 要求先前操作已经停止，并清空回合状态。

由**运行器**而非 SearchPolicy 在规定的初始化、到达和恢复时刻调用 `start_observation`。B 转动装有固定相机的机器人；A 处理产生的观察。例如：C 选择 F3 → B 导航至 F3 → 运行器请求到达扫描 → B 执行扫描，A 同时观察 → 运行器等待新状态并刷新候选评估 → C 再次选择。目标发现声明或停止条件可以中断该序列。共用流程使初始四策略比较聚焦于前沿选择；由策略选择扫描属于后续扩展。

- start 验证 ID、地图与位姿时效性和完整轮廓有效性；返回 ACCEPTED 或立即 FAILED。已接受操作可在 EXECUTING 前失败或取消。只锁定一个终止结果；后续轮询返回该结果，不产生新的终止事件。
- B 只执行选定目标。检查路径段和旋转中的轮廓净空、未知及越界区域、运动约束与切角。到达必须同时满足位置和 yaw 容差。
- 仅接收同回合、同栅格几何的新地图。重新检查剩余路径，必要时向同一目标重规划。重规划前停止不安全运动，不得选择其他前沿。
- 规划完成后发布 PlannerResult，并保留在反馈中。成功规划之后，执行仍可 FAILED，并有独立 failure_reason。重规划完成后，反馈携带最新结果；记录每次变化。
- FAILED 必须有 FailureReason；其他状态的 failure_reason=None。规划前验证失败、取消或观察运动时，PlannerResult 可以保持 None。导航到达要求成功规划；观察操作不要求存在规划结果。
- cancel 停止运动且具有幂等性，除非已经终止，否则返回 CANCELLED。取消不算前沿失败。
- 观察流程使用固定配置和经过检查的运动。完整旋转要求能力及净空允许。不安全流程应失败，不能盲目旋转；隐式恢复也必须有界。
- 观察操作的 REACHED 表示运动与停留完成。再次选择前，运行器单独执行 A 的已处理帧同步要求。观察耗时和运动计入预算。

### 4.4 仅用于评估的评分

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

在清单中固定可见实例与类别匹配规则、像素重叠或定位容差、最小可见程度及所需证据帧数。要求指定类别出现在声明引用的观察中，而不是仅存在于场景某处。结果和仿真器 ID 只写入终止日志。验证器错误产生 evaluation_error，绝不自动视为成功。

## 5. 运行器控制流程与恢复

### 5.1 决策与运动生命周期

1. 验证清单和出生位置净空；重置 A、B、C 及评估器。观察运动前先采集初始帧，让 B 有地图可用。
2. 执行固定初始观察流程，同时推进仿真、处理 RGB-D 并检查声明与限制。不安全运动返回明确类型的失败，只允许有界恢复。
3. 在决策边界停止机器人，处理完帧同步要求内的观察，然后暂停仿真。获取状态与 B 的评估，应用恢复排除项，并以匹配修订号调用 C。
4. 验证选择。若下发前地图或位姿变化，重新评估候选。身份或接近位姿变化时刷新或重新选择。仅全局修订号增加不会使未变候选失效，但 B 必须依据当前地图重新验证路径。记录评估、选择和下发的修订号。
5. 恢复仿真并启动 B。持续更新 A、转发地图并轮询。运行器独立执行步数、仿真时间、实际时间、操作和决策限制，包括处理挂起的 LLM 调用。
6. 导航 REACHED 时保留 PlannerAttempt，围绕规定朝向执行到达观察，等待新帧处理完成。FAILED 时记录原因并恢复。CANCELLED 时记录原因，不标记前沿失败。
7. 目标发现声明优先于运动和决策。取消 B、冻结证据，并在评分后以 success 或 false_target_claim 结束。绝不把评估器结果返回 C。
8. 达到限制时取消运动，以 search_limit 结束。测量实际执行距离；规划代价不是执行距离。

**帧同步要求（Frame barrier）：**在每个配置的扫描朝向，满足朝向与停留条件后采集规定样本。即使 A 通常采用抽帧处理，也必须处理这些指定帧。运动完成后，处理一个时间戳等于或晚于终止仿真时间戳的帧。为等待处理完毕设置实际时间截止限制；感知失败或缺失属于 pipeline_error，不得算作成功观察或伪造空状态。

### 5.2 失败与无进展策略

运行器按谱系和空间支撑区域维护历史，并将排除项投影到 SelectionContext。

- NO_PATH/INVALID_GOAL 持续排除，直到相关证据变化：失败路线或连通分量中的可通行性、接近几何，或机器人可达连通分量。没有详细路线证据时，保守比较机器人可达分量与候选邻域的变化。物体标签变化或无关修订号增加不符合条件。
- BLOCKED/TIMEOUT 在仿真时间冷却期后允许重新评估，但受重试上限约束。评估超时不能变成永久 NO_PATH；评估重试也必须有界。
- 分裂或合并的后代继承重叠的排除状态和累计尝试次数。保留空间历史，防止暂时消失再出现重置上限。
- 前沿下发时固定局部观察邻域。到达观察后，统计该邻域内从未知变为已知的单元数。低于阈值表示无进展。排除该机会，直到冷却期结束或相关证据变化；达到无进展上限后，本回合持续排除。
- 相关证据可以解除几何排除，但不能重置每机会或每回合的硬性尝试上限。
- 没有合格候选时，最终判定耗尽前最多执行一次恢复观察循环。在预算内安全停止并持续观察，等待最早待结束的冷却期，然后重新评估。没有剩余重试时，以 no_frontiers 或 no_feasible_frontiers 结束。新的导航尝试可以允许下一次有界恢复循环；不发生导航的循环也受全局上限限制。

到达观察失败时进入有界恢复；没有新证据时，不得立即重新选择已经到达的位置。初始化经过有界恢复仍不能生成可用状态时，以 initialization_failed 结束。

最终原因包括：success、false_target_claim、no_frontiers、no_feasible_frontiers、search_limit、initialization_failed、pipeline_error、evaluation_error、externally_cancelled。只有 success 计为成功。基础设施错误应保留并单独报告，不得悄悄丢弃。

## 6. 实验与日志约定

- 使用配对场景、起点和目标配置，以及多组重复随机种子。隔离随机数流，防止策略额外采样影响场景生成。通过仅评估器可访问的数据，离线验证目标存在且可从可达视点观察。
- 固定检测器权重与标签、仿真器和资源、几何、建图、频率、候选与评估、扫描、恢复、预算，以及代码修订号或源码快照哈希。
- 所有策略在评估与选择期间暂停仿真。分别记录仿真时间和实际时间；LLM 延迟仍计入实际时间。限制由运行器负责。
- 记录状态及评估修订号、特征、合格 ID 与排除项、选定位姿、操作事件、数据时效、声明与证据、验证器结果、实际距离、碰撞、重规划和终止原因。
- LLM 调用记录准确模型与配置、提示词与输入、响应、延迟、可获得的用量与成本、回退原因，以及动作是否由回退策略选择。保留符号化输入以便审计；随机种子不保证托管模型可精确重放。
- 报告所有回合的成功率与错误声明率、带不确定性区间的配对比较、终止计数和效率。标明仅成功回合的距离与时间及样本数。明确报告运行错误。
- 相同候选指在给定状态下使用相同的生成、可行性和恢复规则，不要求不同轨迹中不断变化的列表完全相同。

## 7. 版本、验证与必需检查

约定 1.0 是首次实现的目标。未来字段或行为的破坏性变更需要增加主版本号；可选新增项可增加次版本号。拒绝不支持的主版本，并说明边界异常。API 失败不能伪装成零位姿、空地图或任意前沿。

在相关边界验证回合身份、栅格几何与维度及取值、有限数值与范围、序列号与修订号、深度不可变性、ID 与谱系引用，以及选择与评估的一致性。

正式实验前，实现有针对性的检查：

- 策略状态不包含隐藏占据、物体实例和标准条件下的房间标签，仅允许声明过的先验。
- 占地轮廓净空、未知区域阻挡、狭窄通道、对角切角和安全旋转。
- 固定状态下合格输入相同，使用路径代价，拒绝虚构或过期子目标。
- 分裂、合并和消失时的身份处理，证据变化及超时重试，全局上限。
- 唯一终止结果、规划前取消、规划成功后的执行失败。
- 已处理帧同步要求，以及到达后的无进展排除。
- 不同帧确认、正确与错误评分、评估器不向搜索反馈。
- LLM 无效或超时回退，以及模块停滞时的有界终止。
- 配对清单重放、实际距离统计、完整失败回合日志。

## 8. 集成前需固定的配置

将取值和单位记录在同一份经过验证的清单中。除非有明确名称的消融实验修改，所有策略共用这些配置。

| 配置 | 必须确定的内容 |
|---|---|
| 传感器与变换 | RGB 编码、深度单位与无效值、图像尺寸、内参与畸变、变换约定、量程与视场、采集与处理频率 |
| RobotConfig | 占地轮廓、底盘与相机坐标系、碰撞高度、净空、运动模型与限制、到达容差 |
| MappingConfig | 固定栅格、深度/地面/高度/证据模型、覆盖栅格化、前沿邻接与聚类、身份重叠与容差及历史窗口、接近朝向、增益射线模型；目标标签映射、置信度、至少两个不同帧、确认窗口、定位与关联容差、证据保留 |
| ObservationConfig | 初始/到达/恢复朝向序列、朝向容差、停留时间、已处理样本要求、旋转条件、操作与等待处理截止时间 |
| RecoveryConfig | 相关变化容差、冷却期、评估与导航重试上限、进展邻域与阈值、无进展上限、机会与回合硬性尝试上限 |
| PolicyConfig | 策略与种子、正数 epsilon_m、平局规则；LLM 模型与推理设置、提示词、调用及截止限制、Nearest 回退 |
| NavigationConfig | 批量规划截止时间、导航截止时间、同目标重规划上限、位姿与地图时效容差 |
| 回合 | 场景/目标/起点/种子、先验、步数/仿真时间/实际时间/决策限制、固定决策暂停约定 |
| EvaluationConfig | 可见目标匹配阈值、支持帧数、评估器截止时间、成功定义、报告与不确定性处理方式 |

团队选定机器人、场景和检测器后记录这些选择，并固定配置以保证实验可比。

模拟输入、iGibson 开发辅助函数及向真实模块输出的过渡，参见[模拟数据与独立开发](development_fixtures.md)。
