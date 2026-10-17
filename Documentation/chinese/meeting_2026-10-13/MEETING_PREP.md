# 机器人项目集成会议准备 · 2026-10-13

> **版本更新：** 本材料基于旧本地提交 a0c5250。随后已同步远端 1868da0；A 建图已完成，B 适配器已实现但真实后端尚未验收。请以 [最新 W2 状态](../w2_integration_status.md) 和本次 demo 为准。下文保留为旧快照记录，勿直接用于最新进展汇报。

**会议要产出三件事：一套共同配置、A/B 的可验收交付时间、第三周独立评分的物体搜索闭环计划。**

本材料按提供的 30 分钟会议指南编排，并核对实际代码。范围为本地 `integration/w2`，提交 `a0c52504aff9e9cf1821ba5910f7aec57cf6373f`。A/B 若已有未合入的新实现，会上以提交号、运行命令和测试证据更新下述状态；本材料不能代表他们未共享的代码。

## 可以直接照读的 C 模块汇报（约 1 分钟）

> C 这边已完成四种搜索策略、统一 EpisodeRunner、扫描帧同步、版本检查、有限恢复、预算控制和日志。今天在 Python 3.8.20 下复跑，153 项测试全部通过；mock 示例完成两次导航，实际路程约 1.80 米、零碰撞，最后因没有前沿而停止。
>
> 当前集成使用的是脚本化 A 和 B 的 mock 仿真器。统一链路里的 A 建图 update 和 B 真实 iGibson 适配器，在本地集成版本中仍未实现。此前的 iGibson 演示是独立实现，不能算这条统一链路已验收。
>
> 今天建议先确定共同机器人、场景、传感器和地图配置，再明确 A 的地图/前沿交付与 B 的采集/控制/扫描交付时间。C 随后接入真实工厂，先验收几何探索，再接 A 的多帧目标声明和独立评分，完成第三周物体搜索目标。

## 30 分钟议程

| 时间 | 主持安排 | 必须留下的结果 |
| --- | --- | --- |
| 0–5 分钟 | A/B/C 各约 90 秒，只汇报有证据的完成项、最大阻碍及新提交 | 更新三份 STATUS；区分真实实现、测试替身、未验证项 |
| 5–15 分钟 | 讨论下面三项问题，每项约 3 分钟 | 配置选择、接口交付、目标评分各有决定或调查负责人 |
| 15–23 分钟 | 确定分阶段验收，不在会上调试 | 明确几何闭环与物体搜索的通过标准 |
| 23–30 分钟 | 逐项填写 [会议记录](MINUTES_TEMPLATE.md) | 真实姓名、日期时间、交付文件、验收命令；指定共享记录维护人 |

三份简报：[A](STATUS_A.md)、[B](STATUS_B.md)、[C](STATUS_C.md)。VERIFIED 仅适用于列明的测试范围；MOCK 表示测试替身；PARTIAL 表示部分完成；MISSING 表示缺失；UNTESTED 表示本次未验证的实现。

## 最重要的三个集成问题

### 1. 统一 RGB-D 链路缺少生产 A/B 实现【已确认，P0】

- **证据：** [A 的 update](../../../src/frontier_search/perception/mapper.py#L110) 和 `get_claim_evidence()` 显式抛出 `NotImplementedError`；[B 的 IGibsonAdapter](../../../src/frontier_search/simulation/igibson_adapter.py) 各方法仍是占位。[当前工厂](../../../src/frontier_search/fixtures/w2_episode.py) 构造 `ScriptedMapper + FixtureSimulator + TeamNavigator`。
- **影响：** A→B 评估→C 选择的真实输入尚不存在；当前 mock 的地图变化由脚本产生。153 项通过不能证明生产 mapper 或真实适配器可用。
- **会上决定：** A 交付可运行的 `RGBDMapper()`；B 交付合法 RGB-D、时钟、速度控制、出生净空和扫描钩子；C 何时接真实工厂。先确认 A/B 是否已有更新提交。
- **建议责任：** A/B 分别交付各自模块，C 负责工厂接线和共同验收。验收细项沿用 [A/B 清单](../w2_ab_handoff.md)，不在会上重写接口。

### 2. 开发配置存在分歧，共同实测配置尚未冻结【分歧已确认，最终选择待定，P0】

- **证据：** [A 的采集工具](../../../src/frontier_search/simulation/capture_igibson.py) 默认 `Turtlebot / Rs_int`；[C 的旧演示](../../../src/frontier_search/simulation/igibson_rs.py) 使用 `Locobot / Rs`；[mock manifest](../../../configs/w2.mock.json) 的运动模型为 `differential_drive_placeholder`。当前无 `configs/w2.real.json` 或真实模块工厂。
- **影响：** 足迹、相机外参、出生点和地图几何不能直接共用；同名场景字符串也不能代替资产核对。
- **会上决定：** 一个机器人、一个场景及资产版本；固定图幅/原点/分辨率、相机尺寸/内外参、采样频率、速度与到达容差。共同配置写入一份真实 manifest。
- **建议责任：** B 牵头实测机器人/场景和控制；A 校验 RGB-D 投影及地图几何；C 汇总和校验 manifest。Turtlebot 可作为减少 A 改动的候选，最终选择需实测确认。

### 3. 第三周目标确认与独立评分还没有生产交付【已确认，P1】

- **证据：** A 的 `get_claim_evidence()` 未实现；[TargetEvaluator](../../../src/frontier_search/evaluation/api.py) 只有协议定义，统一链路无生产验证器；mock 配置 `enable_target_evaluation=false`。C 的 [评分路径](../../../src/frontier_search/runner/episode.py) 已实现，并以测试替身验证声明抢占、一次评分和错误声明终止。
- **影响：** `no_frontiers` 只表示搜索停止，不能计为找到目标。演示的实例分割与人工线索不能代替标准检测器、跟踪和证据链；单场景演示也不足以比较策略性能。
- **会上决定：** 目标类别与标签映射、至少两个不同采集序列的确认要求、证据格式、可见性/匹配阈值、验证器截止时间。明确几何闭环和物体搜索各自截止时间。
- **建议责任：** A 负责生产跟踪/声明/证据和隔离验证器；C 负责终止评分接线和日志；B 负责发现目标后的及时停止与取消。

## 建议第三周验收：先补几何链路，再验收物体搜索

| 阶段 | 通过标准 | 建议顺序 |
| --- | --- | --- |
| G0 配置与首帧 | 实测真实 manifest 校验通过；合法 RGB-D 首帧；出生净空、直行和正 yaw 方向正确 | 第 1 天 |
| G1 离线数据联调 | 同一多帧录制经真实 A → B 评估 → C 选择；版本匹配；同时覆盖可达/不可达候选；旧快照不变 | 第 2–3 天 |
| G2 真实几何探索 | Nearest 至少两次选择→派发→移动→到达扫描；真实新观测使已知区域扩大；前沿更新；正常场景零碰撞；不可达不派发；受阻、无进展和超时有限终止 | 第 3–5 天 |
| G3 第三周物体搜索 | 同一真实链路中，检测/定位/多帧跟踪 → TargetClaim → 停止并冻结证据 → 独立验证；有效声明为 success，错误声明为 false_target_claim，验证错误为 evaluation_error；每回合恰有一条终止记录 | 第 5–7 天 |

以上是建议顺序，会上填写实际日期时间。录制回放只能验证数据链，不能响应策略的新动作；G1 不等同 G2。若 G2 有风险，明确调整里程碑及负责人，同时保留 G3 的生产交付范围。

G3 最小场景建议：固定场景/起点/种子/目标，目标能从可达视点观察；先用 Nearest 跑通正确目标声明，再验证错误声明和预算终止。接入其他策略时共享候选生成、可达性、扫描、恢复和预算规则，LLM 回退必须留日志。批量配对实验在闭环稳定后开展。

## 共同接口速查（准确签名以当前代码为准）

| 提供方 | 接口 |
| --- | --- |
| A | `reset(request: MapperInit) -> None` |
| A | `update(observation: RGBDObservation) -> WorldState` |
| A | `get_claim_evidence(episode_id: str, claim_id: str) -> EvidenceBundle` |
| B | `reset(request: NavigatorInit) -> None` |
| B | `evaluate_candidates(request: CandidateEvaluationRequest) -> CandidateEvaluation` |
| B | `start(request: NavigationRequest) -> NavigationFeedback` |
| B | `start_observation(request: ObservationRequest) -> NavigationFeedback` |
| B | `update_map(episode_id: str, goal_id: str, occupancy_map: OccupancyGrid) -> None` |
| B | `poll(episode_id: str, goal_id: str) -> NavigationFeedback` |
| B | `cancel(episode_id: str, goal_id: str, reason: str) -> NavigationFeedback` |
| B 运行钩子 | `on_step() -> None`；`capture_marks(goal_id: str) -> Tuple[float, ...]` |
| B 仿真器 | `capture() -> RGBDObservation`；`sim_time_s() -> float`；`set_velocity(linear_mps: float, angular_radps: float) -> None`；其余生命周期见 [协议](../../../src/frontier_search/simulation/api.py) |
| C 策略 | `reset(episode_id: str, seed: int, config: PolicyConfig) -> None`；`select(state: WorldState, context: SelectionContext) -> SelectionResult` |
| C 运行器 | `run(manifest: EpisodeManifest) -> TerminationReason` |
| A 独立评分 | `reset(episode_id: str, config: EvaluationConfig) -> None`；`evaluate(request: EvaluationRequest) -> EvaluationResult` |

数据类型见 [contracts.py](../../../src/frontier_search/contracts.py)；行为遵循 [Interface Freeze 1.0](../../english/interface_freeze.md)。重点统一：

- 世界 x/y 为水平米制、z 向上、yaw 逆时针；栅格 `[row][column]`，row 随世界 +y；占据值 `-1/0/100`。A 输出未膨胀地图，B 按整车足迹判断可达。
- RGB 为 `rgb8`，深度为 `depth32f_le` 米制轴向深度、0 无效；相机轴 right/down/forward；使用采集时的 camera-to-world。
- `WorldState` 含回合、revision、时间、已处理帧序号、位姿、地图/覆盖、物体、区域、前沿和可选声明；嵌套集合使用 tuple，不可变。
- B 评估关联 `state_revision/start_pose`，每候选一条 assessment；`reachable=None` 的超时保持不确定。C 只选共同 eligible 集合；Nearest 使用 B 的路径代价。
- runner 推进仿真并在每步调用 B 的 `on_step()`；poll 不推进仿真。扫描完成后指定帧必须经 A 处理，才能继续选择。验证器仅供终止评分，结果不返回策略。

## 今天实际验证的证据

- Python **3.8.20**，全套 **153 项测试通过，0 失败、0 跳过**，运行耗时 **47.471 秒**。
- 再运行统一 mock progress：**2 次导航派发、4 次决策、1.8049 m、0 碰撞**，终态 **no_frontiers**，489 步。原始 [JSONL 日志](../../../runs/meeting_2026-10-13/progress.jsonl) 和 [验证摘要](verification.json) 已保存。
- 测试包含 A 的 reset/RGB-D/投影/障碍工具，B 的足迹/规划/执行/扫描，3 项跨模块接缝，以及 C 的恢复/扫描同步/挂起监督/终止评分。生产 A update 和 B adapter 的缺失不因测试通过而消失。
- 本次没有运行付费 LLM 请求或重新跑 YOLO 权重推理。上游依赖有弃用和资源提示，测试无失败。

复核命令（项目目录 `D:\CS5478_PROJECT\integration`）：

```powershell
$env:PYTHONPATH = Join-Path $PWD 'src'
& .\data\venv-igibson\Scripts\python.exe -m unittest discover -s tests -v
& .\data\venv-igibson\Scripts\python.exe -m frontier_search.runner.w2 --manifest configs/w2.mock.json --log runs/meeting-progress-NEW.jsonl
```

每次使用新的日志文件名。会后把填写完的同一份会议记录同步给三位组员及其 Agent；后续开发以会上确认的决策为准。
