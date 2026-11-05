# C 侧优化交接（2026-11-05）

本轮没有启动完整原生实验。代码基于 `integration/w2` 的 `09159c6`，完成后通过
`604 passed, 4 skipped, 5579 subtests passed`；跳过项保持为环境相关可选项。

## 1. LLM 的职责与调用门控

- 目标接近点仍由安全合同优先处理；只有一个可达候选时直接选择，不调用 hosted model。
- 对候选、代价、语义对象、区域覆盖、排除和执行反馈完全相同的上下文复用上次有效选择；
  revision 或时间戳单独变化不会制造重复调用。
- 提示只包含高层需要的摘要，不发送 occupancy grid、独立 evaluator 或完整规划路径。
  `previous_attempt` 只保留终态、失败原因、路径代价和 waypoint 数量。
- 模型看到 B 的逐候选可达性/失败原因、A 的覆盖进度、候选信息增益和最近执行结果。
- 输出继续要求主 `frontier_id` 和理由；可选提供候选排序与置信度，非法或不可选 ID
  仍回退到 Nearest，不能绕过共享 eligible 集合。
- 报告分开记录模型选择尝试 `llm_attempts` 与真实请求 `llm_calls`，call-cap、未配置客户端和确定性旁路
  不再伪装成 hosted-model 请求。

相关配置位于 `PolicyConfig`：`llm_bypass_single_candidate`、
`llm_cache_identical_context`、`llm_feedback_window`、`llm_max_observed_objects`。

## 2. A/B 与 C 的双向反馈

每次实际派发前记录 `strategy_directive`：C 的原选择、恢复机制后的最终目标、候选排序、
理由及 eligible 集合。B 仍只执行合同内的已评估 `Subgoal`，C 不接管路径规划。

每次导航终态后记录并交给策略 `PolicyFeedback`：

- B → C：到达/失败/取消、失败原因和规划路径代价；
- A → C：到达扫描后的新增已知面积和完成 revision；
- runner → C：实际 action source，区分 policy、epsilon retry 和 stuck retry。

LLM 只保留配置允许的最近若干条反馈。Baseline 接受同一接口但保持无状态，确保四策略
仍共享 runner 与导航边界。

## 3. 录制配置绑定

dry-run summary 和 validation certificate 现在保存规范化的 `runtime_configuration`，包含：

- 完整 C recovery 配置，包括 epsilon 和 stuck retry；
- B `effective_configuration()` 返回的真实 strict/relaxed costmap、机器人几何和控制参数；
- policy 配置、profile 与 policy 名称。

录制入口在构造 detector、模拟器或 recorder 前比较该配置。旧证书缺字段，或任一开关、
relaxed 参数、模型配置不同，都会拒绝录制。

## 4. 运行设施

benchmark watchdog 的共享内存名缩短为 27 字符，保留 48-bit 路径摘要，满足 macOS
31 字符限制，同时保持父子进程稳定推导。跨进程、写入完整性和非阻塞日志测试均通过。

## 尚未验证

- 尚未运行原生 U0/U05 或完整 36-job paired suite；不能据本轮单元回归宣称成功率提高。
- LLM 新提示、上下文缓存命中率、token/延迟和策略质量需要在冻结配置的小规模 smoke 后量化。
- 首轮原生测试应先选择旧失败与成功控制各少量案例，确认反馈事件、真实调用数和录制 gate，
  再启动完整实验。
