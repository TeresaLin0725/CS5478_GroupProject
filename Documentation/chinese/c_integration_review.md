# C 模块集成审查与单地图验证

基线：`feat/frontier-search-scaffold`，提交 `811580f1fe4d54eab7d42c5b788155376c495a8f`。
集成分支：`feat/c-search-gemini-integration`。审查日期：2026-09-30。

## 审查结论

旧原型满足早期 Google Docs 中“使用 mock WorldState 独立开发”的目标，但不能直接作为最新 Interface Freeze 1.0 的实现。以组员框架为主迁移功能，不再在新仓库保留第二套冲突的数据类和策略接口。原本工作区的独立原型保留作历史参考。

| 项目 | 旧原型的问题或差异 | 集成处理 |
|---|---|---|
| Nearest 定义 | 用欧氏距离；遇到墙和绕路时可能选错标准基线 | 复用 `BaselinePolicy`，仅在共用 eligible 集合中最小化 B 的 `path_cost_m` |
| 选择前可达性 | 先选点再规划，没有统一候选评估 | mock B 先生成完整 CandidateEvaluation；`build_context` 与 `decide` 验证修订号、身份、位姿和候选 |
| 数据与坐标 | 可变列表；y 向下、整数坐标当中心；占据值 1 | 使用 tuple/frozen records；右手世界系、格中心 +0.5、占据值 100；yaw 朝未知区 |
| 高层输出 | 只有位置/ID，没有源修订号和策略标识 | 使用原框架 SelectionResult/Subgoal，姿态来自 A，不允许模型编造坐标 |
| 目标成功 | `target_found` 直接作为成功 | mock A 产生有两个不同采集序列的 TargetClaim/EvidenceBundle，独立 fixture evaluator 决定 success/false_target_claim |
| LLM 出错 | 终止 policy_error | 复用原框架的 Nearest fallback、调用上限和日志；集成 smoke 检查遇到 fallback 会返回非零退出码 |
| 超时 | urllib 的 I/O timeout 不能单独保证整个请求截止时间 | 在独立子进程中执行 HTTP；父进程限制总等待时间，超时终止并回收子进程 |
| 可审计性 | 记录决策但缺少 token 用量 | 保留原始响应、确切消息、请求/响应模型、reasoning 设置、用量、延迟、fallback；未知 cost 保持 None |
| 兼容性 | Python 3.10 联合类型和内置泛型 | 新增代码采用 Python 3.8 兼容语法；当前机器仅用 Python 3.10 执行测试 |
| 实验效力 | 仅一张地图，两条路线长度相同 | 保留为接口/语义选择 case，不据此宣称 LLM 搜索效率更优 |

旧原型“失败 frontier 整回合永久屏蔽”也不符合新版恢复约定；没有将这套逻辑迁入生产接口。新版要求按失败类型、局部证据、冷却、谱系和空间重叠管理重试，仍待 `RecoveryManager` 实现。

## 集成位置

- `policies/provider.py`：Gemini OpenAI-compatible 客户端、本地配置加载、JSON/代码围栏解析、显式回放客户端。
- `policies/transport.py`：只负责单次 HTTP 请求的隔离进程。凭据走 stdin，不进入命令参数或日志。HTTP 错误只记录状态码/错误类型。
- `policies/llm.py`：原来的策略与 fallback 保留，补充客户端原始记录、理由及 usage。
- `fixtures/three_rooms.py`：新版三房间 WorldState、点机器人网格 mock B、脚本化 mock A、隔离的 mock evaluator。
- `runner/synthetic.py`：单地图多步集成 case。调用已有 `decide`，保存 fixture JSONL，不替代生产 `EpisodeRunner`。
- `tests/test_c_integration.py`：新增针对迁移行为的回归测试；保留组员原有测试。

## 新地图

下图上方为较大 y，内部按 `[row][column]` 存储且 row 随 y 增加。每格 1 m。

```text
#############  row 4
#...#...#...#  row 3
#...#...#...#  row 2
#S1...2...3.#  row 1
#############  row 0
```

S=(1.5,1.5)，F1=(2.5,1.5)，F2=(6.5,1.5)，F3=(10.5,1.5)，frontier yaw=π/2。
初始房间内部未知；走廊和墙体是明确标注的手工观测数据，不是从隐藏仿真地图获取。
区域物体线索分别为 sofa/television、bed/wardrobe、sink/refrigerator；room_type 均为 None。
脚本到达扫描揭示对应房间；F3 的脚本提供 mug 的两帧检测。策略不能访问脚本、隐藏评分条件或评估结果。

这里的 region boundary_source 使用契约要求的 `igibson_room_prior`，但所有几何是 authored fixture，**不是实际从 iGibson 导出**；外层事件明确标注 source=synthetic。

## 已验证结果

2026-09-30，模型请求名 `gemini-3.8-flash`：

| 模式 | 决策顺序 | 决策数 | 模拟执行距离 | fallback | 终止 |
|---|---|---:|---:|---:|---|
| nearest | F1 → F2 → F3 | 3 | 9 m | 0 | success |
| llm_guided / replay | F3 | 1 | 9 m | 0 | success |
| llm_guided / live | F3 | 1 | 9 m | 0 | success |

真实模型一次调用约 2.52 s，服务返回的 total_tokens=1373；完整 usage 对象原样保存，费用未估算。此数据只是一次集成测试，不是统计结论。22 项测试包括原框架的 13 项和新增的 9 项。

本地完整记录：`runs/c-integration-replay.jsonl`、`runs/c-integration-live.jsonl`（runs 被忽略，不会自动提交）。可分享的精简运行证据见 `Documentation/c_integration_results.json`。

## 复现

在仓库根目录、激活项目 Python 环境后：

```powershell
$env:PYTHONPATH = "$PWD/src"
python -m unittest discover -s tests -v
python -m frontier_search.runner.synthetic --log runs/replay-new.jsonl
python -m frontier_search.runner.synthetic --backend live --env-file ../.env --log runs/live-new.jsonl
```

当前工作区 checkout 位于 `D:\CS5478_PROJECT\integration`，可用 `py -3.10` 替换 `python`。其他组员应复制 `.env.example` 到自己的 `.env` 并填写密钥，再将 `--env-file` 指向该文件。日志以独占创建模式打开，请使用新文件名。

## 仍不能声称完成的内容

原框架的 `EpisodeRunner.run()` 明确抛 NotImplementedError；RecoveryManager 仍是接口。这是其公开的 scaffold 状态，不是此次迁移新引入的缺陷。

本次 synthetic loop 不提供真实 RGB-D、iGibson、机器人 footprint/转弯验证、连续感知、固定扫描动作及 frame barrier、运动中 claim 抢占、stale dispatch 重评估、谱系/空间恢复、全模块挂起监督或正式实验 manifest。mock B 是点机器人四邻接 BFS，不能当作组员 B 的真实规划器；mock evaluator 只做脚本评分，不实现可见性真值匹配。

后续 C 的重点是按 Interface Freeze 第 5 节实现生产 Runner 和 RecoveryManager，再与 A/B 接入；不能仅凭这张 case 将完整 C 工作标成完成。
