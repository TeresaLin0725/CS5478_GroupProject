# W2 集成现状与执行计划

核对日期：2026-10-07。分支：`integration/w2`。

## 本次集成

保留三个分工分支，以 C 分支为基础，通过 merge 保留 A/B 的提交历史：

| 分工 | 远端分支 | 本次纳入的提交 |
| --- | --- | --- |
| A | `a-perception` | `ab52cf4` |
| B | `feat/b-navigation` | `224bfcf` |
| C | `feat/c-search-gemini-integration` | `8f817f8` |

解决 README 的内容冲突，保留三方实现与开发说明。共享契约继续采用仓库
`Documentation/english/interface_freeze.md` 的 1.0 版本。
已读取 [Google 分工文档](https://docs.google.com/document/d/166bdSuJ9GpVUPGL-ijPUvzIAremgalJn3605ZVkWvaY/edit)：
其中是较早的总体分工与接口草案；当前代码已有更细的候选评估、执行反馈和版本校验，
集成不退回草案中的简化字段。

**已完成代码合并、C 的统一基本流程及 mock 闭环；完整 A/B/C 实时 RGB-D 闭环仍待 A/B 交付。**

新增：[C runner 使用说明](w2_runner.md)、[给 A/B 的任务清单与逐项验收标准](w2_ab_handoff.md)。
你现在可以运行统一 runner，并以这份清单协调共同配置和交付时间；不用等 A/B 才开发 C 的执行流程。

## 当前可用与缺口

| 模块 | 已实现 | W2 尚需完成 |
| --- | --- | --- |
| A | 原子 reset、出生净空种子、区域先验、RGB-D 校验/录制/回放/投影、障碍端点标记、检测器可行性工具 | `perception.mapper.RGBDMapper.update()` 仍抛 `NotImplementedError`；空闲空间融合、覆盖率、前沿/接近位姿、ID/谱系、信息增益、不可变状态发布 |
| B | 足迹代价地图、A* 与距离场、候选评估、差速控制、观察转动、同目标重规划、取消与超时、MockSimulator | `simulation.igibson_adapter.IGibsonAdapter` 仍是占位；真实速度控制/时钟/捕获、出生净空和机器人参数校验 |
| C | 四种策略、统一 EpisodeRunner、同步扫描帧屏障、版本/位姿校验、SpatialRecovery、可执行 manifest、预算/进程监督、JSONL 日志和 mock 回归 | A/B 真模块工厂接入、真实配置冻结、录制数据与实时闭环验收；真实场景的恢复规则调优 |

C 的 `simulation/igibson_rs.py` 内也有名为 RGBDMapper 的演示实现，不能与 A 的
`perception/mapper.py` 混淆。演示成功不代表统一 runner 已接上 A/B。
演示中的分割/人工地标语义也不能计作标准 YOLO 感知结果。

## 验证

使用本机 `data/venv-igibson/Scripts/python.exe`（Python 3.8）验证：

- 合并后的原有 **119 项测试全部通过，无跳过**，包含 iGibson、PyBullet、A 的 RGB-D 工具、B 的导航以及 C 的策略测试。
- 新增 `tests/test_w2_integration.py` 的 **3 项跨模块测试通过**：B 真实评估 → C Nearest → B mock 执行；A 障碍更新 → B 可达性变化 → C 改选并拒绝旧评估；B 拒绝缺少完整足迹净空的旧 C 人工候选。
- 首次合并为 122 项测试；本次 C 增加 31 项运行/恢复回归，**全套 153 项通过、无跳过**。人工地图/占位机器人不代替录制 RGB-D 的完整 ABC 验收。
- 测试环境出现上游 iGibson/Gym 的弃用与资源提示，但测试无失败。没有重新调用付费 LLM，也没有重新验证 A 的 YOLO 权重推理。

C 的 progress mock 已连续完成 2 次导航、约 1.80 m 实际距离、零碰撞，终态为 `no_frontiers`；
blocked/no_progress 负例在有限次数后以 `no_feasible_frontiers` 停止。此处地图由测试替身脚本化生成，不是 A 的真实建图。
四个监督运行场景的实测结果见 [verification.json](../w2_runner/verification.json)。

运行全套（仓库根目录）：

```powershell
$env:PYTHONPATH = "$PWD/src"
& ./data/venv-igibson/Scripts/python.exe -m unittest discover -s tests -v
```

仅检查跨模块连接：

```powershell
$env:PYTHONPATH = "$PWD/src"
python -m unittest discover -s tests -p test_w2_integration.py -v
```

## 对组员建议的调整

总体节奏可执行，建议将验收拆成“合并通过、离线数据通过、实时闭环通过”三道门。

1. **共同基础已合并。** 后续各角色向 `integration/w2` 提交小 PR，运行全套测试；定期把集成分支 merge 回自己的分支，不改写其他同学已共享的历史。
2. **先冻结共同配置。** Python 3.8 已在本机验证。机器人和场景仍应明确选定后再冻结。A 的采集工具用 Turtlebot / `Rs_int`，C 用 Locobot / `Rs`，B 用约 0.18 m 半径的占位机器人；不能直接混用。建议优先评估 Turtlebot，减少 A 采集链的变动；场景优先选择三人均有同版本资产的一种。`Rs` 与 `Rs_int` 是不同场景配置，不能靠改字符串互换。最终足迹、碰撞高度、相机外参、出生点、图幅和分辨率由实测写入 manifest；通过直行/转向/净空/RGB-D 投影检查后才标记冻结。
3. **将演示的复用边界写清。** A 独占生产 mapper、frontier 与区域覆盖；B 独占模拟器适配、控制、规划和代价地图；C 维护 runner、策略、恢复和日志。C 演示可以给 A/B 参考或迁移，但统一 runner 不并行维护第二套 mapper/导航。
4. **周中离线验证分两层。** 先用固定 RGB-D 录制序列检查 A 输出 → B 评估 → C 选择；录制序列必须含序号、时间、位姿、内外参和共享地图配置。再单独验证派发与 B 执行。录制轨迹不能响应新策略动作，不能当作交互闭环或策略效率实验。
5. **周末用 Nearest 做几何探索闭环。** 首先要求地图随真实观测扩展、前沿更新、只选择 B 判定可达的候选、移动和观察后重新决策。至少完成两次选择/派发周期，并覆盖不可达候选、运动中受阻、无新增覆盖和超时终止。记录实际距离、碰撞、覆盖增量和停止原因；不以“目标检测成功”作为 W2 验收。

## 已明确的接口约定与易漏点

- 沿用 A 的 `RGBD_SENSOR_CONVENTIONS`：`rgb8`、`depth32f_le`、米制轴向深度、0 无效；相机轴 right/down/forward；像素中心为整数坐标；capture-time camera-to-world。图像尺寸/内参/FOV/频率仍需冻结，不能只统一 Python 版本。
- 世界 x/y 为水平米制，z 向上，yaw 逆时针；地图 `[row][column]`，行随世界 +y，值 `-1/0/100`。A 输出不膨胀地图，B 膨胀并将未知视作不可通行。
- 已知空闲的单个接近点不代表整车可达。现有 C selection fixture 的人工 reachable 标签在 B 足迹检查下不成立；新测试明确验证由 B 重新评估，不放宽碰撞约束迁就 fixture。
- B 的 `MockSimulator.capture()` 返回占位图像，不能直接输入 A 的 RGB-D 校验器。离线联调使用 A 的 recording 工具，真实运行使用 B 的适配器。
- `TeamNavigator` 除公共 Navigator 协议外，还需要每个仿真步后的 `on_step()`，以及 `capture_marks()` 的扫描采样时刻。C/B 必须显式接好这两个扩展；只调用 `poll()` 不会驱动机器人。指定扫描帧要由 A 确认处理，转动完成不等于观察完成。
- 推荐执行顺序：验证出生净空 → reset → 初始采集/建图 → B 初始观察并持续 A 更新 → 停车/帧屏障 → 同版本 B 评估 → C 选择 → 派发前重新核对位姿与地图 → B 移动、A 更新、B 接收新图 → 到达扫描/帧屏障 → 再评估。
- 恢复不能只记 frontier ID：ID 拆分/合并/暂时消失仍需继承次数。TIMEOUT 保持“不确定”，按冷却和预算重试；NO_PATH/INVALID_GOAL 需相关几何变化才能解禁。一次无候选不能直接等同探索完成。
- 完整物体跟踪、目标声明和独立验证可留 W3；W2 保持 `target_claim=None`，不得用模拟器物体真值补齐感知或宣称物体搜索 success。契约当前允许已知房间边界先验，W2 无需另起完整房间分割项目。

## 下一步交付顺序

| 优先级 | 负责人 | 可验收结果 |
| --- | --- | --- |
| P0 | B 牵头，A/C 接入 | 同一机器人/场景/地图与传感器配置；可校验的捕获和出生净空 |
| P0 | A | 多帧 RGB-D 经 `update()` 产生递增版本、不可变地图/覆盖/前沿；旧快照不变 |
| P0 | C | 基本 runner/on_step/帧屏障/版本校验已完成；待 A/B 交付后替换构造工厂并进行真实验收 |
| P1 | A/B/C | 固定录制序列的 ABC 回归；可达和不可达候选均有断言 |
| P1 | C 牵头，B 配合 | mock 已有 Nearest 闭环、有限恢复、实际距离和失败日志；接真实 adapter 后验证同一流程 |
| W3 | A 牵头，C 接入 | 对象跟踪、证据/声明、隔离验证，随后开展语义策略比较 |
