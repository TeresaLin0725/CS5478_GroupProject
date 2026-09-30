# 模拟数据与独立开发

机器人项目的开发指南。[接口约定](interface_freeze.md)定义了这些测试数据使用的 1.0 版契约。

在 A 的建图器准备好之前，B 和 C 可以使用预设测试数据（fixtures）开发。测试数据必须与真实模块输出使用相同的数据约定，包括回合 ID、修订号、坐标、占据值和不可变快照。模拟值是人工编写的输入，不必由感知或规划算法生成。

## 1. B：合成栅格与导航请求

B 需要栅格、机器人配置、起始位姿，以及候选接近位姿或选定子目标。先使用小型、确定性的栅格，便于检查预期路线及失败情况。例如：

```python
import numpy as np

cells = np.full((40, 40), -1, dtype=np.int16)  # unknown
cells[5:35, 5:35] = 0                         # known free
cells[5:35, 20] = 100                         # wall
cells[18:23, 20] = 0                          # doorway

mock_grid = OccupancyGrid(
    frame_id="world",
    revision=1,
    resolution_m=0.1,
    origin_x_m=0.0,
    origin_y_m=0.0,
    width_cells=40,
    height_cells=40,
    data=tuple(tuple(int(value) for value in row) for row in cells),
)
```

该测试栅格为 4 m × 4 m，门口宽五个单元（0.5 m）。机器人是否能通过取决于占地轮廓与净空裕量。按照[共用约定](interface_freeze.md#2-共用约定与先验)转换单元中心；行列索引不是世界坐标。测试栅格保持未膨胀状态，让 B 应用自己的代价地图规则。

仅测试规划器时，将起点和目标单元放在墙的两侧，可检查绕行。完整前沿集成测试还应提供一致的前沿支撑单元、接近朝向、区域和已探索覆盖情况。将它们封装为 WorldState 供 evaluate_candidates 使用，或提供 NavigationRequest 执行选定目标。纯几何可以不运行 iGibson 进行测试；物理路径跟随需要仿真器或控制适配器替代实现。

必需用例包括开阔空间、绕墙、对所选机器人过窄的门口、被未知空间隔开的路线，以及阻断正在执行路线的新地图。相关 API 实现后，再加入观察运动与取消用例。

## 2. C：人工编写的 WorldState 与模拟 B 评估

C 可以从人工编写的 WorldState 和 SelectionContext 记录开始。包含目标、机器人位姿、一致地图、区域与物体摘要、前沿位姿与增益，以及回合、修订号和起始位姿匹配的 CandidateEvaluation。直接模拟 B 的路径代价和可达性即可；C 不需要实现规划器或前沿提取器来生成输入。

目标为**杯子（cup）**的选择测试示例：

| 前沿 | 所属区域中已观察物体 | 模拟 path_cost_m | information_gain_m2 | reachable |
|---|---|---:|---:|---|
| F1 | 沙发、电视 | 2.0 | 6.0 | True |
| F2 | 水槽、冰箱 | 5.0 | 20.0 | True |
| F3 | 床 | None | 12.0 | False |

将 F3 设置为 failure_reason=NO_PATH，成功评估的失败原因保持 None。没有其他排除项时，eligible_frontier_ids 包含 F1 和 F2。Nearest 选择 F1；对任意正数 epsilon_m，Information Gain 都选择 F2；Random 只能选择合格 ID。LLM 可能偏好 F2，但不强制如此：应将资格检查与响应处理测试，同语义偏好测试分开。

还应提供无前沿、所有候选被排除、终止搜索的目标声明、无法确定的评估，以及先前导航失败等测试数据。用 LLM 响应桩确定性地测试格式错误、超出集合和超时情况，集成时再单独进行真实调用。在完整运行器测试中，单独模拟评估器；目标声明绝不自动算作成功。

## 3. 从 iGibson 生成开发测试数据

A 开发期间，iGibson 可以提供真实感更强的几何与参考输出。以下辅助函数存在于上游源码中；编写适配器前应固定安装版本，并核实可用性和约定。

| 功能 | 开发用途 | 适配器注意事项 |
|---|---|---|
| scene.floor_map[floor] | 加载场景可通行性地图作为规划器测试数据 | 这是场景生成的可通行性，不是 A 累积的占据状态；记录是否已进行腐蚀和连通性筛选 |
| scene.get_random_point(floor=...) | 采样开发用起点与目标位置 | 保存坐标和种子；执行前验证真实机器人净空 |
| scene.world_to_map(...) 和 scene.map_to_world(...) | 转换场景地图与仿真器坐标 | 将轴顺序、原点、分辨率和单元中心约定适配到本项目接口 |
| scene.get_shortest_path(...) | 提供调试用参考路径与距离 | 使用场景图；仅在几何和运动假设匹配时比较，不能视为绝对可靠的无碰撞判据 |

[IndoorScene 实现](https://github.com/StanfordVL/iGibson/blob/master/igibson/scenes/indoor_scene.py)加载可通行性地图，可能腐蚀空闲区域并仅保留最大连通分量，也提供以上辅助函数。其最短路径流程可能将图外端点连接到附近图节点。因此，不要把输出当作机器人能够执行路线的证明；对已腐蚀的测试数据，若未考虑该预处理，也不要再次膨胀障碍物。

对于二值可通行性测试数据，明确将可通行单元转换为 0、不可通行单元转换为 100，并说明“不可通行”不一定表示传感器观测到了实体障碍物。构造部分已知测试数据时，应用单独定义的观察掩码，将所有尚未揭示单元设为 -1。不要意外把完整地图中每个单元都标记为已探索。人工选取的掩码适合开发，但不是 RGB-D 观察模型。

iGibson 还通过 ScanSensor 流程提供 occupancy_grid 观察，见[环境实现](https://github.com/StanfordVL/iGibson/blob/master/igibson/envs/igibson_env.py)。它与 scene.floor_map 以及计划中的 RGB-D 建图器均不同。适配前检查传感器假设、数值编码、坐标系、分辨率和更新行为；它不能直接替代 WorldState 栅格。若在实验中用扫描建图替代 RGB-D 建图，需要明确变更所有策略共用的传感器设计。

## 4. 测试数据归属与集成过渡

- B 负责合成导航栅格和场景到测试数据的转换；C 负责策略状态测试数据及模拟候选评估。A 审查字段是否符合建图器约定。
- 通过统一加载器存储和读取测试数据，附带 fixture_id、source（synthetic、simulator_oracle 或 recorded_robot_state）、schema_version、seed 和预处理等元数据。这些是测试数据元数据，不是额外策略特征。
- 开发测试数据及结果应与正式实验清单分开。开发和调试可以使用完整场景地图及内置路径；标准实验使用 A 的机器人自建地图和 B 的团队自研规划器，只允许[已声明的先验](interface_freeze.md#2-共用约定与先验)。
- 将测试数据提供器替换成 A 的输出与 B 的真实评估，无需修改 C 的 select 接口。集成开始后，已记录的机器人状态快照可用于重放实际失败。

```text
阶段 1：合成栅格 / 人工状态与评估 → B 和 C 独立开发
阶段 2：适配后的 iGibson 场景测试数据 → 更真实的几何与仿真器集成
阶段 3：A 的机器人自建 WorldState → B 的评估 → C 的选择 → B 的执行
```
