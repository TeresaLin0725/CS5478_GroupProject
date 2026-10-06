# C 的统一 W2 runner

## 已实现

本次在 Python 3.8.20 下全套 **153 项测试通过，无跳过**；包含新增加的 31 项运行/恢复回归。
四个 CLI 场景的实测汇总与源代码指纹见 [verification.json](../w2_runner/verification.json)。

`runner/episode.py` 现在实现 `EpisodeRunner.run(EpisodeManifest)`：

1. 校验配置，reset 模拟器，验证出生净空，reset A/B/C；先采集再转动。
2. 执行 B 初始扫描；每步由 runner 推进模拟器，再调用 `navigator.on_step()`。
3. 扫描 mark 当步采集所需样本，核对 heading/时间/序号；操作终态后再处理一帧。
4. 停车、采集、暂停模拟器；同版本评估、恢复过滤、策略选择；派发前核对位姿、yaw 和地图新鲜度，过期则重新决策。
5. 移动期间持续采集和更新地图；记录实际位姿距离与反馈；到达后扫描，比较派发时固定邻域内 unknown→known 面积。
6. 有限失败重试、冷却、无进展抑制；无候选时至多一次恢复扫描再决定退出。保留空间/谱系历史，分裂/合并/重命名不会清空硬次数上限。
7. 检查步数、决策数、尝试数、仿真时间、墙钟和操作期限；终止时停止、取消、关闭；输出一个终态。

`runner/supervisor.py` 在单独进程运行工厂和 episode。父进程保存日志，监督模块和总墙钟期限；
挂起模块被终止后仍有失败日志和最后已收到的部分指标。
**需要硬超时保证时使用下面的 CLI / `run_supervised()`；直接 `EpisodeRunner.run()` 只能在同步调用返回后检查超时。**
监督器只适用于本项目进程内模拟器，未实现物理机器人独立电机 watchdog。

## 立即运行

仓库根目录；已有 Python 3.8 环境可直接使用。运行不需要 iGibson、YOLO 或网络依赖。

```powershell
$env:PYTHONPATH = "$PWD/src"
$pythonPath = './data/venv-igibson/Scripts/python.exe'
& $pythonPath -m frontier_search.runner.w2 --manifest configs/w2.mock.json --log runs/w2-progress-001.jsonl
& $pythonPath -m frontier_search.runner.w2 --scenario blocked --log runs/w2-blocked-001.jsonl
& $pythonPath -m frontier_search.runner.w2 --scenario no_progress --log runs/w2-no-progress-001.jsonl
& $pythonPath -m frontier_search.runner.w2 --scenario no_frontiers --log runs/w2-empty-001.jsonl
& $pythonPath -m unittest discover -s tests -p 'test_w2_*.py' -v
```

日志文件使用新名字，不覆盖旧记录。没有本机专用环境时可使用另一个 Python 3.8+ 解释器。
CLI 对 pipeline/initialization/evaluation error 返回非零；no_frontiers/no_feasible_frontiers/search_limit
是受控停止，可返回零，验收仍须检查 JSONL 的 termination 和指标。

示例预期：

| 场景 | 预期终态 | 关键断言 |
| --- | --- | --- |
| progress | no_frontiers | 两次导航派发、约 1.80 m 实际位移、零碰撞、新已知面积大于零 |
| blocked | no_feasible_frontiers | 计划成功但隐藏障碍导致执行 BLOCKED；两次失败后停止 |
| no_progress | no_feasible_frontiers | 到达却没有新已知面积，两次无进展后停止 |
| no_frontiers | no_frontiers | 不派发导航，初始扫描后只做一次恢复扫描 |

blocked 是故意让真实 mock 障碍与输入地图不同的负例；其碰撞计数是**发生接触的仿真步数**，
不是独立碰撞事件数，不能把它与正常场景的安全表现混为一谈。

四种策略均可用 `--policy` 指定。`llm_guided` 默认没有网络客户端，会明确记录 unconfigured_client
并回退 Nearest；本次没有调用付费模型。真实客户端可由自定义工厂注入已有 LLMGuidedPolicy。

## 新文件与接口

| 文件 | 用途 |
| --- | --- |
| `runner/manifest.py` | 可执行配置、严格 JSON 字段解析、参数检查；不是原 experiment.template.json 草案 |
| `runner/episode.py` | 统一执行流程和同步帧屏障 |
| `runner/recovery.py` | SpatialRecovery 与历史计数 |
| `runner/supervisor.py` | 可终止子进程和父进程日志 |
| `runner/w2.py` | CLI 与工厂入口 |
| `fixtures/w2_episode.py` | 单独的 A 脚本化测试替身、合法一像素 RGB-D 包装、mock 工厂 |
| `configs/w2.mock.json` | 可复现开发配置；机器人参数仍为 B 的占位值 |
| `tests/test_w2_runner.py`, `tests/test_w2_recovery.py` | 流程、预算、错误、挂起、恢复与谱系回归 |

Navigator/Simulator 的基础协议未改字段；新增 `SteppedNavigator` 和 `ControlledSimulatorAdapter`
明确此前 B 已实现的 on_step/capture_marks/sim_time_s/set_velocity 扩展。
A 的同步 update 必须返回刚输入那帧的 state；如果以后异步处理，需要额外同步适配层。
一个扫描 mark 可在机器人不动、仿真时间不变时采集多个样本，但 capture sequence 必须不同。

## 日志与可复现性

父进程记录 manifest、工厂、Python 版本和项目 Python 源文件 SHA-256。
日志包括输入 state、B 评估、eligible/exclusions、选择、派发版本、反馈/新规划、扫描样本、
局部进展、逐步实际位姿/距离/碰撞与终态；LLM 记录沿用现有策略接口。
不会把 evaluator 或隐藏实例 ID 传给 policy。
发生硬中断时 `partial_metrics=true`，数值是父进程最后收到的记录，不能当作完整运行统计。

## 当前限制和接入条件

- 这是 **C 的基本运行流程已实现**，不是完整真实 ABC 已完成。A 的 update 和 B 的真实 adapter 仍待交付。
- fixture 的空闲区域/前沿按位姿阶段人工揭示，RGB-D 像素不参与建图；它验证调度，不证明感知性能。
- 恢复以一格空间邻域匹配历史，用已知空闲连通区域和候选邻域判断几何变化；实际放行仍由 B 足迹规划决定。这是保守规则，复杂真实场景的性能和过度排除需联调验证。
- W2 默认禁用目标声明，不能产生物体搜索 success。可选声明/评分接口已预留并有真假/异常测试，但真实追踪与验证仍由 A 在 W3 实现。
- 真机型/场景参数、真实 RGB-D 回放和真实连续运动尚未验收；不得把这里的脚本化地图指标写成真实实验结果。

下一步交付和每项验收标准见 [A/B 清单](w2_ab_handoff.md)。
