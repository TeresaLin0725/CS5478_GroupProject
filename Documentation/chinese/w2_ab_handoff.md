# 发给 A/B 的 W2 交付清单与验收标准

2026-10-07；共同分支 `integration/w2`。请先把集成分支合入自己的分工分支，再以小 PR 交付。

## 共同目标与当前边界

W2 验收“几何探索闭环”：RGB-D → A 地图/前沿 → B 可达性 → C Nearest → B 移动/扫描 → A 更新。
地图扩展指已知区域扩大；**单个 episode 内网格原点、分辨率和图幅固定**。
物体跟踪、目标声明与独立验证留 W3；W2 的 `target_claim` 为 `None`。

C 已提供统一 `EpisodeRunner`、`SpatialRecovery`、执行配置、JSONL 日志、进程级超时监督和故障回归。
当前示例使用 **C 的脚本化 A 替身 + B 的 TeamNavigator/MockSimulator**，不代表 A 的建图或真实适配器已完成。
统一入口不依赖 C 旧 iGibson 演示中的 mapper/规划器。

## P0：第一天必须共同确定的输入

| 事项 | 负责人 | 交付与通过标准 |
| --- | --- | --- |
| 机器人、场景及资产版本 | B 牵头，A/C 确认 | 一种共同配置，三人可加载同版本资产；记录型号/场景/版本/安装方法。A 的 Turtlebot/Rs_int 与 C 的 Locobot/Rs 不直接混用 |
| 机器人参数 | B | 实际足迹多边形、碰撞高度（相对 base）、安全边距、差速速度上限、位置/yaw 容差；在模拟器验证，不能沿用 `differential_drive_placeholder` 当实测参数 |
| 传感器和地图 | A/B 提供，C 汇总 | 固定图幅、原点、分辨率、相机尺寸/内参/FOV/外参和采样频率；坐标/编码符合共享约定；一个可复现起点 |
| 可执行 manifest | C 汇总，A/B 校验 | 参考 `configs/w2.mock.json` 的结构生成真实配置，并保留 `source=igibson`；通过 `EpisodeManifest.validate()`，不把 mock 配置重命名成真实配置 |

推荐先评估 Turtlebot，但最终型号要经过上述检查。C 的 mock 可继续开发，不需要等型号冻结。

## A：感知与建图

| ID | 要交付什么 | 明确验收标准 |
| --- | --- | --- |
| A1 | 完成 `perception.mapper.RGBDMapper.update(observation)` | 每次调用同步返回对应输入帧的完整 `WorldState`。`episode_id`/target 不变；`last_processed_sequence == observation.sequence`、时间与 capture pose 一致；revision 严格增加；不再抛 `NotImplementedError` |
| A2 | 空闲/占用融合 | 只使用观测和已验证出生净空；值为 `-1/0/100`，不做机器人足迹膨胀；无效深度不清空未知区；用固定墙面/地面/障碍样例断言更新位置；不能把桌面或碰撞高度之外的射线当作整车可通行证据 |
| A3 | 覆盖率 | occupancy/explored 几何和 revision 一致；出生净空不算观测覆盖；重复静止帧不虚增覆盖；已知区域按合法新观测增长；区域覆盖率在 [0,1]，空区域分母返回 None |
| A4 | 前沿与接近位姿 | 确定性提取已知空闲邻接未知的连通候选；支持格为已知空闲；approach 点为已知空闲，yaw 朝未知；至少一个公开样例能让 B 完整足迹检查通过。A 不宣称可达，最终由 B 判断 |
| A5 | 稳定 ID 与谱系 | 相同前沿重复观测保留 ID；分裂/合并产生新 ID，父 ID 指向之前的快照；episode 内不把旧 ID 分配给无关区域；提交分裂、合并、临时消失/重现测试 |
| A6 | 信息增益与不可变状态 | gain 为有限非负的预测面积，采用明确的范围/FOV/遮挡规则；不读取隐藏地图。更新后旧 state 的栅格/候选内容完全不变，嵌套数据为不可变值 |
| A7 | 录制数据回归包 | 至少一段多帧序列（含移动与扫描），包含 RGB-D、逐帧位姿/内外参、序号/时间、共同配置和运行说明；同配置回放产生一致地图与候选。用 `fixtures.recording` 的格式，数据可放忽略目录并提供获取方式 |

**A 的完成条件：** A1–A7 测试通过；给 C 一个可直接构造的 `RGBDMapper()` 和运行命令。
不要求 W2 完成 YOLO 跟踪、目标确认或完整房间分割；契约允许使用显式的已知房间边界先验。

## B：模拟器与导航

| ID | 要交付什么 | 明确验收标准 |
| --- | --- | --- |
| B1 | 完成真实 `IGibsonAdapter` | 实现 reset/capture/current_pose/step/stop/pause/resume/close，以及 `sim_time_s()`、`set_velocity(v,w)`。初始化稳定后 episode 时钟从 0 开始；只有 runner 的 step 推进时间；capture/poll 不推进时间 |
| B2 | RGB-D 转换与标定 | 返回 `rgb8` + `depth32f_le`，米制轴向深度、0 无效；相机轴 right/down/forward、整数像素中心、capture-time camera-to-world；可复用 A 的 `convert_igibson_frames`。已知平面/标志点投影通过；全程内参固定 |
| B3 | 出生净空 | `verify_spawn_clearance()` 仅返回初始足迹加安全边距覆盖的格子；地图索引与 A 一致；墙内/边界外/净空不足出生必须拒绝。可用隐藏几何做初始化检查，但不能把整张自由地图交给 A/C |
| B4 | 真实控制接现有导航 | `TeamNavigator` 使用同一个 adapter；直行方向与正 yaw 符号实测正确；正常目标在位置和 yaw 容差内到达；测试窄门、未知区域和斜角，禁止不安全路径 |
| B5 | 评估/执行一致性 | 相同地图、足迹、运动模型用于 evaluate 与 start；每个输入候选恰有一条 assessment；True 有有限非负成本；False 为 NO_PATH/INVALID_GOAL；超时为 None/TIMEOUT，不能伪装 NO_PATH |
| B6 | 扫描与运行钩子 | 保留 `on_step()` 和 `capture_marks(goal_id)`。runner 每步后调用 on_step；每个扫描角度达到容差并完成 dwell 后发布一个当前仿真时间 mark；mark 时机器人静止。runner 当步捕获指定样本，逐帧序号递增；扫描缺 mark、错 heading 或旧 timestamp 都会被 C 拒绝 |
| B7 | 失败、重规划、停止与资源释放 | 新地图堵路后先停止再对同一目标重规划；次数有上限；REACHED/FAILED/CANCELLED 只锁存一次；cancel 幂等；已规划成功仍能报告执行失败；stop/close 后轮速为 0，退出不遗留模拟器进程 |
| B8 | 可复现真实 smoke 与指标 | 提交正常到达、受阻、扫描三种脚本/测试，记录配置、实际路程、碰撞、终态和重规划；可报告 collisions 计数，需写清它是接触帧数还是独立碰撞次数 |

**B 的完成条件：** B1–B8 通过，现有 `test_navigation_b.py` 不退化，并给出适配器构造说明。
B 的 `MockSimulator.capture()` 本身是旧占位格式，不可直接传给 A；C 的 fixture 内已有符合格式的独立测试包装，不需要为迁就测试削弱 A 的校验。

## 如何接 C：只替换构造，不改策略或重写 runner

增加一个可导入的工厂，例如 `frontier_search.integration_factory:build_runner`：

```python
def build_runner(manifest, logger=None):
    adapter = ...  # B：按共同 manifest 构造真实 adapter
    mapper = ...   # A：RGBDMapper()
    navigator = TeamNavigator(adapter)
    policy = BaselinePolicy(PolicyName.NEAREST)
    return EpisodeRunner(adapter, mapper, navigator, policy, logger=logger)
```

工厂在被监督的子进程内构造所有模块，不在父进程创建 renderer。
真实适配器的构造函数目前未实现，因此这里保留显式待填项，不提供伪可运行的真实工厂。

```powershell
$env:PYTHONPATH = "$PWD/src"
python -m frontier_search.runner.w2 --manifest configs/w2.real.json --factory frontier_search.integration_factory:build_runner --log runs/w2-real-001.jsonl
```

上面是 A/B 完成交付后的命令，`w2.real.json` 及真实工厂需要按实测创建。
W2 模式不传 evaluator；如果 A 意外返回 target claim，会显式失败，不会误报搜索成功。

## 三道验收门

1. **合并门（已具备）：** Python 3.8 全套测试通过；保留角色分支，PR 合入 integration/w2；不提交权重、资产、虚拟环境或密钥。
2. **离线数据门（A/B 交付后）：** 同一录制序列通过 A → B 评估 → C 选择；版本匹配，无隐藏标签；包含可达与不可达候选。回放不执行策略生成的新轨迹，不用回放距离比较策略优劣。
3. **真实闭环门（本周末）：** 相同配置下 Nearest 至少两次选择/移动/到达扫描；日志证明地图有新观测、前沿变化、B 评估和 C 派发版本正确；正常场景无碰撞；不可达候选不派发；受阻、无进展、超时都有限终止；进程/轮速正确停止。

## 建议交付节奏

- 第 1 天：A/B/C 冻结共同配置；B 提供首帧合法采集与净空，A 给出 update 的最小实现。
- 第 2–3 天：A 完成地图/前沿/ID 回归；B 打通真实控制和扫描；各自提交小 PR。
- 第 3–4 天：共同录制数据联调，C 接真实工厂，修复接口差异。
- 第 5–7 天：通过真实闭环门，保留正常与失败日志；W3 再加入对象/声明/验证和语义策略实验。

你作为 C 现在需要协调的是**机器人/场景配置和上述交付时间**。C 的 mock 执行骨架已可运行，
不用等 A/B 才开始写；真实工厂接入与真实验收仍需要他们的模块完成。
