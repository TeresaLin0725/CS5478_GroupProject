# B 导航：机器人两种"卡住"的情况与修复

范围：只涉及 B（导航与规划）。代码：`navigation/navigator.py`、`navigation/costmap.py`、`navigation/planner.py`。测试：`tests/test_stuck_fixes.py`。
**状态：仅在 mock 仿真器中验证，尚未在原生 iGibson 中验证。** 默认配置（规划边距等于起点边距）下行为与此前完全一致。英文版为准：`Documentation/english/b_navigation_stuck_fixes.md`。

## 1. 扫描朝向卡在容差边界

现象（如 `apartment-s29-nearest`）：原地扫描结束后，机器人停在误差 0.0503 rad，容差 0.05 rad。这一步既不完成也不失败，一直拖到单次操作上限。

原因：
- **转速指令太小。** 比例控制给 `K × 误差 = 2.0 × 0.0503 = 0.1 rad/s`，低于底盘静摩擦能通过的值，机器人停在容差之外。
- **停滞检测被驻留重置。** 观察驻留免于停滞检测，而每次重新进入容差都会把 5 s 的停滞窗口清零，在 0.05 rad 附近抖动的机器人永远到不了停滞上限。

修复（`navigator.py`）：
- 新增 `_turn_rate`：保留比例控制，但在朝向仍在容差外时转速不低于 `MIN_TURN_RADPS = 0.2` rad/s（也不超过机器人自身上限）。按 0.05 s 步长，一步最多转约 0.01 rad，不会跨过 0.1 rad 宽的容差带。0.2 是按步长选的，没有在真实底盘上调过。
- 驻留仍免于停滞检测，但不再重置窗口，只有完成一个扫描朝向才重置。抖动的机器人约 5 s 后被判 `BLOCKED`。
- 容差本身不变（进入驻留仍是 0.05 rad）。

机器人学依据：对有静摩擦的对象，纯比例控制在零误差附近存在死区；加一个最小指令（克服静摩擦的前馈项）即可消除。

## 2. 起点边距没有用于第一次转向和第一步

现象（3 个回合）：机器人停车误差或新扫到的障碍格，让它落进 1 cm 的规划缓冲带。起点位姿被接受（`start_margin_m = 0`），但所有第一步都被判 `no_path`，扫描被判 `blocked`。失败策略只在证据变化时重试，不动的机器人不产生新证据，于是死锁。

原因：`start_margin_m` 只用在 `start_pose_valid`；第一次原地转向和第一步（`first_move_valid`、`rotation_valid`、`segment_valid`）仍按规划边距检查。

修复：
- `Costmap.first_move_valid` 对转向和前 `START_ZONE_M = 0.10` m 的行驶使用 `start_margin_m`，出了这一段恢复规划边距。`pose_valid`、`rotation_valid`、`full_rotation_valid` 和 `segment_valid(start_zone_m=...)` 增加了可选边距参数。
- 起点处的原地观察扫描、导航中起点附近的第一次原地转向、`planner.path_valid` 以及从起点出发的平滑捷径也使用起点边距。
- 起点格被膨胀阻塞时，`planner._success` 不再让路径经过起点格中心。该中心可能落在障碍里，而 `first_move_valid` 已经批准了直接驶向下一个格子。报告的代价不变，仍与 `evaluate_candidates` 使用的距离场一致。

机器人学依据：规划边距是构型空间里的障碍膨胀，是规划缓冲，不是物理接触。已经在缓冲带内的机器人必须能走出来，同时仍用精确足迹做碰撞检查。

取舍：当 `start_margin_m < planning_margin_m` 时，起点这一小段没有缓冲；`path_valid` 对机器人当前位姿改用起点边距，重规划时约放宽 1 cm。两者只对可选配置（`SOFT_CONTACT_NAV_OPTIONS`、`ROUND_FOOTPRINT_*`）生效。

## 没有改动的部分

- 机器人尺寸：0.19 m 的圆形足迹与实测 Locobot 吻合（碰撞圆柱 0.178 m，网格约 0.188 m），不缩小。
- `navigation/lattice.py` 不是 B 的模块，未改动。
- 失败类别 3（只按距离选目标，`policies/baselines.py`）属于 C；类别 4（接近点放不下机器人，`frontier_approach_search_m`）属于 A。

## 验证

- `tests/test_stuck_fixes.py`（14 个测试）在旧代码上失败，现在通过。全部测试 440 个，4 个跳过。
- 36 个 mock 仿真场景（12 个扫描朝向，分有无静摩擦；24 个起点在缓冲带内，分 3 个离墙距离 × 4 个朝向 × 导航/扫描）：修改前 6/36，修改后 36/36。这些场景是针对这两处修复设计的，说明修复有效，不代表整体成功率。
- 足迹说明：16 边形圆形足迹的外接半径是 0.1835 m，略大于名义 0.18 m，因此名义间隙小于约 3.5 mm 的起点会被正确拒绝。
- 待做：在原生 iGibson 上重跑配对实验。
