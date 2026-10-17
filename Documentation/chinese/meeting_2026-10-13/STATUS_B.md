# STATUS_B · 导航与规划

> 旧提交快照；当前进展见 ../w2_integration_status.md。

2026-10-13，本地 a0c5250；完整签名/格式见 MEETING_PREP.md。

- VERIFIED：navigation/的costmap/planner/TeamNavigator；足迹/A*、评估、执行、重规划、取消/扫描及MockSimulator。今日相关与3项跨模块测试通过。
- MISSING：simulation/igibson_adapter.py的真实IGibsonAdapter。MOCK：当前采集/时钟/控制与机器人参数。
- 接口：evaluate_candidates(request: CandidateEvaluationRequest) -> CandidateEvaluation；start(request: NavigationRequest) -> NavigationFeedback；另有start_observation/update_map/poll/cancel/on_step/capture_marks。评估含版本、位姿及每候选的可达性/路径成本/失败原因。
- 依赖A：未膨胀地图、前沿接近位姿；依赖C推进step后调用on_step并处理扫描帧。
- 计划差异：真实链未验收，PARTIAL。
- 下一步：完成adapter；验收净空、直行/转向、到达、受阻、扫描和退出。
