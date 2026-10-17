# STATUS_A · 感知与建图

> 旧提交快照；当前进展见 ../w2_integration_status.md。

2026-10-13，本地 a0c5250；完整签名/格式见 MEETING_PREP.md。

- VERIFIED：perception/的RGBDMapper.reset、RGB-D校验/投影、mark_obstacles、先验及fixtures/recording回放；今日相关测试通过。
- MISSING：生产 update、空闲/覆盖融合、前沿/谱系/增益、物体跟踪/声明、证据读取和生产验证器。MOCK：当前 runner 的地图来自 ScriptedMapper。
- 接口：update(observation: RGBDObservation) -> WorldState；get_claim_evidence(episode_id: str, claim_id: str) -> EvidenceBundle；reset(request: MapperInit) -> None。占据值-1/0/100，嵌套tuple。
- 依赖B：合法RGB-D、采集位姿/变换、实测足迹和出生净空；依赖共同manifest。
- 计划差异：W2地图/前沿未在本地生产实现中交付；W3跟踪/评分待补。
- 风险/下一步：先交付update及多帧回归，再完成证据和独立评分。UNTESTED：本次未重跑已有YOLO推理工具。
