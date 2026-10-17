# STATUS_C · 搜索策略与运行器

> 旧提交快照；当前进展见 ../w2_integration_status.md。

a0c5250；接口详见MEETING_PREP.md。

- VERIFIED：policies四策略/LLM校验回退；runner/的EpisodeRunner、SpatialRecovery、manifest/supervisor及JSONL。今日153项通过；mock两次派发、1.8049m、零碰撞，no_frontiers。
- MOCK：mapper、simulator与评分替身。MISSING：真实工厂/manifest；生产ABC闭环PARTIAL。UNTESTED：本次未跑live LLM。
- 接口：run(manifest: EpisodeManifest) -> TerminationReason；select(state: WorldState, context: SelectionContext) -> SelectionResult。输出Subgoal携带frontier、姿态和来源revision。
- 依赖A：update/证据/验证器；依赖B：adapter、评估/执行、on_step及capture_marks。
- 计划差异：旧iGibson演示未验收统一ABC；物体搜索/配对实验未完成。
- 下一步：配置冻结后接真实工厂，验收离线链、几何探索、独立评分；保留失败日志/预算。
