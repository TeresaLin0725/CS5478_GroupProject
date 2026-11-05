# LLM 权威模型

自 2026-11-05 起，C 侧新实验的权威 hosted model 为 `gemini-3.5-flash`。

- 默认值由 `frontier_search.policies.provider.CANONICAL_LLM_MODEL` 唯一定义。
- `.env`、冻结 manifest、连接检查和报告必须记录同一个模型名。
- `gemini-3.8-flash` 曾连续返回 HTTP 503 `UNAVAILABLE`，不得作为权威批次的静默替代。
- 模型切换必须创建新冻结批次；不得修改已经开始运行的 protocol、manifest 或 frozen source。
- 报告同时保留请求模型、服务端响应模型、真实调用数、回退原因、用量与延迟。

2026-11-05 的 27 个非 LLM 回合保留在 `runs/paired-w2-20261105`。对应的 9 个
LLM 回合在 `runs/paired-w2-20261105-gemini35-llm` 中独立冻结和运行；两者使用相同的
engine fingerprint、任务、seed、场景和导航配置，模型补充批次显式标注为 `llm_guided only`。
