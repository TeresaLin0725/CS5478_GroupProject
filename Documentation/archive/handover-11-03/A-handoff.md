# A → C 交接

2026-11-04。**本交接与 A 的代码一并提交；局部地图实验默认关闭。** C 在其他工作区运行前，先同步包含本文件的提交。以下为 A 的唯一维护交接文档。

## 已完成

- 新运行入口统一使用 `shared_round_manifest_for()`：A、B 与初始化共享 16 边形、0.19 m 顶点半径、0.01 m 正常净空；B 起步余量仍为 0 m。历史预设和旧运行不改写，旧方形初始化证书不能用于圆形配置。
- 实验局部地图保留全局 10 cm 地图，在 3 m × 3 m 窗口内融合 5 cm 证据。新增有界历史缓存，细格离窗后保留真实观测证据，返回时恢复，解决 Random-17 的证据遗忘。新障碍仍优先，UNKNOWN 不直接释放，障碍清除计数不能跨离窗累计。
- 实验导航适配器已修复地图更新时丢失到点保持的问题。C 使用整套已验证代码即可；本轮未修改 B 核心导航或 C 策略/预算。

## C 如何接入

在新冻结协议中显式加入下列字段，由 `scripts/benchmark_episode.py` 消费：

```json
{
  "navigation_preset": "shared_round",
  "local_collision_refinement": true,
  "local_collision_scope": "window",
  "local_collision_retain_history": true
}
```

做缓存 OFF/ON 对照时，**两组都保持 refinement=true、scope=window，只切换 retain_history**。其余源码、权重、manifest、种子、初始化、几何和预算相同；epsilon/stuck/relaxed 恢复均关闭。使用新输出目录，保留旧结果及超时记录。当前仅授权三个本地策略，不运行 LLM/Gemini。

主要代码：`src/frontier_search/experimental/collision_refinement/`；共享入口为 `src/frontier_search/runner/integrated.py`，协议接入为 `scripts/benchmark_episode.py`。

## 已验证

完整回归 **603 项通过**，本机 Windows/iGibson。源码/模型指纹：`9b4683d690e05e7d37dc0ad8630fbcfe23335b98f69e0bfde16a7021f3abef86`。

| 案例 | 缓存 OFF | 缓存 ON |
|---|---|---|
| Dynamic-17 Random | 87.33 s 无可行候选 | **200.33 s 独立任务验收成功** |
| Apartment-17 Nearest | 263.00 s 成功 | 263.00 s 成功，运动轨迹相同 |

四条有效回合均零接触；Nearest ON 观测墙钟约多 2.2%，仅一次配对，不能作稳定开销估计。首次 Random OFF 起步超时已保留，表中使用用户授权补跑的有效结果，未将基础设施异常计作算法失败。

证据：[有效配对复盘](../../../runs/local-history-rerun-20261104-01/PAIR_REVIEW.md)、[回归证书](../../../runs/local-history-20261104-01/regression.json)。

## C 下一步

1. 在同一合并版本上跑三个本地策略的完整 paired suite，覆盖其他历史失败、成功控制和种子；先固定 `window`，不要同时调策略或预算。`current_conflicts` 加缓存的组合尚未验证。
2. 记录实际缓存模式/恢复格数、候选及可达数、任务结束原因、独立成功验收、碰撞、路径、仿真与墙钟时间。离线可达不等于任务成功，基础设施失败单列。
3. 检查恢复与回退后再决定是否启用默认。若使用录像，先完成 recovery/navigation 实际配置的绑定验证；本轮没有修复该验证缺口。其他平台的运行设施需单独确认。

目前只证明这两个案例中的恢复与控制表现，尚未完成全套基准验收；不要据此宣称总体成功率或性能提升。
