# 当前四策略配对表现

完整完成 36 回合、9 组配对：同一 Gibson Rs 房屋，3 种条件 × 3 个种子/起始朝向 × 4 策略。所有失败纳入统计，没有单回合重试。

| 策略 | 成功 / 9 | 成功率 | Wilson 95% 区间 | 失败计 600 s 的平均时间 | 成功平均时间 | 成功平均路程 |
| --- | ---: | ---: | --- | ---: | ---: | ---: |
| Random | 4/9 | 44.4% | 18.9%–73.3% | 393.0 s | 134.2 s (n=4) | 9.79 m |
| Nearest | 6/9 | 66.7% | 35.4%–87.9% | 307.5 s | 161.3 s (n=6) | 8.08 m |
| Information Gain | 7/9 | 77.8% | 45.3%–93.7% | 291.0 s | 202.7 s (n=7) | 13.12 m |
| LLM-guided | 4/9 | 44.4% | 18.9%–73.3% | 391.0 s | 129.7 s (n=4) | 8.08 m |

成功率优先于仅成功样本的效率；失败计罚时间避免“很早失败”被误当作高效率。区间描述当前小样本，不代表跨房屋泛化。

## 逐任务结果

| 场景 / 种子 / 朝向 | Random | Nearest | IG | LLM-guided |
| --- | --- | --- | --- | --- |
| 原始公寓 / 17 / +90° | 成功 · 91.0 s | 成功 · 146.0 s | 成功 · 92.0 s | no_feasible_frontiers · 109.6 s |
| 拥挤公寓 / 17 / +90° | 成功 · 147.0 s | 成功 · 105.0 s | 成功 · 145.7 s | 成功 · 97.3 s |
| 动态障碍 / 17 / +90° | no_feasible_frontiers · 112.5 s | 成功 · 155.0 s | 成功 · 242.0 s | 成功 · 199.7 s |
| 原始公寓 / 29 / +0° | no_feasible_frontiers · 96.3 s | no_feasible_frontiers · 152.6 s | 成功 · 103.0 s | 成功 · 125.0 s |
| 拥挤公寓 / 29 / +0° | 成功 · 218.0 s | no_feasible_frontiers · 142.9 s | 成功 · 243.0 s | no_feasible_frontiers · 119.9 s |
| 动态障碍 / 29 / +0° | no_feasible_frontiers · 121.3 s | 成功 · 118.3 s | 成功 · 274.0 s | no_feasible_frontiers · 214.3 s |
| 原始公寓 / 43 / -90° | 成功 · 81.0 s | no_feasible_frontiers · 158.7 s | no_feasible_frontiers · 535.8 s | 成功 · 96.7 s |
| 拥挤公寓 / 43 / -90° | no_feasible_frontiers · 262.9 s | 成功 · 217.3 s | search_limit · 290.7 s | search_limit · 290.7 s |
| 动态障碍 / 43 / -90° | no_feasible_frontiers · 203.7 s | 成功 · 226.0 s | 成功 · 319.0 s | no_feasible_frontiers · 200.4 s |

## 失败与运行成本

| 策略 | 终止分布 | 误声明 | 接触步数 / 有接触回合 | 重规划 | 无进展尝试 | 前沿访问 | 平均实际计算耗时 |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Random | success: 4; no_feasible_frontiers: 5 | 0 | 0 / 0 | 1 | 14 | 31 | 375.3 s |
| Nearest | success: 6; no_feasible_frontiers: 3 | 0 | 0 / 0 | 1 | 11 | 33 | 393.3 s |
| Information Gain | success: 7; no_feasible_frontiers: 1; search_limit: 1 | 0 | 0 / 0 | 4 | 37 | 37 | 604.7 s |
| LLM-guided | no_feasible_frontiers: 4; success: 4; search_limit: 1 | 0 | 0 / 0 | 1 | 11 | 28 | 413.0 s |

LLM 共 33 次调用，0 次回退，回退率 0.0%；记录 token 合计 547558；调用延迟中位数 5.25 s，P95 8.16 s。
服务未返回数值费用，无法报告实际总费用。

响应模型：gemini-3.8-flash。回退原因：无。

实际计算耗时来自 3 个并发工作进程，共享本机资源，不能解释为单机器人独占硬件延迟。固定单操作墙钟超时也可终止搜索，并发计算会影响这类终止；本批描述当前并发模式下的整条管线表现。

## 复核与限制

原始事件、manifest、失败回合、原始 LLM 回答、私有验收证据均保留。audit.json 重算全部成功证据与 10,000 次配对重采样统计；原生代码、模型与资产校验和前后保持一致。

Rs 几何与目标位置固定；种子对应预先声明的初始朝向，Random 另有独立策略 RNG。Nearest / IG 与检测器在同样输入下确定；在线 LLM 不能由本地种子保证完全重复。初始化包含 0.65 m 已验证自由区域，多视角近距离停稳确认对四策略一致。

有 9 回合在搜索结束后发生调用记录导出异常；已从完整原生日志恢复原终止结果与调用记录，修复前文件保留，没有重跑或排除回合。


[主统计与配对区间](benchmark.html) · [实际轨迹图](trajectories.png) · [完整统计 JSON](evidence/benchmark/results.json) · [独立复核](evidence/benchmark/audit.json)
