# W2 最新 36 次实验与修复后复核报告

日期：2026-11-05（Asia/Shanghai）  
场景：iGibson `Rs` 单房屋  
目标类别：`cup`  
原始实验规模：3 个场景配置 × 3 个 seed/初始朝向 × 4 个策略 = 36 个 episode

> 后续：修复后以新指纹完整重跑 36 次的结果见 [w2_postfix_full36_report_20261105.md](w2_postfix_full36_report_20261105.md)（27 个基线 27/27 成功；该批次 LLM 数据因 API 失败无效，待补跑）。

## 1. 执行摘要

原始 36 次实验由两批结果合并得到：

- 27 个非 LLM episode：`runs/paired-w2-20261105`
- 9 个 Gemini 3.5 LLM episode：`runs/paired-w2-20261105-gemini35-llm`

两批实验的冻结引擎指纹完全相同：

`2599c6069cc435c3141448b14366fd23ec65839781d6255d2a2d20f9222b78b0`

因此它们可以组成 9 个完整配对块、共 36 个原始 episode。原始结果为：

- 总成功数：33/36（91.7%）
- 总失败数：3/36（8.3%）
- 总碰撞数：0
- 误报目标数：0
- 所有结果完整性检查：通过
- 冻结源码与共享资源检查：通过
- 各执行作用域内均无 episode 重试

按策略看：

- `llm_guided`：9/9 成功，平均仿真时间 153.41s，平均路程 8.00m
- `nearest`：9/9 成功，平均仿真时间 190.56s，平均路程 11.30m
- `information_gain`：8/9 成功，平均仿真时间 161.79s，平均路程 12.43m
- `random`：7/9 成功，平均仿真时间 171.41s，平均路程 14.07m

原始失败的 3 个 episode 均属于同一个工程问题：10cm 占用栅格把机器人边缘和新观测障碍压入同一格，导致导航器误判当前起点不可用，所有候选路径一起变成 `start_unusable/no_path`。修复后只重跑这 3 个 case，结果全部成功、碰撞仍为 0。

需要严格区分两个口径：

1. **原始、同指纹、可直接做配对比较的正式结果：33/36。**
2. **用修复后 3 次结果替换原失败项的工程更新视图：36/36。** 该视图混合了两个引擎指纹，不能冒充一次同版本、无重试的完整 36 次实验。

如果结果要用于最终论文或正式汇报，建议以新引擎指纹重新完整运行全部 36 个 episode；当前 3 次修复重跑只能证明问题已被修掉。

## 2. 实验设计与数据口径

### 2.1 配对设计

场景配置：

- `apartment`：基础公寓场景
- `clutter`：增加杂物干扰
- `dynamic`：包含动态物体/小车干扰

每个场景使用 3 个预先声明的 seed/初始朝向块：

- seed 17：初始 yaw 为 `+π/2`
- seed 29：初始 yaw 为 `0`
- seed 43：初始 yaw 为 `-π/2`

每个块运行 4 个策略：

- `random`
- `nearest`
- `information_gain`
- `llm_guided`，权威模型为 `gemini-3.5-flash`

所有策略共享相同场景、目标位置、机器人几何、传感器配置和评估条件。实验范围仅覆盖 `Rs` 这一套物理房屋，不代表跨房屋泛化结果。

### 2.2 运行限制

- 最大仿真时间：600s
- 最大 wall time：2400s
- 最大尝试数：45
- 最大决策数：80
- 最大仿真步数：18000
- 导航几何：`shared_round`
- 目标终止条件需要独立评估证据通过

### 2.3 指标解释

- `sim time`：仿真内部时间，适合比较搜索效率。
- `wall time`：真实运行耗时，受到并发、GPU、模型服务延迟影响。
- `distance`：机器人实际行驶距离。
- `penalized time`：失败 episode 按 600s 计入，用于避免“快速失败看起来更快”。
- `decision`：C 层做出选择的次数。
- `attempt`：真正执行的导航/探索尝试次数。
- `replan`：导航过程中因地图更新触发的局部重规划次数。

### 2.4 审计状态说明

两个原始结果目录都沿用“完整协议包含 36 个 job”的定义，因此单独查看时会看到 `complete: false`：第一个目录只执行了 27 个非 LLM job，第二个目录只执行了 9 个 LLM job。这不表示 episode 被中断。27/27 非 LLM 与 9/9 LLM 的预定作用域均已完成；两者的冻结引擎指纹相同，合并后恰好覆盖 36 个不同 job 和 9 个完整配对块。每条结果的 source、manifest、独立成功证据、capture sequence、decision revision 与 LLM 计数完整性检查均通过。

## 3. 原始 36 次总体结果

### 3.1 按策略汇总

| 策略 | 成功 | 成功率 | Wilson 95% CI | 平均 sim time (s) | 失败惩罚后时间 (s) | 平均路程 (m) | 平均 wall time (s) | 决策/尝试 | 重规划 | frontier 访问 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Random | 7/9 | 77.8% | 45.3%–93.7% | 171.41 | 282.19 | 14.07 | 305.18 | 59/55 | 9 | 38 |
| Nearest | 9/9 | 100.0% | 70.1%–100.0% | 190.56 | 190.56 | 11.30 | 340.11 | 71/71 | 10 | 42 |
| Information Gain | 8/9 | 88.9% | 56.5%–98.0% | 161.79 | 216.53 | 12.43 | 290.31 | 53/51 | 5 | 30 |
| LLM Guided | 9/9 | 100.0% | 70.1%–100.0% | **153.41** | **153.41** | **8.00** | **271.81** | 51/51 | **1** | **20** |

说明：Random 和 Information Gain 的普通平均时间包含提前失败，因此不能单独用来判断策略效率；比较含失败结果时应优先看失败惩罚后时间。

### 3.2 按场景配置汇总

| 场景配置 | 成功 | 成功率 | 平均 sim time (s) | 平均路程 (m) | 平均 wall time (s) |
|---|---:|---:|---:|---:|---:|
| Apartment | 10/12 | 83.3% | 167.71 | 14.45 | 297.66 |
| Clutter | 12/12 | 100.0% | 163.94 | 10.10 | 295.83 |
| Dynamic | 11/12 | 91.7% | 176.22 | 9.81 | 312.07 |

Apartment 的两个失败都集中在 seed 43；Dynamic 的一个失败位于 seed 17。Clutter 反而达到 12/12，说明本轮主要失败因素不是“杂物多就必然失败”，而是特定路线与栅格量化共同形成的起点自锁。

### 3.3 按 seed/初始朝向汇总

| Seed | 初始 yaw | 成功 | 成功率 | 平均 sim time (s) | 平均路程 (m) |
|---:|---:|---:|---:|---:|---:|
| 17 | +π/2 | 11/12 | 91.7% | 169.78 | 12.75 |
| 29 | 0 | 12/12 | 100.0% | 172.37 | 10.70 |
| 43 | -π/2 | 10/12 | 83.3% | 165.72 | 10.89 |

seed 43 的平均时间较短，部分原因是两个失败 episode 提前终止，不能据此认定 seed 43 更容易。

### 3.4 全局运行量

原始 36 次合计：

- 234 次决策
- 228 次实际尝试
- 25 次局部重规划
- 49 次低进展尝试
- 130 次 frontier 访问
- 0 次碰撞
- 平均仿真时间 169.29s
- 平均实际路程 11.45m
- 平均 wall time 301.85s

## 4. Gemini 3.5 / LLM Guided 结果

LLM 配置：

- 模型：`gemini-3.5-flash`
- 单次调用 deadline：30s
- 单 episode 调用上限：30
- 单候选时绕过 LLM：启用
- 相同上下文缓存：启用
- 向 LLM 提供：候选 frontier、信息增益、B 层路径成本、可达性/排除原因、最近执行反馈、观测物体与区域覆盖信息
- 不向 LLM 提供：隐藏地图、评估器真值、隐藏目标位置

### 4.1 LLM 总体调用数据

- LLM episode：9
- 系统成功：9/9
- 模型调用尝试：20
- 成功获得模型响应：19
- 服务失败并触发 fallback：1（HTTP 503）
- fallback 率：5.0%
- 调用延迟中位数：9.84s
- 调用延迟 P95：15.81s
- 总 token：196,640
- 可记录费用：0（provider 未返回价格数据，不能解释为真实成本为零）
- 实际响应模型：`gemini-3.5-flash`

唯一 fallback 出现在 `clutter-s29-llm_guided` 的第二次 LLM 决策。Gemini 请求返回 HTTP 503，系统使用已验证候选排序执行 fallback；该 episode 仍然成功。因此：

- 系统层面的可靠性是 9/9。
- 纯模型服务层面是 19/20 次调用成功返回。
- 这次成功不能全部归因于 LLM 服务本身，fallback 机制贡献了容错能力。

### 4.2 9 个 LLM episode 明细

| Episode | sim time (s) | 路程 (m) | wall time (s) | 决策/尝试 | LLM 调用 | fallback | token |
|---|---:|---:|---:|---:|---:|---:|---:|
| apartment-s17-llm_guided | 259.67 | 16.343 | 429.83 | 9/9 | 4 | 0 | 38,536 |
| apartment-s29-llm_guided | 117.00 | 6.629 | 226.72 | 4/4 | 2 | 0 | 20,489 |
| apartment-s43-llm_guided | 105.67 | 6.510 | 214.39 | 4/4 | 2 | 0 | 21,851 |
| clutter-s17-llm_guided | 154.00 | 7.173 | 261.00 | 6/6 | 2 | 0 | 22,181 |
| clutter-s29-llm_guided | 83.33 | 6.436 | 187.03 | 3/3 | 2 | 1 | 11,525 |
| clutter-s43-llm_guided | 164.67 | 7.211 | 271.67 | 6/6 | 2 | 0 | 19,170 |
| dynamic-s17-llm_guided | 154.00 | 7.173 | 253.42 | 6/6 | 2 | 0 | 20,487 |
| dynamic-s29-llm_guided | 183.33 | 7.355 | 310.22 | 7/7 | 2 | 0 | 19,618 |
| dynamic-s43-llm_guided | 159.00 | 7.213 | 291.98 | 6/6 | 2 | 0 | 22,783 |

## 5. 配对比较与统计解释

原始 36 次形成 9 个完整配对块。以下差值均为“前一个策略减去 LLM Guided”；正的时间/距离表示 LLM 更省。

| 比较 | 成功率差 | 共同成功块的时间差 (s) | 共同成功块的距离差 (m) | McNemar Holm p |
|---|---:|---:|---:|---:|
| Random − LLM | -22.2 pp | +31.24，95% CI [9.29, 53.19] | +6.48，95% CI [5.16, 7.80] | 1.0 |
| Nearest − LLM | 0.0 pp | +37.15，95% CI [-7.00, 80.37] | +3.29，95% CI [1.09, 5.35] | 1.0 |
| Information Gain − LLM | -11.1 pp | +9.23，95% CI [-24.46, 43.37] | +4.50，95% CI [3.15, 5.86] | 1.0 |

解释：

- LLM Guided 在本轮 9 个配对块中路程最短；与三个基线的配对距离差置信区间均未跨 0。
- 相对 Random，LLM 在共同成功块上的时间也明显更短。
- 相对 Nearest 和 Information Gain，时间差置信区间仍跨 0，不能声称时间优势已达到稳定统计结论。
- 成功率的 McNemar 检验经 Holm 校正后均不显著；样本只有 9 个配对块，统计功效有限。
- 这些区间只描述 `Rs` 单房屋内的 seed/朝向变化，不能当作跨房屋泛化区间。

## 6. 原始 3 个失败与修复后复核

### 6.1 原始失败

| Episode | 策略 | 原终止 | sim time (s) | 路程 (m) | 决策/尝试 | 重规划 | 碰撞 |
|---|---|---|---:|---:|---:|---:|---:|
| dynamic-s17-random | Random | no_feasible_frontiers | 87.33 | 9.947 | 5/3 | 1 | 0 |
| apartment-s43-random | Random | no_feasible_frontiers | 115.67 | 12.971 | 6/4 | 2 | 0 |
| apartment-s43-information_gain | Information Gain | no_feasible_frontiers | 107.33 | 10.339 | 6/4 | 2 | 0 |

三例的共同因果链：

1. RGB-D 新观测将机器人边缘附近的 10cm 栅格标成 occupied。
2. 机器人真实物理足迹与该粗栅格相交，但仿真接触反馈为 0。
3. B 层认为当前 pose 已在障碍中，规划起点变成 `start_unusable`。
4. 所有候选同时返回 `no_path`。
5. 原地 recovery 扫描也因相同栅格而被判定 blocked。
6. C 层最终只能以 `no_feasible_frontiers` 结束。

### 6.2 修复方式

地图发布给导航器时，对“当前机器人真实足迹与 occupied 粗栅格相交”的格子做当前帧遮罩，允许机器人从其实际所在位置离开。

该修复不是永久删除障碍：

- 底层原始传感器占用仍保留。
- 机器人离开后，原障碍会重新出现在发布地图中。
- unknown 格子仍然是 unknown。
- 不伪造 explored/coverage。
- 足迹外障碍不变。
- 物理接触、路径、转向和安全检查仍然有效。

### 6.3 修复后 3 次复核结果

修复批次：`runs/paired-w2-20261105-start-footprint-fix-3cases`  
新引擎指纹：`72a51a52547257257ca4213ceabf3bc9c64f1a615cfc965c473ca28387071783`

| Episode | 修复后结果 | sim time (s) | 路程 (m) | 决策/尝试 | 重规划 | 碰撞 |
|---|---|---:|---:|---:|---:|---:|
| dynamic-s17-random | success | 158.67 | 11.984 | 5/5 | 0 | 0 |
| apartment-s43-random | success | 201.33 | 15.200 | 8/7 | 1 | 0 |
| apartment-s43-information_gain | success | 151.67 | 15.492 | 5/5 | 1 | 0 |

三例均越过原来的失败时间与位置，继续探索并成功找到目标，说明修复命中了真实失败链，而不是简单增加重试次数。

## 7. 修复后“更新视图”

如果只用于工程状态汇报，把修复后的 3 条记录替换原失败项，可得到：

- 36/36 成功
- 0 碰撞
- 平均仿真时间 174.88s
- 平均路程 11.71m

| 策略 | 更新后成功 | 平均 sim time (s) | 平均路程 (m) |
|---|---:|---:|---:|
| Random | 9/9 | 188.85 | 14.54 |
| Nearest | 9/9 | 190.56 | 11.30 |
| Information Gain | 9/9 | 166.72 | 13.00 |
| LLM Guided | 9/9 | **153.41** | **8.00** |

但是这张表混合了旧引擎与新引擎：Random/Information Gain 的 3 条记录使用修复后指纹，其他 33 条仍使用原指纹。因此它适合说明“当前代码已经能解决全部已知 case”，不适合直接作为最终受控策略比较。

## 8. 结论

1. 原始、同版本的 36 次实验成功率为 91.7%，没有碰撞和目标误报。
2. LLM Guided 与 Nearest 都达到 9/9 成功；LLM Guided 同时取得最短平均时间和最短平均路程。
3. LLM Guided 的主要优势在路程效率：相对 Nearest 平均少 3.29m，相对 Information Gain 少 4.50m，相对 Random 少 6.48m。
4. LLM 服务并非完全稳定：20 次调用中有一次 HTTP 503，但 fallback 保证了 episode 成功。
5. 原始 3 个失败不是策略层本身不可解，而是粗栅格造成的当前起点自锁；修复后 3/3 成功且无碰撞。
6. 当前工程状态已经达到已知 36 个 case 全部可通过，但最终正式对比仍应在新指纹下完整重跑 36 次。

## 9. 原始 36 条 episode 数据

下表保留原始同指纹结果，不用修复后记录覆盖失败项。

| Episode | Profile | Seed | Policy | 终止 | sim time (s) | 路程 (m) | 决策 | 尝试 | 碰撞 | 重规划 |
|---|---|---:|---|---|---:|---:|---:|---:|---:|---:|
| apartment-s17-information_gain | apartment | 17 | information_gain | success | 147.67 | 15.557 | 5 | 5 | 0 | 1 |
| apartment-s17-llm_guided | apartment | 17 | llm_guided | success | 259.67 | 16.343 | 9 | 9 | 0 | 1 |
| apartment-s17-nearest | apartment | 17 | nearest | success | 161.33 | 16.035 | 7 | 7 | 0 | 2 |
| apartment-s17-random | apartment | 17 | random | success | 227.67 | 28.397 | 7 | 7 | 0 | 3 |
| apartment-s29-information_gain | apartment | 29 | information_gain | success | 156.80 | 15.312 | 5 | 5 | 0 | 2 |
| apartment-s29-llm_guided | apartment | 29 | llm_guided | success | 117.00 | 6.629 | 4 | 4 | 0 | 0 |
| apartment-s29-nearest | apartment | 29 | nearest | success | 272.33 | 16.972 | 11 | 11 | 0 | 2 |
| apartment-s29-random | apartment | 29 | random | success | 188.67 | 13.016 | 7 | 7 | 0 | 2 |
| apartment-s43-information_gain | apartment | 43 | information_gain | no_feasible_frontiers | 107.33 | 10.339 | 6 | 4 | 0 | 2 |
| apartment-s43-llm_guided | apartment | 43 | llm_guided | success | 105.67 | 6.510 | 4 | 4 | 0 | 0 |
| apartment-s43-nearest | apartment | 43 | nearest | success | 152.67 | 15.280 | 5 | 5 | 0 | 1 |
| apartment-s43-random | apartment | 43 | random | no_feasible_frontiers | 115.67 | 12.971 | 6 | 4 | 0 | 2 |
| clutter-s17-information_gain | clutter | 17 | information_gain | success | 159.00 | 11.791 | 6 | 6 | 0 | 0 |
| clutter-s17-llm_guided | clutter | 17 | llm_guided | success | 154.00 | 7.173 | 6 | 6 | 0 | 0 |
| clutter-s17-nearest | clutter | 17 | nearest | success | 192.00 | 7.529 | 8 | 8 | 0 | 1 |
| clutter-s17-random | clutter | 17 | random | success | 169.67 | 14.495 | 6 | 6 | 0 | 0 |
| clutter-s29-information_gain | clutter | 29 | information_gain | success | 189.33 | 12.254 | 7 | 7 | 0 | 0 |
| clutter-s29-llm_guided | clutter | 29 | llm_guided | success | 83.33 | 6.436 | 3 | 3 | 0 | 0 |
| clutter-s29-nearest | clutter | 29 | nearest | success | 155.33 | 7.087 | 6 | 6 | 0 | 1 |
| clutter-s29-random | clutter | 29 | random | success | 140.33 | 11.842 | 5 | 5 | 0 | 0 |
| clutter-s43-information_gain | clutter | 43 | information_gain | success | 166.00 | 11.888 | 6 | 6 | 0 | 0 |
| clutter-s43-llm_guided | clutter | 43 | llm_guided | success | 164.67 | 7.211 | 6 | 6 | 0 | 0 |
| clutter-s43-nearest | clutter | 43 | nearest | success | 222.00 | 12.482 | 10 | 10 | 0 | 1 |
| clutter-s43-random | clutter | 43 | random | success | 171.67 | 10.966 | 6 | 6 | 0 | 0 |
| dynamic-s17-information_gain | dynamic | 17 | information_gain | success | 147.67 | 11.290 | 5 | 5 | 0 | 0 |
| dynamic-s17-llm_guided | dynamic | 17 | llm_guided | success | 154.00 | 7.173 | 6 | 6 | 0 | 0 |
| dynamic-s17-nearest | dynamic | 17 | nearest | success | 177.33 | 7.323 | 7 | 7 | 0 | 0 |
| dynamic-s17-random | dynamic | 17 | random | no_feasible_frontiers | 87.33 | 9.947 | 5 | 3 | 0 | 1 |
| dynamic-s29-information_gain | dynamic | 29 | information_gain | success | 213.00 | 11.890 | 7 | 7 | 0 | 0 |
| dynamic-s29-llm_guided | dynamic | 29 | llm_guided | success | 183.33 | 7.355 | 7 | 7 | 0 | 0 |
| dynamic-s29-nearest | dynamic | 29 | nearest | success | 138.67 | 6.775 | 7 | 7 | 0 | 1 |
| dynamic-s29-random | dynamic | 29 | random | success | 230.33 | 12.845 | 9 | 9 | 0 | 0 |
| dynamic-s43-information_gain | dynamic | 43 | information_gain | success | 169.33 | 11.533 | 6 | 6 | 0 | 0 |
| dynamic-s43-llm_guided | dynamic | 43 | llm_guided | success | 159.00 | 7.213 | 6 | 6 | 0 | 0 |
| dynamic-s43-nearest | dynamic | 43 | nearest | success | 243.33 | 12.177 | 10 | 10 | 0 | 1 |
| dynamic-s43-random | dynamic | 43 | random | success | 211.33 | 12.147 | 8 | 8 | 0 | 1 |

## 10. 数据与审计文件

- 非 LLM 原始结果：[`../../runs/paired-w2-20261105/results.json`](../../runs/paired-w2-20261105/results.json)
- 非 LLM 完整性证明：[`../../runs/paired-w2-20261105/suite-verification.json`](../../runs/paired-w2-20261105/suite-verification.json)
- Gemini 3.5 原始结果：[`../../runs/paired-w2-20261105-gemini35-llm/results.json`](../../runs/paired-w2-20261105-gemini35-llm/results.json)
- Gemini 3.5 完整性证明：[`../../runs/paired-w2-20261105-gemini35-llm/suite-verification.json`](../../runs/paired-w2-20261105-gemini35-llm/suite-verification.json)
- 修复后 3 例结果：[`../../runs/paired-w2-20261105-start-footprint-fix-3cases/results.json`](../../runs/paired-w2-20261105-start-footprint-fix-3cases/results.json)
- 修复后完整性证明：[`../../runs/paired-w2-20261105-start-footprint-fix-3cases/suite-verification.json`](../../runs/paired-w2-20261105-start-footprint-fix-3cases/suite-verification.json)

