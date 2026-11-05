# W2 修复后 36 次全量重跑报告（2026-11-05）

日期：2026-11-05（Asia/Shanghai）  
场景：iGibson `Rs` 单房屋，目标类别 `cup`  
规模：3 个场景配置 × 3 个 seed/初始朝向 × 4 个策略 = 36 个 episode  
前序报告：[w2_latest_36_experiment_report_20261105.md](w2_latest_36_experiment_report_20261105.md)

## 1. 结论摘要

前序报告建议在起点足迹修复后，用新引擎指纹完整重跑 36 次。本轮已完成该重跑：

- 批次：`runs/paired-w2-20261105-postfix-full36`
- 引擎指纹：`72a51a52547257257ca4213ceabf3bc9c64f1a615cfc965c473ca28387071783`（与 3 例修复复核批次相同）
- 36/36 episode 正常结束，9 个配对块完整，无重试，冻结源码、共享资源、结果完整性检查全部通过
- 36/36 `success`，0 碰撞，0 目标误报

但 **LLM Guided 这 9 条结果无效**：42 次 Gemini 请求全部以 `URLError` 失败，每次都退回到确定性 fallback 排序，其行为与 Nearest 逐条完全相同。因此本轮可用的正式结论只覆盖 27 个非 LLM 基线：

| 结论 | 状态 |
|---|---|
| 修复后基线 27/27 成功、0 碰撞（同一指纹、无重试） | ✅ 可作为正式结果 |
| 起点自锁修复在完整批次中复现，且未改变其他 24 个基线 episode | ✅ 已验证 |
| LLM Guided 修复后表现 | ❌ 无有效数据，待补跑 |
| LLM 补跑 | ⏸ 被 Gemini 免费额度阻塞（每日 20 次） |

## 2. 实验设计

设计与前序报告一致，未做改动：

- 场景配置：`apartment`、`clutter`、`dynamic`
- seed/初始朝向：17（`+π/2`）、29（`0`）、43（`-π/2`）
- 策略：`random`、`nearest`、`information_gain`、`llm_guided`（`gemini-3.5-flash`）
- 限制：最大仿真时间 600s、wall time 2400s、45 次尝试、80 次决策、18000 步；导航几何 `shared_round`；成功需独立评估证据
- 运行方式：3 个并发 worker；正式运行前三个场景均通过 iGibson 原生预检

唯一相对原始批次的代码变化是前序报告第 6.2 节的起点足迹遮罩修复。

## 3. 基线结果（27 个 episode，正式口径）

### 3.1 按策略

| 策略 | 成功 | Wilson 95% CI | 平均 sim time (s) | 中位数 (s) | 范围 (s) | 平均路程 (m) | 决策/尝试 | 重规划 | 低进展尝试 | frontier 访问 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Random | 9/9 | 70.1%–100% | 188.85 | 188.67 | 140.3–230.3 | 14.54 | 61/60 | 7 | 10 | 38 |
| Nearest | 9/9 | 70.1%–100% | 190.56 | 177.33 | 138.7–272.3 | **11.30** | 71/71 | 10 | 19 | 42 |
| Information Gain | 9/9 | 70.1%–100% | **166.72** | **159.00** | 147.7–213.0 | 13.00 | **52/52** | **4** | **7** | **30** |

27 个基线合计：平均仿真时间 182.04s，平均路程 12.95m，0 碰撞。

### 3.2 按场景配置与 seed

| 场景 | 成功 | 平均 sim time (s) | 平均路程 (m) |
|---|---:|---:|---:|
| Apartment | 9/9 | 184.46 | 16.81 |
| Clutter | 9/9 | 173.93 | 11.15 |
| Dynamic | 9/9 | 187.74 | 10.89 |

| Seed | 初始 yaw | 成功 | 平均 sim time (s) | 平均路程 (m) |
|---:|---:|---:|---:|---:|
| 17 | +π/2 | 9/9 | 171.22 | 13.82 |
| 29 | 0 | 9/9 | 187.20 | 12.00 |
| 43 | −π/2 | 9/9 | 187.70 | 13.02 |

Apartment 的平均路程明显更长（16.81m），是三个场景中搜索路线最长的布局。

### 3.3 配对比较

9 个配对块，差值为“前者 − 后者”，负的时间/路程表示前者更省；区间为 t 分布 95% CI（df = 8）。

| 比较 | 时间差 (s) | 95% CI | 前者更快的块数 | 路程差 (m) | 95% CI |
|---|---:|---:|---:|---:|---:|
| Information Gain − Nearest | −23.84 | [−67.39, 19.72] | 7/9 | +1.71 | [−0.48, 3.89] |
| Information Gain − Random | −22.13 | [−49.59, 5.32] | 8/9 | −1.54 | [−4.97, 1.88] |
| Nearest − Random | +1.70 | [−42.77, 46.17] | 3/9 | −3.25 | [−7.14, 0.65] |

解释：

- 三个基线成功率相同（均 9/9），成功率维度无法区分策略。
- Information Gain 在 9 个块中有 7 块用时最短，平均比另两者快约 22–24s，且决策、重规划、低进展尝试都最少；但时间差 CI 仍跨 0，不能宣称统计显著。
- Nearest 路程最短，但用时方差最大（apartment-s29 272.33s、dynamic-s43 243.33s）：它倾向于在近处 frontier 之间来回，路短但决策多（71 次）。
- Random 路程最长（apartment-s17 达 28.40m），平均时间与 Nearest 接近。

## 4. 修复效果验证

与原始批次 `runs/paired-w2-20261105`（旧指纹 `2599c606…`）逐条对比：

- **24 个原本成功的基线 episode：终止原因、仿真时间、路程逐条完全相同**。修复只在机器人足迹与 occupied 栅格相交时生效，没有改变其他 episode 的轨迹。
- **3 个原本失败的 episode：全部成功，且与 3 例修复复核批次的结果逐位一致**，说明修复后的行为是确定性的、可复现的。

| Episode | 原始（旧指纹） | 本轮（新指纹） |
|---|---|---|
| dynamic-s17-random | no_feasible_frontiers，87.33s / 9.947m | success，158.67s / 11.984m |
| apartment-s43-random | no_feasible_frontiers，115.67s / 12.971m | success，201.33s / 15.200m |
| apartment-s43-information_gain | no_feasible_frontiers，107.33s / 10.339m | success，151.67s / 15.492m |

因此，前序报告第 7 节“修复后更新视图”中 Random / Nearest / Information Gain 三行数字，现在已由同一指纹、无重试的完整批次正式确认。

## 5. LLM Guided 结果无效的原因

### 5.1 现象

| 项目 | 数值 |
|---|---:|
| LLM 决策点（请求尝试） | 42 |
| 成功获得模型响应 | 0 |
| fallback | 42（`client_error:RuntimeError`，transport 层 `URLError`） |
| token 用量 | 0 |

由于 fallback 使用确定性候选排序，9 个 LLM episode 与同块 Nearest episode 的决策数、仿真时间、路程**逐条完全相同**。批次报告中 LLM Guided 的 9/9 成功、190.56s、11.30m 实际是 Nearest 的复制，不能当作 LLM 结果引用。

注意：批次的完整性检查只核对 LLM 计数是否自洽，不检查调用是否真正成功，所以 `all_result_integrity_checks_passed: true` 不代表 LLM 数据有效。

### 5.2 排查

- `URLError` 表示请求未到达 Gemini 服务。transport 只记录异常类名，具体网络原因无法事后还原。
- 运行结束后，项目连接检查 `scripts/check_llm_connection.py` 一度返回 HTTP 200，确认 key、端点和模型名均正确。

### 5.3 补跑尝试与额度阻塞

按“不在原批次内重试”的规则，新建 LLM-only 批次 `runs/paired-w2-20261105-postfix-llm-rerun`：

- 冻结后指纹为 `72a51a52…`，与本轮全量批次一致，补跑结果可与 27 个基线直接配对。
- 前 3 个 episode（s17 三场景）的首次调用全部返回 **HTTP 429**。随后连接检查也返回 429：

  > Generate content requests per day limit exceeded on Free Tier（`GenerateRequestsPerDayPerProjectPerModel-FreeTier`，limit 20）

- 已通过 `cancel.request` 取消该批次；3 个已启动 episode 记为 `externally_cancelled`（sim 约 29s），其余 6 个未启动，无残留进程。该目录已被使用，不再复用。

额度结论：当前 key 处于免费层，每模型每天 20 次请求。一个完整 9-episode LLM 批次约需 40+ 次调用（本轮为 42 次决策点），**单日免费额度不足以完成一次完整 LLM 批次**。另外，429 报文中的配额维度显示为 `gemini-3.6-flash`，说明请求名 `gemini-3.5-flash` 在服务端可能按另一模型名计配额；正式补跑时应记录 `response_model` 并核对。

### 5.4 与旧 LLM 结果的关系

旧指纹下的 Gemini 3.5 批次（`runs/paired-w2-20261105-gemini35-llm`）为 9/9 成功、平均 153.41s、8.00m，19/20 次调用成功。由于本轮 24 个未受修复影响的基线 episode 在新旧指纹下完全一致，可推测修复对 LLM episode 的影响有限；但这只是推测，**不能把旧 LLM 结果与新指纹基线混合成正式配对比较**。

## 6. 后续步骤

1. **解决额度（需负责人决定）**：为该 Google 项目开通计费，或在 `D:\CS5478_PROJECT\.env` 中换用付费 key。备选为按免费额度分 3 天、每天 3–4 个 episode，每次新建批次后合并。
2. 补跑前运行 `scripts/check_llm_connection.py`，确认 HTTP 200。
3. 新建批次并执行：

   ```powershell
   ./data/venv-igibson/Scripts/python.exe scripts/run_paired_benchmark.py freeze --output runs/<新目录>
   ./data/venv-igibson/Scripts/python.exe scripts/run_paired_benchmark.py run --only-llm --output runs/<新目录>
   ```

   确认指纹仍为 `72a51a52…`，并在第一次 LLM 调用后检查 `fallback_reason` 为空、token 非零。
4. 建议的工程改进：
   - transport 记录 `URLError.reason`（不含 key），使网络失败可事后诊断；
   - 批次报告中对 `fallback_rate = 100%` 的 LLM episode 给出显式警告或标记为 integrity 问题，避免无效结果被当成成功汇报。

## 7. 全部 36 条 episode 数据

LLM Guided 行为 fallback 结果，仅供审计，不作为策略结果。

| Episode | 终止 | sim time (s) | 路程 (m) | 决策 | 尝试 | 重规划 | 碰撞 | LLM 尝试/fallback |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| apartment-s17-random | success | 227.67 | 28.397 | 7 | 7 | 3 | 0 | – |
| apartment-s17-nearest | success | 161.33 | 16.035 | 7 | 7 | 2 | 0 | – |
| apartment-s17-information_gain | success | 147.67 | 15.557 | 5 | 5 | 1 | 0 | – |
| apartment-s17-llm_guided | success | 161.33 | 16.035 | 7 | 7 | 2 | 0 | 6/6 |
| apartment-s29-random | success | 188.67 | 13.016 | 7 | 7 | 2 | 0 | – |
| apartment-s29-nearest | success | 272.33 | 16.972 | 11 | 11 | 2 | 0 | – |
| apartment-s29-information_gain | success | 156.80 | 15.312 | 5 | 5 | 2 | 0 | – |
| apartment-s29-llm_guided | success | 272.33 | 16.972 | 11 | 11 | 2 | 0 | 6/6 |
| apartment-s43-random | success | 201.33 | 15.200 | 8 | 7 | 1 | 0 | – |
| apartment-s43-nearest | success | 152.67 | 15.280 | 5 | 5 | 1 | 0 | – |
| apartment-s43-information_gain | success | 151.67 | 15.492 | 5 | 5 | 1 | 0 | – |
| apartment-s43-llm_guided | success | 152.67 | 15.280 | 5 | 5 | 1 | 0 | 4/4 |
| clutter-s17-random | success | 169.67 | 14.495 | 6 | 6 | 0 | 0 | – |
| clutter-s17-nearest | success | 192.00 | 7.529 | 8 | 8 | 1 | 0 | – |
| clutter-s17-information_gain | success | 159.00 | 11.791 | 6 | 6 | 0 | 0 | – |
| clutter-s17-llm_guided | success | 192.00 | 7.529 | 8 | 8 | 1 | 0 | 4/4 |
| clutter-s29-random | success | 140.33 | 11.842 | 5 | 5 | 0 | 0 | – |
| clutter-s29-nearest | success | 155.33 | 7.087 | 6 | 6 | 1 | 0 | – |
| clutter-s29-information_gain | success | 189.33 | 12.254 | 7 | 7 | 0 | 0 | – |
| clutter-s29-llm_guided | success | 155.33 | 7.087 | 6 | 6 | 1 | 0 | 3/3 |
| clutter-s43-random | success | 171.67 | 10.966 | 6 | 6 | 0 | 0 | – |
| clutter-s43-nearest | success | 222.00 | 12.482 | 10 | 10 | 1 | 0 | – |
| clutter-s43-information_gain | success | 166.00 | 11.888 | 6 | 6 | 0 | 0 | – |
| clutter-s43-llm_guided | success | 222.00 | 12.482 | 10 | 10 | 1 | 0 | 6/6 |
| dynamic-s17-random | success | 158.67 | 11.984 | 5 | 5 | 0 | 0 | – |
| dynamic-s17-nearest | success | 177.33 | 7.323 | 7 | 7 | 0 | 0 | – |
| dynamic-s17-information_gain | success | 147.67 | 11.290 | 5 | 5 | 0 | 0 | – |
| dynamic-s17-llm_guided | success | 177.33 | 7.323 | 7 | 7 | 0 | 0 | 3/3 |
| dynamic-s29-random | success | 230.33 | 12.845 | 9 | 9 | 0 | 0 | – |
| dynamic-s29-nearest | success | 138.67 | 6.775 | 7 | 7 | 1 | 0 | – |
| dynamic-s29-information_gain | success | 213.00 | 11.890 | 7 | 7 | 0 | 0 | – |
| dynamic-s29-llm_guided | success | 138.67 | 6.775 | 7 | 7 | 1 | 0 | 4/4 |
| dynamic-s43-random | success | 211.33 | 12.147 | 8 | 8 | 1 | 0 | – |
| dynamic-s43-nearest | success | 243.33 | 12.177 | 10 | 10 | 1 | 0 | – |
| dynamic-s43-information_gain | success | 169.33 | 11.533 | 6 | 6 | 0 | 0 | – |
| dynamic-s43-llm_guided | success | 243.33 | 12.177 | 10 | 10 | 1 | 0 | 6/6 |

## 8. 数据与审计文件

- 全量批次结果：[`../../runs/paired-w2-20261105-postfix-full36/results.json`](../../runs/paired-w2-20261105-postfix-full36/results.json)
- 全量批次完整性证明：[`../../runs/paired-w2-20261105-postfix-full36/suite-verification.json`](../../runs/paired-w2-20261105-postfix-full36/suite-verification.json)
- 全量批次自动报告：[`../../runs/paired-w2-20261105-postfix-full36/report/index.html`](../../runs/paired-w2-20261105-postfix-full36/report/index.html)（其中 LLM Guided 数据无效，见第 5 节）
- 已取消的 LLM 补跑批次：[`../../runs/paired-w2-20261105-postfix-llm-rerun/progress.json`](../../runs/paired-w2-20261105-postfix-llm-rerun/progress.json)
- 原始批次与修复复核批次：见前序报告第 10 节
