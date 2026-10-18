# 当前四策略原生配对实验（2026-10-18）

[四策略统计页](../../report/benchmark.html)直接读取保存的回合结果，[逐任务分析](../../report/analysis.html)列出实际轨迹与失败原因。四策略都结束的任务才纳入主配对表。

本批次冻结 36 个全新原生回合：**同一 Gibson Rs 房屋的原始、拥挤、动态障碍三种条件 × 三组种子 / 初始朝向 × Random、Nearest、Information Gain、在线 LLM-guided 四策略**。这是当前已安装场景的条件比较，不能解释为多套房屋的泛化实验。

| 配对块 | 环境种子 | 感知种子 | 策略种子 | 初始朝向 |
| --- | ---: | ---: | ---: | ---: |
| 1 | 17 | 10017 | 20017 | +90° |
| 2 | 29 | 10029 | 20029 | 0° |
| 3 | 43 | 10043 | 20043 | −90° |

起点统一为 `(0.8, -2.4)`，真实 YCB mug 位于厨房。房屋几何、物体位置、模型权重、RGB-D 相机、已知位姿 / 房间边界先验、0.65 m 已验证初始化区域、控制器、扫描 / 停车多视角确认、恢复和预算一致。种子本身不生成另一套房屋；Random 有独立策略 RNG，确定性策略与 YOLO 在相同输入下不必变化，在线模型也不能由本地种子保证完全重复。

原生引擎指纹为 `bbd90a237a14b60bdddcd8e9f32524ea350ada88b94d34028f34caa2fd0140a1`，与已通过 384 项回归及当前 demo 录像的版本一致。冻结配置见 [protocol.json](../../report/evidence/benchmark/protocol.json)，仓库入口为 `configs/experiment.rs-20261018.json`。模板文件继续保留 `draft_not_for_experiments`，不要用空模板代替本批配置。

每种条件先独立验证目标存在、近距离可见，并用完整足迹与朝向的几何规划证明三个起始朝向可达。动态条件的预检查包含“在安全起点等待推车停妥”的可行性证明；正式策略没有读取该路线或全图。私有全图、实例和目标位姿只用于隔离预检查、终止验收及实验后绘图，不传给建图或策略。

成功必须通过独立实例、包围框、定位与可见性检查，至少两份不同时刻的近距离停稳证据，以及终止停止。不能把 YOLO 一次检出或到达目标附近当成成功。

统计口径：

- 全部成功、无可行前沿、误目标声明、预算用尽和运行错误均保留；无单回合重试。
- 主表以完整四策略配对块为单位，完成后每策略分母为 9；单独报告各场景条件的 3 回合结果。
- 成功率使用描述性 Wilson 95% 区间；策略差异按条件分层、对种子 / 朝向配对块做 10,000 次 bootstrap，随机数种子 20261018。六个成功差异比较使用精确 McNemar 检验并做 Holm 校正。
- 失败统一计 600 s，避免很早失败被当成高效率。成功条件下的耗时和路程同时标出成功样本数；实际执行时间和路程也保留在每回合 CSV / JSON。
- 记录接触步、局部重规划、无进展尝试、前沿访问，以及 LLM 原始输入 / 回答、调用延迟、token、回退。没有返回数值费用时不估造总费用。
- 三个并发进程共享硬件，墙钟耗时不能当作单机器人独占硬件延迟。仿真时间为主要时间口径。冻结配置同时含全局和单操作墙钟上限；单操作墙钟超时可在 600 s 仿真上限前终止搜索，并发计算会影响这类终止。本批描述当前并发模式下的整条管线表现，不能直接归因于纯策略差异。失败分析单列超时环节和触发边界。


在当前环境复现，先完成原生安装与自己的在线模型配置，再选择新的输出目录：

```powershell
./data/venv-igibson/Scripts/python.exe -m unittest discover -s tests -p test_paired_benchmark.py -v
./data/venv-igibson/Scripts/python.exe scripts/run_paired_benchmark.py all --output runs/my-paired-benchmark --workers 3
./data/venv-igibson/Scripts/python.exe scripts/recover_benchmark_exports.py runs/my-paired-benchmark
./data/venv-igibson/Scripts/python.exe scripts/check_paired_benchmark.py runs/my-paired-benchmark
./data/venv-igibson/Scripts/python.exe scripts/summarize_paired_benchmark.py runs/my-paired-benchmark
./data/venv-igibson/Scripts/python.exe scripts/verify_demo_report.py --report runs/my-paired-benchmark/report --output runs/my-paired-benchmark/browser-check --checker scripts/check_benchmark_report.js
./data/venv-igibson/Scripts/python.exe scripts/package_paired_benchmark.py --run-root runs/my-paired-benchmark --output runs/my-paired-benchmark/evidence.zip
```

最终复核会根据原始检测、保留的真实实例与运动记录重算每个成功声明，并根据冻结统计代码重算全部最终配对区间。报告可以直接打开 HTML，无需部署或联网加载页面资源。

v3 冻结运行器另有一个仅发生在搜索结束后的记录导出问题：嵌套的 `PolicyName` 枚举使 `llm-calls.json` 导出失败。处理只从完整原始事件恢复已经结束的回合及调用记录，不修改原生终止结果、不重新执行搜索；修复前的监督器汇总、结果和异常日志均保留，逐回合处理依据见 `export-recovery.json`。机器人与冻结运行器快照未中途改动。当前工作区的导出函数已补充枚举支持及回归检查，用于未来新批次。

<!-- FINAL_RESULTS_BEGIN -->
## 本批最终结果

| 策略 | 成功 / 全部 | 成功率 | Wilson 95% 区间 | 失败计 600 s 的平均时间 | 成功平均时间 |
| --- | ---: | ---: | --- | ---: | ---: |
| Random | 4/9 | 44.4% | 18.9%–73.3% | 393.0 s | 134.2 s (n=4) |
| Nearest | 6/9 | 66.7% | 35.4%–87.9% | 307.5 s | 161.3 s (n=6) |
| IG | 7/9 | 77.8% | 45.3%–93.7% | 291.0 s | 202.7 s (n=7) |
| LLM-guided | 4/9 | 44.4% | 18.9%–73.3% | 391.0 s | 129.7 s (n=4) |

在线模型共 33 次调用、0 次回退，记录 token 合计 547558。费用可得性、失败分布、实际轨迹与全部原始结果见报告。

六个成功率配对比较的 Holm 校正 p 最小为 1.0000；当前小样本未显示校正后 p < 0.05 的成功率差异。 不据此推断跨房屋性能。
<!-- FINAL_RESULTS_END -->
