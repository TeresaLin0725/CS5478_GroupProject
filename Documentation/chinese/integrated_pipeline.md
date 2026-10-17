# 完整原生管线与录制门槛

入口为 `src/frontier_search/runner/integrated.py`，启动脚本为 `scripts/run_integrated.py`。原 `igibson_demo.py` 仍是历史演示，不能用于声称完成 W3 生产管线验收。

数据流：原生 Gibson Rs / Locobot → `IGibsonAdapter` → 标定后的 `RGBDObservation` → RGB-only YOLO11s → 共享 `RGBDMapper` 的深度融合、跨帧跟踪、区域关联、稳定前沿与冻结目标证据 → `TeamNavigator` 候选评估、A*、实体足迹与旋转检查、轮速控制 → `EpisodeRunner` / 策略 / `SpatialRecovery` → `RetainedTargetEvaluator` 独立验收。

感知与策略不接收实例分割、目标实体坐标、完整场景可通行图或验收结果。实例分割和同次渲染的可见表面真值只进入私有 `RetainedFrames`，终止时独立检查每张证据的类别、同一实例、可见比例、框 IoU、三维位置误差。另检查两张不同时间、距离不超过 0.95 米、实体速度足够低的证据，且终止时已停止下发轮速。感知侧使用自己的深度估计和观测位姿判断靠近及停稳，阈值为 0.90 米。

实物目标为 YCB `025_mug`，与 COCO 的 `cup` 类对应，放在扫描厨房台面上。相机为机器人真实 `eyes` 链接，640×480、75° 垂直视场；每帧记录并验证实际内参、外参、投影误差与真实机器人 z，不重写位姿。YOLO 使用仓库筛选配置指定的官方 `yolo11s.pt`，SHA256 为 `85a76fe86dd8afe384648546b56a7a78580c7cb7b404fc595f97969322d502d5`。每个 RGB 帧运行全图检测，每三个帧增加固定重叠图块的小目标检测；跳过图块的帧不会复用旧检测框。任务显式采用杯子最大投影尺寸 0.25 米的物理先验，避免将大软凳等误作为杯子。首次识别可以提出暂定接近候选，但不代表完成；最终仍须两张不同时间的近距离停稳证据及独立验收。B 使用覆盖整个车体转身范围的默认外接圆规划，位置到位容差为 0.03 米，避免到达后才发现不能安全转向。

只有起点周围声明的 0.65 米范围允许用场景实体进行初始化检查：对整格完整碰撞高度柱做碰撞查询、对地面多点采样，再保留与机器人初始足迹连通的部分。房间几何只用于关联观察到的语义，不填充自由空间。

双向反馈：新的 RGB-D 地图让剩余路线失效时，B 先停止再在当前目标下重规划；无安全路线、超时或实体接触会返回失败原因，C 接收 `previous_attempt`、排除项与最新候选后重新选择。实体接触在同一控制 tick 停止并报告 blocked。移动物体离开后，只有连续两个不同时刻的完整新自由高度柱加新地面证据才允许清除已占据格；当前障碍点优先，旧自由证据不能擦掉新障碍。

原生试跑还复现了到位控制抖动：29 秒内反复跨过 3 厘米阈值 24 次，导致位置修正和最终朝向控制交替。B 现采用位置捕获滞回：先到达半个位置容差内，再调整朝向；漂移仍在完整容差内时保持朝向调整，超出完整容差则重新定位。最终成功条件没有放宽。修复后的原生组件检查在 5.83 秒到位，位置误差 0.02283 米、朝向误差 0.09963 弧度、零碰撞。诊断和检查结果分别为 `runs/integrated-preflight-20261017/arrival-thrashing-diagnostic.json` 与 `native-arrival-check.json`。

固定相机在机器人近处存在地面盲区。所有普通前沿和目标接近候选不可达时，C 才能使用 A 根据已观测自由格发布的 `navigation_probe` 换视点候选：B 仍检查实体足迹、转向扫掠和整条路线，移动后重新观察旧障碍所在位置。普通目标可达时，这些恢复候选以明确排除项延后，防止干扰正常探索。该过程不根据时间直接擦障碍，也不把场景真值填入导航地图。

## 先验证，后录制

```powershell
python scripts/run_integrated.py --policy llm_guided --profile dynamic --output runs/integrated-dryrun
./data/venv-igibson/Scripts/python.exe scripts/run_integrated_checks.py --output runs/my-llm-checks/tests.log
python scripts/validate_integrated.py --run runs/integrated-dryrun --tests runs/my-llm-checks/tests.log --output runs/my-llm-final/validation.json
python scripts/run_integrated.py --policy llm_guided --profile dynamic --output runs/my-llm-final/recording --record --validation runs/my-llm-final/validation.json
```

无 `--record` 时不会构造视频录制器。录制要求验证报告通过，且源码/模型指纹、场景难度和策略与验证一致。测试和真实不录制端到端运行均需要通过后才能生成报告。运行期间源码变化会使该次试跑不能作为录制依据。

安装 `requirements-integrated.txt` 时使用官方 PyTorch CUDA 12.1 索引（`--extra-index-url https://download.pytorch.org/whl/cu121`）。主入口检测本地 CUDA，存在时将真实 YOLO 推理放到 GPU，否则使用 CPU。运行结果保存 `detector_runtime`（设备、PyTorch/CUDA 版本、GPU 名称）；录制前还要与无录制验证使用的推理环境一致。CPU 机器可将两个 Torch 依赖的 `+cu121` 改成 `+cpu`，使用 `/whl/cpu` 索引。

未确认的单帧目标假设保留 10 秒，超过期限且仍未再次确认时不再发布接近候选，以限制短暂误检的影响。多帧静态目标记忆保留。该期限、扫描周期和近距离成功确认的 3 秒证据窗口分别配置。

视频每秒 15 帧，来自真实仿真步，没有路径位置插值。策略推理期间物理时钟暂停，不用视频重复帧伪装机器人运动。主画面为实体第三人称渲染；右侧小窗是最近一张已经处理的 RGB 及真实 YOLO 框。网页、视频、事件日志、实际标定、配置、冻结证据与验收专用真值一起保存在结果目录。CPU 上管线处理可能慢于实时，视频按真实仿真时间播放。

首次整合时 LLM 服务返回 HTTP 402 / RESOURCE_EXHAUSTED，详细信息确认预付额度已耗尽。当时使用了明确标注的 Nearest 录像。现 API 已恢复 HTTP 200，生产客户端已取得并解析 `gemini-3.8-flash` 的真实决策 JSON，恢复后的完整无录制试跑见下文。服务失败时 `LLMGuidedPolicy` 仍会明确记录回退；LLM 录像验收新增了真实模型回答、零回退及候选合法性检查。

已核对实际读取的是 `D:/CS5478_PROJECT/.env`，key、模型与接口均匹配文件内容，没有继承环境变量覆盖。最小独立请求同样收到 Google 返回的 `Your prepayment credits are depleted`。这说明有效凭据和可用计费额度需要分别检查；[Google 官方计费说明](https://ai.google.dev/gemini-api/docs/billing#prepay)说明余额归零会返回 HTTP 402。错误详情保存在 `runs/integrated-preflight-20261017/llm-connectivity-detail.json`，其中不含密钥。修改额度或本地配置后可以运行：

```powershell
./data/venv-igibson/Scripts/python.exe scripts/check_llm_connection.py --output runs/llm-connection-check.json
```

该命令发送一次最小请求，使用和生产客户端相同的配置加载方式，输出配置来源、服务状态和错误正文，不打印凭据。HTTP 402 的检查进程退出码为 1，表示检查未通过。

这是一张原生 Gibson 扫描公寓，并增加可碰撞的家具与运动推车；扫描网格本身不是逐件可交互的 iGibson 室内资产。检查通过表示当前配置和已覆盖场景的验证通过，不代表任意环境都无缺陷。

## 2026-10-17 的录制前验收

367 项回归全部通过，测试期间源码未变化。该次源码与模型指纹为 `d66a32723416b9489ac148a6378f3e4678a56958a363b08cecd3fe35b7b01799`。证书与完整日志在 `runs/integrated-preflight-20261017/tests-certified-gpu.json` / `.log`。

最近前沿策略的完整动态场景无录制试跑成功：121.6667 秒仿真、7.1597 米实际行驶、零接触；两张冻结证据分别距实体 0.7970、0.7928 米，W3 身份/可见性/框与位置检查、不同时间、停稳、近距离和终止停止检查全部通过。结果在 `runs/integrated-validation-nearest-gpu-20261017/summary.json`，录制门槛在 `runs/integrated-final-20261017/validation.json`。这次完整试跑耗时 315.5 秒，管线尚非实时运行；视频按仿真时钟播放。

同一版本另做信息增益策略的动态反馈专项试跑：23.0 秒新地图使原目标失效，B 局部重规划失败并反馈 C；23.3333 秒第二条路线无路径，C 改选已观测自由空间中的换视点，机器人实际移动恢复，零接触。该专项在完成闭环后主动结束，并非完整目标搜索成功记录。其事件与校验在 `runs/integrated-final-20261017/feedback-validation.json`。主视频的策略以自身运行结果为准。

正式录像随后完成，结果与无录制试跑一致：[打开主入口](../../runs/integrated-final-20261017/recording/index.html)。视频为 15 fps、1826 帧，完整 H.264 解码通过；除初始化帧外的 1825 帧均与物理步日志中的 x/y/yaw 精确对应，最大帧间隔 0.066667 秒。另执行网页实际内联脚本的离线 DOM 检查，19 个事件按钮均可定位到有效时间；已人工检查首帧、5 秒、中段、结尾及最终验收画面。未进行实际浏览器截图验收。

当时将 `runs/igibson-realistic-final-20261014/index.html` 和仓库根目录 `demo.html` 指向了 Nearest 主入口；现已更新为下文的真实 LLM 演示。原独立回放页保存在同目录的 `index-legacy-before-integration-20261017.html` / `demo-legacy-before-integration-20261017.html`。

## LLM 恢复后的验收（2026-10-18）

首次恢复后的试跑中，8 次真实模型调用全部成功且无回退，但小角度到达扫描留下了通道观察盲区，最终在椅凳旁安全停止、未完成搜索。这次未录制，失败结果保留在 `runs/integrated-validation-llm-live-20261017`。

到达后的观察改为原朝向、四个相隔 90° 的朝向，完成整圈扫描；每个朝向仍须停稳并处理新 RGB-D 帧，B 仍检查完整车体与转向扫掠。新源码与模型指纹为 `87c7ad2f3ac8f3d30db8a7ca898d4008f71fce406af1c0d267b84ca49cd070c6`，367 项回归再次全部通过，证书在 `runs/integrated-llm-preflight-20261017/tests-certified.json`。

扩大扫描后的完整无录制试跑成功：204.0333 秒仿真、10.8595 米、零接触；4 次真实 `gemini-3.8-flash` 调用成功、零回退。两张最终证据距实体 0.80044、0.80017 米，通过 W3 与近距离停稳验收。结果在 `runs/integrated-validation-llm-live-scan-20261017/summary.json`，录制前校验在 `runs/integrated-llm-final-20261018/validation.json`。

LLM 使用的请求是公开 Gibson Rs 仿真派生的物体类别、虚拟坐标、候选编号/成本及失败摘要；不发送 RGB、深度图或栅格地图。字段核对记录在 `runs/integrated-llm-preflight-20261017/outbound-scope.json`。正式录像会独立保存实际 API 回答和使用的候选，`scripts/package_llm_demo.py` 校验每个回答与实际下发目标一致后展示原文。早期 Nearest 录像采用旧扫描配置，仅作为历史参考。

正式录像已完成：[打开真实 LLM 演示](../../runs/integrated-llm-final-20261018/recording/index.html)。本次录像重新向 Google 请求了 4 次模型回答，零回退；204.0333 秒仿真、10.8595 米实际行驶、零接触，W3 独立验收通过。录像中有 2 次局部地图重规划、1 次模型决策接收 B 的失败反馈。`llm-verification.json` 检查了实际请求与日志输入一致、回答均属于候选、且每个回答都与实际下发目标对应。

15 fps H.264 的 3062 帧全部解码通过，除初始化外的 3061 帧均与物理步 x/y/yaw 一致，最大帧间隔 0.066667 秒。网页内联脚本的离线 DOM 检查通过，22 个事件按钮均可跳转到有效时间；另人工检查首帧、中段、结尾和最终验收画面。未进行实际浏览器截图验收。你当前打开的旧 `index.html` 和根目录 `demo.html` 均已指向新版，同时保留进入早期 Nearest 录像的链接。
