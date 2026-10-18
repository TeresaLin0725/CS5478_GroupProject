# 当前场景与 demo 录制记录（2026-10-18 / 19）

入口：[HTML 报告](../../report/index.html)（根目录 `demo.html` 与 README 均指向该入口）。四策略 36 回合配对统计见 [当前表现与统计口径](paired_benchmark.md)，跨房屋与 W4 可靠性用例的边界见 [设计核对](demo_design_review.md)。

## 场景

已从 `origin/integration/w2` 快进合入队友提交 `43617b9d2ea16a68c021f1c40d6e608a3ab7953a`。该提交增加动态推车的完整扫掠碰撞和地面支撑检查；静态扫描地图仍为 Gibson Rs，并未换成另一套房屋。

## 场景修订及证据范围

原始五条推车候选均被本机的真实扫描家具阻挡，初始化按设计拒绝运行。最终增加北侧备用路线 `(0.8, 1.15) → (0.8, 2.7)`，完整扫掠与地面支撑均通过原生查询。该路线在仿真 16.0333 秒开始，以原定约 0.2333 m/s 前进，22.7 秒在厨房内侧停妥，不返回门洞。停车位置留出通过门洞后的转弯空间；运动过程中仍逐步检查扫掠，新增障碍会令推车停止。查询空间不会写入感知地图或策略状态。

推车道具原来将没有瓶颈、瓶盖的直筒标为瓶子；真实 YOLO 将它判为杯子，独立验收正确拒绝该次声明。瓶子现补上瓶颈、瓶盖和标签，保持在原推车检查包络内。检测阈值仍为 0.50，目标最大投影尺寸先验仍为 0.25 m，目标位置、导航安全裕量和独立验收条件未改变。外观对照使用单独安排的诊断位姿，不作为真实行驶或搜索成功证据。

Locobot 控制器距离增益为 2/s，速度上限 0.32 m/s、1.1 rad/s；目标附近停车后做多视角确认。推车瓶子使用真实 YCB 纹理网格，碰撞包络不变。录像使用独立旁观相机与补光，已核验不影响机器人的 RGB、深度、实例分割与物理步（[旁观相机隔离检查](../../report/evidence/demo/llm_guided/spectator-isolation.json)）。

## 四策略 demo

报告为每种前沿选择策略各收录一段录像，均使用同一源码与模型指纹（`bbd90a23…`）、同一控制器与独立验收条件，录制前各自通过无录制原生试跑与录制许可检查（共享上述 384 项回归）。

| 策略 | 场景条件 | 仿真时间 | 行驶距离 | 接触 | 策略决策 | 在线调用 |
|---|---|---:|---:|---:|---:|---:|
| Random | 公寓原始 | 170.33 s | 11.08 m | 0 | 6 | 0 |
| Nearest | 移动推车干扰 | 155.00 s | 11.09 m | 0 | 6 | 0 |
| Information Gain | 移动推车干扰 | 242.00 s | 15.57 m | 0 | 9 | 0 |
| LLM-guided | 移动推车干扰 | 199.67 s | 11.56 m | 0 | 7 | 6（`gemini-3.8-flash`，0 回退） |

每段视频除初始化帧外均与真实物理步一一对应（最大间隔 0.066667 秒）。Random 在推车干扰条件的 3 个配对回合均未成功，因此它的展示录像放在公寓原始条件。各段证据：[Random](../../report/evidence/demo/random/)、[Nearest](../../report/evidence/demo/nearest/)、[Information Gain](../../report/evidence/demo/information_gain/)、[LLM-guided](../../report/evidence/demo/llm_guided/)，每个目录含 `recording/`、`dryrun/` 与 `validation.json`。单次录像不用于估计成功率或比较策略，比较见 [配对实验](paired_benchmark.md)。

下文为 LLM-guided 录像的详细验收记录。

## 录制前验证

384 项回归全部通过，测试期间源码未变化：[完整日志](../../report/evidence/demo/checks/tests.log)、[进程退出码及源码证书](../../report/evidence/demo/checks/tests.json)。

真实在线 LLM 无录制试跑通过：[汇总](../../report/evidence/demo/llm_guided/dryrun/summary.json)。同源码 [录制许可](../../report/evidence/demo/llm_guided/validation.json) 的全部 18 项检查通过。源码与模型指纹为 `bbd90a237a14b60bdddcd8e9f32524ea350ada88b94d34028f34caa2fd0140a1`。

## 正式录像

| 指标 | 实测值 |
|---|---:|
| 仿真时间 | 199.67 s |
| 实际计算耗时 | 648.67 s |
| 实际行驶距离 | 11.56 m |
| 物理接触步数 | 0 |
| 策略决策 / 导航 | 7 / 7（全部完成） |
| 真实模型调用 / 回退 | 6 / 0（`gemini-3.8-flash`） |

两份终止证据来自不同采集时刻，匹配保留的真实目标实例，机器人已停稳，通过独立验收。暂定视觉候选不等于任务完成。

2,996 帧 15 fps H.264 全部解码；初始化后的 2,995 帧均与真实物理步的 x/y/yaw 对应，最大间隔 0.066667 秒，无路径位置插值。[视频及轨迹核验](../../report/evidence/demo/llm_guided/recording/artifact-verification.json)保存视频 SHA256。

[模型调用核验](../../report/evidence/demo/llm_guided/recording/llm-verification.json)确认 6 个回答均来自在线服务、属于有效候选、与下发目标一致，日志输入与实际请求一致。本次发生 1 次局部重规划；目标接近与受阻恢复由共享的确定性规则处理，没有请求模型处理导航失败反馈。

真实 Chrome 本地文件验收通过：视频可解码，52 个时间定位按钮均可跳转，手机宽度无页面溢出。

单次录像不用于估计成功率、平均性能或未知环境中的识别准确率。

## 复现

新电脑先按[运行环境、安装、依赖和自检说明](running.md)准备环境。Windows x64 已实测；Linux 原生安装和完整流程尚未在第二台机器验证。

```powershell
python scripts/run_integrated.py --policy llm_guided --profile dynamic --output runs/my-demo/dryrun
./data/venv-igibson/Scripts/python.exe scripts/run_integrated_checks.py --output runs/my-demo/checks/tests.log
./data/venv-igibson/Scripts/python.exe scripts/validate_integrated.py --run runs/my-demo/dryrun --tests runs/my-demo/checks/tests.log --output runs/my-demo/validation.json
python scripts/run_integrated.py --policy llm_guided --profile dynamic --output runs/my-demo/recording --record --validation runs/my-demo/validation.json
./data/venv-igibson/Scripts/python.exe scripts/check_integrated_artifacts.py runs/my-demo/recording
./data/venv-igibson/Scripts/python.exe scripts/build_demo_report.py --recording runs/my-demo/recording --output runs/my-demo/report
```

每次选择新的输出目录；源码、模型或推理环境改变后重新验证。在线调用需要自己的 `.env`，离线可将策略改为 `nearest`。使用 `package_demo_report.py` 导出录像证据包，使用 `package_source_handoff.py` 导出当前源码及运行说明；两个包均排除密钥和虚拟环境。
