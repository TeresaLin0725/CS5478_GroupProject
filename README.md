# Indoor frontier object search

## 最新结果（2026-10-18）

打开 **[report/index.html](report/index.html)**（或根目录 [demo.html](demo.html)）。报告只收录最新一版结果：

- **Demo**：四种前沿选择策略各一段 15 fps 原生仿真录像，Locobot 在 Gibson Rs 公寓中搜索杯子，全部通过独立目标验收；页面可切换策略，并按决策、操作和验收记录定位播放。Random 在推车干扰条件下的配对回合均未成功，因此它的 demo 录在公寓原始条件。单次录像只用于展示，策略比较以下方配对实验为准。

| 策略 | 场景条件 | 仿真时间 | 行驶距离 | 接触 | 策略决策 | 在线调用 |
|---|---|---:|---:|---:|---:|---:|
| Random | 公寓原始 | 170.33 s | 11.08 m | 0 | 6 | 0 |
| Nearest | 移动推车干扰 | 155.00 s | 11.09 m | 0 | 6 | 0 |
| Information Gain | 移动推车干扰 | 242.00 s | 15.57 m | 0 | 9 | 0 |
| LLM-guided | 移动推车干扰 | 199.67 s | 11.56 m | 0 | 7 | 6（`gemini-3.8-flash`，0 回退） |

- **四策略配对实验**：[完整统计与 95% 区间](report/benchmark.html)，[逐任务结果、实际轨迹与失败分析](report/analysis.html)。同一 Rs 房屋的原始、拥挤、动态三种条件 × 3 组种子/初始朝向 × 4 策略，36 回合全部完成，9 组严格配对，全部失败计入统计。

| 策略 | 成功 / 全部 | 成功率 | 失败计 600 s 的平均时间 |
|---|---|---|---|
| Random | 4 / 9 | 44.4% | 393.0 s |
| Nearest | 6 / 9 | 66.7% | 307.5 s |
| Information Gain | 7 / 9 | 77.8% | 291.0 s |
| LLM-guided | 4 / 9 | 44.4% | 391.0 s |

IG 在本批成功率最高，但小样本下校正后差异不显著；结果仅覆盖一套房屋，不能证明跨房屋泛化。统计口径见[配对实验说明](Documentation/chinese/paired_benchmark.md)，设计落实核对见[审查记录](Documentation/chinese/demo_design_review.md)。

原始证据位于 `report/evidence/`（Git 忽略）：`demo/<策略>/` 是录像、事件日志、模型调用与验收记录及同源无录制试跑与录制许可；`benchmark/` 是 36 回合的事件、manifest、冻结的协议与源码快照、`results.json` 和独立复核 `audit.json`。`report/evidence/relocation.json` 记录了每个文件的来源与 SHA-256。

Git 中只跟踪四段 demo 的 `recording/`。其余原始日志（benchmark 全部回合的事件日志、manifest、冻结快照，以及各 demo 的试跑与录制许可）打包为 [report/raw-logs-20261019.zip](report/raw-logs-20261019.zip)（54 MB，含逐文件 SHA-256 清单）；在 `report/` 目录内解压即可恢复 `report/evidence/`，使统计页的逐回合链接可用。打包命令见 `scripts/package_raw_logs.py`。

## 运行

队友首次运行请先看 **[完整运行说明：环境、依赖、安装、自检、LLM、录制与打包](Documentation/chinese/running.md)**。
场景说明见[场景同步与录制记录](Documentation/chinese/updated_scene_demo.md)。

```powershell
# 在包含 pyproject.toml 的仓库根目录；已安装 Git、uv、MSVC x64 / Windows SDK
python scripts/setup_project.py --device cuda   # YOLO 用 CPU 时改为 --device cpu
./data/venv-igibson/Scripts/python.exe scripts/check_environment.py --render
python scripts/run_integrated.py --profile dynamic --policy nearest --output runs/my-first-run
```

固定环境为 Python 3.8.20 / iGibson 2.2.2 / YOLO11s，含 GPU、CPU 依赖文件及间接依赖约束。无需沿用作者的盘符或复制虚拟环境。Windows x64 已实测；Linux 启动路径受支持，原生安装与完整运行尚未在第二台 Linux 机器验证。

新运行默认写入 Git 忽略的 `runs/`。分享报告用 `scripts/package_demo_report.py`，源码与说明用 `scripts/package_source_handoff.py` 打包。

## Integrated native A/B/C + W3 pipeline

The main integration entry point is `frontier_search.runner.integrated`.
It connects native calibrated Rs/Locobot RGB-D, real YOLO11s, the shared A mapper,
B TeamNavigator, C EpisodeRunner/recovery/policies, and independent retained-frame
W3 scoring with close stationary evidence. See the
[validation gate and recording commands](Documentation/chinese/updated_scene_demo.md#复现).

```powershell
python scripts/run_integrated.py --profile dynamic --policy llm_guided --output runs/integrated-dryrun
```

This command does not record video. Recording requires a passing validation report
for the exact source/model fingerprint and inference runtime.
The [connectivity check](scripts/check_llm_connection.py) reports status without printing credentials.

## Design documents

The [English plan](Documentation/english/robotics_project_detailed_plan.md)
defines the project; [Interface Freeze](Documentation/english/interface_freeze.md)
is authoritative for behavior and ownership, with the
[close-object search extension](Documentation/english/close_object_search.md) and
[verified starting area](Documentation/english/verified_start_area.md).
Component notes: [A mapping](Documentation/english/w2_mapping.md),
[RGB-D replay](Documentation/english/w2_rgbd_replay.md),
[W3 detection](Documentation/english/w3_detection.md),
[W3 verifier](Documentation/english/w3_verifier.md),
[Locobot A/B/C handoff](Documentation/english/locobot_abc_handoff.md),
[C runner](Documentation/chinese/w2_runner.md),
[A/B delivery checklist](Documentation/chinese/w2_ab_handoff.md).
Perception capture uses **Locobot's native fixed camera**, configured by
[`configs/perception.locobot.json`](configs/perception.locobot.json).

## Run the development demo

Create the environment once, or activate an existing Python 3.8 environment:

```powershell
conda create -n igibson python=3.8
conda activate igibson
python --version
```

Then run from the repository root in PowerShell:

```powershell
$env:PYTHONPATH = "$PWD/src"
python -m frontier_search --policy nearest
python -m frontier_search --policy information_gain
python -m frontier_search --policy random --seed 7
python -m frontier_search --policy llm_guided
python -m frontier_search --fixture target_claim
python -m unittest discover -s tests -v
```

The standard authored fixture selects F1 with Nearest and F2 with Information
Gain. Random selects F1 or F2. Without a configured client, LLM-guided records
`unconfigured_client` and falls back to F1. `target_claim` produces a stop request,
not a success result. Add `--log runs/demo.jsonl` to create a new JSONL log; use a
new filename on subsequent runs.

Optionally install in a virtual environment with `python -m pip install -e .` to
use `frontier-search` without setting `PYTHONPATH`. The tested simulator and detector versions are recorded in
[screening metadata](configs/detector_screening.json); detector installation is in the
[W1 handoff](Documentation/english/w1_perception_handoff.md#usage-and-checks).
Creating this environment does not install iGibson or its assets.

Keep new code compatible with Python 3.8: use `typing.Tuple`, `typing.List`,
`typing.Optional`, and `typing.Union` for annotations instead of built-in
collection generics or `X | Y` unions.

## Layout and ownership

| Location | Owner / purpose |
| --- | --- |
| `src/frontier_search/contracts.py` | Shared immutable records and enums matching contract 1.0 |
| `src/frontier_search/config.py`, `sensors.py`, `validation.py` | Development configuration, sensor types, boundary checks |
| `src/frontier_search/perception/` | A: RGB-D occupancy/coverage, frontier geometry/identity/gain, priors and validation |
| `src/frontier_search/evaluation/api.py` | A: isolated terminal verifier interface |
| `src/frontier_search/navigation/api.py` | B: assessment, navigation, observation, cancellation interface |
| `src/frontier_search/simulation/` | Simulator adapter interface, iGibson capture and RGB-D conversion |
| `src/frontier_search/policies/` | C: three working baselines and provider-neutral LLM seam |
| `src/frontier_search/runner/` | C: validated decision boundary, runner wiring, recovery interface |
| `src/frontier_search/fixtures/` | Authored state fixtures and RGB-D recording/replay |
| `src/frontier_search/experiments/` | JSONL serialization and logging |
| `configs/experiment.template.json` | Draft freeze checklist; nulls require team decisions |
| `configs/detector_screening.json` | Detector versions, model sources and screening settings |
| `scripts/perception/` | Development capture, rendering, detector screening and scoring tools |
| `tests/` | Contract, perception, replay, scoring and selection checks |

Contract collections are tuples. Nested mutable values are rejected rather than
being treated as immutable just because a dataclass is frozen. Occupancy uses
`-1 / 0 / 100`; geometric coverage is separate. Policy state has no evaluator
handle or hidden scene records. Standard-run room labels must remain `None`.

## Implementation points

See [the implementation checklist](Documentation/implementation_checklist.md) for
the work still needed by A, B, and C. Protocols define the replacement boundaries;
synthetic fixture metadata is not an extra policy feature.

`EpisodeRunner.run()` accepts the validated `EpisodeManifest` in `runner/manifest.py`.
`configs/w2.mock.json` is executable development configuration; the older
`experiment.template.json` remains an experiment checklist. The runner checks
sensor/frame/geometry/version coherence and scan barriers. Real robot, sensor and
mapper acceptance still requires the A/B checks in the design documents.

The LLM seam accepts an injected client and records exact symbolic input, response,
model settings, latency, and fallback. Its client must enforce transport deadlines;
independent process-level supervision of stalled modules belongs in the full
runner. The opt-in provider adapter enforces HTTP deadlines using a killable
subprocess and records raw responses and provider usage. Pricing/cost remains
unknown. Credentials are loaded locally and are not committed or logged.

## Simulator resources

The project-local iGibson runtime is `data/venv-igibson/Scripts/python.exe`.
The existing scan is `data/igibson-data/g_dataset/Rs/mesh_z_up.obj`; robot and YCB
models are under `data/igibson-data/robot_assets/assets/models/`. No licensed
interactive-scene dataset is required for this demo.

```powershell
python scripts/setup_igibson.py # Only when the runtime/assets are absent.
```

The optional [verified starting-area handoff](Documentation/english/verified_start_area.md)
describes A's validated initialization/replay support and B's remaining physical
clearance and floor-support checks. Footprint-only initialization stays the default.
