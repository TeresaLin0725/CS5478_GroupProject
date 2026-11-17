# Indoor frontier object search

Latest completed benchmark: [fixed 48-episode bowl results and LLM comparisons](report/bowl-48-fixed-20261116-01/README.md) — 48/48 successes, zero collisions, with configuration, verification and selected evidence.

## 保留的实验基线

仓库只保留 [2026-10-25 中期报告与 Demo](midterm-report-20261025/README.md)：
36 个配对回合中 29 个成功、7 个 `no_feasible_frontiers`。该目录包含中英文 PDF、
四策略录像、逐回合汇总、冻结协议和报告所需证据。其他历史运行目录与单案例验证报告不再保留；
新实验必须写入新的 Git 忽略目录，不得覆盖这份基线。

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
The authoritative hosted model for new LLM-guided runs is `gemini-3.5-flash`.

Depth noise and missing depth pixels are opt-in: add `--noise`, or use
`--no-noise` for a clean control. See [sensor noise configuration and the pilot
experiment](Documentation/english/sensor_noise.md) for CLI/JSON settings and frozen benchmarks.

The opt-in `--task kitchen_bowl` places a bowl on the kitchen counter and starts
at the original living-room position `(0.8, -2.4)`, with the original full 360° initial scan.
Run `python scripts/check_bowl_setup.py --output runs/bowl-setup-new` in the
iGibson environment to check placement and the fixed start without searching.
See the [placement and setup commands](Documentation/chinese/semantic_target_check.md#厨房碗与卧室旁起点完整一圈初始观察).

For an active bowl pilot, `python scripts/serve_bowl_report.py --run-root <run-directory>`
serves a read-only live report at `http://127.0.0.1:8910/`. It updates every two
seconds from the episode logs, with policy status, actual trajectories, the latest
perception map, decisions, collisions, and terminal results. Authorized LLM trials
can be queued after the local baselines with `scripts/queue_bowl_llm.py` using
the same frozen source snapshot and experiment settings.

The completed bowl benchmark is `runs/bowl-48-noise-20261110-02`: two starts
(A original living room, B bedroom doorway), three scene conditions, four policies,
and two bowl locations. All 24 kitchen episodes run before the 24 bathroom episodes.
Noise remains enabled (0.01 m depth standard deviation, 1% dropout, actual seed 140),
with the validated 5x5 depth filter, recovery viewpoints and 0.5 inspection settling.
The new layout moves the former doorway chair into the bedroom for both target
locations, since isolated full-footprint checks found it otherwise blocked the
bathroom. All other furnishings and legacy layouts remain unchanged. The initial
unexecuted setup is preserved in `runs/bowl-48-noise-20261110-01`.
There are at most two concurrent simulators. The frozen protocol contains 48
paired manifests and 12 isolated target/start/condition feasibility prechecks;
the source passed 663 regression tests before freezing. No completed outcome is
retried. The live report is `http://127.0.0.1:8910/?batch=48`, with a correct start
and target marker for each selected run and automatic per-episode trajectory PNGs.

All 48 episodes completed: 45 successes and no collisions. Information Gain and
LLM-guided both succeeded in 12/12 cases. LLM was more efficient for the kitchen
bowl; IG was more efficient for the bathroom bowl. See the
[Chinese report and portable evidence archive](report/bowl-48-noise-20261110-02/README.md)
for time, path length, paired comparisons, all 48 results, and four trajectory
comparison figures covering both targets and starts. The HTML report embeds its
five figures; its 48 linked WebP previews are included in the archive.

Scene-aware images are saved separately as `trajectory-scene.png`: run
`scripts/render_all_scene_floorplans.py --run-root <run-directory>` with the
iGibson Python, then `scripts/export_run_trajectories.py --run-root <run-directory>
--scene-overlays`. They combine the original scan with native rendered inserted
objects at their initial poses; the dynamic cart can move later. The live report
shows these layers and prefers the scene-aware PNG when available.

The preceding 36-run bowl benchmark used three scene conditions, three seed blocks,
and four policies at one fixed start. Its completed batch is
`runs/bowl-36-noise-20261110-02`. Depth noise was 0.01 m standard deviation with
1% dropout; noise seeds 140, 152, and 166 are matched across policies in each block.
It retains the earlier 35/36 suite's 5x5 range-gated depth filter, unvisited
recovery viewpoints, and 0.5 inspection settling ratio. The previous bowl batch
`runs/bowl-36-noise-20261109-01` was stopped and retained as diagnostic data after
its manifest was found to omit these settings. Its outcomes are excluded from
the corrected batch. Explicit simulation-payload authorization for Google Gemini
was received on 10 November. At the user's request, the batch now runs up to
two cases concurrently: one local baseline and one LLM case. The runtime change
and initially sequential cases are recorded in `execution-amendment.json`;
wall times after that change reflect shared CPU/GPU use.

`python scripts/export_run_trajectories.py --run-root <run-directory> --watch`
saves `episodes/<job_id>/trajectory.png` after each run, including failures and
interrupted runs. Images show the reference floorplan, actual recorded path and
direction arrows, green start, red target, and final heading marker. Metadata is
saved beside each PNG; the live report offers per-run PNG downloads. This exporter
runs separately from the frozen benchmark and should be restarted when resuming
after its process has stopped.

Mapping-side robustness options are independent of injected noise: see
[depth filtering](Documentation/english/depth_filter.md) and
[unvisited recovery viewpoints](Documentation/english/recovery_viewpoints.md), plus
[inspection settling](Documentation/english/inspection_settling.md).
The [same-version clean/noisy results](Documentation/english/noise_experiment_results.md)
record 35/36 successes in each arm, using `gemini-3.5-flash-lite` for LLM-guided cases.

## Design documents

The [English plan](Documentation/english/robotics_project_detailed_plan.md)
defines the project; [Interface Freeze](Documentation/english/interface_freeze.md)
is authoritative for behavior and ownership, with the
[close-object search extension](Documentation/english/close_object_search.md) and
[verified starting area](Documentation/english/verified_start_area.md).
Component notes: [A mapping](Documentation/english/w2_mapping.md),
[fusion profiling and evidence baseline](Documentation/chinese/fusion_profiling.md),
[RGB-D replay](Documentation/english/w2_rgbd_replay.md),
[W3 detection](Documentation/english/w3_detection.md),
[W3 verifier](Documentation/english/w3_verifier.md),
[Locobot A/B/C handoff](Documentation/english/locobot_abc_handoff.md),
[C runner](Documentation/chinese/w2_runner.md),
[C optimization handoff](Documentation/chinese/c_optimization_handoff.md),
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
