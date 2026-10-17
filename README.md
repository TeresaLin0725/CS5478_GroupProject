# Indoor frontier object search

## 演示报告

打开[室内实体机器人目标搜索演示报告](runs/integrated-llm-final-20261018/recording/index.html)。
报告收录 2 项完整集成管线录制和 12 项场景与策略演示，可切换画面，点击决策、操作及验收记录的时间定位播放。
其中 3 项为每秒 15 帧连续视频，11 项为按仿真时间展示的采样画面。
在线大语言模型完整管线本次运行完成目标验收：仿真时间 204.03 秒，实际计算耗时 664.38 秒，行驶 10.86 米，无物理接触，真实在线模型调用 4 次。
报告如实列出导航失败、观察受阻、恢复过程及预置回答回放的执行范围。
运行 `data/venv-igibson/Scripts/python.exe scripts/build_demo_report.py` 可根据保存的日志重新生成报告；原始媒体与日志保持不变。

## Integrated native A/B/C + W3 pipeline (2026-10-18)

The main integration entry point is now `frontier_search.runner.integrated`.
Open [the verified live LLM recording](runs/integrated-llm-final-20261018/recording/index.html).
It connects native calibrated Rs/Locobot RGB-D, real YOLO11s, the shared A mapper,
B TeamNavigator, C EpisodeRunner/recovery/policies, and independent retained-frame
W3 scoring with close stationary evidence. It also handles known-free viewpoint
recovery when a departed obstacle remains hidden in the camera's near blind area.
See the [full pipeline, validation gate and recording commands](Documentation/chinese/integrated_pipeline.md).

```powershell
python scripts/run_integrated.py --profile dynamic --policy llm_guided --output runs/integrated-dryrun
```

This command does not record video. Recording requires a passing validation report
for the exact source/model fingerprint and inference runtime. The configured API
has recovered from HTTP 402 and now serves real `gemini-3.8-flash` responses.
The LLM recording passed 367 regression tests and a successful non-recording
acceptance run before recording: 204.03 simulated seconds, 10.86 m,
zero contacts, two close stationary frames accepted by the independent W3 verifier.
The recording made four fresh hosted model calls with zero fallbacks. All 3,062
H.264 frames decode; the 3,061 frames after initialization match poses from physical
simulation steps. The [LLM verification](runs/integrated-llm-final-20261018/recording/llm-verification.json)
checks that logged inputs match requests and every returned ID matches the dispatched goal.
Two local replans and one hosted decision receiving B's failure feedback were
observed in the recording. The viewer includes actual model reasons and API latency.
Arrival observations now cover a full rotation to reduce missed corridor views.
The [earlier Nearest recording](runs/integrated-final-20261017/recording/index.html)
uses the previous narrower scan configuration and is retained as a historical reference.
The [connectivity check](scripts/check_llm_connection.py) reports status without printing credentials.
The standalone recordings below are historical and use a separate demonstration loop.

## Historical standalone iGibson demo

Open [the previous standalone interactive demo](runs/igibson-realistic-final-20261014/index-legacy-before-integration-20261017.html).
It uses the existing textured **Gibson Rs apartment**, the physical **Locobot**
URDF and native iGibson rendering. Three configurations cover the original
apartment, furnished narrow routes, and a moving cart. All four policy conditions
use measured wheel-driven trajectories, RGB-D mapping and close-range inspection.
The default dynamic/Nearest recording now plays as continuous 15 fps H.264 video
with a stable tracking camera. Other historical conditions remain labeled as
sparse image recordings. New runs record 15 fps by default (`--video-fps 0` disables it).
The [W3 merge and feedback audit](Documentation/chinese/w3_merge_and_demo.md)
records the four upstream commits, preserved local changes, and navigation feedback scope.

```powershell
cd integration # When starting at D:/CS5478_PROJECT.
$env:PYTHONPATH = "$PWD/src"
./data/venv-igibson/Scripts/python.exe -m frontier_search.runner.igibson_demo --scenario suite --policy all --backend replay
```

The PowerShell convenience launcher is also available when script execution is enabled:

```powershell
./scripts/run_igibson_demo.ps1 -Scenario suite -Policy all -Backend replay
# One hard scene:
./scripts/run_igibson_demo.ps1 -Scenario dynamic -Policy nearest
# Actual model calls, using local .env configuration:
./scripts/run_igibson_demo.ps1 -Scenario dynamic -Policy llm_guided -Backend live
```

The offline LLM condition replays authored responses derived from the same
scenario's Information Gain run. It is labeled in the viewer and logs.
See [scene resources, difficulty and verification](Documentation/chinese/igibson_demo.md).
Old standalone PyBullet demos and their launch scripts have been removed;
previous recordings are archived under `runs/archive-pybullet/`.
The shared A/B/C production integration has its own acceptance criteria below.

The sections below record the earlier scaffold and integration history.

Python skeleton for comparing Random, Nearest, Information Gain, and LLM-guided
frontier selection. The [English plan](Documentation/english/robotics_project_detailed_plan.md)
defines the project; [Interface Freeze 1.0](Documentation/english/interface_freeze.md)
is authoritative for behavior and ownership.

## W2 integration status (2026-10-07)

`integration/w2` merges A `ab52cf4`, B `224bfcf`, and C `8f817f8` with their
history preserved. See the [integration report and W2 plan](Documentation/chinese/w2_integration_status.md)
for ownership, configuration differences, test results and acceptance gates.
C's `EpisodeRunner.run()` now implements the shared lifecycle, scan barriers,
spatial recovery, budgets and logs, with a process supervisor for hard deadlines.
The runnable fixture connects B's actual TeamNavigator to an explicitly scripted
A test double. A's `RGBDMapper.update()` now publishes occupancy, geometric
coverage and frontiers with approach poses, stable identity/lineage and predicted
gain. B's real `IGibsonAdapter`, real recordings/configuration and factory wiring
remain pending. See [A's W2 mapping implementation](Documentation/english/w2_mapping.md)
for the evidence model, synthetic A → B → C checks and remaining integration work.
The [RGB-D sequence replay](Documentation/english/w2_rgbd_replay.md) now provides
a portable synthetic moving/scanning dataset, checksum-validated packaging for
existing captures, and repeat-verified A → B assessment → C decisions with HTML
and map reports. It does not execute the selected goals.
See [C runner usage](Documentation/chinese/w2_runner.md) and the
[A/B delivery checklist and acceptance criteria](Documentation/chinese/w2_ab_handoff.md).

Perception capture uses **Locobot's native fixed camera**, configured by
[`configs/perception.locobot.json`](configs/perception.locobot.json).
See the [capture instructions](Documentation/english/development_fixtures.md#real-igibson-capture-and-replay)
and [native-camera visibility check](Documentation/english/locobot_camera_check.md).
For B/C Codex sessions, start with the [Locobot A/B/C handoff](Documentation/english/locobot_abc_handoff.md),
which records the selected settings, owner tasks, reproduction commands and remaining gates.
The [continuous Locobot recording guide](Documentation/english/locobot_multiframe.md)
documents real multiframe replay results and the remaining clearance/goal acceptance gap.

```powershell
$env:PYTHONPATH = "$PWD/src"
python -m frontier_search.runner.w2 --manifest configs/w2.mock.json --log runs/w2-progress-001.jsonl
```

Use a new log filename each time. This runs offline with the standard library.
The progress fixture dispatches two goals, travels about 1.80 m without collision,
and terminates as `no_frontiers`; it never claims object-search success.
All 153 tests pass on Python 3.8.20 with no skips. Measured supervised scenarios
are recorded in [verification.json](Documentation/w2_runner/verification.json).

The default scaffold runs synthetic **selection decisions**, without simulator or
network dependencies. The integration branch also contains A's W1 perception
tools, B's navigation implementation, and C's synthetic fixtures and native iGibson demo. The shared A/B/C RGB-D episode loop is still a W2 deliverable; the existing
C demos use their own mapper/navigation implementations. Use Python 3.8 for the
shared iGibson environment, matching its
[documented installation setup](https://stanfordvl.github.io/iGibson/installation.html).

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
mapper acceptance still requires the A/B checks linked above.

The LLM seam accepts an injected client and records exact symbolic input, response,
model settings, latency, and fallback. Its client must enforce transport deadlines;
independent process-level supervision of stalled modules belongs in the full
runner. The opt-in provider adapter enforces HTTP deadlines using a killable
subprocess and records raw responses and provider usage. Pricing/cost remains
unknown. Credentials are loaded locally and are not committed or logged.

## C integration: nearest and Gemini on one synthetic map

The review and detailed limitations are in
[C integration notes](Documentation/chinese/c_integration_review.md).

```powershell
$env:PYTHONPATH = "$PWD/src"
python -m frontier_search.runner.synthetic --log runs/three-rooms-replay.jsonl
```

This runs Nearest (F1 → F2 → F3) and an explicitly authored LLM replay (F3).
Both execute 9 m in the synthetic point-robot fixture. Nearest uses **B-assessed
path length**, not Euclidean distance. The immutable state, eligibility checks,
and policy interface are shared with the original scaffold. A terminal claim is
scored by a separate fixture evaluator before reporting synthetic success.

For a live call, copy `.env.example` to `.env` and fill `LLM_API_KEY`, or pass
`--env-file` to an existing local file. Process environment variables take
precedence. In the original workspace, `--env-file ../.env` reuses the existing
key without copying it into this checkout.

```powershell
python -m frontier_search.runner.synthetic --backend live --env-file ../.env --log runs/three-rooms-live.jsonl
python -m unittest discover -s tests -v
```

Use a new log filename for each run. The live smoke command exits nonzero if
fallback was needed, even when the Nearest fallback completed the search. Default
LLM deadline is 10 s and call cap is 10; hosted-model selection is not deterministic.
JSONL records use `fixture_*` event names and `source=synthetic` to separate them
from real experiments. The default `python -m frontier_search` selection demo is
unchanged and does not make network calls.

Verified locally with Python 3.10: original and integration tests; replay and live
Gemini cases. The full test suite also passes under the Python 3.8 iGibson runtime
(see below). This older synthetic loop remains separate from the new W2 runner;
use the W2 entry point above to exercise the shared B navigator and lifecycle.

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
