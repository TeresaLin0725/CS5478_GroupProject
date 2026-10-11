# Indoor frontier object search

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
tools, B's navigation implementation, and C's optional synthetic/PyBullet/iGibson
demos. The shared A/B/C RGB-D episode loop is still a W2 deliverable; the existing
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

## PyBullet 3D map demo

A separate optional demo now builds a procedural 12 m × 8 m scene with three
rooms, doorways, furniture and a mug. The robot physically moves in PyBullet;
partial occupancy comes from ray intersections, candidates from observed/unknown
boundaries, and paths from the current footprint-inflated known-free map.
Movement feedback records actual poses, travelled distance and obstacle contacts.

This is an explicitly named **range-sensor / oracle-visible-class ablation**:
semantic labels and positions are obtained only for objects visible in rendered
camera segmentation. It is not the planned RGB-D/YOLO pipeline, not an iGibson
asset, and not a physical robot trial. Full scene geometry is not passed to C.
The robot is a velocity-actuated holonomic cylinder; scans use a virtual panoramic
camera. Production recovery and wheel dynamics remain out of scope.

```powershell
py -3.10 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements-pybullet.txt
.\scripts\run_pybullet_demo.ps1 -Policy nearest
.\scripts\run_pybullet_demo.ps1 -Policy llm_guided
# Visible interactive scene, with movement paced in real time:
.\scripts\run_pybullet_demo.ps1 -Policy nearest -Gui
```

On this workspace the virtual environment is already installed. Each run writes
an animated GIF, initial/final PNGs, detailed JSONL and a summary under a new
`runs/pybullet-*` directory. DIRECT mode renders without opening a desktop window.
Add `-Blocked` to close the target room door; failure to find the target is then
the expected negative-test outcome. See [3D demo notes](Documentation/chinese/pybullet_demo.md).

## iGibson Rs demo

The same Nearest / live Gemini loop now also runs in **iGibson 2.2.2**, using the
public Gibson `Rs` apartment scan and iGibson's differential-drive Locobot. The
robot moves only through wheel velocity commands. The occupancy map is fused from
rendered depth, and the YCB target is detected through rendered instance
segmentation. Furniture classes are hand-annotated scan landmarks, reported only
when rendered depth confirms they are visible (a landmark oracle, not YOLO).

```powershell
python scripts/setup_igibson.py          # once: source build + Python 3.8 env + assets
.\scripts\run_igibson_demo.ps1 -Policy both
```

Measured on 2026-09-30, from the same start state:

| Policy | Result | Decisions | Distance | Collisions |
| --- | --- | ---: | ---: | ---: |
| Nearest | success | 5 | 12.1 m | 0 |
| Gemini | success | 1 | 4.0 m | 0 |

This is one scene and one start pose, so it is not a statistical comparison.
Details, two Windows build fixes for upstream iGibson, and limitations are in
[iGibson demo notes](Documentation/chinese/igibson_demo.md). Animations and
per-decision records are in `Documentation/igibson_demo/`.
