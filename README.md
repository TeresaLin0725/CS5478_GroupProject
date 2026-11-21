# Indoor frontier object search

CS5478 group project: compare Random, Nearest, Information Gain (IG), and LLM-guided frontier selection in an iGibson Rs apartment. All policies share RGB-D mapping, YOLO11s detection, navigation, recovery, and independent target verification. The LLM receives symbolic observations and candidate metrics; hidden target positions are reserved for evaluation.

## Results and demonstration

The **final bowl benchmark achieved 48/48 verified successes with zero collision/contact ticks**. Each policy completed 12 episodes: two target locations, two starts, and three scene conditions. LLM guidance was more efficient than IG for the kitchen bowl; IG was more efficient for the bathroom bowl. These are fixed tasks in one house, not evidence of generalization to other houses.

| Start here | Contents |
|---|---|
| [Final bowl results](report/bowl-48-fixed-20261116-01/README.md) | All 48 results, manifests, verification, selected RGB evidence and trajectories |
| [Experiment index](report/README.md) | Final results and clearly separated historical baselines |
| [Paper figures](report/paper-figures/) | Committed PNG/PDF figures, viewable without running the simulator |
| [LLM decision rationales](report/llm-thinking-matched-20261119-01/README.md) | Separate 12-episode matched run; 36 valid answers and two fallback calls |
| [Midterm report (English)](midterm-report-20261025/report/Midterm-Report-EN.pdf) | Historical baseline report; 29/36 successes |
| [Midterm report (Chinese)](midterm-report-20261025/report/中期报告-中文.pdf) | Same historical stage in Chinese |

Four **historical midterm** demonstration videos are included in the repository: [Random](midterm-report-20261025/demo/random/recording/integrated.mp4), [Nearest](midterm-report-20261025/demo/nearest/recording/integrated.mp4), [IG](midterm-report-20261025/demo/information_gain/recording/integrated.mp4), and [LLM-guided](midterm-report-20261025/demo/llm_guided/recording/integrated.mp4). Use GitHub's raw/download control if a video does not preview. After cloning, open [the local demo viewer](midterm-report-20261025/demo/report/index.html) in a browser; GitHub's file view does not execute the HTML player. These videos are not recordings of the final 48-episode benchmark.

## Install and run

Run commands from the repository root. The tested full simulation environment is **Windows x64, Python 3.8.20, iGibson 2.2.2 and YOLO11s**. Installation requires Git, uv, Visual Studio 2022 C++ Build Tools with the Windows SDK, working OpenGL graphics, and network access. The CPU option changes detector inference; the simulator still needs graphics support.

```powershell
python scripts/setup_project.py --device cuda
# For CPU detector inference, use --device cpu instead.
./data/venv-igibson/Scripts/python.exe scripts/check_environment.py --render
python scripts/run_integrated.py --profile dynamic --policy nearest --output runs/my-first-run
```

The installer creates the pinned Python environment, builds iGibson, downloads the public Rs/robot/YCB assets, installs dependencies and downloads checksum-verified YOLO11s weights. Assets, environments and weights are not committed. See [full installation instructions (Chinese)](Documentation/chinese/running.md) for prerequisites, Linux reference steps, recording and troubleshooting. Linux end-to-end execution has not been independently validated for this project.

A lightweight authored-fixture demo runs without simulator assets or an API key:

```powershell
$env:PYTHONPATH = "$PWD/src"
python -m frontier_search --policy information_gain
```

This fixture checks policy wiring; it is not a simulated benchmark episode.

## Optional hosted LLM

```powershell
Copy-Item .env.example .env
# Edit .env: set LLM_API_KEY, LLM_BASE_URL and a model available to your account.
python scripts/run_integrated.py --profile dynamic --policy llm_guided --env-file .env --output runs/my-llm-run
```

Use your own provider account and quota. Historical model identifiers and access are not guaranteed for other accounts. If no client is configured or a request fails, the policy records a fallback; a fallback episode must not be presented as successful hosted-model execution. Credentials stay in the ignored `.env` file. Live requests send symbolic simulation state rather than RGB images or depth arrays. The archived Gemini bowl benchmark and DeepSeek semantic study used different experimental settings; see their protocols before comparing them.

## Reproduce and validate

[Reproduction instructions](Documentation/reproducing.md) distinguish inspecting the committed evidence, running a fresh experiment, and reconstructing a historical frozen experiment. New runs must use fresh output directories. Archived protocols contain original machine paths and must not be executed unchanged.

In the installed simulation environment:

```powershell
./data/venv-igibson/Scripts/python.exe scripts/run_integrated_checks.py --output runs/submission-checks/tests.log
```

This produces the test log and a source/weight fingerprint certificate at `runs/submission-checks/tests.json`. The final frozen bowl run's historical certificate records 677 tests, six skipped and no failures; the later merged integration certificate records 689 tests. Those certificates describe their recorded source versions, not a new test execution on your machine.

## Repository map

| Path | Purpose |
|---|---|
| `src/frontier_search/perception/` | RGB-D mapping, semantic objects, frontier geometry and information gain |
| `src/frontier_search/navigation/` | Shared path assessment, navigation and observation |
| `src/frontier_search/policies/` | Baseline selectors and hosted LLM adapter |
| `src/frontier_search/runner/` | Episode orchestration, recovery and recording |
| `src/frontier_search/evaluation/` | Independent retained-frame verification |
| `scripts/`, `configs/` | Installation, experiments, analysis and configuration |
| `tests/` | Regression tests |
| `report/` | Committed experiment evidence and paper figures |
| `midterm-report-20261025/` | Preserved historical reports, videos and baseline data |
| [Documentation](Documentation/README.md) | Runtime and architecture guide; development history is marked separately |

Python source remains compatible with Python 3.8. New outputs belong in ignored `runs/`; `data/`, `weights/`, virtual environments and `.env` are local dependencies, not submission files.
