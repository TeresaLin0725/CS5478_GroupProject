# Installation and running

Run commands from the directory containing `pyproject.toml`. The tested native environment is Windows x64, Python 3.8.20, iGibson 2.2.2 and YOLO11s. Linux launch paths are supported, but a full independent Linux reproduction has not been validated. CPU detector inference still requires graphics support for the simulator.

## Windows installation

Install Git, uv, Visual Studio 2022 Build Tools with the C++ desktop workload and Windows SDK, and a working OpenGL graphics driver. Internet access is required for source, dependency, scene and model downloads. Allow approximately 15 GB of free disk space; 16 GB RAM is recommended rather than a measured minimum.

```powershell
git clone --branch main https://github.com/TeresaLin0725/CS5478_GroupProject.git
cd CS5478_GroupProject
python scripts/setup_project.py --device cuda
```

Use `--device cpu` for CPU detector inference. The installer creates `data/venv-igibson/` with Python 3.8.20, builds iGibson revision `7d9598278b95ec1039e1f4366c66b45346521e44`, downloads the public Rs scan and robot/YCB assets, installs the project, and downloads checksum-verified YOLO11s weights. It uses the committed requirements and constraints files. System Python launches the installer; it is not the simulation environment. Use the VS x64 Native Tools command prompt if the compiler is not found.

```powershell
./data/venv-igibson/Scripts/python.exe scripts/check_environment.py --render
python scripts/run_integrated.py --profile dynamic --policy nearest --output runs/my-first-run
```

Supported policies are `random`, `nearest`, `information_gain` and `llm_guided`; scene conditions are `apartment`, `clutter` and `dynamic`. The default task searches for a cup. Add `--task kitchen_bowl` or `--task bathroom_bowl` for bowl placement. Use `--noise` to enable depth noise, or `--no-noise` for a clean run. New outputs require a fresh directory under ignored `runs/`.

The launcher selects the local native environment automatically. Set `FRONTIER_PYTHON` to another compatible interpreter if needed. Do not copy a virtual environment from another computer.

## Hosted LLM configuration

```powershell
Copy-Item .env.example .env
# Edit .env with your own API key, compatible endpoint and accessible model.
python scripts/run_integrated.py --profile dynamic --policy llm_guided --env-file .env --output runs/my-llm-run
```

`LLM_API_KEY`, `LLM_BASE_URL` and `LLM_MODEL` configure the client. Existing process environment variables take precedence. Historical model identifiers do not guarantee access for another account. The client sends symbolic candidate/observation state rather than image or depth arrays. Missing configuration, quota failures, invalid responses and timeouts are logged as fallbacks. Such calls must not be counted as successful model decisions.

The root `.env`, `data/`, `weights/` and new `runs/` are ignored. Keep credentials out of submitted files. The standalone connectivity check accepts `--env-file .env` and makes a live provider request:

```powershell
./data/venv-igibson/Scripts/python.exe scripts/check_llm_connection.py --env-file .env --output runs/llm-connection.json
```

## Validation and video recording

A non-recording run and a passing regression certificate are required before recording. The validation checks source, model, detector runtime and effective configuration.

```powershell
python scripts/run_integrated.py --profile dynamic --policy nearest --output runs/my-demo/dryrun
./data/venv-igibson/Scripts/python.exe scripts/run_integrated_checks.py --output runs/my-demo/checks/tests.log
./data/venv-igibson/Scripts/python.exe scripts/validate_integrated.py --run runs/my-demo/dryrun --tests runs/my-demo/checks/tests.log --output runs/my-demo/validation.json
python scripts/run_integrated.py --profile dynamic --policy nearest --record --validation runs/my-demo/validation.json --output runs/my-demo/recording
./data/venv-igibson/Scripts/python.exe scripts/build_demo_report.py --recording runs/my-demo/recording --output runs/my-demo/report
```

Open the generated `report/index.html` locally. GitHub does not execute an HTML file from its source-code view. Videos are recorded in simulation time; playback speed does not change the recorded performance metrics.

## Linux reference setup

Prepare a Python 3.8.20 environment, a C++ toolchain, CMake, EGL/OpenGL and graphics drivers. Install the pinned requirements for the chosen Torch device and build the recorded iGibson revision. Apply the texture-buffer change from `scripts/setup_igibson.py`; its MSVC-specific build changes are Windows-only. Download assets with `scripts/download_igibson_demo.py --root data/igibson-data`. YOLO11s must match the URL and SHA-256 in `scripts/setup_project.py`.

Place the environment at `data/venv-igibson/` or set `FRONTIER_PYTHON`. Replace Windows `Scripts/python.exe` paths with `bin/python`. The Windows installer intentionally refuses to build on other platforms; macOS and headless/software-only rendering are not validated configurations.

## Lightweight fixture

```powershell
$env:PYTHONPATH = "$PWD/src"
python -m frontier_search --policy information_gain
```

This authored-state demo needs no simulator or API key. It checks policy wiring and is not a benchmark simulation. Full regression and simulation checks require the documented dependencies and assets.
