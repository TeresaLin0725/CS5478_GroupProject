# Reproducing and reviewing the submission

## Inspect the committed evidence

The [experiment index](../report/README.md) links to the final 48-episode bowl benchmark, the original 45/48 suite, semantic study, robustness summary, and midterm baseline. The [matched rationale archive](../report/llm-thinking-matched-20261119-01/README.md) is a separate run, not a replacement for the final benchmark's call logs.

No simulator or API key is needed to read these results. Download the committed [midterm videos](../midterm-report-20261025/README.md) to play them, or clone the repository and open `midterm-report-20261025/demo/report/index.html`. GitHub displays HTML source rather than running the interactive viewer.

## Run a new bowl experiment

First follow the [installation instructions](chinese/running.md). All commands below run from the repository root on Windows using the installed Python 3.8 environment. On Linux, substitute `data/venv-igibson/bin/python` for `data/venv-igibson/Scripts/python.exe`; full Linux execution was not independently validated.

For one new baseline episode with the robust bowl mapping settings:

```powershell
python scripts/run_integrated.py --task kitchen_bowl --profile apartment --policy information_gain --noise --depth-noise-std-m .01 --depth-dropout-prob .01 --noise-seed 140 --initialization-deadline-s 240 --output runs/review-kitchen-ig
```

Use `--task bathroom_bowl` to change the placement, and `--policy nearest`, `random`, or `llm_guided` to change the selector. An output directory must be fresh. This single run uses the current source and its configured seeds; it does not recreate all historical manifests.

To freeze and run a fresh 48-episode paired suite, first create a root `.env` file from `.env.example` and configure your own provider credentials and model. The harness requires an environment file and a regression certificate matching the current source and weights:

```powershell
$env:PYTHONPATH = "$PWD/src"
./data/venv-igibson/Scripts/python.exe scripts/run_integrated_checks.py --output runs/review-checks/tests.log
./data/venv-igibson/Scripts/python.exe scripts/run_paired_benchmark.py all --output runs/review-bowl48 --workers 2 --task kitchen_bowl --design bowl48 --tests-certificate runs/review-checks/tests.json --noise --depth-noise-std-m .01 --depth-dropout-prob .01 --noise-seed 123
```

The suite assigns noise seeds per job: base noise seed 123 plus environment seed 17 gives actual noise seed 140. It creates new manifests, a frozen source/harness snapshot, asset hashes and local paths in the new output directory. This command executes all four policies, including hosted LLM calls. A two-worker run shares CPU/GPU resources; its wall times are not isolated policy timings. Hosted model sampling and availability cannot be reproduced from local seeds alone.

For baseline-only execution, add `--only-baselines` to the suite command. Such a run does not reproduce the LLM arm or the complete 48-episode result. Inspect fallback records when assessing any new LLM run.

## Run the semantic configurations

The original study configurations are `configs/semantic-navigation-start-S0.json` through `S3.json`; `semantic-final-start-S*.json` describe a separate follow-up. To run one configured start:

```powershell
$env:PYTHONPATH = "$PWD/src"
./data/venv-igibson/Scripts/python.exe scripts/check_semantic_targets.py --config configs/semantic-navigation-start-S0.json --output runs/review-semantic-S0/preflight
```

The semantic runner's quota probe loads the historical parent-directory `.env` by default. This command loads the repository-root `.env` into the process environment so both the probe and workers use that configuration:

```powershell
./data/venv-igibson/Scripts/python.exe -c "import os, subprocess, sys; sys.path.insert(0, 'src'); from frontier_search.policies.provider import load_local_env; load_local_env('.env'); raise SystemExit(subprocess.call([sys.executable, 'scripts/run_semantic_experiments.py', 'all', '--config', 'configs/semantic-navigation-start-S0.json', '--output', 'runs/review-semantic-S0', '--preflight', 'runs/review-semantic-S0/preflight/preflight.json', '--workers', '2'], env=os.environ.copy()))"
```

Supply a model available to your account; the configuration and logs must identify any departure from the historical model. The preflight must pass before the experiment can be planned or executed. Repeat with S1–S3 and distinct output directories for the other starts.

## Historical reproduction limits

The final bowl archive identifies commit `296fa0d`, its original source/weight fingerprint, manifests and exact harness scripts. It omits the frozen source copy, simulator assets, model weights, credentials and most raw frame/event logs. Original protocol JSON retains author-machine paths and byte-sensitive snapshot seals. Do not feed it directly to the current harness or edit the preserved archive to fit a new machine. To investigate that historical implementation, use a separate checkout of the recorded commit, install its documented dependencies and generate fresh local protocols/certificates. A fresh current-source run is a replication with the same experiment design, not a byte-identical replay of the historical run.

Some analysis utilities are author-side export tools:

| Tool | Required inputs beyond the clone |
|---|---|
| `scripts/make_paper_figures.py` | Original `runs/bowl-llm-models-20261117/` CSV and per-episode trajectories |
| `scripts/analyze_bowl_llm_models.py`, `scripts/analyze_bowl_llm_cot.py` | Model-comparison episode call logs under their selected `--runs` root |
| `scripts/analyze_semantic_replicates.py` | Original semantic run cohorts; its S0 defaults reference historical local directories |
| `scripts/archive_bowl48_report.py`, `scripts/archive_semantic_report.py` | Completed local runs to export into a report archive |

Their exported figures/reports are committed where listed in the experiment index, but original raw inputs for every model-comparison figure are not all included. The committed rationale archive contains compact returned-answer evidence rather than full prompts or raw transport logs. It cannot be used as a complete model-comparison dataset. The repository therefore supports inspecting the archived results and running fresh experiments, with these limits on exact historical replay and figure regeneration.

## Validation and unchanged evidence

Run `scripts/run_integrated_checks.py` in the installed environment for the complete unittest regression suite and a fresh fingerprint certificate. Historical test logs and hash manifests remain intact. Simulator checks and hosted calls require the documented assets and account access; the lightweight CLI fixture alone does not validate full simulation.
