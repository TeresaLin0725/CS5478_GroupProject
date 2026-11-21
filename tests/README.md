# Tests

Run from the repository root after installing the documented environment and optional test dependencies:

```powershell
uv pip install --python ./data/venv-igibson/Scripts/python.exe -e ".[test]"
./data/venv-igibson/Scripts/python.exe -m pytest
```

| Directory | Scope | Run separately |
|---|---|---|
| `unit/` | Contracts, policy selection, mapping, geometry, recovery and target verification | `python -m pytest tests/unit` |
| `integration/` | Component wiring, RGB-D replay, native adapters, subprocess supervision and report exports | `python -m pytest tests/integration` |
| `experiments/` | Paired manifests, bowl/semantic task settings and preflight consistency | `python -m pytest tests/experiments` |

Commands in the last column assume the configured project environment is active. Pytest collects only this test tree, not downloaded dependencies or ignored run snapshots. Most tests use synthetic observations, recorded fixtures or mocked providers and require no external model calls. Native motion checks require iGibson and its assets; unavailable optional runtimes are reported as skips.

`fixtures/` contains shared test inputs. `rgbd_support.py`, `pose_noise_study.py` and the `reference_*.py` modules support deterministic geometry and differential checks. Reference implementations are intentionally independent of the optimized runtime implementation.

For a source/weight fingerprint certificate used by experiment freezing and video validation, run `scripts/run_integrated_checks.py` as described in the [running guide](../Documentation/running.md). That command runs unittest discovery; pytest additionally includes function-based experiment tests. Test names describe current behavior rather than development milestones.
