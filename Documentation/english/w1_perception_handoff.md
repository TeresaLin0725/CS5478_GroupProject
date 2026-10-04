# Role A: W1 handoff

3 October 2026. Role A's W1 implementation is ready; the shared integration milestone still needs B/C wiring and configuration agreement.

| Deliverable | Status |
|---|---|
| Mapper reset | Atomic initialization, private seeded RNG, clean episode state; only supplied spawn-clearance cells become free. |
| Prior loader | Versioned world-polygon JSON; shared geometry/label validation with reset. |
| RGB-D conventions | Capture conversion, input validation, recording/replay and depth projection implemented. |
| Detector feasibility | YOLO11s with native 640x480 is the development candidate; [comparison and limitations](detector_feasibility.md). |

## Integration boundaries

- B provides RGB-D, robot geometry and verified spawn clearance. A loads exported room polygons; C constructs `MapperInit` and wires the runner. Simulator-specific polygon export remains an integration task.
- Reset publishes no state. `update()` and evidence retrieval raise `NotImplementedError`. Existing obstacle-endpoint helpers are W2 groundwork; free-space mapping, coverage and frontiers come next.
- Diagnostic masks and object identities stay outside standard perception inputs. The approach script's measured steering sign and near-table slowdown need B's review before navigation integration.

## Usage and checks

See [development usage](development_fixtures.md#a-rgb-d-recording-conventions) for formats, capture and replay, and [README](../../README.md#run-the-development-demo) for test commands. Unit tests cover reset, priors, sensor validation, replay, projection and scoring. Optional NumPy tests check renderer conversion; real captures are local smoke checks.

Use the same Python 3.8 iGibson environment for capture, mapping and detector screening. Inference and a same-process simulator capture passed smoke checks. Versions are recorded in [screening metadata](../../configs/detector_screening.json).

```powershell
conda activate igibson
python -m pip install torch==2.4.1 torchvision==0.19.1 --index-url https://download.pytorch.org/whl/cpu
python -m pip install ultralytics==8.3.203 numpy==1.24.4 opencv-python==4.10.0.84
```

Download the listed weights. For OpenCV models, also copy their reference files and save the corresponding `models` entry as `weights/<model>/source.json`.

Run `scripts/perception/check_yolo11.py --images <PNGs> --output <new directory>` or `scripts/perception/check_detector.py` in this environment. Weights, datasets, captures and environments remain local under ignored directories. Final experiment settings are still pending validation.
