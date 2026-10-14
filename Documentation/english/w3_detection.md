# W3 step 1: RGB detection in the mapper


The workspace selects `.venv-detector` as the default VS Code Python interpreter
and sets project-local YOLO settings and `PYTHONPATH` for new terminals.
PowerShell activation scripts are disabled on this machine; use the launcher
to run Python with the detector environment without changing execution policy:

```powershell
.\scripts\detector-python.cmd -c "import ultralytics; print(ultralytics.__version__)"
# Run a project script with the same environment:
.\scripts\detector-python.cmd path\to\your_script.py
```

The replay runner now constructs `RGBDMapper(YOLODetector(...))` when
`--detector-weights` is supplied. The launcher supplies the existing YOLO11s
weights automatically:

```powershell
.\scripts\run-detection.cmd --dataset runs/w2-rgbd-sequence-001 --output runs/w3-detection-001
```

Use an existing packaged RGB-D dataset and a new output directory. Objects
appear in each frame's `state.semantic_objects` in `events.jsonl`; the report
records the detector weights path and checksum. This is offline replay and
does not execute navigation or score target claims.

`RGBDMapper` accepts an optional RGB-only detector. To enable the local YOLO11s
weights used in detector screening, use an environment with `ultralytics` and
`numpy` installed:

```python
from frontier_search.perception.detection import YOLODetector
from frontier_search.perception.mapper import RGBDMapper

mapper = RGBDMapper(YOLODetector("weights/yolo11/yolo11s.pt"))
mapper.reset(mapper_init)
state = mapper.update(observation)
objects = state.semantic_objects
```

The constructor requires an existing weights file; it does not automatically
download weights. `RGBDMapper()` continues to support mapping-only callers.
The detector receives only the packed RGB image. The YOLO adapter converts RGB
to BGR for the model and clips boxes to half-open integer image bounds. The
mapper applies `MappingConfig.confidence_threshold` to every class, including
non-target objects that will later support semantic room summaries.

For each accepted detection, the mapper takes the median valid axial depth in
the central half of its box, then projects the patch centre through the camera
intrinsics and capture-time camera-to-world transform. Depth limits reuse the
mapping configuration. With no valid depth, `position_world` is `None`.
This estimates an observed surface position; box background and occlusion can
bias it. It does not establish the true object centre.

Each accepted detection produces an immutable provisional `ObjectTrack` with
one observation ID and confirmation count 1, plus private `DetectionEvidence`
carrying the capture sequence and box. IDs increase within an episode and reset
with the episode. Only the current frame's provisional objects/evidence are
retained at this stage; they are replaced on the next successful update.
Cross-frame identity, retained track history, room association, target claims,
and claim evidence retrieval remain subsequent W3 steps. No claim is emitted.

Malformed detector output or inference exceptions reject the whole update
without changing maps, counters, evidence or processed sequence. Already
processed frames are rejected before inference. Tests in
`tests/test_mapper_detection.py` exercise localization, confidence filtering,
invalid depth, evidence, rollback, reset, duplicate frames and the YOLO adapter
with a stub model. They do not establish real model accuracy.
