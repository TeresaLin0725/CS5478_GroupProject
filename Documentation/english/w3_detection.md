# W3 steps 1–2: RGB detection, tracking and room association

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

Each accepted detection contributes private `DetectionEvidence` carrying its
capture sequence, timestamp, box and measured position. Localized objects are
associated with existing same-class tracks using a configurable 3D distance
gate (`MappingConfig.object_match_distance_m`, default 0.35 m). The measurement
must be close to both the track estimate and its first measurement, preventing
gradual drift. Candidate matches are processed by shortest distance, then track
ID and canonical detection order, with one detection per track per frame.
Exact duplicate class/box proposals collapse to the strongest confidence.

Track positions use coordinate-wise medians of accepted measurements; confidence
is the maximum observed confidence. IDs, first/last timestamps and evidence are
retained across missed frames and revisits until episode reset, consistent with
the static-scene assumption. `confirmation_count` counts distinct accepted
capture sequences over the track's lifetime. It is not yet a target-confirmation
decision: temporal-window confirmation and claims remain step 3. No claim is
emitted. Unknown-depth detections have a single-frame identity and expire on the
next accepted update; they cannot be associated by image overlap alone.

Localized tracks are associated with continuous room polygons using world x/y.
Exactly one containing polygon is required; positions outside all rooms, on
shared boundaries, or in overlapping rooms remain unassigned. The mapper keeps
track-to-room assignments privately and publishes sorted, deduplicated
`RegionSummary.observed_object_classes` through the frozen public contract.
Classes persist with localized tracks, but hidden semantic room labels remain
absent in standard runs. All track state and evidence reset with the episode.

The distance threshold is a development default requiring scene validation.
Greedy spatial matching may confuse nearby same-class objects, and the static
map retains false detections until reset. Appearance matching, moving-object
tracking, target claims and claim evidence retrieval remain future work.

Malformed detector output or inference exceptions reject the whole update
without changing maps, counters, evidence or processed sequence. Already
processed frames are rejected before inference. Tests in
`tests/test_mapper_detection.py` exercise localization, confidence filtering,
invalid depth, evidence, rollback, reset, duplicate frames and the YOLO adapter
with a stub model. `tests/test_object_tracking.py` covers stable identities,
one-to-one matching, missed frames, spatial/class gates, drift, room ambiguity,
concave polygons and transactional rollback. These do not establish model accuracy.

Local validation on `runs/locobot-full-scan-package-001` produced two chair
tracks, each supported by two distinct frames, over the nine-frame recording.
Two full replays matched exactly (`runs/w3-tracking-001/report.json`). This
recording contains no room priors, so room association is covered by synthetic
polygon tests rather than this real-data replay. The recording still has zero
reachable navigation candidates; tracking does not close that separate W2 gate.
