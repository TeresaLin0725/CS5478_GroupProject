# Detector feasibility summary

Updated: 4 October 2026. Target label: `cup`, covering the tested iGibson cup and mug assets.

**Current choice: YOLO11s with native 640x480 captures for development.** It performed best among the tested models, but results are too limited to establish reliable autonomous search. The project confidence threshold remains 0.8.

## Model and resolution comparison

The same 12 isolated target views were tested at each resolution. Counts show views containing a cup prediction at each confidence cutoff.

| Approach | At 0.25 | At 0.50 | At 0.80 | Median CPU time |
|---|---:|---:|---:|---:|
| [YOLOX-S, 320x240](../../runs/detector-feasibility-probes/results.json) | 1/12 | 1/12 | 1/12 | 316.6 ms |
| [NanoDet, 320x240](../../runs/detector-feasibility-nanodet/results.json) | 0/12 | 0/12 | 0/12 | 95.6 ms |
| [YOLO11s, 320x240](../../runs/detector-feasibility-yolo11s-py38/results.json) | 6/12 | 4/12 | 1/12 | 153.6 ms |
| [YOLO11s, 640x480](../../runs/detector-feasibility-yolo11s-640-py38/results.json) | **9/12** | **5/12** | **4/12** | 204.2 ms |

Native higher resolution improved YOLO11s detection. All approaches produced no detections on the single isolated absent control. All runs use Python 3.8. Timings are median per-image warm prediction times across each tested set; model backends and timing boundaries differ, so these are indicative rather than a controlled speed comparison.

## Furnished scenes and robot camera

YOLO11s at 640x480; detections must overlap the visible target box with IoU >= 0.5. Fully hidden targets are excluded.

| Test | At 0.25 | At 0.50 | At 0.80 | Absent views with cup false alarms (0.25 / 0.50 / 0.80) |
|---|---:|---:|---:|---|
| [Furnished scene, fixed viewpoints](../../runs/detector-feasibility-furnished-yolo11s-v3-py38/scored.json) | 9/20 (45%) | 6/20 (30%) | 4/20 (20%) | 1/6 / 0/6 / 0/6 |
| [TurtleBot approach](../../runs/detector-robot-approach-yolo11s-py38/scored.json) | 29/40 (72.5%) | 27/40 (67.5%) | 17/40 (42.5%) | 0/21 / 0/21 / 0/21 |

These tests use different viewpoints and scoring from the isolated screen, so their rates are not directly comparable. Approach frames are correlated: at 0.25, the cup asset was detected in 11/20 visible frames and the mug in 18/20. The robot moved from 1.50 m to 0.79 m before timing out; targets eventually left the camera view. This was not a completed search episode.

## Later improvements

- **Depth and tracking:** associate detections with 3D positions and confirm consistent observations across frames.
- **Broader evaluation:** test more assets, rooms, distances, occlusions and absent scenes; use separate validation data to choose thresholds.
- **Viewpoint control:** keep targets in view while approaching; investigate steering conventions and the near-table slowdown.
- **Model adaptation:** if misses persist, evaluate a larger model or fine-tuning on labelled simulator images using held-out scenes.
- **Runtime:** measure the complete perception pipeline and test GPU inference if CPU latency limits operation.

Linked results and recordings are local, Git-ignored artifacts under `runs/`.
