# Native Locobot camera visibility check

Measured on 11 October 2026 using the locally installed iGibson Locobot URDF,
with default joints and the robot settled on a flat plane in PyBullet DIRECT.
The camera uses the `eyes` link's position and orientation, with local +X forward
and +Z up, matching iGibson's `render_single_robot_camera` convention. No camera
height or direction override is applied. The robot remains visible to the renderer.

The camera measured **0.858 m above the floor, pitched 20.08 degrees downward**.
The model's head pan and tilt joints are fixed. The earlier C demo uses this
link's position but overrides its direction with -25/0 degree rendered views.

## Controlled results

The test uses PyBullet TinyRenderer, 320x240 images and a 60 degree vertical FOV
(the demo's configured FOV, not a measured physical sensor specification).
Authored targets are red boxes 5 cm wide/deep and either 5 or 13 cm tall.
Furniture fronts are 0.75 m from the robot origin; targets are at x=0.90 m
(front) or x=1.30 m (back). The camera is at approximately x=0.055 m.

| Fixture | Placement / target | Visible target pixels | Result |
| --- | --- | ---: | --- |
| Table, top 0.78 m | Front, small / tall | 182 / 481 | Fully visible |
| Table, top 0.78 m | Back, small / tall | 81 / 225 | Fully visible |
| Counter, top 0.88 m | Front, small / tall | 182 / 507 | Partially visible |
| Counter, top 0.88 m | Back, small / tall | 63 / 207 | Partially visible |
| Open shelf, board top 0.60 m | Back, small | 89 | Fully visible |
| Open shelf, board top 0.80 m | Back, small | 81 | Fully visible |
| Open shelf, board top 1.20 m | Back, small | 0 | Outside native view at this pose |
| Open shelf, board top 1.50 m | Back, small | 0 | Outside native view at this pose |

Visibility uses renderer segmentation solely as a diagnostic. An unobstructed
reference capture removes the furniture while retaining the target, robot and
camera pose, distinguishing furniture occlusion from absence in the camera view.
Pixels do not establish YOLO detection accuracy. High shelves may enter the view
from farther away, but their boards may still hide objects; this test does not
establish that all alternative viewpoints fail.

This is a controlled geometry experiment, not an Rs/Rs_int apartment trial or
a navigation acceptance test. The fixtures omit clutter, table legs and realistic
materials. Cabinet fixtures have an open front; closed-cabinet contents are not
tested. Existing C demo results remain historical results with rendered tilt.

## Reproduce

Use an environment containing iGibson's assets, PyBullet, NumPy and Pillow:

```powershell
& C:/Users/dance/miniconda3/envs/igibson/python.exe scripts/check_locobot_camera.py --output runs/locobot-native-camera-002
```

Choose a new output directory. Each run writes `report.json`, individual PNGs
and `contact-sheet.png`. The report includes the installed URDF SHA-256 and actual
camera quaternion. Local first-run artifacts are in `runs/locobot-native-camera-001`.

Locobot is a plausible baseline for ordinary tables and lower open shelves.
Before choosing it for the final task, repeat native-camera captures at reachable
poses in the agreed apartment scene with the actual target objects and detector.
Include farther viewpoints for high shelves and more deeply recessed small
counter objects. No robot switch is established by this test alone.

## Perception selection and scene smoke capture

After reviewing this check, the project selected Locobot for perception with its
native fixed camera. `configs/perception.locobot.json` is consumed by
`frontier_search.simulation.capture_igibson`; the capture path constructs Locobot
and calls iGibson's native robot-camera renderer without pitch overrides.

A local `Rs_int`, seed 0 smoke capture on 11 October succeeded in the Python 3.8
iGibson environment: 76,800/76,800 valid depth pixels, axial range 0.938–3.735 m,
and maximum XYZ projection error 0.00002950 m. The saved observation passed replay
validation. RGB/depth conversion, recording and projection checks passed 17 tests
under the default Python environment. Artifacts are in
`runs/locobot-native-capture-001`. This one stationary capture does not freeze the
scene or establish multiframe mapping/navigation acceptance.
