# Locobot native-camera handoff for A, B and C

Updated 11 October 2026. Read this first when continuing W2 integration in a
separate Codex session. The shared contracts remain in [interface_freeze.md](interface_freeze.md).

## Decision and configuration

The user selected **Locobot for perception, using the unmodified model's fixed
`eyes` camera** after reviewing the native-camera visibility test. A raised mast,
Fetch and rendered camera pitch sweeps are not part of this selected baseline.
The robot can turn its base to scan.

[`configs/perception.locobot.json`](../../configs/perception.locobot.json) is the
source for the perception capture settings:

| Setting | Value |
| --- | --- |
| Robot / camera link | `Locobot` / `eyes` |
| Camera pose | Native link position **and orientation** at capture time |
| Image dimensions | 320 x 240 |
| Vertical field of view | 60 degrees |
| Robot visibility | Retained (`hide_robot=false`) |

The measured camera was about 0.858 m above a flat floor and pitched about
20 degrees downward. These are measurements, not constants to substitute for
the actual capture-time link transform. Follow iGibson's
`render_single_robot_camera`: rotate local +X to obtain forward and local +Z
to obtain up. `render_robot_cameras` already implements this convention.
Use the installed model's default fixed joints and record its asset version/hash.

This file is a **capture configuration**, not an `EpisodeManifest`. Do not pass
it to `runner.w2 --manifest`. Scene, map bounds/resolution, complete robot
collision envelope, safety margin, motion limits, start pose and capture cadence
still need a shared A/B/C manifest. `configs/w2.mock.json` and
`configs/w2.rgbd-synthetic.json` contain development geometry; their robot
dimensions must not be reused as measured Locobot geometry.

## Delivered and verified by A

- Production `perception.mapper.RGBDMapper`: occupancy/coverage, room coverage,
  frontiers, approach poses, identity/lineage and arrival-sweep gain.
- Portable multiframe packaging and replay: integrity/order/calibration checks,
  repeat verification, B assessments, C baseline decisions and map/HTML reports.
- `simulation.capture_igibson`: now constructs Locobot and uses the native camera.
  It saves **one frame per invocation**, with robot/camera/config metadata,
  camera pose and URDF checksum. Repeated invocations are separate captures with
  sequence 1, not a continuous movement recording.
- `scripts/check_locobot_camera.py`: reproducible controlled furniture visibility
  test with the installed model, using PyBullet TinyRenderer.
- One real iGibson `Rs_int`, seed 0 stationary capture: 76,800 valid depth pixels,
  projection error 0.00002950 m, saved-observation validation passed. Seventeen
  conversion/recording/projection tests passed. The earlier complete W2 suite
  ran 212 tests with five optional simulator skips.

See [camera results](locobot_camera_check.md), [mapping assumptions](w2_mapping.md)
and [replay instructions](w2_rgbd_replay.md). Artifacts in `runs/` are ignored and
are **not distributed by Git**. Reproduce them using the commands below; do not
assume another teammate has A's local recording or licensed scene assets.

## B: simulator, geometry and continuous recording

Continue in your B branch after incorporating `integration/w2`.

1. Construct the same Locobot/native-camera configuration in the real adapter
   (`simulation/igibson_adapter.py` is still a stub). Capture RGB-D from one
   stationary simulation state, with its actual base pose, intrinsics and
   camera-to-world transform. Reuse `simulation.rgbd.convert_igibson_frames` for
   axis/pixel-centre conversion and the XYZ consistency check.
2. Measure the collision footprint and vertical envelope of this actual model,
   relative to the base; agree on margins and motion limits. Do not use the
   camera's height as the robot collision height. B applies navigation inflation;
   A's occupancy grid stays uninflated.
3. Verify only the initial footprint-plus-margin clearance cells. Export those
   cells and the check's provenance; do not seed a broad free radius or provide
   scene occupancy to A/C. Floor/scene truth may be used for setup checks only.
4. Provide one continuous short move/scan recording: one episode ID, strictly
   increasing sequences, nondecreasing simulation timestamps, fixed intrinsics,
   actual per-frame poses/transforms. Export frames using `fixtures.recording`.
   Supply the agreed manifest and exact spawn mask for replay packaging.
5. Support runner-owned stepping and the navigator's pose/velocity/clock APIs,
   stop/pause/resume/close. Base rotations perform scans; no independent camera
   pan/tilt commands or camera pose overrides.

Important acceptance issue: A currently rejects base-z changes greater than
1e-6 m within an episode. Preserve measured poses and report any simulator
vertical drift to A; do not round z to zero to make validation pass. A must
validate or extend its flat-floor evidence model if the physical captures vary.
The reference spawn helper is a development utility, not the completed B
clearance contract. Its use in the smoke capture does not certify an export mask.

## C: runner factory and experiment reporting

Continue in your C branch after incorporating `integration/w2`.

1. Preserve the mock entry point for regression. The default `runner.w2` factory
   is still `fixtures.w2_episode:build_mock_runner`, with `ScriptedMapper`.
   Provide a separate real factory assembling A's production mapper, B's real
   adapter/TeamNavigator and the existing episode runner. Instantiate the mapper
   from `perception.mapper`, not the stand-in class in `simulation.igibson_rs`.
2. Build `MapperInit` from the shared manifest and B's verified clearance mask.
   Record the selected Locobot/camera settings and assets in experiment metadata;
   all policies must receive the same configuration and observations.
3. Reuse scan barriers, revision-coherent assessments, eligibility filtering,
   bounded recovery and supervision. Replay remains an offline selection check;
   actual move/arrival outcomes come from the live adapter.
4. Demonstrate at least two selection -> movement -> base-turn arrival-scan
   cycles jointly with A/B, plus bounded failure handling. Target claims/detector
   tracking/verifier integration remain W3 work; W2 mapping returns no claims.

The historical `simulation.igibson_rs.IGibsonWorld.capture` manually overrides
the view direction to -25/0 degrees and its demo uses mapper/navigation
stand-ins. Those results do not validate the selected native-camera production
loop. Locobot-specific angular-command and caster adjustments in that demo
require independent validation before being adopted by B.

## Reproduce now and run the next acceptance check

From the repository root, with the team's iGibson environment and required assets:

```powershell
conda activate igibson
$env:PYTHONPATH = "$PWD/src"
python scripts/check_locobot_camera.py --output runs/locobot-native-camera-new
python -m frontier_search.simulation.capture_igibson --config configs/perception.locobot.json --scene Rs_int --seed 0 --output runs/locobot-native-capture-new
```

The capture defaults to the scene/spawn helper in
`D:/Projects/CS5478_RoboticProject`; pass `--reference-project PATH` on another
machine. `Rs_int` requires installed interactive assets; it is the smoke-test
scene, not a frozen team scene. The capture utility currently constructs
`InteractiveIndoorScene`, so specifying static `Rs` is not a supported substitute.
Use a new output directory each time. iGibson may write caches beside its assets.

Once B's continuous frames, measured geometry and spawn clearance are available,
use the following command shapes. **These real manifest/clearance filenames are
placeholders and do not exist yet.** Pass frame directories in recording order:

```powershell
python -m frontier_search.runner.replay pack --manifest configs/w2.real.json --frames runs/frame-001 runs/frame-002 runs/frame-003 --clearance configs/spawn.real.json --output runs/locobot-sequence-new
python -m frontier_search.runner.replay run --dataset runs/locobot-sequence-new --output runs/locobot-replay-new
```

A will inspect occupancy/coverage, frontier identity and B assessments, and
measure processing cadence. The first frame's pose must equal the manifest
start pose; the spawn mask must exactly match the footprint plus margin. Keep
failure reports, especially base-z/calibration violations. The fixed-camera
views must provide enough floor/clearance evidence for mapping as well as object
visibility. The previous synthetic 640x480 timing was about 3.48 s/frame and is
not a measured native-camera 320x240 integration rate.

## Paste into the other Codex sessions

For B:

> Incorporate integration/w2 and read Documentation/english/locobot_abc_handoff.md.
> Perception selected unmodified Locobot with its native fixed eyes camera; use
> configs/perception.locobot.json. Implement/validate the real adapter and measured
> robot geometry, then export a continuous short base-movement/scan RGB-D sequence
> with verified footprint clearance for A's existing replay. Coordinate the real
> manifest with A/C and report base-z drift without rewriting measured poses.

For C:

> Incorporate integration/w2 and read Documentation/english/locobot_abc_handoff.md.
> Use A's production perception.mapper.RGBDMapper and B's real Locobot adapter
> in a separate real runner factory. Preserve the mock regression entry point.
> The camera is fixed to the native eyes link; historical rendered pitch sweeps
> are not the selected baseline. Coordinate the real manifest and validate live
> selection/move/arrival-scan cycles once B's adapter and A's replay acceptance pass.
