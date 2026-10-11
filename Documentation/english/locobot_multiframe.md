# Continuous native-camera Locobot recording and replay

Implemented and checked 11 October 2026. The recorder collects real iGibson
frames in one session; the existing portable replay performs offline A mapping,
B candidate assessment and C baseline decisions. It does not execute policy goals.

## Record

```powershell
conda activate igibson
$env:PYTHONPATH = "$PWD/src"
python -m frontier_search.simulation.capture_igibson --sequence --scene Rs_int --seed 1 --output runs/locobot-sequence-new
```

Use a new output directory and the same environment/assets as the single-frame
capture. `--reference-project PATH` overrides the default standalone scene/spawn
helper. The configured Locobot camera stays attached to its native `eyes` pose;
the robot's base supplies all viewpoint changes. No camera pitch/height override
or trajectory teleport is applied.

The bounded development script records seven frames: initial view, three base
turns, a 1-second 0.1 m/s forward command, and two further turns. Angular command
segments are +0.4 rad/s for 1 s, -0.4 for 2 s, +0.4 for 1 s, then after movement
+0.4 for 1 s and -0.4 for 2 s. Each segment is followed by a zero-velocity command
and 12 settling simulation steps before capture. Locobot's command sign differs
from the usual yaw convention in the historical demo; recorded orientations
are measured, and this recorder does not claim target heading control. Commands
and actual frame poses/timestamps are retained separately.

The recorder saves:

- `frames/000001` ...: immutable observations, raw RGB/depth, native camera
  transforms, source/asset metadata and RGB previews.
- `capture-report.json`: frame order, commands, status, actual translation,
  base-z range, strict-default compatibility and measured setup scope.
- `development-manifest.json`: a local replay configuration, not the agreed
  live experiment manifest. Map: 0.1 m cells, 100x100, origin (-5,-5), no room labels.
- `development-clearance-check.json`: the exact initial footprint-plus-margin
  cells and any blocked cells. `development-clearance.json` is written only
  when every tested cell volume is clear.

Geometry is measured with a separate physics client and an aligned copy of the
installed URDF with the recorded joint positions. The conservative collision
AABB rectangle measured +/-0.181 m horizontally; the collision top is about
0.907 m above the base. No recorded robot state is moved for that measurement.
The local manifest uses a 0.02 m clearance margin and the script's speed bounds.
B/C must review these development settings before adopting them for live use.

The spawn check queries full cell-square columns against all nonrobot collision
bodies, above the floor tolerance and up to the padded robot top. It uses scene
collision geometry only for the initial clearance check; it supplies no scene
occupancy to perception/policies. This is not B's completed adapter acceptance.
An obstacle-contact guard aborts scripted motion on force-bearing contacts above
the floor band. It is not a navigation planner or swept-volume clearance proof.
Raw frames/failure reports remain available when collection or clearance fails.

## Bounded base-height drift

The first real seed-0 recording varied by 0.002937 m in base z. The old 1e-6 m
flat-floor restriction rejected it. The new optional mapping field
`base_elevation_margin_m` expands the **fixed world collision envelope** downward
and upward around its first-frame bounds. The lower bound still excludes the
configured floor band. Free evidence must cover the entire padded envelope;
obstacles in its added height band stay blocking. Raw poses and camera transforms
are never rounded or rewritten.

Default margin is zero, preserving the old strict limit. The local recording
manifest explicitly selects 0.005 m, plus the existing 1e-6 numerical tolerance.
Frames beyond this distance from initial base z fail atomically. This is a bounded
single-floor model, not support for stairs/floor transitions. The report's
`flat_floor_sequence_compatible` reports compatibility with the zero-margin
default, not acceptance under the development manifest.

## Package and replay

After reviewing the development manifest and ensuring clearance passed:

```powershell
$frameDirs = Get-ChildItem runs/locobot-sequence-new/frames -Directory | Sort-Object Name | ForEach-Object FullName
python -m frontier_search.runner.replay pack --manifest runs/locobot-sequence-new/development-manifest.json --frames $frameDirs --clearance runs/locobot-sequence-new/development-clearance.json --output runs/locobot-package-new
python -m frontier_search.runner.replay run --dataset runs/locobot-package-new --output runs/locobot-replay-new
```

Repeated standalone single-frame invocations are not a continuous sequence.
Keep frame order, actual poses, shared episode ID and increasing sequences.
Do not turn rejected clearance cells into a seed, enlarge the seed radius,
or change poses to bypass validation. Reports and map SVGs are in the replay
output; `repeat_verified=true` checks the same fixed recording twice.

## Local results and remaining acceptance gap

Seed 0's initial conservative grid mask overlapped scene geometry, so it was
rejected as replay setup. A corrected base-aligned measurement still rejected
one cell. Those raw diagnostic runs remain in ignored `runs/`.

Seed 1 (`runs/locobot-native-sequence-004`) passed the full-cell check. Seven real
frames packaged in `runs/locobot-native-package-001` and replayed in
`runs/locobot-native-replay-001`:

| Check | Result |
| --- | --- |
| Recording source | Real iGibson `Rs_int`, native camera, seed 1 |
| Frames / actual recorded translation | 7 / 0.114046 m |
| Base-z span | 0.000232 m |
| Repeat verification | Passed |
| Known map cells | 184 -> 352 |
| Explored cells | 216 -> 394 |
| Free cells | 99 -> 106 |
| B assessments | 0 reachable, 33 rejected, 0 inconclusive |
| Regression suite | 216 tests, no failures, 5 optional simulator skips |

The pipeline completed, but **live exploration acceptance has not passed**.
B rejected frontier approach poses as `invalid_goal`; goal rotation/footprint
checks still overlap unknown cells. The initial robot footprint is known free,
but from frame 2 onwards its changed footprint overlaps unknown cells too.
The short native-camera scan adds little clearance evidence outside the seed.
This test validates real multiframe plumbing and reveals a floor/clearance
coverage problem; it does not establish successful C goal selection or movement.

Next joint A/B work: examine the per-frame maps and measured footprint, validate
a fuller base scan and reachable motion in a more open start area, and check
whether the conservative height-binned mapper can gather sufficient clearance
with this fixed camera. Preserve unknown-space safety and the verified spawn
mask. C can wire the real factory once these checks yield usable goals. Camera
visibility for tabletop objects alone does not establish navigation-map adequacy.

All `runs/` artifacts are local/ignored. Reproduce them or explicitly exchange a
recording package; Git delivers only the recorder, tests and documentation.

## Full scan follow-up and rejection diagnostics

Use `--sequence --full-scan` to record eight measured 45-degree base turns, each
with a deadline and a capture after stopping. A conservative circumscribed-disc
collision check first verifies the rotation volume. This extra setup check
does not add any cells to A's initial free-space seed. Failures retain raw frames.

```powershell
python -m frontier_search.simulation.capture_igibson --sequence --full-scan --scene Rs_int --seed 1 --output runs/locobot-full-scan-new
```

Package its nine frames using the instructions above. To inspect exact footprint
versus grid-centred planning-disc failures and per-cell missing height evidence:

```powershell
python -m frontier_search.runner.diagnose_clearance --dataset runs/locobot-full-scan-package-001 --output runs/full-scan-diagnostics-new.json
```

This offline developer tool reads A's private evidence buffers without changing
occupancy or B's checks. `exact_blocked_cells` and `lattice_disc_blocked_cells`
identify the two goal checks. Blocked cells include floor evidence and missing
bin indices. The histogram gives world-z ranges for bins missing in
floor-observed unknown cells; start-footprint diagnostics cover nearby blindness.

The timed-command trial rotated only 342.3 degrees and was rejected. With
measured heading stops, the accepted seed-1 run (`runs/locobot-full-scan-003`)
rotated 372.4 degrees including settling overshoot. Cumulative base drift between
captures was 0.0564 m and base-z span 0.000144 m. The rotation-volume check passed.
Its package/replay is in `runs/locobot-full-scan-package-001` and
`runs/locobot-full-scan-replay-001`.

| Full-scan check | Result |
| --- | --- |
| Repeat verification | Passed |
| Known cells | 184 -> 1,618 |
| Explored cells | 216 -> 2,313 |
| Free cells | 99 -> 681 |
| Final valid goal poses | 11 of 19 |
| Total B assessments | 0 reachable; 65 `no_path`, 56 `invalid_goal` |
| Final exact start footprint | Invalid: 5 unknown cells |
| Nearest ideal bottom-edge floor view | About 0.720 m from camera XY |

All five final start-blocking cells lack observed floor and low-height evidence
(roughly the 0.03-0.43/0.53 m bins, depending on cell). The floor-view distance
comes from the actual calibration/transform and a flat z=0 floor, ignoring
occlusion; it is not a depth-range setting. Rotating the base fills distant
coverage but cannot eliminate this nearby floor blind area.

One initial candidate fits the exact footprint yet fails B's larger planning
disc at the cell centre. The square +/-0.181 m footprint produces a circumscribed
radius of about 0.256 m before margin and polygon discretization. Thus planning
conservatism contributes as well as missing sensor evidence.

Next: B validates a tighter collision-containing footprint and planning-grid
conservatism; A/B address near-floor sensing or explicitly revise the permitted
clearance-evidence/initialization model. A tighter footprint alone does not prove
a path across the unseen near-floor area. Keep the native camera pose,
unknown-space blocking and verified seed intact pending that decision. A larger
verified initialization area requires a declared contract change.
