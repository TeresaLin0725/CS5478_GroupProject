# A: portable RGB-D sequence and offline A/B/C replay

The replay tool runs the production `RGBDMapper`, B's `TeamNavigator` candidate
assessment, and C's Nearest, Information Gain and Random policies on the same
recorded frames. It uses a read-only pose/time control object; velocity commands
raise an error. It never executes a selection or advances a simulator.

## Run the development regression

From the repository root, using new output directories:

```powershell
$env:PYTHONPATH = "$PWD/src"
python -m frontier_search.runner.replay generate-synthetic --output runs/w2-rgbd-sequence-001
python -m frontier_search.runner.replay run --dataset runs/w2-rgbd-sequence-001 --output runs/w2-rgbd-replay-001
```

Open `runs/w2-rgbd-replay-001/report.html` for a table of frame-level coverage,
frontier counts, reachable candidates and selections, plus linked maps. Outputs:

- `manifest.json`: the fully resolved shared configuration.
- `events.jsonl`: complete immutable states, B assessments, eligible IDs and C
  decisions per frame, with canonical state hashes.
- `frames/*.svg`: observed occupancy/coverage, frontier support, assessed
  approach poses and recorded robot pose, with world +y upward.
- `report.json`: source label, package/result hashes, frame counts, coverage
  growth, assessment totals and repeat-verification result.
- `report.html`: a local, script-free view of those results.

Replay runs twice by default and requires exactly matching state/assessment/
decision hashes. `--no-verify-repeat` explicitly disables the second pass and
records `repeat_verified=false`. `--policies nearest information_gain` limits
selection checks to those policies. Hosted LLM calls are not part of this tool.
Each policy has its own recorded-seed RNG. B retains its configured assessment
deadline, so overloaded machines can produce inconclusive assessments; repeat
differences fail explicitly rather than being suppressed.

The supplied configuration is `configs/w2.rgbd-synthetic.json`. Its seven frames
contain a stationary sweep, a 0.4 m prescribed translation, and another sweep.
The analytic scene is a flat floor with a wall at x=4 m; RGB is uniform grey and
depth is rendered geometrically. The camera is at 0.6 m and pitched down 0.65 rad
so the initial views can observe nearby floor and connect the spawn footprint
to mapped free space. These are development settings, not the team's real robot
selection. Only the initial footprint-plus-margin cells seed free space.

Measured on 10 October 2026: 7 frames; known cells 186 → 322; observed cells
246 → 334; 69 reachable, 11 rejected, 0 inconclusive assessments across frames;
identical repeat hashes.
Assessment totals count repeated observations of candidates, not unique
opportunities. The recorded translation is descriptive input metadata, not
policy execution distance or a policy-efficiency result. No object-search
success is evaluated. This synthetic package does not complete real-data A7
acceptance; the shared robot and B adapter are still needed for that check.

## Package existing captures

Each input frame uses `fixtures.recording`: `observation.json`, `rgb.bin` and
`depth.bin`, retaining sequence, simulation time, capture pose, intrinsics and
camera-to-world transform. Supply a shared `EpisodeManifest` and a separately
verified initial-clearance mask from B. Example clearance file structure:

```json
{
  "cells": [{"row": 9, "column": 3}, {"row": 10, "column": 3}],
  "provenance": "Describe the adapter's actual spawn-clearance check here"
}
```

Those cell numbers illustrate the format only. The supplied cells must exactly
match the selected robot's footprint plus margin at the manifest start pose;
the tool rejects extra cells that would reveal more free space. The exporter
must establish that this mask really is clear. Replay validates the mask's
extent and records the provenance; it cannot independently verify historical
physical clearance.

After the real configuration and captures exist:

```powershell
python -m frontier_search.runner.replay pack --manifest configs/w2.real.json --frames runs/capture-001 runs/capture-002 runs/capture-003 --clearance configs/spawn.real.json --output runs/w2-real-recording-001
python -m frontier_search.runner.replay run --dataset runs/w2-real-recording-001 --output runs/w2-real-replay-001
```

Pass frame directories in capture order. `pack` preserves each capture's source
metadata. The first pose must equal the manifest start pose, all episode IDs
must match, sequences strictly increase, timestamps never decrease, and
intrinsics stay constant. By default the mapper requires fixed base elevation
to 1e-6 m. `mapping.base_elevation_margin_m` can explicitly allow bounded
floor-relative base drift: the fixed world collision interval is padded by this
margin in both directions, and actual per-frame poses/transforms are preserved.
The recorder's local development manifest uses 0.005 m for observed millimetre
drift. Frames beyond that initial-base-relative bound still fail. This is not
support for changing floors or stairs. Set the manifest source to the actual
capture source; never rewrite physical captures to hide vertical motion.

The package contains a complete `manifest.json`, frame directories and
`sequence.json`. The index records the ordered frame paths, configuration/frame
SHA-256 checksums, clearance cells and provenance. Paths must stay within the
package. Missing/corrupt files, duplicated frames, ordering mismatches and
changed calibration are explicit errors. Existing output directories are never
overwritten. `sequence.json` is written last so incomplete exports are not
accepted as complete packages. These checksums detect accidental input changes;
they are not an authenticity signature.

## Interpretation and next acceptance gate

Every frame is mapped and assessed as an offline decision probe. Policy stop
requests are recorded, and replay continues through the remaining input frames.
No navigation attempts, arrival outcomes or recovery cooldowns are fabricated.
A fixed recording cannot respond to a selected frontier, so it cannot compare
search efficiency or replace a real closed loop.

The next team gate is real recordings with the agreed configuration, followed
by B's real adapter and C's runner factory. The existing W2 mock command still
uses its explicitly scripted mapper. Tracking, target claims and independent
target verification remain W3 work.

## Optional verified starting-area experiment

The exact footprint-mask rules above describe the default format-version-1
baseline. Explicit `verified_start_area` manifests use format version 2 and
require B's bounded connected mask and verification record. See
[the A implementation and B handoff](verified_start_area.md) for the schema,
physical checks and acceptance commands. Do not enlarge a default clearance file.
