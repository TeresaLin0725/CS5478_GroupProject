# Verified starting area: A implementation and B handoff

Historical handoff below. The current integrated native demo implements the
verification in `simulation/start_area_verifier.py` and `IGibsonAdapter.verify_start_area`.
Its manifest opts into a 0.65 m radius; only verified connected cells are seeded,
not a free circle. This remains a stronger initialization prior than the
footprint-only 1.0 baseline. Current code/evidence and remaining experiments are
tracked in the [design audit](../chinese/demo_design_review.md).

The native Locobot camera remains unchanged. The 360-degree recording exposed a
near-floor blind area: the final base overlapped five unknown cells despite 11
valid goal poses. See [measured results](locobot_multiframe.md). A larger, physically
verified initial prior is an **opt-in experiment**, not proof that this blind-area
problem is solved. The default contract remains footprint-only.

## What A has implemented

Set these top-level manifest fields only for the new experiment:

```json
{"initial_clearance_mode": "verified_start_area", "initial_clearance_radius_m": 1.0}
```

Omitting them means `footprint` and radius `0.0`. Default packages remain format
version 1. Verified-area packages use version 2 and carry B's verification record.
A checks cell indices/bounds/uniqueness, inclusion of the complete initial
footprint plus margin, four-neighbour connectivity to that footprint, and that
**all four corners of every cell** lie inside the declared radius. A accepts a
verified subset; the radius does not imply a free circle.

The verification must match the scene, episode, environment seed, start pose,
robot geometry, mapping configuration, mode, radius and exact supplied cell set.
It must assert both clearance and floor support, identify the verifier, method
and scene snapshot, and cover the whole required vertical collision envelope.
These checks validate B's attestation; hashes cannot prove physical safety.

The seed is applied once at reset. It does not count as explored coverage;
subsequent obstacle observations still override seeded free space. Unknown space
outside the seed stays blocked. Replay JSON/events/HTML label the initialization
condition and cell count. Diagnostics test exact-footprint and planner-disc start
usability separately, as well as individual goal validity.

## What B should implement next

1. Freeze the real Locobot geometry and initial settled pose. Review the current
   conservative collision-AABB rectangle; a tighter polygon must still contain
   all relevant robot collision geometry. Keep the native `eyes` camera pose.
2. At the same scene snapshot as the first capture, enumerate candidate **whole
   grid cells** within a declared radius (start with 1.0 m). Test each full cell
   column throughout the required world-z band against non-robot collision
   geometry. A centre-point test, clear ray, or navigation mesh is insufficient.
   Include movable obstacles. Separately verify traversable supporting floor
   across each cell, rejecting holes, unsupported cells and untraversable steps.
   Document the actual method, tolerances, geometry/asset version and snapshot.
3. Keep only cells passing both checks, then retain the component connected to
   the initial footprint. Reject initialization if any required footprint cell
   fails. Never fill rejected cells to complete a circle. The current development
   recorder checks collision columns only; it is **not** this floor-support verifier.
4. Export the manifest, native RGB-D frame directories in capture order, and a
   clearance JSON with exactly `cells`, `provenance`, and `verification`. Produce
   the verification record only after the physical checks succeed. Preserve
   measured poses and timestamps; the first pose must equal manifest.start_pose.
5. Package/replay with A's commands below. Inspect whether the initial and later
   planning starts remain usable and whether there is at least one reachable
   frontier. If they do not, inspect reported blocked cells/height bins before
   changing geometry. A larger initial prior is not evidence for later blind
   areas; do not expand it again during an episode.
6. Once offline acceptance passes, connect the real adapter and demonstrate at
   least two selection/move/arrival-scan cycles. Record collisions, rejected
   paths and map changes. If moving still leaves an unavoidable unseen floor
   gap, bring that evidence back to A/C to decide on sensing or experiment scope.

Use the shared helper to obtain bindings **after selecting physically verified
cells**. It returns requirements and hashes, not a verification result:

```python
from frontier_search.fixtures.clearance import clearance_binding
from frontier_search.fixtures.sequence import mapper_init

binding = clearance_binding(manifest, cells)  # cells: tuple[GridCell, ...]
# B's actual checks must cover binding['required_z_min_m'] through
# binding['required_z_max_m'], over every full supplied cell square.
# Export the following fields using the actual check results:
verification = {
    'context_sha256': binding['context_sha256'],
    'cells_sha256': binding['cells_sha256'],
    'checked_z_min_m': checked_z_min_m,
    'checked_z_max_m': checked_z_max_m,
    'all_cells_clear': all_cells_clear,
    'floor_support_verified': floor_support_verified,
    'verifier': verifier_id,
    'verification_method': actual_method_and_tolerances,
    'scene_snapshot_id': snapshot_id,
}
request = mapper_init(manifest, cells, verification)  # validates before reset
# mapper.reset(request)
```

`checked_z_min_m` must be no higher than the required minimum and
`checked_z_max_m` no lower than the required maximum. Both flags must be true;
all three identity/method strings must be nonempty. Store the detailed check
report alongside the package and reference it in `provenance`. Hash binding
excludes policy settings so the same evidence can be reused for policy comparison.
It is an integrity check, not a signed certificate or an independent verifier.

## A/B replay and acceptance commands

The following filenames are placeholders for B's actual outputs. Use fresh
output directories and pass all frames in chronological order:

```powershell
$env:PYTHONPATH = "$PWD/src"
python -m frontier_search.runner.replay pack --manifest runs/b-start-area/manifest.json --frames runs/b-start-area/frame-001 runs/b-start-area/frame-002 --clearance runs/b-start-area/clearance.json --output runs/start-area-package
python -m frontier_search.runner.replay run --dataset runs/start-area-package --output runs/start-area-replay
python -m frontier_search.runner.diagnose_clearance --dataset runs/start-area-package --output runs/start-area-diagnostics.json
```

A checks repeatability, unchanged observation coverage semantics, blocked-cell
causes and B reachability results. A connected seed alone does not guarantee a
path: the inflated planning footprint still has to fit. Keep failed reports.
No real Rs_int larger-area verification has been fabricated or accepted yet;
current automated coverage uses a labelled analytic floor/wall fixture.

## C's integration requirement

Use the shared `mapper_init(manifest, cells, verification)` guard in the real
factory. The low-level `MapperInit` remains a trusted B-to-A boundary and has no
start pose or physical attestation fields; directly constructing it bypasses
package checks and must not be used to sneak in an enlarged prior. The existing
mock factory is unchanged. Keep scene/start, geometry, radius and verified cell
set identical across compared policies. Report footprint-only and verified-area
runs as separate experimental conditions. This is a documented optional extension
to the frozen baseline's allowed-prior rule, not a new default.
