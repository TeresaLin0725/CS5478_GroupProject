# Shared A/B round geometry

New paired benchmarks use `navigation_preset: shared_round`. Their serialized
robot manifest contains B's existing 16-sided footprint (vertex radius 0.19 m)
and 0.01 m clearance. Collision heights, motion limits, and other robot settings
are preserved. Mapping evidence rules, obstacle clearing, and policies are unchanged.

A's mapper, B's navigator, and the simulator's spawn verification receive this
same manifest robot. B keeps its 0 m start margin for initial escape checks, but
does not override the footprint or ordinary clearance. Native preflight loads
and verifies the checksum of a frozen job manifest rather than rebuilding the
old square configuration. A mislabeled `shared_round` manifest is rejected.

Existing `legacy` and `round_footprint_override` protocols retain their previous
behavior. Existing run directories and recorded manifests are not migrated.
The standalone `manifest_for` default remains unchanged; the new geometry is
applied by the paired benchmark factory before freezing and hashing manifests.

## Validation on 1 November 2026

The targeted regression selection passed 146 tests across shared geometry,
frontiers, mapper updates, navigation, soft-contact options, adapter logic, and
paired benchmark behavior. Adapter tests use a fake backend, not native physics.

```powershell
$env:PYTHONPATH = "$PWD/src;$PWD/tests"
python -m unittest test_shared_round_geometry test_frontiers test_mapper_update test_navigation_b test_soft_contact test_igibson_adapter test_paired_benchmark -v
```

The checked-in `tests/fixtures/shared_round_frontier.json` preserves the final
map and F000220 support from `b-benchmark-20261022/dynamic-s17-random`, with the
source log hash. Its camera yaw offset is reconstructed from the logged frontier
orientation; it is a geometry regression, not raw RGB-D replay.

With the old square robot, today's A rejects F000220 entirely. Shared round
geometry produces a collision-free approach, but the selected pose is still
disconnected. Five alternative approach cells were reachable in the diagnostic.
The geometry-only test checks that safe, reachable alternatives exist and that
adding an occupied cell invalidates an overlapping pose. The subsequent
alternative-approach implementation now lets B select a reachable pose from A's
ranked alternatives; A recalculates gain for that pose before C selects a frontier.
`test_alternative_approaches` verifies the complete F000220 selection on the saved
map, plus runner dispatch on a synthetic episode. Native results are recorded below.

The local `runs/shared-round-validation-20261101/diagnostics.json` contains
saved-map checks for the four historical failures. These reuse recorded frontier
supports, excluding navigation probes and fresh RGB-D mapping. They are not
episode success or collision measurements.

## Native experiments on 1 November 2026

An existing Conda environment at `C:/Users/dance/miniconda3/envs/igibson/python.exe`
was subsequently found. It passed native rendering/dependency checks and 100
targeted regression tests under Python 3.8.20. All three scene preflights passed.

Three baseline tasks from the older B batch were rerun with frozen current code:

| Task | Previous outcome | Current outcome | Simulation seconds | Contact steps |
|---|---|---|---:|---:|
| apartment-s17-nearest | no_feasible_frontiers | success | 161.33 | 0 |
| clutter-s43-nearest | no_feasible_frontiers | success | 247.67 | 0 |
| dynamic-s17-random | no_feasible_frontiers | no_feasible_frontiers | 87.33 | 0 |

Both successes passed the independent near-target, stationary, distinct-frame,
visibility and stopped-state checks. The remaining dynamic failure has one
occupied cell overlapping the final start footprint. Preflight geometry also
marks that cell occupied; the evidence does not establish a stale-cart error.

Local evidence is in `runs/approach-native-20261101-03/`: `SUMMARY.md`,
`comparison.json`, `verification.json`, native summaries, and frozen inputs.
The earlier `-01` setup error and `-02` interrupted performance trial are retained.
These are three selected tasks, not the full paired suite or an isolated ablation
of alternative poses. Shared geometry, first-move handling and alternative
selection differ from the historical baseline. The LLM task and complete paired
suite remain untested here.
