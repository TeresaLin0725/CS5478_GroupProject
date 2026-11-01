# Reachable frontier approaches

Contract 1.2 adds `Frontier.alternative_approach_poses` (default empty) and
`CandidateAssessment.approach_pose` (default None for legacy producers).
`RGBDMapper` publishes 1.2 states. Existing single-pose fixtures still work.

A enumerates collision-free poses within the existing configured approach-search
distance, through known-free cardinal neighbors of each frontier support. The
preferred pose retains its existing clearance/distance ranking; remaining poses
follow the same deterministic order. IDs and retry history still belong to the
frontier support, not to individual poses. Alternatives refresh on every map update.

B computes one distance field for the decision. It tests the preferred pose and
then alternatives, choosing the first valid pose present in that field. B returns
the exact pose and its path cost. It reports `invalid_goal` only when every offered
pose is invalid, `no_path` when valid poses are proven unreachable, and `timeout`
when assessment is inconclusive. The batch deadline also bounds alternative checks.
A reached cell in a partial field is sufficient positive evidence of reachability.

The runner calls A's `resolve_approaches(state, evaluation)` if B selected a new
pose. A returns an immutable decision view with updated approach, Euclidean
distance, and information gain at the selected pose. Gain is computed only for
changed poses using the latest observation. The mapper's original snapshot and
identity memory remain untouched. Stale mapper snapshots and assessments are
rejected. Replay resolves this same view before policy selection and plotting.

All policies receive the resolved poses and scores. The selected subgoal must
match both the decision view and B's explicit pose; the runner records recovery
and dispatch against that view. B revalidates the actual pose when navigation
starts and stops/replans if a newer map blocks it.

## Validation

`tests/test_alternative_approaches.py` covers disconnected preferred poses,
preferred-pose stability, invalid versus unreachable alternatives, partial fields,
deadlines, stale/invented assessments, updated IG/LLM inputs, map changes, mapper
immutability, identity preservation, and exact runner dispatch.

The recorded F000220 regression now finds a reachable offered alternative,
validates its path, recomputes its gain, and selects that exact pose. Its fixture
is a saved-map geometric check with reconstructed camera bearing; it is not a
native RGB-D replay.

Native validation on 1 November 2026 recovered the apartment and clutter Nearest
tasks from the older B batch, with independent success verification and zero
contact steps. Dynamic Random still ended with no feasible frontiers. Details
are in `runs/approach-native-20261101-03/SUMMARY.md` and
[the shared geometry notes](shared_round_geometry.md).

The first native trial exposed collision-check overhead while enumerating many
poses. `footprint_collides` now skips known-free cells before performing polygon
intersection tests. Its boolean result matches the original full cell-overlap
calculation across 2,214 shape/rotation/margin/map configurations. The interrupted
trial is preserved, and completed runs use a new frozen source snapshot.

```powershell
$env:PYTHONPATH = "$PWD/src;$PWD/tests"
python -m unittest test_alternative_approaches -v
```
