# A: W2 — RGB-D mapping, frontiers and information gain

Implemented on `a-perception`, 10 October 2026. The real robot is still undecided.
`RGBDMapper()` consumes the existing `MapperInit` and `RGBDObservation` records
and synchronously returns a complete immutable `WorldState` for each accepted
frame, including frontiers, approach poses, identity/lineage and predicted gain.
Semantic objects remain empty and `target_claim=None`. Real multiframe recording
and live exploration acceptance remain pending the shared robot/adapter.

## Evidence model and assumptions

- The scene is static on one configured flat floor. Grid geometry is fixed for
  the episode. Robot base elevation must stay constant to 1e-6 m; a change is
  rejected atomically. This explicit restriction keeps accumulated vertical
  evidence in one collision envelope. Real simulator base motion may require
  a validated floor-relative envelope model before integration.
- RGB-D uses aligned `rgb8` and axial metre `depth32f_le` buffers, integer pixel
  centres and capture-time camera-to-world transforms. Intrinsics stay fixed;
  camera transforms and horizontal robot pose can change between captures.
- A valid surface within the configured depth range marks its cell observed.
  A non-floor endpoint within the robot collision-height interval marks an
  uninflated obstacle. Obstacles persist until reset and override all free
  evidence, including spawn seeds. Dynamic obstacle removal is not implemented.
- Free-space rays stop before the measured surface by
  `depth_surface_margin_m` (default 0.02 m axial depth). They are clipped to the
  map and collision interval above the floor tolerance band, then traversed in
  3D with `clearance_bin_height_m` (default 0.05 m). All vertical bins in an XY
  cell must have free-ray evidence, and the cell must have an observed floor
  endpoint, before it can become free. Evidence is accumulated across frames.
  The implementation supports at most 4096 height bins per episode.
- This is a discrete sampled evidence model. A ray through a voxel is evidence
  for that voxel, not proof that every point in its volume is empty. Thin or
  unsampled obstacles remain a sensor-resolution limitation. Bin size, floor
  tolerance and camera sampling need validation on the eventual shared robot.
  A ray above the collision interval or a tabletop alone never establishes free
  space underneath. Footprint inflation and reachability remain B's work.
- Coverage is boolean geometric evidence: accepted surface endpoints and cells
  traversed by rays inside the collision interval. Coverage does not imply
  traversability or target absence. Spawn-clearance seeds alone add no coverage;
  repeating a view with a new sequence does not inflate coverage.

Invalid, nonfinite and out-of-range depth pixels add no evidence. Pixels beyond
the configured maximum are discarded, rather than treated as free rays. An
all-invalid but correctly encoded frame still advances the snapshot revision
and processed-frame metadata while preserving existing geometry.

## State publication and regions

Updates require the current episode, a strictly increasing sequence and a
nondecreasing timestamp. Skipped sequence numbers and equal timestamps are
allowed. Reset and update failures preserve the previous episode/state; a
rejected frame does not consume its sequence. Grids and state have matching
revisions, and state time/pose/sequence exactly match the input frame.

Room coverage uses an even/odd polygon fill at cell centres, including centres
on polygon boundaries. The denominator contains all those centres, and an
empty denominator produces `None`. Room polygons must fit inside the fixed map;
this is checked before the first update publishes a state. Reset remains
compatible with the W1 prior-loading API. Standard room labels stay `None`;
explicit oracle mode preserves labels with their required source annotation.

## Frontier geometry and identity

`perception/frontiers.py` deterministically clusters known-free cells with an
in-map cardinal unknown neighbor. Cluster connectivity is configurable; map
borders alone never create frontiers. Supports and clusters use row-major order.

Each cluster gets a known-free approach pose. A bounded cardinal search retreats
from its support into known-free space, preferring the nearest cell with room
for the footprint's circumscribed disc plus margin. A Chebyshev distance field
provides a conservative clearance estimate only for pose placement; occupancy
is never inflated. If no such interior cell exists, the best known-free pose is
still published for B to assess. A does not assert reachability. The camera's
capture-derived mounting yaw is accounted for when facing the unknown side.
Region association uses the majority of support-cell membership, with room-ID
tie-breaking. Euclidean distance is diagnostic; C Nearest uses B's path cost.

Exact support matches take precedence, preventing nearby unchanged clusters
from repeatedly appearing to split/merge. Remaining clusters are matched by
support overlap with a configured Chebyshev tolerance. A match requires the
configured fraction of either cluster to overlap the other's dilated support.
One-to-one matches preserve IDs. Splits/merges allocate monotonically increasing
IDs and reference all directly matched parents from earlier snapshots. Retired
parents no longer compete with descendants. Active IDs do not expire; unmatched
records can reappear within the history window measured from their last sighting.
Expired supports are removed, while the ID serial and compact lineage persist
until reset. Candidate ordering and matching ties are deterministic.

C retains episode-long spatial attempt history independently. Tests feed actual
A-generated split/merge lineage to `SpatialRecovery` and verify that exhausted
attempt budgets remain exhausted. A's detailed-history expiration does not
authorize C to discard its recovery records.

## Information gain

`perception/gain.py` estimates unknown area in square metres from the observed
occupancy grid. It samples the calibrated horizontal FOV at the approach pose
and through the shortest turns of the configured arrival sweep, including the
initial heading. Camera mounting translation rotates with the robot. Range is
radial and capped by `depth_max_m`. Occupied cells stop rays; unknown cells remain
potentially visible. A set deduplicates cells across rays and headings. Corner
touching occupied cells block diagonal visibility.

This is a planar, sampled prediction for a roughly upright camera; it does not
model vertical visibility, hidden obstacles, future detections or navigation
reachability. A vertical optical axis produces zero planar gain. All policies
receive the same estimate and configuration.

## Configuration and verification

The shared `MappingConfig` includes the two mapping settings above and these
frontier/gain parameters under `mapping` in an episode manifest:

| Parameter | Development default | Meaning |
| --- | --- | --- |
| `frontier_min_cells` | 2 | Minimum cluster support size |
| `frontier_connectivity` | 4 | Cluster adjacency, 4 or 8 |
| `frontier_approach_search_m` | 1.0 | Maximum inward cardinal search distance |
| `frontier_match_tolerance_cells` | 1 | Chebyshev support dilation radius |
| `frontier_match_fraction` | 0.25 | Minimum overlap fraction of either cluster |
| `frontier_history_s` | 30.0 | Dormant matching window in simulation seconds |
| `gain_range_m` | 3.0 | Planar prediction range, capped by depth maximum |
| `gain_ray_step_rad` | 0.05 | Maximum angular spacing for rays and sweep samples |

Existing manifests load the defaults for omitted fields.
`configs/w2.mock.json` records the development defaults explicitly, although its
scripted mapper does not perform RGB-D fusion.
Collision bounds come from `MapperInit.robot_config`, relative to capture-time
base height. Tests use explicit development geometry, not a selected real robot.

From the repository root:

```powershell
$env:PYTHONPATH = "$PWD/src"
python -m unittest discover -s tests -p test_mapper_update.py -v
python -m unittest discover -s tests -p test_mapping_rays.py -v
python -m unittest discover -s tests -p test_frontiers.py -v
python -m unittest discover -s tests -p test_information_gain.py -v
python -m unittest discover -s tests
```

The analytic RGB-D renderer in `tests/test_mapper_update.py` provides known
floor/wall/box scenes. Tests cover low obstacles, tabletops, alternative robot
height bounds, translated floors, invalid depth, stationary/moving/scanning
views, atomic failures, room coverage, reset and immutable snapshots. A sequence
is saved and loaded through `fixtures.recording` and must reproduce exactly the
same states. This is synthetic regression evidence; a real multiframe recording
and real-robot calibration remain A/B integration tasks.

`test_frontiers.py` also runs an analytic RGB-D frame through the real mapper,
B's `TeamNavigator.evaluate_candidates`, and C's Nearest and Information Gain
policies. It contains both reachable and rejected A-generated candidates and
checks that C only selects eligible IDs at the assessed revision. Simulator
control is mocked for this offline assessment test; it is not a live episode.

A local Python 3.11 timing smoke check processed one 640x480 analytic frame in
about 3.48 seconds including frontier/identity/gain (500 map cells, 3 candidates).
This standard-library implementation streams
projected points and avoids allocating per-pixel immutable records. It is still
too slow for high-rate perception; real capture cadence and Python 3.8 timing
need validation before the real runner is wired. Synthetic timing is not a
real-simulator performance measurement.

The [portable RGB-D replay tool](w2_rgbd_replay.md) now generates a labelled
synthetic moving/scanning sequence and checks A → B assessment → C selection
twice for reproducibility, with maps and a report. It also packages existing
per-frame captures with the shared manifest and spawn-clearance provenance.

Next: agree on the real robot/sensor/map configuration, deliver a real recording,
run the same replay on it, and wire A/B into C's supervised runner factory. C's
W2 mock entry point still uses its scripted mapper; these changes do not turn it
into a real episode.
