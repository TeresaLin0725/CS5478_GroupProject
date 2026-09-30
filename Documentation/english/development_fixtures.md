# Mock Data and Independent Development

Development guidance for the robot project. The [Interface Freeze](interface_freeze.md) defines the version 1.0 contracts used by these fixtures.

B and C can develop against fixtures before A's mapper is ready. Fixtures must use the same data contracts as real module outputs, including episode IDs, revisions, coordinates, occupancy values, and immutable snapshots. Mock values are deliberately authored inputs; they do not need to be produced by a perception or planning algorithm.

## 1. B: synthetic grids and navigation requests

B needs a grid, robot configuration, start pose, and candidate approach poses or a selected subgoal. Begin with small deterministic grids that make expected routes and failures easy to inspect. For example:

```python
import numpy as np

cells = np.full((40, 40), -1, dtype=np.int16)  # unknown
cells[5:35, 5:35] = 0                         # known free
cells[5:35, 20] = 100                         # wall
cells[18:23, 20] = 0                          # doorway

mock_grid = OccupancyGrid(
    frame_id="world",
    revision=1,
    resolution_m=0.1,
    origin_x_m=0.0,
    origin_y_m=0.0,
    width_cells=40,
    height_cells=40,
    data=tuple(tuple(int(value) for value in row) for row in cells),
)
```

This fixture is 4 m by 4 m, with a five-cell (0.5 m) doorway. Whether the robot fits depends on its footprint and clearance margin. Convert cell centres using [the shared conventions](interface_freeze.md#2-shared-conventions-and-priors); row/column indices are not world coordinates. Keep the fixture uninflated so B applies its own costmap rules.

For planner-only checks, start and goal cells on opposite sides of the wall exercise the detour. For full frontier integration fixtures, also provide consistent frontier support cells, approach headings, regions, and explored coverage. Wrap these in WorldState for evaluate_candidates, or supply a NavigationRequest for selected-goal execution. Geometry alone can be tested without running iGibson; physical path following requires the simulator or a control-adapter substitute.

Required fixture cases include open space, a wall detour, a doorway too narrow for the chosen robot, a route separated by unknown space, and a newer map that blocks an active route. Add observation-motion and cancellation cases when those APIs are implemented.

## 2. C: authored WorldState and mock B assessments

C can start with hand-authored WorldState and SelectionContext records. Include a target, robot pose, consistent maps, region/object summaries, frontier poses/gains, and a CandidateEvaluation with matching episode, revision, and start pose. Mock B's path costs and reachability directly; C does not need to implement a planner or frontier extractor to produce its inputs.

Example selection fixture for target **cup**:

| Frontier | Observed objects in its region | Mock path_cost_m | information_gain_m2 | reachable |
|---|---|---:|---:|---|
| F1 | Sofa, TV | 2.0 | 6.0 | True |
| F2 | Sink, refrigerator | 5.0 | 20.0 | True |
| F3 | Bed | None | 12.0 | False |

Give F3 failure_reason=NO_PATH and leave successful assessment reasons as None. With no other exclusions, eligible_frontier_ids contains F1 and F2. Nearest selects F1; Information Gain selects F2 for any positive epsilon_m; Random chooses only an eligible ID. The LLM may prefer F2 but is not required to do so: test eligibility and response handling separately from semantic preference.

Also provide fixtures for no frontiers, all candidates excluded, a terminal target claim, inconclusive evaluation, and prior navigation failure. Use a stub LLM response for deterministic malformed/out-of-set/timeout checks, and separate live calls for integration. In a full runner fixture, mock the evaluator separately; a target claim is never automatic success.

## 3. iGibson-derived development fixtures

iGibson can provide realistic geometry and reference outputs while A is being built. The following helpers are present in the upstream source; pin the installed version and verify availability and conventions before writing the adapter.

| Facility | Development use | Adapter considerations |
|---|---|---|
| scene.floor_map[floor] | Load scene traversability as a planner fixture | This is scene-derived traversability, not A's accumulated occupancy; record whether erosion and connectivity filtering have already been applied |
| scene.get_random_point(floor=...) | Sample development start/goal positions | Save sampled coordinates and seed; validate actual robot clearance before execution |
| scene.world_to_map(...) and scene.map_to_world(...) | Convert scene-map and simulator coordinates | Adapt axis order, origin, resolution, and cell-centre convention to this contract |
| scene.get_shortest_path(...) | Reference paths/distances for debugging | Uses the scene graph; compare only under matched geometry and motion assumptions, not as a definitive collision-free oracle |

The [IndoorScene implementation](https://github.com/StanfordVL/iGibson/blob/master/igibson/scenes/indoor_scene.py) loads traversability maps, may erode free space and retain the largest connected component, and supplies these helpers. Its shortest-path routine may connect off-graph endpoints to nearby graph nodes. Therefore, do not use its output as proof that your robot can execute a route, and do not inflate an already eroded fixture a second time without accounting for that preprocessing.

For a binary traversability fixture, explicitly translate traversable cells to 0 and non-traversable cells to 100, documenting that non-traversable does not necessarily mean a sensed physical obstacle. To construct partial-knowledge fixtures, apply a separately defined observation mask and set all unrevealed cells to -1. Do not accidentally mark every full-map cell explored. A manually chosen mask is useful for development but is not an RGB-D observation model.

iGibson also exposes an occupancy_grid observation through its ScanSensor pipeline, as shown in the [environment implementation](https://github.com/StanfordVL/iGibson/blob/master/igibson/envs/igibson_env.py). This differs from scene.floor_map and from the planned RGB-D mapper. Check sensor assumptions, value encoding, frame, resolution, and update behavior before adapting it; it is not a drop-in WorldState grid. Replacing RGB-D mapping with scan-based mapping in experiments would require an explicit shared sensor-design change.

## 4. Fixture ownership and transition to integration

- B owns synthetic navigation grids and the scene-to-fixture conversion; C owns policy-state fixtures and mock candidate assessments. A reviews that fixture fields conform to the mapper contract.
- Store fixtures behind a common loader, with metadata such as fixture_id, source (synthetic, simulator_oracle, or recorded_robot_state), schema_version, seed, and any preprocessing. These are fixture metadata, not extra policy features.
- Keep development fixtures and their results separate from the main experiment manifest. Full scene maps and built-in paths are permitted for development/debugging; standard experiments use A's robot-built map and B's team-implemented planner, with only the [declared priors](interface_freeze.md#2-shared-conventions-and-priors).
- Replace the fixture provider with A's output and B's real assessments without changing C's select interface. Recorded robot-state snapshots are useful for replaying actual failures once integration begins.

```text
Stage 1: synthetic grids / authored states and assessments → independent B and C development
Stage 2: adapted iGibson scene fixtures → realistic geometry and simulator integration
Stage 3: A's robot-built WorldState → B's assessments → C's selection → B's execution
```
