# How the system works

## Shared search loop

`EpisodeRunner` coordinates one perception, navigation, selection and verification pipeline. Random, Nearest, Information Gain and LLM-guided policies receive the same public candidate representation and share the robot, mapping, navigation, recovery, observation and evaluation code. They differ in how they select exploration frontiers.

1. Capture calibrated RGB-D observations from the native Locobot camera in the Rs scan.
2. Fuse depth into an occupancy map and track detector observations. Extract frontier clusters and assess reachable approach poses.
3. Build a decision context containing eligible candidates, assessed path costs, predicted information gains, observed object classes/positions, region coverage and previous execution feedback.
4. Select a frontier. Navigate using the shared planner and controller; refresh the map and replan when needed.
5. At arrival, collect scheduled observations. If the target is detected, approach and collect stationary close-range evidence.
6. Verify a target claim independently or continue until a stop condition or budget is reached.

The task is visual object search and verification, not grasping. Known robot pose and predefined region polygons are permitted inputs. Hidden target positions, evaluator records and full-scene reachability oracles are not policy inputs. Standard region labels do not directly reveal room types; the LLM infers semantics from observations.

## Mapping and candidate generation

`src/frontier_search/perception/` maintains occupancy, coverage, semantic tracks and frontier identity. Unknown, free and occupied cells are distinct. Predicted gain estimates visible unknown area from candidate viewpoints; known obstacles block rays. Candidate approach poses must satisfy robot-footprint and navigation constraints. A shared round footprint keeps mapping and navigation geometry consistent.

Optional range-gated depth filtering reduces depth outliers. Sensor noise uses configured streams without changing the global random state. The robust bowl configuration enables a 5x5 depth filter, unvisited recovery viewpoints and an inspection-heading settling ratio of 0.5.

## Frontier policies

- Random samples uniformly from eligible frontiers using its policy seed.
- Nearest minimizes assessed path length.
- Information Gain maximizes information gain divided by path cost plus a small epsilon.
- LLM-guided sends a symbolic decision context to a compatible hosted endpoint and validates the returned frontier identifier. It records model settings, response, rationale, usage, latency and fallback reason. A separate process enforces an absolute transport deadline.

The LLM chooses exploration goals. Target approach, motion control, collision checks, recovery limits and target verification remain shared. Missing credentials or invalid/timed-out responses trigger a logged deterministic fallback. Local seeds cannot guarantee reproducible hosted-model samples.

## Navigation and recovery

`src/frontier_search/navigation/` assesses paths, controls wheel motion and executes observation routines. New map evidence can invalidate a route or goal; the navigator reports failure context to the runner, which can request a different frontier. Recovery tracks spatial locality, retry counts and cooldowns rather than endlessly retrying blocked goals.

A recovery-only straight-egress primitive can leave unknown cells already intersecting the robot footprint. It does not rewrite the map or permit entering new unknown/occupied cells, and stops on contact, drift or timeout. Optional epsilon/stuck retries are explicit configuration choices.

## Independent target verification

A semantic track or policy claim alone is not success. Verification requires separate retained-frame evidence of the target instance at close range while the robot is stationary. Detection confidence, geometry, timestamps and instance matching are checked independently. The evaluator's target truth is reserved for verification and retrospective analysis.

Episode limits bound simulated time, wall time, attempts, control steps and decisions. Results preserve failures and fallbacks alongside successes. Simulation pauses during model inference, so simulated-time efficiency and wall-clock speed are different measurements.

## Code and outputs

Shared immutable records live in `contracts.py`; configuration dataclasses live in `config.py`. `runner/manifest.py` validates episode manifests, and `runner/integrated.py` wires the native components. The single production mapper is `perception/mapper.py`; `simulation/igibson_rs.py` supplies the shared native world and renderer. `scripts/run_paired_benchmark.py` freezes a fresh source/harness snapshot and paired manifests before suite execution.

A run exports its manifest, event log, result summary, model-call records and retained evaluation evidence. A recording adds MP4 video and frame metadata. Committed reports provide selected experiment evidence; environment assets and newly generated raw runs stay local. See [reproduction instructions](reproducing.md) for exact historical-replay limits.
