# Robotics Project — Detailed Implementation Plan

## 1. Project Goal and Scope

Compare **Random, Nearest, Information Gain, and LLM-guided frontier selection** for indoor object search. The initial task is to **visually detect a target class**, such as a cup. Approaching, grasping, and manipulating the object are outside the initial scope.

The standard experiment is **object search with a known room-layout prior and known robot pose**. Use one floor, a fixed RGB-D camera, and a static scene initially. Simulator poses and calibrated camera transforms are permitted inputs; SLAM is not required. Complete room polygons reveal the layout and extent of unseen rooms, but do not provide obstacle occupancy or object locations.

All policies share perception, candidate-generation rules, feasibility filtering, observation routines, navigation, and budgets. Given the same state, they receive the same eligible candidates. Actual maps and frontier sets naturally diverge after policies take different routes.

The behavioral contracts in [Interface Freeze](interface_freeze.md) are authoritative. 
## 2. Work Division

| Owner | Responsibilities | Boundary |
|---|---|---|
| A — Perception & Mapping | RGB-D, occupancy/coverage, object tracks and target claims, room association, frontiers and information gain; evaluation-only target verifier | Does not select destinations or expose evaluator ground truth to the robot |
| B — Navigation & Planning | Simulator adapter, transforms, control, navigation costmap, candidate feasibility/cost, planning, execution, fixed observation motion | Does not rank frontiers by search value |
| C — Search Strategy & Runner | Four policies, episode runner, recovery bookkeeping, experiment configuration, logging and comparisons | Does not construct maps or implement path planning |

C owns end-to-end integration and the runner. B owns simulator stepping and control integration; A owns the independent target verifier. All three review contract changes and validate their boundaries.

## 3. System Architecture

```text
Runner initializes modules, priors, seeds and budgets
    ↓
B executes initial observation routine; A processes RGB-D
    ↓
A publishes WorldState: map, objects, regions and frontiers
    ↓
B evaluates candidate feasibility and path costs
    ↓
Runner applies shared recovery exclusions
    ↓
C selects one eligible frontier
    ↓
B navigates; A observes; runner enforces limits
    ↓
B completes arrival observation routine; A publishes fresh state
    └── repeat

Target claim at any point → stop motion → isolated verifier
                         → success or false_target_claim
```

The runner carries `WorldState`, `CandidateEvaluation`, `SelectionContext`, `Subgoal`, and `NavigationFeedback` between modules. Candidate evaluation does not move the robot or choose a destination. Planning feasibility, completed execution, and correct target detection are separate outcomes.

## 4. A — Perception & Mapping

### RGB-D and traversability

Capture RGB-D during motion and fixed observation routines. Perception may run below simulator frequency; each processed frame retains its sequence, simulation timestamp, and capture-time transform.

A builds unknown, known-free, and occupied cells. Unknown cells are not traversable. Use calibrated depth, floor estimation, and the robot's vertical collision envelope to derive occupancy. Seeing a tabletop or a ray passing above a low obstacle does not establish robot clearance underneath. Invalid depth and unobserved volume remain unknown. Configure floor tolerance, depth range, height filtering, and evidence thresholds identically across policies.

A publishes physical occupancy evidence; B derives a separate footprint-aware navigation costmap. Do not bake robot inflation into A's map.

### Coverage and room priors

Maintain geometric observation coverage separately from occupancy. `explored_fraction` is the fraction of cells inside a room polygon with accepted geometric observations. It is **not** evidence that all tables and shelves were inspected or that the target is absent. Object-inspection coverage is optional future work.

Complete room polygons and simulator poses are the standard allowed priors. Use opaque room IDs without semantic labels. Objects, occupancy, and frontiers come from robot observations, except the explicitly permitted initial footprint-clearance seed. Semantic room labels are hidden unless an oracle-label experiment enables them equally for all policies. The verifier may access hidden data through its isolated evaluation interface only.

### Objects and target claims

Use RGB detection, depth, and capture-time transforms to associate detections into tracks. Check detector labels and performance on the selected targets and rendered scenes in W1.

Multi-frame confirmation is mandatory. Configure confidence, distinct-frame count, temporal window, and spatial consistency. Reprocessing one frame cannot increase confirmation. A confirmed track produces a `TargetClaim` with track ID and evidence observation IDs.

The robot stops on the claim. The evaluation-only verifier checks that the evidence corresponds to a visible instance of the requested class under a fixed matching rule. A false claim ends the episode as `false_target_claim`; the verifier does not tell the policy to resume. Ground truth therefore cannot guide subsequent search.

### Frontiers and information gain

Extract and deterministically cluster boundaries between known-free and unknown cells. Choose approach poses on the known-free side, with arrival heading toward the unexplored side. B still verifies full-body clearance and reachability.

Estimate `information_gain_m2` using camera range, field of view, approach heading, and the fixed arrival sweep. Count each predicted visible unknown cell once and stop rays at known occluders. Hidden obstacles remain unavailable, so gain is an estimate. Fix the estimator and parameters across policies.

Preserve IDs for matched frontier clusters. Record parent IDs on splits/merges and expose cell support so recovery history transfers by spatial overlap. New IDs must not erase failure/no-progress history. The interface contract defines matching and retry behavior.

“Continue deeper into this room” i an interpretation of selecting a frontier, not a special low-level action.

## 5. B — Navigation & Motion Planning

### Evaluate before selection

`B.evaluate_candidates(...)` returns reachability and path-length estimates from the current pose on one map revision. Use the execution planner's motion model and costmap. A grid distance field may amortize work for a suitable robot; otherwise use bounded per-candidate planning. Evaluation timeout means unknown feasibility, not proven unreachability.

All policies select from assessed reachable candidates after shared recovery filtering. B never chooses the most useful frontier. Revalidate planning when execution starts because map or pose may have changed.

### Navigation geometry

Implement the planner within the team and use iGibson as the simulator. A* is suitable for a circular robot capable of turning in place. Otherwise use a planner that respects the chosen robot's constraints.

Freeze footprint, collision height range, clearance margin, motion limits, position/yaw arrival tolerances, and map resolution before integration. Use conservative obstacle inflation for a circular footprint or oriented footprint checks. Check full path segments and rotations; prevent diagonal corner cutting and footprint overlap with unknown space.

Only verified initial footprint clearance may seed the starting traversable area, under the documented spawn-clearance prior. Do not expose additional simulator free space. Scans also require collision checks.

### Execution and failure

B executes the selected goal, accepts updated maps, and may replan to that same goal. Report planning results separately from terminal execution status.

The baseline includes `NO_PATH`, `BLOCKED`, `TIMEOUT`, and `INVALID_GOAL`. Detailed geometric evidence and LLM-specific failure reasoning remain optional. Operations have deadlines; cancellation is idempotent, including before a plan exists.

## 6. C — Search Strategy

The runner supplies state, assessments, and eligible IDs. Policies differ only in selection. Stable ordering and deterministic tie-breaking use frontier ID; Random has its own recorded RNG stream.

| Policy | Selection rule |
|---|---|
| Random | Uniform choice among eligible candidates |
| Nearest | Minimum assessed path length |
| Information Gain | Maximum `information_gain_m2 / (path_cost_m + epsilon_m)` |
| LLM-guided | Select an eligible ID from target, observed objects, region summaries, gain, path cost, and shared recovery context |

Fix positive `epsilon_m` in configuration. Euclidean distance is diagnostic, not the standard Nearest baseline.

The LLM returns an eligible ID; its adapter constructs the subgoal from A's approach pose. Unknown room labels remain unknown, although the model may reason about likely types. Do not supply pixels, hidden object data, evaluator results, or semantic labels outside the oracle condition.

Enforce a wall-clock deadline and bounded calls. Invalid, out-of-set, or timed-out responses use deterministic Nearest fallback, with the reason logged. Record exact model identifier, supported reasoning configuration, prompt, symbolic input, response, and fallback.

## 7. Fixed Observation and Continuous Perception

The camera stays fixed on the robot. Ordinary path following alone does not guarantee sufficient views. The runner invokes the same configured routine for every policy:

1. At initialization, perform a collision-checked heading sweep. Use a full rotation only if robot constraints and clearance permit it.
2. On frontier arrival, attain the required heading toward unknown space and perform the configured bounded sweep.
3. Wait for A to process fresh frames from that observation motion before evaluating candidates again.
4. Before declaring no frontiers or no feasible frontiers, allow one bounded recovery observation cycle, then refresh state and assessments.

B executes these routines through `start_observation`; C does not select scan actions in the baseline. Sweep angles, sampling/dwell requirements, and limits are fixed configuration shared by all policies. Unsafe or timed-out scans yield typed failures and cannot loop indefinitely.

Here, **baseline** means the first implementation used to compare the four frontier-selection policies. Looking around is an automatic part of the shared robot behaviour. The runner decides when the prescribed routine is due, B controls the robot's turning, and A processes what the camera sees. The camera remains fixed to the robot; `start_observation` requests observation motion, not object detection or a separate camera shutter action.

For example, after selecting a frontier:

1. **C chooses F3:** “Go to this frontier.”
2. **B navigates to F3** and reports arrival.
3. **The runner calls `B.start_observation(...)`** for the arrival routine; B turns through the predefined scan angles when safe.
4. **A processes the camera observations** and updates the map and detections. The runner waits for the required fresh observations and obtains updated candidate assessments from B.
5. **C chooses the next frontier**, unless a target claim or another stopping condition has ended the search.

Every policy gets the same observation rules, so the experiment compares frontier choices without also changing how scanning is selected. Letting C choose “scan here again” or “inspect this area” is an optional later extension that expands the policy's action space beyond frontier selection.

Perception continues during navigation. Target claims cause immediate cancellation. Policy decisions occur at initialization, after arrival observations, or after failure/recovery, rather than every image frame.

## 8. Recovery, Progress, and Episode Control

The runner applies shared rules:

- `NO_PATH`/`INVALID_GOAL`: suppress until relevant traversability, approach geometry, or robot connectivity changes.
- `BLOCKED`/`TIMEOUT`: configured cooldown and bounded retries, followed by fresh feasibility assessment.
- Transfer suppression and retry history across overlapping splits/merges. A global revision increment alone does not justify retrying geometric failures.
- After arrival, compare local unknown-area reduction with a configured progress threshold. Suppress reached frontiers that reveal no new area; repeated no-progress attempts terminate within a fixed budget.
- Enforce per-episode attempt, simulation-step, simulated-time, and wall-clock limits. Map changes cannot reset global caps.
- Revalidate stale selections before dispatch. Refresh invalidated candidates instead of substituting arbitrary coordinates.

If no eligible candidate remains, perform the bounded recovery observation, honor pending cooldowns within budget, and reassess. Only then terminate as `no_frontiers`, `no_feasible_frontiers`, or `search_limit`. Frontier exhaustion does not prove object absence.

## 9. Experimental Protocol

Freeze a manifest before main runs:

- Paired scene/start-pose/target-instance configurations across policies, including initial heading. Evaluator-only checks establish that targets are present and visible from a reachable viewpoint.
- Repeated seeds; independent RNG streams for environment, policy, and stochastic perception. Fix scene assets, detector weights, robot/camera configuration, and map parameters.
- Common simulator/perception cadence, observation routine, candidate extraction, feasibility limits, recovery, and episode budgets.
- Standard known-room-layout, known-pose, and spawn-footprint-clearance priors. Report oracle semantic labels separately.
- Pause simulation with the robot stopped at decision boundaries, including candidate evaluation and LLM calls, for every policy. During navigation/scans, simulation and perception continue. Measure decision wall time separately and enforce wall-clock limits.

Report success rate over **all** episodes, false-claim rate, termination reasons, actual executed path length, simulated search time, wall time, collisions, replans, no-progress attempts, and frontier visits. Report success-conditioned efficiency with sample count; do not silently drop failures. Use paired comparisons and uncertainty intervals over repeated trials.

For LLM runs, report call count, latency, tokens, fallback rate, and cost when available. Preserve raw logs and manifest. A seed alone does not establish exact hosted-model reproducibility.

Optional ablations: remove semantic evidence from LLM input, add a fixed semantic heuristic baseline, or enable a separately labeled oracle-room-type condition. Complete the four core policies first.

## 10. Implementation Priorities

| Priority | Features |
|---|---|
| Must work | RGB-D mapping, tracked frontiers, multi-frame target claims, isolated verifier, footprint-aware planner, candidate evaluation, fixed observation, bounded recovery, four policies, automated runner/logging |
| Should work | Reliability across several scenes, failure analysis, repeated paired trials and uncertainty estimates |
| If time allows | Detailed planner evidence, semantic ablations, policy-selected inspection, mid-route high-level reconsideration |

The baseline is frontier-based visual search, not exhaustive inspection. Geometrically explored scenes with an unseen target are a recorded limitation, not a reason to reveal hidden simulator information.

## 11. Six-Week Timeline

| Week | A — Perception & Mapping | B — Navigation & Planning | C — Policies & Runner | Milestone |
|---|---|---|---|---|
| W1 | RGB-D conventions, prior loader, detector feasibility, mapper reset | Simulator stepping/transforms, robot geometry, initial controls | Contract fixtures, runner skeleton, mock Random/Nearest, experiment manifest | Configuration/interfaces agreed; observation passes through modules |
| W2 | Occupancy/coverage, frontiers/identity, gain | Planner, costmap, batch assessment, observation motion | Runner/recovery, Information Gain, LLM adapter | Map expands and common feasible candidates feed navigation |
| W3 | Tracks, claims, room association, verifier | Continuous map updates, failures, cancellation | Integrate all policies, limits/logs | One scene completes independently scored search |
| W4 | Mapping/detection fixes | Clearance, stale goals, blocked paths/timeouts | Paired pilots, budget/fallback validation | Reliability cases pass; freeze main settings |
| W5 | Main runs and perception analysis | Main runs and navigation analysis | Core comparisons, optional ablations after results secured | Complete experiment dataset |
| W6 | Perception/evaluation report sections | Navigation report and demo support | Analysis, reproducibility package, report integration | Final report and demonstration |

W4 cases: target in starting room/another room; target seen during travel; target initially behind robot; repeated false detections; narrow doorway; unreachable/temporarily blocked frontier; split/merge; arrival without new observations; cancellation before planning; LLM timeout/invalid output.

Avoid major algorithmic changes in W6. Develop modules against synthetic state, assessment, subgoal, and feedback fixtures during W1–W2 rather than waiting for one another.

## 12. Example End-to-End Scenario

The target is a cup. After the initial observation routine, A observes a sofa and table in R1 and creates two frontier approach poses. B evaluates both; C selects one under its policy.

B navigates while A maps. At a doorway, A observes a sink and refrigerator in R2. After arrival, B faces unknown space and performs the fixed sweep. A processes fresh frames; B refreshes feasibility/cost. The LLM may prefer R2 based on observed objects; Nearest minimizes path cost; Information Gain maximizes gain/cost.

During the next route, A confirms a cup track across distinct frames and emits a claim. The runner cancels navigation, freezes evidence, and invokes the isolated verifier. A matching visible cup yields `success`; a false detection yields `false_target_claim`. Both remain in results with distance, timing, and fallbacks.
