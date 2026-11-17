# Latest benchmark failure investigation

Investigated 2026-11-16 using the original episode events, final states, summaries, scene trajectory, and frozen source for `bowl-48-noise-20261110-02`. Original benchmark outcomes are preserved. This investigation did not rerun the simulator or change implementation code.

## Findings

| Cases | Confirmed failure mechanism | Confidence |
|---|---|---|
| Kitchen and bathroom / clutter / start B / Nearest | Current pose becomes unusable in the observed navigation map; all 21 candidates fail from that start; recovery observation also fails | Direct logged diagnostics |
| Bathroom / apartment / start A / Random | Repeated blocked observation and negligible progress at F000192, with retry history escaping enforcement through spatial-group merging; ends at wall-time budget | Logged loop plus frozen recovery-code replay |

## Two Nearest cases: the same local navigation failure

Both runs have identical selected frontier IDs, operation terminal poses/times, distance (2.419642 m), and termination simulation time (76.666667 s), despite different bowl locations.

1. At 11.47 s, choose F000037; reach it at 20.0 s and finish observation at 33.37 s.
2. Choose F000064 at (-0.15, 0.75); reach it at 40.33 s. Its arrival observation fails `blocked` at 51.77 s. Recovery observation finishes at 75.67 s; local new known area is only 0.02 m².
3. Choose F000081 at (0.75, 1.55). Initial planning succeeds, but updated-map replanning fails `no_path` at 76.67 s, with the robot at approximately (-0.223, 0.657).
4. All 21 remaining candidates are assessed `no_path`. The recovery observation immediately fails `blocked` without advancing simulation time. Final diagnostics label **every candidate `start_unusable`**, with `strict_start_usable=false`.

This is more specific than “no frontiers left”: candidates still exist, but the planner cannot use the robot's current pose. Frozen diagnostics also try reduced inflation and zero start margin before assigning `start_unusable`, so this is not established as merely an excessive strict clearance margin. The trajectory ends beside the added clutter in the living area.

The logs do not identify whether the invalid start originated in depth-map artefacts, physical clearance, or pose/grid discretization. Zero recorded collisions does not settle that question. Whole-scene target inaccessibility is unsupported: Random, IG, and LLM all succeed on both matching tasks. Neither failed Nearest final state contains a bowl track.

Relevant frozen code: `navigation/diagnostics.py:88–135`, `navigation/navigator.py:369–383`, `runner/episode.py:369–398` (paths relative to the run's `frozen/src/frontier_search`). Recovery here rescans at the same pose; it does not retreat to a previously usable pose. Both epsilon retry and stuck retry are disabled in the manifest.

## Random: recovery history fails to constrain a repeated frontier

The robot makes useful early progress, then selects F000192 seven times: at 153.73, 191.63, 361.00, 417.57, 457.40, 495.67, and 534.73 simulated seconds. Its approach stays near (-0.15, 1.95). Six completed visits report only 0.01 m² total local gain; each of those visits has failed arrival observation. From 361 s onward, F000192 is the only eligible candidate. The final visit is interrupted by the budget before progress feedback completes.

A bowl track appears at 251 s. Two target-approach visits subsequently produce zero local map gain and failed observation, and that approach is excluded after its no-progress cap. No target claim is confirmed. Thus this run includes both unsuccessful target inspection and a later frontier loop; it is not simply failure to ever detect a bowl.

The frozen integrated runner uses `SpatialRecovery(..., locality_radius_m=.6)`. `_group()` matches lineage/support **and** requires the candidate approach to lie within 0.6 m of the group's retained anchor. When groups merge, the first anchor survives.

Replaying the recorded decision states through that implementation reproduces the bookkeeping failure: F000192's group at (-0.15, 1.95), with one attempt, one observation failure, and one no-progress count, is merged into a group anchored at (0.55, 1.95). F000192 is 0.70 m from that anchor, so its next lookup rejects the merged group and creates a fresh group with zero counts. Its history is retained elsewhere but not applied to its own eligibility. Repeated cycles increase exhausted counts on F000186/F000193 while F000192 stays eligible.

Replay limitation: decision evaluations use exact logged states; post-action recovery calls use the next decision snapshot (or final state) for the after-map. This reproduces group/counter behaviour and agrees with logged exclusions, but is not a complete sensor or simulator replay.

The terminal wall time is **2400.609 s**, versus a **2400 s** manifest cap. Simulation time is 564.73/600 s, steps 16942/18000, attempts 17/45, and decisions 17/80. The frozen runner's `_budget()` explains `search_limit`: the wall-clock cap fired before the simulation cap. Concurrent execution may affect wall time; the logs do not quantify its contribution.

Relevant frozen code: `runner/integrated.py:218`, `runner/recovery.py:76–99`, `runner/episode.py:134–140` and `435–452`.

## Recommended follow-up

1. Repair recovery identity/counter preservation across merges: a known frontier must not obtain fresh counters solely because a surviving group anchor changes. Verify using the F000192 sequence, while preserving locality between unrelated opportunities.
2. Add explicit recovery for an unusable current pose, with collision-checked retreat or alternative motion to a previously usable pose. Repeated in-place observation cannot resolve the recorded Nearest terminal condition.
3. Investigate mapping/clearance at Nearest revisions 252–259 and the blocked bowl-inspection views before attributing either failure to noise or a particular object.
4. Record the specific exhausted budget in terminal events; `search_limit` currently combines wall, simulation, step, attempt, and decision caps.

These are proposed fixes, not claims of improved success. Validate with separate diagnostic runs rather than replacing the frozen benchmark results.

## Evidence

- [Extracted original events](evidence.json), including original JSONL line numbers.
- [Frozen recovery replay output](recovery-replay.txt).
- Full originals: `runs/bowl-48-noise-20261110-02/episodes/<case>/events.jsonl`, `final-state.json`, `summary.json`, and `trajectory-scene.png`.
