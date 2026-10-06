# Skeleton implementation checklist

The English design documents remain authoritative. This file records scaffolding
status and intended extension points, without changing the contract.

W2 integration update (2026-10-07): A/B/C branches are merged. B's costmap,
planner, candidate evaluation, navigation and observation control now have a
tested implementation using MockSimulator; the B items below are requirements
to verify against the real adapter, not all unimplemented features. A's update,
and the real B adapter remain pending. C's episode/recovery implementation now
runs against an authored mapper and B's mock adapter, with process supervision.
See [current integration status](chinese/w2_integration_status.md) and
`tests/test_w2_integration.py` for the cross-owner checks and remaining gates.

## Shared foundation

Implemented: all frozen contract records/enums from Interface Freeze 1.0, tuple
collections, immutable-value checks, core snapshot/assessment/selection validation,
owner interfaces, synthetic fixture loader, policy demo, and JSONL event sink.

Pending: full manifest loader/validation; integrated adapter agreement on sensor conventions;
robot- and motion-specific settings; episode-wide sequence, ID, lineage, and fixed
grid validation; complete claim/evidence validation; planner/feedback lifecycle
validation. Reject boundary violations explicitly rather than returning empty data.

## A — Perception and evaluation

Completed W1: mapper reset, JSON room-prior loading, RGB-D validation and replay,
and detector feasibility. See the [handoff](english/w1_perception_handoff.md) for
ownership and setup, and [detector results](english/detector_feasibility.md) for the comparison.

Depth projection and uninflated obstacle-endpoint helpers are available as W2
groundwork. `RGBDMapper.update()`, detector integration and evidence retrieval
remain pending.

- Derive physical occupancy from robot-height clearance evidence. Unknown remains
  non-traversable; spawn clearance seeds free space without observation coverage.
- Rasterize room coverage and associate localized objects without leaking room
  labels in standard runs. Handle unknown object positions explicitly.
- Extract deterministic frontiers with known-free approach poses, predicted gain,
  stable identity, spatial support, and split/merge/disappearance history.
- Confirm localized target tracks across distinct frames; retain evidence through
  scoring and emit at most one terminal claim.
- Implement `TargetEvaluator` against separately retained hidden frame records.
  Check claimed visible evidence, report errors, and never send results to C.

## B — Simulator and navigation

- Pin iGibson/assets and implement the adapter with capture-time transforms,
  verified spawn footprint, runner-owned stepping, stop/pause/resume, and cleanup.
- Build a separate footprint-aware costmap; block unknown/out-of-bounds cells.
- Implement the team planner and bounded batch candidate assessment using the
  same geometry/motion model as execution. Timeout remains inconclusive.
- Validate segments, rotations, corner clearance, pose/map freshness, and both
  position and yaw at arrival. Replan only to the dispatched goal.
- Execute fixed initial/arrival/recovery observation routines safely, exposing
  designated frames for the runner's processed-frame barrier.
- Latch one terminal status; preserve planner results independently of execution
  failures. Make cancellation idempotent, including before a plan exists.

Navigation fixtures: `open_space`, `wall_detour`, `narrow_doorway`,
`unknown_barrier`, and `blocked_update`. Their maps are uninflated; doorway
feasibility depends on the chosen robot. These fixtures do not assert that a
planner or robot can execute a path.

## C — Runner, recovery and experiments

W2 update: `EpisodeRunner`, `SpatialRecovery`, `EpisodeManifest` and the process
supervisor implement the lifecycle/recovery/budget items below for synchronous
module calls. Tests cover normal/blocked/no-progress episodes, stale inputs,
scan barriers, cancellation, lineage caps and hung modules. These remain
acceptance requirements when replacing mocks with real A/B components.
See [runner usage](chinese/w2_runner.md) and [A/B acceptance](chinese/w2_ab_handoff.md).

Implemented: Random, Nearest, Information Gain, LLM response validation/Nearest
fallback, shared eligible-context construction, validated selection, runner
dependency wiring, termination enum, and fixture JSONL output.

- Validate the implemented `EpisodeRunner.run()` on real A/B inputs against section 5.
  Capture before initial motion, enforce scan-frame barriers, pause all decisions,
  and reassess stale dispatches. Continue perception during motion.
- Validate `SpatialRecovery` using spatial overlap and lineage; retain caps
  through relabeling/disappearance and unlock only on relevant evidence/cooldown.
- Freeze local progress neighborhoods at dispatch; count unknown-to-known area
  after arrival observations. Bound retries and recovery-only loops.
- Independently supervise step/time/attempt/decision/operation limits and stalled
  module or LLM calls. Cancel motion immediately on claims and terminal limits.
- Freeze evidence and score once; false claims end the episode. Never allow
  evaluator feedback to resume search.
- Add provider transport cancellation and usage/cost capture to the LLM adapter.
  Gemini-compatible transport cancellation and usage capture are now available in
  `policies/provider.py`; cost is still unknown. A separate contract-based
  three-room synthetic loop and live/replay cases are in `runner/synthetic.py`.
  These older demos are separate from the W2 lifecycle implementation. See
  [integration review](chinese/c_integration_review.md).
- Load/freeze paired manifests with separate RNG streams; log revisions, actual
  executed distance, collisions, replans, barriers, attempts, and terminal outcomes.
- Report all episode outcomes, paired uncertainty, and explicitly labeled
  success-conditioned efficiency with sample counts.

Policy fixtures: `selection`, `no_frontiers`, `all_excluded`, `target_claim`,
`inconclusive`, and `prior_failure`. They are authored inputs, not mapper/planner
output or experimental evidence. The target-claim fixture has no automatic
verifier result.

## Integration gates

Before experiments, complete all checks in Interface Freeze section 7, including
geometry safety, identity transfer, fresh-frame barriers, no-progress suppression,
claim scoring isolation, pre-plan cancellation, bounded hangs, manifest replay,
and complete failed-episode accounting. Existing unit tests cover only the
implemented synthetic decision boundary and fixture behavior.
