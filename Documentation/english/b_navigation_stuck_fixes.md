# B navigation: two ways the robot could get stuck, and the fixes

Scope: B (navigation and planning) only. Code: `navigation/navigator.py`, `navigation/costmap.py`,
`navigation/planner.py`. Tests: `tests/test_stuck_fixes.py`.
**Status: verified on the mock simulator only; not yet run on native iGibson.** Default configuration
(planning margin equal to start margin) behaves exactly as before.

## 1. Scan heading stuck at the tolerance boundary

Symptom (e.g. `apartment-s29-nearest`): after the in-place scan the robot rests at a heading error of
0.0503 rad against a 0.05 rad tolerance. The step never completes and never fails; it runs to the
per-operation limit.

Causes:
- **Turn command too small.** Proportional control gives `K * error = 2.0 * 0.0503 = 0.1 rad/s`, below what
  static friction of the chassis lets through, so the robot sits just outside the tolerance.
- **Stall detection reset by the dwell.** The observation dwell is exempt from stall detection, and every
  re-entry into the tolerance restarted the 5 s stall window. A robot jittering across 0.05 rad therefore
  never reached the stall limit.

Fix (`navigator.py`):
- `_turn_rate`: the commanded turn rate keeps the proportional law but never drops below
  `MIN_TURN_RADPS = 0.2` rad/s while the heading is outside the tolerance (and never above the robot's own
  limit). At the 0.05 s step this turns at most ~0.01 rad per step, so it cannot jump over the 0.1 rad wide
  tolerance band. The 0.2 value is chosen for the step size and has not been tuned on a real chassis.
- The dwell stays exempt from stall detection but no longer restarts the window; the window restarts only
  when a scan heading is completed. A jittering robot is now reported `BLOCKED` after about 5 s.
- The tolerance itself is unchanged (0.05 rad to enter the dwell).

Robotics rationale: a proportional controller on a plant with stiction has a dead zone around zero error;
a minimum command (a feed-forward term to break stiction) removes it.

## 2. Start margin was not used for the first turn and first step

Symptom (3 episodes): the robot stops, or a newly scanned obstacle appears, so it sits inside the 1 cm
planning buffer. The start pose is accepted (`start_margin_m = 0`), but every first move is rejected as
`no_path` and the scan as `blocked`. The failure policy retries only when evidence changes, and a
stationary robot produces none, so the episode deadlocks.

Cause: `start_margin_m` was used only by `start_pose_valid`. The first in-place turn and the first drive
(`first_move_valid`, `rotation_valid`, `segment_valid`) were checked at the planning margin.

Fix:
- `Costmap.first_move_valid` checks the turn and the first `START_ZONE_M = 0.10` m of the drive with
  `start_margin_m`; beyond the zone the planning margin applies again. `pose_valid`, `rotation_valid`,
  `full_rotation_valid` and `segment_valid(start_zone_m=...)` take an optional margin.
- In-place observation sweeps at the start pose, the first in-place turn of a navigation (while within the
  zone), `planner.path_valid` and the smoothing shortcut from the start use the start margin as well.
- `planner._success` no longer routes the path through the start cell's centre when that cell is
  inflation-blocked. The centre can lie inside the obstacle, and `first_move_valid` already approved
  driving straight to the next cell. The reported cost is unchanged, so it still matches the distance
  field used by `evaluate_candidates`.

Robotics rationale: the planning margin is configuration-space obstacle inflation, a planning buffer and
not physics. A robot already inside the buffer must be allowed to leave it, while the exact footprint is
still collision-checked.

Trade-offs: with `start_margin_m < planning_margin_m` the start zone has no buffer, and `path_valid`
re-checks the robot's current pose at the start margin (about 1 cm looser during replanning). Both apply
only to the opt-in options (`SOFT_CONTACT_NAV_OPTIONS`, `ROUND_FOOTPRINT_*`).

## Not changed

- Robot size: the 0.19 m round footprint matches the measured Locobot (collision cylinder 0.178 m, mesh
  about 0.188 m) and is not reduced.
- `navigation/lattice.py` is not B's module and was not touched.
- Failure class 3 (target selection by distance only, `policies/baselines.py`) belongs to C; failure
  class 4 (approach points that cannot hold the robot, `frontier_approach_search_m`) belongs to A.

## Verification

- `tests/test_stuck_fixes.py` (14 tests) fails on the previous code and passes now. Full suite: 440 tests,
  4 skipped.
- 36 mock-simulator scenarios (12 scan-heading cases with and without stiction, 24 start-in-buffer cases
  over 3 wall distances and 4 headings, navigation and scan): 6/36 pass before, 36/36 after. These
  scenarios were designed for these two fixes, so they show the fixes work, not an overall success rate.
- Note on the footprint: the 16-sided round footprint has circumradius 0.1835 m, slightly above the nominal
  0.18 m, so a start with less than about 3.5 mm of nominal clearance is correctly rejected.
- Still to do: rerun the paired benchmark on native iGibson.
