"""Owner B: team Navigator - candidate evaluation, goal execution, observation sweeps.

Implements the ``Navigator`` protocol of Interface Freeze section 4.3. B never
ranks frontiers by search value; it only reports feasibility/cost and executes
the one goal the runner dispatches.

Robotics background
-------------------
* Sense-plan-act: A senses and maps, B plans on the costmap, B's controller
  acts by commanding velocities. Planning (``start``) and control (``on_step``)
  are separate stages; a plan being found does not mean execution succeeds.
* Differential-drive control: velocity commands are (v, w), linear and angular.
  The controller uses *rotate-then-translate* tracking with a proportional
  heading law: ``w = clip(K * heading_error)``. When the heading error is large
  the robot first turns in place (possible because it can turn in place), and
  it drives forward only once roughly aligned. Speeds are capped by the
  robot's configured limits.
* Waypoint following with a lookahead radius: intermediate waypoints are
  considered reached inside a lookahead, so the robot does not stop at every
  grid-cell centre. The final waypoint requires both the position tolerance and
  the yaw tolerance (Interface Freeze: arrival needs both).
* Replanning: when a newer costmap invalidates the remaining path the robot
  stops (safety first), then replans to the *same* goal. It never silently
  chooses another frontier. Replans are bounded.
* Active perception: observation sweeps rotate the fixed camera through a
  configured set of headings so A sees more of the unknown space (frontier-
  based exploration, Yamauchi 1997). Each heading is held for a dwell time and
  then marked as a designated capture for the runner's frame barrier.
* Finite state machine per operation: ACCEPTED -> EXECUTING -> exactly one
  terminal state (REACHED / FAILED / CANCELLED), which is latched.

Open points for the team (not decided here)
-------------------------------------------
1. ``SimulatorAdapter`` (simulation/api.py) has no velocity command or
   simulation clock. B defines ``ControlInterface`` locally; the adapter must
   provide these before real integration.
2. The runner must call ``on_step()`` after each simulation step ("B's
   controller executes at the configured cadence on those steps"); ``poll`` is
   read-only and never advances simulation.
3. Stale start pose or stale map in ``start`` is reported as FAILED with
   INVALID_GOAL, because the frozen FailureReason enum has no better value.
4. Stall detection thresholds are B-local constants (shared ``NavigationConfig`` has
   no field for them and is not edited here).
5. ``map_freshness_s`` cannot be checked: ``OccupancyGrid`` has no timestamp,
   so only the revision is checked.

Only the standard library is used; annotations stay Python 3.8 compatible.
"""

from math import atan2, cos, hypot, pi
from time import monotonic
from typing import Callable, Dict, List, Optional, Tuple

from ..config import NavigationConfig, ObservationConfig
from ..contracts import (
    CandidateAssessment, CandidateEvaluation, CandidateEvaluationRequest, FailureReason,
    NavigationFeedback, NavigationRequest, NavigationStatus, NavigatorInit, ObservationRequest,
    OccupancyGrid, PlannerResult, Pose, RobotConfig,
)
from ..validation import ContractError
from . import planner
from .costmap import Costmap

try:  # typing.Protocol exists from Python 3.8
    from typing import Protocol
except ImportError:  # pragma: no cover
    Protocol = object  # type: ignore

K_ANGULAR = 2.0            # proportional gain on heading error (1/s)
K_LINEAR = 1.0             # proportional gain on distance (1/s)
TURN_IN_PLACE_RAD = 0.6    # rotate without driving above this heading error
STALL_WINDOW_S = 5.0       # simulated seconds without real movement => blocked
STALL_MIN_MOVE_M = 0.02    # position change that counts as progress
STALL_MIN_TURN_RAD = 0.05  # yaw change that counts as progress


class NavigatorBoundaryError(ContractError):
    """A documented boundary violation (overlapping operation, wrong episode, ...)."""


class ControlInterface(Protocol):
    """What B needs from the simulator/control adapter (see open point 1)."""

    def current_pose(self) -> Pose: ...
    def sim_time_s(self) -> float: ...
    def set_velocity(self, linear_mps: float, angular_radps: float) -> None: ...


def _wrap(angle: float) -> float:
    return (angle + pi) % (2 * pi) - pi


def _clip(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


class _Operation:
    """Mutable internal record of one navigation or observation operation."""

    def __init__(self, goal_id: str, kind: str, started_s: float) -> None:
        self.goal_id = goal_id
        self.kind = kind                      # "navigate" | "observe"
        self.started_s = started_s
        self.status = NavigationStatus.ACCEPTED
        self.failure_reason = None            # type: Optional[FailureReason]
        self.planner_result = None            # type: Optional[PlannerResult]
        self.terminal = None                  # type: Optional[NavigationFeedback]
        self.costmap = None                   # type: Optional[Costmap]
        # navigation
        self.goal_pose = None                 # type: Optional[Pose]
        self.path = ()                        # type: Tuple[Pose, ...]
        self.index = 1                        # next waypoint (0 is the start pose)
        self.replans = 0
        # observation
        self.targets = []                     # type: List[float]
        self.leg = 0
        self.dwell_until = None               # type: Optional[float]
        self.marks = []                       # type: List[float]
        self.pose = None                      # type: Optional[Pose]
        # stall detection
        self.stall_pose = None                # type: Optional[Pose]
        self.stall_since_s = started_s


class TeamNavigator:
    """B's Navigator. Construct with a control interface, then ``reset``."""

    def __init__(self, control: ControlInterface, wall_clock: Callable[[], float] = monotonic,
                 smooth_paths: bool = False) -> None:
        self._control = control
        self._wall_clock = wall_clock
        self._smooth = smooth_paths
        self._episode_id = None               # type: Optional[str]
        self._robot = None                    # type: Optional[RobotConfig]
        self._obs_cfg = None                  # type: Optional[ObservationConfig]
        self._nav_cfg = None                  # type: Optional[NavigationConfig]
        self._ops = {}                        # type: Dict[str, _Operation]
        self._active = None                   # type: Optional[_Operation]
        self._costmap_cache = None            # type: Optional[Tuple[int, Costmap]]
        self._latest_revision = -1
        self.events = []                      # type: List[Tuple[str, str]]

    # --- lifecycle ---------------------------------------------------------

    def reset(self, request: NavigatorInit) -> None:
        if self._active is not None:
            raise NavigatorBoundaryError("reset requires the prior operation to be stopped")
        if not request.robot_config.can_turn_in_place:
            # Plain grid A* plus rotate-then-translate control assumes a robot that can
            # turn in place. Refuse loudly rather than plan paths it cannot drive
            # (use Hybrid A* or a lattice planner for car-like robots).
            raise NavigatorBoundaryError("planner supports only robots that can turn in place")
        self._episode_id = request.episode_id
        self._robot = request.robot_config
        self._obs_cfg = request.observation_config
        self._nav_cfg = request.navigation_config
        self._ops, self._costmap_cache, self._latest_revision = {}, None, -1
        self.events = []

    # --- candidate evaluation (read-only) ----------------------------------

    def evaluate_candidates(self, request: CandidateEvaluationRequest) -> CandidateEvaluation:
        """Reachability and path cost of every frontier from the state's pose.

        One Dijkstra distance field answers all candidates (amortised). A
        timeout leaves unreached cells *inconclusive* (None/TIMEOUT), never
        "unreachable". Does not move the robot or touch the active operation.
        """
        self._require_ready(request.episode_id)
        state = request.state
        costmap = self._costmap_for(state.occupancy_map)
        start = state.robot_pose
        result = planner.distance_field(costmap, start, self._nav_cfg.batch_deadline_s,
                                        clock=self._wall_clock)
        assessments = []  # type: List[CandidateAssessment]
        for frontier in state.frontiers:
            goal = frontier.approach_pose
            if not planner.goal_is_valid(costmap, goal):
                assessments.append(CandidateAssessment(frontier.frontier_id, False, None,
                                                       FailureReason.INVALID_GOAL))
            elif result is None:  # start unusable: robot cannot move at all
                assessments.append(CandidateAssessment(frontier.frontier_id, False, None,
                                                       FailureReason.NO_PATH))
            else:
                field, complete = result
                cost = planner.cost_to_goal_m(costmap, start, goal, field)
                if cost is not None:
                    assessments.append(CandidateAssessment(frontier.frontier_id, True, cost, None))
                elif complete:
                    assessments.append(CandidateAssessment(frontier.frontier_id, False, None,
                                                           FailureReason.NO_PATH))
                else:
                    assessments.append(CandidateAssessment(frontier.frontier_id, None, None,
                                                           FailureReason.TIMEOUT))
        return CandidateEvaluation(state.episode_id, state.revision, start, tuple(assessments))

    # --- navigation --------------------------------------------------------

    def start(self, request: NavigationRequest) -> NavigationFeedback:
        self._require_ready(request.episode_id)
        op = self._begin(request.goal_id, "navigate")
        goal = request.subgoal.approach_pose
        op.goal_pose = goal
        live = self._control.current_pose()
        if not self._fresh(request.occupancy_map, request.start_pose, live):
            return self._finish(op, NavigationStatus.FAILED, FailureReason.INVALID_GOAL)
        op.costmap = self._costmap_for(request.occupancy_map)
        result = planner.plan(op.costmap, live, goal, self._nav_cfg.batch_deadline_s,
                              clock=self._wall_clock, smooth=self._smooth)
        op.planner_result = result
        self._log("plan", "%s success=%s reason=%s" % (op.goal_id, result.success, result.failure_reason))
        if not result.success:
            return self._finish(op, NavigationStatus.FAILED, result.failure_reason)
        op.path, op.index = result.path, 1
        return self._feedback(op)  # ACCEPTED

    def update_map(self, episode_id: str, goal_id: str, occupancy_map: OccupancyGrid) -> None:
        """Accept a newer map for the active goal; recheck and replan if needed."""
        self._require_ready(episode_id)
        op = self._ops.get(goal_id)
        if op is None:
            raise NavigatorBoundaryError("unknown goal_id %r" % goal_id)
        if op.terminal is not None or op is not self._active:
            return
        current = op.costmap.grid
        if (occupancy_map.frame_id, occupancy_map.resolution_m, occupancy_map.origin_x_m,
                occupancy_map.origin_y_m, occupancy_map.width_cells, occupancy_map.height_cells) != \
                (current.frame_id, current.resolution_m, current.origin_x_m, current.origin_y_m,
                 current.width_cells, current.height_cells):
            raise NavigatorBoundaryError("map grid geometry changed within an episode")
        if occupancy_map.revision <= current.revision:
            return  # only newer maps are accepted; equal/older is a no-op
        op.costmap = self._costmap_for(occupancy_map)
        live = self._control.current_pose()
        if op.kind == "navigate":
            if self._remaining_path_valid(op, live):
                return
            self._control.set_velocity(0.0, 0.0)  # stop unsafe motion before replanning
            self._replan(op, live)
        elif not self._sweep_safe(op.costmap, live, [live.yaw_rad] + op.targets[op.leg:]):
            self._finish(op, NavigationStatus.FAILED, FailureReason.BLOCKED)

    def poll(self, episode_id: str, goal_id: str) -> NavigationFeedback:
        """Read status only; never advances simulation."""
        self._require_ready(episode_id)
        op = self._ops.get(goal_id)
        if op is None:
            raise NavigatorBoundaryError("unknown goal_id %r" % goal_id)
        return op.terminal if op.terminal is not None else self._feedback(op)

    def cancel(self, episode_id: str, goal_id: str, reason: str) -> NavigationFeedback:
        """Stop motion. Idempotent: a terminal outcome is returned unchanged, and
        cancelling a goal that never started (no plan exists) latches CANCELLED."""
        self._require_ready(episode_id)
        op = self._ops.get(goal_id)
        if op is not None and op.terminal is not None:
            return op.terminal
        if op is None:
            op = _Operation(goal_id, "navigate", self._control.sim_time_s())
            self._ops[goal_id] = op
        self._log("cancel", "%s %s" % (goal_id, reason))
        return self._finish(op, NavigationStatus.CANCELLED, None)

    # --- observation -------------------------------------------------------

    def start_observation(self, request: ObservationRequest) -> NavigationFeedback:
        self._require_ready(request.episode_id)
        if request.routine not in ("initial", "arrival", "recovery"):
            raise NavigatorBoundaryError("unknown observation routine %r" % request.routine)
        op = self._begin(request.goal_id, "observe")
        live = self._control.current_pose()
        if not self._fresh(request.occupancy_map, request.start_pose, live):
            return self._finish(op, NavigationStatus.FAILED, FailureReason.INVALID_GOAL)
        op.costmap = self._costmap_for(request.occupancy_map)
        op.pose = live
        cfg = self._obs_cfg
        if request.routine == "initial" and cfg.allow_full_rotation:
            # Full turn in quarter steps, only if clearance permits it.
            op.targets = [_wrap(request.heading_rad + k * pi / 2) for k in range(1, 5)]
        else:
            offsets = {"initial": cfg.initial_headings_rad, "arrival": cfg.arrival_headings_rad,
                       "recovery": cfg.recovery_headings_rad}[request.routine]
            op.targets = [_wrap(request.heading_rad + o) for o in offsets]
        safe = (self._robot.can_turn_in_place and op.costmap.pose_valid(live)
                and self._sweep_safe(op.costmap, live, [live.yaw_rad] + op.targets)
                and (not (request.routine == "initial" and cfg.allow_full_rotation)
                     or op.costmap.full_rotation_valid(live)))
        if not safe:  # never rotate blindly
            return self._finish(op, NavigationStatus.FAILED, FailureReason.BLOCKED)
        return self._feedback(op)  # ACCEPTED

    def capture_marks(self, goal_id: str) -> Tuple[float, ...]:
        """Simulation times at which a scan heading was reached and dwelled.

        These are the designated frames the runner's frame barrier must see
        processed by A before it selects again.
        """
        op = self._ops.get(goal_id)
        return tuple(op.marks) if op is not None else ()

    # --- controller tick ---------------------------------------------------

    def on_step(self) -> None:
        """Run one control tick. The runner calls this after each simulation step."""
        op = self._active
        if op is None or op.terminal is not None:
            return
        now = self._control.sim_time_s()
        limit = self._nav_cfg.navigation_deadline_s if op.kind == "navigate" \
            else self._obs_cfg.operation_deadline_s
        if now - op.started_s > limit:
            self._finish(op, NavigationStatus.FAILED, FailureReason.TIMEOUT)
            return
        op.status = NavigationStatus.EXECUTING
        if self._stalled(op, now):
            self._finish(op, NavigationStatus.FAILED, FailureReason.BLOCKED)
            return
        if op.kind == "navigate":
            self._tick_navigation(op)
        else:
            self._tick_observation(op, now)

    # --- internals: stall detection ----------------------------------------

    def _stalled(self, op: _Operation, now: float) -> bool:
        """No real movement (position or yaw) for STALL_WINDOW_S of simulated time.

        Catches a robot held up by something the belief map does not show (a
        physical obstacle, a wheel stuck). Deliberate stillness during an
        observation dwell is exempt.
        """
        pose = self._control.current_pose()
        dwelling = op.kind == "observe" and op.dwell_until is not None
        ref = op.stall_pose
        moved = ref is None or dwelling \
            or hypot(pose.x_m - ref.x_m, pose.y_m - ref.y_m) > STALL_MIN_MOVE_M \
            or abs(_wrap(pose.yaw_rad - ref.yaw_rad)) > STALL_MIN_TURN_RAD
        if moved:
            op.stall_pose, op.stall_since_s = pose, now
            return False
        return now - op.stall_since_s > STALL_WINDOW_S

    # --- internals: navigation tick ---------------------------------------

    def _tick_navigation(self, op: _Operation) -> None:
        robot, pose = self._robot, self._control.current_pose()
        lookahead = max(robot.position_tolerance_m, op.costmap.grid.resolution_m)
        last = len(op.path) - 1
        while op.index < last and hypot(op.path[op.index].x_m - pose.x_m,
                                        op.path[op.index].y_m - pose.y_m) < lookahead:
            op.index += 1
        target = op.path[op.index]
        dist = hypot(target.x_m - pose.x_m, target.y_m - pose.y_m)
        if op.index == last and dist <= robot.position_tolerance_m:
            yaw_err = _wrap(op.goal_pose.yaw_rad - pose.yaw_rad)
            if abs(yaw_err) <= robot.yaw_tolerance_rad:
                self._finish(op, NavigationStatus.REACHED, None)
                return
            self._control.set_velocity(0.0, _clip(K_ANGULAR * yaw_err, robot.max_angular_speed_radps))
            return
        heading_err = _wrap(atan2(target.y_m - pose.y_m, target.x_m - pose.x_m) - pose.yaw_rad)
        w = _clip(K_ANGULAR * heading_err, robot.max_angular_speed_radps)
        v = 0.0 if abs(heading_err) > TURN_IN_PLACE_RAD else \
            min(robot.max_linear_speed_mps, K_LINEAR * dist) * max(0.0, cos(heading_err))
        self._control.set_velocity(v, w)

    def _tick_observation(self, op: _Operation, now: float) -> None:
        if op.leg >= len(op.targets):
            self._finish(op, NavigationStatus.REACHED, None)
            return
        robot, cfg = self._robot, self._obs_cfg
        pose = self._control.current_pose()
        err = _wrap(op.targets[op.leg] - pose.yaw_rad)
        if op.dwell_until is None:
            if abs(err) > cfg.heading_tolerance_rad:
                self._control.set_velocity(0.0, _clip(K_ANGULAR * err, robot.max_angular_speed_radps))
                return
            self._control.set_velocity(0.0, 0.0)
            op.dwell_until = now + cfg.dwell_s
        if now >= op.dwell_until:
            op.marks.append(now)  # heading reached and dwelled: designated capture
            op.leg += 1
            op.dwell_until = None
            if op.leg >= len(op.targets):
                self._finish(op, NavigationStatus.REACHED, None)

    # --- internals: replanning and safety ----------------------------------

    def _remaining_path_valid(self, op: _Operation, live: Pose) -> bool:
        remaining = (live,) + tuple(op.path[op.index:])
        return planner.path_valid(op.costmap, remaining)

    def _replan(self, op: _Operation, live: Pose) -> None:
        op.replans += 1
        if op.replans > self._nav_cfg.max_replans:
            self._finish(op, NavigationStatus.FAILED, FailureReason.BLOCKED)
            return
        result = planner.plan(op.costmap, live, op.goal_pose, self._nav_cfg.batch_deadline_s,
                              clock=self._wall_clock, smooth=self._smooth)
        op.planner_result = result  # newest result is carried in feedback
        self._log("replan", "%s #%d success=%s reason=%s"
                  % (op.goal_id, op.replans, result.success, result.failure_reason))
        if not result.success:
            self._finish(op, NavigationStatus.FAILED, result.failure_reason)
            return
        op.path, op.index = result.path, 1

    @staticmethod
    def _sweep_safe(costmap: Costmap, pose: Pose, yaws: List[float]) -> bool:
        """Every leg of a yaw sequence (turning in place at ``pose``) is collision-free."""
        return all(costmap.rotation_valid(Pose(pose.x_m, pose.y_m, pose.z_m, yaws[i]), nxt)
                   for i, nxt in enumerate(yaws[1:]))

    # --- internals: bookkeeping -------------------------------------------

    def _require_ready(self, episode_id: str) -> None:
        if self._episode_id is None:
            raise NavigatorBoundaryError("navigator not reset")
        if episode_id != self._episode_id:
            raise NavigatorBoundaryError("episode mismatch: %r != %r" % (episode_id, self._episode_id))

    def _begin(self, goal_id: str, kind: str) -> _Operation:
        if not goal_id:
            raise NavigatorBoundaryError("empty goal_id")
        if goal_id in self._ops:
            raise NavigatorBoundaryError("goal_id %r already used" % goal_id)
        if self._active is not None and self._active.terminal is None:
            raise NavigatorBoundaryError("another operation is still active")
        op = _Operation(goal_id, kind, self._control.sim_time_s())
        self._ops[goal_id] = op
        self._active = op
        return op

    def _fresh(self, grid: OccupancyGrid, start_pose: Pose, live: Pose) -> bool:
        stale_pose = hypot(start_pose.x_m - live.x_m, start_pose.y_m - live.y_m) \
            > self._nav_cfg.pose_freshness_m
        stale_map = grid.revision < self._latest_revision
        return not (stale_pose or stale_map)

    def _costmap_for(self, grid: OccupancyGrid) -> Costmap:
        cached = self._costmap_cache
        if cached is not None and cached[0] == grid.revision and cached[1].grid is grid:
            return cached[1]
        costmap = Costmap.from_robot(grid, self._robot)
        costmap.prepare()  # pay the one-off clearance precompute here, not inside a deadline
        self._costmap_cache = (grid.revision, costmap)
        self._latest_revision = max(self._latest_revision, grid.revision)
        return costmap

    def _remaining_distance(self, op: _Operation) -> Optional[float]:
        if op.kind != "navigate" or not op.path:
            return None
        pose = self._control.current_pose()
        points = [(pose.x_m, pose.y_m)] + [(p.x_m, p.y_m) for p in op.path[op.index:]]
        return sum(hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))

    def _feedback(self, op: _Operation, status: Optional[NavigationStatus] = None,
                  reason: Optional[FailureReason] = None) -> NavigationFeedback:
        status = status or op.status
        remaining = 0.0 if (status == NavigationStatus.REACHED and op.kind == "navigate") \
            else self._remaining_distance(op)
        feedback = NavigationFeedback(self._episode_id, op.goal_id, self._control.sim_time_s(),
                                      status, self._control.current_pose(), remaining,
                                      op.planner_result, reason)
        # Contract: FAILED has a reason, others none; reached navigation needs a plan.
        if (status == NavigationStatus.FAILED) != (reason is not None):
            raise ContractError("failure_reason must be set exactly when status is FAILED")
        if status == NavigationStatus.REACHED and op.kind == "navigate" and not (
                op.planner_result and op.planner_result.success):
            raise ContractError("reached navigation requires a successful plan")
        return feedback

    def _finish(self, op: _Operation, status: NavigationStatus,
                reason: Optional[FailureReason]) -> NavigationFeedback:
        """Stop, latch exactly one terminal outcome, and free the single slot."""
        self._control.set_velocity(0.0, 0.0)
        op.status, op.failure_reason = status, reason
        op.terminal = self._feedback(op, status, reason)
        self._log("terminal", "%s %s %s" % (op.goal_id, status.value, reason.value if reason else ""))
        if self._active is op:
            self._active = None
        return op.terminal

    def _log(self, kind: str, detail: str) -> None:
        self.events.append((kind, detail))
