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
1. ``ControlledSimulatorAdapter`` (simulation/api.py) adds the velocity command and
   simulation clock. ``ControlInterface`` below is B's minimal local view of
   exactly those plus ``current_pose``, so this module does not import the
   simulation package; ``IGibsonAdapter`` and ``MockSimulator`` both provide them.
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
from dataclasses import asdict, dataclass, replace
import json
from time import monotonic
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from ..config import NavigationConfig, ObservationConfig
from ..contracts import (
    CandidateAssessment, CandidateEvaluation, CandidateEvaluationRequest, FailureReason,
    NavigationFeedback, NavigationRequest, NavigationStatus, NavigatorInit, ObservationRequest,
    OccupancyGrid, PlannerResult, Point2, Pose, RobotConfig,
)
from ..validation import ContractError
from . import diagnostics, planner
from .costmap import START_ZONE_M, Costmap, circumradius
from .geometry import circle_footprint

try:  # typing.Protocol exists from Python 3.8
    from typing import Protocol
except ImportError:  # pragma: no cover
    Protocol = object  # type: ignore

K_ANGULAR = 2.0            # proportional gain on heading error (1/s)
MIN_TURN_RADPS = 0.2       # floor on the commanded turn rate while a heading is still out of tolerance
K_LINEAR = 1.0             # proportional gain on distance (1/s)
TURN_IN_PLACE_RAD = 0.6    # rotate without driving above this heading error
STALL_WINDOW_S = 5.0       # simulated seconds without real movement => blocked
STALL_MIN_MOVE_M = 0.02    # position change that counts as progress
STALL_MIN_TURN_RAD = 0.05  # yaw change that counts as progress


@dataclass(frozen=True)
class NearObstacleLimit:
    """Opt-in speed limit inside a proximity zone around obstacles (B-local, not a shared contract).

    Robotics rationale: in the dynamic-window approach (Fox, Burgard, Thrun 1997) a velocity is only
    admissible if the robot can still stop before the nearest obstacle, so the allowed speed shrinks
    with obstacle distance. Impact energy also grows with v squared, so capping v near obstacles
    turns a would-be collision into a light touch. The controller here is rotate-then-translate with
    a 0.6 rad in-place threshold: below it the robot turns while driving, and the square body then
    sweeps its circumscribed radius, which the planner (inscribed-radius planning) does not account
    for. Limiting v and w only inside the zone removes that sweep risk without slowing open floor.
    """
    distance_m: float = 0.06         # zone = footprint grown by margin + distance_m touches a blocked cell
    max_linear_mps: float = 0.08
    max_angular_radps: float = 0.6

    def __post_init__(self) -> None:
        if self.distance_m < 0 or self.max_linear_mps <= 0 or self.max_angular_radps <= 0:
            raise ValueError("near-obstacle limit needs distance >= 0 and positive speed caps")


@dataclass(frozen=True)
class RelaxedRetry:
    """Opt-in bounded fallback: re-plan ONE dispatched goal with a relaxed costmap after a strict failure (B-local).

    Robotics rationale: the strict costmap (circumscribed disc plus a clearance margin) is a conservative
    configuration-space inflation (Lozano-Perez 1983). It can declare a frontier unreachable although the robot
    physically fits, and a "no path / invalid goal" verdict is only lifted when the map changes, which a trapped robot
    can never produce (deadlock). Like epsilon-greedy exploration (Sutton and Barto), occasionally trying an action
    the strict model rejects trades a little safety for escape from a local optimum. It stays bounded and safe:
    * only a goal the runner actually dispatched is retried (candidate evaluation stays strict and read-only, so
      reachability assessment and strict execution still agree for every policy);
    * only after NO_PATH / INVALID_GOAL, never after TIMEOUT or BLOCKED, and at most once per plan or replan;
    * the exact-footprint guards (first move, in-place turns, sweeps) and stall detection still apply.
    ``planning_radius_m`` None = the inscribed radius, the smallest disc that is still necessary for the robot.
    """
    margin_m: float = 0.0
    planning_radius_m: Optional[float] = None
    start_margin_m: float = 0.0

    def __post_init__(self) -> None:
        if self.margin_m < 0 or self.start_margin_m < 0:
            raise ValueError("relaxed retry margins must be nonnegative")
        if self.planning_radius_m is not None and self.planning_radius_m <= 0:
            raise ValueError("relaxed planning radius must be positive")


class NavigatorBoundaryError(ContractError):
    """A documented boundary violation (overlapping operation, wrong episode, ...)."""


class ControlInterface(Protocol):
    """What B needs from the simulator/control adapter (a subset of ControlledSimulatorAdapter)."""

    def current_pose(self) -> Pose: ...
    def sim_time_s(self) -> float: ...
    def set_velocity(self, linear_mps: float, angular_radps: float) -> None: ...


def _wrap(angle: float) -> float:
    return (angle + pi) % (2 * pi) - pi


def _turn_rate(error_rad: float, max_radps: float) -> float:
    """Proportional turn command with a floor.

    A pure P law gives K * error, which near the tolerance edge (0.05 rad -> 0.1 rad/s) is below what
    static friction of a real chassis lets through, so the robot sits just outside the tolerance and
    never arrives. The floor keeps the command above stiction while the error is still outside the
    tolerance; callers only use it then, and it never exceeds the robot's own limit.
    """
    rate = max(abs(K_ANGULAR * error_rad), min(MIN_TURN_RADPS, max_radps))
    return _clip(rate if error_rad >= 0 else -rate, max_radps)


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
        self.turn_ok = None                   # type: Optional[object]  # turn already checked safe (waypoint index / "goal")
        self.replans = 0
        self.relaxed = False                  # plan came from the opt-in relaxed retry (see RelaxedRetry)
        self.start_pose = None                # type: Optional[Pose]  # real pose when the goal was accepted
        # observation
        self.targets = []                     # type: List[float]
        self.leg = 0
        self.dwell_until = None               # type: Optional[float]
        self.dwell_s = 0.0
        self.marks = []                       # type: List[float]
        self.pose = None                      # type: Optional[Pose]
        # stall detection
        self.stall_pose = None                # type: Optional[Pose]
        self.stall_since_s = started_s


class TeamNavigator:
    """B's Navigator. Construct with a control interface, then ``reset``."""

    def __init__(self, control: ControlInterface, wall_clock: Callable[[], float] = monotonic,
                 smooth_paths: bool = False, planning_radius_m: Optional[float] = None,
                 linear_gain: float = K_LINEAR, near_obstacle_limit: Optional[NearObstacleLimit] = None,
                 start_margin_m: Optional[float] = None, planning_margin_m: Optional[float] = None,
                 footprint_override: Optional[Sequence[Point2]] = None,
                 relaxed_retry: Optional[RelaxedRetry] = None) -> None:
        # ``relaxed_retry``: None = a strict failure is final (default). A RelaxedRetry makes ``start`` and replans try
        # one relaxed plan for the dispatched goal after NO_PATH / INVALID_GOAL (see RelaxedRetry).
        self._relaxed_retry = relaxed_retry
        # ``footprint_override``: None = the footprint of the NavigatorInit robot configuration (default). A polygon
        # replaces it for every B check (costmap, turns, sweeps) without touching the shared manifest that A and C read,
        # e.g. the measured round Locobot base instead of the +/-0.181 m square AABB (see locobot_robot.py).
        # ``planning_margin_m``: None = the robot's configured clearance margin (default); a smaller value
        # trades safety buffer for reachability. It never changes the exact-footprint physics checks of A.
        # ``planning_radius_m``: None = circumscribed disc (conservative, default). A smaller value in
        # [inscribed, circumscribed] plans through narrower gaps; see Costmap and the P6 notes.
        # ``near_obstacle_limit``: None = no speed limit (default). ``start_margin_m``: None = the start
        # pose is checked with the planning margin (default); 0.0 checks it physically (see Costmap).
        self._planning_radius_m = planning_radius_m
        self._near_limit = near_obstacle_limit
        if start_margin_m is not None and start_margin_m < 0:
            raise ValueError('start_margin_m must be nonnegative')
        self._start_margin_m = start_margin_m
        if planning_margin_m is not None and planning_margin_m < 0:
            raise ValueError('planning_margin_m must be nonnegative')
        self._planning_margin_m = planning_margin_m
        if footprint_override is not None:
            polygon = tuple(footprint_override)
            area = sum(a.x_m * b.y_m - b.x_m * a.y_m for a, b in zip(polygon, polygon[1:] + polygon[:1])) / 2.0
            if len(polygon) < 3 or abs(area) < 1e-9:
                raise ValueError('footprint_override needs at least three vertices and a non-zero area')
            footprint_override = polygon
        self._footprint_override = footprint_override
        self._control = control
        self._wall_clock = wall_clock
        self._smooth = smooth_paths
        if not 0 < linear_gain <= 3:
            raise ValueError('linear_gain must be positive and at most 3 / s')
        self._linear_gain = linear_gain
        self._episode_id = None               # type: Optional[str]
        self._robot = None                    # type: Optional[RobotConfig]
        self._shared_robot = None             # type: Optional[RobotConfig]
        self._obs_cfg = None                  # type: Optional[ObservationConfig]
        self._nav_cfg = None                  # type: Optional[NavigationConfig]
        self._ops = {}                        # type: Dict[str, _Operation]
        self._active = None                   # type: Optional[_Operation]
        self._costmap_cache = None            # type: Optional[Tuple[int, Costmap]]
        self._relaxed_cache = None            # type: Optional[Tuple[int, Costmap]]
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
        robot = request.robot_config
        if self._footprint_override is not None:
            robot = replace(robot, footprint_base=self._footprint_override)
        if self._planning_radius_m is not None:
            low, high = Costmap.planning_radius_bounds(robot.footprint_base)
            if not (low - 1e-9 <= self._planning_radius_m <= high + 1e-9):
                raise NavigatorBoundaryError("planning radius %.4f m must lie between the inscribed radius "
                                             "%.4f m and the circumscribed radius %.4f m of this robot"
                                             % (self._planning_radius_m, low, high))
        self._episode_id = request.episode_id
        self._shared_robot = request.robot_config
        self._robot = robot
        self._obs_cfg = request.observation_config
        self._nav_cfg = request.navigation_config
        self._ops, self._costmap_cache, self._relaxed_cache, self._latest_revision = {}, None, None, -1
        self.events = []
        # Existing native runners retain navigator_events in summary.json, so B
        # can publish resolved settings without changing C's runner or manifest.
        self._log('navigation_configuration', json.dumps(self.effective_configuration(),
                                                        sort_keys=True, allow_nan=False))

    def effective_configuration(self) -> dict:
        """Detached JSON-compatible snapshot of B's resolved settings after reset.

        RobotConfig.clearance_margin_m is the shared input. The strict_costmap
        fields resolve overrides and describe what B actually uses. This method
        neither builds a map nor changes navigation state.
        """
        self._require_ready(self._episode_id)
        low, high = Costmap.planning_radius_bounds(self._robot.footprint_base)
        radius = high if self._planning_radius_m is None else self._planning_radius_m
        margin = self._robot.clearance_margin_m if self._planning_margin_m is None else self._planning_margin_m
        start = margin if self._start_margin_m is None else self._start_margin_m
        def robot_record(robot):
            record = asdict(robot)
            record['footprint_base'] = [asdict(p) for p in robot.footprint_base]
            return record
        def costmap_record(r, m, s):
            disc = circle_footprint(r)
            return dict(planning_radius_m=r, planning_margin_m=m, start_margin_m=s,
                        planning_disc_polygon=[asdict(p) for p in disc],
                        planning_disc_vertex_radius_m=circumradius(disc))
        retry = self._relaxed_retry
        relaxed = (costmap_record(low if retry.planning_radius_m is None else retry.planning_radius_m,
                                  retry.margin_m, retry.start_margin_m) if retry is not None else None)
        return dict(schema_version='1.0', navigator_class=type(self).__module__+'.'+type(self).__name__,
            shared_robot_config=robot_record(self._shared_robot),
            effective_robot_config=robot_record(self._robot), footprint_overridden=self._footprint_override is not None,
            footprint_inscribed_radius_m=low, footprint_vertex_radius_m=high,
            strict_costmap=costmap_record(radius, margin, start), relaxed_costmap=relaxed,
            parameter_sources=dict(footprint='override' if self._footprint_override is not None else 'shared_robot',
                planning_radius='footprint_circumradius' if self._planning_radius_m is None else 'override',
                planning_margin='shared_robot' if self._planning_margin_m is None else 'override',
                start_margin='planning_margin' if self._start_margin_m is None else 'override'),
            controller=dict(linear_gain_per_s=self._linear_gain, path_smoothing=self._smooth,
                near_obstacle_limit=asdict(self._near_limit) if self._near_limit is not None else None),
            navigation_limits=asdict(self._nav_cfg), observation=asdict(self._obs_cfg))

    # --- candidate evaluation (read-only) ----------------------------------

    def evaluate_candidates(self, request: CandidateEvaluationRequest) -> CandidateEvaluation:
        """Reachability and path cost of every frontier from the state's pose.

        One Dijkstra distance field answers all candidates (amortised). A
        timeout leaves unreached cells *inconclusive* (None/TIMEOUT), never
        "unreachable".

        "Read-only" here means: it never moves the robot and never touches the
        active operation. Like every entry point that receives a map, it does
        record the newest map revision offered to B, and ``start`` rejects a map
        older than that (stale-map check). The request's episode must match the
        state's, otherwise the answer would be silently labelled with another
        episode's ID.
        """
        self._require_ready(request.episode_id)
        state = request.state
        if state.episode_id != request.episode_id:
            raise NavigatorBoundaryError("request episode %r does not match state episode %r"
                                         % (request.episode_id, state.episode_id))
        costmap = self._costmap_for(state.occupancy_map)
        start = state.robot_pose
        deadline = self._wall_clock() + self._nav_cfg.batch_deadline_s
        result = planner.distance_field(costmap, start, max(0., deadline-self._wall_clock()),
                                        clock=self._wall_clock)
        assessments = []  # type: List[CandidateAssessment]
        for frontier in state.frontiers:
            valid_goal = False
            assessment = None
            for goal in (frontier.approach_pose,) + frontier.alternative_approach_poses:
                if self._wall_clock() >= deadline:
                    assessment = CandidateAssessment(frontier.frontier_id, None, None, FailureReason.TIMEOUT)
                    break
                if not planner.goal_is_valid(costmap, goal):
                    continue
                valid_goal = True
                if result is not None:
                    field, complete = result
                    cost = planner.cost_to_goal_m(costmap, start, goal, field)
                    if cost is not None:
                        assessment = CandidateAssessment(frontier.frontier_id, True, cost, None, goal)
                        break
            if assessment is None:
                if not valid_goal:
                    assessment = CandidateAssessment(frontier.frontier_id, False, None, FailureReason.INVALID_GOAL)
                elif result is None or result[1]:
                    assessment = CandidateAssessment(frontier.frontier_id, False, None, FailureReason.NO_PATH)
                else:
                    assessment = CandidateAssessment(frontier.frontier_id, None, None, FailureReason.TIMEOUT)
            assessments.append(assessment)
        return CandidateEvaluation(state.episode_id, state.revision, start, tuple(assessments))

    def diagnose_candidates(self, request: CandidateEvaluationRequest) -> Dict[str, Dict[str, object]]:
        """Read-only: why each frontier is or is not reachable, per frontier id (see ``diagnostics``).

        Re-asks the reachability question with smaller inflation to separate a conservative planning model from a
        genuinely blocked frontier. It never touches the active operation and is not part of the frozen Navigator
        contract, so callers must treat it as optional (``getattr``).
        """
        self._require_ready(request.episode_id)
        state = request.state
        radius = self._planning_radius_m
        margin = self._planning_margin_m
        goals = {f.frontier_id: f.approach_pose for f in state.frontiers}
        return diagnostics.diagnose_goals(state.occupancy_map, self._robot, state.robot_pose, goals,
                                          radius, margin, self._start_margin_m)

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
        op.start_pose = live
        result = self._plan(op, live)
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
        op.costmap = self._costmap_for(occupancy_map, relaxed=op.relaxed)
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
        # Cancelling an id that is not the running operation must not stop the running one: latch
        # the record (the id is consumed) but leave motion and the active operation alone.
        other_running = self._active is not None and self._active.terminal is None and self._active is not op
        if op is None:
            op = _Operation(goal_id, "navigate", self._control.sim_time_s())
            self._ops[goal_id] = op
        self._log("cancel", "%s %s" % (goal_id, reason))
        return self._finish(op, NavigationStatus.CANCELLED, None, stop=not other_running)

    # --- observation -------------------------------------------------------

    def start_observation(self, request: ObservationRequest) -> NavigationFeedback:
        self._require_ready(request.episode_id)
        if request.routine not in ("initial", "arrival", "recovery", "inspection"):
            raise NavigatorBoundaryError("unknown observation routine %r" % request.routine)
        op = self._begin(request.goal_id, "observe")
        live = self._control.current_pose()
        if not self._fresh(request.occupancy_map, request.start_pose, live):
            return self._finish(op, NavigationStatus.FAILED, FailureReason.INVALID_GOAL)
        op.costmap = self._costmap_for(request.occupancy_map)
        op.pose = live
        cfg = self._obs_cfg
        op.dwell_s = cfg.inspection_hold_s if request.routine == 'inspection' else cfg.dwell_s
        if request.routine == "initial" and cfg.allow_full_rotation:
            # Full turn in quarter steps, only if clearance permits it.
            op.targets = [_wrap(request.heading_rad + k * pi / 2) for k in range(1, 5)]
        else:
            offsets = {"initial": cfg.initial_headings_rad, "arrival": cfg.arrival_headings_rad,
                       "recovery": cfg.recovery_headings_rad,
                       "inspection": cfg.inspection_headings_rad}[request.routine]
            op.targets = [_wrap(request.heading_rad + o) for o in offsets]
        safe = (self._robot.can_turn_in_place and op.costmap.start_pose_valid(live)
                and self._sweep_safe(op.costmap, live, [live.yaw_rad] + op.targets)
                and (not (request.routine == "initial" and cfg.allow_full_rotation)
                     or op.costmap.full_rotation_valid(live, margin_m=op.costmap.start_margin_m)))
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
        op.status = NavigationStatus.EXECUTING
        if getattr(self._control, 'contact_this_step', False):
            self._log('contact_stop', '%s physical bumper/contact feedback' % op.goal_id)
            self._finish(op, NavigationStatus.FAILED, FailureReason.BLOCKED)
            return
        if self._stalled(op, now):
            self._finish(op, NavigationStatus.FAILED, FailureReason.BLOCKED)
            return
        if op.kind == "navigate":
            self._tick_navigation(op)
        else:
            self._tick_observation(op, now)
        # Deadline last and inclusive. The runner enforces the SAME limit as a backstop with ``>=`` and
        # ends the whole episode (search_limit) when it fires, whereas B's TIMEOUT feeds C's cooldown
        # retry. So B must time out on the step that reaches the limit (``>=``), but only after this
        # step's control, so a goal reached exactly on that step still counts as REACHED.
        if op.terminal is None and now - op.started_s >= limit:
            self._finish(op, NavigationStatus.FAILED, FailureReason.TIMEOUT)

    # --- internals: stall detection ----------------------------------------

    def _stalled(self, op: _Operation, now: float) -> bool:
        """No real movement (position or yaw) for STALL_WINDOW_S of simulated time.

        Catches a robot held up by something the belief map does not show (a
        physical obstacle, a wheel stuck). Deliberate stillness during an
        observation dwell is exempt, but it does NOT restart the window: a robot
        jittering across the heading tolerance re-enters the dwell every few
        steps, and restarting the window each time would hide a real stall until
        the operation deadline. The window restarts when a scan heading is
        completed (see ``_tick_observation``).
        """
        pose = self._control.current_pose()
        if op.kind == "observe" and op.dwell_until is not None:
            return False
        ref = op.stall_pose
        moved = ref is None \
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
        # Acquire position inside half the public tolerance before turning to
        # the goal yaw. Native wheel braking can drift across a single tight
        # threshold, otherwise switching back to bearing control on every
        # crossing. Keep yaw acquisition while still within the FULL tolerance;
        # a real departure must resume position control, never report REACHED.
        if op.turn_ok == "goal" and dist > robot.position_tolerance_m:
            op.turn_ok = None
        position_ready = dist <= robot.position_tolerance_m * .5 or (
            op.turn_ok == "goal" and dist <= robot.position_tolerance_m)
        if op.index == last and position_ready:
            yaw_err = _wrap(op.goal_pose.yaw_rad - pose.yaw_rad)
            if abs(yaw_err) <= robot.yaw_tolerance_rad:
                self._finish(op, NavigationStatus.REACHED, None)
                return
            if op.turn_ok != "goal":  # never rotate blindly into a known obstacle
                if not op.costmap.rotation_valid(pose, op.goal_pose.yaw_rad):
                    self._finish(op, NavigationStatus.FAILED, FailureReason.BLOCKED)
                    return
                op.turn_ok = "goal"
            self._control.set_velocity(0.0, _turn_rate(yaw_err, robot.max_angular_speed_radps))
            return
        target_heading = atan2(target.y_m - pose.y_m, target.x_m - pose.x_m)
        heading_err = _wrap(target_heading - pose.yaw_rad)
        if abs(heading_err) > TURN_IN_PLACE_RAD and op.turn_ok != op.index:
            # An in-place turn is about to be commanded. Planning checked the first move toward the
            # nearest waypoint, but the controller may aim at a later one (lookahead) or the map
            # may have changed, so check the turn it will actually perform, once per waypoint.
            near_start = op.start_pose is not None and hypot(
                pose.x_m - op.start_pose.x_m, pose.y_m - op.start_pose.y_m) <= START_ZONE_M
            if not op.costmap.rotation_valid(pose, target_heading,
                                             margin_m=op.costmap.start_margin_m if near_start else None):
                self._finish(op, NavigationStatus.FAILED, FailureReason.BLOCKED)
                return
            op.turn_ok = op.index
        w = _clip(K_ANGULAR * heading_err, robot.max_angular_speed_radps)
        v = 0.0 if abs(heading_err) > TURN_IN_PLACE_RAD else \
            min(robot.max_linear_speed_mps, self._linear_gain * dist) * max(0.0, cos(heading_err))
        v, w = self._limit_near_obstacle(op, pose, v, w)
        self._control.set_velocity(v, w)

    def _limit_near_obstacle(self, op: _Operation, pose: Pose, v: float, w: float) -> Tuple[float, float]:
        """Cap v and w while the footprint is inside the proximity zone (opt-in; identity when off)."""
        limit = self._near_limit
        if limit is None or not op.costmap.near_obstacle(pose, limit.distance_m):
            return v, w
        return _clip(v, limit.max_linear_mps), _clip(w, limit.max_angular_radps)

    def _tick_observation(self, op: _Operation, now: float) -> None:
        if op.leg >= len(op.targets):
            self._finish(op, NavigationStatus.REACHED, None)
            return
        robot, cfg = self._robot, self._obs_cfg
        pose = self._control.current_pose()
        err = _wrap(op.targets[op.leg] - pose.yaw_rad)
        if op.dwell_until is not None and abs(err) > cfg.heading_tolerance_rad:
            # Wheel inertia can carry a physical robot out of tolerance after
            # braking. Reacquire the heading and restart the dwell; never mark
            # an out-of-tolerance frame as a completed scan.
            op.dwell_until = None
        if op.dwell_until is None:
            if abs(err) > cfg.heading_tolerance_rad:
                self._control.set_velocity(0.0, _turn_rate(err, robot.max_angular_speed_radps))
                return
            self._control.set_velocity(0.0, 0.0)
            op.dwell_until = now + op.dwell_s
        if now >= op.dwell_until:
            op.marks.append(now)  # heading reached and dwelled: designated capture
            op.leg += 1
            op.dwell_until = None
            op.stall_pose, op.stall_since_s = pose, now  # progress: the next heading starts a fresh window
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
        result = self._plan(op, live)
        op.planner_result = result  # newest result is carried in feedback
        op.turn_ok = None
        self._log("replan", "%s #%d success=%s reason=%s"
                  % (op.goal_id, op.replans, result.success, result.failure_reason))
        if not result.success:
            self._finish(op, NavigationStatus.FAILED, result.failure_reason)
            return
        op.path, op.index = result.path, 1

    @staticmethod
    def _sweep_safe(costmap: Costmap, pose: Pose, yaws: List[float]) -> bool:
        """Every leg of a yaw sequence (turning in place at ``pose``) is collision-free.

        The robot never leaves ``pose``, so the sweep uses the same ``start_margin_m`` that accepted
        the pose; a stricter margin here would make a scan "blocked" for a robot parked inside the
        planning buffer even though its start pose was accepted.
        """
        return all(costmap.rotation_valid(Pose(pose.x_m, pose.y_m, pose.z_m, yaws[i]), nxt,
                                          margin_m=costmap.start_margin_m)
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

    def _costmap_for(self, grid: OccupancyGrid, relaxed: bool = False) -> Costmap:
        cache = self._relaxed_cache if relaxed else self._costmap_cache
        if cache is not None and cache[0] == grid.revision and cache[1].grid is grid:
            return cache[1]
        if relaxed:
            retry = self._relaxed_retry
            radius = retry.planning_radius_m
            if radius is None:
                radius = Costmap.planning_radius_bounds(self._robot.footprint_base)[0]
            costmap = Costmap.from_robot(grid, self._robot, radius, retry.start_margin_m, retry.margin_m)
        else:
            costmap = Costmap.from_robot(grid, self._robot, self._planning_radius_m, self._start_margin_m,
                                        self._planning_margin_m)
        costmap.prepare()  # pay the one-off clearance precompute here, not inside a deadline
        if relaxed:
            self._relaxed_cache = (grid.revision, costmap)
        else:
            self._costmap_cache = (grid.revision, costmap)
        self._latest_revision = max(self._latest_revision, grid.revision)
        return costmap

    def _plan(self, op: _Operation, live: Pose) -> PlannerResult:
        """Plan to the operation's goal; one relaxed retry after a strict NO_PATH / INVALID_GOAL, if enabled."""
        result = planner.plan(op.costmap, live, op.goal_pose, self._nav_cfg.batch_deadline_s,
                              clock=self._wall_clock, smooth=self._smooth)
        if (result.success or self._relaxed_retry is None or op.relaxed
                or result.failure_reason not in (FailureReason.NO_PATH, FailureReason.INVALID_GOAL)):
            return result
        relaxed_map = self._costmap_for(op.costmap.grid, relaxed=True)
        retry = planner.plan(relaxed_map, live, op.goal_pose, self._nav_cfg.batch_deadline_s,
                             clock=self._wall_clock, smooth=self._smooth)
        self._log("relaxed_retry", "%s strict=%s relaxed_success=%s"
                  % (op.goal_id, result.failure_reason, retry.success))
        if not retry.success:
            return result  # keep the strict reason: the contract has no "relaxed" failure value
        op.costmap, op.relaxed = relaxed_map, True
        return retry

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
                reason: Optional[FailureReason], stop: bool = True) -> NavigationFeedback:
        """Stop, latch exactly one terminal outcome, and free the single slot.

        ``stop=False`` is only for latching a record for an operation that is not the one driving
        the robot (cancelling an unknown goal id while another operation is running).
        """
        if stop:
            self._control.set_velocity(0.0, 0.0)
        op.status, op.failure_reason = status, reason
        op.terminal = self._feedback(op, status, reason)
        self._log("terminal", "%s %s %s" % (op.goal_id, status.value, reason.value if reason else ""))
        if self._active is op:
            self._active = None
        return op.terminal

    def _log(self, kind: str, detail: str) -> None:
        self.events.append((kind, detail))
