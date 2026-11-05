"""Owner C: synchronous episode lifecycle; use supervisor for hard wall deadlines.

Only the runner advances simulation. A must process every supplied frame before
returning; an asynchronous mapper needs an adapter implementing this barrier.
"""


from dataclasses import replace
from enum import Enum
from math import hypot, pi
from random import Random
from time import monotonic
from typing import Optional, Tuple

from ..contracts import (
    CandidateEvaluation, FrontierExclusion, PlannerAttempt, SelectionContext,
    SelectionResult, WorldState,
    CandidateEvaluationRequest, EvaluationRequest, MapperInit, NavigationRequest,
    NavigationStatus, NavigatorInit, ObservationRequest, PolicyName,
)
from ..evaluation.api import TargetEvaluator
from ..navigation.api import SteppedNavigator
from ..perception.api import PerceptionMapper
from ..policies.api import PolicyFeedback, SearchPolicy
from ..simulation.api import ControlledSimulatorAdapter
from ..validation import require, validate_context, validate_evaluation, validate_selection, validate_state
from ..perception.observations import validate_rgbd_observation
from .manifest import EpisodeManifest
from .recovery import RecoveryManager, SpatialRecovery, probe_candidates, probe_subgoal


class TerminationReason(str, Enum):
    SUCCESS = 'success'
    FALSE_TARGET_CLAIM = 'false_target_claim'
    NO_FRONTIERS = 'no_frontiers'
    NO_FEASIBLE_FRONTIERS = 'no_feasible_frontiers'
    SEARCH_LIMIT = 'search_limit'
    INITIALIZATION_FAILED = 'initialization_failed'
    PIPELINE_ERROR = 'pipeline_error'
    EVALUATION_ERROR = 'evaluation_error'
    EXTERNALLY_CANCELLED = 'externally_cancelled'


def build_context(state: WorldState, evaluation: CandidateEvaluation,
                  exclusions: Tuple[FrontierExclusion, ...] = (),
                  previous_attempt: Optional[PlannerAttempt] = None) -> SelectionContext:
    excluded = {e.frontier_id for e in exclusions}
    reachable = {a.frontier_id for a in evaluation.assessments if a.reachable is True}
    ordinary = {f.frontier_id for f in state.frontiers if f.kind != 'navigation_probe'}
    if (reachable-excluded) & ordinary:
        # Alternate viewpoints are recovery actions, used only when no normal
        # frontier/target approach can be dispatched. Record explicit exclusions
        # so the shared reachable-minus-exclusions contract remains exact.
        deferred = tuple(FrontierExclusion(f.frontier_id,'normal_frontier_available',0,
                            'no_normal_frontier',None) for f in state.frontiers
                         if f.kind=='navigation_probe' and f.frontier_id not in excluded)
        exclusions += deferred
        excluded.update(e.frontier_id for e in deferred)
    eligible = tuple(sorted(a.frontier_id for a in evaluation.assessments
                            if a.reachable is True and a.frontier_id not in excluded))
    context = SelectionContext(state.episode_id, evaluation, eligible, exclusions, previous_attempt)
    validate_context(state, context)
    return context


def decide(policy: SearchPolicy, state: WorldState, context: SelectionContext,
           *, oracle_room_labels: bool = False) -> SelectionResult:
    """Call only while stopped/paused with a fresh state and matching assessment."""
    validate_state(state, oracle_room_labels=oracle_room_labels)
    validate_context(state, context)
    result = policy.select(state, context)
    validate_selection(state, context, result)
    return result


class EpisodeRunner:
    def __init__(self, simulator: ControlledSimulatorAdapter, mapper: PerceptionMapper,
                 navigator: SteppedNavigator, policy: SearchPolicy, evaluator: Optional[TargetEvaluator] = None,
                 recovery: Optional[RecoveryManager] = None, logger=None,
                 wall_clock=monotonic, cancelled=None) -> None:
        self.simulator = simulator
        self.mapper = mapper
        self.navigator = navigator
        self.policy = policy
        self._evaluator = evaluator  # Never forwarded to the policy.
        self.recovery = recovery
        self.logger = logger
        self.clock = wall_clock
        self.cancelled = cancelled or (lambda: False)
        self.summary = None

    def _emit(self, event, **payload):
        if self.logger is not None:
            self.logger.write(event, self.manifest.episode_id, source=self.manifest.source, **payload)

    def _call(self, name, function, *args, **kwargs):
        deadline = kwargs.pop('_deadline', self.manifest.module_deadline_s)
        self._emit('module_begin', module=name, deadline_s=deadline)
        started = self.clock()
        try:
            value = function(*args, **kwargs)
        finally:
            elapsed = self.clock()-started
            self._emit('module_end', module=name, elapsed_s=elapsed)
        if elapsed > deadline:
            raise TimeoutError('module exceeded deadline: ' + name)
        return value

    def _notify_policy(self, frontier, subgoal, action_source, navigation_feedback,
                       new_known_area_m2=None, observation_status=None):
        plan = navigation_feedback.planner_result
        feedback = PolicyFeedback(
            episode_id=self.manifest.episode_id,
            decision_number=self.decisions,
            frontier_id=subgoal.frontier_id,
            frontier_kind=subgoal.kind,
            region_id=frontier.region_id,
            action_source=action_source,
            selected_from_revision=subgoal.selected_from_revision,
            completed_revision=self.state.revision,
            timestamp_s=self.sim_time,
            navigation_status=navigation_feedback.status,
            failure_reason=navigation_feedback.failure_reason,
            path_cost_m=plan.path_cost_m if plan is not None else None,
            new_known_area_m2=new_known_area_m2,
            observation_status=observation_status,
        )
        observer = getattr(self.policy, 'observe_feedback', None)
        if observer is not None:
            self._call('policy.observe_feedback', observer, feedback)
        self._emit('policy_feedback', direction='A_B_to_C', feedback=feedback)

    def _budget(self):
        if self.cancelled():
            raise _End(TerminationReason.EXTERNALLY_CANCELLED)
        cfg = self.manifest.limits
        if (self.steps >= cfg.max_steps or self.sim_time >= cfg.max_simulated_time_s or
                self.clock()-self.started >= cfg.max_wall_time_s):
            raise _End(TerminationReason.SEARCH_LIMIT)

    def _capture(self):
        self._budget()
        obs = self._call('capture', self.simulator.capture)
        validate_rgbd_observation(obs)
        if self.intrinsics is None:
            self.intrinsics = obs.camera_intrinsics
            self._emit('sensor_calibration', camera_intrinsics=obs.camera_intrinsics,
                       rgb_encoding=obs.color_image.encoding, depth_encoding=obs.depth_image.encoding,
                       depth_units='metres', depth_measurement='camera_z', invalid_depth_value=0.,
                       camera_axes=('right', 'down', 'forward'))
        require(obs.camera_intrinsics == self.intrinsics, 'camera calibration changed within episode')
        require(obs.episode_id == self.manifest.episode_id, 'capture episode mismatch')
        require(obs.sequence > self.last_capture, 'capture sequence did not increase')
        require(obs.timestamp_s >= self.last_capture_time and
                abs(obs.timestamp_s-self.sim_time) < 1e-6, 'capture must describe current simulation time')
        self.last_capture, self.last_capture_time = obs.sequence, obs.timestamp_s
        state = self._call('mapper.update', self.mapper.update, obs,
                           _deadline=min(self.manifest.module_deadline_s,
                                         self.manifest.observation.drain_deadline_s))
        validate_state(state, oracle_room_labels=self.manifest.oracle_room_labels)
        require(state.episode_id == obs.episode_id and state.target_class == self.manifest.target_class,
                'mapper changed episode or target')
        require(state.last_processed_sequence == obs.sequence and state.timestamp_s == obs.timestamp_s
                and state.robot_pose == obs.robot_pose, 'mapper failed processed-frame barrier')
        cfg, grid = self.manifest.mapping, state.occupancy_map
        require((grid.resolution_m, grid.origin_x_m, grid.origin_y_m, grid.width_cells, grid.height_cells) ==
                (cfg.resolution_m, cfg.origin_x_m, cfg.origin_y_m, cfg.width_cells, cfg.height_cells),
                'mapper changed configured grid geometry')
        require(state.revision > (self.state.revision if self.state else -1), 'state revision did not increase')
        for frontier in state.frontiers:
            require(all(parent in self.seen_ids for parent in frontier.parent_frontier_ids),
                    'frontier parent must refer to an earlier snapshot')
        self.seen_ids.update(f.frontier_id for f in state.frontiers)
        self.state = state
        known = sum(v != -1 for row in grid.data for v in row)
        self._emit('capture_processed', sequence=obs.sequence, revision=state.revision,
                   simulated_time_s=self.sim_time, known_cells=known,
                   odometry_linear_speed_mps=obs.odometry_linear_speed_mps,
                   odometry_angular_speed_radps=obs.odometry_angular_speed_radps,
                   explored_cells=sum(sum(row) for row in state.explored_map.data))
        self._budget()
        if state.target_claim is not None:
            if not self.manifest.enable_target_evaluation:
                raise ValueError('W2 geometry mode must not publish target claims')
            raise _Claim()
        if self.active is not None:
            event_offset = len(getattr(self.navigator, 'events', ()))
            self._call('navigator.update_map', self.navigator.update_map,
                       self.manifest.episode_id, self.active, grid)
            for kind, detail in getattr(self.navigator, 'events', ())[event_offset:]:
                self._emit('navigation_map_feedback', kind=kind, detail=detail,
                           revision=state.revision, simulated_time_s=self.sim_time, goal_id=self.active)
        return state

    def _step(self):
        self._budget()
        before = self._call('current_pose', self.simulator.current_pose)
        self._call('simulator.step', self.simulator.step)
        self.steps += 1
        now = self._call('sim_time_s', self.simulator.sim_time_s)
        require(now > self.sim_time, 'simulation clock did not advance')
        self.sim_time = now
        after = self._call('current_pose', self.simulator.current_pose)
        self.distance += hypot(after.x_m-before.x_m, after.y_m-before.y_m)
        self._call('navigator.on_step', self.navigator.on_step)
        self._emit('motion_sample', steps=self.steps, simulated_time_s=self.sim_time,
                   pose=after, actual_distance_m=self.distance,
                   collisions=getattr(self.simulator, 'collisions', None))
        self._budget()

    def _feedback(self, feedback, kind):
        require(feedback.episode_id == self.manifest.episode_id and feedback.goal_id == self.active,
                'feedback identity mismatch')
        require(feedback.timestamp_s <= self.sim_time+1e-6, 'feedback from the future')
        require(isinstance(feedback.status, NavigationStatus), 'unknown feedback status')
        require((feedback.status == NavigationStatus.FAILED) == (feedback.failure_reason is not None),
                'invalid feedback failure reason')
        if kind == 'navigate' and feedback.status == NavigationStatus.REACHED:
            require(feedback.planner_result is not None and feedback.planner_result.success,
                    'reached without a successful plan')
        plan = feedback.planner_result
        if plan is not None:
            require((plan.success and bool(plan.path) and plan.path_cost_m is not None and
                     plan.path_cost_m >= 0 and plan.failure_reason is None) or
                    (not plan.success and plan.path is None and plan.path_cost_m is None and
                     plan.failure_reason is not None), 'invalid planner result')
        # Log transitions/planner changes, not one duplicate record per poll.
        fingerprint = (feedback.status, feedback.planner_result, feedback.failure_reason)
        if fingerprint != self.last_feedback:
            self._emit('operation_feedback', feedback=feedback, kind=kind)
            self.last_feedback = fingerprint

    def _operation(self, kind, subgoal=None, routine=None, heading=None):
        self.op_serial += 1
        self.active = 'op-%d' % self.op_serial
        self.last_feedback = None
        episode = self.manifest.episode_id
        self._call('resume', self.simulator.resume)
        pose = self._call('current_pose', self.simulator.current_pose)
        if kind == 'navigate':
            request = NavigationRequest(episode, self.active, self.state.occupancy_map, pose, subgoal)
            self._emit('dispatch', request=request, assessment_revision=subgoal.selected_from_revision,
                       dispatch_revision=self.state.revision, simulated_time_s=self.sim_time)
            feedback = self._call('navigator.start', self.navigator.start, request)
            limit = self.manifest.navigation.navigation_deadline_s
        else:
            request = ObservationRequest(episode, self.active, self.state.occupancy_map, pose, routine, heading)
            self._emit('observation_start', request=request)
            feedback = self._call('navigator.start_observation', self.navigator.start_observation, request)
            limit = self.manifest.observation.operation_deadline_s
            cfg = self.manifest.observation
            targets = ([heading+k*pi/2 for k in range(1, 5)] if routine == 'initial' and cfg.allow_full_rotation
                       else [heading+offset for offset in getattr(cfg, routine+'_headings_rad')])
        started, wall_started, marks_seen = self.sim_time, self.clock(), 0
        terminal = (NavigationStatus.REACHED, NavigationStatus.FAILED, NavigationStatus.CANCELLED)
        while True:
            self._feedback(feedback, kind)
            if kind == 'observe':
                marks = self._call('capture_marks', self.navigator.capture_marks, self.active)
                require(len(marks) >= marks_seen, 'scan marks disappeared')
                for index, mark in enumerate(marks[marks_seen:], marks_seen):
                    require(abs(mark-self.sim_time) < 1e-6, 'missed designated scan frame')
                    require(index < len(targets), 'too many scan marks')
                    pose = self._call('current_pose', self.simulator.current_pose)
                    require(abs((pose.yaw_rad-targets[index]+pi) % (2*pi)-pi) <= cfg.heading_tolerance_rad,
                            'scan mark at wrong heading')
                    sequences = []
                    for _ in range(self.manifest.observation.processed_samples_per_heading):
                        sequences.append(self._capture().last_processed_sequence)
                    self._emit('scan_barrier', goal_id=self.active, mark_s=mark, sequences=tuple(sequences))
                marks_seen = len(marks)
            if feedback.status in terminal:
                break
            if self.sim_time-started >= limit or self.clock()-wall_started >= limit:
                self._call('navigator.cancel', self.navigator.cancel, episode, self.active, 'runner_operation_timeout')
                # A cancelled B operation is not falsely recorded as B FAILED.
                self._emit('operation_timeout', goal_id=self.active, kind=kind)
                raise _End(TerminationReason.SEARCH_LIMIT)
            self._step()
            if self.steps % self.manifest.capture_every_steps == 0:
                self._capture()
            feedback = self._call('navigator.poll', self.navigator.poll, episode, self.active)
        self._call('stop', self.simulator.stop)
        if kind == 'observe' and feedback.status == NavigationStatus.REACHED:
            cfg = self.manifest.observation
            expected = 4 if routine == 'initial' and cfg.allow_full_rotation else len(
                getattr(cfg, routine+'_headings_rad'))
            require(marks_seen == expected, 'observation completed without every heading mark')
        self.active = None
        self._capture()  # terminal-time frame, including immediate failure
        self._emit('operation_terminal', feedback=feedback, kind=kind, routine=routine,
                   processed_sequence=self.state.last_processed_sequence)
        if kind == 'navigate' and feedback.status == NavigationStatus.FAILED:
            self._emit('replan_request', direction='B_to_C', feedback=feedback,
                       simulated_time_s=self.sim_time)
        return feedback

    def _fresh_pose(self):
        now = self._call('sim_time_s', self.simulator.sim_time_s)
        require(now == self.sim_time, 'simulation advanced outside runner ownership')
        pose = self._call('current_pose', self.simulator.current_pose)
        recorded = self.state.robot_pose
        yaw = abs((pose.yaw_rad-recorded.yaw_rad+pi) % (2*pi)-pi)
        return (hypot(pose.x_m-recorded.x_m, pose.y_m-recorded.y_m) <=
                self.manifest.navigation.pose_freshness_m and
                abs(pose.z_m-recorded.z_m) <= self.manifest.navigation.pose_freshness_m and
                yaw <= self.manifest.robot.yaw_tolerance_rad and
                self.sim_time-self.state.timestamp_s <= self.manifest.navigation.map_freshness_s)

    def _loop(self):
        initial = self._operation('observe', routine='initial', heading=self.state.robot_pose.yaw_rad)
        if initial.status != NavigationStatus.REACHED:
            retry = self._operation('observe', routine='recovery', heading=self.state.robot_pose.yaw_rad)
            if retry.status != NavigationStatus.REACHED:
                raise _End(TerminationReason.INITIALIZATION_FAILED)
        recovered = False
        idle_cycles = 0
        while True:
            self._budget()
            self._call('stop', self.simulator.stop)
            self._capture()
            self._call('pause', self.simulator.pause)
            require(self._fresh_pose(), 'mapper pose is stale at decision boundary')
            evaluation = self._call('navigator.evaluate', self.navigator.evaluate_candidates,
                CandidateEvaluationRequest(self.manifest.episode_id, self.state))
            validate_evaluation(self.state, evaluation)
            decision_state = self.state
            if any(a.approach_pose is not None and a.reachable is True and
                   a.approach_pose != next(f.approach_pose for f in self.state.frontiers
                                          if f.frontier_id == a.frontier_id)
                   for a in evaluation.assessments):
                decision_state = self._call('mapper.resolve_approaches', self.mapper.resolve_approaches,
                                            self.state, evaluation)
            self.recovery.record_evaluation(decision_state, evaluation)
            exclusions = self.recovery.exclusions(decision_state, evaluation)
            context = build_context(decision_state, evaluation, exclusions, self.previous)
            if self.decisions >= self.manifest.limits.max_decisions:
                raise _End(TerminationReason.SEARCH_LIMIT)
            self.decisions += 1
            selection = self._call('policy.select', decide, self.policy, decision_state, context,
                                   oracle_room_labels=self.manifest.oracle_room_labels)
            policy_selection = selection
            action_source = 'policy'
            self._emit('decision', state=decision_state, context=context, selection=selection)
            records = getattr(self.policy, 'call_records', ())
            for record in records[self.llm_records:]:
                self._emit('llm_call', record=record)
            self.llm_records = len(records)
            self._budget()
            if not self._fresh_pose():
                self._emit('stale_dispatch', revision=self.state.revision)
                continue  # bounded by decision/step/wall budgets; recapture and reassess
            epsilon = self.manifest.recovery.epsilon_retry
            if epsilon > 0 and selection.stop_reason != 'target_claimed':
                draw = self._probe_rng.random()  # one draw per decision, whether or not a probe is possible
                pool = probe_candidates(decision_state, exclusions, self.recovery)
                if draw < epsilon and pool:
                    chosen = pool[int(self._probe_rng.random() * len(pool))]
                    self._emit('epsilon_probe', mode='epsilon', frontier_id=chosen.frontier_id,
                               replaced=selection.subgoal.frontier_id if selection.subgoal else None)
                    selection = replace(selection, subgoal=probe_subgoal(
                        decision_state, chosen, PolicyName(self.manifest.policy_name)), stop_reason=None)
                    action_source = 'epsilon_retry'
            if selection.subgoal is None:
                if not recovered:
                    recovered = True
                    self._operation('observe', routine='recovery', heading=self.state.robot_pose.yaw_rad)
                    continue
                cooldowns = [e.retry_after_s for e in exclusions if e.retry_after_s is not None
                             and e.retry_after_s > self.sim_time]
                if cooldowns and idle_cycles < self.manifest.recovery.max_cycles_without_navigation:
                    idle_cycles += 1
                    until = min(cooldowns)
                    self._emit('cooldown_wait', until_s=until)
                    self._call('resume', self.simulator.resume)
                    while self.sim_time < until:
                        self._step()
                        if self.steps % self.manifest.capture_every_steps == 0:
                            self._capture()
                    continue
                diagnose = getattr(self.navigator, 'diagnose_candidates', None)
                if diagnose is not None and self.state.frontiers:
                    try:  # diagnostics are evidence for the report and must never end an episode
                        self._emit('frontier_diagnostics', revision=self.state.revision,
                                   frontiers=diagnose(CandidateEvaluationRequest(self.manifest.episode_id, self.state)))
                    except Exception as exc:
                        self._emit('frontier_diagnostics_error', error_type=type(exc).__name__)
                pool = probe_candidates(decision_state, exclusions, self.recovery) if self.manifest.recovery.stuck_retry else []
                if selection.stop_reason == 'target_claimed' or not pool:
                    raise _End(TerminationReason.NO_FRONTIERS if not self.state.frontiers
                               else TerminationReason.NO_FEASIBLE_FRONTIERS)
                self._emit('epsilon_probe', mode='stuck', frontier_id=pool[0].frontier_id, replaced=None)
                selection = replace(selection, subgoal=probe_subgoal(
                    decision_state, pool[0], PolicyName(self.manifest.policy_name)), stop_reason=None)
                action_source = 'stuck_retry'
            if self.attempts >= self.manifest.limits.max_attempts:
                raise _End(TerminationReason.SEARCH_LIMIT)
            before = decision_state
            self.attempts += 1
            self.recovery.record_dispatch(before, selection.subgoal.frontier_id)
            self._emit('action_selected', decision_number=self.decisions, state_revision=self.state.revision,
                       simulated_time_s=self.sim_time, policy_selection=policy_selection,
                       selection=selection, action_source=action_source)
            self._emit('strategy_directive', direction='C_to_B', decision_number=self.decisions,
                       selected=selection.subgoal, policy_selection=policy_selection.subgoal,
                       rationale=policy_selection.rationale, action_source=action_source,
                       candidate_ranking=getattr(self.policy, 'last_candidate_ranking', ()),
                       eligible_frontier_ids=context.eligible_frontier_ids)
            if self.previous is not None and self.previous.execution_status == 'failed':
                self._emit('strategy_replan', direction='C_to_B', previous_attempt=self.previous,
                           selected=selection.subgoal, action_source=action_source,
                           simulated_time_s=self.sim_time)
            frontier = next(item for item in before.frontiers
                            if item.frontier_id == selection.subgoal.frontier_id)
            feedback = self._operation('navigate', subgoal=selection.subgoal)
            self.previous = PlannerAttempt(selection.subgoal, feedback.planner_result,
                                           feedback.status.value, feedback.failure_reason)
            # Cooldowns start at terminal time, with dispatch geometry as evidence.
            self.recovery.record_attempt(replace(before, timestamp_s=self.sim_time), self.previous)
            recovered, idle_cycles = False, 0
            if feedback.status == NavigationStatus.CANCELLED:
                self._notify_policy(frontier, selection.subgoal, action_source, feedback)
                raise _End(TerminationReason.EXTERNALLY_CANCELLED)
            if feedback.status == NavigationStatus.FAILED:
                self._notify_policy(frontier, selection.subgoal, action_source, feedback)
            if feedback.status == NavigationStatus.REACHED:
                try:
                    if (selection.subgoal.kind == 'target_approach' and
                            self.manifest.observation.inspection_hold_s > 0):
                        # Shared, fixed micro-sweep: RGB detection can vary with a
                        # few degrees of view. Each safe heading is held for fresh,
                        # stationary evidence; no hidden target state is consulted.
                        self._operation('observe', routine='inspection', heading=selection.subgoal.approach_pose.yaw_rad)
                    arrival = self._operation('observe', routine='arrival', heading=selection.subgoal.approach_pose.yaw_rad)
                except _Claim:
                    # A terminal claim may pre-empt the arrival operation. Preserve
                    # B's completed navigation feedback before scoring ends the loop.
                    self._notify_policy(frontier, selection.subgoal, action_source, feedback)
                    raise
                if arrival.status != NavigationStatus.REACHED:
                    self._operation('observe', routine='recovery', heading=self.state.robot_pose.yaw_rad)
                    self.recovery.record_observation_failure(before, self.state, selection.subgoal.frontier_id)
                    recovered = True
                area = self.recovery.record_progress(before, self.state, selection.subgoal.frontier_id)
                self._emit('progress', frontier_id=selection.subgoal.frontier_id, new_known_area_m2=area)
                self._notify_policy(frontier, selection.subgoal, action_source, feedback,
                                    new_known_area_m2=area, observation_status=arrival.status)

    def _score(self):
        self._call('stop', self.simulator.stop)
        if self.active:
            self._call('navigator.cancel', self.navigator.cancel, self.manifest.episode_id, self.active, 'target_claim')
            self.active = None
        require(self._evaluator is not None, 'target evaluation enabled without evaluator')
        claim = self.state.target_claim
        evidence = self._call('claim_evidence', self.mapper.get_claim_evidence, self.manifest.episode_id, claim.claim_id)
        require(evidence.episode_id == self.manifest.episode_id and evidence.claim_id == claim.claim_id,
                'evidence identity mismatch')
        require({d.observation_id for d in evidence.detections} == set(claim.evidence_observation_ids)
                and len({d.capture_sequence for d in evidence.detections}) >=
                self.manifest.evaluation.required_evidence_frames, 'missing distinct claim evidence frames')
        require(all(d.track_id == claim.track_id and d.class_name == claim.target_class and
                    d.position_world is not None and 0 <= d.timestamp_s <= self.state.timestamp_s
                    and d.capture_sequence <= self.state.last_processed_sequence
                    for d in evidence.detections), 'claim evidence does not match localized target')
        result = self._call('evaluator.evaluate', self._evaluator.evaluate,
            EvaluationRequest(self.manifest.episode_id, claim, evidence),
            _deadline=self.manifest.evaluation.deadline_s)
        require(result.episode_id == self.manifest.episode_id and result.claim_id == claim.claim_id
                and type(result.valid_target) is bool, 'evaluation identity/value mismatch')
        self._emit('target_evaluation', claim=claim, evidence=evidence, result=result)
        return TerminationReason.SUCCESS if result.valid_target else TerminationReason.FALSE_TARGET_CLAIM

    def run(self, manifest: EpisodeManifest) -> TerminationReason:
        self.manifest = manifest
        self.started = self.clock()
        self.steps = self.attempts = self.decisions = self.op_serial = self.llm_records = 0
        self.distance = self.sim_time = 0.0
        self.last_capture, self.last_capture_time = -1, -1.0
        self.state = self.active = self.previous = None
        self.intrinsics = None
        self.seen_ids = set()
        phase = 'initialization'
        reason = TerminationReason.INITIALIZATION_FAILED
        try:
            manifest.validate()
            self._emit('episode_start', manifest=manifest)
            self._call('simulator.reset', self.simulator.reset, manifest.episode_id, manifest.scene_id,
                       manifest.start_pose, manifest.environment_seed,
                       _deadline=manifest.initialization_deadline_s or manifest.module_deadline_s)
            self.sim_time = self._call('sim_time_s', self.simulator.sim_time_s)
            require(self.sim_time == 0.0, 'episode clock must start at zero')
            self._call('stop', self.simulator.stop)
            clearance = self._call('verify_spawn_clearance', self.simulator.verify_spawn_clearance)
            if manifest.initial_clearance_mode == 'verified_start_area':
                verified = self._call('verify_start_area', self.simulator.verify_start_area,
                                      manifest, 'integrated-scene-column-check-v1')
                clearance = verified.cells
                self._emit('verified_start_area', verification=verified)
            require(bool(clearance), 'spawn clearance is empty')
            self._call('mapper.reset', self.mapper.reset, MapperInit(manifest.episode_id, manifest.target_class,
                manifest.perception_seed, manifest.mapping, manifest.observation, manifest.robot,
                manifest.region_priors, clearance, manifest.oracle_room_labels))
            self._call('navigator.reset', self.navigator.reset, NavigatorInit(manifest.episode_id, manifest.robot,
                       manifest.observation, manifest.navigation))
            self._call('policy.reset', self.policy.reset, manifest.episode_id, manifest.policy_seed, manifest.policy)
            if manifest.enable_target_evaluation:
                require(self._evaluator is not None, 'evaluation enabled without evaluator')
                self._call('evaluator.reset', self._evaluator.reset, manifest.episode_id, manifest.evaluation)
            if self.recovery is None:
                self.recovery = SpatialRecovery(manifest.recovery)
            elif isinstance(self.recovery, SpatialRecovery):
                self.recovery.config = manifest.recovery
            self.recovery.reset(manifest.episode_id)
            # Own stream, so drawing a probe never disturbs the Random policy's RNG; paired runs share the seed.
            self._probe_rng = Random(manifest.policy_seed * 1000003 + 17)
            self._capture()
            phase = 'running'
            self._loop()
        except _Claim:
            try:
                reason = self._score()
            except Exception as exc:
                reason = TerminationReason.EVALUATION_ERROR
                self._emit('pipeline_error', phase='evaluation', error_type=type(exc).__name__, error_message=str(exc))
        except _End as stop:
            reason = stop.reason
        except KeyboardInterrupt:
            reason = TerminationReason.EXTERNALLY_CANCELLED
        except Exception as exc:
            reason = (TerminationReason.INITIALIZATION_FAILED if phase == 'initialization'
                      else TerminationReason.PIPELINE_ERROR)
            self._emit('pipeline_error', phase=phase, error_type=type(exc).__name__, error_message=str(exc))
        finally:
            for name, function, args in (
                ('stop', self.simulator.stop, ()),
                ('navigator.cancel', self.navigator.cancel, (manifest.episode_id, self.active, 'episode_terminal')),
                ('close', self.simulator.close, ())):
                if name == 'navigator.cancel' and self.active is None:
                    continue
                try:
                    self._call(name, function, *args)
                except Exception as exc:
                    self._emit('cleanup_error', module=name, error_type=type(exc).__name__)
                    reason = TerminationReason.PIPELINE_ERROR
            self.summary = dict(termination=reason.value, decisions=self.decisions, attempts=self.attempts,
                steps=self.steps, simulated_time_s=self.sim_time, wall_time_s=self.clock()-self.started,
                actual_distance_m=self.distance, collisions=getattr(self.simulator, 'collisions', None),
                final_revision=self.state.revision if self.state else None)
            self._emit('episode_terminal', **self.summary)
        return reason


class _End(Exception):
    def __init__(self, reason):
        self.reason = reason


class _Claim(Exception):
    pass
