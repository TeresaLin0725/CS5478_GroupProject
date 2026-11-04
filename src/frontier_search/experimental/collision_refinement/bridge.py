"""Optional mapper and navigator adapters; stock runner and policies remain unchanged."""
from time import perf_counter
from math import hypot
from ...navigation.navigator import TeamNavigator
from ...contracts import NavigationStatus, FailureReason
from ...perception.mapper import RGBDMapper, _validate_init
from .rolling_local_map import RollingLocalMap
from .patch import LocalCollisionPatch, VERSION, RELEASE_SCOPES, current_conflict_cells
from .costmap import RefinedCostmap


class LocalCollisionMapper(RGBDMapper):
    def __init__(self, detector=None, *, release_scope='window', retain_history=False):
        if release_scope not in RELEASE_SCOPES:
            raise ValueError('unsupported local collision release scope')
        if type(retain_history) is not bool:
            raise ValueError('retain_history must be bool')
        super().__init__(detector)
        self.release_scope = release_scope
        self.retain_history = retain_history

    def reset(self, request):
        _validate_init(request)
        local = RollingLocalMap(request,retain_history=self.retain_history)
        super().reset(request)
        self._local = local
        self.collision_patch = None
        self.local_seconds = 0.
        self.local_frames = 0

    def update(self, observation):
        previous, old_patch = self._state, self.collision_patch
        local_state = self._local.__dict__.copy()
        state = super().update(observation)
        started = perf_counter()
        try:
            grid = self._local.update(observation,self._state.collision_z,state.revision)
            self.collision_patch = LocalCollisionPatch(state,grid,self._state.request.robot_config,
                self._state.request.mapping_config,self._state.collision_z,self._state.base_z_m,
                release_cells=current_conflict_cells(state,self._state.request.robot_config)
                    if self.release_scope=='current_conflicts' else None)
        except Exception:
            self._state, self.collision_patch = previous, old_patch
            self._local.__dict__.clear();self._local.__dict__.update(local_state)
            raise
        self.local_seconds += perf_counter()-started
        self.local_frames += 1
        return state


class LocalCollisionNavigator(TeamNavigator):
    def __init__(self, control, *, patch_provider, mapping_config, release_scope='window', **options):
        if release_scope not in RELEASE_SCOPES:
            raise ValueError('unsupported local collision release scope')
        self._release_scope = release_scope
        self._patch_provider, self._mapping = patch_provider, mapping_config
        self._refined_cache = {}
        super().__init__(control,**options)

    def reset(self, request):
        self._initial_base_z = self._control.current_pose().z_m
        self._refined_cache = {}
        super().reset(request)

    def effective_configuration(self):
        result = super().effective_configuration()
        result['local_collision_refinement'] = dict(enabled=True,version=VERSION,
            resolution_m=.05,extent_m=3.,shift_distance_m=.4,max_age_s=self._nav_cfg.map_freshness_s,
            global_planning_lattice_unchanged=True,release_scope=self._release_scope,
            goal_position_hold_separate=True)
        return result

    def _costmap_for(self, grid, relaxed=False):
        base = super()._costmap_for(grid,relaxed)
        patch = self._patch_provider()
        now = self._control.sim_time_s()
        cached = self._refined_cache.get(relaxed)
        if cached is not None and cached.coarse is base and cached.patch is patch and cached.is_current():
            return cached
        if (not isinstance(patch,LocalCollisionPatch)
                or (patch.release_cells is None) != (self._release_scope=='window')
                or not patch.matches(grid,self._episode_id,
                self._robot,self._mapping,self._initial_base_z,now,self._nav_cfg.map_freshness_s)):
            return base
        key = relaxed
        def current():
            return (self._patch_provider() is patch and
                0 <= self._control.sim_time_s()-patch.state.timestamp_s <= self._nav_cfg.map_freshness_s+1e-9)
        result = RefinedCostmap(base,patch,current)
        self._refined_cache[key] = result
        return result

    def on_step(self):
        op = self._active
        updated = self._costmap_for(op.costmap.grid,op.relaxed) if op is not None and op.terminal is None else None
        if op is not None and op.terminal is None and (updated is not op.costmap or
                isinstance(op.costmap,RefinedCostmap) and not op.costmap.is_current()):
            self._control.set_velocity(0.,0.)
            op.costmap = updated
            self._invalidate_turn_proof(op)
            live = self._control.current_pose()
            safe = (self._remaining_path_valid(op,live) if op.kind=='navigate' else
                op.costmap.start_pose_valid(live) and self._sweep_safe(op.costmap,live,[live.yaw_rad]+op.targets[op.leg:]))
            if not safe:
                self._finish(op,NavigationStatus.FAILED,FailureReason.BLOCKED)
                return
        super().on_step()

    def update_map(self, episode_id, goal_id, occupancy_map):
        op = self._ops.get(goal_id)
        old = op.costmap if op is not None else None
        super().update_map(episode_id,goal_id,occupancy_map)
        if op is not None and op.terminal is None and op.costmap is not old:
            self._invalidate_turn_proof(op)

    @staticmethod
    def _invalidate_turn_proof(op):
        # The core uses 'goal' for both position hysteresis and a cached safe
        # turn. Preserve only the position fact when invalidating that proof.
        if op.turn_ok=='goal':
            op._local_goal_position_acquired = True
        op.turn_ok = None

    def _tick_navigation(self, op):
        if op.turn_ok=='goal':
            op._local_goal_position_acquired = True
        if getattr(op,'_local_goal_position_acquired',False):
            live = self._control.current_pose()
            if (op.index != len(op.path)-1 or
                    hypot(live.x_m-op.goal_pose.x_m,live.y_m-op.goal_pose.y_m)>self._robot.position_tolerance_m):
                op._local_goal_position_acquired = False
                if op.turn_ok=='goal':op.turn_ok = None
            elif op.turn_ok!='goal':
                # A fresh proof is mandatory before allowing the core to keep
                # its goal-position latch. New hazards still stop the robot.
                if not op.costmap.rotation_valid(live,op.goal_pose.yaw_rad):
                    self._finish(op,NavigationStatus.FAILED,FailureReason.BLOCKED)
                    return
                op.turn_ok = 'goal'
        super()._tick_navigation(op)
        op._local_goal_position_acquired = op.turn_ok=='goal'

    def _replan(self, op, live):
        op._local_goal_position_acquired = False
        super()._replan(op,live)

    def diagnose_candidates(self, request):
        result = self.evaluate_candidates(request)
        costmap = self._costmap_for(request.state.occupancy_map)
        start_ok = costmap.start_pose_valid(request.state.robot_pose)
        return {a.frontier_id:dict(cause='start_unusable' if not start_ok else
            'reachable' if a.reachable is True else 'blocked' if a.reachable is False else 'inconclusive',
            local_collision_refinement=isinstance(costmap,RefinedCostmap),strict_start_usable=start_ok)
            for a in result.assessments}
