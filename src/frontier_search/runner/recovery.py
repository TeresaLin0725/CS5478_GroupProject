"""Owner C: bounded recovery with episode-long spatial and lineage history."""
from dataclasses import dataclass, field
from math import floor
from typing import Protocol, Tuple


from ..contracts import CandidateEvaluation, FrontierExclusion, PlannerAttempt, WorldState
from ..config import RecoveryConfig
from ..validation import require


class RecoveryManager(Protocol):
    def reset(self, episode_id: str) -> None: ...
    def record_dispatch(self, state: WorldState, frontier_id: str) -> None: ...
    def record_evaluation(self, state: WorldState, evaluation: CandidateEvaluation) -> None: ...
    def record_attempt(self, state_at_dispatch: WorldState, attempt: PlannerAttempt) -> None: ...
    def record_progress(self, before: WorldState, after: WorldState, frontier_id: str) -> float: ...
    def record_observation_failure(self, before: WorldState, after: WorldState, frontier_id: str) -> None: ...
    def exclusions(self, state: WorldState, evaluation: CandidateEvaluation) -> Tuple[FrontierExclusion, ...]: ...


@dataclass
class _Opportunity:
    ids: set = field(default_factory=set)
    support: set = field(default_factory=set)
    attempts: set = field(default_factory=set)
    failures: set = field(default_factory=set)
    no_progress: set = field(default_factory=set)
    timeouts: set = field(default_factory=set)
    # Each restriction survives merges; any still-locked restriction excludes.
    restrictions: list = field(default_factory=list)


class SpatialRecovery:
    """Conservative one-cell overlap matching; split children share hard caps.

    Historical support is retained for the full episode. Relevant evidence is
    point-grid free connectivity plus the candidate neighborhood/approach pose;
    B still reassesses full-footprint reachability after an unlock.
    """
    def __init__(self, config: RecoveryConfig = RecoveryConfig()) -> None:
        self.config = config
        self.reset('')

    def reset(self, episode_id: str) -> None:
        self.episode_id = episode_id
        self.groups = []
        self.serial = 0

    def _group(self, frontier):
        cells = {(c.row, c.column) for c in frontier.support_cells}
        vicinity = {(r+dr, c+dc) for r, c in cells for dr in (-1, 0, 1) for dc in (-1, 0, 1)}
        ids = {frontier.frontier_id, *frontier.parent_frontier_ids}
        matches = [g for g in self.groups if g.ids & ids or g.support & vicinity]
        group = matches[0] if matches else _Opportunity()
        if not matches:
            self.groups.append(group)
        for other in matches[1:]:
            for key in ('ids', 'support', 'attempts', 'failures', 'no_progress', 'timeouts'):
                getattr(group, key).update(getattr(other, key))
            group.restrictions.extend(other.restrictions)
            self.groups.remove(other)
        group.ids.update(ids)
        group.support.update(cells)
        return group

    def _token(self):
        self.serial += 1
        return self.serial

    def _neighborhood(self, state, frontier):
        grid = state.occupancy_map
        p = frontier.approach_pose
        radius = self.config.progress_radius_m
        return tuple((r, c) for r in range(max(0, floor((p.y_m-radius-grid.origin_y_m)/grid.resolution_m)),
                                          min(grid.height_cells, floor((p.y_m+radius-grid.origin_y_m)/grid.resolution_m)+1))
                     for c in range(max(0, floor((p.x_m-radius-grid.origin_x_m)/grid.resolution_m)),
                                    min(grid.width_cells, floor((p.x_m+radius-grid.origin_x_m)/grid.resolution_m)+1))
                     if (grid.origin_x_m+(c+.5)*grid.resolution_m-p.x_m)**2 +
                        (grid.origin_y_m+(r+.5)*grid.resolution_m-p.y_m)**2 <= radius**2)

    def _signature(self, state, frontier):
        grid = state.occupancy_map
        start = (floor((state.robot_pose.y_m-grid.origin_y_m)/grid.resolution_m),
                 floor((state.robot_pose.x_m-grid.origin_x_m)/grid.resolution_m))
        reached, pending = set(), [start]
        while pending:
            r, c = pending.pop()
            if (r, c) in reached or not (0 <= r < grid.height_cells and 0 <= c < grid.width_cells):
                continue
            if grid.data[r][c] != 0:
                continue
            reached.add((r, c))
            pending.extend(((r-1, c), (r+1, c), (r, c-1), (r, c+1)))
        local = tuple((r, c, grid.data[r][c]) for r, c in self._neighborhood(state, frontier))
        return frozenset(reached), local, frontier.approach_pose

    def _restrict(self, group, state, frontier, reason):
        condition = ('evidence_change' if reason in ('no_path', 'invalid_goal') else
                     'cooldown_or_change' if reason == 'no_progress' else 'cooldown')
        group.restrictions.append((reason, condition, state.timestamp_s+self.config.cooldown_s,
                                   self._signature(state, frontier)))

    def record_dispatch(self, state, frontier_id):
        group = self._group(next(f for f in state.frontiers if f.frontier_id == frontier_id))
        group.attempts.add(self._token())

    def record_attempt(self, state_at_dispatch: WorldState, attempt: PlannerAttempt) -> None:
        if attempt.execution_status != 'failed':
            return
        frontier = next(f for f in state_at_dispatch.frontiers if f.frontier_id == attempt.subgoal.frontier_id)
        group = self._group(frontier)
        group.failures.add(self._token())
        self._restrict(group, state_at_dispatch, frontier, attempt.failure_reason.value)

    def record_progress(self, before: WorldState, after: WorldState, frontier_id: str) -> float:
        frontier = next(f for f in before.frontiers if f.frontier_id == frontier_id)
        cells = self._neighborhood(before, frontier)  # frozen at dispatch
        progress = sum(before.occupancy_map.data[r][c] == -1 and
                       after.occupancy_map.data[r][c] != -1 for r, c in cells)
        area = progress * before.occupancy_map.resolution_m**2
        if area < self.config.minimum_progress_m2:
            group = self._group(frontier)
            group.no_progress.add(self._token())
            self._restrict(group, after, frontier, 'no_progress')
        return area

    def record_observation_failure(self, before, after, frontier_id):
        frontier = next(f for f in before.frontiers if f.frontier_id == frontier_id)
        group = self._group(frontier)
        group.failures.add(self._token())
        self._restrict(group, after, frontier, 'blocked')

    def record_evaluation(self, state, evaluation):
        by_id = {a.frontier_id: a for a in evaluation.assessments}
        for frontier in state.frontiers:
            assessment = by_id[frontier.frontier_id]
            group = self._group(frontier)
            if assessment.reachable is None:
                group.timeouts.add((state.revision, frontier.frontier_id))
                self._restrict(group, state, frontier, 'timeout')
            elif assessment.reachable is False:
                # Assessment is not an execution attempt. Geometry must change.
                self._restrict(group, state, frontier, assessment.failure_reason.value)

    def exclusions(self, state: WorldState, evaluation: CandidateEvaluation) -> Tuple[FrontierExclusion, ...]:
        require(state.episode_id == self.episode_id, 'recovery episode mismatch')
        # Synchronize every merge before projecting shared counters onto children.
        for frontier in state.frontiers:
            self._group(frontier)
        result = []
        for frontier in state.frontiers:
            group = self._group(frontier)
            exhausted = (len(group.attempts) >= self.config.opportunity_attempt_cap or
                         len(group.failures) >= self.config.navigation_retry_cap or
                         len(group.no_progress) >= self.config.no_progress_cap or
                         len(group.timeouts) >= self.config.evaluation_retry_cap)
            active = []
            signature = self._signature(state, frontier) if group.restrictions else None
            for reason, condition, until, old in group.restrictions:
                changed = signature != old
                unlocked = ((condition in ('cooldown', 'cooldown_or_change') and state.timestamp_s >= until)
                            or (condition in ('evidence_change', 'cooldown_or_change') and changed))
                if not unlocked:
                    active.append((reason, condition, until, old))
            # Do not discard shared restrictions when only one split child unlocks.
            if exhausted or active:
                reason, condition, until, _ = active[-1] if active else ('no_progress', 'exhausted', None, None)
                if exhausted:
                    condition, until = 'exhausted', None
                elif any(a[1] == 'evidence_change' for a in active):
                    condition, until = 'evidence_change', None
                else:
                    until = max(a[2] for a in active)
                result.append(FrontierExclusion(frontier.frontier_id, reason, len(group.attempts),
                                               condition, until if 'cooldown' in condition else None))
        return tuple(result)
