"""Authored development data, never a source of main-experiment observations."""


from dataclasses import dataclass, replace
from math import hypot
from typing import Optional, Tuple

from ..contracts import (
    CandidateAssessment, CandidateEvaluation, ExploredGrid, FailureReason, Frontier,
    FrontierExclusion, GridCell, ObjectTrack, OccupancyGrid, Point2, Point3, Pose,
    RegionSummary, SelectionContext, TargetClaim, WorldState,
)
from ..validation import validate_context, validate_state


@dataclass(frozen=True)
class FixtureMetadata:
    fixture_id: str
    source: str = 'synthetic'
    schema_version: str = '1.0'
    seed: int = 0
    preprocessing: Tuple[str, ...] = ('authored uninflated occupancy',)


@dataclass(frozen=True)
class PolicyFixture:
    metadata: FixtureMetadata  # Not passed to SearchPolicy.
    state: WorldState
    context: SelectionContext


@dataclass(frozen=True)
class NavigationFixture:
    metadata: FixtureMetadata
    grid: OccupancyGrid
    start_pose: Pose
    goal_pose: Pose
    updated_grid: Optional[OccupancyGrid] = None


def load_navigation_fixture(name: str = 'wall_detour') -> NavigationFixture:
    names = {'open_space', 'wall_detour', 'narrow_doorway', 'unknown_barrier', 'blocked_update'}
    if name not in names:
        raise ValueError(f'unknown navigation fixture {name!r}; choose from {sorted(names)}')
    cells = [[-1] * 40 for _ in range(40)]
    for row in range(5, 35):
        for col in range(5, 35):
            cells[row][col] = 0
    if name != 'open_space':
        for row in range(5, 35):
            cells[row][20] = -1 if name == 'unknown_barrier' else 100
        if name != 'unknown_barrier':
            for row in (range(20, 21) if name == 'narrow_doorway' else range(18, 23)):
                cells[row][20] = 0
    grid = OccupancyGrid('world', 1, 0.1, 0.0, 0.0, 40, 40,
                         tuple(tuple(row) for row in cells))
    updated = None
    if name == 'blocked_update':
        for row in range(18, 23):
            cells[row][20] = 100
        updated = replace(grid, revision=2, data=tuple(tuple(row) for row in cells))
    # Centres of cells (10, 10) and (10, 30); wall cases require a detour.
    return NavigationFixture(FixtureMetadata(name), grid,
                             Pose(1.05, 1.05, 0.0, 0.0), Pose(3.05, 1.05, 0.0, 0.0), updated)


def load_policy_fixture(name: str = 'selection') -> PolicyFixture:
    names = {'selection', 'no_frontiers', 'all_excluded', 'target_claim',
             'inconclusive', 'prior_failure'}
    if name not in names:
        raise ValueError(f'unknown policy fixture {name!r}; choose from {sorted(names)}')
    nav = load_navigation_fixture('open_space')
    grid = nav.grid
    coverage = ExploredGrid('world', 1, 0.1, 0.0, 0.0, 40, 40,
                            tuple(tuple(v != -1 for v in row) for row in grid.data))
    object_classes = (('sofa', 'TV'), ('sink', 'refrigerator'), ('bed',))
    regions = tuple(RegionSummary(
        f'R{i + 1}', (Point2(0.0, i), Point2(4.0, i), Point2(4.0, i + 1), Point2(0.0, i + 1)),
        'igibson_room_prior', (0.375, 0.75, 0.75)[i], classes, None, None,
    ) for i, classes in enumerate(object_classes))
    objects = tuple(ObjectTrack(f'O{i}-{j}', label, 0.9, None, 0.0, 1.0,
                                (f'obs-{i}-{j}',), 1)
                    for i, classes in enumerate(object_classes) for j, label in enumerate(classes))
    poses = (Pose(0.55, 0.75, 0.0, 3.141592653589793),
             Pose(0.55, 1.75, 0.0, 3.141592653589793),
             Pose(0.55, 2.75, 0.0, 3.141592653589793))
    frontiers = tuple(Frontier(f'F{i + 1}', (), (GridCell(7 + i * 10, 5),), pose,
                               f'R{i + 1}', (6.0, 20.0, 12.0)[i],
                               hypot(pose.x_m - nav.start_pose.x_m, pose.y_m - nav.start_pose.y_m))
                      for i, pose in enumerate(poses))
    assessments = (CandidateAssessment('F1', True, 2.0, None),
                   CandidateAssessment('F2', True, 5.0, None),
                   CandidateAssessment('F3', False, None, FailureReason.NO_PATH))
    exclusions = ()
    previous = None
    claim = None
    if name == 'no_frontiers':
        frontiers, assessments = (), ()
    if name == 'inconclusive':
        assessments = (CandidateAssessment('F1', None, None, FailureReason.TIMEOUT),) + assessments[1:]
    if name in ('all_excluded', 'prior_failure'):
        excluded_ids = ('F1', 'F2') if name == 'all_excluded' else ('F1',)
        exclusions = tuple(FrontierExclusion(fid, 'blocked', 1, 'cooldown', 6.0)
                           for fid in excluded_ids)
    if name == 'prior_failure':
        from ..contracts import PlannerAttempt, PlannerResult, PolicyName, Subgoal
        previous = PlannerAttempt(Subgoal('F1', poses[0], 1, PolicyName.NEAREST),
                                  PlannerResult(True, (nav.start_pose, poses[0]), 2.0, None),
                                  'failed', FailureReason.BLOCKED)
    if name == 'target_claim':
        objects += (ObjectTrack('cup-track', 'cup', 0.95, Point3(1.0, 1.0, 0.8),
                                0.0, 1.0, ('cup-obs-1', 'cup-obs-2'), 2),)
        claim = TargetClaim('claim-1', 'cup-track', 'cup', 1.0, ('cup-obs-1', 'cup-obs-2'))
    state = WorldState('1.0', 'fixture-episode', 1, 1.0, 2, 'cup', nav.start_pose,
                       grid, coverage, objects, regions, frontiers, claim)
    evaluation = CandidateEvaluation(state.episode_id, state.revision, state.robot_pose, assessments)
    excluded = {e.frontier_id for e in exclusions}
    eligible = tuple(a.frontier_id for a in assessments if a.reachable is True and a.frontier_id not in excluded)
    context = SelectionContext(state.episode_id, evaluation, eligible, exclusions, previous)
    validate_state(state)
    validate_context(state, context)
    return PolicyFixture(FixtureMetadata(name), state, context)
