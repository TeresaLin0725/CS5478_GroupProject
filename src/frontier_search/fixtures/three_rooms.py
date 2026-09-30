"""Authored A/B fixtures for C integration, not a sensor or robot implementation.

Grid row increases with world y. Scripts reveal a room after arrival; hidden
scoring records are constructed separately and never sent to the search policy.
"""
from collections import deque
from dataclasses import replace
from math import hypot, pi
from typing import Optional, Tuple

from ..contracts import (
    CandidateAssessment, CandidateEvaluation, DetectionEvidence, EvaluationResult,
    EvidenceBundle, ExploredGrid, FailureReason, Frontier, GridCell, ObjectTrack,
    OccupancyGrid, PlannerResult, Point2, Point3, Pose, RegionSummary, TargetClaim,
    WorldState,
)
from ..validation import validate_state


GRID = ('#############', '#...........#', '#...#...#...#', '#...#...#...#', '#############')
CLUES = (('sofa', 'television'), ('bed', 'wardrobe'), ('sink', 'refrigerator'))


def initial_state(episode_id: str) -> WorldState:
    cells = tuple(tuple(100 if char == '#' else 0 if row == 1 else -1
                        for char in line) for row, line in enumerate(GRID))
    grid = OccupancyGrid('world', 1, 1.0, 0.0, 0.0, 13, 5, cells)
    explored = ExploredGrid('world', 1, 1.0, 0.0, 0.0, 13, 5,
        tuple(tuple(value != -1 for value in row) for row in cells))
    pose = Pose(1.5, 1.5, 0.0, 0.0)
    regions = tuple(RegionSummary('R' + str(i + 1),
        (Point2(x, 1.0), Point2(x + 3, 1.0), Point2(x + 3, 4.0), Point2(x, 4.0)),
        'igibson_room_prior', 1.0 / 3.0, CLUES[i], None, None)
        for i, x in enumerate((1.0, 5.0, 9.0)))
    # Authored localized semantic glimpses; no semantic ground-truth room labels.
    objects = tuple(ObjectTrack('clue-%d-%d' % (i, j), label, 0.9,
        Point3(1.5 + 4 * i + j, 1.5, 0.8), 1.0, 1.0, ('clue-obs-%d-%d' % (i, j),), 1)
        for i, classes in enumerate(CLUES) for j, label in enumerate(classes))
    frontiers = tuple(Frontier('F' + str(i + 1), (), (GridCell(1, col),),
        Pose(col + 0.5, 1.5, 0.0, pi / 2), 'R' + str(i + 1), 6.0,
        float(col - 1)) for i, col in enumerate((2, 6, 10)))
    state = WorldState('1.0', episode_id, 1, 1.0, 1, 'mug', pose,
                       grid, explored, objects, regions, frontiers, None)
    validate_state(state)
    return state


def plan_grid_path(state: WorldState, goal: Pose) -> PlannerResult:
    """Mock B: point robot, 4-connected, known-free cells; no footprint claim."""
    grid = state.occupancy_map
    start = (int(state.robot_pose.y_m), int(state.robot_pose.x_m))
    end = (int(goal.y_m), int(goal.x_m))
    def free(cell):
        row, col = cell
        return 0 <= row < grid.height_cells and 0 <= col < grid.width_cells and grid.data[row][col] == 0
    if not free(start) or not free(end):
        return PlannerResult(False, None, None, FailureReason.INVALID_GOAL)
    queue, parent = deque([start]), {start: None}
    while queue:
        cell = queue.popleft()
        if cell == end:
            break
        for dr, dc in ((0, 1), (0, -1), (1, 0), (-1, 0)):
            other = (cell[0] + dr, cell[1] + dc)
            if free(other) and other not in parent:
                parent[other] = cell
                queue.append(other)
    if end not in parent:
        return PlannerResult(False, None, None, FailureReason.NO_PATH)
    cells, cell = [], end
    while cell is not None:
        cells.append(cell)
        cell = parent[cell]
    cells.reverse()
    path = [Pose(col + 0.5, row + 0.5, 0.0, 0.0) for row, col in cells]
    path[0] = state.robot_pose
    if len(path) == 1:
        path.append(goal)  # orientation-only arrival
    else:
        path[-1] = goal
    cost = sum(hypot(b.x_m - a.x_m, b.y_m - a.y_m) for a, b in zip(path, path[1:]))
    return PlannerResult(True, tuple(path), cost, None)


def assess(state: WorldState) -> CandidateEvaluation:
    values = []
    for frontier in state.frontiers:
        plan = plan_grid_path(state, frontier.approach_pose)
        values.append(CandidateAssessment(frontier.frontier_id, plan.success,
                                           plan.path_cost_m, plan.failure_reason))
    return CandidateEvaluation(state.episode_id, state.revision, state.robot_pose, tuple(values))


def arrival_scan(state: WorldState, frontier_id: str) -> Tuple[WorldState, Optional[EvidenceBundle]]:
    """Mock A: scripted fresh observations; not full RGB-D mapper/scan lifecycle."""
    frontier = next(f for f in state.frontiers if f.frontier_id == frontier_id)
    region_index = int(frontier.region_id[1:]) - 1
    cells = [list(row) for row in state.occupancy_map.data]
    for row in (2, 3):
        for col in range(1 + 4 * region_index, 4 + 4 * region_index):
            cells[row][col] = 0
    revision = state.revision + 1
    stamp, sequence = state.timestamp_s + 1.0, state.last_processed_sequence + 2
    grid = replace(state.occupancy_map, revision=revision, data=tuple(tuple(row) for row in cells))
    explored = replace(state.explored_map, revision=revision,
        data=tuple(tuple(value != -1 for value in row) for row in cells))
    regions = tuple(replace(r, explored_fraction=1.0) if r.region_id == frontier.region_id else r
                    for r in state.regions)
    remaining = tuple(replace(f, distance_from_robot_m=hypot(
        f.approach_pose.x_m - frontier.approach_pose.x_m,
        f.approach_pose.y_m - frontier.approach_pose.y_m))
        for f in state.frontiers if f.frontier_id != frontier_id)
    objects, claim, evidence = state.semantic_objects, None, None
    if frontier_id == 'F3':
        # Authored detector output at this scan, not a hidden-target query by C.
        ids = ('mug-observation-%d' % (sequence - 1), 'mug-observation-%d' % sequence)
        location = Point3(10.5, 2.5, 0.8)
        objects += (ObjectTrack('mug-track', 'mug', 0.95, location,
                                stamp - 0.1, stamp, ids, 2),)
        claim = TargetClaim('claim-1', 'mug-track', 'mug', stamp, ids)
        detections = tuple(DetectionEvidence(oid, seq, t, 'mug-track', 'mug', 0.95,
            (10, 10, 30, 30), location) for oid, seq, t in
            zip(ids, (sequence - 1, sequence), (stamp - 0.1, stamp)))
        evidence = EvidenceBundle(state.episode_id, claim.claim_id, detections)
    updated = replace(state, revision=revision, timestamp_s=stamp,
        last_processed_sequence=sequence, robot_pose=frontier.approach_pose,
        occupancy_map=grid, explored_map=explored, regions=regions,
        semantic_objects=objects, frontiers=remaining, target_claim=claim)
    validate_state(updated)
    return updated, evidence


class FixtureEvaluator:
    """Isolated synthetic scoring; never substitutes for real visibility matching."""
    def __init__(self, target_present: bool = True) -> None:
        self._target_present = target_present

    def evaluate(self, state: WorldState, evidence: EvidenceBundle) -> EvaluationResult:
        claim = state.target_claim
        if claim is None or evidence is None:
            raise ValueError('missing terminal claim/evidence')
        valid = (self._target_present and evidence.episode_id == state.episode_id
            and evidence.claim_id == claim.claim_id
            and len({d.capture_sequence for d in evidence.detections}) >= 2
            and {d.observation_id for d in evidence.detections} == set(claim.evidence_observation_ids)
            and all(d.class_name == claim.target_class and d.track_id == claim.track_id
                    and d.position_world == Point3(10.5, 2.5, 0.8)
                    for d in evidence.detections))
        return EvaluationResult(state.episode_id, claim.claim_id, bool(valid),
            'synthetic-target' if valid else None, 'authored fixture scoring only')
