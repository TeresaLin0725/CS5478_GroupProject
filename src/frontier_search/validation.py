"""Boundary checks for scaffold data. Temporal/lineage checks belong to owners."""
from typing import Set, Tuple


from .contracts import (
    CandidateEvaluation, FailureReason, SelectionContext, SelectionResult, WorldState,
)
from .immutable import require_immutable


class ContractError(ValueError):
    """A module supplied an inconsistent request or snapshot."""


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ContractError(message)


def _unique(values: Tuple[str, ...], label: str) -> Set[str]:
    require(all(values), f"{label}: empty ID")
    require(len(set(values)) == len(values), f"{label}: duplicate ID")
    return set(values)


def validate_state(state: WorldState, *, oracle_room_labels: bool = False) -> None:
    require_immutable(state)
    require(state.schema_version.split('.')[0] == '1', "unsupported contract major version")
    require(bool(state.episode_id and state.target_class), "missing episode or target")
    require(state.revision >= 0 and state.last_processed_sequence >= 0, "negative revision/sequence")
    require(state.timestamp_s >= 0, "negative simulation timestamp")
    grids = (state.occupancy_map, state.explored_map)
    geometry = lambda g: (g.frame_id, g.resolution_m, g.origin_x_m, g.origin_y_m,
                          g.width_cells, g.height_cells)
    require(geometry(grids[0]) == geometry(grids[1]), "grid geometry mismatch")
    for grid in grids:
        require(grid.frame_id == 'world', "grid must use world coordinates")
        require(grid.revision == state.revision, "map/state revision mismatch")
        require(grid.resolution_m > 0, "resolution must be positive")
        require(grid.width_cells > 0 and grid.height_cells > 0, "empty grid geometry")
        require(len(grid.data) == grid.height_cells, "grid height mismatch")
        require(all(len(row) == grid.width_cells for row in grid.data), "grid width mismatch")
    require(all(type(v) is int and v in (-1, 0, 100) for row in grids[0].data for v in row),
            "invalid occupancy encoding")
    require(all(type(v) is bool for row in grids[1].data for v in row), "coverage must be boolean")
    region_ids = _unique(tuple(r.region_id for r in state.regions), 'regions')
    for region in state.regions:
        require(len(region.boundary_world) >= 3, "region requires a polygon")
        require(region.boundary_source == 'igibson_room_prior', "unknown boundary source")
        require(region.explored_fraction is None or 0 <= region.explored_fraction <= 1,
                "coverage fraction outside [0, 1]")
        if not oracle_room_labels:
            require(region.room_type is None and region.room_type_source is None,
                    "standard policy state contains oracle room labels")
        elif region.room_type is not None:
            require(region.room_type_source == 'simulator_annotation', "missing oracle source")
    track_ids = _unique(tuple(o.track_id for o in state.semantic_objects), 'tracks')
    for obj in state.semantic_objects:
        require(0 <= obj.confidence <= 1, "confidence outside [0, 1]")
        require(0 <= obj.first_seen_s <= obj.last_seen_s <= state.timestamp_s, "invalid track time")
        require(0 <= obj.confirmation_count <= len(set(obj.observation_ids)), "invalid confirmation count")
    _unique(tuple(f.frontier_id for f in state.frontiers), 'frontiers')
    for frontier in state.frontiers:
        require(frontier.region_id is None or frontier.region_id in region_ids, "unknown region")
        require(frontier.frontier_id not in frontier.parent_frontier_ids, "self-parent frontier")
        require(frontier.information_gain_m2 >= 0 and frontier.distance_from_robot_m >= 0,
                "negative frontier gain/distance")
        require(bool(frontier.support_cells), "frontier has no spatial support")
        for cell in frontier.support_cells:
            require(0 <= cell.row < grids[0].height_cells and 0 <= cell.column < grids[0].width_cells,
                    "frontier support out of bounds")
            require(grids[0].data[cell.row][cell.column] == 0, "frontier support must be known free")
    if state.target_claim is not None:
        claim = state.target_claim
        require(claim.track_id in track_ids, "claim references unknown track")
        require(claim.target_class == state.target_class, "claim target mismatch")
        require(len(set(claim.evidence_observation_ids)) >= 2, "claim needs multiple observations")


def validate_evaluation(state: WorldState, evaluation: CandidateEvaluation) -> None:
    require(evaluation.episode_id == state.episode_id, "assessment episode mismatch")
    require(evaluation.state_revision == state.revision, "stale assessment")
    require(evaluation.start_pose == state.robot_pose, "assessment pose mismatch")
    ids = _unique(tuple(a.frontier_id for a in evaluation.assessments), 'assessments')
    require(ids == {f.frontier_id for f in state.frontiers}, "assessment frontier set mismatch")
    for assessment in evaluation.assessments:
        if assessment.reachable is True:
            require(assessment.path_cost_m is not None and assessment.path_cost_m >= 0
                    and assessment.failure_reason is None, "invalid reachable assessment")
        elif assessment.reachable is False:
            require(assessment.path_cost_m is None and assessment.failure_reason in
                    (FailureReason.NO_PATH, FailureReason.INVALID_GOAL), "invalid unreachable assessment")
        else:
            require(assessment.reachable is None and assessment.path_cost_m is None
                    and assessment.failure_reason == FailureReason.TIMEOUT, "invalid inconclusive assessment")


def validate_context(state: WorldState, context: SelectionContext) -> None:
    require(context.episode_id == state.episode_id, "context episode mismatch")
    validate_evaluation(state, context.evaluation)
    eligible = _unique(context.eligible_frontier_ids, 'eligible candidates')
    excluded = _unique(tuple(e.frontier_id for e in context.exclusions), 'exclusions')
    require(excluded <= {f.frontier_id for f in state.frontiers}, "unknown excluded candidate")
    reachable = {a.frontier_id for a in context.evaluation.assessments if a.reachable is True}
    require(eligible == reachable - excluded, "eligible IDs must equal reachable minus exclusions")


def validate_selection(state: WorldState, context: SelectionContext, result: SelectionResult) -> None:
    validate_context(state, context)
    require(result.episode_id == state.episode_id, "selection episode mismatch")
    require((result.subgoal is None) != (result.stop_reason is None), "select a goal or a stop reason")
    expected_stop = ('target_claimed' if state.target_claim else 'no_frontiers' if not state.frontiers
                     else 'no_eligible_frontiers' if not context.eligible_frontier_ids else None)
    require(result.stop_reason == expected_stop, "invalid stop reason or ignored terminal claim")
    if result.subgoal is not None:
        goal = result.subgoal
        require(goal.frontier_id in context.eligible_frontier_ids, "ineligible subgoal")
        frontier = next(f for f in state.frontiers if f.frontier_id == goal.frontier_id)
        require(goal.approach_pose == frontier.approach_pose, "invented approach pose")
        require(goal.selected_from_revision == state.revision, "stale selection")
