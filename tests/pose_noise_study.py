"""B-side pose-noise study on the mock simulator (helper module; the assertions live in test_pose_noise.py).

Where a localisation error reaches B (the robot's true pose is never changed, only what B is told):
  E1 stale-pose gate     B compares the pose in the request with the live pose and refuses a dispatch above
                         ``pose_freshness_m`` (5 cm by default).
  E2 evaluation          B is given a noisy start pose; does the reachability verdict flip, or the start become unusable?
  E3 map registration    a pose error while mapping shifts walls in the occupancy map; B plans on the shifted map while
                         the simulator collides against the true one.

Robotics background: a pose error of e metres moves every obstacle B sees by about e (the map is built in the estimated
frame), so safe passage needs clearance greater than the error. Planning clearance is footprint radius plus margin
(configuration-space inflation, Lozano-Perez 1983); the margin is B's only buffer for localisation error.
Run: ``PYTHONPATH=src python -m tests.pose_noise_study``.
"""
from dataclasses import replace
from math import hypot
from random import Random

from frontier_search.config import NavigationConfig, ObservationConfig
from frontier_search.contracts import (CandidateAssessment, CandidateEvaluationRequest, NavigationRequest,
                                       NavigationStatus, NavigatorInit, OccupancyGrid, PolicyName, Pose, Subgoal)
from frontier_search.fixtures.loader import load_policy_fixture
from frontier_search.navigation.locobot_robot import LOCOBOT_ROUND_ROBOT as ROBOT, ROUND_FOOTPRINT_NAV_OPTIONS
from frontier_search.navigation.navigator import TeamNavigator
from frontier_search.simulation.mock_adapter import MockSimulator
from tests.unit.test_planning_radius import DONE, world

START, GOAL = Pose(0.9, 2.0, 0, 0), Pose(4.5, 2.0, 0, 0)


def navigator(grid_true, start=START):
    sim = MockSimulator(grid_true, ROBOT)
    sim.reset('x', 'm', start, 0)
    nav = TeamNavigator(sim, **ROUND_FOOTPRINT_NAV_OPTIONS)
    nav.reset(NavigatorInit('x', ROBOT, ObservationConfig(), NavigationConfig(navigation_deadline_s=600)))
    return sim, nav


def shifted(grid, rows=0, columns=0):
    """The same map registered with a pose error: every cell moves by (rows, columns); new cells are unknown."""
    data = [[-1] * grid.width_cells for _ in range(grid.height_cells)]
    for r in range(grid.height_cells):
        for c in range(grid.width_cells):
            nr, nc = r + rows, c + columns
            if 0 <= nr < grid.height_cells and 0 <= nc < grid.width_cells:
                data[nr][nc] = grid.data[r][c]
    return replace(grid, data=tuple(map(tuple, data)))


def stale_pose_gate(offset_m):
    """E1: status of a dispatch whose request pose is ``offset_m`` away from the live pose."""
    grid, _ = world(10)
    sim, nav = navigator(grid)
    try:
        told = Pose(START.x_m + offset_m, START.y_m, 0, 0)
        feedback = nav.start(NavigationRequest('x', 'g', grid, told, Subgoal('F1', GOAL, 1, PolicyName.NEAREST)))
        return feedback.status, feedback.failure_reason
    finally:
        sim.close()


def evaluation_under_noise(sigma_xy_m, sigma_yaw_rad, trials=40, seed=7):
    """E2: noisy start pose given to ``evaluate_candidates`` (start 0.4 m from the west wall of the room)."""
    grid, _ = world(10)
    state = load_policy_fixture('selection').state
    frontier = replace(state.frontiers[0], approach_pose=GOAL)
    base = replace(state, episode_id='x', occupancy_map=grid, robot_pose=START, frontiers=(frontier,))
    sim, nav = navigator(grid)
    rng = Random(seed)
    try:
        truth = nav.evaluate_candidates(CandidateEvaluationRequest('x', base)).assessments[0]
        flips = unusable = 0
        errors = []
        for _ in range(trials):
            noisy = Pose(START.x_m + rng.gauss(0, sigma_xy_m), START.y_m + rng.gauss(0, sigma_xy_m), 0,
                         rng.gauss(0, sigma_yaw_rad))
            result = nav.evaluate_candidates(
                CandidateEvaluationRequest('x', replace(base, robot_pose=noisy))).assessments[0]
            flips += result.reachable != truth.reachable
            unusable += result.reachable is False
            if result.reachable and truth.reachable:
                errors.append(abs(result.path_cost_m - truth.path_cost_m))
        return dict(sigma_xy_m=sigma_xy_m, flip_rate=flips / trials, unusable_rate=unusable / trials,
                    mean_cost_error_m=sum(errors) / len(errors) if errors else None, truth=truth.reachable)
    finally:
        sim.close()


GAP_GOAL = Pose(4.5, 1.3, 0, 0)


def offset_gap_world(gap_cells, low=10):
    """Room with a wall at column 30 and a gap that is NOT on the straight line from START (rows low..low+gap)."""
    grid, _ = world(0)
    cells = [list(row) for row in grid.data]
    for r in range(low, low + gap_cells):
        cells[r][30] = 0
    return replace(grid, data=tuple(map(tuple, cells)))


def map_registration_error(shift_rows, gap_cells=6):
    """E3: drive through an off-line wall gap (0.6 m by default) while B's map is shifted ``shift_rows`` cells
    (0.1 m each, along the wall) from the truth. The robot is 0.38 m wide, so about 0.1 m of slack remains per side."""
    grid = offset_gap_world(gap_cells)
    sim, nav = navigator(grid)
    try:
        planned = shifted(grid, rows=shift_rows)
        feedback = nav.start(NavigationRequest('x', 'g', planned, START,
                                               Subgoal('F1', GAP_GOAL, 1, PolicyName.NEAREST)))
        for _ in range(14000):
            if feedback.status in DONE:
                break
            sim.step(); nav.on_step(); feedback = nav.poll('x', 'g')
        return dict(shift_m=shift_rows * grid.resolution_m, status=feedback.status.value,
                    reason=feedback.failure_reason.value if feedback.failure_reason else None,
                    collisions=sim.collisions)
    finally:
        sim.close()


def main():
    print('E1 stale-pose gate (request pose vs live pose; limit %.2f m)' % NavigationConfig().pose_freshness_m)
    for offset in (0.0, 0.03, 0.05, 0.06, 0.10, 0.20):
        print('  offset %.2f m -> %s' % (offset, stale_pose_gate(offset)))
    print('E2 evaluation with a noisy start pose')
    for sigma in (0.0, 0.02, 0.05, 0.10, 0.20, 0.30):
        print('  ', evaluation_under_noise(sigma, sigma))
    print('E3 map registration error (0.6 m gap, off the straight line)')
    for rows in (0, 1, 2, 3, 4, 5):
        print('  ', map_registration_error(rows))


if __name__ == '__main__':
    main()
