"""P1: a rectangular robot must not be sent into a turn that sweeps its corners into a wall.

Planning uses the circumscribed disc, which is safe at cell centres. The robot's real start pose is
not at a cell centre and its cell may be inflation-blocked, so the first turn needs an exact check.
Evaluation (distance field) and execution (A*) must agree on which first moves are allowed.
"""

from math import atan2, hypot
from random import Random
import unittest

from frontier_search.config import NavigationConfig, ObservationConfig
from frontier_search.contracts import (FailureReason, NavigationRequest, NavigationStatus,
                                       NavigatorInit, OccupancyGrid, PolicyName, Pose, Subgoal)
from frontier_search.navigation.costmap import Costmap
from frontier_search.navigation.dev_robot import PLACEHOLDER_ROBOT
from frontier_search.navigation.locobot_robot import LOCOBOT_DEV_ROBOT
from frontier_search.navigation.navigator import TeamNavigator
from frontier_search.navigation.planner import distance_field, plan
from frontier_search.simulation.mock_adapter import MockSimulator

DONE = (NavigationStatus.REACHED, NavigationStatus.FAILED, NavigationStatus.CANCELLED)
WALL_EDGE_Y = 0.5          # free room is rows/columns 5..34 of a 0.1 m grid; unknown outside


def room(blocked=()):
    cells = [[-1] * 40 for _ in range(40)]
    for r in range(5, 35):
        for c in range(5, 35):
            cells[r][c] = 0
    for r, c in blocked:
        cells[r][c] = 100
    return OccupancyGrid('w', 1, .1, 0, 0, 40, 40, tuple(map(tuple, cells)))


class Harness:
    def __init__(self, test, robot, start, deadline=300.0, blocked=()):
        self.grid, self.robot = room(blocked), robot
        self.sim = MockSimulator(self.grid, robot)
        test.addCleanup(self.sim.close)
        self.sim.reset('x', 'm', start, 0)
        self.nav = TeamNavigator(self.sim)
        self.nav.reset(NavigatorInit('x', robot, ObservationConfig(), NavigationConfig(navigation_deadline_s=deadline)))

    def go(self, start, goal, steps=8000):
        feedback = self.nav.start(NavigationRequest('x', 'g', self.grid, start,
                                                    Subgoal('F1', goal, 1, PolicyName.NEAREST)))
        first = feedback
        for _ in range(steps):
            if feedback.status in DONE:
                break
            self.sim.step(); self.nav.on_step(); feedback = self.nav.poll('x', 'g')
        return first, feedback


class StartTurnSafetyTests(unittest.TestCase):
    def test_turn_that_sweeps_into_the_wall_is_refused_up_front_not_driven_into(self):
        # 0.21 m from the wall, facing along it, on a cell-centre column: the only traversable first move
        # is straight up at exactly 90 degrees, whose END pose is legal; only the SWEEP to it is not.
        start = Pose(1.55, WALL_EDGE_Y + 0.21, 0.0, 0.0)
        goal = Pose(1.55, 2.5, 0.0, 0.0)
        h = Harness(self, LOCOBOT_DEV_ROBOT, start)
        costmap = Costmap.from_robot(h.grid, LOCOBOT_DEV_ROBOT)
        self.assertTrue(costmap.pose_valid(start))             # the start itself is legal...
        self.assertFalse(costmap.rotation_valid(start, 1.5707963))   # ...but the turn is not
        first, last = h.go(start, goal)
        self.assertEqual(first.status, NavigationStatus.FAILED)
        self.assertEqual(first.failure_reason, FailureReason.NO_PATH)
        self.assertEqual(h.sim.collisions, 0)
        self.assertEqual(h.sim.executed_distance_m, 0.0)

    def test_evaluation_and_execution_agree_on_which_first_moves_exist(self):
        start = Pose(1.55, WALL_EDGE_Y + 0.21, 0.0, 0.0)
        costmap = Costmap.from_robot(room(), LOCOBOT_DEV_ROBOT)
        goal = Pose(1.55, 2.5, 0.0, 0.0)
        self.assertIsNone(distance_field(costmap, start)[0].get((24, 15)))   # goal cell not in the field
        self.assertFalse(plan(costmap, start, goal).success)

    def test_circular_robot_behaviour_is_unchanged(self):
        start, goal = Pose(1.05, 1.05, 0.0, 0.0), Pose(3.05, 2.05, 0.0, 0.0)
        h = Harness(self, PLACEHOLDER_ROBOT, start)
        first, last = h.go(start, goal)
        self.assertEqual(last.status, NavigationStatus.REACHED)
        self.assertEqual(h.sim.collisions, 0)

    def test_property_planner_field_and_execution_agree_and_never_collide(self):
        rng, checked, accepted = Random(7), 0, 0
        for _ in range(40):
            start = Pose(rng.uniform(1.0, 3.0), WALL_EDGE_Y + rng.uniform(0.19, 0.45), 0.0,
                         rng.uniform(-3.1, 3.1))
            goal = Pose(rng.uniform(1.0, 3.0), rng.uniform(1.0, 3.0), 0.0, rng.uniform(-3.1, 3.1))
            costmap = Costmap.from_robot(room(), LOCOBOT_DEV_ROBOT)
            if not costmap.pose_valid(start):
                continue
            field = distance_field(costmap, start)
            if field is None:
                continue
            from frontier_search.navigation.planner import cost_to_goal_m
            in_field = cost_to_goal_m(costmap, start, goal, field[0]) is not None
            planned = plan(costmap, start, goal)
            checked += 1
            with self.subTest(start=start, goal=goal):
                self.assertEqual(in_field, planned.success)    # evaluation agrees with execution planning
                if planned.success:
                    accepted += 1
                    h = Harness(self, LOCOBOT_DEV_ROBOT, start)
                    first, last = h.go(start, goal)
                    self.assertEqual(h.sim.collisions, 0)      # an accepted plan never drives into a wall
        self.assertGreater(checked, 20)
        self.assertGreater(accepted, 5)

    def test_runtime_guard_stops_an_in_place_turn_that_would_sweep_into_the_wall(self):
        # Start in open space so the plan is accepted, then put the robot where the turn is unsafe.
        start, goal = Pose(2.0, 2.0, 0.0, 0.0), Pose(2.0, 3.0, 0.0, 0.0)
        h = Harness(self, LOCOBOT_DEV_ROBOT, start)
        first, _ = h.go(start, goal, steps=0)
        self.assertEqual(first.status, NavigationStatus.ACCEPTED)
        h.sim._pose = Pose(1.5, WALL_EDGE_Y + 0.21, 0.0, 0.0)   # test-only teleport next to the wall
        h.sim.step(); h.nav.on_step()
        feedback = h.nav.poll('x', 'g')
        self.assertEqual((feedback.status, feedback.failure_reason),
                         (NavigationStatus.FAILED, FailureReason.BLOCKED))
        self.assertEqual(h.sim.collisions, 0)


    def test_first_move_check_covers_both_the_turn_and_the_straight_drive(self):
        costmap = Costmap.from_robot(room(blocked=[(15, 20)]), LOCOBOT_DEV_ROBOT)   # a pillar at x 2.0-2.1
        start = Pose(1.5, 1.55, 0.0, 0.0)
        self.assertTrue(costmap.first_move_valid(start, 1.7, 1.55))      # clear, no turn needed
        self.assertFalse(costmap.first_move_valid(start, 2.5, 1.55))     # no turn needed, but the drive hits the pillar
        wall_side = Pose(1.55, WALL_EDGE_Y + 0.21, 0.0, 0.0)
        self.assertFalse(Costmap.from_robot(room(), LOCOBOT_DEV_ROBOT).first_move_valid(wall_side, 1.55, 0.85))  # sweep

    def test_runtime_guard_blocks_the_final_yaw_alignment_that_would_sweep_into_a_pillar(self):
        # A legal goal pose (cell traversable, pose valid at both yaws) whose in-place alignment sweeps the pillar.
        goal = Pose(2.2349, 1.6635, 0.0, 0.0)
        start = Pose(goal.x_m, goal.y_m, 0.0, 1.5707963)
        pillar = [(19, 20)]
        costmap = Costmap.from_robot(room(pillar), LOCOBOT_DEV_ROBOT)
        self.assertTrue(costmap.pose_valid(start) and costmap.pose_valid(goal))
        self.assertFalse(costmap.rotation_valid(start, goal.yaw_rad))
        h = Harness(self, LOCOBOT_DEV_ROBOT, start, blocked=pillar)
        first, last = h.go(start, goal)
        self.assertEqual(first.status, NavigationStatus.ACCEPTED)
        self.assertEqual((last.status, last.failure_reason), (NavigationStatus.FAILED, FailureReason.BLOCKED))
        self.assertEqual(h.sim.collisions, 0)                            # it refused to turn instead of hitting it


if __name__ == '__main__':
    unittest.main()
