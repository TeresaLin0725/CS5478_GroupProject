"""Orientation-aware B planner/controller: forward, reverse, and checked turns.

Each edge tests the rectangular footprint swept through the motion, with unknown
and out-of-bounds blocked. It never substitutes a small point/disc for the body.
The existing Navigator lifecycle, map feedback, cancellation and scan hooks remain.
"""
from heapq import heappop, heappush
from math import atan2, cos, hypot, pi, sin
from time import monotonic

import numpy as np

from ..contracts import (CandidateAssessment, CandidateEvaluation, FailureReason,
                         NavigationStatus, PlannerResult, Pose)
from .costmap import Costmap
from .geometry import cells_overlapping_footprint, world_to_cell
from .navigator import TeamNavigator, _clip, _wrap

DIRECTIONS = ((0, 1), (1, 1), (1, 0), (1, -1),
              (0, -1), (-1, -1), (-1, 0), (-1, 1))


class LatticeSearch:
    def __init__(self, grid, robot):
        self.grid, self.robot = grid, robot
        self.costmap = Costmap.from_robot(grid, robot)
        self.free = np.array(grid.data) == 0
        self.poses, self.turns, self.moves = [], [], []
        for h, (dr, dc) in enumerate(DIRECTIONS):
            yaw = h * pi / 4
            self.poses.append(self._mask(((0, 0, yaw),)))
            self.turns.append(self._mask(tuple((0, 0, yaw + i * pi / 32) for i in range(9))))
            self.moves.append(self._mask(tuple((dr * t, dc * t, yaw) for t in (0, .5, 1))))

    def _mask(self, samples):
        g, res = self.grid, self.grid.resolution_m
        offsets = set()
        for r, c, yaw in samples:
            pose = Pose(g.origin_x_m + (.5 + c) * res,
                        g.origin_y_m + (.5 + r) * res, 0, yaw)
            offsets.update(cells_overlapping_footprint(g, self.robot.footprint_base,
                                                       pose, self.robot.clearance_margin_m))
        valid = np.ones(self.free.shape, dtype=bool)
        height, width = valid.shape
        for dr, dc in offsets:
            shifted = np.zeros_like(valid)
            r0, r1 = max(0, -dr), min(height, height - dr)
            c0, c1 = max(0, -dc), min(width, width - dc)
            if r0 < r1 and c0 < c1:
                shifted[r0:r1, c0:c1] = self.free[r0 + dr:r1 + dr, c0 + dc:c1 + dc]
            valid &= shifted
        return valid

    def pose(self, node):
        r, c, h = node
        g = self.grid
        return Pose(g.origin_x_m + (c + .5) * g.resolution_m,
                    g.origin_y_m + (r + .5) * g.resolution_m, 0, _wrap(h * pi / 4))

    def key(self, pose):
        cell = world_to_cell(self.grid, pose.x_m, pose.y_m)
        return (cell.row, cell.column, int(round(pose.yaw_rad / (pi / 4))) % 8) if cell else None

    def solve(self, start, goals, deadline_s=8):
        """One bounded Dijkstra query for all goal poses, returning physical path lengths."""
        answers = {}
        desired = {}
        for ident, goal in goals.items():
            key = self.key(goal)
            if (key is None or not self.poses[key[2]][key[0], key[1]]
                    or not self.costmap.pose_valid(goal)
                    or not self.costmap.rotation_valid(self.pose(key), goal.yaw_rad)):
                answers[ident] = PlannerResult(False, None, None, FailureReason.INVALID_GOAL)
            else:
                desired.setdefault(key, []).append(ident)
        origin = self.key(start)
        connections = {}
        if origin is not None and self.costmap.pose_valid(start):
            # The continuous robot must not be snapped to a cell or forced to
            # rotate at its current location. Connect forward OR reverse to
            # nearby feasible lattice poses, including backing out of a tight
            # place before turning. Every connector has checked swept volume.
            r,c,h=origin
            nearby=sorted(((dr,dc) for dr in range(-6,7) for dc in range(-6,7)),
                          key=lambda rc:rc[0]*rc[0]+rc[1]*rc[1])
            for dr,dc in nearby:
                # Usually the nearest 2-cell ring suffices. If there is no
                # connection, extend to 6 cells: a non-lattice heading near a
                # chair may need to reverse farther before yaw can be quantized.
                if max(abs(dr),abs(dc))>2 and connections:continue
                nr,nc=r+dr,c+dc
                if not (0<=nr<self.free.shape[0] and 0<=nc<self.free.shape[1]): continue
                centre=self.pose((nr,nc,h))
                distance=hypot(centre.x_m-start.x_m,centre.y_m-start.y_m)
                bearing=atan2(centre.y_m-start.y_m,centre.x_m-start.x_m) if distance>1e-7 else start.yaw_rad
                headings=sorted((bearing,_wrap(bearing+pi)),key=lambda yaw:abs(_wrap(yaw-start.yaw_rad)))
                for heading in headings:
                    first=Pose(start.x_m,start.y_m,0,heading)
                    last=Pose(centre.x_m,centre.y_m,0,heading)
                    if not self.costmap.rotation_valid(start,heading) or not self.costmap.segment_valid(first,last): continue
                    for nh in range(8):
                        node=(nr,nc,nh); end=self.pose(node)
                        if self.poses[nh][nr,nc] and self.costmap.rotation_valid(last,end.yaw_rad):
                            prefix=(start,first,last,end)
                            cost=distance+.025*(abs(_wrap(heading-start.yaw_rad))+abs(_wrap(end.yaw_rad-heading)))/(pi/4)
                            if node not in connections or cost<connections[node][0]: connections[node]=(cost,prefix)
                    break
            # Continuous-pose escape primitive: reverse/advance at the ACTUAL
            # yaw first, then connect to a lattice centre. Requiring even a tiny
            # turn before that move can incorrectly strand a safe robot beside
            # a chair after a non-quantized final observation heading.
            for distance in (-.20,.20,-.40,.40,-.60,.60):
                escape=Pose(start.x_m+distance*cos(start.yaw_rad),
                            start.y_m+distance*sin(start.yaw_rad),0,start.yaw_rad)
                if not self.costmap.segment_valid(start,escape):continue
                key=self.key(escape)
                if key is None:continue
                er,ec,_=key
                for dr,dc in ((0,0),(0,1),(0,-1),(1,0),(-1,0)):
                    nr,nc=er+dr,ec+dc
                    if not (0<=nr<self.free.shape[0] and 0<=nc<self.free.shape[1]):continue
                    centre=self.pose((nr,nc,h))
                    length=hypot(centre.x_m-escape.x_m,centre.y_m-escape.y_m)
                    bearing=atan2(centre.y_m-escape.y_m,centre.x_m-escape.x_m) if length>1e-8 else start.yaw_rad
                    for heading in sorted((bearing,_wrap(bearing+pi)),key=lambda a:abs(_wrap(a-start.yaw_rad))):
                        first=Pose(escape.x_m,escape.y_m,0,heading)
                        last=Pose(centre.x_m,centre.y_m,0,heading)
                        if not self.costmap.rotation_valid(escape,heading) or not self.costmap.segment_valid(first,last):continue
                        for nh in range(8):
                            node=(nr,nc,nh);end=self.pose(node)
                            if self.poses[nh][nr,nc] and self.costmap.rotation_valid(last,end.yaw_rad):
                                cost=abs(distance)+length+.025*(abs(_wrap(heading-start.yaw_rad))+abs(_wrap(end.yaw_rad-heading)))/(pi/4)
                                if node not in connections or cost<connections[node][0]:
                                    connections[node]=(cost,(start,escape,first,last,end))
                        break
        if not connections:
            answers.update({ident: PlannerResult(False, None, None, FailureReason.NO_PATH)
                            for ids in desired.values() for ident in ids})
            return answers
        queue, parent, best = [],{},{}
        for node,(cost,prefix) in connections.items():
            heappush(queue,(cost,node)); parent[node]=None; best[node]=cost
        until = monotonic() + deadline_s
        timed_out = False
        while queue and desired:
            if monotonic() >= until:
                timed_out = True
                break
            cost, node = heappop(queue)
            if cost != best[node]:
                continue
            if node in desired:
                nodes, cursor = [], node
                while cursor is not None:
                    nodes.append(cursor); cursor = parent[cursor]
                root=nodes[-1]
                route = tuple(connections[root][1]) + tuple(self.pose(n) for n in reversed(nodes))
                for ident in desired.pop(node):
                    path = route + (goals[ident],)
                    distance = sum(hypot(b.x_m - a.x_m, b.y_m - a.y_m) for a, b in zip(path, path[1:]))
                    answers[ident] = PlannerResult(True, path, distance, None)
            r, c, h = node
            neighbors = []
            if self.turns[h][r, c]:
                neighbors.append(((r, c, (h + 1) % 8), .025))
            if self.turns[(h - 1) % 8][r, c]:
                neighbors.append(((r, c, (h - 1) % 8), .025))
            dr, dc = DIRECTIONS[h]
            for direction in (1, -1):
                nr, nc = r + direction * dr, c + direction * dc
                if not (0 <= nr < self.free.shape[0] and 0 <= nc < self.free.shape[1]):
                    continue
                mr, mc = (r, c) if direction == 1 else (nr, nc)
                if self.moves[h][mr, mc]:
                    neighbors.append(((nr, nc, h), hypot(dr, dc) * self.grid.resolution_m))
            for other, weight in neighbors:
                total = cost + weight
                if total + 1e-10 < best.get(other, float('inf')):
                    best[other], parent[other] = total, node
                    heappush(queue, (total, other))
        reason = FailureReason.TIMEOUT if timed_out else FailureReason.NO_PATH
        answers.update({ident: PlannerResult(False, None, None, reason)
                        for ids in desired.values() for ident in ids})
        return answers


class LatticeNavigator(TeamNavigator):
    """Drop-in Navigator with yaw in both assessment and execution planning."""
    def _engine(self, grid):
        old = getattr(self, '_lattice', None)
        if old is None or old.grid.data != grid.data or old.grid.resolution_m != grid.resolution_m:
            old = self._lattice = LatticeSearch(grid, self._robot)
        return old

    def evaluate_candidates(self, request):
        self._require_ready(request.episode_id)
        state = request.state
        if state.episode_id != request.episode_id:
            raise ValueError('candidate episode mismatch')
        results = self._engine(state.occupancy_map).solve(state.robot_pose,
            {f.frontier_id: f.approach_pose for f in state.frontiers}, self._nav_cfg.batch_deadline_s)
        self._latest_revision = max(self._latest_revision, state.revision)
        return CandidateEvaluation(state.episode_id, state.revision, state.robot_pose, tuple(
            CandidateAssessment(f.frontier_id, True if results[f.frontier_id].success else
                None if results[f.frontier_id].failure_reason == FailureReason.TIMEOUT else False,
                results[f.frontier_id].path_cost_m, results[f.frontier_id].failure_reason)
            for f in state.frontiers))

    def start(self, request):
        self._require_ready(request.episode_id)
        op = self._begin(request.goal_id, 'navigate')
        op.goal_pose = request.subgoal.approach_pose
        live = self._control.current_pose()
        if not self._fresh(request.occupancy_map, request.start_pose, live):
            return self._finish(op, NavigationStatus.FAILED, FailureReason.INVALID_GOAL)
        op.costmap = self._costmap_for(request.occupancy_map)
        result = self._engine(request.occupancy_map).solve(live, {'goal': op.goal_pose},
                                                         self._nav_cfg.batch_deadline_s)['goal']
        op.planner_result = result
        self._log('plan_se2', '%s success=%s reason=%s' % (op.goal_id, result.success, result.failure_reason))
        if not result.success:
            return self._finish(op, NavigationStatus.FAILED, result.failure_reason)
        op.path, op.index = result.path, 1
        return self._feedback(op)

    def _tick_navigation(self, op):
        pose, robot = self._control.current_pose(), self._robot
        while op.index < len(op.path):
            target = op.path[op.index]
            distance = hypot(target.x_m - pose.x_m, target.y_m - pose.y_m)
            err = _wrap(target.yaw_rad - pose.yaw_rad)
            tolerance = .025 if op.index < len(op.path)-1 else robot.position_tolerance_m
            if distance <= tolerance and abs(err) <= .025:
                op.index += 1
                continue
            if distance <= tolerance:
                if not op.costmap.rotation_valid(pose, target.yaw_rad):
                    self._log('turn_rejected', op.goal_id)
                    return self._finish(op, NavigationStatus.FAILED, FailureReason.BLOCKED)
                self._control.set_velocity(0., _clip(3 * err, robot.max_angular_speed_radps))
                return
            bearing = atan2(target.y_m-pose.y_m, target.x_m-pose.x_m)
            reverse = cos(bearing - target.yaw_rad) < 0
            desired = _wrap(bearing + (pi if reverse else 0))
            error = _wrap(desired - pose.yaw_rad)
            if abs(error) > .08:
                if not op.costmap.rotation_valid(pose, desired):
                    return self._finish(op, NavigationStatus.FAILED, FailureReason.BLOCKED)
                self._control.set_velocity(0., _clip(3 * error, robot.max_angular_speed_radps))
                return
            speed = min(robot.max_linear_speed_mps, 3 * distance) * (-1 if reverse else 1)
            # Verify the short curved segment actually about to be driven; this
            # catches map changes/off-grid tracking errors before any contact.
            horizon = .10
            w = _clip(3 * error, robot.max_angular_speed_radps)
            projected = Pose(pose.x_m + speed*cos(pose.yaw_rad+w*horizon/2)*horizon,
                             pose.y_m + speed*sin(pose.yaw_rad+w*horizon/2)*horizon, 0,
                             _wrap(pose.yaw_rad+w*horizon))
            if not op.costmap.pose_valid(projected):
                self._control.set_velocity(0., 0.)
                return self._finish(op, NavigationStatus.FAILED, FailureReason.BLOCKED)
            self._control.set_velocity(speed, w)
            return
        self._finish(op, NavigationStatus.REACHED, None)

    def _remaining_path_valid(self, op, live):
        if not op.costmap.pose_valid(live):
            return False
        points = (live,) + tuple(op.path[op.index:])
        for a, b in zip(points, points[1:]):
            if hypot(a.x_m-b.x_m, a.y_m-b.y_m) < .03:
                if not op.costmap.rotation_valid(a, b.yaw_rad):
                    return False
            elif not op.costmap.segment_valid(a, b):
                return False
        return True

    def _replan(self, op, live):
        op.replans += 1
        self._control.set_velocity(0., 0.)
        if op.replans > self._nav_cfg.max_replans:
            return self._finish(op, NavigationStatus.FAILED, FailureReason.BLOCKED)
        result = self._engine(op.costmap.grid).solve(live, {'goal':op.goal_pose},
                                                   self._nav_cfg.batch_deadline_s)['goal']
        op.planner_result = result
        self._log('replan_se2', '%s #%d %s' % (op.goal_id, op.replans, result.success))
        if not result.success:
            return self._finish(op, NavigationStatus.FAILED, result.failure_reason)
        op.path, op.index = result.path, 1
