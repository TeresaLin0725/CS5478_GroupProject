"""Measured PyBullet demo; range mapping + oracle visible-class detector ablation.

Not iGibson or the production RGB-D/YOLO pipeline. Geometry is procedural,
the robot is a velocity-actuated cylinder, and scanning uses a virtual panorama.
Only rendered visible objects enter policy state; no full scene map is supplied.
"""
from collections import deque
from math import atan2, cos, hypot, pi, sin
from pathlib import Path

import numpy as np
import pybullet as p
from PIL import Image, ImageDraw

from ..contracts import (CandidateAssessment, CandidateEvaluation, ExploredGrid,
    FailureReason, Frontier, GridCell, ObjectTrack, OccupancyGrid, Point2, Point3,
    Pose, RegionSummary, TargetClaim, WorldState)


RES = 0.25
WIDTH, HEIGHT = 48, 32
RADIUS = 0.22
DT = 1.0 / 120.0


class BulletWorld:
    def __init__(self, gui=False, blocked=False):
        self.client = p.connect(p.GUI if gui else p.DIRECT)
        if self.client < 0:
            raise RuntimeError('Cannot connect to PyBullet')
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(DT)
        self.obstacles, self.labels = [], {}
        self.box((6, 4, -0.1), (6, 4, 0.1), (0.84, 0.86, 0.87, 1), obstacle=False)
        for center, half in [((6, 0, 1), (6, .1, 1)), ((6, 8, 1), (6, .1, 1)),
                             ((0, 4, 1), (.1, 4, 1)), ((12, 4, 1), (.1, 4, 1)),
                             ((4, 5.5, 1), (.1, 2.5, 1)), ((8, 5.5, 1), (.1, 2.5, 1))]:
            self.box(center, half, (.55, .61, .69, 1))
        # Three doorways, width 1.6 m, connect a common corridor to each room.
        for offset in (0, 4, 8):
            self.box((offset + .6, 3, .9), (.6, .1, .9), (.6, .66, .74, 1))
            self.box((offset + 3.4, 3, .9), (.6, .1, .9), (.6, .66, .74, 1))
        self.box((1.2, 6.5, .35), (.7, .45, .35), (.3, .5, .8, 1), label='sofa')
        self.box((3.4, 6.5, .6), (.16, .55, .6), (.16, .2, .24, 1), label='television')
        self.box((5.3, 6.3, .3), (.7, 1.0, .3), (.6, .4, .75, 1), label='bed')
        self.box((7.5, 6.9, .8), (.3, .5, .8), (.55, .38, .2, 1), label='wardrobe')
        self.box((9.0, 5.2, .45), (.4, .55, .45), (.3, .7, .7, 1), label='sink')
        self.box((11.3, 6.7, .8), (.4, .5, .8), (.9, .92, .94, 1), label='refrigerator')
        self.box((10.1, 6.8, .35), (.5, .4, .35), (.64, .48, .3, 1), label='table')
        self.target = self.box((10.1, 6.8, .85), (.10, .10, .15), (.94, .24, .15, 1), label='mug')
        if blocked:
            self.box((10, 3, .9), (.8, .15, .9), (.8, .2, .2, 1))
        shape = p.createCollisionShape(p.GEOM_CYLINDER, radius=RADIUS, height=.4)
        visual = p.createVisualShape(p.GEOM_CYLINDER, radius=RADIUS, length=.4,
                                     rgbaColor=(.12, .72, .28, 1))
        self.robot = p.createMultiBody(1.0, shape, visual, (1.5, 1.5, .22))
        p.changeDynamics(self.robot, -1, lateralFriction=0.0, angularDamping=1.0,
                         linearDamping=0.0)
        for _ in range(30):
            p.stepSimulation()
        self.steps, self.distance, self.collision_steps = 0, 0.0, 0
        self.trajectory = [(1.5, 1.5)]
        self.last_rgb = Image.new('RGB', (256, 192))
        self.visible_target_frames = set()
        if gui:
            p.resetDebugVisualizerCamera(12, 0, -70, (6, 4, 0))

    def box(self, pos, half, color, label=None, obstacle=True):
        collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
        visual = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=color)
        body = p.createMultiBody(0, collision, visual, pos)
        if obstacle:
            self.obstacles.append(body)
        if label:
            self.labels[body] = label
        return body

    def pose(self):
        xyz, orientation = p.getBasePositionAndOrientation(self.robot)
        return Pose(xyz[0], xyz[1], 0.0, p.getEulerFromQuaternion(orientation)[2])

    def step_velocity(self, vx=0.0, vy=0.0, omega=0.0):
        before = self.pose()
        p.resetBaseVelocity(self.robot, (vx, vy, 0), (0, 0, omega))
        p.stepSimulation()
        self.steps += 1
        after = self.pose()
        self.distance += hypot(after.x_m - before.x_m, after.y_m - before.y_m)
        if self.steps % 12 == 0:
            self.trajectory.append((after.x_m, after.y_m))
        touching = any(c[2] in self.obstacles and c[8] <= .002 and c[9] > .01
                       for c in p.getContactPoints(bodyA=self.robot))
        self.collision_steps += int(touching)
        return touching

    def stop(self):
        p.resetBaseVelocity(self.robot, (0, 0, 0), (0, 0, 0))

    def camera(self, yaw):
        pose = self.pose()
        eye = (pose.x_m + .27*cos(yaw), pose.y_m + .27*sin(yaw), .75)
        target = (eye[0] + cos(yaw), eye[1] + sin(yaw), .7)
        view = p.computeViewMatrix(eye, target, (0, 0, 1))
        projection = p.computeProjectionMatrixFOV(85, 256/192, .05, 5.0)
        _, _, rgba, depth, segmentation = p.getCameraImage(256, 192, view, projection,
            renderer=p.ER_TINY_RENDERER)
        self.last_rgb = Image.fromarray(np.asarray(rgba, dtype=np.uint8).reshape(192, 256, 4)[:, :, :3])
        mask = np.asarray(segmentation).reshape(192, 256)
        return mask

    def render(self, mapper, title):
        view = p.computeViewMatrix((6, -5.8, 12.7), (6, 4, 0), (0, 0, 1))
        projection = p.computeProjectionMatrixFOV(56, 704/480, .1, 40)
        _, _, rgba, _, _ = p.getCameraImage(704, 480, view, projection, renderer=p.ER_TINY_RENDERER)
        canvas = Image.new('RGB', (1000, 560), '#142033')
        canvas.paste(Image.fromarray(np.asarray(rgba, dtype=np.uint8).reshape(480, 704, 4)[:, :, :3]), (0, 45))
        canvas.paste(self.last_rgb, (728, 64))
        colors = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        colors[mapper.cells == -1] = (99, 112, 133)
        colors[mapper.cells == 0] = (235, 240, 245)
        colors[mapper.cells == 100] = (26, 34, 49)
        grid = Image.fromarray(colors[::-1]).resize((264, 176), Image.Resampling.NEAREST)
        draw_grid = ImageDraw.Draw(grid)
        points = [(x / 12 * 264, (8-y)/8*176) for x, y in self.trajectory]
        if len(points) > 1:
            draw_grid.line(points, fill='#21b76a', width=2)
        x, y = points[-1]
        draw_grid.ellipse((x-3, y-3, x+3, y+3), fill='#ff8c32')
        canvas.paste(grid, (720, 300))
        draw = ImageDraw.Draw(canvas)
        draw.text((16, 13), title, fill='white')
        draw.text((728, 44), 'Robot RGB view', fill='white')
        draw.text((720, 277), 'Observed map + executed trajectory', fill='white')
        draw.text((16, 531), 'PyBullet | range mapping | oracle visible-class detector | velocity-actuated cylinder', fill='white')
        return canvas

    def close(self):
        p.disconnect(self.client)


class RangeMapper:
    def __init__(self, world, episode_id):
        self.world, self.episode_id = world, episode_id
        self.cells = np.full((HEIGHT, WIDTH), -1, dtype=int)
        self.revision, self.sequence = 0, 0
        self.tracks = {}
        self.claim = None
        self.frontier_history = []
        self.next_id = 1

    def observe(self, panoramic=True):
        world, pose = self.world, self.world.pose()
        angles = np.linspace(-pi, pi, 720, endpoint=False)
        starts = [(pose.x_m + .27*cos(a), pose.y_m + .27*sin(a), .25) for a in angles]
        ends = [(pose.x_m + 4*cos(a), pose.y_m + 4*sin(a), .25) for a in angles]
        hits = p.rayTestBatch(starts, ends)
        for angle, hit in zip(angles, hits):
            distance = .27 + (4 - .27)*hit[2] if hit[0] >= 0 else 4.0
            for distance_sample in np.arange(0, max(0, distance-.08), .10):
                col = int((pose.x_m + distance_sample*cos(angle))/RES)
                row = int((pose.y_m + distance_sample*sin(angle))/RES)
                if 0 <= row < HEIGHT and 0 <= col < WIDTH and self.cells[row, col] != 100:
                    self.cells[row, col] = 0
            if hit[0] >= 0:
                # Quantize just inside the struck surface. A hit exactly on a
                # grid boundary must not occupy the adjacent free-side cell.
                col = int((hit[3][0] + .02*cos(angle))/RES)
                row = int((hit[3][1] + .02*sin(angle))/RES)
                if 0 <= row < HEIGHT and 0 <= col < WIDTH:
                    self.cells[row, col] = 100
        for yaw in ((0, pi/2, pi, -pi/2) if panoramic else (pose.yaw_rad,)):
            for _ in range(2):
                world.step_velocity()
                self.sequence += 1
                mask = world.camera(yaw)
                for body, label in world.labels.items():
                    if int(np.count_nonzero(mask == body)) < 10:
                        continue
                    # Explicit oracle detector: class and localization obtained only
                    # for bodies present in the rendered camera segmentation.
                    xyz, _ = p.getBasePositionAndOrientation(body)
                    if hypot(xyz[0]-pose.x_m, xyz[1]-pose.y_m) > 5:
                        continue
                    track_id = 'track-' + label
                    stamp = world.steps*DT
                    obs_id = 'capture-' + str(self.sequence) + '-' + label
                    prior = self.tracks.get(track_id)
                    ids = prior.observation_ids + (obs_id,) if prior else (obs_id,)
                    self.tracks[track_id] = ObjectTrack(track_id, label, 1.0, Point3(*xyz),
                        prior.first_seen_s if prior else stamp, stamp, ids, len(ids))
                    if label == 'mug':
                        world.visible_target_frames.add(obs_id)
                        if len(ids) >= 2 and self.claim is None:
                            self.claim = TargetClaim('claim-1', track_id, 'mug', stamp, ids[-2:])
        self.revision += 1

    @staticmethod
    def region(x, y):
        return 'R' + str(min(2, int(x//4)) + 1) if y >= 3 else 'R0'

    def traversable(self):
        # Conservative radius+margin inflation, including unknown/out of bounds.
        free = self.cells == 0
        safe = np.zeros_like(free)
        for row in range(1, HEIGHT-1):
            for col in range(1, WIDTH-1):
                safe[row, col] = bool(np.all(free[row-1:row+2, col-1:col+2]))
        return safe

    def paths(self):
        pose = self.world.pose()
        start = (int(pose.y_m/RES), int(pose.x_m/RES))
        safe = self.traversable()
        if not safe[start]:
            return {}, safe
        queue, parent = deque([start]), {start: None}
        while queue:
            row, col = queue.popleft()
            for dr, dc in ((0,1),(0,-1),(1,0),(-1,0)):
                other = row+dr, col+dc
                if 0 <= other[0] < HEIGHT and 0 <= other[1] < WIDTH and safe[other] and other not in parent:
                    parent[other] = (row,col)
                    queue.append(other)
        return parent, safe

    def snapshot(self):
        pose, parent = self.world.pose(), self.paths()[0]
        support = set()
        for row in range(1, HEIGHT-1):
            for col in range(1, WIDTH-1):
                if self.cells[row,col] == 0 and any(self.cells[row+dr,col+dc] == -1
                            for dr,dc in ((0,1),(0,-1),(1,0),(-1,0))):
                    support.add((row,col))
        clusters = []
        while support:
            seed = min(support)
            region = self.region((seed[1]+.5)*RES, (seed[0]+.5)*RES)
            support.remove(seed)
            queue, cluster = deque([seed]), [seed]
            while queue:
                row,col = queue.popleft()
                for dr,dc in ((0,1),(0,-1),(1,0),(-1,0)):
                    cell = row+dr,col+dc
                    if cell in support and self.region((cell[1]+.5)*RES,(cell[0]+.5)*RES) == region:
                        support.remove(cell)
                        queue.append(cell)
                        cluster.append(cell)
            if len(cluster) >= 3:
                clusters.append((region,cluster))
        frontiers, route_by_id, history = [], {}, []
        used = set()
        for region, cluster in clusters:
            center = np.mean(cluster, axis=0)
            # Offset from unknown boundary into B's footprint-safe reachable cells.
            choices = [cell for cell in parent if min(abs(cell[0]-r)+abs(cell[1]-c) for r,c in cluster) <= 4]
            if not choices:
                continue
            goal = min(choices, key=lambda c: ((c[0]-center[0])**2+(c[1]-center[1])**2,c))
            x,y = (goal[1]+.5)*RES, (goal[0]+.5)*RES
            if hypot(x-pose.x_m,y-pose.y_m) < .35:
                continue
            matched = [(fid,len(set(cluster)&cells)) for fid,cells in self.frontier_history if fid not in used]
            matched = [m for m in matched if m[1]>0]
            if matched:
                fid = min(matched, key=lambda item:(-item[1],item[0]))[0]
            else:
                fid = 'F' + str(self.next_id)
                self.next_id += 1
            used.add(fid)
            history.append((fid,set(cluster)))
            yaw = atan2((center[0]+.5)*RES-y,(center[1]+.5)*RES-x)
            frontiers.append(Frontier(fid, (), tuple(GridCell(*c) for c in sorted(cluster)),
                Pose(x,y,0,yaw),region,len(cluster)*RES*RES,hypot(x-pose.x_m,y-pose.y_m)))
            route, cell = [], goal
            while cell is not None:
                route.append(((cell[1]+.5)*RES,(cell[0]+.5)*RES))
                cell = parent[cell]
            route_by_id[fid] = list(reversed(route))
        self.frontier_history = history
        regions = []
        for i,(x0,x1,y0,y1) in enumerate(((0,12,0,3),(0,4,3,8),(4,8,3,8),(8,12,3,8))):
            observed = tuple(sorted({t.class_name for t in self.tracks.values()
                if self.region(t.position_world.x_m,t.position_world.y_m) == 'R'+str(i)}))
            part = self.cells[int(y0/RES):int(y1/RES),int(x0/RES):int(x1/RES)]
            regions.append(RegionSummary('R'+str(i), (Point2(x0,y0),Point2(x1,y0),Point2(x1,y1),Point2(x0,y1)),
                'igibson_room_prior',float(np.mean(part!=-1)),observed,None,None))
        grid = OccupancyGrid('world',self.revision,RES,0,0,WIDTH,HEIGHT,
                              tuple(tuple(int(v) for v in row) for row in self.cells))
        explored = ExploredGrid('world',self.revision,RES,0,0,WIDTH,HEIGHT,
                                 tuple(tuple(bool(v!=-1) for v in row) for row in self.cells))
        state = WorldState('1.0',self.episode_id,self.revision,self.world.steps*DT,self.sequence,'mug',pose,
            grid,explored,tuple(self.tracks.values()),tuple(regions),tuple(frontiers),self.claim)
        assessments = []
        for frontier in frontiers:
            path = route_by_id[frontier.frontier_id]
            cost = hypot(path[0][0]-pose.x_m,path[0][1]-pose.y_m) + (len(path)-1)*RES
            assessments.append(CandidateAssessment(frontier.frontier_id,True,cost,None))
        evaluation = CandidateEvaluation(self.episode_id,self.revision,pose,tuple(assessments))
        return state,evaluation,route_by_id
