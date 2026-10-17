"""Realistic Gibson Rs challenges with a physical Locobot and recorded viewer."""
import argparse
import json
from datetime import datetime
import os
import subprocess
import sys
from math import atan2, cos, hypot, pi
from pathlib import Path
import time

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from ..config import PolicyConfig
from ..contracts import (FailureReason, FrontierExclusion, PlannerAttempt, PlannerResult,
                         PolicyName, Pose)
from ..experiments.logging import JsonlLogger, dumps
from ..policies.baselines import BaselinePolicy
from ..policies.llm import LLMGuidedPolicy
from ..policies.provider import GeminiCompatibleClient, load_local_env
from ..simulation.igibson_rs import (CAMERA_PITCH_RAD, DATA_ROOT, HEIGHT, INITIAL_CLEARANCE_M,
    LANDMARKS, LEVEL_PITCH_RAD, MAX_ANGULAR_RADPS, MAX_LINEAR_MPS, ORIGIN_X, ORIGIN_Y, REGIONS, RES,
    SCENE_ID, TARGET_CLASS, WIDTH, IGibsonWorld, RGBDMapper, world_to_cell, wrap)
from .episode import build_context, decide
from ..simulation.indoor_challenges import PROFILES
from .demo_video import DemoVideo

SCAN_HEADINGS = 6
OBSERVE_EVERY_STEPS = 15
LOOKAHEAD_M = 0.35
GOAL_TOLERANCE_M = 0.12
LIMITS = {'decisions': 25, 'sim_s': 600.0, 'wall_s': 1200.0}
CLOSE_DISTANCE_M = .85
POLICIES = ('random', 'nearest', 'information_gain', 'llm_guided')


class RecordedResponses:
    """Authored interface replay from this scenario's IG run, never inference."""
    def __init__(self, output):
        self.answers = json.loads((output / 'igibson.llm-offline-responses.json').read_text(encoding='utf-8'))['frontier_ids']
        self.index = 0

    def select_id(self, payload, **kwargs):
        answer = self.answers[self.index]
        self.index += 1
        self.last_record = {'reason': 'authored IG interface replay; not model inference', 'usage': None, 'cost': None}
        return answer


def prepare_responses(output):
    ids = []
    for line in (output / 'information_gain.jsonl').read_text(encoding='utf-8').splitlines():
        row = json.loads(line)
        goal = (row.get('selection') or {}).get('subgoal')
        if row['event'] == 'decision' and goal and goal['kind'] == 'frontier':
            ids.append(goal['frontier_id'])
    (output / 'igibson.llm-offline-responses.json').write_text(json.dumps({
        'provenance': 'Authored from this scenario Information Gain selections. NOT LLM inference.',
        'source': 'information_gain.jsonl', 'frontier_ids': ids}, indent=2), encoding='utf-8')


class Episode:
    def __init__(self, name, output, model, profile='dynamic', backend='replay', video_fps=15):
        self.name, self.output, self.model = name, output, model
        self.profile, self.backend = profile, backend
        self.world = IGibsonWorld(profile=profile)
        self.mapper = RGBDMapper(self.world, 'igibson-rs-' + profile + '-' + name, close_distance_m=CLOSE_DISTANCE_M)
        self.llm = name == 'llm_guided'
        self.policy = (LLMGuidedPolicy(GeminiCompatibleClient() if backend == 'live' else RecordedResponses(output))
                       if self.llm else BaselinePolicy(PolicyName(name)))
        self.policy.reset(self.mapper.episode_id, 0,
                          PolicyConfig(llm_model=model, llm_deadline_s=20.0, llm_call_cap=30))
        self.frames, self.frame_records, self.caption = [], [], 'initial scan'
        self.frame_directory = self.output / 'frames' / name
        self.frame_directory.mkdir(parents=True, exist_ok=True)
        self.floorplan = Image.open(DATA_ROOT / 'g_dataset' / SCENE_ID / 'floorplan.png').convert('RGB')
        self.font = ImageFont.truetype('C:/Windows/Fonts/segoeui.ttf', 17)
        self.title_font = ImageFont.truetype('C:/Windows/Fonts/segoeuib.ttf', 24)
        self.navigation_replans = 0
        self.navigation_failures = 0
        self.selected, self.route_drawn = None, []
        self.last_state = None
        self.started = time.monotonic()
        self.event_log, self.active_goal = None, None
        self.feedback_events = []
        self.video = DemoVideo(output / (name + '.mp4'), video_fps) if video_fps else None
        self.world.step_observer = self.record_step

    def record_step(self):
        if self.video is not None and self.video.due(self.world.steps):
            self.video.capture(self.render(live_camera=True), self.world, self.caption)

    def emit_feedback(self, kind, **payload):
        event = {'kind': kind, 'time_s': round(self.world.sim_time, 6),
                 'goal_id': self.active_goal, 'map_revision': self.mapper.revision, **payload}
        self.feedback_events.append(event)
        if self.event_log is not None:
            self.event_log.write('feedback_flow', self.mapper.episode_id, **event)

    def motion_failure(self, reason, detail, replans):
        self.world.stop()
        self.emit_feedback('B_to_C_replan_request', failure_reason=reason.value, detail=detail,
                           replans=replans)
        return 'failed', reason, replans

    # ---------- perception while stopped or moving ----------
    def confirm_target(self, pitch=CAMERA_PITCH_RAD):
        """A single target sighting triggers one extra capture from a later time step."""
        if self.mapper.claim is None and ('track-' + TARGET_CLASS) in self.mapper.tracks:
            self.world.stop(2)
            self.mapper.observe(pitch)

    def rotate_to(self, yaw):
        for _ in range(240):
            error = wrap(yaw - self.world.pose().yaw_rad)
            if abs(error) < 0.05:
                break
            self.world.drive(0.0, max(-MAX_ANGULAR_RADPS, min(MAX_ANGULAR_RADPS, 2.5 * error)))
            self.mapper.mark_footprint()
        self.world.stop()

    def scan(self):
        track = self.mapper.tracks.get('track-' + TARGET_CLASS)
        if track is not None:
            pose = self.world.pose()
            position = track.position_world
            distance = hypot(position.x_m-pose.x_m, position.y_m-pose.y_m)
            if distance <= CLOSE_DISTANCE_M + .25:
                # A countertop target rises above a low camera's level FOV when
                # approached. Aim the inspection view using the observed track.
                self.rotate_to(atan2(position.y_m-pose.y_m, position.x_m-pose.x_m))
                for _ in range(20):
                    self.world.stop(1)
                    linear, angular = self.world.p.getBaseVelocity(next(iter(self.world.robot_bodies)))
                    if hypot(linear[0],linear[1]) < .02 and abs(angular[2]) < .04:
                        break
                eye = self.world.robot.eyes.get_position()
                pitch = max(-pi/6, min(pi/4, atan2(position.z_m-eye[2], max(.1,distance))))
                self.mapper.observe(pitch)
                self.world.stop(3)
                self.mapper.observe(pitch)
                self.caption = 'stationary close-range countertop inspection'
                self.frame()
                if self.mapper.claim:
                    return
        start = self.world.pose().yaw_rad
        for k in range(SCAN_HEADINGS):
            self.rotate_to(wrap(start + 2 * pi * k / SCAN_HEADINGS))
            for pitch in (CAMERA_PITCH_RAD, LEVEL_PITCH_RAD):
                self.mapper.observe(pitch)
                self.confirm_target(pitch)
                if self.mapper.claim:
                    break
            self.frame()
            if self.mapper.claim:
                return

    # ---------- B stand-in: path following with measured feedback ----------
    def follow(self, route, goal_cell):
        world, mapper = self.world, self.mapper
        index, replans, contact_streak, started, progress = 0, 0, 0, world.steps, []
        budget = int((30 + 8 * len(route) * RES) / (1 / 30.0))
        while world.steps - started < budget:
            if world.sim_time > LIMITS['sim_s'] or time.monotonic() - self.started > LIMITS['wall_s']:
                return self.motion_failure(FailureReason.TIMEOUT, 'episode_budget', replans)
            pose = world.pose()
            while index < len(route) - 1 and hypot(route[index][0] - pose.x_m, route[index][1] - pose.y_m) < LOOKAHEAD_M:
                index += 1
            tx, ty = route[index]
            distance = hypot(tx - pose.x_m, ty - pose.y_m)
            final = index == len(route) - 1
            if final and distance < GOAL_TOLERANCE_M:
                return 'reached', None, replans
            error = wrap(atan2(ty - pose.y_m, tx - pose.x_m) - pose.yaw_rad)
            speed = 0.0 if abs(error) > 0.5 else min(MAX_LINEAR_MPS, 1.5 * distance if final else MAX_LINEAR_MPS) * cos(error)
            contact = world.drive(speed, max(-MAX_ANGULAR_RADPS, min(MAX_ANGULAR_RADPS, 2.5 * error)))
            contact_streak = contact_streak + 1 if contact else 0
            if contact_streak > 20:
                return self.motion_failure(FailureReason.BLOCKED, 'persistent_contact', replans)
            if speed > 0.05:
                progress.append((world.steps, pose.x_m, pose.y_m))
                if progress[-1][0] - progress[0][0] >= 90:
                    moved = hypot(progress[-1][1] - progress[0][1], progress[-1][2] - progress[0][2])
                    if moved < 0.05:   # commanded forward for 3 s without moving
                        return self.motion_failure(FailureReason.BLOCKED, 'motion_stalled', replans)
                    progress.clear()
            else:
                progress.clear()
            if world.steps % 3 == 0:
                mapper.mark_footprint()
            if world.steps % OBSERVE_EVERY_STEPS == 0:
                mapper.observe()
                self.confirm_target()
                self.emit_feedback('A_to_B_map_update')
                if world.steps % (2 * OBSERVE_EVERY_STEPS) == 0 or mapper.claim:
                    self.frame()
                if mapper.claim:
                    return 'cancelled', None, replans
                safe = mapper.traversable()
                ahead = [p for p in route[index:] if hypot(p[0] - pose.x_m, p[1] - pose.y_m) > 0.3]
                if any(not safe[world_to_cell(x, y)] for x, y in ahead):
                    blocked = [world_to_cell(x,y) for x,y in ahead if not safe[world_to_cell(x,y)]]
                    self.emit_feedback('A_to_B_route_invalidated', blocked_cells=blocked[:12])
                    # Stop the old command before doing any potentially costly planning.
                    world.stop()
                    if replans >= 5:
                        return self.motion_failure(FailureReason.BLOCKED, 'replan_limit', replans)
                    replans += 1
                    cost, parent, _ = mapper.plan(goal_cell)
                    self.emit_feedback('B_local_replan', attempt=replans, reachable=goal_cell in parent)
                    if goal_cell not in parent:
                        return self.motion_failure(FailureReason.BLOCKED, 'updated_map_no_path', replans)
                    route, index = mapper.route(parent, goal_cell), 0
                    self.route_drawn = route
                    self.caption = 'depth update -> path replanned (%d)' % replans
        return self.motion_failure(FailureReason.TIMEOUT, 'navigation_budget', replans)

    # ---------- visualisation (human debugging only; never policy input) ----------
    def frame(self):
        picture = self.render()
        index = len(self.frame_records)
        filename = self.frame_directory / ('%04d.webp' % index)
        picture.save(filename, quality=86)
        self.frames.append(picture.resize((800, 500), Image.LANCZOS))
        pose = self.world.pose()
        self.frame_records.append({'time_s': round(self.world.sim_time, 3),
            'image': filename.relative_to(self.output).as_posix(), 'caption': self.caption,
            'pose': {'x_m': pose.x_m, 'y_m': pose.y_m, 'yaw_rad': pose.yaw_rad},
            'distance_m': round(self.world.distance, 3), 'contacts': self.world.collision_steps})

    def render(self, live_camera=False):
        world, mapper = self.world, self.mapper
        canvas = Image.new('RGB', (1440, 900), '#0c1724')
        draw = ImageDraw.Draw(canvas)
        pose = world.pose()
        draw.text((24, 16), 'GIBSON Rs / LOCOBOT', fill='#ecf5f6', font=self.title_font)
        draw.text((24, 53), '%s / %s / %.1f m / %.1f s' %
                  (self.profile.upper(), self.name, world.distance, world.sim_time), fill='#7acfc1', font=self.font)
        draw.text((24, 86), self.caption, fill='#c1d2df', font=self.font)
        canvas.paste(Image.fromarray(world.spectator()).resize((880, 660), Image.LANCZOS), (24, 140))
        draw.text((24, 114), 'Third-person / textured scan + physical robot', fill='#a9bdcc', font=self.font)
        camera = Image.fromarray(world.preview_camera() if live_camera else world.last_rgb)
        if world.last_target_bbox and not live_camera:
            x0, y0, x1, y1 = world.last_target_bbox
            ImageDraw.Draw(camera).rectangle((x0, y0, x1, y1), outline='#fc7259', width=3)
        canvas.paste(camera.resize((480, 360), Image.LANCZOS), (936, 140))
        draw.text((936, 114), 'Locobot RGB camera', fill='#a9bdcc', font=self.font)
        grid = mapper.cells
        colors = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        colors[grid == -1] = (64, 81, 99)
        colors[grid == 0] = (214, 228, 228)
        colors[grid == 100] = (18, 29, 42)
        if self.last_state is not None:
            for frontier in self.last_state.frontiers:
                for cell in frontier.support_cells:
                    colors[cell.row, cell.column] = (255, 95, 135) if self.selected == frontier.frontier_id else (250, 179, 62)
        occupancy = Image.fromarray(colors).resize((252, 306), Image.NEAREST)
        md = ImageDraw.Draw(occupancy)
        pixel = lambda x,y: ((x-ORIGIN_X)/RES*252/WIDTH,(y-ORIGIN_Y)/RES*306/HEIGHT)
        if len(world.trajectory) > 1:
            md.line([pixel(x,y) for x,y in world.trajectory], fill='#1fad90', width=2)
        if self.route_drawn:
            md.line([pixel(x,y) for x,y in self.route_drawn], fill='#529dff', width=2)
        x,y = pixel(pose.x_m,pose.y_m)
        md.ellipse((x-4,y-4,x+4,y+4),fill='#ffd45b')
        canvas.paste(occupancy, (936, 542))
        overview = self.floorplan.crop((50, 60, 730, 880))
        od = ImageDraw.Draw(overview)
        pixel = lambda x,y: (x*100+450,y*100+440)
        if len(world.trajectory)>1:
            od.line([pixel(x,y) for x,y in world.trajectory], fill='#1fad90', width=5)
        for record in world.furniture:
            position = record['object'].get_position()
            x,y = pixel(position[0],position[1])
            od.ellipse((x-22,y-22,x+22,y+22),outline='#d78a27',width=4)
        x,y = pixel(pose.x_m,pose.y_m)
        od.ellipse((x-16,y-16,x+16,y+16),fill='#ffd45b',outline='#182d30',width=3)
        canvas.paste(overview.resize((210, 253),Image.LANCZOS), (1206,542))
        draw.text((936,514),'Sensed map',fill='#a9bdcc',font=self.font)
        draw.text((1206,514),'Scan floor plan',fill='#a9bdcc',font=self.font)
        draw.text((24,822),'Wheel velocity control / RGB-D fusion / hidden target / close-range inspection',fill='#a9bdcc',font=self.font)
        tx,ty,_ = world.target_position()
        draw.text((24,853),'%.2f m robot radius / %d contacts / target distance %.2f m (viewer only)' %
                  (world.robot_radius,world.collision_steps,hypot(pose.x_m-tx,pose.y_m-ty)),fill='#7acfc1',font=self.font)
        return canvas

    # ---------- evaluator (ground truth; result never returned to the policy) ----------
    def evaluate(self, claim):
        evidence = dict(self.mapper.target_evidence)
        if not set(claim.evidence_observation_ids) <= set(evidence):
            return False, None
        truth = self.world.target_position()
        errors = [hypot(evidence[i][0] - truth[0], evidence[i][1] - truth[1])
                  for i in claim.evidence_observation_ids]
        pose = self.world.pose()
        nearby = hypot(pose.x_m - truth[0], pose.y_m - truth[1]) <= CLOSE_DISTANCE_M + .08
        observations = [self.mapper.target_observations[i] for i in claim.evidence_observation_ids]
        evidence_near = all(hypot(o['pose'].x_m-truth[0], o['pose'].y_m-truth[1]) <= CLOSE_DISTANCE_M+.08
                            and o['stopped'] for o in observations)
        distinct = len({o['time_s'] for o in observations}) >= 2
        return max(errors) < 0.5 and nearby and evidence_near and distinct, round(max(errors), 3)

    def run(self):
        world, mapper = self.world, self.mapper
        status, decisions, previous = 'search_limit', 0, None
        visited, failed = {}, {}
        path = self.output / (self.name + '.jsonl')
        with JsonlLogger(path) as log:
            self.event_log = log
            log.write('demo_config', mapper.episode_id, source='igibson_static_scan', scene=SCENE_ID,
                      simulator='iGibson 2.2.2 Simulator (headless) + PyBullet physics',
                      robot='Locobot (DifferentialDriveController, wheel velocity commands)',
                      robot_radius_m=round(world.robot_radius, 3), robot_height_m=round(world.robot_height, 3),
                      perception='RGB-D depth fusion (eyes camera, head tilt -25/0 deg); annotated-landmark oracle gated by '
                                 'rendered depth visibility; target from rendered instance segmentation',
                      target=TARGET_CLASS, target_asset='YCB 003_cracker_box', landmarks=[l[0] for l in LANDMARKS],
                      region_prior=[r[0] for r in REGIONS], initial_clearance_m=INITIAL_CLEARANCE_M,
                      grid_resolution_m=RES, model=self.model if self.llm else None, limits=LIMITS)
            log.write('challenge_config', mapper.episode_id, profile=self.profile, backend=self.backend,
                      close_distance_m=CLOSE_DISTANCE_M,
                      furniture=[{k: v for k, v in f.items() if k != 'object'} for f in world.furniture])
            self.frame()
            if self.video is not None:
                self.video.capture(self.render(live_camera=True),world,self.caption)
            self.scan()
            self.frames[0].save(self.output / (self.name + '-start.png'))
            while decisions < LIMITS['decisions']:
                if time.monotonic() - self.started > LIMITS['wall_s'] or world.sim_time > LIMITS['sim_s']:
                    break
                state, evaluation, routes = mapper.snapshot()
                self.last_state = state
                if state.target_claim:
                    valid, error = self.evaluate(state.target_claim)
                    status = 'success' if valid else 'false_target_claim'
                    log.write('terminal_evaluation', mapper.episode_id, claim=state.target_claim,
                              valid_target=valid, position_error_m=error,
                              verifier='claimed evidence is rendered instance segmentation of the YCB body; '
                                       'estimated position within 0.5 m of ground truth; two distinct stopped '
                                       'close-range captures and current robot within 0.93 m')
                    break
                current = {f.frontier_id for f in state.frontiers}
                exclusions = tuple(
                    FrontierExclusion(fid, reason, count, 'exhausted', None)
                    for fid, (reason, count) in sorted({**visited, **failed}.items()) if fid in current)
                context = build_context(state, evaluation, exclusions, previous)
                calls_before = self.policy.calls if self.llm else 0
                choice = decide(self.policy, state, context)
                log.write('decision', mapper.episode_id, state=state, context=context, selection=choice)
                if choice.subgoal is None:
                    status = choice.stop_reason
                    break
                decisions += 1
                if self.llm and self.policy.calls > calls_before:
                    log.write('llm_call', mapper.episode_id, record=self.policy.call_records[-1])
                goal = choice.subgoal
                self.active_goal = goal.frontier_id
                if previous is not None and previous.execution_status == 'failed':
                    self.emit_feedback('C_to_B_strategy_replan', previous_goal=previous.subgoal.frontier_id,
                        previous_failure=previous.failure_reason.value if previous.failure_reason else None,
                        selected_goal=goal.frontier_id, rationale=choice.rationale)
                frontier = next(f for f in state.frontiers if f.frontier_id == goal.frontier_id)
                self.selected = goal.frontier_id
                self.caption = 'decision %d: %s -> %s' % (decisions, goal.frontier_id, frontier.region_id)
                print(self.name, self.caption, '|', choice.rationale, flush=True)
                route = routes[goal.frontier_id]
                self.route_drawn = route
                self.frame()
                cost = next(a.path_cost_m for a in evaluation.assessments if a.frontier_id == goal.frontier_id)
                plan = PlannerResult(True, tuple([state.robot_pose] + [Pose(x, y, 0.0, goal.approach_pose.yaw_rad)
                                                                        for x, y in route]), cost, None)
                before = world.distance
                goal_cell = world_to_cell(*route[-1])
                execution, failure, replans = self.follow(route, goal_cell)
                self.navigation_replans += replans
                self.navigation_failures += int(failure is not None)
                if execution == 'reached':
                    self.rotate_to(goal.approach_pose.yaw_rad)
                    visited[goal.frontier_id] = ('no_progress', visited.get(goal.frontier_id, ('', 0))[1] + 1)
                elif failure:
                    failed[goal.frontier_id] = (failure.value, failed.get(goal.frontier_id, ('', 0))[1] + 1)
                previous = PlannerAttempt(goal, plan, execution, failure)
                log.write('navigation_feedback', mapper.episode_id, attempt=previous, robot_pose=world.pose(),
                          executed_distance_m=round(world.distance - before, 3), replans=replans,
                          collision_steps=world.collision_steps, sim_time_s=round(world.sim_time, 3))
                self.route_drawn = []
                if execution != 'cancelled':
                    self.scan()
            if mapper.claim and status == 'search_limit':
                valid, error = self.evaluate(mapper.claim)
                status = 'success' if valid else 'false_target_claim'
            self.caption = status
            world.stop()
            if mapper.claim:
                track = mapper.tracks[mapper.claim.track_id]
                pose = world.pose()
                distance = hypot(track.position_world.x_m-pose.x_m,track.position_world.y_m-pose.y_m)
                eye = world.robot.eyes.get_position()
                pitch = max(-pi/6,min(pi/4,atan2(track.position_world.z_m-eye[2],max(.1,distance))))
                world.capture(pitch)  # Final display uses the observed countertop height.
            self.frame()
            tx, ty, _ = world.target_position()
            pose = world.pose()
            summary = {'source': 'igibson_static_scan', 'scene': SCENE_ID, 'robot': 'Locobot',
                       'profile': self.profile, 'backend': self.backend if self.llm else None,
                       'physical_robot': True, 'rendered_scene': 'Gibson textured Rs mesh',
                       'policy': self.name, 'termination': status, 'decisions': decisions,
                       'executed_distance_m': round(world.distance, 3), 'sim_time_s': round(world.sim_time, 3),
                       'wall_time_s': round(time.monotonic() - self.started, 3),
                       'collision_steps': world.collision_steps, 'model': self.model if self.llm else None,
                       'llm_calls': self.policy.calls if self.llm else 0,
                       'fallbacks': sum(r['fallback_selected_action'] for r in self.policy.call_records) if self.llm else 0,
                       'observed_classes': sorted(t.class_name for t in mapper.tracks.values()),
                       'known_fraction': round(float((mapper.cells != -1).mean()), 4),
                       'robot_radius_m': round(world.robot_radius, 3),
                       'target_distance_m': round(hypot(pose.x_m-tx,pose.y_m-ty), 3),
                       'close_distance_limit_m': CLOSE_DISTANCE_M + .08,
                       'replans': self.navigation_replans, 'navigation_failures': self.navigation_failures,
                       'environment_events': world.environment_events,
                       'target_detections': len(mapper.target_evidence)}
            if self.video is not None:
                self.video.capture(self.render(live_camera=True),world,self.caption)
                self.video.close()
                summary.update(video_fps=self.video.fps,video_frames=len(self.video.frames),
                               video_max_gap_s=self.video.metadata()['max_gap_s'])
            log.write('terminal', mapper.episode_id, **summary)
            self.event_log = None
        self.frames[-1].save(self.output / (self.name + '-final.png'))
        small = self.frames
        small[0].save(self.output / (self.name + '.gif'), save_all=True, append_images=small[1:],
                      duration=[200] * (len(small) - 1) + [2500], loop=0)
        (self.output / (self.name + '-summary.json')).write_text(dumps(summary), encoding='utf-8')
        (self.output / (self.name + '-trace.json')).write_text(json.dumps({
            'source': 'actual iGibson rendered frames and wheel-driven poses',
            'frames': self.frame_records, 'environment_events': world.environment_events,
            'feedback_events': self.feedback_events,
            'video': self.video.metadata() if self.video is not None else None}), encoding='utf-8')
        print(dumps(summary), flush=True)
        return summary


def run_episode(name, output, model=None, profile='dynamic', backend='replay', video_fps=15):
    episode = Episode(name, output, model, profile, backend, video_fps)
    try:
        return episode.run()
    finally:
        episode.world.step_observer = None
        if episode.video is not None:
            episode.video.close()
        episode.world.close()


def main():
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(errors='replace')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--policy', choices=list(POLICIES)+['both','all'], default='all')
    parser.add_argument('--scenario', choices=list(PROFILES)+['suite'], default='dynamic')
    parser.add_argument('--backend', choices=['replay','live'], default='replay')
    parser.add_argument('--video-fps', type=int, choices=[0,10,15,30], default=15)
    parser.add_argument('--env-file', type=Path, default=Path('../.env'))
    parser.add_argument('--output', type=Path, default=Path('runs') / ('igibson-realistic-' + datetime.now().strftime('%Y%m%d-%H%M%S')))
    parser.add_argument('--existing-output', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args.backend == 'live':
        load_local_env(args.env_file)
        if args.policy in ('llm_guided','both','all') and not os.environ.get('LLM_MODEL'):
            parser.error('LLM_MODEL must be configured for a live LLM run')
    args.output.mkdir(parents=True, exist_ok=args.existing_output)
    if args.scenario == 'suite' or args.policy in ('both','all'):
        profiles = tuple(PROFILES) if args.scenario == 'suite' else (args.scenario,)
        names = POLICIES if args.policy == 'all' else ('nearest','information_gain','llm_guided') if args.policy == 'both' else (args.policy,)
        codes = []
        for profile in profiles:
            output = args.output / profile if args.scenario == 'suite' else args.output
            output.mkdir(parents=True, exist_ok=True)
            for name in names:
                if name == 'llm_guided' and args.backend == 'replay':
                    if not (output / 'information_gain.jsonl').exists():
                        codes.append(subprocess.call([sys.executable,'-m',__spec__.name,'--policy','information_gain',
                            '--scenario',profile,'--output',str(output),'--video-fps',str(args.video_fps),'--existing-output']))
                    prepare_responses(output)
                codes.append(subprocess.call([sys.executable,'-m',__spec__.name,'--policy',name,'--scenario',profile,
                    '--backend',args.backend,'--env-file',str(args.env_file),'--output',str(output),
                    '--video-fps',str(args.video_fps),'--existing-output']))
        from .indoor_viewer import export_viewer
        export_viewer(args.output)
        print('Open:', (args.output / 'index.html').resolve())
        return int(any(codes))
    if (args.output / (args.policy+'.jsonl')).exists():
        parser.error('Choose a new output directory; existing episode logs are preserved')
    if args.policy == 'llm_guided' and args.backend == 'replay' and not (args.output / 'igibson.llm-offline-responses.json').exists():
        code = subprocess.call([sys.executable,'-m',__spec__.name,'--policy','information_gain',
            '--scenario',args.scenario,'--output',str(args.output),'--video-fps',str(args.video_fps),'--existing-output'])
        if not (args.output / 'information_gain.jsonl').exists():
            return code or 1
        prepare_responses(args.output)
    model = os.environ.get('LLM_MODEL') if args.backend == 'live' else 'authored-interface-replay'
    summary = run_episode(args.policy,args.output,model,args.scenario,args.backend,args.video_fps)
    from .indoor_viewer import export_viewer
    export_viewer(args.output)
    print('Artifacts:',args.output.resolve())
    return int(summary['termination'] != 'success')


if __name__ == '__main__':
    raise SystemExit(main())
