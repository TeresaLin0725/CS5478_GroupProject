"""Run Nearest / live Gemini object search in the iGibson Rs scan; export GIF/JSONL."""
import argparse
from datetime import datetime
import os
import subprocess
import sys
from math import atan2, cos, hypot, pi
from pathlib import Path
import time

import numpy as np
from PIL import Image, ImageDraw

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

SCAN_HEADINGS = 6
OBSERVE_EVERY_STEPS = 15
LOOKAHEAD_M = 0.35
GOAL_TOLERANCE_M = 0.12
LIMITS = {'decisions': 25, 'sim_s': 600.0, 'wall_s': 1200.0}


class Episode:
    def __init__(self, name, output, model):
        self.name, self.output, self.model = name, output, model
        self.world = IGibsonWorld()
        self.mapper = RGBDMapper(self.world, 'igibson-rs-' + name)
        self.llm = name == 'llm_guided'
        self.policy = (LLMGuidedPolicy(GeminiCompatibleClient()) if self.llm
                       else BaselinePolicy(PolicyName.NEAREST))
        self.policy.reset(self.mapper.episode_id, 0,
                          PolicyConfig(llm_model=model, llm_deadline_s=20.0, llm_call_cap=30))
        self.frames, self.caption = [], 'initial scan'
        self.selected, self.route_drawn = None, []
        self.last_state = None
        self.started = time.monotonic()

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
                return 'failed', FailureReason.BLOCKED, replans
            if speed > 0.05:
                progress.append((world.steps, pose.x_m, pose.y_m))
                if progress[-1][0] - progress[0][0] >= 90:
                    moved = hypot(progress[-1][1] - progress[0][1], progress[-1][2] - progress[0][2])
                    if moved < 0.05:   # commanded forward for 3 s without moving
                        world.stop()
                        return 'failed', FailureReason.BLOCKED, replans
                    progress.clear()
            else:
                progress.clear()
            if world.steps % 3 == 0:
                mapper.mark_footprint()
            if world.steps % OBSERVE_EVERY_STEPS == 0:
                mapper.observe()
                self.confirm_target()
                if world.steps % (2 * OBSERVE_EVERY_STEPS) == 0 or mapper.claim:
                    self.frame()
                if mapper.claim:
                    return 'cancelled', None, replans
                safe = mapper.traversable()
                ahead = [p for p in route[index:] if hypot(p[0] - pose.x_m, p[1] - pose.y_m) > 0.3]
                if any(not safe[world_to_cell(x, y)] for x, y in ahead):
                    cost, parent, _ = mapper.plan(goal_cell)
                    if goal_cell not in parent or replans >= 5:
                        world.stop()
                        return 'failed', FailureReason.BLOCKED, replans
                    route, index, replans = mapper.route(parent, goal_cell), 0, replans + 1
                    self.route_drawn = route
        world.stop()
        return 'failed', FailureReason.TIMEOUT, replans

    # ---------- visualisation (human debugging only; never policy input) ----------
    def frame(self):
        if len(self.frames) < 260:
            self.frames.append(self.render())

    def render(self):
        world, mapper = self.world, self.mapper
        canvas = Image.new('RGB', (1120, 620), '#142033')
        draw = ImageDraw.Draw(canvas)
        plan = Image.open(DATA_ROOT / 'g_dataset' / SCENE_ID / 'floorplan.png').convert('RGB')
        crop = (50, 60, 730, 880)   # floor-plan pixels, 0.01 m each, origin at (-5, -5)
        overview = plan.crop(crop)
        od = ImageDraw.Draw(overview)
        to_px = lambda x, y: (x * 100 + 500 - crop[0], y * 100 + 500 - crop[1])
        if self.route_drawn:
            od.line([to_px(x, y) for x, y in self.route_drawn], fill='#2f7df6', width=4)
        if len(world.trajectory) > 1:
            od.line([to_px(x, y) for x, y in world.trajectory], fill='#10b060', width=6)
        tx, ty, _ = world.target_position()
        x, y = to_px(tx, ty)
        od.rectangle((x - 9, y - 9, x + 9, y + 9), outline='#e0301e', width=4)
        pose = world.pose()
        x, y = to_px(pose.x_m, pose.y_m)
        od.ellipse((x - 18, y - 18, x + 18, y + 18), fill='#ff8c32', outline='black', width=2)
        od.line((x, y, x + 32 * cos(pose.yaw_rad), y + 32 * np.sin(pose.yaw_rad)), fill='black', width=5)
        overview = overview.resize((460, 555))
        canvas.paste(overview, (12, 50))
        camera = Image.fromarray(world.last_rgb)
        if world.last_target_bbox:
            x0, y0, x1, y1 = world.last_target_bbox
            cd = ImageDraw.Draw(camera)
            cd.rectangle((x0 - 3, y0 - 3, x1 + 3, y1 + 3), outline='#ff2d2d', width=2)
            cd.text((x0, min(y1 + 5, 228)), TARGET_CLASS + ' (instance seg)', fill='#ff2d2d')
        canvas.paste(camera, (486, 50))
        grid = mapper.cells
        colors = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        colors[grid == -1] = (99, 112, 133)
        colors[grid == 0] = (235, 240, 245)
        colors[grid == 100] = (26, 34, 49)
        if self.last_state is not None:
            for frontier in self.last_state.frontiers:
                color = (255, 60, 200) if self.selected == frontier.frontier_id else (255, 150, 0)
                for cell in frontier.support_cells:
                    colors[cell.row, cell.column] = color
        scale = 4
        occupancy = Image.fromarray(colors).resize((WIDTH * scale, HEIGHT * scale), Image.NEAREST)
        md = ImageDraw.Draw(occupancy)
        to_cell_px = lambda x, y: ((x - ORIGIN_X) / RES * scale, (y - ORIGIN_Y) / RES * scale)
        if len(world.trajectory) > 1:
            md.line([to_cell_px(x, y) for x, y in world.trajectory], fill='#10b060', width=2)
        for track in mapper.tracks.values():
            x, y = to_cell_px(track.position_world.x_m, track.position_world.y_m)
            md.ellipse((x - 3, y - 3, x + 3, y + 3), fill='#2f7df6')
            md.text((x + 4, y - 6), track.class_name, fill='#0b3a8c')
        x, y = to_cell_px(pose.x_m, pose.y_m)
        md.ellipse((x - 4, y - 4, x + 4, y + 4), fill='#ff8c32')
        canvas.paste(occupancy, (820, 50))
        draw.text((12, 14), '%s | %s | %.1f m | %.1f s sim' % (self.name, self.caption, world.distance, world.sim_time), fill='white')
        draw.text((12, 34), 'iGibson Rs scan (floor plan for viewer; red box = hidden target, evaluator only)', fill='#aab4c3')
        draw.text((486, 34), 'Locobot RGB camera (iGibson renderer)', fill='#aab4c3')
        draw.text((820, 34), 'Fused RGB-D map + frontiers', fill='#aab4c3')
        draw.text((486, 300), 'Observed classes:', fill='white')
        for i, name in enumerate(sorted({t.class_name for t in mapper.tracks.values()})):
            draw.text((486 + (i % 2) * 150, 320 + (i // 2) * 18), '- ' + name, fill='#d0e4ff')
        draw.text((486, 596), 'Locobot wheel control | depth mapping | landmark + instance-seg detector', fill='#aab4c3')
        return canvas

    # ---------- evaluator (ground truth; result never returned to the policy) ----------
    def evaluate(self, claim):
        evidence = dict(self.mapper.target_evidence)
        if not set(claim.evidence_observation_ids) <= set(evidence):
            return False, None
        truth = self.world.target_position()
        errors = [hypot(evidence[i][0] - truth[0], evidence[i][1] - truth[1])
                  for i in claim.evidence_observation_ids]
        return max(errors) < 0.5, round(max(errors), 3)

    def run(self):
        world, mapper = self.world, self.mapper
        status, decisions, previous = 'search_limit', 0, None
        visited, failed = {}, {}
        path = self.output / (self.name + '.jsonl')
        with JsonlLogger(path) as log:
            log.write('demo_config', mapper.episode_id, source='igibson_static_scan', scene=SCENE_ID,
                      simulator='iGibson 2.2.2 Simulator (headless) + PyBullet physics',
                      robot='Locobot (DifferentialDriveController, wheel velocity commands)',
                      robot_radius_m=round(world.robot_radius, 3), robot_height_m=round(world.robot_height, 3),
                      perception='RGB-D depth fusion (eyes camera, head tilt -25/0 deg); annotated-landmark oracle gated by '
                                 'rendered depth visibility; target from rendered instance segmentation',
                      target=TARGET_CLASS, target_asset='YCB 003_cracker_box', landmarks=[l[0] for l in LANDMARKS],
                      region_prior=[r[0] for r in REGIONS], initial_clearance_m=INITIAL_CLEARANCE_M,
                      grid_resolution_m=RES, model=self.model if self.llm else None, limits=LIMITS)
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
                                       'estimated position within 0.5 m of ground truth')
                    break
                current = {f.frontier_id for f in state.frontiers}
                exclusions = tuple(
                    FrontierExclusion(fid, reason, count, 'exhausted', None)
                    for fid, (reason, count) in sorted({**visited, **failed}.items()) if fid in current)
                context = build_context(state, evaluation, exclusions, previous)
                choice = decide(self.policy, state, context)
                log.write('decision', mapper.episode_id, state=state, context=context, selection=choice)
                if choice.subgoal is None:
                    status = choice.stop_reason
                    break
                decisions += 1
                if self.llm:
                    log.write('llm_call', mapper.episode_id, record=self.policy.call_records[-1])
                goal = choice.subgoal
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
            self.frame()
            summary = {'source': 'igibson_static_scan', 'scene': SCENE_ID, 'robot': 'Locobot',
                       'policy': self.name, 'termination': status, 'decisions': decisions,
                       'executed_distance_m': round(world.distance, 3), 'sim_time_s': round(world.sim_time, 3),
                       'wall_time_s': round(time.monotonic() - self.started, 3),
                       'collision_steps': world.collision_steps, 'model': self.model if self.llm else None,
                       'llm_calls': self.policy.calls if self.llm else 0,
                       'fallbacks': sum(r['fallback_selected_action'] for r in self.policy.call_records) if self.llm else 0,
                       'observed_classes': sorted(t.class_name for t in mapper.tracks.values()),
                       'known_fraction': round(float((mapper.cells != -1).mean()), 4),
                       'target_detections': len(mapper.target_evidence)}
            log.write('terminal', mapper.episode_id, **summary)
        self.frames[-1].save(self.output / (self.name + '-final.png'))
        small = [f.resize((800, 443), Image.LANCZOS) for f in self.frames]
        small[0].save(self.output / (self.name + '.gif'), save_all=True, append_images=small[1:],
                      duration=[200] * (len(small) - 1) + [2500], loop=0)
        (self.output / (self.name + '-summary.json')).write_text(dumps(summary), encoding='utf-8')
        print(dumps(summary), flush=True)
        return summary


def run_episode(name, output, model=None):
    episode = Episode(name, output, model)
    try:
        return episode.run()
    finally:
        episode.world.close()


def main():
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(errors='replace')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--policy', choices=['nearest', 'llm_guided', 'both'], default='nearest')
    parser.add_argument('--env-file', type=Path, default=Path('../.env'))
    parser.add_argument('--output', type=Path,
                        default=Path('runs') / ('igibson-' + datetime.now().strftime('%Y%m%d-%H%M%S')))
    parser.add_argument('--existing-output', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    load_local_env(args.env_file)
    if args.policy in ('llm_guided', 'both') and not os.environ.get('LLM_MODEL'):
        parser.error('LLM_MODEL must be configured for a live LLM run')
    args.output.mkdir(parents=True, exist_ok=args.existing_output)
    if args.policy == 'both':
        # One iGibson simulator per process: the renderer owns a single GL context.
        codes = [subprocess.call([sys.executable, '-m', __spec__.name, '--policy', name,
                                  '--env-file', str(args.env_file), '--output', str(args.output),
                                  '--existing-output']) for name in ('nearest', 'llm_guided')]
        return int(any(codes))
    summary = run_episode(args.policy, args.output, os.environ.get('LLM_MODEL'))
    print('Artifacts:', args.output.resolve())
    return int(summary['termination'] != 'success')


if __name__ == '__main__':
    raise SystemExit(main())
