"""Run nearest / live Gemini on a procedural PyBullet map and export GIF/JSONL."""
import argparse
from datetime import datetime
import json
from math import atan2, hypot, pi
import os
import sys
from pathlib import Path
import time

from ..config import PolicyConfig
from ..contracts import (FailureReason, PlannerAttempt, PlannerResult, PolicyName, Pose)
from ..experiments.logging import JsonlLogger, dumps
from ..policies.baselines import BaselinePolicy
from ..policies.llm import LLMGuidedPolicy
from ..policies.provider import GeminiCompatibleClient, load_local_env
from ..simulation.bullet_demo import BulletWorld, RangeMapper, DT
from .episode import build_context, decide


def run_episode(name, output, model=None, gui=False, blocked=False, max_decisions=40):
    world = BulletWorld(gui=gui, blocked=blocked)
    mapper = RangeMapper(world, 'bullet-' + name)
    llm = name == 'llm_guided'
    policy = LLMGuidedPolicy(GeminiCompatibleClient()) if llm else BaselinePolicy(PolicyName.NEAREST)
    policy.reset(mapper.episode_id, 0, PolicyConfig(llm_model=model, llm_deadline_s=10.0, llm_call_cap=20))
    frames, previous, status = [], None, 'search_limit'
    decisions, next_capture, next_observation = 0, 0, 0
    started = time.monotonic()
    current = 'initial scan'
    def frame():
        if len(frames) < 150:
            frames.append(world.render(mapper, '%s | %s | %.1fm | %.1fs sim' %
                (name, current, world.distance, world.steps*DT)))
    try:
        with JsonlLogger(output / (name + '.jsonl')) as log:
            log.write('demo_config',mapper.episode_id,source='pybullet_procedural',
                perception='360 degree planar range + rendered visible-class oracle',
                motion='velocity-actuated holonomic cylinder', target='mug',
                robot_radius_m=.22,grid_resolution_m=.25, model=model if llm else None,
                limits={'decisions':max_decisions,'wall_s':240,'sim_s':180})
            mapper.observe()
            frame()
            frames[-1].save(output / (name + '-start.png'))
            for _ in range(max_decisions):
                if time.monotonic()-started > 240 or world.steps*DT > 180:
                    break
                state,evaluation,paths = mapper.snapshot()
                if state.target_claim:
                    # Evaluator-only segmentation evidence; never fed back to selection.
                    valid = set(state.target_claim.evidence_observation_ids) <= world.visible_target_frames
                    status = 'success' if valid else 'false_target_claim'
                    log.write('terminal_evaluation',mapper.episode_id,claim=state.target_claim,
                              valid_target=valid,verifier='rendered segmentation evidence')
                    break
                context = build_context(state,evaluation,previous_attempt=previous)
                choice = decide(policy,state,context)
                log.write('decision',mapper.episode_id,state=state,context=context,selection=choice)
                if choice.subgoal is None:
                    status = choice.stop_reason
                    break
                decisions += 1
                if llm:
                    log.write('llm_call',mapper.episode_id,record=policy.call_records[-1])
                goal = choice.subgoal
                current = '%s -> %s' % (goal.frontier_id, next(f.region_id for f in state.frontiers if f.frontier_id==goal.frontier_id))
                print(name, 'decision', decisions, current, choice.rationale, flush=True)
                frame()
                route = paths[goal.frontier_id]
                route_poses = tuple([state.robot_pose] + [Pose(x,y,0,goal.approach_pose.yaw_rad) for x,y in route])
                cost = next(a.path_cost_m for a in evaluation.assessments if a.frontier_id==goal.frontier_id)
                plan = PlannerResult(True,route_poses,cost,None)
                nav_start, distance_before = world.steps, world.distance
                failure, claimed = None, False
                for route_index,(x,y) in enumerate(route):
                    for _ in range(240):
                        pose = world.pose()
                        dx,dy = x-pose.x_m,y-pose.y_m
                        distance = hypot(dx,dy)
                        if distance < .045:
                            break
                        yaw = atan2(dy,dx)
                        error = (yaw-pose.yaw_rad+pi)%(2*pi)-pi
                        speed = min(.7,distance*4)
                        contact = world.step_velocity(speed*dx/distance,speed*dy/distance,
                                                       max(-2,min(2,error*4)))
                        if gui:
                            time.sleep(DT)
                        if contact:
                            failure = FailureReason.BLOCKED
                            break
                        if world.steps >= next_observation:
                            world.stop()
                            mapper.observe(panoramic=False)
                            next_observation = world.steps + 120
                            if mapper.claim:
                                claimed = True
                                break
                            safe = mapper.traversable()
                            if any(not safe[int(ry/.25),int(rx/.25)]
                                   for rx,ry in route[route_index:]):
                                failure = FailureReason.BLOCKED
                                break
                        if world.steps >= next_capture:
                            frame()
                            next_capture = world.steps + 120
                        if world.steps-nav_start > 3600:
                            failure = FailureReason.TIMEOUT
                            break
                    else:
                        failure = FailureReason.TIMEOUT
                    if failure or claimed:
                        break
                world.stop()
                if not failure and not claimed:
                    # Enforce the selected approach heading on the real body.
                    for _ in range(600):
                        error = (goal.approach_pose.yaw_rad-world.pose().yaw_rad+pi)%(2*pi)-pi
                        if abs(error)<.06:
                            break
                        world.step_velocity(omega=max(-2,min(2,error*4)))
                    world.stop()
                execution = 'cancelled' if claimed else 'failed' if failure else 'reached'
                previous = PlannerAttempt(goal,plan,execution,failure)
                log.write('navigation_feedback',mapper.episode_id,attempt=previous,
                    robot_pose=world.pose(),executed_distance_m=world.distance-distance_before,
                    collision_steps=world.collision_steps,sim_time_s=world.steps*DT)
                if failure:
                    status = 'navigation_' + failure.value
                    break  # bounded demo; production spatial recovery is not implemented
                mapper.observe()
                frame()
            else:
                status = 'search_limit'
            if mapper.claim and status == 'search_limit':
                valid = set(mapper.claim.evidence_observation_ids) <= world.visible_target_frames
                status = 'success' if valid else 'false_target_claim'
            current = status
            frame()
            summary = {'source':'pybullet_procedural','policy':name,'termination':status,
                'decisions':decisions,'executed_distance_m':round(world.distance,3),
                'sim_time_s':round(world.steps*DT,3),'wall_time_s':round(time.monotonic()-started,3),
                'collision_steps':world.collision_steps,'model':model if llm else None,
                'llm_calls':policy.calls if llm else 0,
                'fallbacks':sum(r['fallback_selected_action'] for r in policy.call_records) if llm else 0,
                'observed_classes':sorted(t.class_name for t in mapper.tracks.values()),
                'known_fraction':float((mapper.cells!=-1).mean()),
                'target_seen_frames':len(world.visible_target_frames)}
            log.write('terminal',mapper.episode_id,**summary)
        frames[-1].save(output / (name + '-final.png'))
        frames[0].save(output / (name + '.gif'),save_all=True,append_images=frames[1:],
                        duration=[180]*(len(frames)-1)+[2000],loop=0)
        (output / (name + '-summary.json')).write_text(dumps(summary),encoding='utf-8')
        print(dumps(summary),flush=True)
        return summary
    finally:
        world.stop()
        world.close()


def main():
    # Windows consoles may be GBK; preserve exact model text in UTF-8 JSONL.
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(errors='replace')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--policy',choices=['nearest','llm_guided','both'],default='nearest')
    parser.add_argument('--env-file',type=Path,default=Path('../.env'))
    parser.add_argument('--output',type=Path,default=Path('runs') / ('pybullet-'+datetime.now().strftime('%Y%m%d-%H%M%S')))
    parser.add_argument('--gui',action='store_true',help='Open interactive PyBullet window; real-time playback')
    parser.add_argument('--blocked',action='store_true',help='Close the target room doorway for negative testing')
    args = parser.parse_args()
    load_local_env(args.env_file)
    if args.policy in ('llm_guided','both') and not os.environ.get('LLM_MODEL'):
        parser.error('LLM_MODEL must be configured for a live LLM run')
    args.output.mkdir(parents=True,exist_ok=False)
    summaries = [run_episode(name,args.output,os.environ.get('LLM_MODEL'),args.gui,args.blocked)
                 for name in (['nearest','llm_guided'] if args.policy=='both' else [args.policy])]
    print('Artifacts:',args.output.resolve())
    return int(any(s['termination']!='success' for s in summaries))


if __name__ == '__main__':
    raise SystemExit(main())
