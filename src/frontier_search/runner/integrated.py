"""Full A/B/C/W3 native pipeline. Validation runs do not create a video recorder."""
import argparse
from dataclasses import asdict, replace
import hashlib
import json
import os
from pathlib import Path
from math import pi
from time import monotonic

from ..config import (MappingConfig,ObservationConfig,NavigationConfig,PolicyConfig,
                      EpisodeLimits,RecoveryConfig)
from ..contracts import NavigatorInit,Point2,Pose,RobotConfig,PolicyName,RegionPrior
from ..evaluation.retained import RetainedFrames
from ..experiments.logging import dumps
from ..navigation.locobot_robot import shared_round_robot
from ..navigation.navigator import RelaxedRetry,TeamNavigator
from ..perception.mapper import RGBDMapper
from ..perception.detection import TiledYOLODetector
from ..policies.baselines import BaselinePolicy
from ..policies.llm import LLMGuidedPolicy
from ..policies.provider import CANONICAL_LLM_MODEL,GeminiCompatibleClient,load_local_env
from ..simulation.igibson_adapter import AdapterConfig
from ..simulation.integrated_rs import RetainingAdapter,CloseRetainedEvaluator,RsTargetSpec
from ..simulation.sensor_noise import add_noise_arguments,noise_from_arguments
from ..simulation.igibson_rs import REGIONS
from ..simulation.rs_tasks import LEGACY_CUP,RS_TARGETS,bowl_start_pose
from .manifest import EpisodeManifest
from .episode import EpisodeRunner
from .recovery import SpatialRecovery

ROOT = Path(__file__).resolve().parents[3]


class DisplayDetector(TiledYOLODetector):
    def detect(self,image):
        import numpy as np
        detections=super().detect(image)
        self.last_rgb=np.frombuffer(image.data,dtype=np.uint8).reshape(image.height,image.width,3).copy()
        self.last_detections=detections
        return detections


def manifest_for(episode_id, policy='information_gain'):
    """Legacy manifest factory retained for historical protocols and fixtures."""
    robot = RobotConfig(tuple(Point2(x,y) for x,y in ((-.182,-.182),(.182,-.182),(.182,.182),(-.182,.182))),
        .0,.908,.02,'differential_drive',True,.32,1.1,.03,.10)
    mapping = MappingConfig(resolution_m=.1,width_cells=70,height_cells=85,origin_x_m=-4.5,origin_y_m=-4.5,
        floor_tolerance_m=.04,base_elevation_margin_m=.015,confidence_threshold=.50,
        confirmation_window_s=3.,depth_pixel_stride=4,frontier_cluster_span_m=1.,
        dynamic_clear_frames=2,inspection_distance_m=.90,inspection_angular_speed_radps=.02,
        gain_ray_step_rad=.15,target_max_extent_m=.25)
    return EpisodeManifest(episode_id,'Rs','igibson','cup',Pose(.8,-2.4,0.,pi/2),robot,
        policy_name=policy,mapping=mapping,
        observation=ObservationConfig(allow_full_rotation=True,dwell_s=.6,inspection_hold_s=4.,
            arrival_headings_rad=(0.,pi/2,pi,-pi/2,0.),recovery_headings_rad=(pi/2,pi,-pi/2,0.),
            operation_deadline_s=400.,drain_deadline_s=30.),
        navigation=NavigationConfig(batch_deadline_s=5.,navigation_deadline_s=400.,max_replans=4,
                                    map_freshness_s=.4),
        policy=PolicyConfig(llm_model=os.environ.get('LLM_MODEL',CANONICAL_LLM_MODEL),
                            llm_deadline_s=30.,llm_call_cap=30),
        recovery=RecoveryConfig(opportunity_attempt_cap=6,max_cycles_without_navigation=3),
        limits=EpisodeLimits(max_attempts=45,max_steps=18000,max_simulated_time_s=600.,
                             max_wall_time_s=2400.,max_decisions=80),
        region_priors=tuple(RegionPrior(name,tuple(Point2(x,y) for x,y in ((x0,y0),(x1,y0),(x1,y1),(x0,y1))),None)
                            for name,(x0,y0,x1,y1) in REGIONS),
        capture_every_steps=10,module_deadline_s=60.,enable_target_evaluation=True,
        initial_clearance_mode='verified_start_area',initial_clearance_radius_m=.65)


def shared_round_manifest_for(episode_id, policy='information_gain'):
    """New native runs share B's existing geometry across mapping and verification."""
    manifest=manifest_for(episode_id,policy)
    return replace(manifest,robot=shared_round_robot(manifest.robot))


def with_robust_mapping(manifest):
    """The robustness configuration validated by the 35/36 noisy suite and used by bowl48."""
    return replace(manifest,
        mapping=replace(manifest.mapping,depth_filter_kernel_size=5,
                        unvisited_recovery_viewpoints=True),
        observation=replace(manifest.observation,inspection_heading_settle_ratio=.5))


def task_manifest_for(episode_id, policy='information_gain', task='legacy_cup'):
    manifest=shared_round_manifest_for(episode_id,policy)
    if task not in RS_TARGETS:
        raise ValueError('Unknown Rs task: '+str(task))
    if task in ('kitchen_bowl','bathroom_bowl'):
        return replace(with_robust_mapping(manifest),target_class='bowl',start_pose=bowl_start_pose())
    return manifest


class PipelineLog:
    def __init__(self,path):
        self.file = path.open('w',encoding='utf-8')
        self.runner = None
        self.recorder = None
        self.previous_progress = -10.

    def write(self,event,episode_id,**payload):
        # Motion samples and module boundaries stay small; full state is logged at decisions.
        self.file.write(dumps(dict(event=event,episode_id=episode_id,**payload))+'\n')
        if event in ('pipeline_error','target_evaluation','episode_terminal','decision','operation_terminal'):
            self.file.flush()
            print(event, dumps(payload.get('selection',payload.get('feedback',payload))),flush=True)
        if event=='capture_processed' and payload['simulated_time_s']-self.previous_progress>=10:
            self.previous_progress=payload['simulated_time_s']
            print('capture',payload['sequence'],'sim',round(payload['simulated_time_s'],1),
                  'known',payload['known_cells'],'frontiers',len(self.runner.state.frontiers),
                  'objects',sorted({o.class_name for o in self.runner.state.semantic_objects}),flush=True)
        if self.recorder:
            self.recorder.on_event(event,payload)

    def close(self):
        self.file.close()


def source_fingerprint():
    digest=hashlib.sha256()
    for path in sorted((ROOT/'src/frontier_search').rglob('*.py')):
        digest.update(str(path.relative_to(ROOT)).encode());digest.update(path.read_bytes())
    digest.update((ROOT/'weights/yolo11/yolo11s.pt').read_bytes())
    return digest.hexdigest()


def runtime_configuration(manifest, profile, relaxed_retry, target=LEGACY_CUP):
    """Normalize the actual C recovery and B navigation settings before launch."""
    navigator=TeamNavigator(None,linear_gain=2.0,start_margin_m=0.0,
                            relaxed_retry=relaxed_retry)
    navigator.reset(NavigatorInit(manifest.episode_id,manifest.robot,
                                  manifest.observation,manifest.navigation))
    return dict(schema_version='c-navigation-recovery-v1',profile=profile,
        policy=manifest.policy_name,recovery=asdict(manifest.recovery),
        policy_config=asdict(manifest.policy),navigation=navigator.effective_configuration(),
        sensor_noise=asdict(manifest.sensor_noise),target=asdict(target),
        start_pose=asdict(manifest.start_pose),observation=asdict(manifest.observation))


def recording_validation_matches(report,fingerprint,profile,policy,configuration):
    """Fail closed for old certificates or any effective runtime mismatch."""
    normalized=json.loads(json.dumps(configuration,sort_keys=True))
    return (report.get('passed') is True and report.get('source_fingerprint')==fingerprint
            and report.get('profile')==profile and report.get('policy')==policy
            and report.get('runtime_configuration')==normalized)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--profile',choices=('apartment','clutter','dynamic'),default=None)
    parser.add_argument('--task',choices=tuple(RS_TARGETS),default='legacy_cup')
    parser.add_argument('--policy',choices=('nearest','information_gain','random','llm_guided'),default='information_gain')
    parser.add_argument('--env-file',type=Path,default=ROOT.parent/'.env')
    parser.add_argument('--record',action='store_true')
    # Opt-in recovery from over-conservative exclusions; all default off, so a plain run is unchanged.
    parser.add_argument('--relaxed-retry',action='store_true',help='B: one relaxed re-plan after a strict NO_PATH/INVALID_GOAL')
    parser.add_argument('--epsilon-retry',type=float,default=0.,help='C: probability of probing an excluded frontier per decision')
    parser.add_argument('--stuck-retry',action='store_true',help='C: probe an excluded frontier instead of ending when stuck')
    parser.add_argument('--validation',type=Path)
    # Custom target (semantic experiments): giving --target-model overrides the --task target
    # while keeping that task's manifest. Without it, the registered --task target is used.
    parser.add_argument('--target-class')
    parser.add_argument('--target-model')
    parser.add_argument('--target-position',type=float,nargs=3,metavar=('X','Y','Z'))
    parser.add_argument('--target-roll',type=float,default=0.)
    parser.add_argument('--target-pitch',type=float,default=0.)
    parser.add_argument('--target-yaw',type=float,default=0.)
    parser.add_argument('--target-instance-id',default='Rs-inserted-target')
    parser.add_argument('--start-pose',type=float,nargs=3,metavar=('X','Y','YAW'))
    parser.add_argument('--environment-seed',type=int)
    parser.add_argument('--perception-seed',type=int)
    parser.add_argument('--policy-seed',type=int)
    parser.add_argument('--llm-prompt-version',choices=('v1','v2'),default='v1',
                        help='System prompt variant for llm_guided (v1 = original wording)')
    parser.add_argument('--llm-deadline-s',type=float,
                        help='Override the per-decision LLM deadline (wall clock; simulation is paused)')
    parser.add_argument('--robust-mapping',action='store_true',
                        help='Apply the bowl48 robustness settings (5x5 depth filter, unvisited '
                             'recovery viewpoints, 0.5 inspection settling) to any task')
    parser.add_argument('--initialization-deadline-s',type=float,
                        help='Initialization watchdog deadline (bowl48 protocol used 240 s)')
    add_noise_arguments(parser)
    args=parser.parse_args()
    if args.profile is None:
        args.profile='apartment' if args.task in ('kitchen_bowl','bathroom_bowl') else 'dynamic'
    try:
        noise=noise_from_arguments(args)
    except (ValueError, OSError, TypeError) as exc:
        parser.error(str(exc))
    load_local_env(args.env_file)
    fingerprint=source_fingerprint()
    try:
        manifest=task_manifest_for(args.output.name,args.policy,args.task)
    except ValueError as exc:
        parser.error(str(exc))
    if args.robust_mapping:
        manifest=with_robust_mapping(manifest)
    if args.initialization_deadline_s is not None:
        manifest=replace(manifest,initialization_deadline_s=args.initialization_deadline_s)
    if args.target_model is not None:
        if args.target_class is None or args.target_position is None:
            parser.error('--target-model requires --target-class and --target-position')
        try:
            target=RsTargetSpec(args.target_class,args.target_model,tuple(args.target_position),
                                args.target_yaw,args.target_instance_id,
                                args.target_roll,args.target_pitch).validate()
        except ValueError as exc:
            parser.error(str(exc))
    else:
        if args.target_class is not None or args.target_position is not None:
            parser.error('--target-class/--target-position require --target-model')
        target=RS_TARGETS[args.task]
    start=(Pose(args.start_pose[0],args.start_pose[1],0.,args.start_pose[2])
           if args.start_pose is not None else manifest.start_pose)
    seed_overrides={name:getattr(args,name) for name in
                    ('environment_seed','perception_seed','policy_seed')
                    if getattr(args,name) is not None}
    manifest=replace(manifest,target_class=target.class_name,start_pose=start,sensor_noise=noise,
        recovery=replace(manifest.recovery,epsilon_retry=args.epsilon_retry,
                         stuck_retry=args.stuck_retry),
        policy=replace(manifest.policy,llm_prompt_version=args.llm_prompt_version,
                       **({'llm_deadline_s':args.llm_deadline_s} if args.llm_deadline_s else {})),**seed_overrides)
    manifest.validate()
    relaxed_retry=RelaxedRetry() if args.relaxed_retry else None
    effective_runtime=runtime_configuration(manifest,args.profile,relaxed_retry,target)
    if args.record:
        if not args.validation:
            parser.error('--record requires a successful validation report')
        report=json.loads(args.validation.read_text(encoding='utf-8'))
        if not recording_validation_matches(report,fingerprint,args.profile,args.policy,effective_runtime):
            parser.error('validation missing, failed, stale or has different recovery/navigation/noise settings')
    args.output.mkdir(parents=True,exist_ok=True)
    if (args.output/'events.jsonl').exists():
        parser.error('choose a fresh output directory; prior runs are retained')
    import torch
    torch.set_num_threads(4)
    device='0' if torch.cuda.is_available() else 'cpu'
    detector_runtime=dict(device=device,torch_version=torch.__version__,cuda_version=torch.version.cuda,
        gpu_name=torch.cuda.get_device_name(0) if device=='0' else None)
    if args.record and report['dry_run_summary'].get('detector_runtime')!=detector_runtime:
        parser.error('detector runtime changed; validate this inference backend before recording')
    print('detector_runtime',json.dumps(detector_runtime),flush=True)
    print('sensor_noise',json.dumps(asdict(noise)),flush=True)
    os.environ.setdefault('YOLO_CONFIG_DIR',str(ROOT/'data/yolo-config'))
    detector=DisplayDetector(str(ROOT/'weights/yolo11/yolo11s.pt'),target_class=manifest.target_class,tile_every_frames=3,
                             minimum_confidence=.50,device=device)
    # Warm imports/model kernels outside the episode's module budgets.
    from ..sensors import ImageBuffer
    detector.detect(ImageBuffer(640,480,'rgb8',bytes(640*480*3)))
    (args.output/'manifest.json').write_text(dumps(manifest),encoding='utf-8')
    (args.output/'controller.json').write_text(json.dumps(dict(linear_gain_per_s=2.0,
        path_smoothing=False,control_hz=30,odometry_source='latest control-step known-pose difference'),indent=2),encoding='utf-8')
    store=RetainedFrames()
    sim=RetainingAdapter(manifest.robot,manifest.mapping,
        AdapterConfig(scene_id='Rs',settle_steps=0,image_width=640,image_height=480,vertical_fov_deg=75.),store,args.profile,target)
    mapper=RGBDMapper(detector)
    # Grid waypoints are only 0.1 m apart. The default 1/s gain made the
    # robot crawl below its speed limit even on a straight, validated path.
    # A, B and spawn verification read the manifest; only B's initial escape margin differs.
    nav=TeamNavigator(sim,linear_gain=2.0,start_margin_m=0.0,
                      relaxed_retry=relaxed_retry)
    policy=(LLMGuidedPolicy(GeminiCompatibleClient()) if args.policy=='llm_guided'
            else BaselinePolicy(PolicyName(args.policy)))
    evaluator=CloseRetainedEvaluator(store,sim,target_class=manifest.target_class)
    log=PipelineLog(args.output/'events.jsonl')
    runner=EpisodeRunner(sim,mapper,nav,policy,evaluator,logger=log,
                         recovery=SpatialRecovery(manifest.recovery,locality_radius_m=.6),
                         cancelled=lambda:(args.output/'cancel.request').exists())
    log.runner=runner
    if args.record:
        from .integrated_recording import IntegratedRecorder
        log.recorder=IntegratedRecorder(args.output,runner,sim,detector)
        sim.on_close=log.recorder.close
    result=runner.run(manifest)
    log.close()
    llm_records=getattr(policy,'call_records',())
    (args.output/'llm-calls.json').write_text(dumps(list(llm_records)),encoding='utf-8')
    summary=dict(runner.summary,profile=args.profile,policy=args.policy,task=args.task,
        custom_target=args.target_model is not None,
        environment_seed=manifest.environment_seed,perception_seed=manifest.perception_seed,
        policy_seed=manifest.policy_seed,start_pose=asdict(manifest.start_pose),
        llm_attempts=len(llm_records),
        llm_calls=sum(bool(r.get('model_called',
            r.get('fallback_reason') not in ('unconfigured_client','call_cap'))) for r in llm_records),
        llm_fallbacks=sum(bool(r.get('fallback_reason')) for r in llm_records),
        source_fingerprint=fingerprint,source_unchanged=fingerprint==source_fingerprint(),evaluation=evaluator.metrics,
        detector_runtime=detector_runtime,
        runtime_configuration=effective_runtime,target=asdict(target),
        scene_events=getattr(getattr(getattr(sim,'_backend',None),'world',None),'environment_events',[]),
        navigator_events=nav.events,recorded=args.record,
        components=dict(scene='Gibson Rs',robot='native Locobot',sensor='native calibrated eyes RGB-D',
            perception='shared RGBDMapper + official YOLO11s (overlapping RGB target tiles)',navigation='shared TeamNavigator',
            runner='shared EpisodeRunner + SpatialRecovery',evaluator='independent retained-instance W3 + proximity'),
        detector_sha256=hashlib.sha256((ROOT/'weights/yolo11/yolo11s.pt').read_bytes()).hexdigest())
    (args.output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    if runner.state:
        (args.output/'final-state.json').write_text(dumps(runner.state),encoding='utf-8')
        if runner.state.target_claim:
            bundle=mapper.get_claim_evidence(manifest.episode_id,runner.state.target_claim.claim_id)
            truth=[store.get_frame(manifest.episode_id,item.capture_sequence) for item in bundle.detections]
            (args.output/'evaluation-only.json').write_text(dumps(dict(frames=truth,
                motion={item.capture_sequence:sim.capture_motion[item.capture_sequence] for item in bundle.detections})),encoding='utf-8')
    from PIL import Image
    Image.fromarray(detector.last_rgb).save(args.output/'last-processed-rgb.png')
    if args.record:
        from .integrated_recording import export_viewer
        export_viewer(args.output,summary)
    return int(result.value!='success')


if __name__=='__main__':
    raise SystemExit(main())
