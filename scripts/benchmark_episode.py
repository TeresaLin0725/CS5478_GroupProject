"""One native benchmark episode, or isolated evaluator-only task feasibility check."""
import argparse
from dataclasses import asdict
import hashlib,json,os,sys,time
from enum import Enum
from pathlib import Path

def read(p):return json.loads(Path(p).read_text(encoding='utf-8'))
def navigation_options(protocol):
    """Resolve the frozen B preset; legacy protocols keep their original behavior."""
    preset=protocol.get('navigation_preset','legacy')
    if preset=='legacy':return {}
    if preset=='shared_round':return dict(start_margin_m=0.0)
    if preset=='round_footprint_override':
        from frontier_search.navigation.locobot_robot import ROUND_FOOTPRINT_OVERRIDE_NAV_OPTIONS
        return dict(ROUND_FOOTPRINT_OVERRIDE_NAV_OPTIONS)
    raise ValueError('Unknown navigation preset: '+str(preset))

def validate_shared_geometry(protocol, manifest):
    """Reject a mislabeled new preset instead of silently diverging from A."""
    if protocol.get('navigation_preset')=='shared_round':
        from frontier_search.navigation.locobot_robot import shared_round_robot
        if manifest.robot != shared_round_robot(manifest.robot):
            raise ValueError('shared_round requires the round footprint and 0.01 m manifest clearance')

def preflight_manifest(protocol, profile):
    from dataclasses import replace
    from frontier_search.runner.integrated import manifest_for
    from frontier_search.runner.manifest import load_manifest
    if protocol.get('navigation_preset')=='shared_round':
        job=next(j for j in protocol['jobs'] if j['profile']==profile)
        path=Path(protocol['run_root'])/job['manifest_path']
        if hashlib.sha256(path.read_bytes()).hexdigest()!=job['manifest_sha256']:
            raise ValueError('preflight manifest changed')
        manifest=load_manifest(path)
        validate_shared_geometry(protocol,manifest)
        return replace(manifest,episode_id='oracle-'+profile)
    return manifest_for('oracle-'+profile)
def replace_shared(source,target):
    # Windows readers briefly open files without FILE_SHARE_DELETE. A monitor
    # reading progress must not abort the episode's observation callback.
    for attempt in range(30):
        try:return Path(source).replace(target)
        except PermissionError:
            if attempt==29:raise
            time.sleep(min(.005*(attempt+1),.05))
def atomic(p,value):
    p=Path(p);p.parent.mkdir(parents=True,exist_ok=True)
    def encode(v):
        if isinstance(v,Enum):return v.value
        raise TypeError('Unsupported JSON value: '+type(v).__name__)
    temporary=p.with_suffix(p.suffix+'.tmp');temporary.write_text(json.dumps(value,ensure_ascii=False,indent=2,default=encode),encoding='utf-8');replace_shared(temporary,p)

def preflight(protocol,profile,out):
    import numpy as np
    from math import atan2,hypot,pi
    from PIL import Image
    from frontier_search.simulation.integrated_rs import RetainingAdapter
    from frontier_search.simulation.igibson_adapter import AdapterConfig
    from frontier_search.evaluation.retained import RetainedFrames
    from frontier_search.contracts import OccupancyGrid,Pose
    from frontier_search.navigation.costmap import Costmap
    from frontier_search.navigation.lattice import LatticeSearch
    from frontier_search.fixtures.clearance import clearance_binding,footprint_cells
    m=preflight_manifest(protocol,profile)
    sim=RetainingAdapter(m.robot,m.mapping,AdapterConfig(scene_id='Rs',image_width=640,image_height=480,vertical_fov_deg=75.),RetainedFrames(),profile)
    result=dict(profile=profile,passed=False,environment_seed=m.environment_seed,
        start_pose=asdict(m.start_pose),
        scope='Evaluator-only native geometry and visibility. Never supplied to the search mapper or policy.')
    try:
        sim.reset(m.episode_id,'Rs',m.start_pose,m.environment_seed)
        result['spawn_clearance_cells']=len(sim.verify_spawn_clearance())
        result['start_area']=asdict(sim.verify_start_area(m,'paired-benchmark-preflight'))
        backend=sim._backend;w=backend.world
        if profile=='dynamic':
            result['cart_route']=dict(start=w.cart_motion.start,end=w.cart_motion.end,
                initial_sweep_clear=w.cart_motion.clear_sweep(w.cart_motion.start,w.cart_motion.end))
            assert w.cart_motion.move(1.),'cart cannot park'
            result['temporal_condition']='Wait safely at start until the scheduled cart has parked; oracle route checks the parked state.'
        probe=backend.scene_probe(m.mapping);cfg=m.mapping
        band=clearance_binding(m,footprint_cells(m))
        grid=[]
        for r in range(cfg.height_cells):
            row=[]
            for c in range(cfg.width_cells):
                x=cfg.origin_x_m+c*cfg.resolution_m;y=cfg.origin_y_m+r*cfg.resolution_m
                clear=probe.column_clear(x,y,x+cfg.resolution_m,y+cfg.resolution_m,band['required_z_min_m'],band['required_z_max_m'])
                row.append(0 if clear and probe.floor_supported(x,y,x+cfg.resolution_m,y+cfg.resolution_m) else 100)
            grid.append(tuple(row))
        g=OccupancyGrid('world',1,cfg.resolution_m,cfg.origin_x_m,cfg.origin_y_m,cfg.width_cells,cfg.height_cells,tuple(grid))
        cm=Costmap.from_robot(g,m.robot);cm.prepare();oracle=LatticeSearch(g,m.robot)
        result['oracle_planner']='Evaluator-only oriented full-footprint LatticeSearch; formal search retains the original TeamNavigator.'
        atomic(out/'oracle-grid.json',asdict(g))
        candidates=[(-.75,2.45),(-.85,2.35),(-.65,2.45),(-.75,2.35)]
        for x,y in candidates:
            goal=Pose(x,y,0.,atan2(2.95-y,-1.4-x))
            paths=[oracle.solve(Pose(.8,-2.4,0.,trial['yaw_rad']),{'view':goal},30.)['view'] for trial in protocol['seed_heading_blocks']]
            if not all(p.success for p in paths):continue
            body=next(iter(w.robot_bodies))
            w.p.resetBasePositionAndOrientation(body,[x,y,.01],w.p.getQuaternionFromEuler([0,0,goal.yaw_rad]))
            w.p.resetBaseVelocity(body,[0,0,0],[0,0,0]);w.stop(30)
            obs=sim.capture();frame=sim.truth_store.get_frame(m.episode_id,obs.sequence)
            visible=[v for v in frame.instances if v.visibility>=m.evaluation.minimum_visibility]
            if not visible:continue
            v=visible[0];distance=hypot(obs.robot_pose.x_m-v.position_world.x_m,obs.robot_pose.y_m-v.position_world.y_m)
            if distance>.95:continue
            Image.fromarray(np.frombuffer(obs.color_image.data,dtype=np.uint8).reshape(480,640,3)).save(out/'target-view.png')
            result.update(passed=True,goal=asdict(goal),physical_view_pose=asdict(obs.robot_pose),
                instance=asdict(v),distance_m=distance,paths=[asdict(p) for p in paths],
                free_cells=sum(v==0 for row in grid for v in row))
            break
        if not result['passed']:result['error']='No verified visible close viewpoint connected to all declared start headings.'
    except Exception as exc:
        result['error']=type(exc).__name__+': '+str(exc)
        import traceback;traceback.print_exc()
    finally:sim.close()
    atomic(out/'preflight.json',result)
    print(json.dumps(dict(profile=profile,passed=result['passed'],error=result.get('error'))),flush=True)
    return int(not result['passed'])

def episode(protocol,job,out):
    import random,numpy as np,torch
    from PIL import Image
    from frontier_search.runner.integrated import DisplayDetector,PipelineLog,source_fingerprint
    from frontier_search.runner.manifest import load_manifest
    from frontier_search.runner.episode import EpisodeRunner
    from frontier_search.runner.recovery import SpatialRecovery
    from frontier_search.simulation.igibson_adapter import AdapterConfig
    from frontier_search.simulation.integrated_rs import RetainingAdapter,CloseRetainedEvaluator
    from frontier_search.evaluation.retained import RetainedFrames
    from frontier_search.navigation.navigator import TeamNavigator
    from frontier_search.perception.mapper import RGBDMapper
    from frontier_search.policies.baselines import BaselinePolicy
    from frontier_search.policies.llm import LLMGuidedPolicy
    from frontier_search.policies.provider import GeminiCompatibleClient
    from frontier_search.contracts import PolicyName
    from frontier_search.sensors import ImageBuffer
    fingerprint=source_fingerprint()
    assert fingerprint==protocol['engine_fingerprint'],'frozen engine fingerprint mismatch'
    manifest_path=Path(protocol['run_root'])/job['manifest_path']
    assert hashlib.sha256(manifest_path.read_bytes()).hexdigest()==job['manifest_sha256'],'manifest changed'
    m=load_manifest(manifest_path)
    validate_shared_geometry(protocol,m)
    random.seed(m.perception_seed);np.random.seed(m.perception_seed);torch.manual_seed(m.perception_seed)
    torch.set_num_threads(protocol['runtime']['torch_threads'])
    device='0' if torch.cuda.is_available() else 'cpu'
    assert device==protocol['runtime']['device'],'inference device changed'
    detector=DisplayDetector(str(Path(protocol['snapshot_root'])/'weights/yolo11/yolo11s.pt'),
        target_class='cup',tile_every_frames=3,minimum_confidence=.50,device=device)
    detector.detect(ImageBuffer(640,480,'rgb8',bytes(640*480*3)))
    store=RetainedFrames()
    sim=RetainingAdapter(m.robot,m.mapping,AdapterConfig(scene_id='Rs',settle_steps=0,
        image_width=640,image_height=480,vertical_fov_deg=75.),store,job['profile'])
    mapper=RGBDMapper(detector);nav=TeamNavigator(sim,linear_gain=2.,**navigation_options(protocol))
    policy=LLMGuidedPolicy(GeminiCompatibleClient()) if job['policy']=='llm_guided' else BaselinePolicy(PolicyName(job['policy']))
    evaluator=CloseRetainedEvaluator(store,sim)
    from benchmark_watchdog import WatchdogChannel,AsyncEventWriter
    channel=WatchdogChannel(out,create=True) if protocol.get('watchdog_transport')=='shared_memory_v1' else None
    last_progress={}
    class WatchedLog(PipelineLog):
        log_write_total_s=0.
        log_write_max_s=0.
        ignored={'current_pose','sim_time_s','simulator.step','navigator.on_step','navigator.poll',
                 'capture_marks','stop','pause','resume'}
        def __init__(self,path):
            super().__init__(path)
            self.writer=AsyncEventWriter(super().write) if channel is not None else None

        def close(self):
            try:
                if self.writer is not None:self.writer.close()
            finally:super().close()

        def write(self,event,episode_id,**payload):
            log_started=time.perf_counter()
            if self.writer is not None:self.writer.submit(event,episode_id,**payload)
            else:super().write(event,episode_id,**payload)
            log_elapsed=time.perf_counter()-log_started
            self.log_write_total_s+=log_elapsed
            self.log_write_max_s=max(self.log_write_max_s,log_elapsed)
            if event in ('module_begin','module_end') and payload['module'] not in self.ignored:
                if channel is None:self.file.flush()
                r=self.runner
                progress=dict(job_id=job['job_id'],updated_at=time.time(),
                    pending_module=payload['module'] if event=='module_begin' else None,
                    deadline_at=time.time()+payload['deadline_s'] if event=='module_begin' else None,
                    steps=getattr(r,'steps',0),simulated_time_s=getattr(r,'sim_time',0.),
                    actual_distance_m=getattr(r,'distance',0.),decisions=getattr(r,'decisions',0),
                    attempts=getattr(r,'attempts',0),collisions=sim.collisions,
                    peak_contact_force_n=sim.peak_contact_force_n,
                    longest_contact_run_steps=sim.longest_contact_run_steps,
                    log_enqueue_total_s=self.log_write_total_s,log_enqueue_max_s=self.log_write_max_s,
                    log_write_total_s=self.writer.total_s if self.writer else self.log_write_total_s,
                    log_write_max_s=self.writer.max_s if self.writer else self.log_write_max_s,
                    log_backlog=self.writer.queue.qsize() if self.writer else 0)
                last_progress.update(progress)
                try:
                    if channel is not None:channel.publish(progress)
                    else:atomic(out/'watchdog.json',progress)
                except PermissionError:
                    # Telemetry is best effort. A subsequent update replaces it;
                    # the parent's independent total deadline remains enforced.
                    self.file.write(json.dumps(dict(event='watchdog_write_warning',episode_id=episode_id))+'\n')
    log=WatchedLog(out/'events.jsonl')
    runner=EpisodeRunner(sim,mapper,nav,policy,evaluator,logger=log,
        recovery=SpatialRecovery(m.recovery,locality_radius_m=.6),cancelled=lambda:(Path(protocol['run_root'])/'cancel.request').exists())
    log.runner=runner
    try:result=runner.run(m)
    finally:
        try:log.close()
        finally:
            if channel is not None:channel.close()
    # Final disk export happens after the robot has stopped, outside operation clocks.
    if last_progress:atomic(out/'watchdog.json',last_progress)
    calls=getattr(policy,'call_records',[])
    summary=dict(runner.summary,**job,recorded=False,source_fingerprint=fingerprint,
        source_unchanged=fingerprint==source_fingerprint(),evaluation=evaluator.metrics,
        llm_calls=len(calls),llm_fallbacks=sum(bool(c.get('fallback_reason')) for c in calls),
        detector_runtime=dict(device=device,torch_version=torch.__version__,cuda_version=torch.version.cuda,
            gpu_name=torch.cuda.get_device_name(0) if device=='0' else None),
        navigation_preset=protocol.get('navigation_preset','legacy'),
        navigator_events=nav.events,scene_events=getattr(getattr(sim._backend,'world',None),'environment_events',[]))
    if runner.state:
        atomic(out/'final-state.json',asdict(runner.state))
        if runner.state.target_claim:
            bundle=mapper.get_claim_evidence(m.episode_id,runner.state.target_claim.claim_id)
            atomic(out/'evaluation-only.json',dict(evidence=asdict(bundle),
                frames=[asdict(store.get_frame(m.episode_id,d.capture_sequence)) for d in bundle.detections],
                motion={d.capture_sequence:dict(sim.capture_motion[d.capture_sequence],pose=asdict(sim.capture_motion[d.capture_sequence]['pose'])) for d in bundle.detections}))
    Image.fromarray(detector.last_rgb).save(out/'last-processed-rgb.png')
    atomic(out/'llm-calls.json',calls)
    atomic(out/'summary.json',summary)
    return int(result.value!='success')

def main():
    p=argparse.ArgumentParser();p.add_argument('--protocol',type=Path,required=True)
    p.add_argument('--job');p.add_argument('--preflight',choices=('apartment','clutter','dynamic'))
    args=p.parse_args();protocol=read(args.protocol)
    os.environ['GIBSON_ASSETS_PATH']=protocol['asset_paths']['robot_assets']
    os.environ['GIBSON_DATASET_PATH']=protocol['asset_paths']['scenes']
    os.environ['YOLO_CONFIG_DIR']=str(Path(protocol['run_root'])/'yolo-config')
    from frontier_search.policies.provider import load_local_env
    load_local_env(Path(protocol['env_file']))
    if args.preflight:
        out=Path(protocol['run_root'])/'preflight'/args.preflight;out.mkdir(parents=True,exist_ok=True)
        return preflight(protocol,args.preflight,out)
    job=next(j for j in protocol['jobs'] if j['job_id']==args.job)
    out=Path(protocol['run_root'])/'episodes'/job['job_id'];out.mkdir(parents=True,exist_ok=True)
    if (out/'events.jsonl').exists():raise FileExistsError('Never overwrite or retry a benchmark episode')
    return episode(protocol,job,out)

if __name__=='__main__':raise SystemExit(main())
