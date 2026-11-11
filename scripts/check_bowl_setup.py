"""Native bowl setup check: fixed start and a full initial scan, no search dispatch.

Stops after the initial observation operation. Does not call a search policy or
LLM. Actual spawn/start-area checks and the ordinary navigation turn guard run.
"""
import argparse
from dataclasses import asdict,replace
import json
import os
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--start-pose',type=float,nargs=4,metavar=('X','Y','Z','YAW'))
    parser.add_argument('--target-position',type=float,nargs=3,metavar=('X','Y','Z'))
    parser.add_argument('--profile',choices=('apartment','clutter','dynamic'),default='apartment')
    from frontier_search.simulation.sensor_noise import add_noise_arguments,noise_from_arguments
    add_noise_arguments(parser)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    os.environ.setdefault('YOLO_CONFIG_DIR',str(ROOT/'data/yolo-config'))
    import numpy as np
    from PIL import Image
    import torch
    from frontier_search.contracts import PolicyName,Pose
    from frontier_search.evaluation.retained import RetainedFrames
    from frontier_search.experiments.logging import dumps
    from frontier_search.navigation.navigator import TeamNavigator
    from frontier_search.perception.mapper import RGBDMapper
    from frontier_search.policies.baselines import BaselinePolicy
    from frontier_search.runner.integrated import DisplayDetector,task_manifest_for,source_fingerprint
    from frontier_search.runner.episode import EpisodeRunner
    from frontier_search.runner.recovery import SpatialRecovery
    from frontier_search.simulation.igibson_adapter import AdapterConfig
    from frontier_search.simulation.integrated_rs import RetainingAdapter,CloseRetainedEvaluator
    from frontier_search.simulation.rs_tasks import KITCHEN_BOWL
    from frontier_search.sensors import ImageBuffer

    torch.set_num_threads(4)
    device='0' if torch.cuda.is_available() else 'cpu'
    target=replace(KITCHEN_BOWL,position=tuple(args.target_position)) if args.target_position else KITCHEN_BOWL
    noise=noise_from_arguments(args)
    report=dict(source_fingerprint=source_fingerprint(),device=device,
        scope='Native placement, spawn, and initial observation only; no search policy decisions or navigation to target.',
        target=asdict(target),profile=args.profile,noise=asdict(noise),cases=[])
    for heading in ('fixed',):
        out=args.output/heading
        out.mkdir()
        manifest=task_manifest_for('bowl-check-'+heading,'nearest','kitchen_bowl')
        manifest=replace(manifest,sensor_noise=noise)
        if args.start_pose:
            manifest=replace(manifest,start_pose=Pose(*args.start_pose))
        manifest.validate()
        (out/'manifest.json').write_text(dumps(manifest),encoding='utf-8')
        store=RetainedFrames()
        sim=RetainingAdapter(manifest.robot,manifest.mapping,
            AdapterConfig(scene_id='Rs',settle_steps=0,image_width=640,image_height=480,vertical_fov_deg=75.),
            store,args.profile,target)
        detector=DisplayDetector(str(ROOT/'weights/yolo11/yolo11s.pt'),target_class='bowl',
                                 minimum_confidence=.5,tile_every_frames=3,device=device)
        detector.detect(ImageBuffer(640,480,'rgb8',bytes(640*480*3)))
        mapper=RGBDMapper(detector)
        nav=TeamNavigator(sim,linear_gain=2.,start_margin_m=0.)
        signals=dict(done=False,initial_operation=None,start_area=None,visible_frames=[],captures=0,
                     initial_scan_sequence=None,scan_frames=[])
        events=(out/'events.jsonl').open('w',encoding='utf-8')

        class SetupLog:
            def write(self,event,episode_id,**payload):
                events.write(dumps(dict(event=event,episode_id=episode_id,**payload))+'\n')
                if event=='verified_start_area':
                    signals['start_area']=payload
                if event=='operation_terminal' and payload.get('kind')=='observe':
                    signals['initial_operation']=payload
                    signals['done']=True
                    signals['initial_scan_sequence']=signals['captures']
                if event=='capture_processed':
                    seq=payload['sequence']
                    signals['captures']=seq
                    frame=store.get_frame(episode_id,seq)
                    if frame.instances:
                        signals['visible_frames'].append(seq)
                    backend=sim._backend
                    seg=backend.cached[2]
                    ids=np.rint(seg[:,:,0]*1024).astype(int).clip(0,1023)
                    target_pixels=int(np.count_nonzero(backend.world.sim.renderer.instance_id_to_pb_id[ids]==backend.world.target_body))
                    filename='scan-%03d.png'%seq
                    Image.fromarray(detector.last_rgb).save(out/filename)
                    signals['scan_frames'].append(dict(sequence=seq,simulated_time_s=payload['simulated_time_s'],
                        target_pixels=target_pixels,image=filename))
                    if seq==1:
                        Image.fromarray(detector.last_rgb).save(out/'initial-rgb.png')
                        # Support and silhouette are evaluator-only diagnostics.
                        w=sim._backend.world
                        p=w.p
                        lower,upper=map(np.array,p.getAABB(w.target_body))
                        xs=np.linspace(lower[0]+.002,upper[0]-.002,3)
                        ys=np.linspace(lower[1]+.002,upper[1]-.002,3)
                        starts=[[float(x),float(y),1.55] for x in xs for y in ys]
                        # Move only the target for setup-only support rays. Static
                        # scene collision groups differ across Bullet versions.
                        target_position,target_orientation=p.getBasePositionAndOrientation(w.target_body)
                        p.resetBasePositionAndOrientation(w.target_body,[100.,100.,100.],target_orientation)
                        try:
                            hits=p.rayTestBatch(starts,[[x,y,.05] for x,y,_ in starts])
                        finally:
                            p.resetBasePositionAndOrientation(w.target_body,target_position,target_orientation)
                        signals['support']=dict(all_nine_hit_scan=all(h[0]==w.scene.mesh_body_id for h in hits),
                            heights_m=[float(h[3][2]) for h in hits],target_pose=list(w.target_position()))
                        rgb,seg=w.sim.renderer.render_robot_cameras(modes=('rgb','ins_seg'))
                        ids=np.rint(seg[:,:,0]*1024).astype(int).clip(0,1023)
                        signals['initial_target_pixels']=int(np.count_nonzero(w.sim.renderer.instance_id_to_pb_id[ids]==w.target_body))
                    if signals['done']:
                        Image.fromarray(detector.last_rgb).save(out/'after-initial-scan-rgb.png')

        # A guard makes an accidental search dispatch a hard error.
        class NoSearchPolicy(BaselinePolicy):
            def select(self,*args,**kwargs):
                raise RuntimeError('Setup check must stop before policy selection')

        runner=EpisodeRunner(sim,mapper,nav,NoSearchPolicy(PolicyName.NEAREST),
            CloseRetainedEvaluator(store,sim,target_class='bowl'),logger=SetupLog(),
            recovery=SpatialRecovery(manifest.recovery,locality_radius_m=.6),cancelled=lambda:signals['done'])
        try:
            runner.run(manifest)
        finally:
            events.close()
        if runner.state:
            (out/'after-initial-scan-state.json').write_text(dumps(runner.state),encoding='utf-8')
            Image.fromarray(detector.last_rgb).save(out/'after-initial-scan-rgb.png')
        operation=signals['initial_operation'] or {}
        feedback=operation.get('feedback')
        initial_status=getattr(getattr(feedback,'status',None),'value',None)
        support=signals.get('support',{})
        heights=support.get('heights_m',[0.])
        passed=bool(signals['start_area'] and initial_status=='reached' and support.get('all_nine_hit_scan')
                    and max(heights)-min(heights)<.015 and not signals['visible_frames']
                    and runner.summary['decisions']==0 and runner.summary['collisions']==0
                    and runner.summary['termination']=='externally_cancelled')
        result=dict(heading=heading,passed=passed,start_pose=asdict(manifest.start_pose),
            observation=asdict(manifest.observation),summary=runner.summary,
            checks=signals,initial_scan_status=initial_status,
            maximum_target_pixels=max((f['target_pixels'] for f in signals['scan_frames']),default=0),
            observed_classes=sorted({o.class_name for o in runner.state.semantic_objects}) if runner.state else [],
            frontiers=len(runner.state.frontiers) if runner.state else 0)
        (out/'setup.json').write_text(dumps(result),encoding='utf-8')
        report['cases'].append(result)
        print(heading,'passed',passed,'initial_scan',initial_status,'summary',runner.summary,flush=True)
    report['source_unchanged']=source_fingerprint()==report['source_fingerprint']
    report['passed']=all(c['passed'] for c in report['cases']) and report['source_unchanged']
    (args.output/'report.json').write_text(dumps(report),encoding='utf-8')
    return int(not report['passed'])


if __name__=='__main__':
    raise SystemExit(main())
