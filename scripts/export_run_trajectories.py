"""Save reference-floorplan trajectory PNGs after each recorded episode finishes.

Runs separately from the frozen experiment. Uses only recorded motion samples,
never planned waypoints. Failed and interrupted episodes are exported as well.
"""
import argparse
from datetime import datetime,timezone
import json
from math import cos,hypot,sin
from pathlib import Path
import time

ROOT=Path(__file__).resolve().parents[1]


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def signature(summary,events):
    return [summary.stat().st_mtime_ns,summary.stat().st_size,events.stat().st_size]


def render_episode(run_root,protocol,job,scene_overlay=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import FancyArrowPatch,Polygon
    from PIL import Image

    folder=run_root/'episodes'/job['job_id']
    summary_path=folder/'summary.json'
    events_path=folder/'events.jsonl'
    metadata_path=folder/('trajectory-scene-metadata.json' if scene_overlay else 'trajectory-metadata.json')
    output=folder/('trajectory-scene.png' if scene_overlay else 'trajectory.png')
    if not summary_path.exists() or not events_path.exists():
        return None
    stamp=signature(summary_path,events_path)
    if scene_overlay:
        overlay_stat=scene_overlay.stat()
        stamp.extend((overlay_stat.st_mtime_ns,overlay_stat.st_size))
    if output.exists() and metadata_path.exists() and read(metadata_path).get('input_signature')==stamp:
        return dict(job_id=job['job_id'],image=str(output.relative_to(run_root)),already_saved=True)
    poses=[]
    with events_path.open(encoding='utf-8') as stream:
        for line in stream:
            try:
                event=json.loads(line)
            except ValueError:
                continue
            if event.get('event')=='motion_sample':
                poses.append(event['pose'])
    manifest=read(run_root/job['manifest_path'])
    summary=read(summary_path)
    start=manifest['start_pose']
    points=[(start['x_m'],start['y_m'])]+[(p['x_m'],p['y_m']) for p in poses]
    end=poses[-1] if poses else start
    target=(protocol['targets_evaluator_only'][job['target_location']] if 'targets_evaluator_only' in protocol
            else protocol['target_evaluator_only'])['position_world']
    fig=plt.figure(figsize=(6.6,8.2),dpi=180,facecolor='white')
    ax=fig.add_axes([0,0,1,1])
    ax.imshow(Image.open(ROOT/'data/igibson-data/g_dataset/Rs/floorplan.png'),
              origin='lower',extent=(-5,5,-5,5),interpolation='bilinear')
    if scene_overlay:
        overlay_metadata=read(scene_overlay.with_suffix('.json'))
        ax.imshow(Image.open(scene_overlay),origin='upper',
                  extent=overlay_metadata['floor_extent_xy_m'],interpolation='bilinear')
    ax.set_xlim(-4.4,2.2)
    ax.set_ylim(-4.4,3.8)
    ax.set_aspect('equal')
    ax.axis('off')
    color='#218db5'
    ax.plot([p[0] for p in points],[p[1] for p in points],color=color,lw=1.6,alpha=.96,zorder=3)
    # Accumulated translational distance places arrows uniformly along actual
    # travel, without adding misleading arrows for in-place yaw changes.
    cumulative=[0.]
    for a,b in zip(points,points[1:]):
        cumulative.append(cumulative[-1]+hypot(b[0]-a[0],b[1]-a[1]))
    step=max(.9,cumulative[-1]/28.)
    next_arrow=.55
    index=1
    while next_arrow<cumulative[-1]-.15 and index<len(points):
        while index<len(points)-1 and cumulative[index]<next_arrow:
            index+=1
        lo=index
        while lo>0 and cumulative[index]-cumulative[lo]<.07:
            lo-=1
        hi=index
        while hi<len(points)-1 and cumulative[hi]-cumulative[index]<.07:
            hi+=1
        a,b=points[lo],points[hi]
        length=hypot(b[0]-a[0],b[1]-a[1])
        if length>.025:
            cx,cy=points[index]
            dx,dy=(b[0]-a[0])/length*.09,(b[1]-a[1])/length*.09
            ax.add_patch(FancyArrowPatch((cx-dx,cy-dy),(cx+dx,cy+dy),arrowstyle='-|>',
                mutation_scale=10,linewidth=1.2,color=color,zorder=4))
        next_arrow+=step
    ax.scatter([start['x_m']],[start['y_m']],s=100,c='#2a9f75',edgecolors='white',linewidths=1.6,zorder=7)
    ax.scatter([target[0]],[target[1]],s=280,marker='*',c='#d85b42',edgecolors='white',linewidths=1.7,zorder=7)
    yaw=end['yaw_rad']
    triangle=[]
    for x,y in ((.145,0),(-.10,.085),(-.10,-.085)):
        triangle.append((end['x_m']+x*cos(yaw)-y*sin(yaw),end['y_m']+x*sin(yaw)+y*cos(yaw)))
    ax.add_patch(Polygon(triangle,closed=True,facecolor='#187aa3',edgecolor='white',linewidth=1.6,zorder=8))
    temporary=output.with_name('trajectory-scene.tmp.png' if scene_overlay else 'trajectory.tmp.png')
    fig.savefig(temporary,dpi=180,facecolor='white',pad_inches=0)
    plt.close(fig)
    temporary.replace(output)
    metadata=dict(job_id=job['job_id'],policy=job['policy'],profile=job['profile'],
        environment_seed=job['environment_seed'],termination=summary['termination'],
        partial_metrics=summary.get('partial_metrics',False),
        start_pose=start,last_recorded_pose=end,target_position_world=target,
        motion_samples=len(poses),simulated_time_s=summary.get('simulated_time_s'),
        actual_distance_m=summary.get('actual_distance_m'),input_signature=stamp,
        created_utc=datetime.now(timezone.utc).isoformat(),
        source='Recorded motion_sample events; no planned path, interpolation, or hidden-map navigation.',
        background=('Original Rs scan plus actual iGibson overhead render of inserted objects at t=0; '
                    'the dynamic cart can move later.' if scene_overlay else
                    'Original Rs scan reference only; added furnishings and moving cart are not drawn.'),
        scene_overlay=(str(scene_overlay.relative_to(run_root)) if scene_overlay else None),
        world_bounds=dict(x_min=-4.4,x_max=2.2,y_min=-4.4,y_max=3.8),
        markers=dict(start='green circle',target='red star',end='blue heading triangle',trajectory='blue path with travel-direction arrows'))
    metadata_path.write_text(json.dumps(metadata,ensure_ascii=False,indent=2),encoding='utf-8')
    return dict(job_id=job['job_id'],image=str(output.relative_to(run_root)),already_saved=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-root',type=Path,required=True)
    parser.add_argument('--watch',action='store_true')
    parser.add_argument('--scene-overlays',action='store_true',
                        help='Save trajectory-scene.png using rendered inserted-object layers')
    parser.add_argument('--interval',type=float,default=5.)
    args=parser.parse_args()
    if args.interval<1:
        parser.error('--interval must be at least one second')
    root=args.run_root.resolve()
    protocol=read(root/'protocol.json')
    while True:
        saved=[]
        errors=[]
        for job in protocol['jobs']:
            try:
                overlay=(root/'scene-floorplans'/
                         f"{job['target_location']}-{job['profile']}-start{job['start_id']}-objects.png"
                         if args.scene_overlays else None)
                result=render_episode(root,protocol,job,overlay)
                if result:
                    saved.append(result)
                    if not result['already_saved']:
                        print('Saved',result['image'],flush=True)
            except Exception as error:
                errors.append(dict(job_id=job['job_id'],error=type(error).__name__+': '+str(error)))
        complete=len(saved)==len(protocol['jobs'])
        status=dict(status='complete' if complete else 'watching' if args.watch else 'exported_available_runs',
            saved=len(saved),planned=len(protocol['jobs']),images=saved,errors=errors,
            updated_utc=datetime.now(timezone.utc).isoformat())
        status_name='trajectory-scene-status' if args.scene_overlays else 'trajectory-status'
        temp=root/(status_name+'.tmp.json')
        temp.write_text(json.dumps(status,ensure_ascii=False,indent=2),encoding='utf-8')
        temp.replace(root/(status_name+'.json'))
        if not args.watch or complete or (root/'cancel.request').exists():
            return int(bool(errors))
        time.sleep(args.interval)


if __name__=='__main__':
    raise SystemExit(main())
