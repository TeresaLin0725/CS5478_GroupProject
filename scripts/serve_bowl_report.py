"""Read-only local live report for a running noisy bowl pilot."""
import argparse
from datetime import datetime,timezone
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
import json
from pathlib import Path
import threading
import time
from urllib.parse import urlsplit,parse_qs

ROOT=Path(__file__).resolve().parents[1]
POLICIES=('nearest','information_gain','random','llm_guided')


def read(path,default=None):
    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except (OSError,ValueError):
        return default


class Episode:
    def __init__(self,folder):
        self.folder=folder
        self.offset=0
        self.buffer=b''
        self.motion={}
        self.capture={}
        self.state=None
        self.decisions=0
        self.goal=None
        self.rationale=''
        self.operation='Starting'
        self.route=[]
        self.timeline=[]
        self.llm_calls=0
        self.llm_fallbacks=0
        self.target_seen=False
        self.summary=None

    def update(self):
        path=self.folder/'events.jsonl'
        if not path.exists():
            return
        with path.open('rb') as stream:
            stream.seek(self.offset)
            data=stream.read()
            self.offset=stream.tell()
        parts=(self.buffer+data).split(b'\n')
        self.buffer=parts.pop()
        for line in parts:
            try:
                event=json.loads(line)
            except ValueError:
                continue
            kind=event['event']
            text=None
            if kind=='motion_sample':
                self.motion=event
                if event['steps']%3==0:
                    self.route.append([event['pose']['x_m'],event['pose']['y_m']])
            elif kind=='capture_processed':
                self.capture=event
            elif kind=='decision':
                self.decisions+=1
                self.state=event['state']
                self.goal=event['selection'].get('subgoal')
                self.rationale=event['selection'].get('rationale','')
                self.target_seen=self.target_seen or any(o['class_name']=='bowl' for o in self.state['semantic_objects'])
                text='Selected '+self.goal['frontier_id'] if self.goal else 'No search goal selected'
            elif kind=='observation_start':
                self.operation='Observing · '+event['request']['routine']
            elif kind=='dispatch':
                self.operation='Navigating'
            elif kind=='operation_terminal':
                feedback=event['feedback']
                self.operation='Observation complete' if event['kind']=='observe' else 'Goal reached'
                if feedback['status']!='reached':
                    self.operation='Recovering'
                    text='Navigation: '+str(feedback.get('failure_reason') or feedback['status'])
            elif kind=='llm_call':
                record=event['record']
                self.llm_calls+=int(bool(record.get('model_called',True)))
                self.llm_fallbacks+=int(bool(record.get('fallback_reason')))
                text='LLM response: '+str(record.get('response') or record.get('fallback_reason'))
            elif kind=='target_evaluation':
                text='Independent target evaluation received'
            elif kind in ('pipeline_error','cleanup_error'):
                text=kind+': '+str(event.get('error_type','error'))
            elif kind=='episode_terminal':
                self.operation=event['termination']
                text='Finished: '+event['termination']
            if text:
                self.timeline.append(dict(sim_s=event.get('simulated_time_s',self.motion.get('simulated_time_s',0)),text=text))
                self.timeline=self.timeline[-12:]

    def view(self,policy,active,llm_progress):
        self.update()
        if self.summary is None:
            self.summary=read(self.folder/'summary.json')
        summary=self.summary
        path=self.folder/'events.jsonl'
        started=path.exists()
        status=summary['termination'] if summary else ('running' if started else 'queued')
        if policy=='llm_guided' and not started and llm_progress.get('status') in ('process_error','cancelled_before_start'):
            status=llm_progress['status']
        motions=self.motion
        grid=self.state.get('occupancy_map') if self.state else None
        objects=self.state.get('semantic_objects',[]) if self.state else []
        classes=sorted(set(o['class_name'] for o in objects))
        route=self.route[::max(1,len(self.route)//1400)]
        if self.route and (not route or route[-1]!=self.route[-1]):
            route.append(self.route[-1])
        return dict(policy=policy,status=status,operation=self.operation if started else 'Waiting in queue',
            sim_s=summary['simulated_time_s'] if summary else motions.get('simulated_time_s',0),
            wall_s=summary['wall_time_s'] if summary else (max(0,time.time()-path.stat().st_ctime) if started else 0),
            distance_m=summary['actual_distance_m'] if summary else motions.get('actual_distance_m',0),
            collisions=summary['collisions'] if summary else motions.get('collisions',0),
            decisions=summary['decisions'] if summary else self.decisions,
            known_cells=self.capture.get('known_cells',0),capture_sequence=self.capture.get('sequence',0),
            pose=motions.get('pose'),goal=self.goal,rationale=self.rationale,
            target_seen=self.target_seen or bool(self.state and self.state.get('target_claim')),
            classes=classes,route=route,grid=grid,frontiers=self.state.get('frontiers',[]) if self.state else [],
            timeline=list(reversed(self.timeline)),
            llm_calls=summary.get('llm_calls',0) if summary else self.llm_calls,
            llm_fallbacks=summary.get('llm_fallbacks',0) if summary else self.llm_fallbacks,
            evaluation=summary.get('evaluation',{}) if summary else {},
            trajectory_available=(self.folder/'trajectory.png').is_file(),
            trajectory_scene_available=(self.folder/'trajectory-scene.png').is_file(),
            age_s=round(time.time()-path.stat().st_mtime,1) if started else None)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-root',type=Path,required=True)
    parser.add_argument('--port',type=int,default=8910)
    args=parser.parse_args()
    out=args.run_root.resolve()
    lock=threading.Lock()
    protocol=read(out/'protocol.json')
    batch='engine_fingerprint' in protocol and len(protocol['jobs'])>4
    episodes={j['job_id']:Episode(out/'episodes'/j['job_id']) for j in protocol['jobs']} if batch else {p:Episode(out/p) for p in POLICIES}
    scene_keys={f"{j['target_location']}-{j['profile']}-start{j['start_id']}"
                for j in protocol['jobs'] if j.get('target_location') and j.get('start_id')}

    def batch_data(query):
        views=[]
        for job in protocol['jobs']:
            view=episodes[job['job_id']].view(job['policy'],None,{})
            view.update(job_id=job['job_id'],profile=job['profile'],seed=job['environment_seed'])
            target=(protocol['targets_evaluator_only'][job['target_location']] if 'targets_evaluator_only' in protocol
                    else protocol['target_evaluator_only'])
            view.update(start=job['start_pose'],target=dict(class_name=target['class_name'],position=target['position_world']),
                        start_id=job.get('start_id'),target_location=job.get('target_location'))
            scene_key=(f"{job['target_location']}-{job['profile']}-start{job['start_id']}"
                       if job.get('target_location') and job.get('start_id') else None)
            layer=out/'scene-floorplans'/(scene_key+'-objects.png') if scene_key else None
            layer_meta=read(layer.with_suffix('.json'),{}) if layer and layer.is_file() else {}
            view.update(scene_overlay_url=('/scene-objects/'+scene_key+'.png') if layer_meta else None,
                        scene_overlay_extent=layer_meta.get('floor_extent_xy_m'))
            views.append(view)
        cases=[]
        selected=query.get('job',[None])[0]
        for policy in POLICIES:
            group=[v for v in views if v['policy']==policy]
            chosen=next((v for v in group if v['job_id']==selected),None)
            chosen=chosen or next((v for v in group if v['status']=='running'),None)
            chosen=chosen or next((v for v in reversed(group) if v['status']!='queued'),group[0])
            chosen=dict(chosen,finished=sum(v['status'] not in ('queued','running') for v in group),
                        planned=len(group),successes=sum(v['status']=='success' for v in group))
            cases.append(chosen)
        keys=('job_id','policy','profile','seed','status','sim_s','wall_s','distance_m','collisions','decisions','llm_calls','llm_fallbacks','trajectory_available','trajectory_scene_available','start_id','target_location')
        target=protocol['target_evaluator_only']
        progress=read(out/'progress.json',{})
        launch=read(out/'launch-status.json',{})
        execution=read(out/'execution-amendment.json',{})
        return dict(updated_utc=datetime.now(timezone.utc).isoformat(),cases=cases,
            jobs=[{k:v[k] for k in keys} for v in views],noise=protocol['sensor_noise'],
            target=dict(class_name=target['class_name'],model=target['model'],position=target['position_world']),
            limits=protocol['base_manifest']['limits'],start=protocol['base_manifest']['start_pose'],
            active=next((v['policy'] for v in views if v['status']=='running'),None),
            active_job=next((v['job_id'] for v in views if v['status']=='running'),None),
            completed=sum(v['status'] not in ('queued','running') for v in views),planned=len(views),
            phase='complete' if all(v['status'] not in ('queued','running') for v in views) else
                  'running' if any(v['status']=='running' for v in views) else progress.get('status','preparing'),
            concurrent_workers=execution.get('max_concurrent_workers',protocol['runtime']['workers']),
            llm_authorized=read(out/'llm-authorization.json',{}).get('authorized',False),
            llm_status='blocked_by_approval' if launch.get('llm_approval_blocked') else 'included_in_batch',
            llm_model=protocol['base_manifest']['policy']['llm_model'],
            source_fingerprint=protocol['engine_fingerprint'],report_type='benchmark',design=protocol.get('design'))

    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):
            pass

        def do_GET(self):
            route=urlsplit(self.path).path
            if route=='/api/status':
                with lock:
                    if batch:
                        payload=batch_data(parse_qs(urlsplit(self.path).query))
                    else:
                        progress=read(out/'progress.json',{})
                        llm=read(out/'llm-progress.json',{})
                        authorization=read(out/'llm-authorization.json',{})
                        cases=[episodes[p].view(p,progress.get('active'),llm) for p in POLICIES]
                        payload=dict(updated_utc=datetime.now(timezone.utc).isoformat(),
                        cases=cases,noise=protocol['noise'],target=protocol['target'],
                        limits=protocol['jobs'][0]['manifest']['limits'],
                        start=protocol['jobs'][0]['manifest']['start_pose'],
                        active=next((c['policy'] for c in cases if c['status']=='running'),None),
                        llm_authorized=authorization.get('authorized',False),
                        llm_status=llm.get('status','queued'),llm_model=authorization.get('model'),
                        source_fingerprint=protocol['source_fingerprint'])
                    body=json.dumps(payload,ensure_ascii=False).encode('utf-8')
                content='application/json; charset=utf-8'
            elif route=='/':
                body=(ROOT/'scripts/templates/bowl_live_report.html').read_bytes()
                content='text/html; charset=utf-8'
            elif route=='/floorplan.png':
                body=(ROOT/'data/igibson-data/g_dataset/Rs/floorplan.png').read_bytes()
                content='image/png'
            elif route.startswith('/scene-objects/') and route.endswith('.png'):
                scene_key=route[len('/scene-objects/'):-len('.png')]
                layer=out/'scene-floorplans'/(scene_key+'-objects.png')
                if scene_key not in scene_keys or not layer.is_file():
                    self.send_error(404)
                    return
                body=layer.read_bytes()
                content='image/png'
            elif route.startswith('/trajectory/') and route.endswith('.png'):
                job_id=route[len('/trajectory/'):-len('.png')]
                if job_id not in episodes or not (episodes[job_id].folder/'trajectory.png').is_file():
                    self.send_error(404)
                    return
                scene_image=episodes[job_id].folder/'trajectory-scene.png'
                body=(scene_image if scene_image.is_file() else episodes[job_id].folder/'trajectory.png').read_bytes()
                content='image/png'
            elif route.startswith('/trajectory-reference/') and route.endswith('.png'):
                job_id=route[len('/trajectory-reference/'):-len('.png')]
                if job_id not in episodes or not (episodes[job_id].folder/'trajectory.png').is_file():
                    self.send_error(404)
                    return
                body=(episodes[job_id].folder/'trajectory.png').read_bytes()
                content='image/png'
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type',content)
            self.send_header('Cache-Control','no-store')
            self.send_header('Content-Length',str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    server=ThreadingHTTPServer(('127.0.0.1',args.port),Handler)
    print('Live report: http://127.0.0.1:%d/'%args.port,flush=True)
    server.serve_forever()


if __name__=='__main__':
    main()
