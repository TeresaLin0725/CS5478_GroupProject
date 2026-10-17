"""Record physical simulator steps, displaying only shared public mapper/navigation state."""
import json
from math import cos,sin
from pathlib import Path
import numpy as np
from PIL import Image,ImageDraw,ImageFont
from .demo_video import DemoVideo


class IntegratedRecorder:
    def __init__(self,output,runner,sim,detector):
        self.output,self.runner,self.sim,self.detector=output,runner,sim,detector
        self.video=None
        self.caption='Initialize calibrated RGB-D pipeline'
        self.events=[]
        self.path=[]
        self.started=False
        self.last_picture=None
        self.font=ImageFont.truetype('C:/Windows/Fonts/consola.ttf',18)
        self.small=ImageFont.truetype('C:/Windows/Fonts/consola.ttf',15)
        self.title=ImageFont.truetype('C:/Windows/Fonts/consolab.ttf',28)

    def on_event(self,event,payload):
        if event=='capture_processed' and not self.started:
            self.video=DemoVideo(self.output/'integrated.mp4',15)
            self.started=True
            self.capture()
        if event=='decision':
            goal=payload['selection'].subgoal
            self.caption=('C selects '+goal.frontier_id+' / '+goal.kind) if goal else 'C requests recovery scan'
        elif event=='dispatch':
            self.caption='B executes '+payload['request'].subgoal.frontier_id
        elif event=='operation_feedback':
            fb=payload['feedback']
            if fb.planner_result and fb.planner_result.path:
                self.path=fb.planner_result.path
            self.caption='B '+payload['kind']+': '+fb.status.value
            if fb.failure_reason:self.caption+=' / '+fb.failure_reason.value
        elif event=='target_evaluation':
            self.caption='Independent evaluation: '+('SUCCESS' if payload['result'].valid_target else 'REJECTED')
        elif event=='navigation_map_feedback':
            self.caption='A > B: map update / '+payload['kind']
        elif event=='replan_request':
            self.caption='B > C: no safe path; request a new goal'
        elif event=='strategy_replan':
            self.caption='C > B: revised goal '+payload['selected'].frontier_id
        elif event=='observation_start':
            self.caption='A/B scan: '+payload['request'].routine
        if event in ('decision','dispatch','operation_feedback','target_evaluation','observation_start','operation_terminal',
                     'navigation_map_feedback','replan_request','strategy_replan'):
            self.events.append(dict(time_s=self.runner.sim_time,event=event,text=self.caption))
        if event=='motion_sample' and self.video and self.video.due(self.sim._backend.world.steps):
            self.capture()

    def picture(self):
        world=self.sim._backend.world
        world.sim.sync(force_sync=True)
        state=self.runner.state
        canvas=Image.new('RGB',(1440,900),'#101720');d=ImageDraw.Draw(canvas)
        d.text((24,18),'INTEGRATED SEARCH / GIBSON RS',font=self.title,fill='#e7eff8')
        d.text((26,62),'Native Locobot  |  RGB-D + YOLO11s  >  A mapper  >  B navigation  >  C policy  >  W3 verifier',font=self.small,fill='#80bdce')
        scene=Image.fromarray(world.spectator()).resize((880,660),Image.Resampling.LANCZOS)
        canvas.paste(scene,(24,106))
        d.text((38,118),'PHYSICAL SIMULATOR / 15 FPS',font=self.small,fill='white',stroke_width=1,stroke_fill='black')
        if state:
            grid=state.occupancy_map
            colors=np.array([[[(19,28,40) if v==-1 else (207,215,218) if v==0 else (77,92,104)
                              for v in row] for row in grid.data]],dtype=np.uint8)[0]
            map_img=Image.fromarray(colors[::-1]).resize((420,510),Image.Resampling.NEAREST)
            canvas.paste(map_img,(970,106))
            def pixel(x,y):
                return (970+(x-grid.origin_x_m)/grid.resolution_m*6,
                        616-(y-grid.origin_y_m)/grid.resolution_m*6)
            route=[pixel(p.x_m,p.y_m) for p in self.path]
            if len(route)>1:d.line(route,fill='#21a895',width=3)
            for f in state.frontiers:
                x,y=pixel(f.approach_pose.x_m,f.approach_pose.y_m)
                d.ellipse((x-2,y-2,x+2,y+2),fill='#ffad47' if f.kind=='target_approach' else '#8a9cff')
            for o in state.semantic_objects:
                if o.class_name==state.target_class and o.position_world:
                    x,y=pixel(o.position_world.x_m,o.position_world.y_m)
                    d.ellipse((x-6,y-6,x+6,y+6),outline='#ffbf47',width=2)
            p=self.sim.current_pose();x,y=pixel(p.x_m,p.y_m)
            d.ellipse((x-6,y-6,x+6,y+6),fill='#23bdd0')
            d.line((x,y,x+16*cos(p.yaw_rad),y-16*sin(p.yaw_rad)),fill='#23bdd0',width=3)
            d.text((972,83),'A: OBSERVED MAP / r%d'%state.revision,font=self.small,fill='#bbcbd7')
            d.text((970,620),'unknown / free / occupied; path by B',font=self.small,fill='#94a6b6')
        detector=self.detector
        if getattr(detector,'last_rgb',None) is not None:
            rgb=Image.fromarray(detector.last_rgb).copy();rd=ImageDraw.Draw(rgb)
            for item in detector.last_detections:
                if item.confidence<self.runner.manifest.mapping.confidence_threshold:continue
                rd.rectangle(item.bbox_xyxy_px,outline='#ffd36a',width=2)
                rd.text(item.bbox_xyxy_px[:2],'%s %.2f'%(item.class_name,item.confidence),fill='#ffd36a')
            canvas.paste(rgb.resize((300,225),Image.Resampling.LANCZOS),(1026,654))
            d.text((970,636),'Last processed RGB / detector boxes',font=self.small,fill='#bbcbd7')
        d.text((26,788),self.caption[:85],font=self.font,fill='#e3edf5')
        d.text((26,824),'SIM %6.1fs   DIST %5.2fm   CONTACT STEPS %d   DECISIONS %d' % (
            self.runner.sim_time,self.runner.distance,self.sim.collisions,self.runner.decisions),font=self.font,fill='#9db1c1')
        d.text((26,858),'Truth retained privately for scoring. Policy sees observations and assessed candidates only.',font=self.small,fill='#788d9f')
        return canvas

    def capture(self):
        self.last_picture=self.picture()
        self.video.capture(self.last_picture,self.sim._backend.world,self.caption)
        if self.video.last_step==0:self.last_picture.save(self.output/'start.png')

    def close(self):
        if self.video:
            self.capture()
            self.last_picture.save(self.output/'final.png')
            self.video.close()
            (self.output/'video.json').write_text(json.dumps(self.video.metadata(),indent=2),encoding='utf-8')


def export_viewer(output,summary):
    events=[]
    for line in (output/'events.jsonl').read_text(encoding='utf-8').splitlines():
        row=json.loads(line)
        if row['event']=='decision':
            selection=row['selection'];goal=selection['subgoal']
            events.append(dict(t=row['state']['timestamp_s'],text='C: '+(goal['frontier_id'] if goal else selection['stop_reason'])+' / '+str(selection['rationale'])))
        if row['event']=='operation_terminal':
            fb=row['feedback'];events.append(dict(t=fb['timestamp_s'],text='B: '+fb['status']+' / '+str(fb['failure_reason'])))
        if row['event']=='target_evaluation':
            events.append(dict(t=row['claim']['timestamp_s'],text='W3: '+row['result']['reason']))
        if row['event']=='navigation_map_feedback':
            events.append(dict(t=row['simulated_time_s'],text='A → B: '+row['detail']))
        if row['event']=='replan_request':
            events.append(dict(t=row['simulated_time_s'],text='B → C: 请求改选目标'))
        if row['event']=='strategy_replan':
            events.append(dict(t=row['simulated_time_s'],text='C → B: '+row['selected']['frontier_id']))
    payload=json.dumps(dict(summary=summary,events=events),ensure_ascii=False).replace('</','<\\/')
    html='''<!doctype html><html lang="zh-CN"><meta charset="utf-8"><title>完整管线 · Gibson Rs</title>
<style>body{margin:0;background:#101720;color:#e1edf7;font:16px system-ui}main{max-width:1400px;margin:32px auto;padding:0 24px}h1{font-size:25px}p{color:#9eb4c6}video{width:100%;border-radius:12px;background:#000}section{display:flex;gap:12px;flex-wrap:wrap}a,button{background:#233746;color:#bdeafa;padding:10px 14px;border:0;border-radius:6px;text-decoration:none;cursor:pointer}pre{white-space:pre-wrap;line-height:1.6;background:#17232e;padding:18px;border-radius:10px}.events button{display:block;text-align:left;width:100%;margin:6px 0}</style>
<main><h1>完整搜索管线 · Gibson Rs / 实体 Locobot</h1><p>原生 RGB-D → YOLO11s → 共享 A 地图与跟踪 → 共享 B 规划控制 → C 策略与恢复 → W3 独立验收</p>
<video id="video" src="integrated.mp4" controls preload="metadata" poster="start.png"></video><p>视频按仿真时间连续采集。右侧 RGB 为最近一次已处理观测；场景画面为每秒 15 帧真实渲染。</p>
<section><a href="summary.json">运行结果</a><a href="events.jsonl">完整事件日志</a><a href="manifest.json">实验配置</a><a href="video.json">视频帧元数据</a><a href="../validation.json">录制前验证</a></section><pre id="summary"></pre><h2>决策与反馈</h2><div class="events" id="events"></div></main>
<script>const data=__PAYLOAD__;const s=data.summary;document.getElementById('summary').textContent=JSON.stringify({termination:s.termination,policy:s.policy,profile:s.profile,simulated_time_s:s.simulated_time_s,actual_distance_m:s.actual_distance_m,collisions:s.collisions,evaluation:s.evaluation},null,2);for(const e of data.events){const b=document.createElement('button');b.textContent=e.t.toFixed(1)+'s  '+e.text;b.onclick=()=>{document.getElementById('video').currentTime=e.t;};document.getElementById('events').appendChild(b);}</script></html>'''
    (output/'index.html').write_text(html.replace('__PAYLOAD__',payload),encoding='utf-8')
