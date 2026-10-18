// Presentation controls read the same original simulation timeline as the tables.
const playToggle=byId('play-toggle');
const clockLabel=t=>Math.floor(t/60)+':'+String(Math.floor(t%60)).padStart(2,'0');
function playbackEstimate(){
 const speed=Number(byId('speed').value);
 byId('playback-estimate').textContent=speed+'× 播放 · 约 '+clockLabel(current.duration/speed)+' 看完 · 时间轴始终显示原始仿真时间';
}
function playerState(){
 const active=current.media_type==='video'?!video.paused:playing;
 playToggle.textContent=active?'Ⅱ 暂停':'▶ 播放';
 playToggle.setAttribute('aria-label',active?'暂停录像':'播放录像');
 playToggle.setAttribute('aria-pressed',String(active));
}
function togglePlayback(){
 if(current.media_type==='video'){
  if(video.paused){if(video.ended||currentTime>=current.duration-.08)seek(0);video.play().catch(()=>{byId('playback-estimate').textContent='无法自动播放，请使用视频原生播放按钮。';});}
  else video.pause();
 }else byId('sample-play').click();
 playerState();
}
function missionStatus(){
 const active=current.operations.find(o=>o.time<=currentTime+1e-6&&currentTime<o.end-1e-6);
 const recent=current.decisions.filter(d=>d.time<=currentTime+1e-6).at(-1);
 const finished=currentTime>=current.duration-.08;
 byId('live-stage').textContent=finished?(current.success?'目标验收通过':'本次任务已结束'):(active?.kind||'观测更新与任务衔接');
 byId('live-reason').textContent=finished?'终止画面与独立验收证据见下方原始材料。':recent?recent.reason:'先通过固定朝向扫描建立地图，随后从可达前沿中选择目标。';
 const cs=[...byId('chapters').children];
 let index=-1;cs.forEach((b,i)=>{if(Number(b.dataset.time)<=currentTime+.01)index=i;});
 cs.forEach((b,i)=>{b.classList.toggle('active',i===index);b.setAttribute('aria-current',i===index?'step':'false');});
}
function buildPresentation(){
 const failureCalls=current.llm_failure_reselections||0;
 const failureRow=document.querySelector('#design-check tbody tr:last-child');
 failureRow.querySelector('.tag').textContent=failureCalls?'本次日志已观测':'本次未触发';
 failureRow.cells[2].textContent=failureCalls?
  failureCalls+' 次在线前沿选择收到上次导航失败上下文；模型实际回答可在决策记录中核对。':
  '接口可传递导航失败上下文；本次录像未证明失败后再次在线选择。共享恢复规则仍负责冷却与重试限制。';
 byId('mission-target-name').textContent='寻找'+current.target;
 byId('mission-backend').textContent=current.model_calls?'在线模型前沿选择':current.key.includes('llm_guided')?'预置回答回放':'本地策略前沿选择';
 byId('hero-metrics').replaceChildren();
 for(const [value,label] of [[current.success?'验收通过':'已结束','本次任务结果'],[current.duration.toFixed(1)+' s','原始仿真时间'],[current.distance.toFixed(2)+' m','实际行驶距离'],[current.model_calls+' 次','在线模型调用']]){
  const d=document.createElement('div');d.append(text('strong',value),text('span',label));byId('hero-metrics').append(d);
 }
 byId('operation-track').replaceChildren();
 for(const o of current.operations){
  const segment=document.createElement('span');
  segment.style.left=(o.time/current.duration*100)+'%';
  segment.style.width=(Math.max(0,o.end-o.time)/current.duration*100)+'%';
  segment.className=o.status==='失败'?'failed':o.kind==='导航行驶'?'navigate':'observe';
  byId('operation-track').append(segment);
 }
 const firstNav=current.operations.find(o=>o.kind==='导航行驶');
 const fail=current.operations.find(o=>o.status==='失败');
 const approach=current.decisions.find(d=>d.kind.includes('接近')||d.goal.includes('target'));
 const markers=[{time:0,label:'初始观察'},firstNav&&{time:firstNav.time,label:'前沿探索'},
  fail&&{time:Math.max(0,fail.end-.5),label:'受阻与恢复'},
  approach&&{time:approach.time,label:'接近目标'},
  current.evidence[0]&&{time:Math.max(0,current.evidence[0].time-1),label:'目标确认'},
  {time:current.duration,label:'任务结果'}].filter(Boolean).sort((a,b)=>a.time-b.time);
 byId('chapters').replaceChildren();
 for(const m of markers){
  const button=text('button',m.label);button.dataset.time=m.time;button.append(text('small',clockLabel(m.time)));
  button.addEventListener('click',()=>{seek(m.time);missionStatus();playerState();});byId('chapters').append(button);
 }
 playbackEstimate();missionStatus();playerState();
}
playToggle.addEventListener('click',togglePlayback);
byId('restart').addEventListener('click',()=>{seek(0);togglePlayback();});
byId('theater-toggle').addEventListener('click',()=>{
 const enabled=document.body.classList.toggle('theater');
 byId('theater-toggle').textContent=enabled?'退出专注':'专注演示';
 byId('theater-toggle').setAttribute('aria-pressed',String(enabled));
 byId('playback').scrollIntoView({behavior:'smooth'});
});
for(const event of ['play','pause','ended'])video.addEventListener(event,playerState);
video.addEventListener('timeupdate',missionStatus);
byId('speed').addEventListener('change',playbackEstimate);
scrub.addEventListener('input',()=>{missionStatus();playerState();});
// choose() remains the single source of truth for tables, media and time.
const originalChoose=choose;
choose=function(key){originalChoose(key);buildPresentation();};
const originalUpdate=update;
update=function(t){originalUpdate(t);missionStatus();};
window.addEventListener('hashchange',()=>{const key=location.hash.slice(1);if(report.cases.some(c=>c.key===key)&&current.key!==key)choose(key);});
document.addEventListener('keydown',event=>{
 if(event.code==='Escape'&&document.body.classList.contains('theater'))byId('theater-toggle').click();
 if(event.code!=='Space'||event.target.closest('button,a,input,select,summary,textarea'))return;
 const box=byId('playback').getBoundingClientRect();
 if(box.bottom>0&&box.top<innerHeight){event.preventDefault();togglePlayback();}
});
buildPresentation();
if(report.cases.some(c=>'#'+c.key===entryHash))byId('playback').scrollIntoView();
