// Real Chromium checks of local-file media, record correspondence and timeline controls.
const fs=require('fs'),path=require('path'),assert=require('assert/strict');
const {pathToFileURL}=require('url');
const root=path.resolve(__dirname,'..');
const output=path.join(root,'runs/demo-report-20261018/browser-check');
let socket,sequence=0;const pending=new Map();
async function command(method,params={}){const id=++sequence;return new Promise((resolve,reject)=>{pending.set(id,{resolve,reject});socket.send(JSON.stringify({id,method,params}));});}
async function evaluate(expression){const r=await command('Runtime.evaluate',{expression,awaitPromise:true,returnByValue:true});if(r.exceptionDetails)throw new Error(JSON.stringify(r.exceptionDetails));return r.result.value;}
async function until(expression){for(let i=0;i<100;i++){if(await evaluate(expression))return;await new Promise(r=>setTimeout(r,50));}throw new Error('Timed out: '+expression);}
async function screenshot(name){const r=await command('Page.captureScreenshot',{format:'png',captureBeyondViewport:false});fs.writeFileSync(path.join(output,name),Buffer.from(r.data,'base64'));}
function readJSON(file){return JSON.parse(fs.readFileSync(file,'utf8'));}
function sourceCounts(c){
 const source=c.links.find(l=>l.title==='完整事件日志');
 const file=path.resolve(currentDirectory,source.href);
 const rows=fs.readFileSync(file,'utf8').trim().split(/\r?\n/).map(JSON.parse);
 const ds=rows.filter(r=>r.event==='decision'&&r.selection.subgoal);
 assert.equal(c.decisions.length,ds.length);
 for(let i=0;i<ds.length;i++){
  assert.equal(c.decisions[i].time,ds[i].state.timestamp_s);
  assert.equal(c.decisions[i].raw.selection.subgoal.frontier_id,ds[i].selection.subgoal.frontier_id);
 }
 if(c.group==='完整集成管线'){
  const starts=rows.filter(r=>['dispatch','observation_start'].includes(r.event));
  const ends=rows.filter(r=>r.event==='operation_terminal');
  assert.equal(c.operations.length,starts.length);
  assert.equal(c.operations.filter(o=>o.status==='任务验收时结束').length,starts.length-ends.length);
  const failed=ends.filter(r=>r.feedback.status==='failed');
  assert.equal(c.operations.filter(o=>o.status==='失败').length,failed.length);
  const artifact=readJSON(path.resolve(currentDirectory,c.links.find(l=>l.title==='录像与物理轨迹核验').href));
  const sha=require('crypto').createHash('sha256').update(fs.readFileSync(path.resolve(currentDirectory,c.video))).digest('hex');
  assert.equal(sha,artifact.video_sha256,'Raw recording changed');
 }else assert.equal(c.operations.length,rows.filter(r=>r.event==='navigation_feedback').length);
 for(const link of c.links)assert.ok(fs.existsSync(path.resolve(currentDirectory,link.href)),link.href);
 for(const frame of c.frames)assert.ok(fs.existsSync(path.resolve(currentDirectory,frame.image)),frame.image);
}
let currentDirectory;
(async()=>{
 const targets=await (await fetch('http://127.0.0.1:9227/json/list')).json();
 const target=targets.find(t=>t.type==='page');assert.ok(target);
 socket=new WebSocket(target.webSocketDebuggerUrl);
 await new Promise((resolve,reject)=>{socket.onopen=resolve;socket.onerror=reject;});
 socket.onmessage=e=>{const message=JSON.parse(e.data);if(message.id){const p=pending.get(message.id);pending.delete(message.id);message.error?p.reject(new Error(JSON.stringify(message.error))):p.resolve(message.result);}};
 await command('Page.enable');await command('Runtime.enable');
 await command('Emulation.setDeviceMetricsOverride',{width:1500,height:1150,deviceScaleFactor:1,mobile:false});
 const pages=[];let buttons=0;
 for(const [folder,expected] of [['integrated-llm-final-20261018','integrated-language'],['integrated-final-20261017','integrated-nearest']]){
  currentDirectory=path.join(root,'runs',folder,'recording');
  await command('Page.navigate',{url:pathToFileURL(path.join(currentDirectory,'index.html')).href});
  await until(`typeof current !== 'undefined' && current.key===${JSON.stringify(expected)} && document.readyState==='complete'`);
  const payload=await evaluate('report');assert.equal(payload.cases.length,14);
  assert.equal(await evaluate(`document.querySelectorAll('#case-select option').length`),14);
  const visible=await evaluate('document.body.innerText');
  assert.ok(!/A\/B\/C|\bW3\b|\bLLM\b|\bAPI\b|RGB-D|早期|往期|历史参考|相较|新版/.test(visible),'Unexpected shorthand or retrospective comparison');
  if(folder==='integrated-llm-final-20261018')await screenshot('report-overview.png');
  const cases=[];
  for(const c of payload.cases){
   sourceCounts(c);
   await evaluate(`selector.value=${JSON.stringify(c.key)};selector.dispatchEvent(new Event('change'))`);
   const counts=await evaluate(`({d:byId('decisions').children.length,o:byId('operations').children.length,f:byId('feedback').children.length,e:byId('evidence').children.length})`);
   assert.equal(counts.d,c.decisions.length);assert.equal(counts.o,c.operations.length);assert.equal(counts.e,c.evidence.length);
   if(c.media_type==='video'){
    await until('video.readyState>=2 && video.videoWidth>0');
    assert.equal(await evaluate('video.videoWidth'),1440);
    await evaluate('video.currentTime=Math.min(2,current.duration/2)');await until('!video.seeking');
   }else{
    await until(`byId('sample-image').complete && byId('sample-image').naturalWidth>0`);
    await evaluate(`byId('sample-play').click()`);await new Promise(r=>setTimeout(r,120));
    assert.ok(await evaluate('currentTime>0'),'Sample playback did not advance');
    await evaluate(`byId('sample-play').click()`);
   }
   const seeks=await evaluate(`(()=>{let count=0;for(const b of document.querySelectorAll('button[data-time]')){b.click();if(Math.abs(currentTime-Number(b.dataset.time))>0.002)throw Error('Wrong seek time');count++;}return count;})()`);
   buttons+=seeks;
   const midpoint=(c.operations[0].time+c.operations[0].end)/2;
   await evaluate(`seek(${midpoint})`);
   assert.equal(await evaluate(`document.querySelectorAll('#operations tr.active').length`),1);
   await evaluate(`scrub.value='0';scrub.dispatchEvent(new Event('input'))`);
   assert.equal(await evaluate('currentTime'),0);
   cases.push({key:c.key,decisions:counts.d,operations:counts.o,seek_buttons:seeks,media:c.media_type});
  }
  pages.push({file:path.relative(root,path.join(currentDirectory,'index.html')),default:expected,cases});
 }
 currentDirectory=path.join(root,'runs/integrated-llm-final-20261018/recording');
 await command('Page.navigate',{url:pathToFileURL(path.join(currentDirectory,'index.html')).href});
 await until(`typeof current!=='undefined' && current.key==='integrated-language' && video.readyState>=2`);
 await evaluate(`byId('playback').scrollIntoView();seek(23.3333333333)`);await until('!video.seeking');
 await screenshot('report-player-and-records.png');
 await evaluate(`byId('decisions').scrollIntoView()`);await screenshot('report-decision-records.png');
 await evaluate(`byId('operations').scrollIntoView()`);await screenshot('report-operation-records.png');
 await command('Emulation.setDeviceMetricsOverride',{width:430,height:932,deviceScaleFactor:1,mobile:true});
 await evaluate('window.scrollTo(0,0)');await screenshot('report-mobile.png');
 assert.ok(await evaluate('document.documentElement.scrollWidth<=window.innerWidth+1'),'Mobile page overflows');
 const result={passed:true,browser:'Chrome headless, real local-file media decoding',pages,cases_checked:28,timeline_buttons_checked:buttons,raw_videos_unchanged:true,source_log_correspondence:true,mobile_no_page_overflow:true};
 fs.writeFileSync(path.join(output,'verification.json'),JSON.stringify(result,null,2));
 console.log(JSON.stringify({passed:true,cases_checked:28,timeline_buttons_checked:buttons,screenshots:5}));
 await command('Browser.close');socket.close();
})().catch(error=>{console.error(error.stack);if(socket)socket.close();process.exitCode=1;});
