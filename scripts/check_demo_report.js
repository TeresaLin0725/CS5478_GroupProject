// Real Chromium checks of local-file media, record correspondence and timeline controls.
const fs=require('fs'),path=require('path'),assert=require('assert/strict');
const {pathToFileURL}=require('url');
const root=path.resolve(__dirname,'..');
const output=process.env.DEMO_REPORT_CHECK_OUTPUT||path.join(root,'report/verification/demo');
const reportDirectories=process.argv.slice(2).length?process.argv.slice(2).map(p=>path.resolve(p)):
 [path.join(root,'report')];
let socket,sequence=0;const pending=new Map(),runtimeErrors=[];
async function command(method,params={}){const id=++sequence;return new Promise((resolve,reject)=>{pending.set(id,{resolve,reject});socket.send(JSON.stringify({id,method,params}));});}
async function evaluate(expression,userGesture=false){const r=await command('Runtime.evaluate',{expression,awaitPromise:true,returnByValue:true,userGesture});if(r.exceptionDetails)throw new Error(JSON.stringify(r.exceptionDetails));return r.result.value;}
async function until(expression){for(let i=0;i<100;i++){if(await evaluate(expression))return;await new Promise(r=>setTimeout(r,50));}throw new Error('Timed out: '+expression);}
async function screenshot(name){await new Promise(r=>setTimeout(r,200));const r=await command('Page.captureScreenshot',{format:'png',captureBeyondViewport:false});fs.writeFileSync(path.join(output,name),Buffer.from(r.data,'base64'));}
function readJSON(file){return JSON.parse(fs.readFileSync(file,'utf8'));}
function sourceCounts(c){
 const source=c.links.find(l=>l.title==='Full event log');
 const file=path.resolve(currentDirectory,source.href);
 const rows=fs.readFileSync(file,'utf8').trim().split(/\r?\n/).map(JSON.parse);
 const ds=rows.filter(r=>r.event==='decision'&&(c.group==='Integrated pipeline'||r.selection.subgoal));
 assert.equal(c.decisions.length,ds.length);
 for(let i=0;i<ds.length;i++){
  assert.equal(c.decisions[i].time,ds[i].state.timestamp_s);
  assert.equal(c.decisions[i].raw.selection.subgoal?.frontier_id,ds[i].selection.subgoal?.frontier_id);
 }
 if(c.group==='Integrated pipeline'){
  const starts=rows.filter(r=>['dispatch','observation_start','inspection_start'].includes(r.event));
  const ends=rows.filter(r=>['operation_terminal','inspection_end'].includes(r.event));
  assert.equal(c.operations.length,starts.length);
  assert.equal(c.operations.filter(o=>o.status==='Ended at task verification').length,starts.length-ends.length);
  const failed=ends.filter(r=>r.feedback?.status==='failed');
  assert.equal(c.operations.filter(o=>o.status==='Failed').length,failed.length);
  const artifact=readJSON(path.resolve(currentDirectory,c.links.find(l=>l.title==='Video and physical-trajectory verification').href));
  const sha=require('crypto').createHash('sha256').update(fs.readFileSync(path.resolve(currentDirectory,c.video))).digest('hex');
  assert.equal(sha,artifact.video_sha256,'Raw recording changed');
 }else assert.equal(c.operations.length,rows.filter(r=>r.event==='navigation_feedback').length);
 for(const link of c.links)assert.ok(fs.existsSync(path.resolve(currentDirectory,link.href)),link.href);
 for(const frame of c.frames)assert.ok(fs.existsSync(path.resolve(currentDirectory,frame.image)),frame.image);
}
let currentDirectory;
(async()=>{
 const targets=await (await fetch('http://127.0.0.1:'+(process.env.DEMO_REPORT_CHECK_PORT||'9227')+'/json/list')).json();
 const target=targets.find(t=>t.type==='page');assert.ok(target);
 socket=new WebSocket(target.webSocketDebuggerUrl);
 await new Promise((resolve,reject)=>{socket.onopen=resolve;socket.onerror=reject;});
 socket.onmessage=e=>{const message=JSON.parse(e.data);if(message.method==='Runtime.exceptionThrown')runtimeErrors.push(message.params.exceptionDetails);if(message.id){const p=pending.get(message.id);pending.delete(message.id);message.error?p.reject(new Error(JSON.stringify(message.error))):p.resolve(message.result);}};
 await command('Page.enable');await command('Runtime.enable');
 await command('Emulation.setDeviceMetricsOverride',{width:1500,height:1150,deviceScaleFactor:1,mobile:false});
 const pages=[];let buttons=0;
 for(const directory of reportDirectories){
  currentDirectory=directory;
  await command('Page.navigate',{url:pathToFileURL(path.join(currentDirectory,'index.html')).href});
  await until(`typeof current !== 'undefined' && current.key===report.default && document.readyState==='complete'`);
  const payload=await evaluate('report');assert.ok(payload.cases.length>0);
  assert.equal(await evaluate(`document.querySelectorAll('#case-select option').length`),payload.cases.length);
  const visible=await evaluate('document.body.innerText');
  assert.ok(!/__INTRO__|__FINDINGS__|__DATA__|__FACTS__/.test(visible),'Unresolved template marker');
  if(pages.length===0)await screenshot('report-overview.png');
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
   if(await evaluate(`!!byId('play-toggle')`)){
    assert.equal(await evaluate(`Number(byId('speed').value)`),2,'Default presentation speed');
    if(c.media_type==='video'){
     assert.equal(await evaluate('video.playbackRate'),2);
     await evaluate(`seek(0);byId('play-toggle').click()`,true);
     await until('!video.paused && video.currentTime>0');
     await evaluate(`byId('play-toggle').click()`);await until('video.paused');
     await evaluate(`byId('chapters').lastElementChild.click()`);
     assert.equal(await evaluate('currentTime'),c.duration,'Result chapter uses terminal simulation time');
     assert.ok(await evaluate(`byId('live-stage').textContent.includes('Verification passed')`));
     await evaluate(`byId('play-toggle').click()`,true);
     await until('!video.paused && video.currentTime>0 && video.currentTime<5');
     await evaluate(`byId('play-toggle').click()`);await until('video.paused');
     await evaluate(`byId('speed').value='1';byId('speed').dispatchEvent(new Event('change'))`);
     assert.equal(await evaluate('video.playbackRate'),1);
     await evaluate(`byId('speed').value='2';byId('speed').dispatchEvent(new Event('change'))`);
    }
    assert.ok(await evaluate(`byId('chapters').children.length>=3`));
    await evaluate(`byId('theater-toggle').click()`);
    assert.ok(await evaluate(`document.body.classList.contains('theater')`));
    assert.equal(await evaluate(`getComputedStyle(byId('design-check')).display`),'none');
    await evaluate(`byId('theater-toggle').click()`);
    assert.ok(await evaluate(`!document.body.classList.contains('theater')`));
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
  pages.push({file:path.relative(root,path.join(currentDirectory,'index.html')),default:payload.default,cases});
 }
 currentDirectory=reportDirectories[0];
 await command('Page.navigate',{url:pathToFileURL(path.join(currentDirectory,'index.html')).href});
 await until(`typeof current!=='undefined' && current.key===report.default && video.readyState>=2`);
 await evaluate(`byId('playback').scrollIntoView();seek(23.3333333333)`);await until('!video.seeking && video.readyState>=3');
 await evaluate(`byId('play-toggle').click()`,true);await until('!video.paused && video.currentTime>23.4');
 await evaluate(`byId('play-toggle').click()`);await until('video.paused');
 await screenshot('report-player-and-records.png');
 await evaluate(`byId('decisions').scrollIntoView()`);await screenshot('report-decision-records.png');
 await evaluate(`byId('operations').scrollIntoView()`);await screenshot('report-operation-records.png');
 await command('Emulation.setDeviceMetricsOverride',{width:430,height:932,deviceScaleFactor:1,mobile:true});
 await evaluate('window.scrollTo(0,0)');await screenshot('report-mobile.png');
 assert.ok(await evaluate('document.documentElement.scrollWidth<=window.innerWidth+1'),'Mobile page overflows');
 await evaluate(`byId('theater-toggle').click()`);
 assert.notEqual(await evaluate(`getComputedStyle(byId('theater-toggle')).display`),'none','Narrow theater view must retain its exit');
 await evaluate(`byId('theater-toggle').click()`);
 const casesChecked=pages.reduce((n,p)=>n+p.cases.length,0);
 assert.deepEqual(runtimeErrors,[],'Browser script exceptions');
 const result={passed:true,browser:'Chrome headless, real local-file media decoding',pages,cases_checked:casesChecked,timeline_buttons_checked:buttons,raw_videos_unchanged:true,source_log_correspondence:true,mobile_no_page_overflow:true,presentation_controls_checked:true,no_script_exceptions:true};
 fs.writeFileSync(path.join(output,'verification.json'),JSON.stringify(result,null,2));
 console.log(JSON.stringify({passed:true,cases_checked:casesChecked,timeline_buttons_checked:buttons,screenshots:5}));
 await command('Browser.close');socket.close();
})().catch(error=>{console.error(error.stack);if(socket)socket.close();process.exitCode=1;});
