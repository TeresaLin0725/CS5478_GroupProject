// Chromium checks of the evidence links, statistics tables and responsive layout.
const fs=require('fs'),path=require('path'),assert=require('assert/strict');
const {pathToFileURL}=require('url');
const directory=path.resolve(process.argv[2]);
const compact=fs.existsSync(path.join(directory,'benchmark.html'));
const root=compact?path.join(directory,'evidence/benchmark'):path.dirname(directory);
const output=process.env.DEMO_REPORT_CHECK_OUTPUT,port=process.env.DEMO_REPORT_CHECK_PORT;
let socket,sequence=0;const pending=new Map(),errors=[];
async function call(method,params={}){const id=++sequence;return new Promise((resolve,reject)=>{pending.set(id,{resolve,reject});socket.send(JSON.stringify({id,method,params}));});}
async function evaluate(expression){const r=await call('Runtime.evaluate',{expression,awaitPromise:true,returnByValue:true});if(r.exceptionDetails)throw Error(JSON.stringify(r.exceptionDetails));return r.result.value;}
async function waitFor(expression){for(let i=0;i<100;i++){if(await evaluate(expression))return;await new Promise(r=>setTimeout(r,50));}throw Error('Timeout: '+expression);}
async function screenshot(name){const r=await call('Page.captureScreenshot',{format:'png',captureBeyondViewport:false});fs.writeFileSync(path.join(output,name),Buffer.from(r.data,'base64'));}
(async()=>{
 const targets=await(await fetch('http://127.0.0.1:'+port+'/json/list')).json();
 socket=new WebSocket(targets.find(t=>t.type==='page').webSocketDebuggerUrl);
 await new Promise((resolve,reject)=>{socket.onopen=resolve;socket.onerror=reject;});
 socket.onmessage=e=>{const m=JSON.parse(e.data);if(m.method==='Runtime.exceptionThrown')errors.push(m.params.exceptionDetails);if(m.id){const p=pending.get(m.id);pending.delete(m.id);m.error?p.reject(Error(JSON.stringify(m.error))):p.resolve(m.result);}};
 await call('Page.enable');await call('Runtime.enable');
 await call('Emulation.setDeviceMetricsOverride',{width:1500,height:1100,deviceScaleFactor:1,mobile:false});
 await call('Page.navigate',{url:pathToFileURL(path.join(directory,compact?'benchmark.html':'index.html')).href});
 await waitFor(`document.readyState==='complete' && !!document.querySelector('#episodes')`);
 const text=await evaluate('document.body.innerText');
 assert.ok(text.includes('Search performance'));assert.ok(!/__CARDS__|__TABLE__|__TIME__|\ufffd/.test(text));
 const n=await evaluate(`document.querySelectorAll('#episodes tr').length`);
 const stats=JSON.parse(fs.readFileSync(path.join(root,'results.json'),'utf8'));
 if(stats.complete){assert.equal(n,36);assert.ok(!await evaluate(`!!document.querySelector('meta[http-equiv="refresh"]')`));}
 const links=await evaluate(`Array.from(document.querySelectorAll('a[href]')).map(a=>a.getAttribute('href'))`);
 for(const href of links){assert.ok(!/^[a-z]+:/i.test(href),'Unexpected remote dependency');assert.ok(fs.existsSync(path.resolve(directory,href)),href);}
 await screenshot('benchmark-overview.png');
 const filters={};
 for(const policy of ['random','nearest','information_gain','llm_guided']){
  await evaluate(`document.querySelector('#filter').value=${JSON.stringify(policy)};document.querySelector('#filter').dispatchEvent(new Event('change'))`);
  const visible=await evaluate(`Array.from(document.querySelectorAll('#episodes tr')).filter(r=>!r.hidden).map(r=>r.dataset.policy)`);
  assert.ok(visible.every(v=>v===policy));if(stats.complete)assert.equal(visible.length,9);filters[policy]=visible.length;
 }
 await evaluate(`document.querySelector('#filter').value='all';document.querySelector('#filter').dispatchEvent(new Event('change'))`);
 await call('Emulation.setDeviceMetricsOverride',{width:430,height:932,deviceScaleFactor:1,mobile:true});
 await evaluate('window.scrollTo(0,0)');await screenshot('benchmark-mobile.png');
 assert.ok(await evaluate('document.documentElement.scrollWidth<=window.innerWidth+1'),'Mobile page overflows');
 if(fs.existsSync(path.join(directory,'analysis.html'))){
  await call('Page.navigate',{url:pathToFileURL(path.join(directory,'analysis.html')).href});
  await waitFor(`document.readyState==='complete' && !!document.querySelector('img') && document.querySelector('img').complete`);
  assert.ok(await evaluate(`document.querySelector('img').naturalWidth>0`));
  const analysisLinks=await evaluate(`Array.from(document.querySelectorAll('a[href]')).map(a=>a.getAttribute('href'))`);
  for(const href of analysisLinks)assert.ok(fs.existsSync(path.resolve(directory,href)),href);
  assert.equal(await evaluate('document.querySelectorAll("tbody tr").length'),13+stats.results.filter(r=>!r.success).length);
  assert.ok(await evaluate('document.documentElement.scrollWidth<=window.innerWidth+1'));
  await call('Emulation.setDeviceMetricsOverride',{width:1500,height:1100,deviceScaleFactor:1,mobile:false});
  await screenshot('benchmark-task-grid.png');
 }
 assert.equal(errors.length,0);
 const result={passed:true,complete:stats.complete,completed_visible:n,filters,verified_evidence_links:links.length,runtime_errors:errors,mobile_no_page_overflow:true};
 fs.writeFileSync(path.join(output,'verification.json'),JSON.stringify(result,null,2));console.log(JSON.stringify(result));socket.close();
})().catch(e=>{console.error(e.stack);if(socket)socket.close();process.exitCode=1;});
