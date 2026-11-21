// Offline DOM harness. Browser automation is unavailable in this workspace.
const fs=require('fs'),vm=require('vm'),assert=require('assert'),path=require('path');
const directory=process.argv[2];if(!directory)throw Error('Usage: node scripts/check_igibson_viewer.js RUN_DIRECTORY');
const html=fs.readFileSync(path.join(directory,'index.html'),'utf8');
function element(id=''){return {id,value:'0',children:[],dataset:{},textContent:'',src:'',
 currentTime:0,duration:10000,readyState:1,paused:true,
 pause(){this.paused=true},play(){this.paused=false;return Promise.resolve()},load(){this.currentTime=0},
 classList:{toggle(){}},append(e){this.children.push(e)},replaceChildren(){this.children=[]},
 querySelector(){return this.body||(this.body=element('tbody'))}};}
const nodes={};for(const match of html.matchAll(/id="([^"]+)"/g))nodes[match[1]]=element(match[1]);nodes.speed.value='1';
let tick;const sandbox={document:{getElementById:id=>nodes[id],createElement:()=>element()},performance:{now:()=>0},requestAnimationFrame:f=>{tick=f}};
vm.createContext(sandbox);vm.runInContext(html.match(/<script>([\s\S]*?)<\/script>/)[1],sandbox,{timeout:10000});
const read=s=>vm.runInContext(s,sandbox);const episodes=read('DATA.episodes');assert(episodes.length>0);
let checks=0;for(const profile of nodes.scenario.children){nodes.scenario.value=profile.value;nodes.scenario.onchange();
 for(const button of [...nodes.policies.children]){button.onclick();const video=read('hasVideo()');assert(fs.existsSync(path.join(directory,video?nodes.movie.src:nodes.scene.src)));assert(nodes.events.children.length>0);
 nodes.seek.value='1000';nodes.seek.oninput();assert.equal(read('t'),read('current.summary.sim_time_s'));assert.equal(read('playing'),false);
 assert.equal(nodes.distance.textContent,read('current.summary.executed_distance_m.toFixed(1)')+' m');
 nodes.play.onclick();assert.equal(read('t'),0);assert.equal(read('playing'),true);tick(100);nodes.play.onclick();assert.equal(read('playing'),false);
 nodes.events.children[0].onclick();assert.equal(read('playing'),false);assert(fs.existsSync(path.join(directory,video?nodes.movie.src:nodes.scene.src)));
 if(video){assert.equal(nodes.movie.currentTime,read('t'));assert(nodes.scene.hidden);assert(!nodes.movie.hidden);
  nodes.speed.value='4';nodes.speed.onchange();assert.equal(nodes.movie.playbackRate,4);nodes.speed.value='1';
  nodes.movie.onended();assert.equal(read('playing'),false);assert.equal(read('t'),read('current.summary.sim_time_s'));}
 nodes.restart.onclick();assert.equal(read('t'),0);assert.equal(read('playing'),true);
 if(button.dataset.id.endsWith('/llm_guided'))assert(nodes.llm.textContent.includes('No model inference')||nodes.llm.textContent.includes('Live model'));
 checks++;}}
assert.equal(nodes.results.querySelector('tbody').children.length,episodes.length);
const report={passed:true,mode:'offline DOM runtime harness; browser screenshot unavailable',episodes:checks,
 checks:['scenario/policy selection','local image/video loading','terminal seek and measured distance','pause','restart from end','event jump','video rate/end handling','LLM provenance','result rows']};
fs.writeFileSync(path.join(directory,'viewer-check.json'),JSON.stringify(report,null,2));console.log(JSON.stringify(report));
