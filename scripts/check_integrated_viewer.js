// Offline DOM check; does not substitute for inspection of rendered video frames.
const fs = require('fs');
const vm = require('vm');
const path = require('path');
const directory = process.argv[2];
const html = fs.readFileSync(path.join(directory, 'index.html'), 'utf8');
const scripts = [...html.matchAll(/<script>([\s\S]*?)<\/script>/g)];
if (scripts.length !== 1) throw new Error('Expected one self-contained viewer script');
const nodes = {};
function node() { return {textContent: '', children: [], currentTime: 0,
  appendChild(child) { this.children.push(child); }}; }
const document = {getElementById(id) { return nodes[id] ||= node(); }, createElement: node};
vm.runInNewContext(scripts[0][1], {document}, {timeout: 5000});
const summary = JSON.parse(nodes.summary.textContent);
if (summary.termination !== 'success') throw new Error('Recording did not succeed');
if (!nodes.events.children.length) throw new Error('No decisions/feedback in viewer');
for (const button of nodes.events.children) {
  button.onclick();
  if (!(nodes.video.currentTime >= 0 && nodes.video.currentTime <= summary.simulated_time_s)) {
    throw new Error('Timeline button seeks outside physical episode');
  }
}
if (!html.includes('controls preload="metadata"')) throw new Error('Native playback controls missing');
const report = {passed: true, mode: 'offline DOM execution + video frame inspection separately',
  event_buttons: nodes.events.children.length, timeline_seek: true, native_video_controls: true};
fs.writeFileSync(path.join(directory, 'viewer-verification.json'), JSON.stringify(report, null, 2));
console.log(JSON.stringify(report, null, 2));
