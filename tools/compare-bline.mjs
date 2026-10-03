// node tools/compare-bline.mjs OUTPUT_DIR
import fs from 'node:fs';
import path from 'node:path';
const dir = process.argv[2];
function load(name) {
  return fs.readFileSync(path.join(dir, name), 'utf8').trim().split('\n').slice(1).map(line => {
    const v = line.split(',');
    return { t: +v[0], x: +v[1], y: +v[2], h: +v[3], active: v[4] === 'true' };
  });
}
const gui = load('gui-preview.csv');
const allRobot = load('robot-trace.csv');
const lastActive = allRobot.findLastIndex(p => p.active);
const robot = allRobot.slice(0, Math.min(lastActive + 2, allRobot.length)).filter((p, i) => p.active || i === lastActive + 1);
if (gui.length < 2 || robot.length < 2) throw Error('Insufficient active trajectory data');
const origin = robot[0].t;
robot.forEach(p => p.t -= origin);
const distance = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);
const headingError = (a, b) => Math.atan2(Math.sin(a.h - b.h), Math.cos(a.h - b.h)) * 180 / Math.PI;
const length = points => points.slice(1).reduce((sum, p, i) => sum + distance(p, points[i]), 0);
function stats(values) {
  const sorted = [...values].sort((a, b) => a - b);
  return { mean: values.reduce((a,b) => a+b,0)/values.length, rms: Math.sqrt(values.reduce((a,b) => a+b*b,0)/values.length), p95: sorted[Math.ceil(sorted.length*.95)-1], max: sorted.at(-1) };
}
// Order-preserving dynamic time warping: no freely jumping between repeated/crossing legs.
// Alignment minimizes XY distance only; this is a comparison, not a PID certification.
const m = robot.length, n = gui.length;
const cost = Array.from({length:m}, () => new Float64Array(n).fill(Infinity));
const prev = Array.from({length:m}, () => new Uint8Array(n));
for (let i=0;i<m;i++) for(let j=0;j<n;j++) {
  const error = distance(robot[i], gui[j]);
  if (!i && !j) { cost[i][j] = error; continue; }
  const candidates = [i && j ? cost[i-1][j-1] : Infinity, i ? cost[i-1][j] : Infinity, j ? cost[i][j-1] : Infinity];
  const k = candidates.indexOf(Math.min(...candidates));
  cost[i][j] = error + candidates[k]; prev[i][j] = k;
}
const matches = Array.from({length:m},()=>[]);
let i=m-1,j=n-1;
while (true) {
  matches[i].push(j);
  if (!i && !j) break;
  const k=prev[i][j];
  if(k===0){i--;j--;} else if(k===1)i--;else j--;
}
const rows = robot.map((p,i) => {
  const j = matches[i].reduce((a,b) => distance(p,gui[a]) <= distance(p,gui[b]) ? a : b);
  const g=gui[j];
  return [p.t,p.x,p.y,p.h,g.t,g.x,g.y,g.h,distance(p,g),headingError(p,g)];
});
fs.writeFileSync(path.join(dir,'aligned-comparison.csv'), 'robot_time_seconds,robot_x_meters,robot_y_meters,robot_heading_radians,gui_time_seconds,gui_x_meters,gui_y_meters,gui_heading_radians,position_error_meters,heading_error_degrees\n'+rows.map(r=>r.join(',')).join('\n')+'\n');
let cursor=0;
const timedErrors=robot.filter(p=>p.t<=gui.at(-1).t).map(p=>{
  while(cursor+1<n && gui[cursor+1].t<p.t) cursor++;
  const a=gui[cursor],b=gui[Math.min(cursor+1,n-1)];
  const u=b.t===a.t ? 0 : Math.max(0,Math.min(1,(p.t-a.t)/(b.t-a.t)));
  return distance(p,{x:a.x+(b.x-a.x)*u,y:a.y+(b.y-a.y)*u});
});
const report = {
  method: 'XY-cost order-preserving dynamic time warping; one lowest-distance matched GUI sample per robot sample. Heading is evaluated after XY alignment. Includes first post-completion pose. Timing comparison uses common elapsed-time interval only.',
  gui: {samples:n,duration_seconds:gui.at(-1).t,length_meters:length(gui)},
  robot: {samples:m,duration_seconds:robot.at(-1).t,length_meters:length(robot)},
  aligned_position_error_meters:stats(rows.map(r=>r[8])),
  aligned_absolute_heading_error_degrees:stats(rows.map(r=>Math.abs(r[9]))),
  same_elapsed_time_position_error_meters:stats(timedErrors),
  endpoint_position_difference_meters:distance(robot.at(-1),gui.at(-1)),
  endpoint_heading_difference_degrees:headingError(robot.at(-1),gui.at(-1)),
  caveats:['Current upstream GUI simulator revision, not verified against the team desktop GUI version.','Simulation is not real-hardware accuracy or PID validation.','Dynamic time warping removes timing differences and is an optimistic geometric alignment, not proof of identical segment/event behavior.']
};
fs.writeFileSync(path.join(dir,'comparison-summary.json'),JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify(report,null,2));
const W=1100,H=700,pad=65;
const points=[...gui,...robot];
const xmin=Math.min(...points.map(p=>p.x))-.25,xmax=Math.max(...points.map(p=>p.x))+.25;
const ymin=Math.min(...points.map(p=>p.y))-.25,ymax=Math.max(...points.map(p=>p.y))+.25;
const scale=Math.min((W-2*pad)/(xmax-xmin),(H-2*pad)/(ymax-ymin));
const sx=x=>pad+(x-xmin)*scale,sy=y=>H-pad-(y-ymin)*scale;
const poly=ps=>ps.map(p=>`${sx(p.x).toFixed(2)},${sy(p.y).toFixed(2)}`).join(' ');
let grid='';
for(let x=Math.ceil(xmin*2)/2;x<=xmax;x+=.5)grid+=`<path d="M ${sx(x)} ${pad} V ${H-pad}" stroke="#dce2e8"/><text x="${sx(x)}" y="${H-pad+22}" text-anchor="middle">${x.toFixed(1)}</text>`;
for(let y=Math.ceil(ymin*2)/2;y<=ymax;y+=.5)grid+=`<path d="M ${pad} ${sy(y)} H ${W-pad}" stroke="#dce2e8"/><text x="${pad-12}" y="${sy(y)+4}" text-anchor="end">${y.toFixed(1)}</text>`;
fs.writeFileSync(path.join(dir,'trajectory-overlay.svg'),`<svg xmlns="http://www.w3.org/2000/svg" width="${W}" height="${H}" viewBox="0 0 ${W} ${H}"><rect width="100%" height="100%" fill="white"/><g font-family="Arial" font-size="13" fill="#263344"><text x="65" y="28" font-size="20">phase-1-canvas-draft — GUI preview vs saved robot simulation</text><text x="65" y="49" fill="#007eaa">Blue: GUI preview (${report.gui.duration_seconds.toFixed(2)} s)</text><text x="440" y="49" fill="#db6524">Orange: robot simulation (${report.robot.duration_seconds.toFixed(2)} s)</text>${grid}<polyline points="${poly(gui)}" fill="none" stroke="#007eaa" stroke-width="4"/><polyline points="${poly(robot)}" fill="none" stroke="#db6524" stroke-width="2.5" stroke-dasharray="7 4"/><circle cx="${sx(robot[0].x)}" cy="${sy(robot[0].y)}" r="6" fill="#142b43"/><text x="${W/2}" y="${H-12}" text-anchor="middle">Field X (metres); field Y increases upward; equal XY scale</text></g></svg>`);
