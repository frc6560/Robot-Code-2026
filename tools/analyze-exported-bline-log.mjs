// Read-only analysis of an AdvantageScope CSV; outputs separate derived evidence.
import fs from 'node:fs';
import path from 'node:path';
import assert from 'node:assert/strict';
import {createHash} from 'node:crypto';
const [input,previewFile,pathFile,output,spacingArg='.02',waypointToleranceArg]=process.argv.slice(2);
const spacing=Number(spacingArg);assert(Number.isFinite(spacing)&&spacing>=.005&&spacing<=.1);
const waypointTolerance=waypointToleranceArg===undefined?null:Number(waypointToleranceArg);
assert(waypointTolerance===null||(Number.isFinite(waypointTolerance)&&waypointTolerance>0&&waypointTolerance<=2));
const text=fs.readFileSync(input,'utf8');
function csv(s){const rows=[];let row=[],field='',quoted=false;for(let i=0;i<s.length;i++){const c=s[i];if(c==='"'){if(quoted&&s[i+1]==='"'){field+='"';i++;}else quoted=!quoted;}else if(c===','&&!quoted){row.push(field);field='';}else if(c==='\n'&&!quoted){row.push(field.replace(/\r$/,''));rows.push(row);row=[];field='';}else field+=c;}if(field||row.length){row.push(field);rows.push(row);}return rows;}
assert.deepEqual(csv('a,"b,c"\n1,"x""y"\n'),[['a','b,c'],['1','x"y']]);
const rows=csv(text),headers=rows.shift();
assert(rows.every(r=>r.length===headers.length),'Nonrectangular CSV');
const prefix='NT:/AdvantageKit/RealOutputs/BLine/FollowPath/';
function latest(name){const i=headers.indexOf(name);assert(i>=0,'Missing '+name);return rows.findLast(r=>r[i]&&r[i]!=='null')?.[i];}
const simulationTrace=headers[0]==='time_seconds'&&headers.includes('auto_active');
const syntheticTrace=simulationTrace&&headers.includes('synthetic_mock');
let raw;
if(simulationTrace){
  const activeColumn=headers.indexOf('auto_active');
  const first=rows.findIndex(r=>r[activeColumn]==='true');
  const last=rows.findLastIndex(r=>r[activeColumn]==='true');
  assert(first>=0&&last>=first,'No active autonomous samples');
  assert(rows.slice(first,last+1).every(r=>r[activeColumn]==='true'),'Multiple autonomous intervals');
  raw=rows.slice(first,Math.min(rows.length,last+2)).map(r=>({t:+r[0],x:+r[1],y:+r[2],heading:+r[3]}));
}else{
  const count=Number(latest(prefix+'robotTranslations/length'));assert(count>1&&count<250,'Possibly truncated position history');
  raw=Array.from({length:count},(_,i)=>({x:Number(latest(prefix+`robotTranslations/${i}/x`)),y:Number(latest(prefix+`robotTranslations/${i}/y`))}));
}
const count=raw.length;
assert(raw.every(p=>Number.isFinite(p.x)&&Number.isFinite(p.y)));
const model=JSON.parse(fs.readFileSync(pathFile,'utf8'));
const waypointElements=model.path_elements.filter(p=>p.translation_target);
const waypoints=waypointElements.map((p,i)=>({
  x:p.translation_target.x_meters,
  y:p.translation_target.y_meters,
  radius:i===waypointElements.length-1
    ? model.constraints.end_translation_tolerance_meters
    : waypointTolerance??p.translation_target.intermediate_handoff_radius_meters
}));
if(!simulationTrace){
  assert.equal(Number(latest(prefix+'pathTranslations/length')),waypoints.length);
  waypoints.forEach((p,i)=>{assert.equal(Number(latest(prefix+`pathTranslations/${i}/x`)),p.x);assert.equal(Number(latest(prefix+`pathTranslations/${i}/y`)),p.y);});
}
const referenceText=fs.readFileSync(previewFile,'utf8');
const referenceRows=csv(referenceText),referenceHeaders=referenceRows.shift();
const referenceIsSimulation=referenceHeaders.includes('auto_active');
const referenceIsStraight=referenceHeaders.includes('straight_reference');
let selectedReferenceRows=referenceRows;
if(referenceIsSimulation){
  const k=referenceHeaders.indexOf('auto_active');
  const first=referenceRows.findIndex(r=>r[k]==='true'),last=referenceRows.findLastIndex(r=>r[k]==='true');
  assert(first>=0&&last>=first,'No active simulation reference');
  assert(referenceRows.slice(first,last+1).every(r=>r[k]==='true'),'Multiple simulation reference intervals');
  selectedReferenceRows=referenceRows.slice(first,Math.min(referenceRows.length,last+2));
}
const preview=selectedReferenceRows.filter(r=>r.length>=4).map(r=>({t:+r[0],x:+r[1],y:+r[2],heading:+r[3]}));
assert(preview.length>1&&preview.every(p=>Number.isFinite(p.x)&&Number.isFinite(p.y)));
const distance=(a,b)=>Math.hypot(a.x-b.x,a.y-b.y);
function resample(points,step=spacing){const ss=[0];for(let i=1;i<points.length;i++)ss.push(ss.at(-1)+distance(points[i-1],points[i]));const total=ss.at(-1),result=[];let i=1;for(let s=0;s<total;s+=step){while(i<ss.length-1&&ss[i]<s)i++;const f=(s-ss[i-1])/(ss[i]-ss[i-1]||1);result.push({x:points[i-1].x+f*(points[i].x-points[i-1].x),y:points[i-1].y+f*(points[i].y-points[i-1].y),s});}result.push({...points.at(-1),s:total});return {points:result,length:total};}
function project(p,a,b){const dx=b.x-a.x,dy=b.y-a.y;const u=Math.max(0,Math.min(1,((p.x-a.x)*dx+(p.y-a.y)*dy)/(dx*dx+dy*dy||1)));const q={x:a.x+u*dx,y:a.y+u*dy};return {...q,error:distance(p,q),u};}
assert.equal(project({x:1,y:1},{x:0,y:0},{x:2,y:0}).error,1);
function nearest(p,indices){let best={error:Infinity};for(const j of indices){const q=project(p,preview[j],preview[j+1]);if(q.error<best.error)best={...q,segment:j};}return best;}
// Full monotone sequence alignment of distance-uniform samples; no pose transformation.
const robot=resample(raw),gui=resample(preview),r=robot.points,g=gui.points,n=r.length,m=g.length;
const cost=new Float64Array(n*m).fill(Infinity),back=new Uint8Array(n*m);
for(let i=0;i<n;i++)for(let j=0;j<m;j++){const k=i*m+j,d=distance(r[i],g[j])**2;if(i===0&&j===0){cost[k]=d;continue;}let best=Infinity,dir=0;if(i&&j){best=cost[(i-1)*m+j-1];dir=1;}if(i&&cost[(i-1)*m+j]<best){best=cost[(i-1)*m+j];dir=2;}if(j&&cost[i*m+j-1]<best){best=cost[i*m+j-1];dir=3;}cost[k]=best+d;back[k]=dir;}
const groups=Array.from({length:n},()=>[]);let i=n-1,j=m-1;for(;;){groups[i].push(j);if(i===0&&j===0)break;const dir=back[i*m+j];if(dir===1){i--;j--;}else if(dir===2)i--;else if(dir===3)j--;else throw Error('Broken alignment');}
const allSegments=Array.from({length:preview.length-1},(_,i)=>i);
const comparison=r.map((p,i)=>{const min=Math.min(...groups[i]),max=Math.max(...groups[i]);const lo=g[Math.max(0,min-1)].s,hi=g[Math.min(m-1,max+1)].s;const indices=[];let s=0;for(let j=0;j<preview.length-1;j++){const end=s+distance(preview[j],preview[j+1]);if(end>=lo&&s<=hi)indices.push(j);s=end;}const q=nearest(p,indices),unrestricted=nearest(p,allSegments);return {...p,gui_x:q.x,gui_y:q.y,error:q.error,gui_segment:q.segment,unrestricted_error:unrestricted.error};});
function stats(values){const a=[...values].sort((a,b)=>a-b);const q=.95*(a.length-1),lo=Math.floor(q);return {rms_cm:100*Math.sqrt(values.reduce((s,x)=>s+x*x,0)/values.length),mean_cm:100*values.reduce((s,x)=>s+x,0)/values.length,p95_cm:100*(a[lo]+(q-lo)*(a[Math.min(lo+1,a.length-1)]-a[lo])),max_cm:100*a.at(-1)};}
const primary=stats(comparison.map(p=>p.error));
const summary={source:path.resolve(input),source_sha256:createHash('sha256').update(text).digest('hex'),git_path:path.resolve(pathFile),path_sha256:createHash('sha256').update(fs.readFileSync(pathFile)).digest('hex'),waypoints_match:true,waypoint_tolerance_visualization_m:waypoints.find((_,i)=>i<waypoints.length-1)?.radius??null,history_positions:count,robot_polyline_length_m:robot.length,gui_polyline_length_m:gui.length,distance_sample_spacing_m:.02,distance_uniform_samples:n,order_aligned_geometric_deviation:primary,unrestricted_nearest_curve_lower_bound:stats(comparison.map(p=>p.unrestricted_error)),endpoint_error_cm:100*distance(raw.at(-1),waypoints.at(-1)),start_error_cm:100*distance(raw[0],waypoints[0]),peak:comparison.reduce((a,b)=>a.error>b.error?a:b),heading_error:null,timing_error:null,method:'Distance-uniform 2 cm robot/GUI sampling, full monotone XY DTW, continuous projection onto local matched GUI preview segments. Approximate order-aligned geometry, not a time-aligned control error. No shifting, rotation, scale, or start/end snapping.',limitations:['Position history has no headings or per-position timestamps. No 60 ms timestamps fabricated.','History is every third FollowPath execution, not an independently measured ground-truth trace.','Logged waypoints match, but export does not identify historical constraints/config; comparison assumes current checked-in values.','Linear interpolation between stored robot positions can miss excursions between samples.','DTW is optimistic about progress/timing; nearest-curve lower bound can hide wrong-lap matches.']};
summary.distance_sample_spacing_m=spacing;
summary.method=summary.method.replace('2 cm',`${100*spacing} cm`);
if(simulationTrace){
  summary.waypoints_match=null;
  summary.source_kind='Saved WPILOG simulation pose trace';
  summary.robot_duration_seconds=raw.at(-1).t-raw[0].t;
  summary.gui_duration_seconds=preview.at(-1).t-preview[0].t;
  summary.limitations=['One contiguous active command interval plus first post-completion pose; excludes three seconds of coast monitoring.',
    'Geometry metric removes timing differences; pose headings and timestamps are retained in robot-trace.csv but not scored here.',
    'YAGSL MapleSim defaults to the 2025 Reefscape arena, not a validated 2026 field.',
    'Simulation is not independent evidence of real-robot physical accuracy.',
    'DTW/local continuous projection is approximate and can hide progress/timing differences.'];
}
fs.mkdirSync(output,{recursive:true});
if(referenceIsSimulation){
  summary.reference_kind='Saved simulation autonomous pose trace';
  summary.reference_path=path.resolve(previewFile);
  summary.reference_sha256=createHash('sha256').update(referenceText).digest('hex');
  summary.reference_positions=preview.length;
  summary.simulation_polyline_length_m=summary.gui_polyline_length_m;
  delete summary.gui_polyline_length_m;
  summary.method=summary.method.replaceAll('GUI','simulation reference');
  summary.limitations=['Recorded robot history has no headings or per-position timestamps; timing and heading differences cannot be calculated.',
    'Simulation reference is one contiguous active command interval plus its first post-completion pose; excludes coast monitoring.',
    'Real robot positions are estimated, not independently measured ground truth. Latest physical run and historical configuration are unverified.',
    'Sparse recorded robot samples and denser simulation samples are linearly interpolated; excursions between recorded samples may be missed.',
    'Order-aligned DTW/local continuous projection is approximate and hides timing/progress differences.',
    'Simulation uses current checked-in PID gains and defaults to a 2025 MapleSim arena; historical real-robot gains are unverified.'];
}
if(referenceIsStraight){
  summary.reference_kind='Ordered straight waypoint polyline';
  summary.reference_path=path.resolve(previewFile);
  summary.reference_sha256=createHash('sha256').update(referenceText).digest('hex');
  summary.reference_positions=preview.length;
  summary.straight_reference_length_m=summary.gui_polyline_length_m;
  delete summary.gui_polyline_length_m;
  summary.method=summary.method.replaceAll('GUI','straight waypoint reference');
  summary.limitations.unshift('The straight reference is a geometric ideal with instantaneous corners, not a dynamically feasible time parameterization.');
}
if(syntheticTrace){
  summary.source_kind='Synthetic mock pose trace (not robot data)';
  summary.limitations.unshift('This input is explicitly synthetic demonstration data and must not be represented as a physical robot run.');
}
fs.writeFileSync(path.join(output,'analysis-summary.json'),JSON.stringify(summary,null,2)+'\n');
fs.writeFileSync(path.join(output,'reconstructed-positions.csv'),'history_index,x_meters,y_meters\n'+raw.map((p,i)=>[i,p.x,p.y].join(',')).join('\n')+'\n');
fs.writeFileSync(path.join(output,'deviation-by-distance.csv'),'robot_distance_meters,robot_x_meters,robot_y_meters,matched_gui_x_meters,matched_gui_y_meters,deviation_cm,nearest_curve_lower_bound_cm,gui_segment_index\n'+comparison.map(p=>[p.s,p.x,p.y,p.gui_x,p.gui_y,100*p.error,100*p.unrestricted_error,p.gui_segment].join(',')).join('\n')+'\n');
const xmin=Math.min(...raw.map(p=>p.x),...preview.map(p=>p.x),...waypoints.map(p=>p.x-p.radius))-.15,xmax=Math.max(...raw.map(p=>p.x),...preview.map(p=>p.x),...waypoints.map(p=>p.x+p.radius))+.15,ymin=Math.min(...raw.map(p=>p.y),...preview.map(p=>p.y),...waypoints.map(p=>p.y-p.radius))-.15,ymax=Math.max(...raw.map(p=>p.y),...preview.map(p=>p.y),...waypoints.map(p=>p.y+p.radius))+.15;
const scale=Math.min(760/(xmax-xmin),390/(ymax-ymin));const px=x=>70+(x-xmin)*scale,py=y=>475-(y-ymin)*scale;
const poly=a=>a.map(p=>`${px(p.x).toFixed(2)},${py(p.y).toFixed(2)}`).join(' ');
const errorMax=Math.max(primary.max_cm,1)*1.1,ex=s=>70+s/robot.length*760,ey=e=>790-e/errorMax*180;
let svg=`<svg xmlns="http://www.w3.org/2000/svg" width="920" height="850" viewBox="0 0 920 850"><rect width="920" height="850" fill="white"/><g font-family="Arial" fill="#172033"><text x="50" y="32" font-size="22">Logged robot path vs BLine GUI prediction</text><text x="50" y="58" font-size="14">${count} stored positions · same field coordinates · no fitted pose transform</text><text x="70" y="95" fill="#2563eb">Blue: GUI preview</text><text x="280" y="95" fill="#e05228">Orange: logged robot history</text><text x="580" y="95" fill="#dc2626">Red dashed: waypoint tolerance</text>`;
for(let x=Math.ceil(xmin*2)/2;x<=xmax;x+=.5)svg+=`<line x1="${px(x)}" y1="${py(ymin)}" x2="${px(x)}" y2="${py(ymax)}" stroke="#e5e7eb"/><text x="${px(x)-12}" y="${py(ymin)+24}" font-size="12">${x.toFixed(1)}</text>`;
for(let y=Math.ceil(ymin*2)/2;y<=ymax;y+=.5)svg+=`<line x1="${px(xmin)}" y1="${py(y)}" x2="${px(xmax)}" y2="${py(y)}" stroke="#e5e7eb"/><text x="${px(xmin)-34}" y="${py(y)+4}" font-size="12">${y.toFixed(1)}</text>`;
svg+=`<polyline points="${poly(preview)}" fill="none" stroke="#2563eb" stroke-width="3"/><polyline points="${poly(raw)}" fill="none" stroke="#e05228" stroke-width="2"/>`;
for(const p of waypoints.filter((p,i,a)=>a.findIndex(q=>q.x===p.x&&q.y===p.y&&q.radius===p.radius)===i))svg+=`<circle cx="${px(p.x)}" cy="${py(p.y)}" r="${p.radius*scale}" fill="none" stroke="#dc2626" stroke-width="2" stroke-dasharray="7 5"/>`;
svg+=`<text x="400" y="520" font-size="13">Field x (meters); y axis in meters</text><text x="50" y="558" font-size="20">Order-aligned deviation along traveled distance</text><text x="50" y="583" font-size="14">RMS ${primary.rms_cm.toFixed(2)} cm · P95 ${primary.p95_cm.toFixed(2)} cm · max ${primary.max_cm.toFixed(2)} cm</text>`;
for(let e=0;e<=errorMax;e+=10)svg+=`<line x1="70" y1="${ey(e)}" x2="830" y2="${ey(e)}" stroke="#e5e7eb"/><text x="32" y="${ey(e)+4}" font-size="12">${e}</text>`;
svg+=`<polyline points="${comparison.map(p=>`${ex(p.s).toFixed(2)},${ey(100*p.error).toFixed(2)}`).join(' ')}" fill="none" stroke="#e05228" stroke-width="2"/><text x="70" y="815" font-size="13">0 m</text><text x="770" y="815" font-size="13">${robot.length.toFixed(2)} m</text><text x="350" y="835" font-size="13">Robot traveled distance (meters); deviation axis in cm</text></g></svg>`;
if(simulationTrace)svg=svg.replace('Logged robot path vs BLine GUI prediction','Simulated robot path vs BLine GUI prediction')
  .replace('stored positions','saved simulation poses').replace('Orange: logged robot history','Orange: simulated robot path');
if(referenceIsSimulation&&!simulationTrace)svg=svg.replace('Logged robot path vs BLine GUI prediction','Recorded robot path vs simulation')
  .replace('Blue: GUI preview','Blue: simulation').replace('Orange: logged robot history','Orange: recorded robot history');
if(syntheticTrace&&referenceIsSimulation)svg=svg.replace('Simulated robot path vs BLine GUI prediction','Path vs new simulation')
  .replace('saved simulation poses','path points').replace('Blue: GUI preview','Blue: new simulation')
  .replace('Orange: simulated robot path','Orange: path');
if(simulationTrace&&referenceIsStraight)svg=svg.replace('Simulated robot path vs BLine GUI prediction','Simulation vs straight waypoint segments')
  .replace('Blue: GUI preview','Blue: straight segments');
fs.writeFileSync(path.join(output,'path-comparison.svg'),svg);
console.log(JSON.stringify(summary,null,2));
