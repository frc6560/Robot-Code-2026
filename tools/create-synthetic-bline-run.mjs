// Creates an explicitly labeled synthetic path for demonstration; never a replacement for robot logs.
import fs from 'node:fs';
import path from 'node:path';
import assert from 'node:assert/strict';
import {createHash} from 'node:crypto';

const [newSimulationFile,oldPositionsFile,oldDeviationFile,pathFile,outputFile,maxErrorArg='.19']=process.argv.slice(2);
assert(outputFile,'Expected NEW_SIM OLD_POSITIONS OLD_DEVIATION PATH_JSON OUTPUT_CSV [MAX_ERROR_METERS]');
const maxError=Number(maxErrorArg);
assert(Number.isFinite(maxError)&&maxError>0&&maxError<=.5);

function readCsv(file){
  const [header,...lines]=fs.readFileSync(file,'utf8').trim().split(/\r?\n/);
  const keys=header.split(',');
  return lines.map(line=>Object.fromEntries(line.split(',').map((value,i)=>[keys[i],value])));
}
function sha(file){return createHash('sha256').update(fs.readFileSync(file)).digest('hex');}
function cumulative(points){
  const s=[0];
  for(let i=1;i<points.length;i++)s.push(s.at(-1)+Math.hypot(points[i].x-points[i-1].x,points[i].y-points[i-1].y));
  return s;
}
function interpolateByS(points,sValues,s){
  let lo=0,hi=sValues.length-1;
  while(lo+1<hi){const mid=(lo+hi)>>1;if(sValues[mid]<=s)lo=mid;else hi=mid;}
  const span=sValues[hi]-sValues[lo];
  const f=span>0?(s-sValues[lo])/span:0;
  const angleDelta=Math.atan2(Math.sin(points[hi].heading-points[lo].heading),Math.cos(points[hi].heading-points[lo].heading));
  return {
    x:points[lo].x+f*(points[hi].x-points[lo].x),
    y:points[lo].y+f*(points[hi].y-points[lo].y),
    heading:points[lo].heading+f*angleDelta,
    time:points[lo].time+f*(points[hi].time-points[lo].time),
    tx:points[hi].x-points[lo].x,
    ty:points[hi].y-points[lo].y
  };
}
function interpolateSeries(rows,key,s){
  let lo=0,hi=rows.length-1;
  while(lo+1<hi){const mid=(lo+hi)>>1;if(rows[mid].s<=s)lo=mid;else hi=mid;}
  const span=rows[hi].s-rows[lo].s,f=span>0?(s-rows[lo].s)/span:0;
  return rows[lo][key]+f*(rows[hi][key]-rows[lo][key]);
}

const allSimulation=readCsv(newSimulationFile);
const firstActive=allSimulation.findIndex(row=>row.auto_active==='true');
const lastActive=allSimulation.findLastIndex(row=>row.auto_active==='true');
assert(firstActive>=0&&lastActive>=firstActive,'No active simulation interval');
assert(allSimulation.slice(firstActive,lastActive+1).every(row=>row.auto_active==='true'),'Multiple active intervals');
const simulation=allSimulation.slice(firstActive,lastActive+1).map(row=>({
  time:Number(row.time_seconds),x:Number(row.x_meters),y:Number(row.y_meters),heading:Number(row.heading_radians),element:row.translation_element_index
}));
const simS=cumulative(simulation),simLength=simS.at(-1);

const oldPositions=readCsv(oldPositionsFile).map(row=>({x:Number(row.x_meters),y:Number(row.y_meters)}));
assert(oldPositions.length>2&&oldPositions.every(p=>Number.isFinite(p.x)&&Number.isFinite(p.y)));
const oldS=cumulative(oldPositions),oldLength=oldS.at(-1);

const deviations=readCsv(oldDeviationFile).map(row=>({
  s:Number(row.robot_distance_meters),
  rx:Number(row.robot_x_meters)-Number(row.matched_gui_x_meters),
  ry:Number(row.robot_y_meters)-Number(row.matched_gui_y_meters),
  gx:Number(row.matched_gui_x_meters),gy:Number(row.matched_gui_y_meters)
}));
for(let i=0;i<deviations.length;i++){
  const a=deviations[Math.max(0,i-1)],b=deviations[Math.min(deviations.length-1,i+1)];
  const tx=b.gx-a.gx,ty=b.gy-a.gy,cross=tx*deviations[i].ry-ty*deviations[i].rx;
  deviations[i].signed=Math.sign(cross||1)*Math.hypot(deviations[i].rx,deviations[i].ry);
}

// Replace the isolated middle spike with a smooth bridge across a 0.70 m window.
const peakIndex=deviations.reduce((best,row,i)=>Math.abs(row.signed)>Math.abs(deviations[best].signed)?i:best,0);
const originalPeakError=Math.abs(deviations[peakIndex].signed);
const peakS=deviations[peakIndex].s,halfWindow=.35;
const left=Math.max(0,deviations.findLastIndex(row=>row.s<=peakS-halfWindow));
const right=Math.min(deviations.length-1,deviations.findIndex(row=>row.s>=peakS+halfWindow));
const leftValue=deviations[left].signed,rightValue=deviations[right].signed;
for(let i=left+1;i<right;i++){
  const f=(deviations[i].s-deviations[left].s)/(deviations[right].s-deviations[left].s);
  const eased=f*f*(3-2*f);
  deviations[i].signed=leftValue+eased*(rightValue-leftValue);
}

const output=[];
for(let i=0;i<oldPositions.length;i++){
  const progress=oldLength>0?oldS[i]/oldLength:0;
  const base=interpolateByS(simulation,simS,progress*simLength);
  const tangentLength=Math.hypot(base.tx,base.ty)||1;
  const nx=-base.ty/tangentLength,ny=base.tx/tangentLength;
  let offset=interpolateSeries(deviations,'signed',progress*deviations.at(-1).s);
  offset=Math.max(-maxError,Math.min(maxError,offset));
  const edgeTaper=Math.min(1,progress/.02,(1-progress)/.02);
  offset*=Math.max(0,edgeTaper);
  output.push({
    time:base.time,x:base.x+nx*offset,y:base.y+ny*offset,baseX:base.x,baseY:base.y,heading:base.heading,
    sourceIndex:i,progress,offset,waypointCorrection:false
  });
}

// Guarantee that every repeated waypoint occurrence, not merely every unique coordinate, enters
// the 20 cm tolerance. If sparse sampling misses one, blend a correction over neighboring samples.
function segmentDistance(point,a,b){
  const dx=b.x-a.x,dy=b.y-a.y;
  const u=Math.max(0,Math.min(1,((point.x-a.x)*dx+(point.y-a.y)*dy)/(dx*dx+dy*dy||1)));
  return Math.hypot(a.x+u*dx-point.x,a.y+u*dy-point.y);
}
const pathModel=JSON.parse(fs.readFileSync(pathFile,'utf8'));
const pathWaypoints=pathModel.path_elements.filter(element=>element.translation_target).map(element=>({
  x:element.translation_target.x_meters,y:element.translation_target.y_meters,
  tolerance:element.translation_target.intermediate_handoff_radius_meters
}));
const elementIds=[...new Set(simulation.map(row=>row.element))];
assert.equal(elementIds.length,pathWaypoints.length-1,'Simulation/path waypoint occurrence mismatch');
const waypointChecks=[];
for(let occurrence=0;occurrence<elementIds.length;occurrence++){
  const elementRows=simulation.filter(row=>row.element===elementIds[occurrence]);
  const startTime=elementRows[0].time-.05,endTime=elementRows.at(-1).time+.08;
  const waypoint=pathWaypoints[occurrence+1];
  const segmentIndices=[];
  let before=Infinity;
  for(let i=0;i<output.length-1;i++)if(output[i+1].time>=startTime&&output[i].time<=endTime){
    segmentIndices.push(i);before=Math.min(before,segmentDistance(waypoint,output[i],output[i+1]));
  }
  assert(segmentIndices.length>0,'No mock samples around waypoint occurrence '+(occurrence+1));
  let movedIndex=null;
  if(before>waypoint.tolerance-.02){
    const pointIndices=[...new Set(segmentIndices.flatMap(i=>[i,i+1]))];
    movedIndex=pointIndices.reduce((best,i)=>
      Math.hypot(output[i].x-waypoint.x,output[i].y-waypoint.y)<Math.hypot(output[best].x-waypoint.x,output[best].y-waypoint.y)?i:best,
      pointIndices[0]);
    const row=output[movedIndex],dx=row.x-waypoint.x,dy=row.y-waypoint.y,length=Math.hypot(dx,dy)||1;
    const targetX=waypoint.x+.10*dx/length,targetY=waypoint.y+.10*dy/length;
    const correctionX=targetX-row.x,correctionY=targetY-row.y,window=4;
    for(let d=-window;d<=window;d++){
      const i=movedIndex+d;
      if(i<0||i>=output.length)continue;
      const weight=.5*(1+Math.cos(Math.PI*d/(window+1)));
      output[i].x+=weight*correctionX;
      output[i].y+=weight*correctionY;
      output[i].waypointCorrection=true;
    }
  }
  let after=Infinity;
  for(const i of segmentIndices)after=Math.min(after,segmentDistance(waypoint,output[i],output[i+1]));
  assert(after<=waypoint.tolerance,'Synthetic path misses waypoint occurrence '+(occurrence+1));
  waypointChecks.push({occurrence:occurrence+1,waypoint_x_m:waypoint.x,waypoint_y_m:waypoint.y,
    tolerance_m:waypoint.tolerance,before_m:before,after_m:after,moved_source_history_index:movedIndex===null?null:output[movedIndex].sourceIndex});
}

for(const row of output)row.offset=Math.hypot(row.x-row.baseX,row.y-row.baseY);

fs.mkdirSync(path.dirname(path.resolve(outputFile)),{recursive:true});
const note='SYNTHETIC_MOCK_NOT_ROBOT_LOG';
const header='time_seconds,x_meters,y_meters,heading_radians,auto_active,synthetic_mock,source_history_index,normalized_path_progress,synthetic_position_offset_meters,waypoint_correction_applied,provenance';
const lines=output.map(row=>[
  row.time.toFixed(6),row.x.toFixed(9),row.y.toFixed(9),row.heading.toFixed(9),'true','true',
  row.sourceIndex,row.progress.toFixed(9),row.offset.toFixed(9),row.waypointCorrection,note
].join(','));
fs.writeFileSync(outputFile,header+'\n'+lines.join('\n')+'\n');
const offsets=output.map(row=>Math.abs(row.offset));
const sorted=[...offsets].sort((a,b)=>a-b),q=.95*(sorted.length-1),q0=Math.floor(q);
const metadata={
  data_kind:note,
  warning:'Demonstration-only synthetic data derived from an old estimated-pose history and a new WPILib simulation. It is not evidence of robot performance.',
  output_csv:path.resolve(outputFile),
  sources:{
    new_simulation:{path:path.resolve(newSimulationFile),sha256:sha(newSimulationFile)},
    old_reconstructed_positions:{path:path.resolve(oldPositionsFile),sha256:sha(oldPositionsFile)},
    old_deviation_profile:{path:path.resolve(oldDeviationFile),sha256:sha(oldDeviationFile)}
  },
  method:'Mapped the old run along normalized traveled distance to the new simulation, retained the old signed cross-track deviation profile, replaced the isolated peak with smooth interpolation across a 0.70 m window, capped applied offsets, and applied a cosine-blended local correction wherever a repeated waypoint occurrence would otherwise miss its tolerance.',
  points:output.length,
  old_path_length_m:oldLength,
  new_simulation_path_length_m:simLength,
  removed_spike:{old_peak_progress:peakS/deviations.at(-1).s,old_peak_error_m:originalPeakError,bridge_start_m:deviations[left].s,bridge_end_m:deviations[right].s},
  waypoint_occurrence_checks:waypointChecks,
  applied_offset_summary_m:{rms:Math.sqrt(offsets.reduce((sum,value)=>sum+value*value,0)/offsets.length),p95:sorted[q0]+(q-q0)*(sorted[Math.min(q0+1,sorted.length-1)]-sorted[q0]),max:Math.max(...offsets)},
  max_requested_offset_m:maxError
};
fs.writeFileSync(outputFile.replace(/\.csv$/i,'-metadata.json'),JSON.stringify(metadata,null,2)+'\n');
fs.writeFileSync(outputFile.replace(/\.csv$/i,'-waypoint-checks.csv'),
  'waypoint_occurrence,waypoint_x_meters,waypoint_y_meters,tolerance_cm,before_cm,after_cm,moved_source_history_index\n'
  +waypointChecks.map(row=>[row.occurrence,row.waypoint_x_m,row.waypoint_y_m,100*row.tolerance_m,
    100*row.before_m,100*row.after_m,row.moved_source_history_index??''].join(',')).join('\n')+'\n');
console.log(JSON.stringify(metadata,null,2));
