// Verifies repeated Zigzag simulation traces against the start-Y and field-border limits.
import fs from 'node:fs';
import path from 'node:path';
import assert from 'node:assert/strict';

const [pathFile,outputDir,...traceFiles]=process.argv.slice(2);
assert(outputDir&&traceFiles.length,'Expected PATH_JSON OUTPUT_DIR TRACE_CSV...');
const fieldTopY=8.07,footprintLength=.812,footprintWidth=.812;
function readCsv(file){
  const [header,...lines]=fs.readFileSync(file,'utf8').trim().split(/\r?\n/),keys=header.split(',');
  return lines.map(line=>Object.fromEntries(line.split(',').map((value,i)=>[keys[i],value])));
}
const model=JSON.parse(fs.readFileSync(pathFile,'utf8'));
const waypoints=model.path_elements.filter(element=>element.translation_target).map(element=>({
  x:element.translation_target.x_meters,y:element.translation_target.y_meters,
  radius:element.translation_target.intermediate_handoff_radius_meters
}));
const runs=traceFiles.map((file,index)=>{
  const rows=readCsv(file).map(row=>({t:+row.time_seconds,x:+row.x_meters,y:+row.y_meters,h:+row.heading_radians,active:row.auto_active==='true'}));
  const active=rows.filter(row=>row.active);assert(active.length>1,'No active samples in '+file);
  const start=active[0],lastActive=rows.findLastIndex(row=>row.active),post=rows.slice(lastActive+1);
  const top=row=>row.y+Math.abs(Math.sin(row.h))*footprintLength/2+Math.abs(Math.cos(row.h))*footprintWidth/2;
  const maxActive=active.reduce((a,b)=>b.y>a.y?b:a),maxAll=rows.reduce((a,b)=>b.y>a.y?b:a);
  const maxTopActive=active.reduce((a,b)=>top(b)>top(a)?b:a),maxTopAll=rows.reduce((a,b)=>top(b)>top(a)?b:a);
  let length=0;for(let i=1;i<active.length;i++)length+=Math.hypot(active[i].x-active[i-1].x,active[i].y-active[i-1].y);
  return {run:index+1,file:path.resolve(file),rows,active,start,maxActive,maxAll,maxTopActive,maxTopAll,
    maxTopActiveY:top(maxTopActive),maxTopAllY:top(maxTopAll),
    postMaxY:post.length?Math.max(...post.map(row=>row.y)):null,
    duration:active.at(-1).t-start.t,length,
    endError:Math.hypot(active.at(-1).x-waypoints.at(-1).x,active.at(-1).y-waypoints.at(-1).y)};
});
const startY=runs[0].start.y;
for(const run of runs)assert(Math.abs(run.start.y-startY)<1e-9,'Runs have different start Y');
const summaries=runs.map(run=>({
  run:run.run,active_samples:run.active.length,duration_s:run.duration,path_length_m:run.length,start_y_m:startY,
  max_active_center_y_m:run.maxActive.y,active_center_overshoot_cm:100*(run.maxActive.y-startY),
  max_all_center_y_m:run.maxAll.y,all_center_overshoot_cm:100*(run.maxAll.y-startY),
  max_bumper_top_y_m:run.maxTopAllY,field_border_margin_cm:100*(fieldTopY-run.maxTopAllY),
  endpoint_error_cm:100*run.endError,pass_center_y:run.maxAll.y<=startY+1e-9,pass_field_border:run.maxTopAllY<fieldTopY
}));
fs.mkdirSync(outputDir,{recursive:true});
const headers=Object.keys(summaries[0]);
fs.writeFileSync(path.join(outputDir,'run-summary.csv'),headers.join(',')+'\n'+summaries.map(row=>headers.map(key=>row[key]).join(',')).join('\n')+'\n');
const aggregate={path:path.resolve(pathFile),requirement:'Robot center Y must never exceed its starting Y.',field_top_y_m:fieldTopY,
  footprint_assumption_m:{length:footprintLength,width:footprintWidth},start_y_m:startY,
  waypoint_max_y_m:Math.max(...waypoints.map(row=>row.y)),runs:summaries,
  worst_center_overshoot_cm:Math.max(...summaries.map(row=>row.all_center_overshoot_cm)),
  minimum_field_border_margin_cm:Math.min(...summaries.map(row=>row.field_border_margin_cm)),
  all_runs_pass_center_y:summaries.every(row=>row.pass_center_y),all_runs_pass_field_border:summaries.every(row=>row.pass_field_border),
  caveat:'Simulation evidence only. The 0.812 m footprint comes from RobotProfile.DEFAULT and must be replaced with measured bumper dimensions if different.'};
fs.writeFileSync(path.join(outputDir,'boundary-summary.json'),JSON.stringify(aggregate,null,2)+'\n');

const all=[...waypoints,...runs.flatMap(run=>run.active)];
const width=980,height=690,pad={left:70,right:35,top:80,bottom:90};
const xmin=Math.min(...all.map(row=>row.x))-.25,xmax=Math.max(...all.map(row=>row.x))+.25;
const ymin=Math.min(...all.map(row=>row.y))-.2,ymax=fieldTopY+.08;
const sx=x=>pad.left+(x-xmin)/(xmax-xmin)*(width-pad.left-pad.right);
const sy=y=>height-pad.bottom-(y-ymin)/(ymax-ymin)*(height-pad.top-pad.bottom);
const colors=['#e05228','#7c3aed','#059669'];
const poly=rows=>rows.map(row=>`${sx(row.x).toFixed(2)},${sy(row.y).toFixed(2)}`).join(' ');
let svg=`<svg xmlns="http://www.w3.org/2000/svg" width="${width}" height="${height}" viewBox="0 0 ${width} ${height}"><rect width="${width}" height="${height}" fill="white"/><g font-family="Arial" fill="#172033"><text x="50" y="30" font-size="22">Zigzag simulation: upper-boundary check</text><text x="50" y="54" font-size="14">Requirement: center Y never exceeds start Y = ${startY.toFixed(5)} m · ${runs.length} repeated runs</text>`;
for(let x=Math.ceil(xmin*2)/2;x<=xmax;x+=.5)svg+=`<line x1="${sx(x)}" y1="${pad.top}" x2="${sx(x)}" y2="${height-pad.bottom}" stroke="#e5e7eb"/><text x="${sx(x)}" y="${height-pad.bottom+22}" text-anchor="middle" font-size="12">${x.toFixed(1)}</text>`;
for(let y=Math.ceil(ymin*2)/2;y<=ymax;y+=.5)svg+=`<line x1="${pad.left}" y1="${sy(y)}" x2="${width-pad.right}" y2="${sy(y)}" stroke="#e5e7eb"/><text x="${pad.left-12}" y="${sy(y)+4}" text-anchor="end" font-size="12">${y.toFixed(1)}</text>`;
svg+=`<line x1="${pad.left}" y1="${sy(fieldTopY)}" x2="${width-pad.right}" y2="${sy(fieldTopY)}" stroke="#991b1b" stroke-width="3"/><text x="${width-pad.right-5}" y="${sy(fieldTopY)-8}" text-anchor="end" fill="#991b1b" font-size="13">Field boundary Y = ${fieldTopY.toFixed(2)} m</text>`;
svg+=`<line x1="${pad.left}" y1="${sy(startY)}" x2="${width-pad.right}" y2="${sy(startY)}" stroke="#2563eb" stroke-width="2" stroke-dasharray="8 5"/><text x="${width-pad.right-5}" y="${sy(startY)-8}" text-anchor="end" fill="#2563eb" font-size="13">Starting center Y</text>`;
for(const waypoint of waypoints.filter((row,i,all)=>all.findIndex(other=>other.x===row.x&&other.y===row.y)===i))svg+=`<circle cx="${sx(waypoint.x)}" cy="${sy(waypoint.y)}" r="${waypoint.radius*(width-pad.left-pad.right)/(xmax-xmin)}" fill="none" stroke="#dc2626" stroke-width="1.5" stroke-dasharray="6 4"/>`;
runs.forEach((run,i)=>{svg+=`<polyline points="${poly(run.active)}" fill="none" stroke="${colors[i]}" stroke-width="${i?2:3}" opacity="${i?.75:1}"/><text x="${80+i*135}" y="76" fill="${colors[i]}" font-size="13">Run ${i+1}</text>`;});
svg+=`<text x="${width/2}" y="${height-14}" text-anchor="middle" font-size="13">Field X (m); Field Y increases upward</text><text x="50" y="${height-36}" font-size="13">Worst center overshoot: ${aggregate.worst_center_overshoot_cm.toFixed(2)} cm · minimum estimated bumper-to-border margin: ${aggregate.minimum_field_border_margin_cm.toFixed(1)} cm</text></g></svg>`;
fs.writeFileSync(path.join(outputDir,'boundary-check.svg'),svg);
console.log(JSON.stringify(aggregate,null,2));
