// Generates a dense, order-preserving straight-line reference through every waypoint occurrence.
import fs from 'node:fs';
import path from 'node:path';
import assert from 'node:assert/strict';
import {createHash} from 'node:crypto';

const [pathFile,outputFile,spacingArg='.02',speedArg='2.5']=process.argv.slice(2);
assert(outputFile,'Expected PATH_JSON OUTPUT_CSV [SPACING_METERS] [REFERENCE_SPEED_MPS]');
const spacing=Number(spacingArg),speed=Number(speedArg);
assert(Number.isFinite(spacing)&&spacing>=.005&&spacing<=.1);
assert(Number.isFinite(speed)&&speed>0);
const raw=fs.readFileSync(pathFile,'utf8'),model=JSON.parse(raw);
const points=model.path_elements.filter(element=>element.translation_target).map(element=>({
  x:element.translation_target.x_meters,
  y:element.translation_target.y_meters,
  heading:element.rotation_target?.rotation_radians??0
}));
assert(points.length>=2);
const rows=[],segments=[];
let distance=0;
for(let segment=0;segment<points.length-1;segment++){
  const a=points[segment],b=points[segment+1];
  const length=Math.hypot(b.x-a.x,b.y-a.y),count=Math.max(1,Math.ceil(length/spacing));
  const headingDelta=Math.atan2(Math.sin(b.heading-a.heading),Math.cos(b.heading-a.heading));
  const startRow=rows.length;
  for(let i=0;i<count;i++){
    const u=i/count;
    rows.push({time:distance/speed,x:a.x+u*(b.x-a.x),y:a.y+u*(b.y-a.y),heading:a.heading+u*headingDelta,segment});
    distance+=length/count;
  }
  segments.push({segment,start_waypoint:segment,end_waypoint:segment+1,length_m:length,start_row:startRow,end_row:rows.length});
}
const last=points.at(-1);
rows.push({time:distance/speed,x:last.x,y:last.y,heading:last.heading,segment:points.length-2});
fs.mkdirSync(path.dirname(path.resolve(outputFile)),{recursive:true});
fs.writeFileSync(outputFile,'time_seconds,x_meters,y_meters,heading_radians,straight_reference,segment_index\n'
  +rows.map(row=>[row.time,row.x,row.y,row.heading,'true',row.segment].join(',')).join('\n')+'\n');
const metadata={kind:'ordered_straight_waypoint_polyline',source_path:path.resolve(pathFile),source_path_sha256:createHash('sha256').update(raw).digest('hex'),spacing_m:spacing,reference_speed_mps:speed,waypoint_occurrences:points.length,segments,samples:rows.length,total_length_m:distance,duration_s:distance/speed};
fs.writeFileSync(outputFile.replace(/\.csv$/i,'-metadata.json'),JSON.stringify(metadata,null,2)+'\n');
console.log(JSON.stringify(metadata,null,2));
