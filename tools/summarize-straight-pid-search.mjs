// Flattens one or more straight-line PID optimization searches into reviewable CSV tables.
import fs from 'node:fs';
import path from 'node:path';
const [outputDir,...searchDirs]=process.argv.slice(2);
if(!outputDir||!searchDirs.length)throw Error('Expected OUTPUT_DIR SEARCH_DIR...');
const rows=[];
for(const searchDir of searchDirs){
  const data=JSON.parse(fs.readFileSync(path.join(searchDir,'optimization-results.json'),'utf8'));
  for(const row of data.observations)rows.push({search:path.basename(searchDir),...row});
}
fs.mkdirSync(outputDir,{recursive:true});
const header=['search','number','label','translation_kp','translation_kd','cross_track_kp','cross_track_kd','failed','score','rms_cm','p95_cm','max_cm','duration_seconds'];
const value=row=>[row.search,row.number,row.label,...row.vector,row.failed,row.score,
  row.summary?100*row.summary.aligned_position_error_meters.rms:'',
  row.summary?100*row.summary.aligned_position_error_meters.p95:'',
  row.summary?100*row.summary.aligned_position_error_meters.max:'',
  row.summary?row.summary.robot.duration_seconds:''];
fs.writeFileSync(path.join(outputDir,'all-optimizer-runs.csv'),header.join(',')+'\n'+rows.map(row=>value(row).join(',')).join('\n')+'\n');
const groups=new Map();
for(const row of rows.filter(row=>!row.failed)){
  const key=JSON.stringify(row.vector);if(!groups.has(key))groups.set(key,[]);groups.get(key).push(row);
}
const aggregate=[...groups].map(([key,records])=>{
  const mean=field=>records.reduce((sum,row)=>sum+field(row),0)/records.length;
  return {vector:JSON.parse(key),runs:records.length,score:mean(row=>row.score),rms:mean(row=>100*row.summary.aligned_position_error_meters.rms),p95:mean(row=>100*row.summary.aligned_position_error_meters.p95),max:mean(row=>100*row.summary.aligned_position_error_meters.max)};
}).sort((a,b)=>a.score-b.score);
fs.writeFileSync(path.join(outputDir,'gain-set-averages.csv'),
  'translation_kp,translation_kd,cross_track_kp,cross_track_kd,runs,mean_score,mean_rms_cm,mean_p95_cm,mean_max_cm\n'
  +aggregate.map(row=>[...row.vector,row.runs,row.score,row.rms,row.p95,row.max].join(',')).join('\n')+'\n');
console.log(JSON.stringify({runs:rows.length,successful:rows.filter(row=>!row.failed).length,gain_sets:aggregate.length,best_repeated:aggregate.filter(row=>row.runs>=2).slice(0,5)},null,2));
