// Bounded deterministic search for translation/cross-track gains against straight waypoint segments.
// Rotation gains remain at the checked-in values because heading is not part of this geometry target.
import fs from 'node:fs';
import path from 'node:path';
import {spawnSync} from 'node:child_process';

const [rootArg,referenceFile,budgetArg='20',startArg,stepArg]=process.argv.slice(2);
if(!rootArg||!referenceFile)throw Error('Expected OUTPUT_DIR STRAIGHT_REFERENCE_CSV [SEARCH_BUDGET]');
const root=path.resolve(rootArg),budget=Number(budgetArg);
if(fs.existsSync(root))throw Error('Use a fresh optimization output directory');
if(!Number.isInteger(budget)||budget<8||budget>60)throw Error('Budget must be an integer from 8 to 60');
fs.mkdirSync(root,{recursive:true});

const names=['TRANSLATION_KP','TRANSLATION_KD','CROSS_TRACK_KP','CROSS_TRACK_KD'];
const bounds=[[1.5,10],[0,1.5],[0,15],[0,2]];
const initial=[5,.5,5,0];
const searchStart=startArg?JSON.parse(startArg):initial;
let step=stepArg?JSON.parse(stepArg):[1.5,.25,3,.35];
if(searchStart.length!==names.length||step.length!==names.length)throw Error('Start and step vectors must contain four values');
const observations=[],cache=new Map();
let calls=0;
const clamp=(value,k)=>Math.round(Math.max(bounds[k][0],Math.min(bounds[k][1],value))*1e6)/1e6;
const same=(a,b)=>JSON.stringify(a)===JSON.stringify(b);
function geometryObjective(summary){
  const p=summary.aligned_position_error_meters;
  const parts={rms:p.rms,p95_weighted:.35*p.p95,peak_weighted:.15*p.max,endpoint_weighted:.1*summary.endpoint_position_difference_meters};
  return {score:Object.values(parts).reduce((a,b)=>a+b,0),parts};
}
function save(){
  fs.writeFileSync(path.join(root,'optimization-results.json'),JSON.stringify({
    algorithm:'seeded bounded coordinate polling with pattern extrapolation and mesh reduction',
    target:'ordered straight-line segments connecting every waypoint occurrence',
    score:'XY RMS + 0.35*P95 + 0.15*max + 0.10*endpoint; completion/stopping is mandatory',
    budget,bounds,names,observations
  },null,2)+'\n');
}
function evaluate(vector,label,repeat=false){
  const v=vector.map(clamp),key=JSON.stringify(v);
  if(!repeat&&cache.has(key))return cache.get(key);
  const number=++calls,dir=path.join(root,String(number).padStart(3,'0')+'-'+label);
  const gains=Object.fromEntries(names.map((name,k)=>[name,v[k]]));
  const input=path.join(root,'input-'+number+'.json');
  fs.writeFileSync(input,JSON.stringify([{name:'run',gains}],null,2)+'\n');
  console.log('EVALUATION',number,label,JSON.stringify(gains));
  let actualDir=dir;
  let run=spawnSync('node',['tools/tune-bline.mjs',input,actualDir,referenceFile],{encoding:'utf8',timeout:150000,maxBuffer:30*1024*1024});
  let resultsFile=path.join(actualDir,'trials-results.json');
  if(run.status!==0||!fs.existsSync(resultsFile)){
    const retryDir=dir+'-retry';
    const retry=spawnSync('node',['tools/tune-bline.mjs',input,retryDir,referenceFile],{encoding:'utf8',timeout:150000,maxBuffer:30*1024*1024});
    fs.writeFileSync(path.join(root,'runner-'+number+'.txt'),(run.stdout??'')+(run.stderr??'')+'\n--- RETRY ---\n'+(retry.stdout??'')+(retry.stderr??''));
    run=retry;actualDir=retryDir;resultsFile=path.join(actualDir,'trials-results.json');
  }else fs.writeFileSync(path.join(root,'runner-'+number+'.txt'),(run.stdout??'')+(run.stderr??''));
  if(run.status!==0||!fs.existsSync(resultsFile))throw Error('Evaluation runner and retry failed: '+run.stderr);
  const result=JSON.parse(fs.readFileSync(resultsFile,'utf8'))[0];
  const record={number,label,vector:v,gains,directory:path.join(actualDir,'run'),failed:!!result.failed,score:100};
  if(!record.failed)Object.assign(record,geometryObjective(result.summary),{summary:result.summary});
  observations.push(record);if(!repeat)cache.set(key,record);save();
  console.log('RESULT',number,record.failed?'REJECTED':`score=${record.score.toFixed(5)} RMS=${(100*record.summary.aligned_position_error_meters.rms).toFixed(2)}cm P95=${(100*record.summary.aligned_position_error_meters.p95).toFixed(2)}cm max=${(100*record.summary.aligned_position_error_meters.max).toFixed(2)}cm duration=${record.summary.robot.duration_seconds.toFixed(2)}s`);
  return record;
}

let current=evaluate(initial,'baseline');
if(!same(searchStart,initial)&&calls<budget){const candidate=evaluate(searchStart,'search-start');if(candidate.score<current.score)current=candidate;}
for(const [label,vector] of startArg?[]:[
  ['high-cross-damped',[5,.5,10,.35]],
  ['slower-damped-high-cross',[3.5,.85,12,.6]],
  ['responsive-cross',[6,.25,9,.15]]
])if(calls<budget){const candidate=evaluate(vector,label);if(candidate.score<current.score)current=candidate;}

let iteration=0;
while(calls<budget){
  const center=current;
  for(let k=0;k<names.length&&calls<budget;k++)for(const sign of [1,-1]){
    if(calls>=budget)break;
    const vector=[...center.vector];vector[k]+=sign*step[k];
    const candidate=evaluate(vector,`poll${iteration}-${names[k]}-${sign>0?'plus':'minus'}`);
    if(candidate.score<current.score)current=candidate;
  }
  if(current.number!==center.number&&calls<budget){
    const vector=current.vector.map((value,k)=>value+(value-center.vector[k]));
    const candidate=evaluate(vector,`pattern${iteration}`);
    if(candidate.score<current.score)current=candidate;
  }else step=step.map(value=>value/2);
  iteration++;
}

const unique=[...observations].filter(row=>!row.failed).sort((a,b)=>a.score-b.score)
  .filter((row,i,all)=>all.findIndex(other=>same(other.vector,row.vector))===i);
const finalists=[{name:'baseline',vector:initial},...unique.filter(row=>!same(row.vector,initial)).slice(0,2).map((row,i)=>({name:'finalist'+i,vector:row.vector}))];
const validation=[];
for(const finalist of finalists){
  const first=observations.find(row=>same(row.vector,finalist.vector));
  const records=[first,evaluate(finalist.vector,finalist.name+'-repeat1',true),evaluate(finalist.vector,finalist.name+'-repeat2',true)];
  const scores=records.map(row=>row.score),mean=scores.reduce((a,b)=>a+b,0)/scores.length;
  const sd=Math.sqrt(scores.reduce((sum,value)=>sum+(value-mean)**2,0)/(scores.length-1));
  validation.push({...finalist,records:records.map(row=>row.number),failed:records.some(row=>row.failed),mean_score:mean,standard_deviation:sd,
    mean_rms_m:records.reduce((sum,row)=>sum+(row.summary?.aligned_position_error_meters.rms??1),0)/records.length});
}
const baseline=validation[0],best=validation.slice(1).filter(row=>!row.failed).sort((a,b)=>a.mean_score-b.mean_score)[0];
const improvement=best?(baseline.mean_score-best.mean_score)/baseline.mean_score:0;
const accepted=!!best&&!baseline.failed&&improvement>=.03&&(baseline.mean_score-best.mean_score)>Math.sqrt(baseline.standard_deviation**2/3+best.standard_deviation**2/3);
const decision={validation,accepted,improvement_fraction:improvement,recommended_vector:accepted?best.vector:initial,
  recommended_gains:Object.fromEntries(names.map((name,k)=>[name,(accepted?best.vector:initial)[k]])),
  fixed_rotation_gains:{ROTATION_KP:3,ROTATION_KD:.3},
  rule:'Three completed/stopped runs each; >=3% lower mean geometry score; advantage exceeds one combined standard error.',
  caveat:'WPILib simulation optimization only; not physical-robot validation.'};
fs.writeFileSync(path.join(root,'validation-decision.json'),JSON.stringify(decision,null,2)+'\n');save();
console.log('DECISION',JSON.stringify(decision,null,2));
