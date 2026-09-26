// Deterministic bounded coordinate/pattern search, using actual WPILib simulations.
// node tools/optimize-bline-pid.mjs OUTPUT_DIR GUI_PREVIEW_CSV
import fs from 'node:fs';
import path from 'node:path';
import {spawnSync} from 'node:child_process';
import {objective, improvementDecision} from './bline-pid-objective.mjs';
const [rootArg,preview]=process.argv.slice(2);
const root=path.resolve(rootArg);
if(fs.existsSync(root))throw Error('Use a fresh optimization output directory');
fs.mkdirSync(root,{recursive:true});
const names=['TRANSLATION_KP','TRANSLATION_KD','ROTATION_KP','ROTATION_KD','CROSS_TRACK_KP','CROSS_TRACK_KD'];
const bounds=[[3.5,8],[.15,.85],[2.8,8],[0,1],[0,8],[0,.15]];
const initial=[5,.5,4,.5,0,0];
const original=[5,.5,3,.3,5,0];
let step=[1,.15,1,.2,2,.05];
const budget=20;
const observations=[];
const cache=new Map();
let calls=0;
const clamp=(x,k)=>Math.round(Math.max(bounds[k][0],Math.min(bounds[k][1],x))*1e6)/1e6;
function save() {
  fs.writeFileSync(path.join(root,'optimization-results.json'),JSON.stringify({algorithm:'bounded coordinate polling plus pattern extrapolation; halved mesh on failed improvement',budget,bounds,names,observations},null,2)+'\n');
}
function evaluate(vector,label,repeat=false) {
  const v=vector.map(clamp),key=JSON.stringify(v);
  if(!repeat && cache.has(key))return cache.get(key);
  const number=++calls;
  const dir=path.join(root,String(number).padStart(3,'0')+'-'+label);
  const gains=Object.fromEntries(names.map((n,k)=>[n,v[k]]));
  const trialsFile=path.join(root,'input-'+number+'.json');
  fs.writeFileSync(trialsFile,JSON.stringify([{name:'run',gains}],null,2)+'\n');
  console.log('EVALUATION',number,label,JSON.stringify(gains));
  const run=spawnSync('node',['tools/tune-bline.mjs',trialsFile,dir,preview],{encoding:'utf8',timeout:150000,maxBuffer:20*1024*1024});
  fs.writeFileSync(path.join(root,'runner-'+number+'.txt'),(run.stdout??'')+(run.stderr??''));
  if(run.status!==0)throw Error('Evaluation runner failed: '+run.stderr);
  const result=JSON.parse(fs.readFileSync(path.join(dir,'trials-results.json'),'utf8'))[0];
  const record={number,label,vector:v,gains,directory:path.join(dir,'run'),failed:!!result.failed};
  if(!record.failed)Object.assign(record,objective(result.summary,record.directory),{summary:result.summary});
  else record.score=100;
  observations.push(record);
  if(!repeat)cache.set(key,record);
  save();
  console.log('RESULT',number,record.failed?'REJECTED':`score=${record.score.toFixed(4)} XY=${(record.summary.aligned_position_error_meters.rms*100).toFixed(2)}cm heading=${record.summary.aligned_absolute_heading_error_degrees.rms.toFixed(2)}deg peakHeading=${record.summary.aligned_absolute_heading_error_degrees.max.toFixed(2)}deg`);
  return record;
}
let current=evaluate(initial,'current-baseline');
const prior=evaluate(original,'original-baseline');
if(prior.score<current.score)current=prior;
let iteration=0;
while(calls<budget) {
  const old=current;
  // Full poll around a fixed center before choosing a new one (not greedy coordinate ordering).
  for(let k=0;k<names.length && calls<budget;k++)for(const sign of [1,-1]) {
    if(calls>=budget)break;
    const vector=[...old.vector];vector[k]+=sign*step[k];
    const candidate=evaluate(vector,`poll${iteration}-${names[k]}-${sign>0?'plus':'minus'}`);
    if(candidate.score<current.score)current=candidate;
  }
  if(current.number!==old.number && calls<budget) {
    const extrapolated=current.vector.map((v,k)=>v+(v-old.vector[k]));
    const candidate=evaluate(extrapolated,'pattern'+iteration);
    if(candidate.score<current.score)current=candidate;
  } else step=step.map(s=>s/2);
  iteration++;
  if(step.every(s=>s<.005))break;
}
// Three executions each for baseline and best two candidates. Search's lucky minima are not proof.
const finalists=[...observations].filter(o=>!o.failed).sort((a,b)=>a.score-b.score)
  .filter((o,i,a)=>a.findIndex(p=>JSON.stringify(p.vector)===JSON.stringify(o.vector))===i)
  .filter(o=>JSON.stringify(o.vector)!==JSON.stringify(initial)).slice(0,2);
const groups=[{name:'current-baseline',vector:initial},...finalists.map((o,i)=>({name:'finalist'+i,vector:o.vector}))];
const validation=[];
for(const group of groups) {
  const existing=observations.find(o=>JSON.stringify(o.vector)===JSON.stringify(group.vector));
  const records=[existing,evaluate(group.vector,group.name+'-repeat1',true),evaluate(group.vector,group.name+'-repeat2',true)];
  validation.push({...group,records:records.map(r=>r.number),failed:records.some(r=>r.failed),mean_score:records.reduce((a,b)=>a+b.score,0)/records.length});
}
const baseline=validation[0];
const best=validation.slice(1).filter(v=>!v.failed).sort((a,b)=>a.mean_score-b.mean_score)[0];
const screen=best ? improvementDecision(
  baseline.records.map(n=>observations.find(o=>o.number===n)),
  best.records.map(n=>observations.find(o=>o.number===n))) : {accepted:false};
const decision={validation,...screen,
  recommendation:screen.accepted ? best.vector : initial,
  note:'Hardware gains are not modified by this script; holdout tests precede any adoption.'};
fs.writeFileSync(path.join(root,'validation-decision.json'),JSON.stringify(decision,null,2)+'\n');
save();
console.log('DECISION',JSON.stringify(decision,null,2));
