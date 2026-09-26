// node tools/review-bline-pid.mjs SEARCH_DIR ARTIFACT_DIR
// Runs independent fixtures after repeated phase-path validation, without editing constants.
import fs from 'node:fs';
import path from 'node:path';
import {spawnSync} from 'node:child_process';
import {improvementDecision,repeatedStats,objective} from './bline-pid-objective.mjs';
const [searchArg,artifactArg,runSuffix='']=process.argv.slice(2);
if(!/^[a-z0-9-]*$/.test(runSuffix))throw Error('Invalid holdout run suffix');
const search=path.resolve(searchArg),artifact=path.resolve(artifactArg);
const data=JSON.parse(fs.readFileSync(path.join(search,'optimization-results.json'),'utf8'));
if(!fs.existsSync(path.join(search,'validation-decision.json')))throw Error('Finish the search validation before review');
const baselineVector=[5,.5,4,.5,0,0];
const same=(a,b)=>JSON.stringify(a)===JSON.stringify(b);
const baseline=data.observations.filter(o=>same(o.vector,baselineVector));
// Select distinct non-baseline finalists from search evaluations, not from a lucky repeat.
const vectors=data.observations.filter(o=>o.number<=data.budget&&!o.failed&&!same(o.vector,baselineVector))
  .sort((a,b)=>a.score-b.score).filter((o,i,a)=>a.findIndex(p=>same(p.vector,o.vector))===i).slice(0,2).map(o=>o.vector);
const names=data.names;
for(let i=0;i<vectors.length;i++) {
  const vector=vectors[i];
  let count=data.observations.filter(o=>same(o.vector,vector)).length;
  while(count<3) {
    const number=Math.max(...data.observations.map(o=>o.number))+1;
    const dir=path.join(search,`review-finalist${i}-repeat${count}`);
    const trialsFile=path.join(search,`review-input-${number}.json`);
    const gains=Object.fromEntries(names.map((n,k)=>[n,vector[k]]));
    fs.writeFileSync(trialsFile,JSON.stringify([{name:'run',gains}],null,2)+'\n');
    console.log('DISTINCT FINALIST REPEAT',number,JSON.stringify(vector));
    const run=spawnSync('node',['tools/tune-bline.mjs',trialsFile,dir,path.join(data.observations[0].directory,'gui-preview.csv')],{encoding:'utf8',timeout:150000,maxBuffer:20*1024*1024});
    fs.writeFileSync(path.join(search,`review-runner-${number}.txt`),(run.stdout??'')+(run.stderr??''));
    if(run.status!==0)throw Error(run.stderr);
    const result=JSON.parse(fs.readFileSync(path.join(dir,'trials-results.json'),'utf8'))[0];
    const record={number,label:'distinct-finalist-review-repeat',vector,gains,directory:path.join(dir,'run'),failed:!!result.failed,score:100};
    if(!record.failed)Object.assign(record,objective(result.summary,record.directory),{summary:result.summary});
    data.observations.push(record);
    fs.writeFileSync(path.join(search,'optimization-results.json'),JSON.stringify(data,null,2)+'\n');
    count++;
  }
}
const candidates=vectors.map(vector=>{
  const records=data.observations.filter(o=>same(o.vector,vector));
  return {vector,records,screen:improvementDecision(baseline,records)};
}).sort((a,b)=>Number(b.screen.accepted)-Number(a.screen.accepted)||repeatedStats(a.records).mean-repeatedStats(b.records).mean);
if(!candidates.length)throw Error('No distinct validated finalist to review');
const candidate=candidates[0];
const fixtures=[{name:'straight',preview:'straight-preview'},{name:'profiled-corner',preview:'corner-preview'}];
const holdouts=[];
for(const fixture of fixtures) {
  const resultPair=[];
  for(const [label,vector] of [['baseline',baselineVector],['candidate',candidate.vector]]) {
    const dir=path.join(search,'holdout-'+fixture.name+'-'+label+runSuffix);
    const trialsFile=path.join(search,'holdout-input-'+fixture.name+'-'+label+runSuffix+'.json');
    fs.writeFileSync(trialsFile,JSON.stringify([{name:'run',gains:Object.fromEntries(names.map((n,k)=>[n,vector[k]]))}],null,2)+'\n');
    console.log('HOLDOUT',fixture.name,label,JSON.stringify(vector));
    const run=spawnSync('node',['tools/tune-bline.mjs',trialsFile,dir,path.join(artifact,'holdouts',fixture.preview,'gui-preview.csv')],{
      env:{...process.env,BLINE_COMPARE_AUTOS_DIR:path.join(artifact,'holdouts'),BLINE_COMPARE_PATH_NAME:fixture.name},encoding:'utf8',timeout:150000,maxBuffer:20*1024*1024});
    fs.writeFileSync(path.join(search,'holdout-runner-'+fixture.name+'-'+label+runSuffix+'.txt'),(run.stdout??'')+(run.stderr??''));
    if(run.status!==0)throw Error(run.stderr);
    const result=JSON.parse(fs.readFileSync(path.join(dir,'trials-results.json'),'utf8'))[0];
    resultPair.push({label,vector,directory:path.join(dir,'run'),...result,
      objective:result.failed?null:objective(result.summary,path.join(dir,'run'))});
  }
  const [b,c]=resultPair;
  const reasons=[];
  if(b.failed||c.failed)reasons.push('Baseline or candidate failed completion/stopping');
  else {
    const bs=b.summary,cs=c.summary;
    if(cs.aligned_position_error_meters.rms>Math.max(bs.aligned_position_error_meters.rms*1.15,bs.aligned_position_error_meters.rms+.02))reasons.push('XY RMS regression');
    if(cs.aligned_position_error_meters.max>bs.aligned_position_error_meters.max+.05)reasons.push('Peak XY regression');
    if(cs.aligned_absolute_heading_error_degrees.rms>Math.max(bs.aligned_absolute_heading_error_degrees.rms*1.15,bs.aligned_absolute_heading_error_degrees.rms+3))reasons.push('Heading RMS regression');
    if(cs.aligned_absolute_heading_error_degrees.max>bs.aligned_absolute_heading_error_degrees.max+10)reasons.push('Peak heading regression');
    if(cs.robot.duration_seconds>bs.robot.duration_seconds*1.2+.2)reasons.push('Duration regression');
  }
  holdouts.push({fixture:fixture.name,results:resultPair,passed:reasons.length===0,reasons});
  console.log('HOLDOUT RESULT',fixture.name,reasons.length?reasons:'PASS');
}
const accepted=candidate.screen.accepted&&holdouts.every(h=>h.passed);
const report={baseline_vector:baselineVector,candidate_vector:candidate.vector,phase_screen:candidate.screen,
  all_phase_finalists:candidates.map(c=>({vector:c.vector,records:c.records.map(o=>o.number),screen:c.screen})),
  holdouts,accepted,recommended_vector:accepted?candidate.vector:baselineVector,
  caveat:'Small-sample simulation comparison only; no physical-robot validation or automatic constants mutation.'};
fs.writeFileSync(path.join(artifact,'adoption-decision.json'),JSON.stringify(report,null,2)+'\n');
console.log('ADOPTION',JSON.stringify({accepted,recommended_vector:report.recommended_vector,phase_screen:candidate.screen},null,2));
