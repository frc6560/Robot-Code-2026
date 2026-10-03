// node tools/tune-bline.mjs TRIALS_JSON OUTPUT_DIR PREVIEW_CSV
import fs from 'node:fs';
import path from 'node:path';
import { spawnSync } from 'node:child_process';
const [trialsFile, output, preview] = process.argv.slice(2);
const trials = JSON.parse(fs.readFileSync(trialsFile, 'utf8'));
fs.mkdirSync(output, {recursive:true});
const results = [];
for (const trial of trials) {
  const dir = path.resolve(output, trial.name);
  fs.mkdirSync(dir,{recursive:true});
  fs.copyFileSync(preview,path.join(dir,'gui-preview.csv'));
  const env={...process.env,BLINE_HEADLESS_DIAGNOSTIC:'1',BLINE_HEADLESS_AUTO:'editor',BLINE_COMPARE_LOG_DIR:path.join(dir,'logs')};
  for(const [name,gain] of Object.entries(trial.gains))env['BLINE_TUNE_'+name]=String(gain);
  console.log('Running',trial.name,trial.gains);
  const run=spawnSync('./gradlew',['simulateJava','-Pheadless'],{env,encoding:'utf8',timeout:120000,maxBuffer:20*1024*1024});
  fs.writeFileSync(path.join(dir,'simulation-console.txt'),(run.stdout??'')+(run.stderr??''));
  fs.writeFileSync(path.join(dir,'gains.json'),JSON.stringify(trial,null,2)+'\n');
  if(run.status!==0 || !run.stdout.includes('BLINE_HEADLESS_RESULT: PASS')) {
    results.push({...trial,failed:true}); console.log('FAILED',trial.name); continue;
  }
  const logs=fs.readdirSync(path.join(dir,'logs')).filter(f=>f.endsWith('.wpilog'));
  if(logs.length!==1)throw Error('Use fresh output folders: expected one log');
  for(const [exe,args] of [
    ['java',['-cp','build/libs/Robot-Code-2026.jar','tools/ExtractBLineTrace.java',path.join(dir,'logs',logs[0]),path.join(dir,'robot-trace.csv')]],
    ['node',['tools/compare-bline.mjs',dir]]
  ]) {
    const step=spawnSync(exe,args,{encoding:'utf8'});
    if(step.status!==0)throw Error(step.stderr);
    fs.writeFileSync(path.join(dir,exe==='java'?'extraction-console.txt':'comparison-console.txt'),step.stdout+step.stderr);
    if(exe==='java') {
      const effectiveGains=Object.fromEntries([...step.stdout.matchAll(/BLine\/Tuning\/([^ ]+) = ([^\r\n]+)/g)].map(match=>[match[1],Number(match[2])]));
      fs.writeFileSync(path.join(dir,'effective-gains.json'),JSON.stringify(effectiveGains,null,2)+'\n');
    }
  }
  const summary=JSON.parse(fs.readFileSync(path.join(dir,'comparison-summary.json'),'utf8'));
  // Explicit tradeoff: normalized geometry, rotation, and travel-time mismatch. Lower is better.
  const score=summary.aligned_position_error_meters.rms/.07
    +summary.aligned_absolute_heading_error_degrees.rms/18.1
    +Math.abs(summary.robot.duration_seconds-summary.gui.duration_seconds)/3.02;
  results.push({...trial,score,summary});
  console.log(trial.name,'score',score.toFixed(3),'XY rms',summary.aligned_position_error_meters.rms.toFixed(4),'heading rms',summary.aligned_absolute_heading_error_degrees.rms.toFixed(2),'duration',summary.robot.duration_seconds.toFixed(2));
  fs.writeFileSync(path.join(output,'trials-results.json'),JSON.stringify(results,null,2)+'\n');
}
fs.writeFileSync(path.join(output,'trials-results.json'),JSON.stringify(results,null,2)+'\n');
