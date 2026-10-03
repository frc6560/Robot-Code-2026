// node tools/summarize-bline-tuning.mjs OUTPUT_DIR TRIAL_DIR...
import fs from 'node:fs';
import path from 'node:path';
const [out,...sources]=process.argv.slice(2);
fs.mkdirSync(out,{recursive:true});
const trials=sources.flatMap(source=>JSON.parse(fs.readFileSync(path.join(source,'trials-results.json'),'utf8')).map(trial=>{
  const consoleText=fs.readFileSync(path.join(source,trial.name,'simulation-console.txt'),'utf8');
  const diagnostic=consoleText.match(/BLINE_HEADLESS_RESULT: ([^\r\n]+)/)?.[1] ?? 'MISSING';
  return {...trial,source:path.resolve(source,trial.name),diagnostic};
}));
const ranked=trials.filter(t=>!t.failed).sort((a,b)=>a.score-b.score);
fs.writeFileSync(path.join(out,'all-trials.json'),JSON.stringify(trials,null,2)+'\n');
const cols=['name','translation_p','translation_d','rotation_p','rotation_d','cross_track_p','cross_track_d','status','position_rms_m','position_max_m','heading_rms_deg','heading_max_deg','duration_s','score'];
const rows=trials.map(t=>[t.name,t.gains.TRANSLATION_KP,t.gains.TRANSLATION_KD,t.gains.ROTATION_KP,t.gains.ROTATION_KD,t.gains.CROSS_TRACK_KP,t.gains.CROSS_TRACK_KD,t.diagnostic.split(' ')[0],t.summary?.aligned_position_error_meters.rms??'',t.summary?.aligned_position_error_meters.max??'',t.summary?.aligned_absolute_heading_error_degrees.rms??'',t.summary?.aligned_absolute_heading_error_degrees.max??'',t.summary?.robot.duration_seconds??'',t.score??'']);
fs.writeFileSync(path.join(out,'all-trials.csv'),cols.join(',')+'\n'+rows.map(r=>r.join(',')).join('\n')+'\n');
console.log('Trials:',trials.length,'completed/stopped:',ranked.length);
console.log('Best scored trial:',JSON.stringify(ranked[0],null,2));
