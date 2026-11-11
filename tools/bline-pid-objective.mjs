import fs from 'node:fs';
import path from 'node:path';

/** Predeclared comparison objective; lower is better. Hard stopping checks precede this. */
export function objective(summary,dir) {
  const trace=fs.readFileSync(path.join(dir,'robot-trace.csv'),'utf8').trim().split('\n').slice(1).map(l=>l.split(','));
  let chatter=0;
  for(let i=1;i<trace.length;i++) {
    const a=trace[i-1],b=trace[i];
    if(b[4]!=='true')continue;
    const delta=Math.abs(Math.atan2(Math.sin((+b[5]-+a[5])*Math.PI/180),Math.cos((+b[5]-+a[5])*Math.PI/180)));
    if(delta<.035 && +a[6]*+b[6]<0 && Math.abs(+a[6])>.2 && Math.abs(+b[6])>.2)chatter++;
  }
  const parts={position_rms:summary.aligned_position_error_meters.rms/.07,
    position_peak:.5*summary.aligned_position_error_meters.max/.21,
    heading_rms:summary.aligned_absolute_heading_error_degrees.rms/18,
    heading_peak:.5*summary.aligned_absolute_heading_error_degrees.max/65,
    timing:.25*Math.abs(summary.robot.duration_seconds-summary.gui.duration_seconds)/3,
    chatter:.05*chatter};
  return {score:Object.values(parts).reduce((a,b)=>a+b,0),parts,chatter};
}

export function repeatedStats(records) {
  const mean=records.reduce((a,b)=>a+b.score,0)/records.length;
  const variance=records.length>1 ? records.reduce((a,b)=>a+(b.score-mean)**2,0)/(records.length-1) : Infinity;
  return {mean,standard_deviation:Math.sqrt(variance),samples:records.length};
}

/** Small-sample noise screen, not a formal confidence interval or hardware validation. */
export function improvementDecision(baseline,candidate) {
  const b=repeatedStats(baseline),c=repeatedStats(candidate);
  const gain=(b.mean-c.mean)/b.mean;
  const combinedStandardError=Math.sqrt(b.standard_deviation**2/b.samples+c.standard_deviation**2/c.samples);
  return {baseline:b,candidate:c,improvement_fraction:gain,combined_standard_error:combinedStandardError,
    accepted:b.samples>=3&&c.samples>=3&&!baseline.some(r=>r.failed)&&!candidate.some(r=>r.failed)&&gain>=.03&&(b.mean-c.mean)>combinedStandardError,
    rule:'All repeated runs complete/stop; >=3% lower mean score; mean-score advantage exceeds one combined standard error. This is only a small-sample noise screen.'};
}
