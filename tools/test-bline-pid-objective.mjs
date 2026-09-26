import assert from 'node:assert/strict';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import {objective,improvementDecision,repeatedStats} from './bline-pid-objective.mjs';

const records=scores=>scores.map(score=>({score,failed:false}));
assert.equal(repeatedStats(records([2,3,4])).mean,3);
assert.equal(repeatedStats(records([2,3,4])).standard_deviation,1);
assert.equal(improvementDecision(records([3,3.1,2.9]),records([2.5,2.6,2.4])).accepted,true);
assert.equal(improvementDecision(records([3,3.1,2.9]),records([2.99,3,3.01])).accepted,false);
assert.equal(improvementDecision(records([2.7,3,3.3]),records([2.6,2.9,3.2])).accepted,false);
assert.equal(improvementDecision(records([3,3.1,2.9]),[{score:2,failed:true},...records([2,2])]).accepted,false);
assert.equal(improvementDecision(records([3,3]),records([2,2])).accepted,false);

const dir=fs.mkdtempSync(path.join(os.tmpdir(),'bline-objective-test-'));
fs.writeFileSync(path.join(dir,'robot-trace.csv'),'t,x,y,h,active,target,omega\n0,0,0,0,true,0,0.4\n0.02,0,0,0,true,0,-0.4\n0.04,0,0,0,false,0,0.4\n');
const summary={aligned_position_error_meters:{rms:.07,max:.21},aligned_absolute_heading_error_degrees:{rms:18,max:65},robot:{duration_seconds:6},gui:{duration_seconds:3}};
const score=objective(summary,dir);
assert.equal(score.chatter,1);
assert.deepEqual(score.parts,{position_rms:1,position_peak:.5,heading_rms:1,heading_peak:.5,timing:.25,chatter:.05});
assert.equal(score.score,3.3);
console.log('PASS: objective weights, chatter filtering, failures, and repeat/noise decision');
