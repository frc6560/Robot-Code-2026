import fs from 'node:fs/promises';
import { Workbook, SpreadsheetFile } from '@oai/artifact-tool';
const dir='/Users/andrei/Robot-Code-2026/outputs/teleop-controls';
const w=Workbook.create(), s=w.worksheets.add('Teleop controls');
s.showGridLines=false;
s.getRange('A2').values=[['Teleop controller reference']];
s.getRange('A3').values=[['Current working code • October 30, 2026 • includes uncommitted changes']];
const rows=[
['Xbox · USB 1','Left stick Y','Move forward / backward','Move stick','Field-oriented, alliance-relative driving; 10% deadband.'],
['Xbox · USB 1','Left stick X','Strafe left / right','Move stick','Field-oriented, alliance-relative driving; 10% deadband.'],
['Xbox · USB 1','Right stick X','Rotate robot','Move stick','10% deadband; aimed shooting takes over rotation.'],
['Xbox · USB 1','Right trigger (RT)','Auto aim, spin up and shoot / pass','Hold','Aim toward hub; auto pass outside own alliance zone. Feed waits for alignment, RPM and other readiness checks.'],
['Xbox · USB 1','Right trigger (RT)','Stop aimed shooting','Release','Stops shot command and feeding; left trigger / board 3 must also be released before manual feed can restart.'],
['Xbox · USB 1','Left trigger (LT)','Run floor and tunnel indexers','Hold','Manual feed; shooter stays at idle RPM. Blocked during aimed shooting and pit coast. No aimed-shot readiness gate.'],
['Xbox · USB 1','Left bumper (LB)','Retract intake rack','Press','Selects retracted endpoint; takes priority if both bumpers are pressed. Disabled in pit coast.'],
['Xbox · USB 1','Right bumper (RB)','Extend intake rack; run rollers','Press / hold','Press selects extended endpoint and requests rollers; release stops rollers, rack stays extended. LB takes priority. Disabled in pit coast.'],
['Xbox · USB 1','A','Align robot for trench','Press','Turns toward a trench-aligned heading (0° or 180°); finishes within 2°. Takes drivetrain control during alignment.'],
['Xbox · USB 1','X','Reset pose using front camera','Press','Requests a hard vision pose reset from the front Limelight; teleop only.'],
['Xbox · USB 1','Start / Menu','Zero heading / gyro','Press','Sets odometry heading to 0° while retaining current position.'],
['Xbox · USB 1','Back / View','Toggle pit coast mode','Press to toggle','Zero-output coast mode lets mechanisms move by hand; press again to restore normal control.'],
['Xbox · USB 1','B; Y','No assigned action','—','No binding in current code.'],
['Xbox · USB 1','D-pad (all directions)','No assigned action','—','No binding in current code.'],
['Xbox · USB 1','Left / right stick click','No assigned action','—','No binding in current code.'],
['Xbox · USB 1','Right stick Y','No assigned action','—','Not used by current drive bindings.'],
['Button board · USB 0','Button 3','Run floor and tunnel indexers','Hold','Same manual feed as LT; shooter stays at idle RPM. Both controls must release after aimed shooting to rearm.'],
['Button board · USB 0','Other buttons (including 5)','No active binding','—','Button 5 has a helper method, but that method is not connected to an action.']
];
s.getRange('A5:E5').values=[['Controller','Control','Action','How to use','Behavior / conditions']];
s.getRange(`A6:E${5+rows.length}`).values=rows;
s.getRange('A2:E23').format.font={name:'Arial',size:11,color:'#172033'};
s.getRange('A2').format.font={name:'Arial',size:16,bold:true};
s.getRange('A3').format.font={name:'Arial',size:10,color:'#586474'};
s.getRange('A5:E5').format={fill:'#253B59',font:{name:'Arial',size:11,bold:true,color:'#FFFFFF'},horizontalAlignment:'center',verticalAlignment:'center',rowHeight:28};
for(const [c,width] of [['A',23],['B',27],['C',36],['D',19],['E',75]]) s.getRange(`${c}1:${c}28`).format.columnWidth=width;
s.getRange('A6:E23').format.wrapText=true;
s.getRange('A6:E23').format.verticalAlignment='center';
s.getRange('A6:E23').format.rowHeight=61;
for(let r=6;r<=23;r++) if(r%2===0) s.getRange(`A${r}:E${r}`).format.fill='#EFF3F8';
s.getRange('A25').values=[['Driving while RT is held: translation is limited to 1.2 m/s; driver can still translate while automatic aim controls rotation.']];
s.getRange('A26').values=[['Default: shooter idles at 1,000 RPM; indexers stop unless commanded. Pit coast is cleared before teleop / autonomous starts.']];
s.getRange('A28').values=[['Sources: RobotContainer.java; ManualControls.java; Constants.java; FuelShootCommand.java; SwerveSubsystem.java (src/main/java/frc/robot/).']];
s.getRange('A25:A28').format.font={name:'Arial',size:10,color:'#586474'};
s.freezePanes.freezeRows(5);
w.recalculate();
console.log((await w.inspect({kind:'table',range:"'Teleop controls'!A5:E8",tableMaxRows:4,tableMaxCols:5})).ndjson);
const p=await w.render({sheetName:s.name,range:'A1:E28',scale:1,format:'png'});
await fs.writeFile(`${dir}/preview.png`,new Uint8Array(await p.arrayBuffer()));
await (await SpreadsheetFile.exportXlsx(w)).save(`${dir}/Teleop-controls.xlsx`);
