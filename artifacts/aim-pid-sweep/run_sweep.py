from pathlib import Path
import subprocess,re,csv,json,shutil
p=Path('src/main/java/frc/robot/Constants.java');original=p.read_text()
out=Path('artifacts/aim-pid-sweep');results=[]
try:
 for kp,kd in [(1.2,.2),(2.0,.2),(3.0,.2),(4.0,.2),(6.0,.2),(3.0,0),(4.0,0),(6.0,0),(4.0,.5)]:
  s=re.sub(r'HUB_AIM_KP = [\d.]+;',f'HUB_AIM_KP = {kp};',original)
  s=re.sub(r'HUB_AIM_KD = [\d.]+;',f'HUB_AIM_KD = {kd};',s);p.write_text(s)
  dest=out/f'p{kp}-d{kd}';dest.mkdir(exist_ok=True)
  with (dest/'console.txt').open('w') as f:
   r=subprocess.run(['./gradlew','movingShotStressSimulation','--console=plain'],stdout=f,stderr=subprocess.STDOUT)
  src=Path('build/reports/moving-shot-stress-simulation')
  for f in src.glob('*'):
   if f.is_file():shutil.copy2(f,dest/f.name)
  rows=[x for x in csv.DictReader((dest/'summary.csv').open()) if x['scenario'] not in ['stationary','stationary-p-only-reference']]
  result=dict(p=kp,i=0,d=kd,passed=r.returncode==0,feed=sum(float(x['moving_feed_fraction']) for x in rows)/len(rows),shots=sum(int(x['moving_shots']) for x in rows),scored=sum(int(x['moving_scored']) for x in rows),heading_p95_mean=sum(float(x['moving_heading_error_p95_deg']) for x in rows)/len(rows))
  results.append(result);(out/'results.json').write_text(json.dumps(results,indent=2));print(result,flush=True)
finally:p.write_text(original)
