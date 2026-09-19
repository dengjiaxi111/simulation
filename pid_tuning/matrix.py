"""Sequential full-route qualification; keep full traces and reject terminal-only success."""
import argparse,subprocess,sys,json,time
from pathlib import Path
from gains import source_gains,require_isolation
ROOT=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--gains',type=float,nargs=6);p.add_argument('--speeds',type=float,nargs='+',default=[2,3]);p.add_argument('--sizes',type=float,nargs='+',default=[14,28]);p.add_argument('--shapes',nargs='+',default=['straight','S','J','U','C','circle','ellipse']);p.add_argument('--repeats',type=int,default=1);a=p.parse_args();require_isolation();g=a.gains or source_gains();out=ROOT/'results'/time.strftime('matrix_%Y%m%d_%H%M%S');out.mkdir();results=[]
for repeat in range(a.repeats):
 for speed in a.speeds:
  for size in a.sizes:
   for shape in a.shapes:
    f=out/f'{repeat}_{shape}_{size}_{speed}.json';cmd=[sys.executable,str(ROOT/'trial.py'),'--shape',shape,'--size',str(size),'--speed',str(speed),'--acceleration','1.5','--gains',*map(str,g),'--output',str(f)]
    if shape!='straight':cmd+=['--width',str(size*15/28)]
    subprocess.run(cmd,check=True,timeout=150);s=json.loads(f.read_text())['summary'];results.append(s)
    (out/'summary.json').write_text(json.dumps(dict(gains=g,completed_tests=len(results),expected_tests=a.repeats*len(a.speeds)*len(a.sizes)*len(a.shapes),all_passed=all(x['passed'] for x in results),results=results),indent=2))
print('QUALIFIED',all(x['passed'] for x in results),'consecutive full-suite passes',a.repeats if all(x['passed'] for x in results) else 0)
