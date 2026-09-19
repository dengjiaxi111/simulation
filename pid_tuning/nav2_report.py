"""Aggregate measured full-algorithm tests without declaring incomparable routes best."""
import json,math
from pathlib import Path
import numpy as np
root=Path(__file__).resolve().parent/'results';out=[]
for f in sorted(root.glob('*.json')):
 try:d=json.loads(f.read_text());s=d.get('summary',{})
 except Exception:continue
 if s.get('stage')!='full_navigationros2_with_LIO':continue
 rows=d['rows'];ages=[r['tf_age'] for r in rows if r.get('tf_age') is not None];stats={}
 for side in ['front_left','front_right','rear_left','rear_right']:
  w=[];a=[];lim=[]
  for r in rows:
   j=r.get('joints',{});g=r.get('targets',{});k=side+'_steer_joint';v=j.get(k)
   if v:lim.append(abs(v[1])>=19.5)
   if v and k in g:a.append(abs(math.atan2(math.sin(g[k]-v[0]),math.cos(g[k]-v[0]))))
   k=side+'_wheel_joint';v=j.get(k)
   if v and k in g:w.append(abs(g[k]-v[1]))
  stats[side]=dict(steer_speed_limit_fraction=float(np.mean(lim)) if lim else None,steer_error_p95_rad=float(np.quantile(a,.95)) if a else None,wheel_speed_error_p95_rad_s=float(np.quantile(w,.95)) if w else None)
 item=dict(file=f.name,summary=s,tf_age_median=float(np.median(ages)) if ages else None,joint_statistics=stats);out.append(item)
(root/'nav2_report.json').write_text(json.dumps(out,indent=2));print(json.dumps(out[-2:],indent=2))
