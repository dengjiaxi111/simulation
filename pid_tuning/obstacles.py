"""Add static boxes to isolated Gazebo only; coordinates explicit in world frame."""
import argparse,subprocess,json
from pathlib import Path
from gains import require_isolation
p=argparse.ArgumentParser();p.add_argument('--world',default='default');p.add_argument('--x',type=float,required=True);p.add_argument('--y',type=float,required=True);p.add_argument('--name',default='pid_tuning_obstacle');p.add_argument('--width',type=float,default=1);a=p.parse_args();require_isolation()
sdf=f'<sdf version="1.9"><model name="{a.name}"><static>true</static><pose>{a.x} {a.y} 1.108656 0 0 0</pose><link name="box"><collision name="c"><geometry><box><size>{a.width} {a.width} 1.5</size></box></geometry></collision><visual name="v"><geometry><box><size>{a.width} {a.width} 1.5</size></box></geometry></visual></link></model></sdf>'
from google.protobuf.text_format import MessageToString
# Protobuf string quoting via JSON is valid for this ASCII SDF payload.
r=subprocess.run(['gz','service','-s',f'/world/{a.world}/create','--reqtype','gz.msgs.EntityFactory','--reptype','gz.msgs.Boolean','--timeout','3000','--req','sdf: '+json.dumps(sdf)],capture_output=True,text=True,check=True,timeout=5)
if 'true' not in r.stdout:raise RuntimeError('Obstacle insertion was not confirmed: '+r.stdout+r.stderr)
f=Path(__file__).resolve().parent/'results/obstacles.json';v=json.loads(f.read_text()) if f.exists() else [];v.append(dict(name=a.name,x=a.x,y=a.y,width=a.width));f.write_text(json.dumps(v,indent=2));print(r.stdout)
