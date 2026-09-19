"""Use the existing plugin interface; never edit a production configuration."""
import subprocess, math
from pathlib import Path
import xml.etree.ElementTree as ET
KEYS=['wheel_kp','wheel_ki','wheel_kd','steer_kp','steer_ki','steer_kd']
def source_gains():
    f=Path(__file__).resolve().parents[1]/'src/seu_sentry_description/resource/xmacro/new_seu_sentry_sim.sdf.xmacro'
    root=ET.fromstring(f.read_text());p=root.find(".//plugin[@filename='libsentry_motor_controller.so']")
    if p is None:
        p=next(p for p in root.findall('.//plugin') if p.find('wheel_kp') is not None)
    return [float(p.find(k).text) for k in KEYS]
def apply(g):
    if len(g)!=6 or any(not math.isfinite(x) or x<0 or x>100 for x in g): raise ValueError('invalid gains')
    subprocess.run(['gz','topic','-t','/simulation/motor_gains','-m','gz.msgs.Double_V','-p','data: ['+', '.join(map(str,g))+']'],check=True,timeout=5,capture_output=True)
    # Existing plugin has no ACK. This is a request, not verified readback.

def require_isolation():
    import os
    if os.environ.get('ROS_DOMAIN_ID')!='77' or os.environ.get('GZ_PARTITION')!='seu_pid_tuning':
        raise RuntimeError('Actuation requires ROS_DOMAIN_ID=77 GZ_PARTITION=seu_pid_tuning')
def reset_pose():
    require_isolation()
    subprocess.run(['gz','service','-s','/world/default/set_pose','--reqtype','gz.msgs.Pose','--reptype','gz.msgs.Boolean','--timeout','3000','--req','name: "sentry" position {x: 4.5 y: 11 z: 0.65} orientation {w: 1}'],check=True,timeout=5,capture_output=True)
