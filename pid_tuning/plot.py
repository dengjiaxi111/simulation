import argparse,json,math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from routes import distances
def math_angle(x):return math.atan2(math.sin(x),math.cos(x))
p=argparse.ArgumentParser();p.add_argument('file');a=p.parse_args();f=Path(a.file);data=json.loads(f.read_text());r=data['rows'];t=[x['t'] for x in r]
fig,axs=plt.subplots(2,2,figsize=(12,8));poses=np.array([x['pose'] for x in r]);axs[0,0].plot(poses[:,0],poses[:,1],label='Actual')
if 'path' in data:
 path=np.array(data['path']);axs[0,0].plot(path[:,0],path[:,1],'--',label='Original reference');axs[1,0].plot(t,distances(poses,path));axs[1,0].axhline(.1,color='r',linestyle='--');axs[1,0].set_ylabel('Cross-track error [m]')
 axs[0,1].plot(t,[np.linalg.norm(x['desired']) for x in r],label='Target');axs[0,1].plot(t,[np.linalg.norm(x['actual']) for x in r],label='Actual');axs[0,1].set_ylabel('Speed [m/s]')
for k in r[-1]['joints']:
 if 'steer' in k:axs[1,1].plot(t,[math_angle(x['targets'].get(k,x['joints'].get(k,[0])[0])-x['joints'].get(k,[0])[0]) for x in r],label=k.replace('_steer_joint',''))
axs[1,1].set_ylabel('Steer error [rad]')
for ax in axs.flat:ax.grid();ax.set_xlabel('Time [s]' if ax!=axs[0,0] else 'X [m]');ax.legend()
axs[0,0].axis('equal');fig.tight_layout();fig.savefig(f.with_suffix('.png'),dpi=140)
