"""Plot immutable Nav2 initial plan and actual Gazebo trajectory, with speed traces."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from routes import distances
p=argparse.ArgumentParser();p.add_argument('file');a=p.parse_args();d=json.loads(Path(a.file).read_text());r=d['rows'];xy=np.array(d['path']);q=np.array([x['pose'] for x in r]);t=np.array([x['sim_time'] for x in r]);t-=t[0];v=np.array([x['velocity'] for x in r]);cmd=np.array([x['cmd'] or [0,0,0] for x in r]);e=distances(q,xy)
f,ax=plt.subplots(2,2,figsize=(12,8));ax[0,0].plot(*xy.T,label='Initial Nav2 plan');ax[0,0].plot(*q.T,label='Gazebo truth');ax[0,0].axis('equal');ax[0,0].legend();ax[0,0].set_xlabel('map x (m)');ax[0,0].set_ylabel('map y (m)');ax[0,1].plot(t,e);ax[0,1].axhline(.1,color='r',ls='--');ax[0,1].set_ylabel('Cross-track error (m)');ax[1,0].plot(t,np.linalg.norm(v[:,:2],axis=1),label='Actual');ax[1,0].plot(t,np.linalg.norm(cmd[:,:2],axis=1),label='Chassis command');ax[1,0].legend();ax[1,0].set_ylabel('Speed (m/s)');ax[1,1].plot(t,v[:,2]);ax[1,1].set_ylabel('Chassis angular speed (rad/s)')
for x in [ax[0,1],ax[1,0],ax[1,1]]:x.set_xlabel('Simulation time (s)');x.grid(True)
f.tight_layout();out=Path(a.file).with_suffix('.png');f.savefig(out,dpi=160);print(out)
