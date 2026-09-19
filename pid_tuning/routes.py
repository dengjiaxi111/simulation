"""Arc-length sampled paths; dimensions are explicit, not presumed obstacle-free."""
import math
import numpy as np

def route(shape, size=8.0, spacing=0.025, width=None):
    u=np.linspace(0,1,1201); a=size/2
    if shape=='straight': x=size*u; y=0*u
    elif shape=='S': x=size*u; y=a*.4*np.sin(2*math.pi*u)
    elif shape=='J':
        x=np.where(u<.5,0,a*(1-np.cos(math.pi*(u-.5)*2)))
        y=np.where(u<.5,size*u,a+a*np.sin(math.pi*(u-.5)*2))
    elif shape in ('U','C'):
        th=math.pi*u if shape=='U' else 1.5*math.pi*u
        x=a*np.sin(th); y=a*(1-np.cos(th))
    elif shape in ('circle','ellipse'):
        th=2*math.pi*u; x=a*np.sin(th); y=a*(1-np.cos(th))*(.55 if shape=='ellipse' else 1)
    else: raise ValueError(shape)
    if width is not None:
        extent=float(np.ptp(x))
        if extent>0:x=x*width/extent
    p=np.column_stack((x,y)); ds=np.linalg.norm(np.diff(p,axis=0),axis=1); s=np.r_[0,np.cumsum(ds)]
    q=np.arange(0,s[-1],spacing);q=np.r_[q,s[-1]]
    return np.column_stack((np.interp(q,s,p[:,0]),np.interp(q,s,p[:,1])))

def cumulative(p):return np.r_[0,np.cumsum(np.linalg.norm(np.diff(p,axis=0),axis=1))]

def reference(p,s,d):
    i=min(len(p)-2,max(0,int(np.searchsorted(s,d)-1)))
    v=p[i+1]-p[i];return p[i]+v*max(0,min(1,(d-s[i])/(s[i+1]-s[i]))), v/np.linalg.norm(v)

def distances(points,path):
    """Exact distance to line segments, retaining every sample."""
    a=path[:-1];v=path[1:]-a;den=np.sum(v*v,axis=1)
    result=[]
    for q in points:
        t=np.clip(np.sum((q-a)*v,axis=1)/den,0,1)
        result.append(float(np.min(np.linalg.norm(q-a-t[:,None]*v,axis=1))))
    return np.array(result)
