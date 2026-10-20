"""Distinct PX4 identities and bounded 3D group kernels (no plant pose writes)."""
import math
import numpy as np


def identities(count):
    if type(count) is not int or not 2 <= count <= 4:
        raise ValueError('Aerial sessions support 2–4 complete X500/FCU instances')
    return [dict(id='air'+str(i+1),instance=i,system_id=i+1,port=14580+i,
                 entity='x500_'+str(i),namespace='/swarm/air'+str(i+1)) for i in range(count)]


def velocities(mode, positions, spacing=3., anchor=None, maximum=.4):
    if mode not in ('formation','rendezvous','dispersion','flocking') or not 2 <= len(positions) <= 4:
        raise ValueError('Select a supported 3D swarm kernel and 2–4 fresh members')
    if not math.isfinite(spacing) or not 2. <= spacing <= 8.:
        raise ValueError('Aerial spacing must be 2–8 m')
    names=sorted(positions);p=np.asarray([positions[n] for n in names],dtype=float)
    if p.shape!=(len(names),3) or not np.isfinite(p).all():raise ValueError('Fresh finite XYZ positions required')
    centre=np.mean(p,axis=0);anchor=centre if anchor is None else np.asarray(anchor,dtype=float)
    if anchor.shape!=(3,) or not np.isfinite(anchor).all():raise ValueError('Finite XYZ anchor required')
    result={}
    for index,name in enumerate(names):
        if mode=='formation':
            target=anchor+np.array([0.,(index-(len(names)-1)/2)*spacing,0.])
            v=.5*(target-p[index])
        elif mode=='rendezvous':v=.3*(centre-p[index])
        elif mode=='dispersion':v=np.zeros(3)
        else:v=.15*(centre-p[index])+np.array([.2,0.,0.])
        for other in range(len(names)):
            if other==index:continue
            delta=p[index]-p[other];distance=float(np.linalg.norm(delta))
            if distance<1e-6:raise ValueError('Coincident aerial estimates; Hold before restarting')
            if distance<spacing:
                v+=(spacing-distance)*delta/distance*.5
        speed=float(np.linalg.norm(v))
        if speed>maximum:v*=maximum/speed
        result[name]=v.tolist()
    return result
