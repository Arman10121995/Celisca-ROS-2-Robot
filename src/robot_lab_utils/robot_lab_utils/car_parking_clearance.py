"""Conservative swept convex footprint checks in a published Nav2 grid.

The polygon comes from the selected robot's navigation configuration. Grid
cell squares (including unknowns) intersecting the swept footprint block it.
This is a local command guard, not a replacement for Nav2 path planning.
"""
import math
import numpy as np


def footprint_points(values):
    points=np.asarray(values,dtype=float).reshape(-1,2)
    if not 3<=len(points)<=16 or not np.all(np.isfinite(points)):
        raise ValueError('A finite convex source footprint with 3–16 corners is required')
    edges=np.roll(points,-1,axis=0)-points
    following=np.roll(edges,-1,axis=0)
    turns=edges[:,0]*following[:,1]-edges[:,1]*following[:,0]
    if not (np.all(turns>1e-9) or np.all(turns<-1e-9)):
        raise ValueError('Footprint must be strictly convex with nonzero area')
    return points


def footprint_clear(grid,polygon,padding=.02):
    """Check the entire polygon against occupied/unknown cell squares."""
    width,height=int(grid.info.width),int(grid.info.height)
    resolution=float(grid.info.resolution)
    if width<=0 or height<=0 or width*height>4_000_000 or not math.isfinite(resolution) or resolution<=0:
        raise ValueError('Invalid or oversized parking costmap')
    data=np.asarray(grid.data,dtype=int)
    if data.size!=width*height:raise ValueError('Costmap dimensions/data differ')
    origin=grid.info.origin;q=origin.orientation
    if not all(map(math.isfinite,(q.w,q.x,q.y,q.z))) or abs(q.x)+abs(q.y)>1e-6 or abs(q.w*q.w+q.z*q.z-1)>1e-3:
        raise ValueError('Costmap origin needs a normalized planar rotation')
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    c,s=math.cos(yaw),math.sin(yaw)
    points=(np.asarray(polygon)-[origin.position.x,origin.position.y])@np.array([[c,-s],[s,c]])/resolution
    if not np.all(np.isfinite(points)):raise ValueError('Nonfinite footprint/grid origin')
    margin=padding/resolution
    low=np.floor(points.min(axis=0)-margin).astype(int);high=np.floor(points.max(axis=0)+margin).astype(int)
    if np.any(low<0) or high[0]>=width or high[1]>=height:return False
    cells=data.reshape(height,width)[low[1]:high[1]+1,low[0]:high[0]+1]
    rows,cols=np.where((cells<0)|(cells>=99))
    if not len(rows):return True
    centers=np.column_stack((cols+low[0]+.5,rows+low[1]+.5))
    edges=np.roll(points,-1,axis=0)-points
    axes=np.vstack((np.eye(2),np.column_stack((-edges[:,1],edges[:,0]))))
    projections=points@axes.T;centre=centers@axes.T
    radius=.5*np.abs(axes).sum(axis=1)
    extra=margin*np.linalg.norm(axes,axis=1)
    separated=(centre+radius<projections.min(axis=0)-extra-1e-9)|(centre-radius>projections.max(axis=0)+extra+1e-9)
    return bool(np.all(np.any(separated,axis=1)))


def swept_clear(grid,footprint,pose,vx,wz,horizon=.8):
    """Constant-command sweep, including present pose; <=1 cm corner steps."""
    points=footprint_points(footprint);x,y,yaw=map(float,pose)
    if not all(map(math.isfinite,(x,y,yaw,vx,wz,horizon))) or horizon<=0:
        raise ValueError('Finite pose, command and positive horizon required')
    corner_speed=abs(vx)+abs(wz)*float(np.linalg.norm(points,axis=1).max())
    steps=max(1,math.ceil(corner_speed*horizon/.01))
    if steps>128:raise ValueError('Parking command exceeds sweep budget')
    for t in np.linspace(0.,horizon,steps+1):
        angle=yaw+wz*t
        if abs(wz)<1e-9:px,py=x+vx*t*math.cos(yaw),y+vx*t*math.sin(yaw)
        else:px,py=x+vx/wz*(math.sin(angle)-math.sin(yaw)),y-vx/wz*(math.cos(angle)-math.cos(yaw))
        c,s=math.cos(angle),math.sin(angle)
        if not footprint_clear(grid,points@np.array([[c,s],[-s,c]])+[px,py]):return False
    return True
