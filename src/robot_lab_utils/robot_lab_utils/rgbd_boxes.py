"""Known-size upright box targets from color pixels and aligned metric depth.

No simulator object pose is an input. Appearance/size/upright assumptions are declared
configuration, not semantic recognition or a general six-DOF pose estimator.
"""
import math

import numpy as np
from scipy.spatial.transform import Rotation


def bindings(document):
    if not isinstance(document,dict) or document.get('schema')!='robot_lab.rgbd_boxes.v1':
        raise ValueError('Choose an rgbd_boxes.v1 configuration')
    rows=document.get('boxes')
    if not isinstance(rows,list) or not 1<=len(rows)<=16:raise ValueError('Configure 1–16 boxes')
    result={}
    for row in rows:
        if not isinstance(row,dict):raise ValueError('Each box needs a binding')
        name=row.get('object');size=np.asarray(row.get('size_m'),dtype=float)
        low=np.asarray(row.get('hsv_low'),dtype=float);high=np.asarray(row.get('hsv_high'),dtype=float)
        if not isinstance(name,str) or not 1<=len(name)<=200 or name in result:
            raise ValueError('Each box needs a unique named object')
        if size.shape!=(3,) or not np.all(np.isfinite(size)) or np.any(size<.01) or np.any(size>1.):
            raise ValueError('Known box dimensions must be 0.01–1 m')
        if (low.shape!=(3,) or high.shape!=(3,) or not np.all(np.isfinite([low,high]))
                or np.any(low<0) or np.any(high>[179,255,255]) or np.any(high<low)
                or np.any(low!=np.floor(low)) or np.any(high!=np.floor(high))):
            raise ValueError('Use ordered integer OpenCV HSV bounds')
        noise=float(row.get('depth_noise_m',.001))
        if not math.isfinite(noise) or not .0001<=noise<=.003:
            raise ValueError('Declared depth noise must be 0.1–3 mm')
        for old in result.values():
            if np.all(high>=old['hsv_low']) and np.all(np.asarray(old['hsv_high'])>=low):
                raise ValueError('Named box color ranges must not overlap')
        result[name]=dict(object=name,size_m=size.tolist(),hsv_low=low.astype(int).tolist(),
                          hsv_high=high.astype(int).tolist(),depth_noise_m=noise)
    return result


def estimate_box(rgb,depth,k,distortion,world_from_camera,binding):
    import cv2
    rgb=np.asarray(rgb);depth=np.asarray(depth);k=np.asarray(k,dtype=float)
    world=np.asarray(world_from_camera,dtype=float);distortion=np.asarray(distortion,dtype=float)
    if (rgb.ndim!=3 or rgb.shape[2]!=3 or rgb.dtype!=np.uint8 or depth.shape!=rgb.shape[:2]
            or not 16<=rgb.shape[0]<=960 or not 16<=rgb.shape[1]<=1280):
        raise ValueError('Bounded aligned RGB8 and metric depth images required')
    if (k.shape!=(3,3) or not np.all(np.isfinite(k)) or min(k[0,0],k[1,1])<=0
            or not np.all(np.isfinite(distortion)) or world.shape!=(4,4)
            or not np.all(np.isfinite(world)) or not np.allclose(world[3],[0,0,0,1])
            or not np.allclose(world[:3,:3].T@world[:3,:3],np.eye(3),atol=1e-6)
            or not np.isclose(np.linalg.det(world[:3,:3]),1.,atol=1e-6)):
        raise ValueError('Calibrated camera and rigid world transform required')
    hsv=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
    mask=cv2.inRange(hsv,np.asarray(binding['hsv_low'],dtype=np.uint8),np.asarray(binding['hsv_high'],dtype=np.uint8))
    mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),dtype=np.uint8))
    count,labels,stats,_=cv2.connectedComponentsWithStats(mask)
    candidates=[i for i in range(1,count) if stats[i,cv2.CC_STAT_AREA]>=80]
    if len(candidates)!=1:raise ValueError('Need exactly one visible object of the configured color')
    label=candidates[0];x,y,width,height,_=stats[label]
    if min(width,height)<12 or x<2 or y<2 or x+width>rgb.shape[1]-2 or y+height>rgb.shape[0]-2:
        raise ValueError('Object too small or clipped by the image boundary')
    ys,xs=np.nonzero(labels==label);values=depth[ys,xs].astype(float)
    valid=np.isfinite(values)&(values>=.08)&(values<=3.)
    if np.mean(valid)<.9:raise ValueError('Too many missing/out-of-range object depths')
    xs,ys,values=xs[valid],ys[valid],values[valid]
    stride=max(1,math.ceil(len(values)/20000));xs,ys,values=xs[::stride],ys[::stride],values[::stride]
    rays=cv2.undistortPoints(np.column_stack((xs,ys)).astype(float).reshape(-1,1,2),k,distortion).reshape(-1,2)
    camera=np.column_stack((rays,np.ones(len(rays))))*values[:,None]
    points=camera@world[:3,:3].T+world[:3,3]
    noise=binding['depth_noise_m'];band=max(.0015,3*noise)
    top_mask=np.abs(points[:,2]-np.percentile(points[:,2],95))<=band
    top=points[top_mask]
    if len(top)<50:raise ValueError('Insufficient visible top-face depth points')
    pixel_hull=cv2.convexHull(np.column_stack((xs[top_mask],ys[top_mask])).astype(np.float32))
    if len(top)*stride<.7*cv2.contourArea(pixel_hull):raise ValueError('Top-face color coverage is incomplete')
    center=top.mean(axis=0);_,singular,vectors=np.linalg.svd(top-center,full_matrices=False)
    if singular[1]<.002 or singular[2]>.18*singular[1] or abs(vectors[-1,2])<.995:
        raise ValueError('A complete upright planar top face is required')
    rectangle=cv2.minAreaRect(top[:,:2].astype(np.float32))
    hull=cv2.convexHull(top[:,:2].astype(np.float32))
    measured=np.asarray(rectangle[1]);known=np.asarray(binding['size_m'],dtype=float)
    if np.min(measured)<=0:raise ValueError('Degenerate top-face rectangle')
    sides=cv2.boxPoints(rectangle).astype(float)
    edge=sides[1]-sides[0];other=sides[2]-sides[1]
    observed=np.array([np.linalg.norm(edge),np.linalg.norm(other)])
    if abs(observed[0]-known[0])+abs(observed[1]-known[1])>abs(observed[1]-known[0])+abs(observed[0]-known[1]):
        edge=other;observed=observed[::-1]
    tolerance=np.maximum(.003,known[:2]*.12)
    if np.any(np.abs(observed-known[:2])>tolerance):raise ValueError('Visible rectangle does not match known box dimensions')
    if cv2.contourArea(hull)<.7*float(known[0]*known[1]):raise ValueError('Top face is incomplete or occluded')
    # A rectangle has pi yaw symmetry; a square has pi/2 symmetry. Report
    # the canonical representative instead of claiming observable face labels.
    square=abs(known[0]-known[1])<1e-6;period=math.pi/2 if square else math.pi
    yaw=(math.atan2(edge[1],edge[0])+period/2)%period-period/2
    position=[float(rectangle[0][0]),float(rectangle[0][1]),float(np.median(top[:,2])-known[2]/2)]
    pixel_bound=float(np.max(values)/min(k[0,0],k[1,1]))
    bound=3*noise+2*pixel_bound+float(np.max(np.abs(observed-known[:2])))/2+float(np.std(top[:,2]))
    if not math.isfinite(bound) or bound>.02:raise ValueError('Declared target uncertainty exceeds 2 cm')
    return dict(object=binding['object'],type='box',size=known.tolist(),position=position,
        orientation_wxyz=Rotation.from_euler('z',yaw).as_quat()[[3,0,1,2]].tolist(),
        pose_reference='collision_box_center',position_std_bound_m=bound,
        observed_top_size_m=observed.tolist(),top_points=len(top),yaw_symmetry_order=4 if square else 2,
        scope='Known-size upright colored box; declared depth/pixel bound, not calibrated covariance')
