"""Camera-only marker pose estimation and explicit marker-to-object geometry.

OpenCV supplies detection and IPPE PnP. Source/scene IDs associate an estimate
with a task; no simulator object pose is an input to these calculations.
"""
import math
import numpy as np
from scipy.spatial.transform import Rotation


def bindings(document):
    if not isinstance(document,dict) or document.get('schema')!='robot_lab.object_markers.v1':
        raise ValueError('Choose an object_markers.v1 configuration')
    if document.get('dictionary','DICT_4X4_50')!='DICT_4X4_50':
        raise ValueError('This adapter uses the reviewed DICT_4X4_50 dictionary')
    rows=document.get('markers')
    if not isinstance(rows,list) or not 1<=len(rows)<=64:raise ValueError('Configure 1–64 marker bindings')
    out={};objects=set()
    for row in rows:
        if not isinstance(row,dict):raise ValueError('Marker binding must be an object')
        number=row.get('id');name=row.get('object')
        if isinstance(number,bool) or not isinstance(number,int) or not 0<=number<50 or number in out:
            raise ValueError('Marker IDs must be unique integers in [0, 50)')
        if not isinstance(name,str) or not name or len(name)>200 or name in objects:
            raise ValueError('Each marker needs a unique named object')
        size=float(row['marker_size_m']);dimensions=np.asarray(row['object_size_m'],dtype=float)
        pose=np.asarray(row.get('object_from_marker',[0.,0.,0.,1.,0.,0.,0.]),dtype=float)
        if not math.isfinite(size) or not .005<=size<=1. or dimensions.shape!=(3,) or not np.all(np.isfinite(dimensions)) or np.any(dimensions<=0):
            raise ValueError('Finite positive physical marker/object dimensions required')
        if pose.shape!=(7,) or not np.all(np.isfinite(pose)) or not .99<=np.linalg.norm(pose[3:])<=1.01:
            raise ValueError('object_from_marker is [xyz, unit wxyz]')
        out[number]=dict(id=number,object=name,marker_size_m=size,object_size_m=dimensions.tolist(),object_from_marker=pose.tolist())
        objects.add(name)
    return out


def marker_pose(corners,size,k,distortion):
    import cv2
    pixels=np.asarray(corners,dtype=float).reshape(4,2);k=np.asarray(k,dtype=float).reshape(3,3)
    d=np.asarray(distortion,dtype=float)
    if not np.all(np.isfinite(pixels)) or not np.all(np.isfinite(k)) or not np.all(np.isfinite(d)) or min(k[0,0],k[1,1])<=0:
        raise ValueError('Finite image corners and calibrated pinhole camera required')
    edge=float(np.min(np.linalg.norm(np.roll(pixels,-1,axis=0)-pixels,axis=1)))
    if edge<12.:raise ValueError('Marker too small: need at least 12 pixels per edge')
    half=float(size)/2
    model=np.array([[-half,half,0.],[half,half,0.],[half,-half,0.],[-half,-half,0.]])
    solved=cv2.solvePnPGeneric(model,pixels,k,d,flags=cv2.SOLVEPNP_IPPE_SQUARE)
    candidates=[]
    for r,t in zip(solved[1],solved[2]):
        r=np.asarray(r).ravel();t=np.asarray(t).ravel()
        rotation=Rotation.from_rotvec(r)
        if not np.all(rotation.apply(model)[:,2]+t[2]>.01) or not .02<=np.linalg.norm(t)<=5.:continue
        projected,jac=cv2.projectPoints(model,r,t,k,d)
        error=float(np.sqrt(np.mean(np.sum((projected.reshape(4,2)-pixels)**2,axis=1))))
        j=jac[:,:6];singular=np.linalg.svd(j,compute_uv=False)
        if singular[-1]<1e-6 or singular[0]/singular[-1]>1e7:continue
        # Declared 0.5 px corner noise floor; not an empirical calibration certificate.
        covariance=np.linalg.inv(j.T@j)*max(.5,error)**2
        translation_std=float(np.sqrt(np.linalg.eigvalsh(covariance[3:,3:]).max()))
        rotation_std=float(np.sqrt(np.linalg.eigvalsh(covariance[:3,:3]).max()))
        if error<=1.5 and translation_std<=.015 and rotation_std<=.25:
            candidates.append(dict(position=t.tolist(),orientation_wxyz=rotation.as_quat()[[3,0,1,2]].tolist(),
                reprojection_rms_px=error,translation_std_m=translation_std,rotation_std_rad=rotation_std,
                minimum_edge_px=edge))
    candidates.sort(key=lambda item:item['reprojection_rms_px'])
    if not candidates:raise ValueError('Marker PnP failed reprojection/conditioning/uncertainty bounds')
    if len(candidates)>1:
        a,b=candidates[:2]
        ra=Rotation.from_quat(np.array(a['orientation_wxyz'])[[1,2,3,0]])
        rb=Rotation.from_quat(np.array(b['orientation_wxyz'])[[1,2,3,0]])
        if (ra.inv()*rb).magnitude()>.35 and b['reprojection_rms_px']<=a['reprojection_rms_px']+.15:
            raise ValueError('Planar marker pose is ambiguous; change the camera angle')
    return candidates[0]


def object_pose(estimate,binding,world_from_camera):
    """T_world_object = T_world_camera T_camera_marker inverse(T_object_marker)."""
    def transform(p,q):
        result=np.eye(4);result[:3,:3]=Rotation.from_quat(np.asarray(q)[[1,2,3,0]]).as_matrix();result[:3,3]=p
        return result
    cm=transform(estimate['position'],estimate['orientation_wxyz'])
    om=binding['object_from_marker']
    result=np.asarray(world_from_camera)@cm@np.linalg.inv(transform(om[:3],om[3:]))
    offset=float(np.linalg.norm(om[:3]))
    error=estimate['translation_std_m']+offset*estimate['rotation_std_rad']
    if not np.all(np.isfinite(result)) or error>.02:raise ValueError('Object pose/offset uncertainty exceeds 2 cm')
    return dict(object=binding['object'],type='box',size=binding['object_size_m'],
        position=result[:3,3].tolist(),orientation_wxyz=Rotation.from_matrix(result[:3,:3]).as_quat()[[3,0,1,2]].tolist(),
        position_std_bound_m=error,**{k:estimate[k] for k in ('reprojection_rms_px','translation_std_m','rotation_std_rad','minimum_edge_px')})
