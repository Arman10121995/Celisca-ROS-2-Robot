"""Geometry-only parallel-jaw top grasps, to be checked by IK and MoveIt.

Coordinates use native_world; quaternions are wxyz. No candidate certifies
reachability, contact, grasp quality or collision clearance.
"""
import math
import numpy as np
from scipy.spatial.transform import Rotation


def top_grasps(obj,max_opening=.08,clearance=.004,approach=.08,lift=.10):
    if not isinstance(obj,dict):
        raise ValueError('Object geometry must be a named record')
    if not all(math.isfinite(v) and v>0 for v in (max_opening,clearance,approach,lift)):
        raise ValueError('Positive finite gripper/approach geometry required')
    if obj.get('frame')!='native_world':
        raise ValueError('Object pose must be expressed in native_world')
    position=np.asarray(obj['position'],dtype=float)
    quat=np.asarray(obj['orientation_wxyz'],dtype=float)
    size=np.asarray(obj['size'],dtype=float)
    if (position.shape!=(3,) or quat.shape!=(4,) or not np.all(np.isfinite(position))
            or not np.all(np.isfinite(quat)) or not np.all(np.isfinite(size)) or np.any(size<=0)
            or not .9<=np.linalg.norm(quat)<=1.1):
        raise ValueError('Finite physical object pose/dimensions required')
    kind=obj.get('type','box')
    if (kind=='box' and size.shape!=(3,)) or (kind=='sphere' and size.shape!=(1,)) or (kind=='cylinder' and size.shape!=(2,)):
        raise ValueError('Object primitive dimensions do not match its type')
    if kind not in ('box','sphere','cylinder'):
        raise ValueError('Review a source geometry adapter for this grasp object')
    rotation=Rotation.from_quat((quat/np.linalg.norm(quat))[[1,2,3,0]]).as_matrix()
    def projected_width(direction):
        local=rotation.T@direction
        return (float(np.sum(np.abs(local)*size)) if kind=='box' else
                float(2*size[0]) if kind=='sphere' else
                float(2*size[0]*np.linalg.norm(local[:2])+size[1]*abs(local[2])))
    approach_height=max(approach,.5*projected_width(np.array([0.,0.,1.]))+.02)
    candidates=[];directions=[]
    for axis in range(3):
        closing=rotation[:,axis].copy();closing[2]=0.
        if np.linalg.norm(closing)<.2:continue
        closing/=np.linalg.norm(closing)
        for sign in (1.,-1.):
            jaw=sign*closing
            if any(np.linalg.norm(jaw-previous)<1e-4 for previous in directions):continue
            directions.append(jaw)
            width=projected_width(jaw)
            if width+clearance>max_opening:continue
            down=np.array([0.,0.,-1.]);side=np.cross(jaw,down)
            tool=np.column_stack((side,jaw,down))
            q=Rotation.from_matrix(tool).as_quat()[[3,0,1,2]].tolist()
            poses={name:dict(position=(position+[0.,0.,dz]).tolist(),orientation_wxyz=q)
                   for name,dz in (('approach',approach_height),('grasp',0.),('lift',lift))}
            candidates.append(dict(id='top_'+str(len(candidates)+1),object_id=str(obj.get('id','object')),
                opening_m=width+clearance,closing_axis_world=jaw.tolist(),poses=poses,
                scope='Geometry candidate; requires collision-aware IK/plan and measured grasp'))
    return candidates
