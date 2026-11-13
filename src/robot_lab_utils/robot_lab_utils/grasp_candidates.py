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


def support_placement(obj, attachment, support, *, clearance=.003, approach=.08, edge_margin=.01):
    """Keep the measured object orientation and place its bottom on a box top.

    The acknowledged object-in-tool transform is inverted to compute the real
    TCP target. Geometry/fit alone does not establish reachability or release.
    """
    top_grasps(obj)  # Shared measured-pose validation; need not fit an open jaw.
    if not all(math.isfinite(v) and v>0 for v in (clearance,approach,edge_margin)):
        raise ValueError('Positive finite placement clearances required')
    if not isinstance(attachment,dict) or attachment.get('object_id')!=obj.get('id'):
        raise ValueError('Matching acknowledged object-in-tool transform required')
    for key in ('plant_id','source_sha256'):
        if not obj.get(key) or obj[key]!=attachment.get(key):
            raise ValueError('Placement requires the current source/plant attachment')
    if not isinstance(support,dict) or support.get('frame')!='native_world' or support.get('type')!='box':
        raise ValueError('Select a measured horizontal box support')
    position=np.asarray(support['position'],dtype=float)
    size=np.asarray(support['size'],dtype=float)
    local_position=np.asarray(attachment['position'],dtype=float)
    def matrix(values):
        q=np.asarray(values,dtype=float)
        if q.shape!=(4,) or not np.all(np.isfinite(q)) or not .9<=np.linalg.norm(q)<=1.1:
            raise ValueError('Finite unit placement orientation required')
        return Rotation.from_quat((q/np.linalg.norm(q))[[1,2,3,0]]).as_matrix()
    if (position.shape!=(3,) or size.shape!=(3,) or local_position.shape!=(3,)
            or not all(np.all(np.isfinite(v)) for v in (position,size,local_position)) or min(size)<=0):
        raise ValueError('Finite physical support/attachment dimensions required')
    support_rotation=matrix(support['orientation_wxyz'])
    object_rotation=matrix(obj['orientation_wxyz'])
    local_rotation=matrix(attachment['orientation_wxyz'])
    if abs(support_rotation[2,2])<1.-1e-6:
        raise ValueError('Sloped placement surfaces need a separate support adapter')
    if obj.get('type','box')!='box':
        raise ValueError('Only reviewed box support/placement geometry is implemented')
    object_size=np.asarray(obj['size'],dtype=float)
    projected=np.abs(support_rotation.T@object_rotation)@object_size
    if np.any(projected[:2]+2*edge_margin>size[:2]):
        raise ValueError('Object footprint does not fit inside the support edge margin')
    half_height=float((np.abs(object_rotation)@object_size)[2]/2)
    center=position.copy();center[2]+=size[2]/2+half_height+clearance
    # T_world_tool = T_world_object * inverse(T_tool_object).
    tool_rotation=object_rotation@local_rotation.T
    tcp=center-tool_rotation@local_position
    orientation=Rotation.from_matrix(tool_rotation).as_quat()[[3,0,1,2]].tolist()
    return dict(object_id=obj['id'],support_id=support.get('id','support'),object_center=center.tolist(),
        poses={name:dict(position=(tcp+[0.,0.,dz]).tolist(),orientation_wxyz=orientation)
               for name,dz in (('above',approach),('lower',0.))},
        scope='Measured support/attachment geometry; collision-aware IK/plan and physical release still required')
