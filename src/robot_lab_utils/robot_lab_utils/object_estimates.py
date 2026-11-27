"""Validate fresh perception targets against the actual task/source identity."""
import copy
import math
import time
import numpy as np


RGBD_BOX_SOURCE = 'rgbd_box_top'
RGBD_PRIMITIVE_SOURCE = 'rgbd_primitive_top'
CAMERA_POSE_SOURCES = ('aruco_pnp', RGBD_BOX_SOURCE, RGBD_PRIMITIVE_SOURCE)


class ObjectEstimates:
    def __init__(self,pose_source='aruco_pnp'):
        if pose_source not in CAMERA_POSE_SOURCES:raise ValueError('Unknown camera pose source')
        self.pose_source=pose_source;self.targets={};self.received=-math.inf;self.identity=None
        self.stamp = -math.inf
        self.reason='No observation received; check map bindings and camera'

    def receive(self,record,identity,now):
        if not isinstance(record,dict) or record.get('schema')!='robot_lab.object_estimates.v1':
            raise ValueError('Invalid object-estimate record')
        if any(record.get(k)!=identity[k] for k in ('plant_id','source_sha256','scene_id')):
            raise ValueError('Object estimate belongs to a different plant/source/scene')
        targets=record.get('targets')
        if record.get('frame')!='native_world' or record.get('pose_source')!=self.pose_source or not isinstance(targets,dict) or len(targets)>64:
            raise ValueError('Expected bounded native-world camera estimates')
        stamp=float(record['stamp_s'])
        if not math.isfinite(stamp) or not -.05<=now-stamp<=.5:raise ValueError('Object image is stale or future-dated')
        for name,item in targets.items():
            p=np.asarray(item['position'],dtype=float);q=np.asarray(item['orientation_wxyz'],dtype=float)
            if (name!=item['object'] or p.shape!=(3,) or q.shape!=(4,) or not np.all(np.isfinite(p))
                    or not np.all(np.isfinite(q)) or not .99<=np.linalg.norm(q)<=1.01
                    or not isinstance(item.get('observation_id'),str) or not item['observation_id']):
                raise ValueError('Invalid named perception pose')
            error=float(item['position_std_bound_m'])
            if not math.isfinite(error) or not 0<=error<=.02:raise ValueError('Uncertain perception target')
            if self.pose_source != 'aruco_pnp':
                shape = item.get('type')
                allowed = ('box',) if self.pose_source == RGBD_BOX_SOURCE else ('box', 'cylinder')
                if shape not in allowed or item.get('pose_reference') != f'collision_{shape}_center':
                    raise ValueError('RGB-D pose reference must match its physical shape')
                size = np.asarray(item.get('size'), dtype=float)
                if size.shape != (3,) or not np.all(np.isfinite(size)) or np.any(size <= 0):
                    raise ValueError('RGB-D targets require finite physical dimensions')
                if shape == 'cylinder' and not np.isclose(size[0], size[1], rtol=0., atol=1e-6):
                    raise ValueError('Use [diameter, diameter, height] for a cylinder estimate')
        self.targets=copy.deepcopy(targets);self.received=time.monotonic();self.stamp=stamp;self.identity=dict(identity)
        rejected=record.get('rejected',{})
        self.reason=str(record.get('reason') or ('; '.join(str(k)+': '+str(v) for k,v in rejected.items())
            if isinstance(rejected,dict) else '') or ('' if targets else 'No matching visible object'))[:1024]

    def fresh(self, identity, now):
        return (self.identity == identity and time.monotonic() - self.received <= 1.
                and -.05 <= now - self.stamp <= .5)

    def get(self,name,identity,now):
        if not self.fresh(identity, now):
            raise ValueError('Fresh source/scene-bound camera observation required')
        if name not in self.targets:raise ValueError('Selected object is not visible in the camera')
        return copy.deepcopy(self.targets[name])
