"""Validate fresh perception targets against the actual task/source identity."""
import copy
import math
import time
import numpy as np


class ObjectEstimates:
    def __init__(self,pose_source='aruco_pnp'):
        if pose_source not in ('aruco_pnp','rgbd_box_top'):raise ValueError('Unknown camera pose source')
        self.pose_source=pose_source;self.targets={};self.received=-math.inf;self.identity=None
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
            if self.pose_source=='rgbd_box_top' and item.get('pose_reference')!='collision_box_center':
                raise ValueError('RGB-D box targets must name their collision-box centre reference')
            if self.pose_source=='rgbd_box_top':
                size=np.asarray(item.get('size'),dtype=float)
                if item.get('type')!='box' or size.shape!=(3,) or not np.all(np.isfinite(size)) or np.any(size<=0):
                    raise ValueError('RGB-D targets require finite physical box dimensions')
        self.targets=copy.deepcopy(targets);self.received=time.monotonic();self.stamp=stamp;self.identity=dict(identity)
        rejected=record.get('rejected',{})
        self.reason=str(record.get('reason') or ('; '.join(str(k)+': '+str(v) for k,v in rejected.items())
            if isinstance(rejected,dict) else '') or ('' if targets else 'No matching visible object'))[:1024]

    def get(self,name,identity,now):
        if self.identity!=identity or time.monotonic()-self.received>1. or not -.05<=now-self.stamp<=.5:
            raise ValueError('Fresh source/scene-bound camera observation required')
        if name not in self.targets:raise ValueError('Selected object marker is not visible')
        return copy.deepcopy(self.targets[name])
