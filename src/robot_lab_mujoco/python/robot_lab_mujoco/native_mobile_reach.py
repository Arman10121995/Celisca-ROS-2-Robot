"""Bounded local source-control IK and collision checks for Stretch tasks.

Runs on an immutable snapshot/private MjData in a worker. It only returns motor
waypoints; the task executor owns application, stop, heartbeat and live checks.
"""
import time
import numpy as np
from robot_lab_utils.native_control_kinematics import ControlKinematics


def plan_reach(node, snapshot, channels, initial, step, robot_bodies, robot_joints,
               excluded, payload=None, budget=8.):
    mj, model = node.mujoco, node.model
    deadline = time.monotonic()+budget
    controls = np.asarray(initial).copy()
    ik = ControlKinematics(model, snapshot, channels, robot_joints, excluded)
    bodies = [mj.mj_name2id(model,mj.mjtObj.mjOBJ_BODY,step[key]) for key in ('left_body','right_body')]
    obj = mj.mj_name2id(model,mj.mjtObj.mjOBJ_BODY,step['object'])
    if min(bodies+[obj]) < 0:
        raise ValueError('Reach object or physical TCP pad is missing')
    # Only source lift/telescope/wrist controls participate; wheels/head/grip stay held.
    movable = [i for i,c in enumerate(channels) if c['name'] in
               ('lift','arm_extend','arm','wrist_yaw','wrist_pitch','wrist_roll')]
    if not movable:
        raise ValueError('No reviewed source Stretch reach channels')
    object_geoms = [g for g in range(model.ngeom) if model.geom_bodyid[g]==obj
                    and (model.geom_contype[g] or model.geom_conaffinity[g])]
    if not object_geoms:
        raise ValueError('Object has no physical collision geometry')
    target = np.asarray(step.get('target_xyz',np.mean(snapshot.geom_xpos[object_geoms],axis=0)
                                 +np.asarray(step.get('offset',[0.,0.,0.]))),dtype=float)
    ik.forward(controls)
    start_tcp = ik.midpoint(bodies).copy()
    if np.linalg.norm(target-start_tcp) > .8:
        raise ValueError('Reach exceeds local 0.8 m planning bound; reposition the base')
    if step['kind']=='stow_arm':
        extension = [i for i,c in enumerate(channels) if c['name'] in ('arm','arm_extend')]
        if len(extension)!=1:
            raise ValueError('No unique reviewed source telescope control')
        target_controls = controls.copy()
        target_controls[extension[0]] = channels[extension[0]]['limits'][0]
        ik.forward(target_controls)
        target = ik.midpoint(bodies).copy()
    else:
        target_controls = ik.reach(controls,bodies,target,movable,deadline)
    # Sample the actual motor-coordinate edge, not merely its endpoints.
    rates = np.asarray([c['max_rate'] for c in channels])
    count = max(1,int(np.ceil(np.max(np.abs(target_controls-controls)/(rates*.25)))))
    if count > 64:
        raise ValueError('Source reach edge exceeds inspection budget')
    baseline = {tuple(sorted(map(int,c.geom))):float(c.dist) for c in snapshot.contact[:snapshot.ncon]}
    allowed = set(bodies)
    for body in range(1,robot_bodies):
        if int(model.body_parentid[body]) in allowed:
            allowed.add(body)
    payload_address = int(model.body_jntadr[payload]) if payload is not None else -1
    if payload_address >= 0:
        payload_address = int(model.jnt_qposadr[payload_address])
    if payload_address >= 0:
        ref = bodies[0]
        relative = snapshot.xmat[ref].reshape(3,3).T @ (snapshot.xpos[payload]-snapshot.xpos[ref])
        relative_quat = np.zeros(4)
        mj.mju_mulQuat(relative_quat,snapshot.xquat[ref]*[1.,-1.,-1.,-1.],snapshot.xquat[payload])
    points = []
    for fraction in np.linspace(0.,1.,count+1):
        if time.monotonic()>deadline:
            raise ValueError('Source reach collision inspection deadline exceeded')
        candidate = controls+fraction*(target_controls-controls)
        probe = ik.forward(candidate)
        if payload_address >= 0:
            # Planning-only attachment uses the measured rigid relation to one
            # pad. Live object motion is still entirely contact physics.
            probe.qpos[payload_address:payload_address+3] = probe.xpos[ref]+probe.xmat[ref].reshape(3,3) @ relative
            mj.mju_mulQuat(probe.qpos[payload_address+3:payload_address+7],probe.xquat[ref],relative_quat)
            mj.mj_forward(model,probe)
        for contact in probe.contact[:probe.ncon]:
            pair = tuple(sorted(map(int,contact.geom)))
            owners = [int(model.geom_bodyid[g]) for g in contact.geom]
            if obj in owners and any(body in allowed for body in owners):
                # A reach with open fingers must not pass through the object.
                if payload==obj:
                    continue
            if (contact.dist < -.001 and contact.dist < baseline.get(pair,0.)-.0002
                    and (any(2<=body<robot_bodies for body in owners) or payload in owners)):
                raise ValueError('Native collision blocks the local reach; use another approach')
        if fraction > 0:
            points.append(dict(targets={c['name']:float(v) for c,v in zip(channels,candidate)},settle_s=.2))
    return dict(waypoints=points,target_xyz=target.tolist(),
                object_position=snapshot.xpos[obj].tolist(),object_quaternion=snapshot.xquat[obj].tolist(),
                initial_controls=controls.tolist(),
                scope='Local position-only source-control IK and sampled collision edge; no global planner')
