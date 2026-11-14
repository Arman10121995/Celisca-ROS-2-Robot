"""Source-control IK and bounded configuration detours for Stretch tasks.

Runs on an immutable snapshot/private MjData in a worker. It only returns motor
waypoints; the task executor owns application, stop, heartbeat and live checks.
"""
import time
import numpy as np
from robot_lab_utils.native_control_kinematics import ControlKinematics
from robot_lab_utils.bounded_control_planner import connect_controls, PlanningBudgetExceeded


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
    if step.get('keep_orientation'):
        movable=[i for i in movable if channels[i]['name'] in ('lift','arm_extend','arm')]
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
    orientation=step.get('orientation_wxyz')
    if step.get('keep_orientation'):
        orientation=snapshot.xquat[bodies[0]].copy()
    if orientation is not None:
        orientation=np.asarray(orientation,dtype=float)
        if (orientation.shape!=(4,) or not np.all(np.isfinite(orientation))
                or not .9<=np.linalg.norm(orientation)<=1.1):
            raise ValueError('Finite unit world pad orientation required')
        orientation=orientation/np.linalg.norm(orientation)
    target_controls=None
    if step['kind']=='stow_arm':
        extension = [i for i,c in enumerate(channels) if c['name'] in ('arm','arm_extend')]
        if len(extension)!=1:
            raise ValueError('No unique reviewed source telescope control')
        target_controls = controls.copy()
        target_controls[extension[0]] = channels[extension[0]]['limits'][0]
        ik.forward(target_controls)
        target = ik.midpoint(bodies).copy()
    rates = np.asarray([c['max_rate'] for c in channels])
    if not np.all(np.isfinite(rates)) or np.any(rates <= 0):
        raise ValueError('Positive finite source actuator rates required')
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
    inspections = 0
    def collision_free(candidate):
        nonlocal inspections
        if time.monotonic()>deadline:
            raise PlanningBudgetExceeded('Source reach collision inspection deadline exceeded')
        if inspections>=12000:
            raise PlanningBudgetExceeded('Source reach collision inspection budget exhausted')
        inspections += 1
        try:
            probe = ik.forward(candidate)
        except ValueError:
            return False  # A sampled source control can violate a coupled joint limit.
        if step.get('keep_orientation') and 2*np.arccos(np.clip(abs(np.dot(
                probe.xquat[bodies[0]],orientation)),0.,1.))>.05:
            return False
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
                return False
        return True

    def motor_points(a, b):
        # These are the exact intermediate targets returned for execution.
        count = max(1,int(np.ceil(np.max(np.abs(b-a)/(rates*.25)))))
        return [a+(b-a)*(index/count) for index in range(1,count+1)]

    span = np.asarray([c['limits'][1]-c['limits'][0] for c in channels])
    sample_dt = min(.025,float(np.min(.005*span/rates)))
    if not np.isfinite(sample_dt) or sample_dt <= 0:
        raise ValueError('Finite nonzero source control spans required')
    edge_checks=0
    def edge_free(a, b):
        nonlocal edge_checks
        edge_checks+=1
        origin = a.copy()
        for end in motor_points(a,b):
            delta = end-origin
            duration = float(np.max(np.abs(delta)/rates))
            count = max(1,int(np.ceil(duration/sample_dt)))
            # Inspect the controller's per-channel rate clipping, including
            # each joint's stopping instant, rather than a straight-line guess.
            times = np.unique(np.r_[np.linspace(0.,duration,count+1),np.abs(delta)/rates])
            for elapsed in times:
                command = origin+np.sign(delta)*np.minimum(np.abs(delta),rates*elapsed)
                if not collision_free(command):
                    return False
            origin = end
        return True

    def expand(values):
        result = controls.copy()
        result[movable] = values
        return result
    limits = np.asarray([c['limits'] for c in channels])
    # Never sample gripper/head/base controls. The measured compressive gripper
    # target remains held in every private and returned configuration.
    # Try the measured seed first. Only a blocked/unavailable solution pays for
    # further source IK seeds. Reserve a substantial part of the same total
    # worker budget for the shared multi-goal configuration search.
    rng=np.random.default_rng(0)
    seeds=[controls.copy()]
    if step['kind']!='stow_arm':
        for values in [np.mean(limits[movable],axis=1),
                       *[rng.uniform(limits[movable,0],limits[movable,1]) for _ in range(4)]]:
            candidate=controls.copy();candidate[movable]=values;seeds.append(candidate)
    goals=[];goal_attempts=[];attempts=[];seen=[];plan=None
    collection_end=deadline-min(3.,budget*.5)
    for index,guess in enumerate(seeds):
        if index and time.monotonic()>=collection_end:break
        attempt=dict(index=index,seed_kind='measured' if index==0 else 'midpoint' if index==1 else 'seeded_random')
        attempts.append(attempt)
        try:
            if step['kind']=='stow_arm':
                goal=target_controls.copy()
            else:
                local_deadline=(min(deadline,time.monotonic()+2.) if index==0 else
                                min(collection_end,time.monotonic()+.5))
                goal=ik.reach(guess,bodies,target,movable,local_deadline,orientation=orientation)
        except ValueError as exc:
            attempt.update(result='ik_refused',reason=str(exc));continue
        if any(np.max(np.abs((goal-previous)/span))<1e-3 for previous in seen):
            attempt['result']='duplicate_solution';continue
        seen.append(goal.copy())
        if not collision_free(goal):
            attempt['result']='goal_collision';continue
        attempt['result']='goal_available'
        goals.append(goal.copy());goal_attempts.append(index)
        if edge_free(controls,goal):
            plan=dict(path=[controls[movable].copy(),goal[movable].copy()],planner='direct',
                      seed=0,nodes=2,collision_edges=edge_checks,goal_index=len(goals)-1,goal_count=len(goals))
            break
    if plan is None:
        if not goals:
            reason=next((entry.get('reason') for entry in reversed(attempts) if entry.get('reason')),
                        'all source IK solutions collide')
            raise ValueError('No collision-free source IK candidate within the bounded search: '+reason)
        plan=connect_controls(controls[movable],np.asarray(goals)[:,movable],limits[movable],
            lambda a,b:edge_free(expand(a),expand(b)),deadline=deadline,seed=0)
    points = []
    for a,b in zip(plan['path'],plan['path'][1:]):
        points.extend(motor_points(expand(a),expand(b)))
    if len(points)>64:
        raise ValueError('Source configuration route exceeds the 64-waypoint execution budget')
    waypoints = [dict(targets={c['name']:float(v) for c,v in zip(channels,candidate)},settle_s=.2)
                 for candidate in points]
    return dict(waypoints=waypoints,target_xyz=target.tolist(),
                target_orientation_wxyz=None if orientation is None else np.asarray(orientation).tolist(),
                orientation_body=step['left_body'] if orientation is not None else None,
                object_position=snapshot.xpos[obj].tolist(),object_quaternion=snapshot.xquat[obj].tolist(),
                initial_controls=controls.tolist(),
                planner=dict(algorithm=plan['planner'],seed=plan['seed'],nodes=plan['nodes'],
                             collision_edges=plan['collision_edges'],collision_samples=inspections,
                             inspected_motor_edges=edge_checks,ik_attempts=attempts,
                             valid_ik_goals=len(goals),selected_ik_attempt=goal_attempts[plan['goal_index']]),
                scope='Bounded source multi-seed point/pad-pose IK and multi-goal configuration search; sampled original motor slew')
