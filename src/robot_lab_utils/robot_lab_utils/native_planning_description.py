"""Source-derived planning descriptions for fixed, open-chain native arms.

This is a separate, opt-in adapter. It does not change the Panda adapter,
native actuator parameters or the measured-body display description. Tool
sites and body origins below are authored frames, not guessed fingertip TCPs.
"""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from .native_actuation import position_channels
from .native_mjcf_assets import body_frame, numbers


# Reviewed source frame choices. The actual chain and actuator contract are
# inspected again at launch; an ID alone never qualifies a model or mission.
ARM_FRAMES = {
    'menagerie_flexiv_rizon4': 'body:link7',
    'menagerie_flexiv_rizon4s': 'body:link7',
    'menagerie_franka_fr3': 'site:attachment_site',
    'menagerie_franka_fr3_v2': 'body:fr3v2_link8',
    'menagerie_kuka_iiwa_14': 'site:attachment_site',
    'menagerie_rethink_robotics_sawyer': 'site:attachment_site',
    'menagerie_robotis_omy_3m': 'site:attachment_site',
    'menagerie_unitree_z1': 'body:link06',
    'menagerie_universal_robots_ur5e': 'site:attachment_site',
    'menagerie_universal_robots_ur10e': 'site:attachment_site',
    'menagerie_low_cost_robot_arm': 'body:gripper_static_finger',
    'menagerie_robotstudio_so101': 'site:gripperframe',
    'menagerie_trs_so_arm100': 'body:Fixed_Jaw',
}


def arm_contract(model, frame, robot_joints=None):
    """Require an independently actuated serial chain with source bounds."""
    import mujoco as mj
    standalone = robot_joints is None
    robot_joints = model.njnt if standalone else int(robot_joints)
    if model.neq or model.npair or (standalone and model.nflex) or np.any(~np.isin(model.jnt_type[:robot_joints],
            [int(mj.mjtJoint.mjJNT_HINGE), int(mj.mjtJoint.mjJNT_SLIDE)])):
        raise ValueError('Planning needs a fixed hinge/slide model without unexported equality/flex/contact-pair constraints')
    kind, name = frame.split(':', 1)
    if kind == 'site':
        site = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, name)
        if site < 0:
            raise ValueError('Authored tool site is missing: '+name)
        tip_body = int(model.site_bodyid[site])
        tip_link = 'native_site_'+str(site)
    elif kind == 'body':
        tip_body = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, name)
        if tip_body <= 0:
            raise ValueError('Authored tool body is missing: '+name)
        tip_link = body_frame(tip_body)
    else:
        raise ValueError('Choose an authored site or body origin')
    branches, body = [], tip_body
    while body:
        branches.append(list(range(int(model.body_jntadr[body]),
            int(model.body_jntadr[body])+int(model.body_jntnum[body]))))
        body = int(model.body_parentid[body])
    joints = [joint for branch in reversed(branches) for joint in branch]
    if not 3 <= len(joints) <= 12:
        raise ValueError('Cartesian planning needs a reviewed 3–12 joint serial arm')
    channels = position_channels(model, robot_joints)
    direct = {channel['joint']: channel for channel in channels if channel['units'] in ('rad', 'm')}
    names = [mj.mj_id2name(model, mj.mjtObj.mjOBJ_JOINT, joint) for joint in joints]
    if not all(name and name in direct for name in names) or len(set(names)) != len(names):
        raise ValueError('Every arm joint needs a unique bounded unit-gear position actuator')
    selected = [direct[name] for name in names]
    # Ambiguous/multiple transmissions must not receive independent targets.
    for joint, channel in zip(joints, selected):
        drivers = [a for a in range(model.nu) if int(model.actuator_trntype[a]) == int(mj.mjtTrn.mjTRN_JOINT)
                   and int(model.actuator_trnid[a, 0]) == joint]
        if drivers != [channel['index']]:
            raise ValueError('Arm joint has an ambiguous actuator transmission')
    margins = [min(.005 if channel['units'] == 'rad' else .001,
        .02*(channel['limits'][1]-channel['limits'][0])) for channel in selected]
    return dict(group='native_arm', tip=tip_link, frame='native_world',
        source_frame=frame, tip_body=tip_body, joint_names=names, joint_indices=joints,
        actuator_indices=[channel['index'] for channel in selected],
        limits=[channel['limits'] for channel in selected], margins=margins,
        max_velocity=[min(.3 if channel['units'] == 'rad' else .03, channel['max_rate'])
                      for channel in selected],
        units=[channel['units'] for channel in selected], position_only_ik=len(joints) < 6,
        all_joint_names=[mj.mj_id2name(model, mj.mjtObj.mjOBJ_JOINT, j) or 'joint_'+str(j)
                         for j in range(robot_joints)])


def compiled_arm_sha256(model, robot_bodies, robot_joints, robot_actuators):
    """Bind included kinematics, collision meshes and actuator contracts too.

    Selecting native body geoms rather than a global geom prefix allows the
    real plant to add its independent world/floor geometry. Spawn is included.
    """
    import mujoco as mj
    def values(array):
        return np.round(np.asarray(array, dtype=float), 10).tolist()
    payload = {name: values(getattr(model, name)[:robot_bodies]) for name in (
        'body_parentid', 'body_pos', 'body_quat', 'body_mass', 'body_inertia', 'body_gravcomp')}
    payload.update({name: values(getattr(model, name)[:robot_joints]) for name in (
        'jnt_bodyid', 'jnt_type', 'jnt_pos', 'jnt_axis', 'jnt_range', 'jnt_limited')})
    payload['qpos0'] = values(model.qpos0[:robot_joints])
    payload.update({name: values(getattr(model, name)[:robot_actuators]) for name in (
        'actuator_trntype', 'actuator_trnid', 'actuator_gear', 'actuator_dyntype',
        'actuator_gaintype', 'actuator_biastype', 'actuator_gainprm', 'actuator_biasprm',
        'actuator_ctrlrange', 'actuator_ctrllimited', 'actuator_forcerange', 'actuator_forcelimited')})
    geoms = np.flatnonzero((model.geom_bodyid > 0) & (model.geom_bodyid < robot_bodies))
    payload['geoms'] = {name: values(getattr(model, name)[geoms]) for name in (
        'geom_bodyid', 'geom_type', 'geom_size', 'geom_pos', 'geom_quat', 'geom_contype',
        'geom_conaffinity', 'geom_friction', 'geom_margin', 'geom_gap')}
    payload['meshes'] = []
    for geom in geoms:
        if int(model.geom_type[geom]) != int(mj.mjtGeom.mjGEOM_MESH):
            continue
        mesh = int(model.geom_dataid[geom])
        begin, count = int(model.mesh_vertadr[mesh]), int(model.mesh_vertnum[mesh])
        payload['meshes'].append(values(model.mesh_vert[begin:begin+count]))
    payload['exclude'] = [int(value) for value in model.exclude_signature
                          if int(value) >> 16 < robot_bodies and int(value) & 0xffff < robot_bodies]
    payload['filter_parent'] = not bool(int(model.opt.disableflags) & int(mj.mjtDisableBit.mjDSBL_FILTERPARENT))
    return hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(',', ':')).encode()).hexdigest()


def _origin(joint, position, quaternion=(1., 0., 0., 0.)):
    # atan2 remains well-conditioned at the exact +/-90 degree source frames.
    # asin pitch extraction produces a spurious ~2e-8 rotation there.
    import mujoco as mj
    quat = np.asarray(quaternion, dtype=float)
    quat = quat/np.linalg.norm(quat)
    matrix = np.zeros(9)
    mj.mju_quat2Mat(matrix, quat)
    matrix = matrix.reshape(3, 3)
    horizontal = np.hypot(matrix[0, 0], matrix[1, 0])
    pitch = np.arctan2(-matrix[2, 0], horizontal)
    if horizontal > 1e-10:
        roll, yaw = np.arctan2(matrix[2, 1], matrix[2, 2]), np.arctan2(matrix[1, 0], matrix[0, 0])
    else:
        roll, yaw = np.arctan2(-matrix[1, 2], matrix[1, 1]), 0.
    ET.SubElement(joint, 'origin', xyz=numbers(position), rpy=numbers([roll, pitch, yaw]))


def _fixed(robot, parent, child, position, quaternion=(1., 0., 0., 0.)):
    joint = ET.SubElement(robot, 'joint', name=child+'_fixed', type='fixed')
    ET.SubElement(joint, 'parent', link=parent)
    ET.SubElement(joint, 'child', link=child)
    _origin(joint, position, quaternion)


def _collisions(model, robot, output):
    """Export source collision hulls and exact primitive unions (capsules)."""
    import mujoco as mj
    import trimesh
    links = {link.get('name'): link for link in robot.findall('link')}
    count = 0
    for g in range(model.ngeom):
        if not (int(model.geom_contype[g]) or int(model.geom_conaffinity[g])):
            continue
        kind, size = int(model.geom_type[g]), model.geom_size[g]
        position, quaternion = model.geom_pos[g].copy(), model.geom_quat[g]
        items = []
        if kind == int(mj.mjtGeom.mjGEOM_MESH):
            mesh_id = int(model.geom_dataid[g])
            begin, n = int(model.mesh_vertadr[mesh_id]), int(model.mesh_vertnum[mesh_id])
            # The native engine uses convex collision hulls for these geoms.
            hull = trimesh.convex.convex_hull(model.mesh_vert[begin:begin+n].copy())
            path = output/('collision_'+str(g)+'.stl')
            hull.export(path)
            items.append(('mesh', dict(filename=path.resolve().as_uri()), position, quaternion))
        elif kind == int(mj.mjtGeom.mjGEOM_BOX):
            items.append(('box', dict(size=numbers(2*size)), position, quaternion))
        elif kind == int(mj.mjtGeom.mjGEOM_SPHERE):
            items.append(('sphere', dict(radius=str(size[0])), position, quaternion))
        elif kind in (int(mj.mjtGeom.mjGEOM_CYLINDER), int(mj.mjtGeom.mjGEOM_CAPSULE)):
            if size[1] > 0:
                items.append(('cylinder', dict(radius=str(size[0]), length=str(2*size[1])), position, quaternion))
            if kind == int(mj.mjtGeom.mjGEOM_CAPSULE):
                for sign in (-1., 1.):
                    delta = np.zeros(3)
                    mj.mju_rotVecQuat(delta, np.array([0., 0., sign*size[1]]), quaternion)
                    items.append(('sphere', dict(radius=str(size[0])), position+delta, quaternion))
        else:
            raise ValueError('Unexported source collision geometry: '+str(kind))
        for part, (shape, attributes, pos, quat) in enumerate(items):
            collision = ET.SubElement(links[body_frame(int(model.geom_bodyid[g]))], 'collision',
                name='native_collision_%d_%d' % (g, part))
            _origin(collision, pos, quat)
            ET.SubElement(ET.SubElement(collision, 'geometry'), shape, **attributes)
            count += 1
    return count


def fixed_arm_planning_model(native_mjcf, output_directory, frame, spawn=(0., 0., 0., 0.)):
    """Export full-body kinematics; the actual plant owns motion and state."""
    import mujoco as mj
    path = Path(native_mjcf).resolve()
    spec = mj.MjSpec.from_file(str(path))
    x, y, z, yaw = map(float, spawn)
    if not np.all(np.isfinite([x, y, z, yaw])):
        raise ValueError('Finite native spawn coordinates required')
    rotation = np.array([[np.cos(yaw), -np.sin(yaw), 0.],
                         [np.sin(yaw), np.cos(yaw), 0.], [0., 0., 1.]])
    quaternion = np.array([np.cos(yaw/2), 0., 0., np.sin(yaw/2)])
    for body in spec.worldbody.bodies:
        body.pos = rotation @ body.pos+[x, y, z]
        result = np.zeros(4)
        mj.mju_mulQuat(result, quaternion, body.quat)
        body.quat = result
    model = spec.compile()
    contract = arm_contract(model, frame)
    output = Path(output_directory)
    output.mkdir(parents=True, exist_ok=True)
    robot = ET.Element('robot', name='native_fixed_arm')
    for body in range(model.nbody):
        ET.SubElement(robot, 'link', name=body_frame(body))
    channels = {channel['joint']: channel for channel in position_channels(model) if channel['units'] in ('rad', 'm')}
    # MuJoCo joints rotate/slide about authored body-frame anchors relative
    # to qpos0. Dummy frames preserve nonzero anchors, references and ordered
    # multiple joints; geometry stays on the original measured-body links.
    for body in range(1, model.nbody):
        parent, body_link = body_frame(int(model.body_parentid[body])), body_frame(body)
        count = int(model.body_jntnum[body])
        if not count:
            _fixed(robot, parent, body_link, model.body_pos[body], model.body_quat[body])
            continue
        rest = 'native_rest_'+str(body)
        ET.SubElement(robot, 'link', name=rest)
        _fixed(robot, parent, rest, model.body_pos[body], model.body_quat[body])
        previous, anchor = rest, np.zeros(3)
        for j in range(int(model.body_jntadr[body]), int(model.body_jntadr[body])+count):
            name = contract['all_joint_names'][j]
            slide = int(model.jnt_type[j]) == int(mj.mjtJoint.mjJNT_SLIDE)
            child = 'native_joint_frame_'+str(j)
            ET.SubElement(robot, 'link', name=child)
            joint = ET.SubElement(robot, 'joint', name=name, type='prismatic' if slide else
                'revolute' if model.jnt_limited[j] or name in channels else 'continuous')
            ET.SubElement(joint, 'parent', link=previous)
            ET.SubElement(joint, 'child', link=child)
            ref = float(model.qpos0[int(model.jnt_qposadr[j])])
            pos, axis = model.jnt_pos[j]-anchor, model.jnt_axis[j]
            quat = np.array([1., 0., 0., 0.])
            if slide:
                pos = pos-axis*ref
            else:
                quat = np.r_[np.cos(ref/2), -axis*np.sin(ref/2)]
            _origin(joint, pos, quat)
            ET.SubElement(joint, 'axis', xyz=numbers(axis))
            channel = channels.get(name)
            bound = channel['limits'] if channel else model.jnt_range[j]
            limits = dict(velocity=str(channel['max_rate'] if channel else .5), effort='1.0')
            if channel:
                actuator = channel['index']
                if model.actuator_forcelimited[actuator]:
                    limits['effort'] = str(float(np.max(np.abs(model.actuator_forcerange[actuator]))))
            if joint.get('type') != 'continuous':
                margin = contract['margins'][contract['joint_names'].index(name)] if name in contract['joint_names'] else 0.
                limits.update(lower=str(bound[0]+margin), upper=str(bound[1]-margin))
            ET.SubElement(joint, 'limit', **limits)
            previous, anchor = child, model.jnt_pos[j].copy()
        _fixed(robot, previous, body_link, -anchor)
    for site in range(model.nsite):
        link = 'native_site_'+str(site)
        ET.SubElement(robot, 'link', name=link)
        _fixed(robot, body_frame(int(model.site_bodyid[site])), link, model.site_pos[site], model.site_quat[site])
    contract['collision_geometry_count'] = _collisions(model, robot, output)
    semantic = ET.Element('robot', name='native_fixed_arm')
    group = ET.SubElement(semantic, 'group', name=contract['group'])
    ET.SubElement(group, 'chain', base_link=contract['frame'], tip_link=contract['tip'])
    for name in contract['all_joint_names']:
        if name not in contract['joint_names']:
            ET.SubElement(semantic, 'passive_joint', name=name)
    disabled = {tuple(sorted((int(signature) >> 16, int(signature) & 0xffff)))
                for signature in model.exclude_signature}
    collision_geoms = {body: np.flatnonzero(model.geom_bodyid == body) for body in range(model.nbody)}
    filter_parent = not (int(model.opt.disableflags) & int(mj.mjtDisableBit.mjDSBL_FILTERPARENT))
    for a in range(model.nbody):
        for b in range(a+1, model.nbody):
            wa, wb = int(model.body_weldid[a]), int(model.body_weldid[b])
            pa, pb = int(model.body_weldid[int(model.body_parentid[wa])]), int(model.body_weldid[int(model.body_parentid[wb])])
            mask_contact = any((int(model.geom_contype[i]) & int(model.geom_conaffinity[k])) or
                (int(model.geom_contype[k]) & int(model.geom_conaffinity[i]))
                for i in collision_geoms[a] for k in collision_geoms[b])
            if wa == wb or filter_parent and (wa == pb or wb == pa) or not mask_contact:
                disabled.add((a, b))
    for a, b in sorted(disabled):
        ET.SubElement(semantic, 'disable_collisions', link1=body_frame(a), link2=body_frame(b), reason='Native collision filter')
    urdf, srdf = output/'arm.urdf', output/'arm.srdf'
    ET.ElementTree(robot).write(urdf, encoding='unicode')
    ET.ElementTree(semantic).write(srdf, encoding='unicode')
    # Includes and all imported mesh bytes remain source-pinned in the store.
    # This manifest binds this launch's actual export and direct native source.
    def fixed_body(body):
        while body:
            if model.body_jntnum[body]:
                return False
            body = int(model.body_parentid[body])
        return True
    contract.update(native_mjcf=str(path), source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        compiled_sha256=compiled_arm_sha256(model, model.nbody, model.njnt, model.nu),
        urdf=str(urdf), srdf=str(srdf), robot_bodies=model.nbody, robot_actuators=model.nu,
        link_padding_m=.001,
        fixed_links=[body_frame(body) for body in range(1, model.nbody)
                     if fixed_body(body)],
        body_links=[body_frame(body) for body in range(1, model.nbody)],
        qualification='experimental source-derived fixed-arm planning; physical missions pending')
    config = output/'controller.json'
    config.write_text(json.dumps(contract, indent=2)+'\n')
    contract['config'] = str(config)
    return contract
