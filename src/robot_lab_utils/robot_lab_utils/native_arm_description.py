"""A MoveIt planning description of the pinned native Panda plant.

The passive display URDF intentionally has fixed measured-body frames. It
cannot supply arm kinematics or collision checking. This separate description
exports the compiled hinge/slide chain and the plant's collision hulls. The
native model, actuators and measured-body visualization are unchanged.
"""
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from .native_mjcf_assets import body_frame, numbers, quaternion_rpy


def panda_planning_model(native_mjcf, output_directory, spawn=(0., 0., 0., 0.)):
    import mujoco as mj
    import trimesh

    output = Path(output_directory)
    output.mkdir(parents=True, exist_ok=True)
    spec = mj.MjSpec.from_file(str(native_mjcf))
    x, y, z, yaw = (float(v) for v in spawn)
    quaternion = np.array([np.cos(yaw/2), 0., 0., np.sin(yaw/2)])
    rotation = np.array([[np.cos(yaw), -np.sin(yaw), 0.],
                         [np.sin(yaw), np.cos(yaw), 0.], [0., 0., 1.]])
    for body in spec.worldbody.bodies:
        body.pos = rotation @ body.pos + [x, y, z]
        result = np.zeros(4)
        mj.mju_mulQuat(result, quaternion, body.quat)
        body.quat = result
    model = spec.compile()
    names = [mj.mj_id2name(model, mj.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)]
    expected = ['joint'+str(i) for i in range(1, 8)]+['finger_joint1', 'finger_joint2']
    if names != expected or model.nbody != 12:
        raise ValueError('MoveIt adapter requires the pinned native Panda joint/body contract')
    if np.any(model.body_jntnum > 1) or not np.allclose(model.jnt_pos, 0., atol=1e-12) \
            or not np.allclose(model.qpos0, 0., atol=1e-12):
        raise ValueError('Native joint anchor/reference needs a separate kinematic export')
    robot = ET.Element('robot', name='native_panda')
    links = [ET.SubElement(robot, 'link', name=body_frame(i)) for i in range(model.nbody)]
    for i in range(1, model.nbody):
        count = int(model.body_jntnum[i])
        attributes = dict(name='native_frame_'+str(i), type='fixed')
        if count:
            j = int(model.body_jntadr[i])
            kind = int(model.jnt_type[j])
            attributes = dict(name=names[j], type='prismatic' if kind == int(mj.mjtJoint.mjJNT_SLIDE) else 'revolute')
        joint = ET.SubElement(robot, 'joint', **attributes)
        ET.SubElement(joint, 'parent', link=body_frame(int(model.body_parentid[i])))
        ET.SubElement(joint, 'child', link=body_frame(i))
        ET.SubElement(joint, 'origin', xyz=numbers(model.body_pos[i]),
                      rpy=numbers(quaternion_rpy(model.body_quat[i])))
        if count:
            ET.SubElement(joint, 'axis', xyz=numbers(model.jnt_axis[j]))
            direct = [a for a in range(model.nu) if int(model.actuator_trntype[a]) == int(mj.mjtTrn.mjTRN_JOINT)
                      and int(model.actuator_trnid[a, 0]) == j]
            effort = float(np.max(np.abs(model.actuator_forcerange[direct]))) if direct else 20.
            # The physical action retains a 0.005 rad soft margin. Give KDL
            # the same arm bounds so a valid plan can also be executed.
            margin = .005 if names[j] in expected[:7] else 0.
            ET.SubElement(joint, 'limit', lower=str(model.jnt_range[j, 0]+margin),
                          upper=str(model.jnt_range[j, 1]-margin), velocity='.5', effort=str(effort))

    collision_count = 0
    for g in range(model.ngeom):
        if not (int(model.geom_contype[g]) or int(model.geom_conaffinity[g])):
            continue
        kind, size = int(model.geom_type[g]), model.geom_size[g]
        geometry = None
        if kind == int(mj.mjtGeom.mjGEOM_MESH):
            index = int(model.geom_dataid[g])
            begin, count = int(model.mesh_vertadr[index]), int(model.mesh_vertnum[index])
            # MuJoCo collides these meshes as convex hulls. Export that same
            # hull rather than a decorative visual or a different concavity.
            mesh = trimesh.convex.convex_hull(model.mesh_vert[begin:begin+count].copy())
        elif kind == int(mj.mjtGeom.mjGEOM_BOX):
            geometry = ('box', dict(size=numbers(2*size)))
        elif kind == int(mj.mjtGeom.mjGEOM_SPHERE):
            geometry = ('sphere', dict(radius=str(size[0])))
        elif kind == int(mj.mjtGeom.mjGEOM_CYLINDER):
            geometry = ('cylinder', dict(radius=str(size[0]), length=str(2*size[1])))
        else:
            raise ValueError('Unconverted native Panda collision geometry: '+str(kind))
        if geometry is None:
            path = output/('collision_%d.stl' % g)
            mesh.export(path)
            geometry = ('mesh', dict(filename=path.resolve().as_uri()))
        collision = ET.SubElement(links[int(model.geom_bodyid[g])], 'collision', name='native_collision_'+str(g))
        ET.SubElement(collision, 'origin', xyz=numbers(model.geom_pos[g]),
                      rpy=numbers(quaternion_rpy(model.geom_quat[g])))
        ET.SubElement(ET.SubElement(collision, 'geometry'), *geometry)
        collision_count += 1

    data = mj.MjData(model)
    mj.mj_resetDataKeyframe(model, data, 0)
    mj.mj_forward(model, data)
    fingers = {mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, n) for n in ('left_finger', 'right_finger')}
    pads = [g for g in range(model.ngeom) if int(model.geom_bodyid[g]) in fingers
            and int(model.geom_type[g]) == int(mj.mjtGeom.mjGEOM_BOX)
            and np.allclose(model.geom_size[g], [.0085, .004, .0085])]
    if len(pads) != 2:
        raise ValueError('Native Panda fingertip pads differ')
    hand = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, 'hand')
    tcp = data.xmat[hand].reshape(3, 3).T @ (np.mean(data.geom_xpos[pads], axis=0)-data.xpos[hand])
    ET.SubElement(robot, 'link', name='panda_tcp')
    joint = ET.SubElement(robot, 'joint', name='panda_tcp_joint', type='fixed')
    ET.SubElement(joint, 'parent', link=body_frame(hand))
    ET.SubElement(joint, 'child', link='panda_tcp')
    ET.SubElement(joint, 'origin', xyz=numbers(tcp))

    semantic = ET.Element('robot', name='native_panda')
    arm = ET.SubElement(semantic, 'group', name='panda_arm')
    ET.SubElement(arm, 'chain', base_link='native_world', tip_link='panda_tcp')
    for name in ('finger_joint1', 'finger_joint2'):
        ET.SubElement(semantic, 'passive_joint', name=name)
    disabled = set()
    for signature in model.exclude_signature:
        disabled.add(tuple(sorted((int(signature) >> 16, int(signature) & 0xffff))))
    # Match native filterparent/weld filtering, including the fixed hand.
    for a in range(model.nbody):
        for b in range(a+1, model.nbody):
            wa, wb = int(model.body_weldid[a]), int(model.body_weldid[b])
            pa = int(model.body_weldid[int(model.body_parentid[wa])])
            pb = int(model.body_weldid[int(model.body_parentid[wb])])
            if wa == wb or wa == pb or wb == pa:
                disabled.add((a, b))
    for a, b in sorted(disabled):
        ET.SubElement(semantic, 'disable_collisions', link1=body_frame(a), link2=body_frame(b),
                      reason='Native collision filter')
    # TCP is a frame without geometry and does not add collision exclusions.
    urdf, srdf = output/'panda.urdf', output/'panda.srdf'
    ET.ElementTree(robot).write(urdf, encoding='unicode')
    ET.ElementTree(semantic).write(srdf, encoding='unicode')
    return dict(urdf=str(urdf), srdf=str(srdf), group='panda_arm', tip='panda_tcp',
                collision_geometry_count=collision_count, joint_names=expected[:7],
                disabled_pairs=sorted(disabled))
