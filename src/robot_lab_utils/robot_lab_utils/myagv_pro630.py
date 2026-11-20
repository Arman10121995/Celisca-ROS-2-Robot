"""Official myCobot Pro630 arm composed onto the myAGV Pro base (ROADMAP R14.3).

Source: ``elephantrobotics/mycobot_ros2`` (BSD-2-Clause), the
``mycobot_pro_630.urdf`` used by the official ``mycobot_630_moveit2``.
Kept from the source: link and joint kinematics, joint axes and position
limits, collision primitives and the visual meshes (converted from Collada,
whose unit is millimetres, to metre-scaled STL so every backend can load
them).

The source has no ``<inertial>`` data and zero velocity limits, and the vendor
publishes no arm weight or joint speeds. Declared lab estimates, never vendor
specifications:

* ``ARM_MASS_KG`` spread over the links by convex-hull volume, each link's
  centre of mass and inertia tensor from its hull at that density;
* ``MAX_JOINT_SPEED`` for every joint;
* effort caps: 1.5 x (the largest gravity torque plus the largest
  mass-matrix diagonal x DESIGN_ACCEL) over sampled poses, with the rated
  2 kg payload at the flange (PyBullet inverse dynamics and mass matrix);
* the mount: the arm base on the deck rails, centred (the vendor states six
  M6 screws but no position).

Link and joint names gain a ``pro630_`` prefix so they cannot collide with
the base.
"""
import hashlib
import math
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

PREFIX = 'pro630_'
ARM_URDF = 'mycobot_description/urdf/mycobot_pro_630/mycobot_pro_630.urdf'
F100_URDF = 'mycobot_pro/mycobotpro630_gazeboros2/config/mycobot_pro_630_force_gripper.urdf'
F100_LEADER = 'gripper_controller'
ARM_MASS_KG = 8.0            # lab estimate: the vendor publishes no arm weight
PAYLOAD_KG = 2.0             # vendor rated payload
MAX_JOINT_SPEED = 1.0        # rad/s, lab cap: the vendor publishes no joint speeds
EFFORT_MARGIN = 1.5
DESIGN_ACCEL = 2.0           # rad/s^2 the effort caps must also deliver (lab choice)
MOUNT_XYZ = (0.0, 0.0, 0.220)   # base_link: deck rails top (deck 0.200 m, rails 0.219-0.220 m)
COLOURS = {'base': '0.15 0.15 0.15 1', 'link6': '0.15 0.15 0.15 1'}   # dark base and flange
DEFAULT_COLOUR = '0.92 0.92 0.92 1'                                    # white arm shells


def _digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _origin_matrix(element):
    from scipy.spatial.transform import Rotation
    matrix = np.eye(4)
    if element is not None:
        matrix[:3, 3] = [float(v) for v in element.get('xyz', '0 0 0').split()]
        matrix[:3, :3] = Rotation.from_euler('xyz', [float(v) for v in element.get('rpy', '0 0 0').split()]).as_matrix()
    return matrix


def _vector(values):
    return ' '.join(format(float(v), '.9g') for v in values)


def convert_meshes(arm, source_dir, out_dir):
    """Collada (mm) -> metre STL beside the model; returns {link: hull in link frame}."""
    import trimesh
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    hulls, receipt = {}, {}
    for link in arm.findall('link'):
        visual = link.find('visual')
        mesh = visual.find('geometry/mesh') if visual is not None else None
        if mesh is None:
            continue
        uri = mesh.get('filename')
        name = Path(uri).name
        source = Path(source_dir) / name
        loaded = trimesh.load(str(source), force='mesh')
        loaded.apply_scale(0.001)          # <unit meter="0.001"/> in every Pro630 file
        target = out_dir / (Path(name).stem + '.stl')
        loaded.export(str(target))
        mesh.set('filename', str(target))
        material = ET.SubElement(visual, 'material', name=PREFIX + link.get('name') + '_colour')
        ET.SubElement(material, 'color', rgba=COLOURS.get(link.get('name'), DEFAULT_COLOUR))
        in_link = loaded.copy()
        in_link.apply_transform(_origin_matrix(visual.find('origin')))
        hulls[link.get('name')] = in_link.convex_hull
        receipt[name] = dict(source_sha256=_digest(source), stl=str(target), stl_sha256=_digest(target),
                             faces=int(len(loaded.faces)))
    return hulls, receipt


def estimate_inertials(arm, hulls):
    """Uniform density over the hull volumes so the links total ARM_MASS_KG."""
    volume = sum(h.volume for h in hulls.values())
    density = ARM_MASS_KG / volume
    estimates = {}
    for link in arm.findall('link'):
        hull = hulls[link.get('name')]
        hull.density = density
        mass, com, tensor = float(hull.mass), hull.center_mass, hull.moment_inertia
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'origin', xyz=_vector(com), rpy='0 0 0')
        ET.SubElement(inertial, 'mass', value=format(mass, '.9g'))
        ET.SubElement(inertial, 'inertia', ixx=format(tensor[0, 0], '.9g'), iyy=format(tensor[1, 1], '.9g'),
                      izz=format(tensor[2, 2], '.9g'), ixy=format(tensor[0, 1], '.9g'),
                      ixz=format(tensor[0, 2], '.9g'), iyz=format(tensor[1, 2], '.9g'))
        estimates[link.get('name')] = dict(mass_kg=round(mass, 4), com_m=[round(float(v), 4) for v in com])
    return estimates, density


def gravity_efforts(arm_xml, samples=400, seed=7):
    """Per joint over sampled poses, with the payload at the flange (fixed
    base, PyBullet): the largest |gravity torque| and the largest mass-matrix
    diagonal (the inertia that joint must accelerate)."""
    import pybullet as p
    document = ET.fromstring(arm_xml)
    flange = document.find("link[@name='link6']/inertial")
    mass = flange.find('mass')
    mass.set('value', format(float(mass.get('value')) + PAYLOAD_KG, '.9g'))
    with tempfile.NamedTemporaryFile('w', suffix='.urdf', delete=False) as handle:
        handle.write(ET.tostring(document, encoding='unicode'))
        path = handle.name
    client = p.connect(p.DIRECT)
    try:
        p.setGravity(0, 0, -9.81, physicsClientId=client)
        body = p.loadURDF(path, useFixedBase=True, physicsClientId=client)
        movable = [i for i in range(p.getNumJoints(body, physicsClientId=client))
                   if p.getJointInfo(body, i, physicsClientId=client)[2] == p.JOINT_REVOLUTE]
        limits = [p.getJointInfo(body, i, physicsClientId=client)[8:10] for i in movable]
        rng = np.random.RandomState(seed)
        worst, inertia = np.zeros(len(movable)), np.zeros(len(movable))
        poses = [np.zeros(len(movable))] + [np.array([rng.uniform(lo, hi) for lo, hi in limits]) for _ in range(samples)]
        for q in poses:
            tau = p.calculateInverseDynamics(body, list(q), [0.] * len(q), [0.] * len(q), physicsClientId=client)
            worst = np.maximum(worst, np.abs(tau))
            inertia = np.maximum(inertia, np.diag(np.array(p.calculateMassMatrix(body, list(q), physicsClientId=client))))
        names = [p.getJointInfo(body, i, physicsClientId=client)[1].decode() for i in movable]
    finally:
        p.disconnect(client)
        Path(path).unlink(missing_ok=True)
    return {n: (float(g), float(i)) for n, g, i in zip(names, worst, inertia)}


def compose(base, arm_root, mesh_dir):
    """Fix the Pro630 to ``base`` (the myAGV Pro URDF element, edited in
    place). Returns (articulation config, receipt)."""
    arm_root = Path(arm_root)
    arm = ET.parse(str(arm_root / ARM_URDF)).getroot()
    hulls, meshes = convert_meshes(arm, (arm_root / ARM_URDF).parent, mesh_dir)
    estimates, density = estimate_inertials(arm, hulls)
    # Velocity cap on every joint (the source has velocity 0); then efforts.
    for joint in arm.findall('joint'):
        joint.find('limit').set('velocity', format(MAX_JOINT_SPEED, 'g'))
    efforts = gravity_efforts(ET.tostring(arm, encoding='unicode'))
    joints = {}
    for joint in arm.findall('joint'):
        name = joint.get('name')
        limit = joint.find('limit')
        gravity, inertia = efforts[name]
        effort = max(5.0, round(EFFORT_MARGIN * (gravity + inertia * DESIGN_ACCEL), 1))
        limit.set('effort', format(effort, 'g'))
        joints[PREFIX + name] = dict(mode='position', lower=float(limit.get('lower')), upper=float(limit.get('upper')),
                                     max_velocity=MAX_JOINT_SPEED, effort=effort,
                                     kp=round(40.0 * effort, 1), kd=round(2.0 * math.sqrt(effort), 2))
    # Prefix names; attach the arm base to the deck.
    for link in arm.findall('link'):
        link.set('name', PREFIX + link.get('name'))
        base.append(link)
    for joint in arm.findall('joint'):
        joint.set('name', PREFIX + joint.get('name'))
        for tag in ('parent', 'child'):
            joint.find(tag).set('link', PREFIX + joint.find(tag).get('link'))
        base.append(joint)
    mount = ET.SubElement(base, 'joint', name=PREFIX + 'mount_joint', type='fixed')
    ET.SubElement(mount, 'parent', link='base_link')
    ET.SubElement(mount, 'child', link=PREFIX + 'base')
    ET.SubElement(mount, 'origin', xyz=_vector(MOUNT_XYZ), rpy='0 0 0')
    # Arm reaction rocks the light compliant base up to ~0.06 m/s while it stands still
    # (measured 2026-11-13); the interlock must still catch driving (0.25 m/s).
    articulation = dict(schema='robot_lab.urdf_articulation.v1', panel='arm', joints=joints,
                        home={name: 0.0 for name in joints}, base_motion_limits=[0.10, 0.20])
    receipt = dict(source_urdf=ARM_URDF, source_urdf_sha256=_digest(arm_root / ARM_URDF), meshes=meshes,
                   estimates=dict(arm_mass_kg=ARM_MASS_KG, density_kg_m3=round(density, 2), links=estimates,
                                  payload_kg=PAYLOAD_KG, max_joint_speed_rad_s=MAX_JOINT_SPEED,
                                  gravity_torque_nm={PREFIX + k: round(v[0], 2) for k, v in efforts.items()},
                                  max_joint_inertia_kg_m2={PREFIX + k: round(v[1], 3) for k, v in efforts.items()},
                                  design_accel_rad_s2=DESIGN_ACCEL,
                                  effort_margin=EFFORT_MARGIN, mount_xyz_base_link=list(MOUNT_XYZ)),
                   declared_estimates=['arm masses and inertias (vendor publishes no weight)',
                                       'joint speed cap (vendor publishes none)',
                                       'effort caps: 1.5 x (gravity torque + inertia x 2 rad/s^2) with the rated 2 kg payload',
                                       'mount position on the deck (vendor gives six M6 screws, no position)',
                                       'visual colours (Collada materials not carried to STL)'],
                   kept_from_source=['kinematics', 'joint axes', 'URDF position limits', 'collision primitives',
                                     'visual geometry'])
    return articulation, receipt


def attach_f100(base, articulation, source_root, mesh_dir):
    """Mount the vendor F100 subassembly while keeping the pinned bare arm."""
    import trimesh

    source_root, mesh_dir = Path(source_root).resolve(), Path(mesh_dir)
    urdf = source_root / F100_URDF
    expanded = subprocess.check_output(['xacro', str(urdf), 'with_gripper:=true',
        'collision_mesh_dir:=collision_gazebo', 'gazebo_contact_response:=false'], timeout=45)
    source = ET.fromstring(expanded)
    flange_joint = source.find("joint[@name='joint6_to_joint5']")
    mounted_joint = base.find("joint[@name='pro630_joint6_to_joint5']")
    if (flange_joint is None or mounted_joint is None or not np.allclose(
            _origin_matrix(flange_joint.find('origin')), _origin_matrix(mounted_joint.find('origin')),
            atol=1e-8, rtol=0.)):
        raise ValueError('F100 source flange transform differs from the composed arm')
    links = [link for link in source.findall('link') if link.get('name').startswith('gripper_')]
    names = {link.get('name') for link in links}
    joints = [joint for joint in source.findall('joint') if joint.find('child').get('link') in names]
    if len(links) != 9 or len(joints) != 9 or any(joint.find('parent').get('link') not in names | {'link6'}
                                                 for joint in joints):
        raise ValueError('Unexpected F100 subassembly topology')
    driver = next(joint for joint in joints if joint.get('name') == F100_LEADER)
    if driver.get('type') != 'revolute' or driver.find('mimic') is not None:
        raise ValueError('F100 source needs one independent rotary actuator')
    low, high = (float(driver.find('limit').get(key)) for key in ('lower', 'upper'))
    if (low, high) != (0., 1.):
        raise ValueError('Unexpected F100 actuator range')
    mesh_dir.mkdir(parents=True, exist_ok=True)
    resources = {}
    for link in links:
        for mesh in link.findall('.//mesh'):
            uri = mesh.get('filename', '')
            if not uri.startswith('package://mycobot_description/'):
                raise ValueError('Unexpected F100 model mesh '+uri)
            path = (source_root/uri.removeprefix('package://')).resolve()
            if not path.is_relative_to(source_root) or not path.is_file():
                raise ValueError('F100 source mesh is missing: '+uri)
            resources[uri] = _digest(path)
            if path.suffix.lower() == '.dae':
                asset = ET.parse(path).find('{*}asset/{*}unit')
                if asset is None:
                    raise ValueError('F100 Collada mesh needs declared source units: '+uri)
                visual = trimesh.load(str(path), force='mesh')
                visual.apply_scale(float(asset.get('meter')))
                staged = mesh_dir/(path.stem+'.stl')
                visual.export(str(staged))
                path = staged
            mesh.set('filename', str(path))
        link.set('name', PREFIX+link.get('name'))
        base.append(link)

    leader = PREFIX+F100_LEADER
    caps = dict(mode='position', lower=low, upper=high, effort=.8, max_velocity=.5, kp=12., kd=.45)
    couplings, reversed_axes = {}, []
    for joint in joints:
        name = PREFIX+joint.get('name')
        joint.set('name', name)
        for tag in ('parent', 'child'):
            frame = joint.find(tag)
            frame.set('link', PREFIX+frame.get('link'))
        if joint.get('type') == 'revolute':
            limit = joint.find('limit')
            if name != leader:
                mimic = joint.find('mimic')
                if (mimic is None or mimic.get('joint') != F100_LEADER or float(mimic.get('offset', 0.)) != 0.
                        or float(mimic.get('multiplier', 0.)) not in (-1., 1.)):
                    raise ValueError('Unexpected F100 follower linkage: '+name)
                sign = float(mimic.get('multiplier'))
                bounds = sorted(sign*float(limit.get(key)) for key in ('lower', 'upper'))
                if low < bounds[0] or high > bounds[1]:
                    raise ValueError('F100 coupled travel exceeds its original follower limits: '+name)
                # Flip the negative follower's coordinate axis together with
                # its position sign. The original spatial motion is preserved.
                if sign < 0.:
                    axis = joint.find('axis')
                    axis.set('xyz', _vector(-float(value) for value in axis.get('xyz').split()))
                    reversed_axes.append(name)
                mimic.attrib.update(joint=leader, multiplier='1', offset='0')
                couplings[name] = dict(joint=leader, multiplier=1., offset=0., mechanical=True, passive=True)
            for key, value in (('lower', low), ('upper', high), ('effort', caps['effort']),
                               ('velocity', caps['max_velocity'])):
                limit.set(key, format(value, 'g'))
            joint.remove(joint.find('dynamics'))
            ET.SubElement(joint, 'dynamics', damping='.08', friction='0')
        base.append(joint)
    if len(couplings) != 5:
        raise ValueError('F100 requires five source follower joints')
    articulation['joints'][leader] = caps
    articulation['home'][leader] = low
    articulation['couplings'] = couplings
    articulation['grip_joints'] = [leader]
    articulation['grip_presets'] = {leader: {'open': high, 'close': low}}
    return dict(source_urdf=F100_URDF, source_urdf_sha256=_digest(urdf), resource_sha256=resources,
                mesh_license_sha256=_digest(source_root/'mycobot_description/LICENSE'),
                reversed_follower_axes=reversed_axes, lab_servo=caps,
                source_mount='joint6_to_gripper_connect', original_followers=5)
