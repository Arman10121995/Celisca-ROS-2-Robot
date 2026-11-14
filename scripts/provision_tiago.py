#!/usr/bin/env python3
"""Install the complete pinned official PAL TIAGo, not a mesh-only derivative."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/robot_lab_utils'))
from robot_lab_utils.tiago_source import contracts, portable_description
from robot_lab_utils.urdf_articulation import URDFArticulation
from catalog_external_assets import checkout, digest, load_source
from provision_extension_assets import (checkpoint_robots, robot_entity,
                                         source_package_prefix, write_json)

SOURCES = ('pal_tiago_robot', 'pal_pmb2_robot', 'pal_urdf_utils',
           'pal_gripper', 'pal_hey5')
ENTRY = 'tiago_description/robots/tiago.urdf.xacro'
NAME = 'asset_pal_tiago'


def provision(store, download=False):
    sources = [load_source(n) for n in SOURCES]
    roots = []
    for source in sources:
        if download:
            for entry in source['entries']:
                checkout(source, entry, store)
        path = store/'sources'/source['id']/source['revision']
        if subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=path, text=True).strip() != source['revision']:
            raise ValueError('Unexpected source revision: '+source['id'])
        if subprocess.check_output(['git', 'status', '--porcelain'], cwd=path, text=True):
            raise ValueError('Preserve edited source before provisioning: '+source['id'])
        license_text = (path/'LICENSE').read_text()
        if 'Apache License' not in license_text or 'Version 2.0' not in license_text:
            raise ValueError('Unexpected source license: '+source['id'])
        roots.append(path)
    environment = source_package_prefix(roots, store/'prefixes/pal_tiago')
    expanded = subprocess.check_output(['xacro', str(roots[0]/ENTRY), 'base_type:=pmb2',
        'end_effector:=pal-gripper', 'use_sim_time:=true', 'is_public_sim:=true',
        'gazebo_version:=gazebo'], env=environment, text=True, timeout=60)
    document = ET.fromstring(expanded)
    drive, articulation = contracts(document)
    motion_path = roots[0]/'tiago_bringup/config/motions/tiago_motions_general.yaml'
    source_home = yaml.safe_load(motion_path.read_text())['/**']['ros__parameters']['motions']['home']
    endpoint = dict(zip(source_home['joints'], source_home['positions'][-len(source_home['joints']):]))
    if any(articulation['home'].get(n) != v for n, v in endpoint.items()):
        raise ValueError('TIAGo Home endpoint differs from pinned official motion')
    degenerate = {}
    for link in document.findall('link'):
        tensor = link.find('inertial/inertia')
        if tensor is None:
            continue
        xx, xy, xz, yy, yz, zz = (float(tensor.get(k)) for k in ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz'))
        determinant = xx*(yy*zz-yz*yz)-xy*(xy*zz-yz*xz)+xz*(xy*yz-yy*xz)
        if not (xx > 0 and xx*yy-xy*xy > 0 and determinant > 0):
            degenerate[link.get('name')] = dict(mass=float(link.find('inertial/mass').get('value')),
                                                inertia=dict(tensor.attrib))
    sensors = portable_description(document)
    packages = {}
    for source_root in roots:
        for xml in source_root.rglob('package.xml'):
            package = ET.parse(xml).getroot().findtext('name')
            if package in packages:
                raise ValueError('Duplicate source package: '+package)
            packages[package] = xml.parent
    resources = {}
    for item in [*document.findall('.//mesh'), *document.findall('.//texture')]:
        uri = item.get('filename', '')
        if not uri.startswith('package://'):
            raise ValueError('Unexpected source resource URI: '+uri)
        package, _, relative = uri.removeprefix('package://').partition('/')
        if package not in packages:
            raise ValueError('Missing source package: '+package)
        path = (packages[package]/relative).resolve(strict=True)
        if not path.is_relative_to(packages[package].resolve()) or not path.is_file():
            raise ValueError('Source resource escapes package: '+uri)
        resources[uri] = dict(path=str(path), sha256=digest(path))
        item.set('filename', str(path))
    document.set('name', NAME)
    description = ET.tostring(document, encoding='unicode')
    identity = hashlib.sha256((description+json.dumps([drive, articulation, sensors], sort_keys=True)).encode()).hexdigest()
    target = store/'models'/NAME/identity
    target.mkdir(parents=True, exist_ok=True)
    model = target/'robot.urdf'
    model.write_text(description)
    (target/'source-expanded.urdf').write_text(expanded)
    # Validate the complete adapter against exactly the installed source model.
    URDFArticulation(articulation, description, model)
    for source, path in zip(sources, roots):
        (target/('LICENSE-'+source['id'])).write_bytes((path/'LICENSE').read_bytes())
    changes = [
        'Complete official PMB2/pal-gripper public-simulation assembly; source kinematics, inertias, collisions and fixed casters retained',
        'Source 6 Nm wheels with declared 0.2 m/s / 0.4 rad/s lab velocity caps',
        'Twelve independent source position channels, declared bounded lab PD gains/rates plus existing plant gravity feedforward',
        'Isaac opt-in native actual-pose generalized gravity compensation each step, replacing slide-only feedforward within the same source effort budget',
        'Isaac opt-in scalar source link-friction bindings on original/fused/instanced colliders with minimum combination; not Gazebo solver equivalence',
        'Source home arm/torso target from official motion file; lab 0.04 m finger-opening Home target, head zero',
        'Declared panoramic lab lidar at vendor laser mount, ideal root IMU, source-head RGB-D optics at reduced 320x240/5 Hz',
        'Source modern Gazebo runtime plugins removed for portable backends; no Fortress support claimed',
        'Source sonars, wrist FT and hardware driver not exposed by this adapter']
    receipt = dict(schema='robot_lab.source_adapter.v1', robot=NAME, source_id=sources[0]['id'],
        source_entry=ENTRY, sources=[dict(id=s['id'], repository=s['repository'], revision=s['revision'],
            license='Apache-2.0', license_sha256=digest(p/'LICENSE')) for s, p in zip(sources, roots)],
        source_expanded_sha256=digest(target/'source-expanded.urdf'), description=str(model),
        description_sha256=digest(model), resources=resources, drive=drive, articulation=articulation,
        sensor_config=sensors, source_mass_kg=drive['mujoco_expected_mass_kg'], lab_changes=changes,
        source_home=dict(path=str(motion_path), sha256=digest(motion_path), endpoint=endpoint,
                         scope='Endpoint joint target only; not the full vendor waypoint trajectory or collision plan'),
        source_degenerate_inertials=degenerate,
        importer_limitations='Original antenna tensors are indefinite and tool-frame tensors zero. '
            'Source URDF remains unchanged; existing backend importers may regularize these. '
            'MuJoCo also bounds tiny masses/inertias and excludes overlapping rest-pose contact pairs. '
            'Compiled dynamics fidelity is not qualified.',
        runtime_mission_qualified=False)
    evidence = target/'integration.json'
    write_json(evidence, receipt)
    notes = ('Complete official Apache-2.0 PAL TIAGo (PMB2, 7-axis arm, torso, independent parallel fingers, pan/tilt head). '
        'Original mechanics and source public-simulation fixed low-friction casters; declared bounded lab servo/sensor adapters. '
        'Drive and Mobile arm use existing command leases, Stop, measured feedback and mutual inhibition. '
        'Source antenna/tool inertias are degenerate; backend importer regularization and rest-contact exclusions are declared, not faithful-dynamics qualification. '
        'MuJoCo/PyBullet/Isaac experimental adapter; no physical mission certificate. '
        'Gazebo Fortress source port, camera/object pipelines, payload/costmap integration and wider missions remain pending.')
    profile = dict(package='robot_lab_robots', name=NAME, xacro=str(model), type='mobile_manipulator',
        external_asset=True, asset_origin='extension', source_id=sources[0]['id'],
        source_revision=sources[0]['revision'], source_entry=ENTRY, drive=drive,
        articulation=articulation, sensor_config=sensors, drive_in_display=True,
        supported_simulators=['mujoco', 'pybullet', 'isaac'], supported_modes=['display'],
        experimental_modes_by_simulator={sim: ['display'] for sim in ('mujoco', 'pybullet', 'isaac')},
        default_mode='display', default_map='nav_empty', spawn=dict(z=.006),
        features=['velocity_base', 'articulated_arm', 'lidar_2d', 'rgbd_camera', 'imu'],
        implementation_state='experimental', validation_state='pending', qualification_state='experimental_unqualified',
        notes=notes)
    entity = robot_entity(NAME, profile, sources[0], ENTRY, evidence)
    entity.update(name='PAL Robotics TIAGo', robot_class='mobile',
        tags=['extension', 'mobile_manipulator', 'complete_robot', 'two_wheels', 'arm', 'parallel_gripper', 'experimental_control'],
        capabilities=['display', 'control', 'joint_control', 'manipulation', 'lidar_2d', 'rgbd_camera', 'imu'])
    entity['source']['license'] = 'Apache-2.0 (all five pinned official PAL sources)'
    entity['frames'] = list(dict.fromkeys(['base_footprint', *entity['frames']]))
    for sensor in entity.get('sensors', []):
        if sensor.get('type') == 'imu':
            sensor['frame'] = sensors['imu_link_name']
    for item in entity['evidence']:
        item['date'] = '2026-11-14'
    checkpoint_robots(store, {NAME: profile}, [entity], [dict(source_id=sources[0]['id'],
        revision=sources[0]['revision'], entry=ENTRY, status='installed', check=str(evidence),
        profiles=[dict(id=NAME, kind='robot', support='Experimental complete source adapter', notes=notes)])])
    return receipt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--download', action='store_true')
    parser.add_argument('--store', type=Path, default=Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT',
        '/workspace/molar/robot_lab_runtime'))/'external_assets')
    args = parser.parse_args()
    args.store = args.store.resolve()
    args.store.mkdir(parents=True, exist_ok=True)
    if args.store.stat().st_dev == Path('/').stat().st_dev:
        parser.error('Assets must remain on the mounted workspace SSD')
    # Same publication lock as the other incremental source provisioners.
    lock = args.store/'locks/provision.lock'
    lock.parent.mkdir(parents=True, exist_ok=True)
    with lock.open('a') as handle:
        fcntl.flock(handle, fcntl.LOCK_EX)
        result = provision(args.store, args.download)
    print(json.dumps(dict(robot=result['robot'], description=result['description'],
                         source_mass_kg=result['source_mass_kg']), indent=2))


if __name__ == '__main__':
    main()
