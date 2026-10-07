#!/usr/bin/env python3
"""Install pinned extension models and worlds into the normal Launch catalog.

An agent/bootstrap operation. Downloads and derived geometry remain on SSD;
the operator selects installed assets in Launch, without a download tab.
"""
import argparse
import copy
import fcntl
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time
import urllib.request
import xml.etree.ElementTree as ET

import yaml

from catalog_external_assets import CATALOG, ROOT, checkout, digest, load_source, run_git, prepare_urdf_for_import

sys.path.insert(0, str(ROOT/'src/robot_lab_utils'))


def write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix('.new')
    temporary.write_text(json.dumps(data, indent=2)+'\n')
    temporary.replace(path)


def write_yaml(path, data):
    temporary = path.with_suffix('.new')
    temporary.write_text(yaml.safe_dump(data, sort_keys=False))
    temporary.replace(path)


def source_checkout(source, store, download):
    root = store/'sources'/source['id']/source['revision']
    if download:
        first = next(e for e in source['entries'] if isinstance(e, str))
        root, _ = checkout(source, first, store)
        subtree = {'robot_assets': 'urdfs/robots', 'gazebo_world_dataset': 'worlds',
                   'gazebo_examples':'examples/worlds','turtlebot3_vendor':'turtlebot3_description',
                   'husky_vendor':'husky_description','turtlebot4_vendor':'turtlebot4_description',
                   'create3_vendor':'irobot_create_common'}.get(source['id'])
        directories = [subtree] if subtree else [e for e in source['entries'] if isinstance(e, str)]
        with (store/'locks'/(source['id']+'-'+source['revision']+'.lock')).open('a') as handle:
            fcntl.flock(handle, fcntl.LOCK_EX)
            run_git(['sparse-checkout', 'add', *directories], root, timeout=3600)
    revision = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=root, text=True).strip()
    if revision != source['revision']:
        raise ValueError('Unexpected source revision: '+str(root))
    return root


def hand_dependencies(store, download):
    """Fetch the manufacturer's exact hand revision, retaining its license."""
    source = load_source('qb_hand_description')
    root = store/'dependencies'/source['id']/source['revision']
    for entry in source['entries']:
        path = root/entry
        if not path.is_file() and download:
            url = 'https://api.bitbucket.org/2.0/repositories/qbrobotics/qbhand-ros/src/'+source['revision']+'/'+entry
            with urllib.request.urlopen(url, timeout=60) as response:
                content = response.read(8*1024*1024+1)
            if len(content) > 8*1024*1024:
                raise ValueError('Hand dependency exceeds size bound: '+entry)
            path.parent.mkdir(parents=True, exist_ok=True)
            temporary = path.with_suffix(path.suffix+'.new')
            temporary.write_bytes(content); temporary.replace(path)
        if not path.is_file():
            raise ValueError('Missing pinned hand dependency: '+str(path))
    write_json(root/'manifest.json', dict(source=source, sha256={e:digest(root/e) for e in source['entries']}))
    return [root/entry for entry in source['entries']]


def source_package_prefix(roots, prefix):
    """Expose pinned description dependencies to Xacro without system copies."""
    for root in roots:
        for package_xml in root.rglob('package.xml'):
            package = ET.parse(package_xml).getroot().findtext('name')
            marker = prefix/'share/ament_index/resource_index/packages'/package
            marker.parent.mkdir(parents=True, exist_ok=True); marker.touch()
            share = prefix/'share'/package
            if share.is_symlink() and share.resolve() != package_xml.parent.resolve():
                raise ValueError('Conflicting source-backed package: '+package)
            if not share.exists():
                share.symlink_to(package_xml.parent, target_is_directory=True)
    return dict(os.environ, AMENT_PREFIX_PATH=str(prefix)+':'+os.environ.get('AMENT_PREFIX_PATH',''))


def resolve_resource(uri, model, source, paths):
    if not uri:
        raise ValueError('Empty model resource URI')
    relative = uri.removeprefix('file://')
    direct = (model.parent/relative).resolve()
    if direct.is_file() and direct.is_relative_to(source):
        return direct
    if uri.startswith('package://'):
        package, _, relative = uri.removeprefix('package://').partition('/')
        for parent in model.parents:
            if parent == source: break
            for prefix in ('', 'model'):
                resource = parent/prefix/relative
                if resource.is_file(): return resource
    parts = Path(relative).parts
    for length in range(min(len(parts), 7), 0, -1):
        suffix = '/'.join(parts[-length:])
        matches = [path for path in paths if path.as_posix().endswith('/'+suffix)]
        if len(matches) == 1:
            return matches[0]
        if matches and len({digest(p) for p in matches}) == 1:
            return matches[0]
    raise ValueError('Missing or ambiguous original model resource: '+uri)


def prepare_robot_urdf(model, source, output, paths):
    text = model.read_text()
    # One Fetch snapshot contains an obsolete unbound sensor:camera extension.
    # Remove only the recorded Classic Gazebo blocks before XML parsing.
    text = re.sub(r'<gazebo\b[^>]*>.*?</gazebo>', '', text, flags=re.S)
    tree = ET.ElementTree(ET.fromstring(text))
    root = tree.getroot()
    if root.tag != 'robot':
        raise ValueError('Not a standalone URDF robot')
    if not root.findall('link'):
        raise ValueError('Upstream fixture contains no robot links; not a standalone robot')
    # ROS 1 control plugins cannot be loaded by the ROS 2 simulator adapter.
    removed = [child.tag for child in root if child.tag in ('gazebo', 'transmission', 'ros2_control')]
    for child in list(root):
        if child.tag in ('gazebo', 'transmission', 'ros2_control'):
            root.remove(child)
    duplicates = []
    for tag in ('link', 'joint'):
        seen = {}
        for node in list(root.findall(tag)):
            name = node.get('name')
            if name in seen:
                if ET.tostring(node) != ET.tostring(seen[name]):
                    raise ValueError('Conflicting duplicate '+tag+': '+name)
                root.remove(node); duplicates.append(tag+':'+name)
            else: seen[name] = node
    repairs = []
    if model.name == 'r2_left_gripper.urdf':
        # This snapshot contains an unrelated leaf ATI sensor with a missing
        # ankle parent. Preserve the source; omit only that disconnected leaf.
        for joint in list(root.findall('joint')):
            if joint.get('name') != 'r2/fixed/left_ankle_roll/left_leg_ati':
                continue
            parent = joint.find('parent').get('link')
            child = joint.find('child').get('link')
            links = {link.get('name'): link for link in root.findall('link')}
            if parent not in links and child in links:
                if any(j is not joint and j.find('child').get('link') == child for j in root.findall('joint')):
                    repairs.append({'removed_dangling_duplicate_parent_joint':joint.get('name'),
                                    'missing_parent':parent, 'child_retained':child,
                                    'original_joint_xml':ET.tostring(joint,encoding='unicode')})
                    root.remove(joint)
                    continue
                disconnected = {child}
                branch_joints = [joint]
                while True:
                    descendants = [j for j in root.findall('joint') if j not in branch_joints
                                   and j.find('parent').get('link') in disconnected]
                    if not descendants: break
                    branch_joints.extend(descendants)
                    disconnected.update(j.find('child').get('link') for j in descendants)
                repairs.append({'removed_disconnected_branch_joints':[j.get('name') for j in branch_joints],
                                'links':sorted(disconnected), 'missing_parent':parent,
                                'original_joint_xml':[ET.tostring(j,encoding='unicode') for j in branch_joints]})
                for j in branch_joints: root.remove(j)
                for name in disconnected: root.remove(links[name])
    link_names = {link.get('name') for link in root.findall('link')}
    for joint in root.findall('joint'):
        for tag in ('parent', 'child'):
            name = joint.find(tag).get('link')
            if name not in link_names:
                raise ValueError('Upstream joint '+joint.get('name')+' references missing '+tag+' link: '+name)
    resource_hashes = {}
    for node in [*root.iter('mesh'), *root.iter('texture')]:
        resource = resolve_resource(node.get('filename', ''), model, source, paths)
        if resource.read_bytes()[:100].startswith(b'version https://git-lfs.github.com/spec/'):
            raise ValueError('Unfetched LFS geometry: '+str(resource))
        node.set('filename', str(resource))
        resource_hashes[str(resource)] = digest(resource)
    output.mkdir(parents=True, exist_ok=True)
    resolved = output/'resolved.urdf'
    tree.write(resolved, encoding='unicode')
    derived = prepare_urdf_for_import(resolved, output)
    import pybullet as bullet
    client = bullet.connect(bullet.DIRECT)
    try:
        body = bullet.loadURDF(str(derived), useFixedBase=True, physicsClientId=client)
        checked = {'engine': 'pybullet', 'api_version': bullet.getAPIVersion(),
                   'joints': bullet.getNumJoints(body, physicsClientId=client),
                   'visual_shapes': len(bullet.getVisualShapeData(body, physicsClientId=client))}
    finally:
        bullet.disconnect(client)
    return derived, {'native_import': checked, 'removed_legacy_control_tags': removed,
                     'identical_duplicates_removed': duplicates, 'fragment_repairs':repairs,
                     'resource_sha256':resource_hashes}


def native_model(directory):
    scene = directory/'scene.xml'
    if scene.is_file():
        for include in ET.parse(scene).getroot().findall('include'):
            path = directory/include.get('file', '')
            if path.is_file() and ET.parse(path).getroot().tag == 'mujoco':
                return path
    candidates = [p for p in sorted(directory.glob('*.xml'))
                  if 'scene' not in p.stem and ET.parse(p).getroot().tag == 'mujoco']
    if not candidates:
        raise ValueError('No standalone native robot XML')
    return candidates[0]


def turtlebot4_drive(derived, name):
    """Separate control derivative; preserve original geometry and suspension."""
    root = ET.parse(derived).getroot()
    wheels = ('left_wheel_joint', 'right_wheel_joint')
    joints = {joint.get('name'): joint for joint in root.findall('joint')}
    upstream = ET.parse(derived.parent/'input.urdf').getroot()
    springs = [block for block in upstream.findall('gazebo')
               if block.find('springStiffness') is not None]
    if {block.get('reference') for block in springs} != {
            'wheel_drop_left_joint', 'wheel_drop_right_joint'}:
        raise ValueError('Missing original Create3 wheel-drop springs')
    for spring in springs:
        root.append(copy.deepcopy(spring))
    # Retain the original mounting geometry, optical frame and calibration.
    # Explicit lab sampling rates avoid importing the vendor's 62/30 Hz load;
    # the untouched source still records those upstream rates.
    sensors = {}
    for block in upstream.findall('gazebo'):
        original = block.find('sensor')
        if original is None or original.get('name') not in ('rplidar', 'rgbd_camera', 'imu'):
            continue
        mounted = copy.deepcopy(block)
        sensor = mounted.find('sensor')
        frame = block.get('reference')
        rate = 50.0 if original.get('name') == 'imu' else 5.0
        sensor.find('update_rate').text = str(rate)
        ET.SubElement(sensor, 'gz_frame_id').text = (
            original.findtext('camera/optical_frame_id')
            if original.get('name') == 'rgbd_camera' else frame)
        if original.get('name') in ('rplidar', 'imu'):
            ET.SubElement(sensor, 'topic').text = '/scan' if original.get('name') == 'rplidar' else '/imu'
        sensors[original.get('name')] = (frame, original)
        root.append(mounted)
    if set(sensors) != {'rplidar', 'rgbd_camera', 'imu'}:
        raise ValueError('Missing original TurtleBot4 lidar/camera/IMU declarations')
    # Keep the original sensor links in Gazebo. Fixed-joint lumping otherwise
    # changes camera topic paths away from the link named by the ROS bridge.
    sensor_links = {frame for frame, _ in sensors.values()}
    sensor_joints = {joint.get('name') for joint in upstream.findall('joint')
                     if joint.find('child').get('link') in sensor_links}
    for block in upstream.findall('gazebo'):
        if block.get('reference') in sensor_joints and block.find('preserveFixedJoint') is not None:
            root.append(copy.deepcopy(block))
    lidar_frame, lidar = sensors['rplidar']
    camera_frame, camera = sensors['rgbd_camera']
    sensor_config = dict(laser_link_name=lidar_frame, scan_rate=5.0,
        scan_samples=int(lidar.findtext('lidar/scan/horizontal/samples')),
        scan_range_min=float(lidar.findtext('lidar/range/min')),
        scan_range_max=float(lidar.findtext('lidar/range/max')),
        camera_link_name=camera_frame,
        camera_optical_frame=camera.findtext('camera/optical_frame_id'),
        camera_rate=5.0, camera_width=int(camera.findtext('camera/image/width')),
        camera_height=int(camera.findtext('camera/image/height')),
        camera_horizontal_fov=float(camera.findtext('camera/horizontal_fov')),
        camera_near=float(camera.findtext('camera/clip/near')),
        camera_far=float(camera.findtext('camera/clip/far')))
    from robot_lab_utils.sensor_config import sensor_parameters
    sensor_config = sensor_parameters(sensor_config, ET.tostring(root, encoding='unicode'))
    for wheel in wheels:
        if joints[wheel].get('type') != 'continuous':
            raise ValueError('Unexpected TurtleBot4 wheel joint: '+wheel)
        # Original continuous joints omit limits. Match the physics bridges'
        # 5 Nm motor cap and bound this simulation derivative's joint speed.
        ET.SubElement(joints[wheel], 'limit', effort='5', velocity='15')
    drive = dict(type='diff', left_wheel_joint=wheels[0], right_wheel_joint=wheels[1],
                 wheel_radius=0.03575, wheel_separation=0.233, max_speed=0.3,
                 max_accel=0.5, max_angular_speed=1.2, max_angular_accel=2.0)
    params = dict(use_sim_time=True, use_stamped_vel=False, left_wheel_names=[wheels[0]],
                  right_wheel_names=[wheels[1]], wheel_radius=drive['wheel_radius'],
                  wheel_separation=drive['wheel_separation'], base_frame_id='base_link',
                  odom_frame_id='odom', enable_odom_tf=False, open_loop=False,
                  cmd_vel_timeout=0.5, publish_rate=50.0, publish_limited_velocity=True)
    for axis, key, speed, accel in (('linear','x','max_speed','max_accel'),
                                  ('angular','z','max_angular_speed','max_angular_accel')):
        params[axis] = {key: dict(has_velocity_limits=True, max_velocity=drive[speed],
                                  has_acceleration_limits=True, max_acceleration=drive[accel])}
    config = {'controller_manager': {'ros__parameters': {
        'update_rate': 100, 'use_sim_time': True,
        'robot_lab_controller': {'type': 'diff_drive_controller/DiffDriveController'},
        'joint_state_broadcaster': {'type': 'joint_state_broadcaster/JointStateBroadcaster'}}},
        'joint_state_broadcaster': {'ros__parameters': {'use_sim_time': True}},
        'robot_lab_controller': {'ros__parameters': params}}
    config_path = derived.parent/'drive-controllers.yaml'
    write_yaml(config_path, config)
    control = ET.SubElement(root, 'ros2_control', name='TurtleBot4System', type='system')
    hardware = ET.SubElement(control, 'hardware')
    ET.SubElement(hardware, 'plugin').text = 'ign_ros2_control/IgnitionSystem'
    for joint in (*wheels, 'wheel_drop_left_joint', 'wheel_drop_right_joint'):
        element = ET.SubElement(control, 'joint', name=joint)
        if joint in wheels:
            command = ET.SubElement(element, 'command_interface', name='velocity')
            ET.SubElement(command, 'param', name='min').text = '-15'
            ET.SubElement(command, 'param', name='max').text = '15'
        for interface in ('position', 'velocity'):
            ET.SubElement(element, 'state_interface', name=interface)
    gazebo = ET.SubElement(root, 'gazebo')
    plugin = ET.SubElement(gazebo, 'plugin', filename='ign_ros2_control-system',
                           name='ign_ros2_control::IgnitionROS2ControlPlugin')
    ET.SubElement(plugin, 'parameters').text = str(config_path)
    ET.SubElement(gazebo, 'plugin', filename='ignition-gazebo-imu-system',
                  name='ignition::gazebo::systems::Imu')
    rendering = ET.SubElement(gazebo, 'plugin', filename='ignition-gazebo-sensors-system',
                              name='ignition::gazebo::systems::Sensors')
    ET.SubElement(rendering, 'render_engine').text = 'ogre2'
    truth = ET.SubElement(gazebo, 'plugin', filename='ignition-gazebo-odometry-publisher-system',
                          name='ignition::gazebo::systems::OdometryPublisher')
    for key, value in {'odom_frame': 'world', 'robot_base_frame': 'base_link',
                       'odom_topic': '/model/'+name+'/odometry_truth',
                       'tf_topic': '/model/'+name+'/truth_pose',
                       'odom_publish_frequency': '50', 'dimensions': '3'}.items():
        ET.SubElement(truth, key).text = value
    for link in ('left_wheel', 'right_wheel', 'front_caster_link'):
        surface = ET.SubElement(root, 'gazebo', reference=link)
        for key in ('mu1', 'mu2'):
            ET.SubElement(surface, key).text = '0.1' if 'caster' in link else '1.0'
    path = derived.parent/'drive.urdf'
    ET.ElementTree(root).write(path, encoding='unicode', xml_declaration=True)
    return path, drive, config_path, sensor_config


def robot_class(name):
    if any(value in name for value in ('turtlebot3','turtlebot4','husky')):
        return 'mobile'
    if any(value in name for value in ('fetch', 'pr2', 'stretch', 'tidybot', 'google_robot', 'eve', 'halodi')):
        return 'hybrid'
    if any(value in name for value in ('anymal', 'spot', 'go1', 'go2', 'a1', 'barkour')):
        return 'legged'
    if any(value in name for value in ('humanoid', 'cassie', 'apollo', 'booster', 'n1', 'g1', 'h1', 'adam', 'op3', 'toddler', 'valkyrie', 'r2', 'ginger', 'human')):
        return 'humanoid'
    if any(value in name for value in ('crazyflie', 'skydio')):
        return 'aerial'
    return 'manipulator'


def robot_entity(name, profile, source, entry, evidence):
    entity = {'id': name, 'version': '1.0.0', 'name': name.replace('_', ' ').title(),
        'status': 'available', 'robot_class': robot_class(name), 'maturity': 'prototype',
        'ros_package': profile['package'], 'supported_simulators': profile['supported_simulators'],
        'capabilities': (['display', 'joint_control'] if profile.get('arm_control') else
                         ['display', 'control'] if profile.get('drive_in_display') else ['display']),
        'assets': {'urdf': profile['xacro']},
        'source': {'repository': source['repository'], 'revision': source['revision'],
                   'license': source['license_note']},
        'description': profile['notes'], 'state_interfaces': ['sensor_msgs/JointState'],
        'evidence': [{'kind': 'integration_test', 'reference': str(evidence),
                     'description': 'Named model import/geometry check only; no movement or mission qualification',
                     'date': '2026-10-05'}], 'tags': ['extension', source['id'], 'display']}
    sensor = profile.get('sensor_config')
    if sensor:
        entity['sensors'] = [
            dict(type='lidar', name='lds' if source['id'] == 'turtlebot3_vendor' else 'rplidar', frame=sensor['laser_link_name'],
                 topic='/scan', message_type='sensor_msgs/LaserScan'),
            dict(type='imu', name='imu', frame='imu_link', topic='/imu/out',
                 message_type='sensor_msgs/Imu'),
            dict(type='odometry', name='wheel_odometry', frame='odom', topic='/odom',
                 message_type='nav_msgs/Odometry')]
        if sensor.get('camera_rate', 0) > 0:
            entity['sensors'][1:1] = [
                dict(type='camera', name='oakd_rgb', frame=sensor['camera_optical_frame'],
                     topic='/oakd/rgb/image_raw', message_type='sensor_msgs/Image'),
                dict(type='depth_camera', name='oakd_depth', frame=sensor['camera_optical_frame'],
                     topic='/oakd/depth/image_raw', message_type='sensor_msgs/Image')]
        entity['state_interfaces'].append('nav_msgs/Odometry')
        entity['command_interfaces'] = ['geometry_msgs/Twist']
        entity['frames'] = ['base_footprint' if source['id'] == 'turtlebot3_vendor' else 'base_link',
                            'odom', sensor['laser_link_name'], 'imu_link']
        if sensor.get('camera_rate', 0) > 0:
            entity['frames'].append(sensor['camera_optical_frame'])
        entity['locomotion'] = dict(type='differential_drive', dof=2,
            max_velocity=profile['drive']['max_speed'],
            max_acceleration=profile['drive']['max_accel'])
    modes = set(profile.get('supported_modes', []))
    capabilities = set(entity['capabilities'])
    if modes & {'loc', 'slam', '3d_slam', 'nav'}:
        capabilities.update(['localization', 'state_estimation', 'sensor_fusion', 'perception'])
    if modes & {'slam', '3d_slam'}:
        capabilities.add('mapping')
    if 'nav' in modes:
        capabilities.update(['navigation', 'costmap', 'global_planning', 'local_planning'])
    entity['capabilities'] = sorted(capabilities)
    for screen in profile.get('runtime_screens', []):
        for reference in screen['reports']:
            entity['evidence'].append(dict(kind='runtime_test', reference=reference,
                description='%s / %s / %s; named screen only' % (screen['backend'], screen['mode'],
                    ', '.join(screen['maps'])), date='2026-10-06'))
    return entity


def checkpoint_robots(store, profiles, entities, records):
    """Publish completed imports incrementally, so interrupted batches resume."""
    installed = store/'installed'
    path = installed/'robots.yaml'
    old = yaml.safe_load(path.read_text()).get('robots', {}) if path.is_file() else {}
    old.update(profiles)
    write_yaml(path, {'robots': old})
    path = installed/'registry_robots.yaml'
    previous = yaml.safe_load(path.read_text()) or [] if path.is_file() else []
    merged = {item['id']: item for item in previous}
    merged.update({item['id']: item for item in entities})
    write_yaml(path, list(merged.values()))
    path = installed/'integration-report.json'
    previous = json.loads(path.read_text()) if path.is_file() else {'entries': []}
    merged = {(item['source_id'], item['entry']): item for item in previous['entries']}
    merged.update({(item['source_id'], item['entry']): item for item in records})
    previous.update(entries=list(merged.values()), scope='Incremental installed imports; no robot mission qualification')
    write_json(path, previous)


def install_robots(store, download, selected_source=None):
    from robot_lab_utils.native_mjcf_assets import export_display_urdf
    import mujoco
    profiles, entities, records = {}, [], []
    for source in yaml.safe_load(CATALOG.read_text())['sources']:
        if source['id'] not in ('robot_assets', 'mujoco_menagerie','turtlebot3_vendor','husky_vendor',
                               'turtlebot4_vendor') or selected_source and source['id'] != selected_source:
            continue
        source_root = source_checkout(source, store, download)
        resources = [p for p in source_root.rglob('*') if p.is_file() and '.git' not in p.parts]
        if source['id'] == 'robot_assets':
            resources.extend(hand_dependencies(store, download))
        dependency_roots = []
        if source['id'] == 'turtlebot4_vendor':
            dependency_roots.append(source_checkout(load_source('create3_vendor'), store, download))
            resources.extend(p for root in dependency_roots for p in root.rglob('*')
                             if p.is_file() and '.git' not in p.parts)
        if source['id'] == 'turtlebot3_vendor':
            dependency_roots.append(source_checkout(load_source('turtlebot3_simulation_vendor'), store, download))
        for entry in source['entries']:
            record = {'source_id': source['id'], 'revision': source['revision'], 'entry': entry, 'profiles': []}
            evidence = store/'installed'/'checks'/(source['id']+'-'+hashlib.sha256(entry.encode()).hexdigest()[:16]+'.json')
            try:
                path = source_root/entry
                if source['id'] == 'mujoco_menagerie':
                    model_path = native_model(path)
                    model = mujoco.MjModel.from_xml_path(str(model_path))
                    name = 'menagerie_'+path.name
                    derived = store/'models'/name/source['revision']/'display.urdf'
                    check = export_display_urdf(model, derived)
                    if check['unsupported_geometry']:
                        raise ValueError('Unexported native geometry: '+str(check['unsupported_geometry']))
                    check['native_import'] = {'engine': 'mujoco', 'version': mujoco.__version__,
                        'bodies': model.nbody, 'joints': model.njnt, 'geometries': model.ngeom, 'actuators': model.nu}
                    profile = {'package': 'robot_lab_robots', 'xacro': str(derived), 'native_mjcf': str(model_path),
                               'supported_simulators': ['mujoco'], 'spawn': {'z': '0.0'}}
                    if name == 'menagerie_franka_emika_panda':
                        profile['arm_control'] = 'panda'
                    del model
                else:
                    if source['id'] in ('turtlebot3_vendor','husky_vendor','turtlebot4_vendor'):
                        stem = path.name.removesuffix('.xacro').removesuffix('.urdf')
                        name = 'asset_'+stem
                        if source['id'] == 'turtlebot4_vendor':
                            name = 'asset_turtlebot4_'+path.parent.name
                        output = store/'models'/name/source['revision']
                        output.mkdir(parents=True,exist_ok=True)
                        prefix = store/'ros_source_prefixes'/source['id']/source['revision']
                        environment = source_package_prefix([source_root, *dependency_roots], prefix)
                        xacro_args = ['is_sim:=false','gazebo_controllers:=',
                                      'urdf_extras:='+str(path.parent/'empty.urdf')] if source['id']=='husky_vendor' else []
                        if source['id'] == 'turtlebot4_vendor':
                            xacro_args = ['gazebo:=ignition', 'namespace:=']
                        xacro_source = path
                        if source['id']=='husky_vendor':
                            # Xacro eagerly resolves this obsolete controller
                            # package default, even when is_sim=false. The
                            # Display derivative has no upstream controller.
                            text = re.sub(r'(<xacro:arg\s+name="gazebo_controllers"\s+default=")[^"]*',
                                          r'\1', path.read_text())
                            xacro_source = output/'display-input.xacro'
                            xacro_source.write_text(text)
                        expanded = subprocess.check_output(['xacro',str(xacro_source),*xacro_args],env=environment,text=True,timeout=60)
                        derived_input = output/'input.urdf'; derived_input.write_text(expanded)
                        path = derived_input
                    else:
                        family = entry.split('/')[2]
                        stem = re.sub(r'[^a-zA-Z0-9_]+', '_', path.name.removesuffix('.urdf').removesuffix('.urdf'))
                        name = 'asset_'+family+('_'+stem if stem not in ('model', family) else '')
                    cached = json.loads(evidence.read_text()) if evidence.is_file() else {}
                    derived = store/'models'/name/source['revision']/'model.urdf'
                    if (cached.get('import_recipe') == 'resolved-original-resources-v3'
                            and cached.get('model_sha256') == digest(path) and derived.is_file()
                            and cached.get('derived_urdf_sha256') == digest(derived)
                            and all(Path(p).is_file() and digest(Path(p)) == sha
                                    for p,sha in cached.get('resource_sha256',{}).items())):
                        check = cached
                    else:
                        derived, check = prepare_robot_urdf(path, source_root,
                            store/'models'/name/source['revision'], resources)
                        check['import_recipe'] = 'resolved-original-resources-v3'
                    model_path = path
                    profile = {'package': 'robot_lab_robots', 'xacro': str(derived),
                               'supported_simulators': ['gazebo', 'pybullet', 'mujoco', 'isaac']}
                profile.update(name=name, supported_modes=['display'], default_mode='display',
                    features=['joint_control'] if profile.get('arm_control') else [],
                    source_id=source['id'], source_entry=entry,
                    notes='Installed source-pinned model for Display. Joint control, walking, localization, SLAM and navigation require separate controller integration and qualification.')
                if profile.get('arm_control') == 'panda':
                    from robot_lab_utils.asset_support import apply_recorded_panda_hand, apply_recorded_panda_planning
                    support_path = ROOT/'docs/status/asset-runtime-support.yaml'
                    support = yaml.safe_load(support_path.read_text()) if support_path.is_file() else {}
                    profile = apply_recorded_panda_hand(profile, source['revision'],
                        (support or {}).get('hands', {}).get(name), ROOT)
                    profile = apply_recorded_panda_planning(profile, source['revision'],
                        (support or {}).get('arm_planning', {}).get(name), ROOT)
                    profile['notes'] = ('Native Panda joint/Home/Stop controls in the Arm tab on MuJoCo. '
                                        'Position trajectories use actual actuators. Cartesian planning and other backends remain pending.')
                    if profile.get('arm_planning'):
                        profile['notes'] = ('Arm tab: joint/Home and MoveIt Cartesian Plan/Execute through native '
                            'MuJoCo actuators. Two offsets, static nav_empty collision rejection, stale-plan '
                            'invalidation, Cancel/Stop/watchdog and reset measured. Other maps are experiments; '
                            'payload planning, Servo and other backends remain pending.')
                    if profile.get('hand_control'):
                        profile['notes'] += (' Hand tab: original coupled fingers, bounded-force opening/closing, '
                            'Cancel/Stop and reset. Physical cube lift/hold/release recorded on MuJoCo/nav_empty; '
                            'other objects, planned grasping and dexterous hands require separate qualification.')
                if source['id'] == 'turtlebot4_vendor':
                    controlled, drive, controllers, sensor_config = turtlebot4_drive(derived, name)
                    profile.update(xacro=str(controlled), drive=drive, drive_in_display=True,
                                   sensor_config=sensor_config,
                                   features=['velocity_base', 'lidar_2d', 'rgbd_camera'],
                                   controllers=['joint_state_broadcaster', 'robot_lab_controller'])
                    profile['notes'] = ('Original TurtleBot4/Create3 geometry and suspension with physical wheel '
                        'control in Display; bounded GUI Drive/WASD uses the lab controller, not vendor firmware. '
                        'Sensors, SLAM/navigation, hazards/docking and visual-material parity remain pending.')
                    check.update(drive_urdf=str(controlled), drive_urdf_sha256=digest(controlled),
                                 drive_controllers_sha256=digest(controllers))
                    from robot_lab_utils.gazebo_physics_world import COMPLIANT_STEP_BOUNDS
                    if name in COMPLIANT_STEP_BOUNDS:
                        profile['gazebo_max_physics_step_s'] = COMPLIANT_STEP_BOUNDS[name]
                    from robot_lab_utils.asset_support import apply_recorded_modes
                    support_path = ROOT/'docs/status/asset-runtime-support.yaml'
                    support = yaml.safe_load(support_path.read_text()) if support_path.is_file() else {}
                    profile = apply_recorded_modes(profile, source['revision'],
                        (support or {}).get('robots', {}).get(name), ROOT)
                    if profile.get('runtime_screens'):
                        profile['notes'] = ('Original TurtleBot4/Create3 wheels, suspension, RPLidar and OAK-D frames; '
                            'lab sensors at 5 Hz. Recorded localization, mapping and Nav2 modes vary by backend; '
                            'see robot status and exact runtime screens. Other maps are experiments. '
                            'Vendor hazards/docking and complete visual-material parity remain pending.')
                    # Measured 2026-10-06: the suspended Create3 base slips its
                    # wheels in the coarse (10 ms) Gazebo worlds and the wheel
                    # odometry diverges from truth; Gazebo launches a capped
                    # derivative of the selected world for this robot only.
                    if name in COMPLIANT_STEP_BOUNDS:
                        profile['notes'] += (' Gazebo caps the selected world to a 2 ms physics '
                            'step for this base (measured wheel-odometry divergence in the stock '
                            '10 ms worlds); other backends launch the world unchanged.')
                if source['id'] == 'turtlebot3_vendor':
                    from extension_mobile_control import turtlebot3_drive
                    controlled, drive, controllers, sensor_config, control_source = turtlebot3_drive(
                        derived, name, dependency_roots[0])
                    profile.update(xacro=str(controlled), drive=drive, drive_in_display=True,
                                   sensor_config=sensor_config, features=['velocity_base', 'lidar_2d'],
                                   spawn_by_map={'nav_empty': {'x':-4., 'y':-4., 'yaw':0.}},
                                   controllers=['joint_state_broadcaster', 'robot_lab_controller'])
                    profile['notes'] = ('Original ROBOTIS TurtleBot3 wheels and LDS sensor frame; lab physical '
                        'wheel control with 5 Hz lidar and 50 Hz IMU. Original RGB-camera geometry retained; '
                        'RGB/depth rendering and 3D SLAM are not enabled. Mapping/navigation modes require '
                        'recorded backend trials; OpenCR firmware, longer missions and materials remain pending.')
                    check.update(drive_urdf=str(controlled), drive_urdf_sha256=digest(controlled),
                                 drive_controllers_sha256=digest(controllers), control_source=control_source,
                                 source_sdf_sha256=digest(Path(control_source['source_sdf'])))
                    from robot_lab_utils.asset_support import apply_recorded_modes
                    support_path = ROOT/'docs/status/asset-runtime-support.yaml'
                    support = yaml.safe_load(support_path.read_text()) if support_path.is_file() else {}
                    profile = apply_recorded_modes(profile, source['revision'],
                        (support or {}).get('robots', {}).get(name), ROOT)
                    if profile.get('runtime_screens'):
                        profile['notes'] += ' Named localization/mapping/Nav2 screens are available in Robot status; other maps are experiments.'
                check.update(source_id=source['id'], repository=source['repository'], revision=source['revision'], entry=entry,
                             model_sha256=digest(model_path), derived_urdf_sha256=digest(derived),
                             runtime_mission_qualified=False)
                if 'halodi' in entry:
                    check['hand_dependency_manifest'] = str(store/'dependencies/qb_hand_description'/
                        load_source('qb_hand_description')['revision']/'manifest.json')
                if source['id'] in ('turtlebot3_vendor','husky_vendor','turtlebot4_vendor'):
                    check.update(source_description_sha256=digest(source_root/entry), xacro_arguments=xacro_args)
                if dependency_roots:
                    check['source_dependencies'] = [{'source_id': root.parent.name,
                        'revision': root.name, 'repository': load_source(root.parent.name)['repository'],
                        'license_sha256': digest(root/'LICENSE')} for root in dependency_roots]
                write_json(evidence, check)
                profiles[name] = profile
                entities.append(robot_entity(name, profile, source, entry, evidence))
                support_label = '; '.join(backend+': '+', '.join(modes) for backend, modes in
                    profile.get('supported_modes_by_simulator',
                        {backend: profile['supported_modes'] for backend in profile['supported_simulators']}).items())
                record.update(status='installed', check=str(evidence), profiles=[{'id': name, 'kind': 'robot',
                    'support': support_label, 'notes': profile['notes']}])
            except Exception as exc:
                record.update(status='needs integration repair', reason=str(exc))
            records.append(record)
            checkpoint_robots(store, profiles, entities, records)
            print(source['id'], entry, record['status'], record.get('reason', ''), flush=True)
    return profiles, entities, records


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--kind', choices=['robots', 'maps', 'all'], default='all')
    parser.add_argument('--source')
    parser.add_argument('--no-download', action='store_true')
    parser.add_argument('--store', type=Path, default=Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT', '/workspace/molar/robot_lab_runtime'))/'external_assets')
    args = parser.parse_args()
    store = args.store.resolve(); store.mkdir(parents=True, exist_ok=True)
    if store.stat().st_dev == Path('/').stat().st_dev:
        parser.error('Assets must remain on the mounted workspace SSD')
    installed = store/'installed'; installed.mkdir(exist_ok=True)
    (store/'locks').mkdir(exist_ok=True)
    report_path = installed/'integration-report.json'
    with (store/'locks'/'provision.lock').open('a') as handle:
        fcntl.flock(handle, fcntl.LOCK_EX)
        return provision(args, store, installed, report_path)


def provision(args, store, installed, report_path):
    for kind in ('robots', 'maps'):
        if args.kind not in (kind, 'all'):
            continue
        if kind == 'robots':
            profiles, entities, entries = install_robots(store, not args.no_download, args.source)
        else:
            from integrate_external_worlds import install_worlds
            profiles, entities, entries = {}, [], []
            for source_id in ('gazebo_world_dataset', 'gazebo_examples'):
                if args.source and args.source != source_id: continue
                imported, catalog, records = install_worlds(store, not args.no_download, source_id)
                profiles.update(imported); entities.extend(catalog); entries.extend(records)
        if args.source:
            old = yaml.safe_load((installed/(kind+'.yaml')).read_text()) if (installed/(kind+'.yaml')).is_file() else {kind: {}}
            # A failed new import must not remove previously installed assets.
            # Successful IDs replace their own record; omission is not removal.
            old[kind].update(profiles); profiles = old[kind]
            old_entities = yaml.safe_load((installed/('registry_'+kind+'.yaml')).read_text()) if (installed/('registry_'+kind+'.yaml')).is_file() else []
            replacements = {e['id']: e for e in entities}
            retained = {e['id']: e for e in old_entities}
            retained.update(replacements)
            entities = list(retained.values())
        write_yaml(installed/(kind+'.yaml'), {kind: profiles})
        write_yaml(installed/('registry_'+kind+'.yaml'), entities)
        report = json.loads(report_path.read_text()) if report_path.exists() else {'entries': []}
        sources = {e['source_id'] for e in entries}
        report['entries'] = [e for e in report['entries'] if e['source_id'] not in sources] + entries
        report.update(updated_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                      scope='Installed display/model and world/map assets; catalog size is not mission qualification')
        write_json(report_path, report)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
