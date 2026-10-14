"""Prepare source-preserving, namespaced robots for ONE Gazebo world.

ROS namespaces do not namespace TF frames or Gazebo sensor topics.  Keep the
source links/joints (per-model physics resources) intact and prefix *reported*
frames through robot_state_publisher, sensors and the feedback controller.
Generated descriptions/configuration live on SSD and never edit source assets.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
import os
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
import yaml


MEMBER_MODELS = ('bumperbot', 'labbot')
MAX_MEMBERS = 8
ID_PATTERN = re.compile(r'^[a-z][a-z0-9_]{0,31}$')


def finite(value, label):
    if isinstance(value, bool):
        raise ValueError(label + ' must be a finite number')
    try:
        number = float(value)
    except (ValueError, TypeError):
        raise ValueError(label + ' must be a finite number') from None
    if not math.isfinite(number):
        raise ValueError(label + ' must be a finite number')
    return number


def validate_session(data):
    """Small explicit contract; refuse aliases, unsupported plants and collisions."""
    if not isinstance(data, dict) or set(data) - {'schema_version', 'map', 'members'}:
        raise ValueError('Fleet session requires schema_version, map and members only')
    if type(data.get('schema_version')) is not int or data.get('schema_version') != 1:
        raise ValueError('Fleet session schema_version must be 1')
    members = data.get('members')
    if not isinstance(members, list) or not 1 <= len(members) <= MAX_MEMBERS:
        raise ValueError('A shared-world session supports 1–8 complete robots')
    seen, result = set(), []
    for item in members:
        if not isinstance(item, dict) or set(item) - {'id', 'model', 'spawn'}:
            raise ValueError('Each member requires id/model and optional spawn')
        name, model = item.get('id', ''), item.get('model', '')
        if not isinstance(name, str) or not ID_PATTERN.fullmatch(name) or name in seen:
            raise ValueError('Robot ids must be distinct lowercase ROS names')
        if name in {'fleet', 'map', 'world', 'clock', 'tf', 'tf_static'}:
            raise ValueError('Reserved fleet id: ' + name)
        if model not in MEMBER_MODELS:
            raise ValueError('Shared-world Gazebo control currently supports ' + ', '.join(MEMBER_MODELS))
        seen.add(name)
        pose = item.get('spawn')
        if pose is not None:
            if not isinstance(pose, dict) or not {'x', 'y'} <= set(pose) or set(pose) - {'x', 'y', 'z', 'yaw'}:
                raise ValueError('Explicit spawn requires x/y; z/yaw are optional')
            pose = {key: finite(value, 'spawn ' + key) for key, value in pose.items()}
        result.append(dict(id=name, model=model, spawn=pose))
    map_name = data.get('map', 'nav_empty')
    if not isinstance(map_name, str) or not map_name or len(map_name) > 128:
        raise ValueError('Select a registered map id')
    return dict(schema_version=1, map=map_name, members=result)


class SpawnGrid:
    """Conservative occupancy clearance, including unknown and exterior cells.

    Occupancy is a preflight hint, not proof that dynamic or 3D geometry is
    clear. All world collision geometry remains in the simulator.
    """
    def __init__(self, path):
        path = Path(path).resolve()
        meta = yaml.safe_load(path.read_text())
        self.resolution = finite(meta['resolution'], 'map resolution')
        if self.resolution <= 0:
            raise ValueError('Map resolution must be positive')
        self.origin = tuple(finite(v, 'map origin') for v in meta['origin'])
        if len(self.origin) != 3:
            raise ValueError('Map origin needs x, y and yaw')
        raster = np.asarray(Image.open(path.parent / meta['image']).convert('L'), dtype=float)
        probability = raster / 255.0 if int(meta.get('negate', 0)) else 1 - raster / 255.0
        self.free = np.flipud(probability < float(meta.get('free_thresh', .196)))
        from scipy.ndimage import distance_transform_edt
        # An all-free image still has an impassable boundary.
        self.clearance = distance_transform_edt(np.pad(self.free, 1))[1:-1, 1:-1] * self.resolution

    def cell(self, x, y):
        dx, dy = x - self.origin[0], y - self.origin[1]
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        return (int(math.floor((c * dx + s * dy) / self.resolution)),
                int(math.floor((-s * dx + c * dy) / self.resolution)))

    def clear(self, x, y, radius):
        col, row = self.cell(x, y)
        return (0 <= row < self.free.shape[0] and 0 <= col < self.free.shape[1]
                and self.clearance[row, col] > radius + math.sqrt(2) * self.resolution)


def choose_spawns(members, seed, grid=None):
    """Find nearby free centres without overlapping another member's footprint."""
    placed = []
    for member in members:
        radius = member['radius']
        pose = member.get('spawn')
        def available(x, y):
            return ((grid is None or grid.clear(x, y, radius + .10)) and all(
                math.hypot(x - other['spawn']['x'], y - other['spawn']['y'])
                > radius + other['radius'] + .30 for other in placed))
        if pose is None:
            if grid is None:
                raise ValueError('This world has no occupancy map: enter explicit x/y spawns for every member')
            candidates = [(float(seed['x']), float(seed['y']))]
            # Search locally; do not silently move robots to a different floor/wing.
            # Prefer a lateral row, leaving the reference robot's forward
            # corridor clear instead of placing its peer directly ahead.
            for distance in np.arange(.85, 3.01, .25):
                for angle in np.arange(math.pi / 2, 5 * math.pi / 2, math.pi / 12):
                    candidates.append((float(seed['x']) + distance * math.cos(angle),
                                       float(seed['y']) + distance * math.sin(angle)))
            centre = next(((x, y) for x, y in candidates if available(x, y)), None)
            if centre is None:
                raise ValueError('No nearby clear spawn for ' + member['id'] + '; enter explicit poses')
            pose = dict(x=centre[0], y=centre[1])
        elif not available(pose['x'], pose['y']):
            raise ValueError('Spawn is outside free occupancy or overlaps a member: ' + member['id'])
        pose = dict(x=float(pose['x']), y=float(pose['y']), z=float(pose.get('z', float(seed.get('z', .1)))),
                    yaw=float(pose.get('yaw', float(seed.get('yaw', 0)))))
        placed.append(dict(member, spawn=pose))
    return placed


def namespace_description(urdf, member, controller_file):
    """Instance identity plus the documented first-party Labbot lidar clearance."""
    root = ET.fromstring(urdf)
    name, prefix = member['id'], member['id'] + '/'
    root.set('name', name)
    if member['model'] == 'labbot':
        # The first-party source camera visual spans z=.195–.225 m, while
        # its source lidar plane is z=.20 m. Gazebo's GPU scan sees the
        # robot's own camera at ~.12 m. Do not invent free ranges or ignore
        # close obstacles: physically raise the lidar 5 cm in THIS fleet
        # derivative and publish its actual new transform. Single-robot
        # descriptions and their historical certificates stay unchanged.
        origin = root.find("joint[@name='laser_joint']/origin")
        if origin is None:
            raise ValueError('Labbot source laser mount is missing')
        coordinates = [float(v) for v in origin.get('xyz', '').split()]
        if len(coordinates) != 3:
            raise ValueError('Invalid source laser mount')
        coordinates[2] += .05
        origin.set('xyz', ' '.join(format(v, '.12g') for v in coordinates))
    found = 0
    # Sensors and IMU systems are installed once on the world, not per model.
    for gazebo in root.findall('gazebo'):
        for plugin in list(gazebo.findall('plugin')):
            kind = plugin.get('name', '')
            if kind.endswith(('::Sensors', '::Imu')):
                gazebo.remove(plugin)
            elif 'ROS2ControlPlugin' in kind:
                found += 1
                for tag, text in (('parameters', str(controller_file)),
                                  ('robot_param', 'robot_description'),
                                  # Humble's plugin prepends <ros><namespace>
                                  # to this node itself; a full path becomes
                                  # /alpha//alpha/... and aborts the server.
                                  ('robot_param_node', 'robot_state_publisher')):
                    element = plugin.find(tag)
                    if element is None:
                        element = ET.SubElement(plugin, tag)
                    element.text = text
                ros = plugin.find('ros')
                if ros is None:
                    ros = ET.SubElement(plugin, 'ros')
                namespace = ros.find('namespace')
                if namespace is None:
                    namespace = ET.SubElement(ros, 'namespace')
                namespace.text = '/' + name
            elif kind.endswith('::OdometryPublisher'):
                for tag, text in (('odom_frame', 'map'), ('robot_base_frame', prefix + 'base_footprint'),
                                  ('odom_topic', '/model/' + name + '/odometry_truth'),
                                  ('tf_topic', '/model/' + name + '/truth_pose')):
                    element = plugin.find(tag)
                    if element is None:
                        element = ET.SubElement(plugin, tag)
                    element.text = text
        for sensor in gazebo.findall('sensor'):
            frame = sensor.find('gz_frame_id')
            if frame is not None and frame.text:
                frame.text = prefix + frame.text.lstrip('/')
            topic = sensor.find('topic')
            if topic is None:
                topic = ET.SubElement(sensor, 'topic')
            kind = sensor.get('type')
            suffix = {'imu': 'imu', 'gpu_lidar': 'scan', 'rgbd_camera': 'oakd'}.get(kind)
            if suffix is None:
                raise ValueError('No shared-world sensor contract for ' + str(kind))
            topic.text = '/fleet/' + name + '/' + suffix
    if found != 1:
        raise ValueError('Each fleet model must have exactly one source ros2_control plugin')
    return ET.tostring(root, encoding='unicode')


def controller_parameters(source, member):
    """Keep source geometry/gains; wheel feedback owns odom, never Gazebo truth."""
    data = copy.deepcopy(source)
    data.pop('simple_velocity_controller', None)
    data['controller_manager']['ros__parameters'].pop('simple_velocity_controller', None)
    params = data['robot_lab_controller']['ros__parameters']
    params.update(use_stamped_vel=False, enable_odom_tf=True, tf_frame_prefix_enable=False,
                  odom_frame_id=member['id'] + '/odom', base_frame_id=member['id'] + '/base_footprint',
                  cmd_vel_timeout=.4, open_loop=False)
    params['linear']['x'].update(max_velocity=member['max_linear'], min_velocity=-member['max_linear'])
    params['angular']['z'].update(max_velocity=member['max_angular'], min_velocity=-member['max_angular'])
    # Full node paths avoid a repeated name picking up another member's parameters.
    return {'/' + member['id'] + '/' + key: value for key, value in data.items()}


def navigation_parameters(config_dir, localization_dir, member, map_yaml):
    """Reuse the installed Nav2/AMCL configuration with explicit frames/topics."""
    data = {}
    for file in ('controller_server.yaml', 'planner_server.yaml', 'smoother_server.yaml',
                 'bt_navigator.yaml', 'behavior_server.yaml'):
        data.update(yaml.safe_load((Path(config_dir) / file).read_text()))
    def merge(target, overlay):
        for key, value in overlay.items():
            if isinstance(value, dict) and isinstance(target.get(key), dict):
                merge(target[key], value)
            else:
                target[key] = copy.deepcopy(value)
    merge(data, yaml.safe_load((Path(config_dir) / 'robots' / (member['model'] + '.yaml')).read_text()))
    data['amcl'] = yaml.safe_load((Path(localization_dir) / 'amcl.yaml').read_text())['amcl']
    replacements = {'base_footprint': member['id'] + '/base_footprint',
                    'base_link': member['id'] + '/base_link', 'odom': member['id'] + '/odom',
                    '/scan': '/' + member['id'] + '/scan',
                    '/odometry/filtered': '/' + member['id'] + '/robot_lab_controller/odom'}
    def rewrite(value):
        if isinstance(value, dict):
            return {key: rewrite(item) for key, item in value.items()}
        if isinstance(value, list):
            return [rewrite(item) for item in value]
        return replacements.get(value, value) if isinstance(value, str) else value
    data = rewrite(data)
    amcl = data['amcl']['ros__parameters']
    amcl.update(scan_topic='/' + member['id'] + '/scan',
                initial_pose={axis: member['spawn'].get(axis, 0.) for axis in ('x', 'y', 'z', 'yaw')})
    amcl['initial_pose']['z'] = 0.
    follow = data['controller_server']['ros__parameters']['FollowPath']
    follow['desired_linear_vel'] = min(.25, member['max_linear'])
    follow['rotate_to_heading_angular_vel'] = min(.5, member['max_angular'])
    follow['regulated_linear_scaling_min_speed'] = min(.1, member['max_linear'])
    data['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = str(
        Path(config_dir).parent / 'behavior_tree' / 'simple_navigation_w_replanning_and_recovery.xml')
    for name in ('local_costmap', 'global_costmap'):
        params = data[name][name]['ros__parameters']
        params['robot_radius'] = member['radius']
        params['obstacle_layer']['scan']['topic'] = '/' + member['id'] + '/scan'
    # Nav2 creates nested costmap nodes under the member namespace.
    return {'/' + member['id']: data}


def bridge_entries(members):
    entries = [dict(ros_topic_name='/clock', gz_topic_name='/clock',
                    ros_type_name='rosgraph_msgs/msg/Clock', gz_type_name='gz.msgs.Clock', direction='GZ_TO_ROS')]
    for member in members:
        name = member['id']
        mappings = [('/odom/ground_truth', '/model/' + name + '/odometry_truth', 'nav_msgs/msg/Odometry', 'gz.msgs.Odometry'),
                    ('/scan', '/fleet/' + name + '/scan', 'sensor_msgs/msg/LaserScan', 'gz.msgs.LaserScan'),
                    ('/imu/out', '/fleet/' + name + '/imu', 'sensor_msgs/msg/Imu', 'gz.msgs.IMU')]
        for suffix, gz_suffix, ros_type, gz_type in (
                ('/oakd/rgb/image_raw', '/image', 'sensor_msgs/msg/Image', 'gz.msgs.Image'),
                ('/oakd/rgb/camera_info', '/camera_info', 'sensor_msgs/msg/CameraInfo', 'gz.msgs.CameraInfo'),
                ('/oakd/depth/image_raw', '/depth_image', 'sensor_msgs/msg/Image', 'gz.msgs.Image'),
                ('/oakd/points_gz', '/points', 'sensor_msgs/msg/PointCloud2', 'gz.msgs.PointCloudPacked')):
            mappings.append((suffix, '/fleet/' + name + '/oakd' + gz_suffix, ros_type, gz_type))
        entries += [dict(ros_topic_name='/' + name + suffix, gz_topic_name=gz_topic,
                         ros_type_name=ros_type, gz_type_name=gz_type, direction='GZ_TO_ROS')
                    for suffix, gz_topic, ros_type, gz_type in mappings]
    return entries


def prepare_world(source, folder):
    """One sensor/IMU system and complete source collision geometry in a derivative."""
    source = Path(source).resolve()
    root = ET.fromstring(source.read_bytes())
    worlds = root.findall('world')
    if len(worlds) != 1:
        raise ValueError('Shared-world sessions require exactly one SDF world')
    world = worlds[0]
    for system in ('Physics', 'UserCommands', 'SceneBroadcaster', 'Sensors', 'Imu'):
        if not any(p.get('name', '').endswith('::' + system) for p in world.findall('plugin')):
            filename = {'SceneBroadcaster': 'scene-broadcaster', 'UserCommands': 'user-commands'}.get(system, system.lower())
            # This ground-robot ROS Humble installation uses Gazebo Fortress 6
            # (ros_gz_sim forces version 6). PX4's separate Harmonic 8 runtime
            # must not leak ABI-incompatible plugins into this server.
            plugin = ET.SubElement(world, 'plugin', filename='ignition-gazebo-' + filename + '-system',
                                   name='ignition::gazebo::systems::' + system)
            if system == 'Sensors':
                ET.SubElement(plugin, 'render_engine').text = 'ogre2'
    # Every source surface/plugin remains; paths are resolved for the SSD derivative.
    for element in world.iter():
        if element.tag in ('uri', 'image', 'diffuse', 'normal', 'albedo_map', 'normal_map'):
            value = (element.text or '').strip()
            if value and '://' not in value and not Path(value).is_absolute() and (source.parent / value).exists():
                element.text = (source.parent / value).resolve().as_uri()
    target = folder / 'world.sdf'
    target.write_bytes(ET.tostring(root, encoding='utf-8', xml_declaration=True))
    return str(target), world.get('name')


def prepare_session(session_file, map_override='', navigation=False, runtime_root=None):
    from ament_index_python.packages import get_package_share_directory as share
    from .robot_description import load_description
    from .gazebo_physics_world import cap_physics_step
    requested = validate_session(yaml.safe_load(Path(session_file).read_text()))
    map_name = map_override or requested['map']
    bringup, robots = Path(share('robot_lab_bringup')), Path(share('robot_lab_robots'))
    from .installed_assets import merge_installed_profiles
    maps = merge_installed_profiles(yaml.safe_load((bringup / 'config/sim_maps.yaml').read_text()), 'maps')['maps']
    if map_name not in maps:
        raise ValueError('Unknown installed map: ' + map_name)
    map_profile = maps[map_name]
    gz = map_profile['gazebo']
    def resolve(package, path):
        return str(Path(path) if Path(path).is_absolute() else Path(share(package)) / path)
    world_source = resolve(gz.get('world_package', 'robot_lab_maps'), gz['world_path'])
    map_cfg = map_profile.get('map', {})
    map_yaml = resolve(map_cfg.get('package', 'robot_lab_maps'), map_cfg['path']) if map_cfg.get('has_2d_map') else ''
    if navigation and not map_yaml:
        raise ValueError('Fleet Nav2 requires the selected world\'s occupancy map')
    profiles = yaml.safe_load((robots / 'config/robots.yaml').read_text())['robots']
    members = []
    for item in requested['members']:
        profile = profiles[item['model']]
        members.append(dict(item, radius=.118 if item['model'] == 'bumperbot' else .18,
                            max_linear=min(.35, profile.get('drive', {}).get('max_speed', .35)),
                            max_angular=min(.8, profile.get('drive', {}).get('max_angular_speed', .8))))
    seed = dict(x=0., y=0., z=.1, yaw=0.)
    seed.update(map_profile.get('spawn', {}))
    members = choose_spawns(members, seed, SpawnGrid(map_yaml) if map_yaml else None)
    base = Path(runtime_root or os.environ.get('ROBOT_LAB_RUNTIME_ROOT', '/workspace/molar/robot_lab_runtime')) / 'fleet_sessions'
    base.mkdir(parents=True, exist_ok=True)
    import tempfile
    folder = Path(tempfile.mkdtemp(prefix='session-', dir=base))
    # Dynamics are bounded at 2 ms while preserving the user's selected world.
    bounded_world = cap_physics_step(world_source, .002, runtime_root=base)
    world, world_name = prepare_world(bounded_world, folder)
    for member in members:
        name, profile = member['id'], profiles[member['model']]
        model_path = resolve(profile['package'], profile['xacro'])
        urdf = load_description(model_path, 'is_ignition:=true', 'is_sim:=true')
        if member['model'] == 'bumperbot':
            ctrl_source = Path(share('robot_lab_controller')) / 'config/robot_lab_controllers.yaml'
        else:
            ctrl_source = robots / 'labbot/config/robot_lab_controllers.yaml'
        controller_file = folder / (name + '-controllers.yaml')
        controller_file.write_text(yaml.safe_dump(controller_parameters(yaml.safe_load(ctrl_source.read_text()), member)))
        description = namespace_description(urdf, member, controller_file)
        path = folder / (name + '.urdf')
        path.write_text(description)
        member.update(description=str(path), description_sha256=hashlib.sha256(description.encode()).hexdigest(),
                      source_description_sha256=hashlib.sha256(urdf.encode()).hexdigest(),
                      source_model=model_path, source_controller=str(ctrl_source), controller_file=str(controller_file))
        member['physical_adjustments'] = ([dict(joint='laser_joint', xyz_delta=[0., 0., .05],
            reason='Move the actual lidar plane above the source OAK-D camera; no fabricated scan rays')]
            if member['model'] == 'labbot' else [])
        if navigation:
            nav_file = folder / (name + '-navigation.yaml')
            nav_file.write_text(yaml.safe_dump(navigation_parameters(
                Path(share('robot_lab_navigation')) / 'config', Path(share('robot_lab_localization')) / 'config', member, map_yaml)))
            member['navigation_file'] = str(nav_file)
    bridge_file = folder / 'bridge.yaml'
    bridge_file.write_text(yaml.safe_dump(bridge_entries(members)))
    result = dict(schema_version=1, map=map_name, backend='gazebo', navigation=navigation,
                  world=world, world_name=world_name, world_source=world_source,
                  world_source_sha256=hashlib.sha256(Path(world_source).read_bytes()).hexdigest(),
                  world_sha256=hashlib.sha256(Path(world).read_bytes()).hexdigest(),
                  map_yaml=map_yaml, bridge_file=str(bridge_file), members=members,
                  session_source=str(Path(session_file).resolve()), directory=str(folder),
                  scope='Shared physical world; only Gazebo Bumperbot/Labbot adapters implemented')
    resolved_file = folder / 'session.json'
    result['resolved_file'] = str(resolved_file)
    result['session_source_sha256'] = hashlib.sha256(Path(session_file).read_bytes()).hexdigest()
    (folder / 'requested-session.yaml').write_bytes(Path(session_file).read_bytes())
    resolved_file.write_text(json.dumps(result, indent=2) + '\n')
    return result
