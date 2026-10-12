"""Run upstream PX4/Harmonic X500 and its ROS adapter on the workspace SSD."""
import argparse
import fcntl
from collections import deque
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid
import xml.etree.ElementTree as ET

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from .px4_ros_controller import PX4Controller


def prepare_world(source, destination, upstream):
    root = ET.parse(source if source else upstream/'worlds/default.sdf').getroot()
    world = root.find('world')
    if world is None:
        raise ValueError('The selected map does not contain an SDF world.')
    world.set('name', 'robot_lab_flight')
    for plugin in world.iter('plugin'):
        plugin.set('filename', plugin.get('filename', '').replace('ignition-gazebo-', 'gz-sim-'))
        plugin.set('name', plugin.get('name', '').replace('ignition::gazebo::', 'gz::sim::'))
    for tag in ('spherical_coordinates', 'magnetic_field', 'gravity', 'atmosphere'):
        if world.find(tag) is None:
            node = ET.parse(upstream/'worlds/default.sdf').getroot().find('world/'+tag)
            if node is not None:
                world.append(copy.deepcopy(node))
    for uri in world.iter('uri'):
        value = (uri.text or '').strip()
        if value.startswith('package://'):
            from ament_index_python.packages import get_package_share_directory
            package, tail = value[10:].split('/', 1)
            uri.text = str(Path(get_package_share_directory(package))/tail)
        elif value and not value.startswith(('model://', 'file://', 'http://', 'https://')):
            uri.text = str((Path(source).parent/value).resolve()) if source else value
    root.set('version', '1.9')
    ET.ElementTree(root).write(destination, encoding='unicode', xml_declaration=True)


def prepare_model(destination, upstream):
    root = ET.parse(upstream/'models/x500/model.sdf').getroot()
    model = root.find('model')
    truth = ET.SubElement(model, 'plugin', filename='gz-sim-odometry-publisher-system',
                          name='gz::sim::systems::OdometryPublisher')
    for tag, value in [('odom_topic', '/px4/odometry_truth_native'), ('dimensions', '3'),
                       ('odom_frame', 'map'), ('robot_base_frame', 'base_link'),
                       ('xyz_offset', '0 0 0.24'),
                       ('odom_publish_frequency', '20'), ('tf_topic', '/px4/truth_tf_native')]:
        ET.SubElement(truth, tag).text = value
    joints = ET.SubElement(model, 'plugin', filename='gz-sim-joint-state-publisher-system',
                           name='gz::sim::systems::JointStatePublisher')
    ET.SubElement(joints, 'topic').text = '/px4/joint_states_native'
    destination.mkdir(parents=True)
    ET.ElementTree(root).write(destination/'model.sdf', encoding='unicode', xml_declaration=True)


# RGB-D variant (upstream x500_depth airframe 4002 with its OakD-Lite).  The
# camera link sits 0.12/0.03/0.002 m from base_link: the merged include
# places it 0.242 m above the model frame, base_link 0.24 m (gz sdf -p).
CAMERA_GZ_TOPIC = '/robot_lab/x500/rgbd'
CAMERA_ROS_PREFIX = '/x500/camera'
CAMERA_OPTICAL_FRAME = 'camera_optical_frame'
CAMERA_MOUNT = (0.12, 0.03, 0.002)
CAMERA_SENSOR = (0.01233, -0.03, 0.01878)   # sensor pose in camera_link (upstream)
CAMERA_COLLISION = ((0.00358, -0.03, 0.014), (0.0175, 0.091, 0.028))

# Harmonic (gz-transport13) and the Fortress ground-robot stack
# (ign-transport11) share the default discovery ports.  Fortress cannot parse
# Harmonic's discovery messages: with one flight running, every Fortress
# process logged "Unknown message type" ~6,500 times per second (2026-10-12,
# ~1M launch-log lines per Gazebo mission).  The partition does not help (it
# is checked after parsing), so the flight gets its own ports; Fortress reads
# only IGN_DISCOVERY_*, Harmonic only GZ_DISCOVERY_*.
HARMONIC_DISCOVERY_PORTS = {'GZ_DISCOVERY_MSG_PORT': '10417', 'GZ_DISCOVERY_SRV_PORT': '10418'}


def _copy_config(source, destination, name):
    config = ET.parse(source).getroot()
    if config.find('name') is not None:
        config.find('name').text = name
    ET.ElementTree(config).write(destination, encoding='unicode', xml_declaration=True)


def prepare_depth_model(models, upstream, width=320, height=240, rate=5.0):
    """x500_depth with one registered RGB-D sensor in place of upstream's two.

    Upstream pairs a 1920x1080 30 Hz colour camera with a 640x480 30 Hz
    depth camera of a different field of view.  The lab model keeps the
    OakD-Lite link, mass and collision and renders a single gz rgbd_camera
    (colour and depth share intrinsics, so the images are registered) at the
    5 Hz the lab's other RGB-D robots use, with the depth camera's 1.274 rad
    field of view and 0.2-19.1 m range.
    """
    root = ET.parse(upstream/'models/OakD-Lite/model.sdf').getroot()
    model = root.find('model')
    model.set('name', 'robot_lab_oakd_lite')
    link = model.find('link')
    for sensor in link.findall('sensor'):
        link.remove(sensor)
    sensor = ET.SubElement(link, 'sensor', name='rgbd', type='rgbd_camera')
    ET.SubElement(sensor, 'pose').text = ' '.join(map(str, CAMERA_SENSOR))+' 0 0 0'
    ET.SubElement(sensor, 'topic').text = CAMERA_GZ_TOPIC
    ET.SubElement(sensor, 'update_rate').text = str(rate)
    ET.SubElement(sensor, 'always_on').text = '1'
    camera = ET.SubElement(sensor, 'camera')
    ET.SubElement(camera, 'horizontal_fov').text = '1.274'
    image = ET.SubElement(camera, 'image')
    ET.SubElement(image, 'width').text = str(int(width))
    ET.SubElement(image, 'height').text = str(int(height))
    ET.SubElement(image, 'format').text = 'R8G8B8'
    clip = ET.SubElement(camera, 'clip')
    ET.SubElement(clip, 'near').text = '0.2'
    ET.SubElement(clip, 'far').text = '19.1'
    depth_clip = ET.SubElement(ET.SubElement(camera, 'depth_camera'), 'clip')
    ET.SubElement(depth_clip, 'near').text = '0.2'
    ET.SubElement(depth_clip, 'far').text = '19.1'
    camera_dir = models/'robot_lab_oakd_lite'
    camera_dir.mkdir(parents=True)
    ET.ElementTree(root).write(camera_dir/'model.sdf', encoding='unicode', xml_declaration=True)
    _copy_config(upstream/'models/OakD-Lite/model.config', camera_dir/'model.config', 'robot_lab_oakd_lite')

    mount = ' '.join(map(str, (CAMERA_MOUNT[0], CAMERA_MOUNT[1], CAMERA_MOUNT[2]+0.24)))+' 0 0 0'
    sdf = ET.Element('sdf', version='1.9')
    depth = ET.SubElement(sdf, 'model', name='x500_depth')
    ET.SubElement(ET.SubElement(depth, 'include', merge='true'), 'uri').text = 'model://x500'
    include = ET.SubElement(depth, 'include', merge='true')
    ET.SubElement(include, 'uri').text = 'model://robot_lab_oakd_lite'
    ET.SubElement(include, 'pose').text = mount
    joint = ET.SubElement(depth, 'joint', name='CameraJoint', type='fixed')
    ET.SubElement(joint, 'parent').text = 'base_link'
    ET.SubElement(joint, 'child').text = 'camera_link'
    ET.SubElement(joint, 'pose', relative_to='base_link').text = ' '.join(map(str, CAMERA_MOUNT))+' 0 0 0'
    depth_dir = models/'x500_depth'
    depth_dir.mkdir(parents=True)
    ET.ElementTree(sdf).write(depth_dir/'model.sdf', encoding='unicode', xml_declaration=True)
    _copy_config(upstream/'models/x500_depth/model.config', depth_dir/'model.config', 'x500_depth')
    return camera_dir/'model.sdf'


def camera_collision_corners():
    """OakD-Lite collision-box corners in base_link (metres)."""
    import itertools
    (cx, cy, cz), size = CAMERA_COLLISION
    return [[CAMERA_MOUNT[0]+cx+sx*size[0]/2, CAMERA_MOUNT[1]+cy+sy*size[1]/2,
             CAMERA_MOUNT[2]+cz+sz*size[2]/2]
            for sx, sy, sz in itertools.product((-1, 1), repeat=3)]


def x500_urdf(upstream, camera=False):
    """RViz description from the actual upstream X500 link/joint visuals.

    Physics continues to use the upstream SDF. This converter deliberately
    accepts its five-link base/rotor tree, and rejects unsupported geometry.
    """
    model = ET.parse(upstream/'models/x500_base/model.sdf').getroot().find('model')
    robot = ET.Element('robot', name='px4_x500')
    poses = {}
    for link in model.findall('link'):
        poses[link.get('name')] = link.findtext('pose', '0 0 0 0 0 0').split()
        out = ET.SubElement(robot, 'link', name=link.get('name'))
        for visual in link.findall('visual'):
            geom = visual.find('geometry')
            if geom is None or geom.find('plane') is not None:
                continue  # tiny textured decals do not affect the body shape
            v = ET.SubElement(out, 'visual', name=visual.get('name'))
            pose = visual.findtext('pose', '0 0 0 0 0 0').split()
            ET.SubElement(v, 'origin', xyz=' '.join(pose[:3]), rpy=' '.join(pose[3:]))
            g = ET.SubElement(v, 'geometry')
            if geom.find('mesh') is not None:
                mesh = geom.find('mesh')
                uri = mesh.findtext('uri')
                if not uri.startswith('model://'):
                    raise ValueError('Unsupported X500 mesh '+uri)
                ET.SubElement(g, 'mesh', filename=(upstream/'models'/uri[8:]).as_uri(),
                              scale=mesh.findtext('scale', '1 1 1'))
            elif geom.find('box') is not None:
                ET.SubElement(g, 'box', size=geom.findtext('box/size'))
            elif geom.find('cylinder') is not None:
                ET.SubElement(g, 'cylinder', radius=geom.findtext('cylinder/radius'), length=geom.findtext('cylinder/length'))
            elif geom.find('sphere') is not None:
                ET.SubElement(g, 'sphere', radius=geom.findtext('sphere/radius'))
            else:
                raise ValueError('Unsupported X500 visual geometry')
    for joint in model.findall('joint'):
        parent, child = joint.findtext('parent'), joint.findtext('child')
        if parent != 'base_link' or joint.find('pose') is not None:
            raise ValueError('Unsupported X500 joint frame')
        j = ET.SubElement(robot, 'joint', name=joint.get('name'), type='continuous')
        ET.SubElement(j, 'parent', link=parent)
        ET.SubElement(j, 'child', link=child)
        pose = poses[child]
        ET.SubElement(j, 'origin', xyz=' '.join(pose[:3]), rpy=' '.join(pose[3:]))
        ET.SubElement(j, 'axis', xyz=joint.findtext('axis/xyz', '0 0 1'))
    if camera:
        # OakD-Lite body and the ROS optical frame (z forward, x right).
        link = ET.SubElement(robot, 'link', name='camera_link')
        visual = ET.SubElement(link, 'visual', name='oakd_lite')
        (cx, cy, cz), size = CAMERA_COLLISION
        ET.SubElement(visual, 'origin', xyz='%g %g %g' % (cx, cy, cz), rpy='0 0 0')
        ET.SubElement(ET.SubElement(visual, 'geometry'), 'box', size='%g %g %g' % size)
        mount = ET.SubElement(robot, 'joint', name='camera_joint', type='fixed')
        ET.SubElement(mount, 'parent', link='base_link')
        ET.SubElement(mount, 'child', link='camera_link')
        ET.SubElement(mount, 'origin', xyz='%g %g %g' % CAMERA_MOUNT, rpy='0 0 0')
        ET.SubElement(robot, 'link', name=CAMERA_OPTICAL_FRAME)
        optical = ET.SubElement(robot, 'joint', name='camera_optical_joint', type='fixed')
        ET.SubElement(optical, 'parent', link='camera_link')
        ET.SubElement(optical, 'child', link=CAMERA_OPTICAL_FRAME)
        ET.SubElement(optical, 'origin', xyz='%g %g %g' % CAMERA_SENSOR,
                      rpy='%r 0 %r' % (-math.pi/2, -math.pi/2))
    return ET.tostring(robot, encoding='unicode')


def rtabmap_command(database, localization, parameter_file, ros_prefix=CAMERA_ROS_PREFIX):
    """RTAB-Map 3D mapping on the FCU odometry (TF map -> base_link).

    The FCU estimate is the odometry frame ('map', the frame the Drone
    controls and RViz already use); RTAB-Map publishes its loop-closure
    correction as aerial_map -> map, so nothing else in the flight graph
    changes.  Mapping starts a new database; localization reuses a saved
    one read-only.  RTAB-Map's own 'Group/Name' parameters cannot be given
    as -p overrides, so all parameters go through *parameter_file*.
    """
    import yaml
    parameters = {
        'frame_id': 'base_link', 'odom_frame_id': 'map', 'map_frame_id': 'aerial_map',
        'subscribe_depth': True, 'subscribe_rgb': True, 'approx_sync': True,
        'approx_sync_max_interval': 0.1, 'sync_queue_size': 10, 'use_sim_time': False,
        'publish_tf': True, 'wait_for_transform': 0.3, 'database_path': str(database),
        'Reg/Force3DoF': 'false', 'Grid/3D': 'true', 'Grid/RayTracing': 'true',
        'Grid/RangeMax': '8.0', 'Grid/CellSize': '0.1', 'Rtabmap/DetectionRate': '1',
        'RGBD/LinearUpdate': '0.1', 'RGBD/AngularUpdate': '0.1',
        'Mem/IncrementalMemory': 'false' if localization else 'true',
        'Mem/InitWMWithAllNodes': 'true' if localization else 'false',
    }
    Path(parameter_file).write_text(yaml.safe_dump({'/**': {'ros__parameters': parameters}}))
    command = ['ros2', 'run', 'rtabmap_slam', 'rtabmap']
    if not localization:
        command.append('--delete_db_on_start')
    return command+['--ros-args', '-r', '__ns:=/rtabmap',
                    '-r', 'rgb/image:='+ros_prefix+'/rgb/image_raw',
                    '-r', 'depth/image:='+ros_prefix+'/depth/image_raw',
                    '-r', 'rgb/camera_info:='+ros_prefix+'/rgb/camera_info',
                    '--params-file', str(parameter_file)]


def rtabmap_environment(environment):
    """Prefer the RTAB-Map core shipped with the ROS packages.

    A separate /usr/local RTAB-Map build on this host links a RealSense
    library that is not installed, so the ROS executables fail to start
    unless their own core comes first (as robot_lab_mapping's launch does).
    """
    import sysconfig
    from ament_index_python.packages import get_package_prefix
    prefix = Path(get_package_prefix('rtabmap_slam'))
    multiarch = sysconfig.get_config_var('MULTIARCH')
    directories = ([prefix/'lib'/multiarch] if multiarch else [])+[prefix/'lib']
    library_path = os.pathsep.join(str(d) for d in directories if d.is_dir())
    inherited = environment.get('LD_LIBRARY_PATH', '')
    return dict(environment, LD_LIBRARY_PATH=library_path+(os.pathsep+inherited if inherited else ''))


def export_cloud(database, output_dir):
    """Write the saved map's assembled cloud (PLY) next to the database."""
    if not Path(database).is_file():
        return None
    from ament_index_python.packages import get_package_prefix
    tool = Path(get_package_prefix('rtabmap_slam'))/'bin/rtabmap-export'
    result = subprocess.run([str(tool) if tool.is_file() else 'rtabmap-export', '--cloud', '--voxel', '0.05',
                             '--output_dir', str(output_dir), '--output', 'aerial_map',
                             str(database)], capture_output=True, text=True, timeout=600,
                            env=rtabmap_environment(dict(os.environ)))
    clouds = sorted(Path(output_dir).glob('aerial_map*.ply'))
    return str(clouds[-1]) if result.returncode == 0 and clouds else None


class NativeStateRelay(Node):
    """Relay independent Gazebo pose and measured rotor states to ROS 2.

    Harmonic transport is read directly; the Humble/Fortress bridge is not
    reused across incompatible transport versions. Stamp in ROS wall time.
    """
    def __init__(self, environment):
        super().__init__('px4_gazebo_state')
        self.truth_pub = self.create_publisher(Odometry, '/px4/odometry_truth', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.readers, self.queues = [], []
        for topic in ('/px4/odometry_truth_native', '/px4/joint_states_native'):
            queue = deque(maxlen=1)
            process = subprocess.Popen(['gz', 'topic', '-e', '-t', topic, '--json-output'],
                env=environment, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
            self.readers.append(process)
            self.queues.append(queue)
            def read(proc=process, output=queue):
                for line in proc.stdout:
                    try:
                        output.append(json.loads(line))
                    except json.JSONDecodeError:
                        pass
            threading.Thread(target=read, daemon=True).start()
        self.create_timer(0.05, self.publish)

    def publish(self):
        if self.queues[0]:
            raw = self.queues[0].pop()
            p = raw.get('pose', {})
            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id, msg.child_frame_id = 'map', 'base_link'
            for key in 'xyz':
                setattr(msg.pose.pose.position, key, float(p.get('position', {}).get(key, 0.0)))
            for key in 'xyzw':
                setattr(msg.pose.pose.orientation, key, float(p.get('orientation', {}).get(key, 0.0)))
            self.truth_pub.publish(msg)
        if self.queues[1]:
            raw = self.queues[1].pop()
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            for joint in raw.get('joint', []):
                msg.name.append(joint['name'])
                msg.position.append(float(joint.get('axis1', {}).get('position', 0.0)))
                msg.velocity.append(float(joint.get('axis1', {}).get('velocity', 0.0)))
            if msg.name:
                self.joint_pub.publish(msg)

    def close(self):
        for proc in self.readers:
            proc.terminate()
        for proc in self.readers:
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                proc.kill()


def _interrupt(signum, frame):
    raise KeyboardInterrupt


def stop_group(process):
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            break
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            continue
        # The parent can exit before its Gazebo children. Stop their group.
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        time.sleep(0.2)
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        break


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--px4-root', default=os.environ.get('PX4_ROOT', '/workspace/molar/px4/PX4-Autopilot'))
    parser.add_argument('--world', default='')
    parser.add_argument('--gui', default='false', choices=('true', 'false', 'auto'))
    parser.add_argument('--flight-enabled', default='true', choices=('true', 'false'))
    parser.add_argument('--spawn-x', type=float, default=0.0)
    parser.add_argument('--spawn-y', type=float, default=0.0)
    parser.add_argument('--spawn-z', type=float, default=0.0)
    parser.add_argument('--rviz', default='false', choices=('true', 'false'))
    parser.add_argument('--log-limit-mb', type=float, default=256.)
    parser.add_argument('--vehicle', default='x500', choices=('x500', 'x500_depth'),
                        help='x500_depth adds the upstream OakD-Lite RGB-D camera')
    parser.add_argument('--mapping', default='none', choices=('none', 'rtabmap'),
                        help='RTAB-Map 3D mapping (needs --vehicle x500_depth)')
    parser.add_argument('--map-database', default='',
                        help='Saved RTAB-Map database to localize in (read-only); '
                             'empty starts a new map in the run directory')
    args = parser.parse_args()
    if not math.isfinite(args.log_limit_mb) or args.log_limit_mb <= 0:
        parser.error('PX4 log limit must be positive and finite')
    if args.mapping != 'none' and args.vehicle != 'x500_depth':
        parser.error('Aerial mapping needs the RGB-D vehicle (--vehicle x500_depth)')
    if args.map_database and (args.mapping == 'none' or not Path(args.map_database).is_file()):
        parser.error('--map-database needs --mapping rtabmap and an existing database file')
    root = Path(args.px4_root).resolve()
    build = root/'build/px4_sitl_default'
    upstream = root/'Tools/simulation/gz'
    runtime = Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT', '/workspace/molar/robot_lab_runtime'))
    output = os.environ.get('ROBOT_LAB_EXPERIMENT_OUTPUT')
    run = (Path(output)/'px4_native' if output else runtime/'px4'/
        ('ros-flight-'+time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:6]))
    if not (build/'bin/px4').is_file():
        raise RuntimeError('PX4 SITL is not built at '+str(build)+'; see docs/tutorials/px4_x500.md')
    run.mkdir(parents=True)
    if run.stat().st_dev != Path('/workspace').stat().st_dev:
        raise RuntimeError('PX4 runtime must be on the mounted workspace SSD.')
    (runtime/'px4').mkdir(parents=True, exist_ok=True)
    if runtime.stat().st_dev != Path('/workspace').stat().st_dev:
        raise RuntimeError('PX4 instance lease must stay on the mounted workspace SSD.')
    instance_lease = (runtime/'px4/instance-0.lock').open('a')
    try:
        fcntl.flock(instance_lease, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        raise RuntimeError('Another Robot Lab PX4 instance owns MAVLink port 14580; stop it before this launch.') from exc
    # ros2 launch escalates to SIGTERM; run the same landing and process
    # cleanup as for SIGINT instead of dying with PX4 and Gazebo still up.
    signal.signal(signal.SIGTERM, _interrupt)
    for folder in ('rootfs', 'worlds', 'models'):
        (run/folder).mkdir()
    prepare_world(args.world, run/'worlds/robot_lab_flight.sdf', upstream)
    prepare_model(run/'models/x500', upstream)
    _copy_config(upstream/'models/x500/model.config', run/'models/x500/model.config', 'x500')
    camera = args.vehicle == 'x500_depth'
    camera_model = prepare_depth_model(run/'models', upstream) if camera else None
    environment = dict(os.environ)
    environment.update(PX4_GZ_MODELS=str(run/'models'), PX4_GZ_WORLDS=str(run/'worlds'),
        PX4_GZ_PLUGINS=str(build/'src/modules/simulation/gz_plugins'),
        PX4_GZ_WORLD='robot_lab_flight', PX4_SIM_MODEL='gz_'+args.vehicle,
        PX4_SYS_AUTOSTART='4002' if camera else '4001',
        GZ_SIM_SERVER_CONFIG_PATH=str(root/'src/modules/simulation/gz_bridge/server.config'),
        GZ_PARTITION='robot_lab_px4_'+uuid.uuid4().hex,
        PATH=str(build/'bin')+os.pathsep+os.environ['PATH'],
        PX4_GZ_MODEL_POSE=f'{args.spawn_x},{args.spawn_y},{args.spawn_z}')
    environment.update(HARMONIC_DISCOVERY_PORTS)
    if args.gui == 'false' or (args.gui == 'auto' and not environment.get('DISPLAY')):
        environment['HEADLESS'] = '1'
    else:
        environment.pop('HEADLESS', None)
    resources = [str(run/'models'), str(upstream/'models'), str(run/'worlds')]
    if args.world:
        resources.extend([str(Path(args.world).parent), str(Path(args.world).parent.parent/'models')])
    from ament_index_python.packages import get_package_share_directory
    maps = Path(get_package_share_directory('robot_lab_maps'))
    resources.extend(str(p) for p in maps.glob('maps/*/models'))
    environment['GZ_SIM_RESOURCE_PATH'] = os.pathsep.join(resources+[environment.get('GZ_SIM_RESOURCE_PATH', '')])
    environment['GZ_SIM_SYSTEM_PLUGIN_PATH'] = os.pathsep.join([environment['PX4_GZ_PLUGINS'], environment.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')])
    description = x500_urdf(upstream, camera=camera)
    (run/'x500.urdf').write_text(description)
    import yaml
    (run/'description.yaml').write_text(yaml.safe_dump({'robot_state_publisher': {'ros__parameters': {'robot_description': description, 'use_sim_time': False}}}))
    (run/'px4-revision.txt').write_text(subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True))
    processes = []
    log = (run/'px4.log').open('w')
    print('PX4 runtime and logs: '+str(run), flush=True)
    controller = relay = planner = database = None
    try:
        # PX4 (and the Gazebo server it starts) inherit the lease, so a PX4
        # that outlives this runtime still blocks a second instance on the
        # same MAVLink port instead of silently sharing it.
        fcu = subprocess.Popen([str(build/'bin/px4'), '-d', str(build/'etc'), '-w', str(run/'rootfs')],
                               env=environment, stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                               pass_fds=(instance_lease.fileno(),))
        processes.append(fcu)
        rsp = subprocess.Popen(['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher', '--ros-args',
                                '--params-file', str(run/'description.yaml')], start_new_session=True)
        processes.append(rsp)
        if camera:
            # System interpreter, isolated from per-user packages: the
            # Harmonic messages need the distribution's protobuf.
            processes.append(subprocess.Popen(
                ['/usr/bin/python3', '-s', '-m', 'robot_lab_adapter.gz_camera_bridge',
                 '--gz-topic', CAMERA_GZ_TOPIC, '--ros-prefix', CAMERA_ROS_PREFIX,
                 '--frame-id', CAMERA_OPTICAL_FRAME],
                env=environment, start_new_session=True))
        database = None
        if args.mapping == 'rtabmap':
            database = run/'aerial_map.db'
            if args.map_database:
                # Localize in a copy: the saved map itself is never written.
                import shutil
                shutil.copy2(Path(args.map_database).resolve(), database)
            (run/'mapping.json').write_text(json.dumps(dict(
                mapper='rtabmap', database=str(database),
                source_database=str(Path(args.map_database).resolve()) if args.map_database else None,
                mode='localization' if args.map_database else 'mapping',
                odometry='FCU estimate (TF map -> base_link)', camera_topic=CAMERA_ROS_PREFIX)))
            processes.append(subprocess.Popen(
                rtabmap_command(database, bool(args.map_database), run/'rtabmap.yaml'),
                stdout=(run/'rtabmap.log').open('w'), stderr=subprocess.STDOUT,
                env=rtabmap_environment(dict(os.environ)), start_new_session=True))
            print('Aerial RTAB-Map database: '+str(database), flush=True)
        if args.rviz == 'true':
            config = Path(get_package_share_directory('robot_lab_bringup'))/'config/px4_x500.rviz'
            processes.append(subprocess.Popen(['rviz2', '-d', str(config)], start_new_session=True))
        rclpy.init(args=['--ros-args', '-p', 'flight_enabled:='+args.flight_enabled,
                        '-p', f'origin_x:={args.spawn_x}', '-p', f'origin_y:={args.spawn_y}',
                        '-p', f'origin_z:={args.spawn_z+0.24}'])
        controller = PX4Controller()
        relay = NativeStateRelay(environment)
        executor = SingleThreadedExecutor()
        executor.add_node(controller)
        executor.add_node(relay)
        from .px4_planning import PX4Planning
        from robot_lab_utils.aerial_takeoff import x500_envelope
        envelope = x500_envelope(upstream/'models', model_file=run/'models/x500/model.sdf')
        if camera:
            from robot_lab_utils.aerial_takeoff import extend_envelope
            envelope = extend_envelope(envelope, camera_collision_corners(), camera_model)
        planner = PX4Planning(controller, args.world or str(upstream/'worlds/default.sdf'), envelope)
        executor.add_node(planner)
        while rclpy.ok() and fcu.poll() is None:
            executor.spin_once(timeout_sec=0.05)
            if (run/'px4.log').stat().st_size > args.log_limit_mb*1024**2:
                raise RuntimeError('PX4 log budget exceeded; closing the owned flight runtime')
    except KeyboardInterrupt:
        pass
    finally:
        if controller and controller.flight.armed():
            try:
                controller.command_mode(4, 6)
            except Exception as exc:
                print('PX4 landing request during cleanup failed: '+str(exc), flush=True)
        cleanup = []
        if relay:
            cleanup.extend([relay.close, relay.destroy_node])
        if planner:
            cleanup.extend([planner.close, planner.destroy_node])
        if controller:
            cleanup.extend([controller.flight.master.close, controller.destroy_node])
        if rclpy.ok():
            cleanup.append(rclpy.shutdown)
        cleanup.extend(lambda proc=proc: stop_group(proc) for proc in reversed(processes))
        cleanup.extend([log.close, instance_lease.close])
        for action in cleanup:
            try:
                action()
            except Exception as exc:
                print('PX4 cleanup: '+str(exc), flush=True)
        if database is not None and not args.map_database:
            # RTAB-Map writes its database when it stops (SIGINT above).
            try:
                cloud = export_cloud(database, run)
                print('Aerial map saved: %s%s' % (database, '; cloud '+cloud if cloud else
                      '; no cloud exported'), flush=True)
            except Exception as exc:
                print('Aerial map export: '+str(exc), flush=True)



if __name__ == '__main__':
    main()
