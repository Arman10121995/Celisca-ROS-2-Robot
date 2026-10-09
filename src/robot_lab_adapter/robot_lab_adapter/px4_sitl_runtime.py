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


def x500_urdf(upstream):
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
    return ET.tostring(robot, encoding='unicode')


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
    args = parser.parse_args()
    if not math.isfinite(args.log_limit_mb) or args.log_limit_mb <= 0:
        parser.error('PX4 log limit must be positive and finite')
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
    for folder in ('rootfs', 'worlds', 'models'):
        (run/folder).mkdir()
    prepare_world(args.world, run/'worlds/robot_lab_flight.sdf', upstream)
    prepare_model(run/'models/x500', upstream)
    environment = dict(os.environ)
    environment.update(PX4_GZ_MODELS=str(run/'models'), PX4_GZ_WORLDS=str(run/'worlds'),
        PX4_GZ_PLUGINS=str(build/'src/modules/simulation/gz_plugins'),
        PX4_GZ_WORLD='robot_lab_flight', PX4_SIM_MODEL='gz_x500', PX4_SYS_AUTOSTART='4001',
        GZ_SIM_SERVER_CONFIG_PATH=str(root/'src/modules/simulation/gz_bridge/server.config'),
        GZ_PARTITION='robot_lab_px4_'+uuid.uuid4().hex,
        PATH=str(build/'bin')+os.pathsep+os.environ['PATH'],
        PX4_GZ_MODEL_POSE=f'{args.spawn_x},{args.spawn_y},{args.spawn_z}')
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
    description = x500_urdf(upstream)
    (run/'x500.urdf').write_text(description)
    import yaml
    (run/'description.yaml').write_text(yaml.safe_dump({'robot_state_publisher': {'ros__parameters': {'robot_description': description, 'use_sim_time': False}}}))
    (run/'px4-revision.txt').write_text(subprocess.check_output(['git', '-C', str(root), 'rev-parse', 'HEAD'], text=True))
    processes = []
    log = (run/'px4.log').open('w')
    print('PX4 runtime and logs: '+str(run), flush=True)
    controller = relay = planner = None
    try:
        fcu = subprocess.Popen([str(build/'bin/px4'), '-d', str(build/'etc'), '-w', str(run/'rootfs')],
                               env=environment, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(fcu)
        rsp = subprocess.Popen(['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher', '--ros-args',
                                '--params-file', str(run/'description.yaml')], start_new_session=True)
        processes.append(rsp)
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
        planner = PX4Planning(controller, args.world or str(upstream/'worlds/default.sdf'))
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



if __name__ == '__main__':
    main()
