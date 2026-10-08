#!/usr/bin/env python3
"""Observe normal GUI localization and compare lidar endpoints with its map.

No movement or goal is requested; normal Stop publishes zeros on shutdown.
This diagnostic does not qualify navigation.
Run serially with sourced ROS/SSD environments, Xvfb, and an isolated domain.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import time

import numpy as np
from PIL import Image
from scipy.spatial import cKDTree
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot', default='bumperbot')
    parser.add_argument('--backend', default='mujoco', choices=['gazebo', 'mujoco', 'pybullet', 'isaac'])
    parser.add_argument('--map', required=True)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--timeout', type=float, default=240.)
    parser.add_argument('--scans', type=int, default=20)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.output.parent.stat().st_dev == Path('/').stat().st_dev:
        parser.error('Outputs must be on the workspace SSD')
    args.output.mkdir(exist_ok=False)
    out = args.output
    (out/'producer.py').write_bytes(Path(__file__).read_bytes())

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from ament_index_python.packages import get_package_share_directory
    from robot_lab_gui.launcher import SimulationLauncherGui
    from robot_lab_utils.robot_description import load_description
    from robot_lab_utils.sim_frames import mounted_pose, offset_from_root, yaw_of

    app = SimulationLauncherGui()
    app.update()
    if not rclpy.ok():
        rclpy.init()
    node = Node('scan_map_alignment_diagnostic')
    samples, scans, commands = {}, [], []
    report = dict(robot=args.robot, backend=args.backend, map=args.map,
        scope='Read-only stationary normal-GUI localization diagnostic; no navigation or motion qualification.',
        revision=subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        domain=os.environ.get('ROS_DOMAIN_ID'), completed=False)

    def stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9

    def pose(msg, key):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        samples[key] = dict(stamp_s=stamp(msg), position=[p.x, p.y, p.z], quaternion=[q.w, q.x, q.y, q.z])

    def scan(msg):
        if not all(k in samples for k in ('truth', 'amcl', 'filtered')):
            return
        t = stamp(msg)
        if abs(t-samples['truth']['stamp_s']) > .1:
            return
        scans.append(dict(stamp_s=t, frame=msg.header.frame_id, angle_min=msg.angle_min,
            angle_increment=msg.angle_increment, range_min=msg.range_min, range_max=msg.range_max,
            ranges=[float(v) if math.isfinite(v) else None for v in msg.ranges],
            poses=dict(samples)))

    subscriptions = [
        node.create_subscription(Odometry, '/odom/ground_truth', lambda m: pose(m, 'truth'), qos_profile_sensor_data),
        node.create_subscription(Odometry, '/odometry/filtered', lambda m: pose(m, 'filtered'), qos_profile_sensor_data),
        node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m: pose(m, 'amcl'), qos_profile_sensor_data),
        node.create_subscription(LaserScan, '/scan', scan, qos_profile_sensor_data),
        node.create_subscription(Twist, '/key_vel', lambda m: commands.append([m.linear.x, m.linear.y, m.angular.z]), 10),
    ]

    def pump():
        app.update()
        rclpy.spin_once(node, timeout_sec=0.)
        time.sleep(.005)

    try:
        app.robot_var.set(args.robot)
        app.simulator_var.set(args.backend)
        app.map_var.set(args.map)
        app.mode_var.set('loc')
        app.gui_var.set('false')
        app._update_from_selection()
        app.update()
        assert app.mode_var.get() == 'loc' and app.start_button.instate(['!disabled']), app.validation_var.get()
        report['command'] = app.command_var.get()
        map_path = Path(app.map_profiles[args.map]['map']['path'])
        if not map_path.is_absolute():
            map_path = Path(get_package_share_directory(app.map_profiles[args.map]['map']['package']))/map_path
        metadata = yaml.safe_load(map_path.read_text())
        image_path = map_path.parent/metadata['image']
        image = np.asarray(Image.open(image_path))
        darkness = image/255. if metadata.get('negate', 0) else 1-image/255.
        rows, cols = np.nonzero(darkness > metadata['occupied_thresh'])
        resolution = float(metadata['resolution'])
        ox, oy, theta = metadata['origin']
        points = np.column_stack(((cols+.5)*resolution, (image.shape[0]-rows-.5)*resolution))
        rotation = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
        tree = cKDTree(points@rotation.T+[ox, oy])
        robot_profile = app.robot_profiles[args.robot]
        model_path = Path(robot_profile['xacro'])
        if not model_path.is_absolute():
            model_path = Path(get_package_share_directory(robot_profile['package']))/model_path
        description = load_description(model_path)
        offset = offset_from_root(description, 'laser_link')
        if offset is None:
            raise ValueError('Diagnostic requires the measured laser_link mount')
        report['laser_mount'] = offset
        report['map_files'] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in [map_path, image_path]}
        report['source_files'] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in [
            Path(__file__), Path('src/robot_lab_mujoco/python/robot_lab_mujoco/mujoco_spawner.py'),
            Path('src/robot_lab_localization/config/robots')/(args.robot+'.yaml')] if p.is_file()}
        (out/'pre-trial.json').write_text(json.dumps(report, indent=2)+'\n')
        app.start_button.invoke()
        report['owned_pid'] = app.process.pid
        deadline = time.monotonic()+args.timeout
        while len(scans) < args.scans and time.monotonic() < deadline:
            pump()
            assert app.process.poll() is None, 'Owned localization launch exited'
        assert len(scans) >= args.scans, f'Only {len(scans)} synchronized scans'
        distances = {'truth': [], 'amcl': []}
        for sample in scans:
            for kind in distances:
                base = sample['poses'][kind]
                origin, quaternion = mounted_pose(base['position'], base['quaternion'], offset)
                yaw = yaw_of(quaternion)
                endpoints = [[origin[0]+r*math.cos(yaw+sample['angle_min']+i*sample['angle_increment']),
                    origin[1]+r*math.sin(yaw+sample['angle_min']+i*sample['angle_increment'])]
                    for i, r in enumerate(sample['ranges']) if r is not None and sample['range_min'] < r < sample['range_max']]
                distances[kind].extend(tree.query(endpoints)[0].tolist())
        report['endpoint_distance_to_occupied_cell_m'] = {k: dict(samples=len(v), median=float(np.median(v)),
            p90=float(np.percentile(v, 90)), within_two_cells=float(np.mean(np.asarray(v) <= 2*resolution))) for k, v in distances.items()}
        report['last_poses'] = scans[-1]['poses']
        report['keyboard_commands_before_stop'] = list(commands)
        assert not commands, 'Stationary GUI diagnostic must not send keyboard commands'
        report['completed'] = True
    except BaseException as exc:
        report['exception'] = repr(exc)
        raise
    finally:
        if app.process and app.process.poll() is None:
            app._stop_launch()
            deadline = time.monotonic()+60
            while app._launch_running and time.monotonic() < deadline:
                pump()
        report['launch_returncode'] = app.process.poll() if app.process else None
        (out/'scan-pose-trace.json').write_text(json.dumps(scans, indent=2)+'\n')
        (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        (out/'gui-output.log').write_text(app.output.get('1.0', 'end'))
        node.destroy_node()
        if getattr(app, 'ros_executor', None):
            app.ros_executor.shutdown(timeout_sec=2.)
        if app.arm_tab:
            app.arm_tab.close()
        if app.hand_tab:
            app.hand_tab.close()
        app.destroy()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
