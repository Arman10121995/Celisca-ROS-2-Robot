#!/usr/bin/env python3
"""Drive the tracked vehicle through the manual input and measure it by truth.

Usage: track_probe.py <simulator> <output_dir> [map] [robot]

Launches tracked_simulator.launch.py (display task, headless), waits for the
command owner to report ready, then on /key_vel: forward 0.2 m/s for 8 s,
settle 3 s, turn 0.4 rad/s for 6 s, settle 3 s, reverse 0.15 m/s for 6 s,
settle 3 s, then stops publishing (watchdog). Truth is /odom/ground_truth;
the vehicle's own estimate is /robot_lab_controller/odom.
"""
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

PHASES = [('forward', 0.2, 0.0, 8.0), ('settle', 0, 0, 3.0), ('turn', 0.0, 0.4, 6.0), ('settle', 0, 0, 3.0),
          ('reverse', -0.15, 0.0, 6.0), ('settle', 0, 0, 3.0)]


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    simulator, output = sys.argv[1], Path(sys.argv[2])
    world = sys.argv[3] if len(sys.argv) > 3 else 'nav_empty'
    robot = sys.argv[4] if len(sys.argv) > 4 else 'lab_tracked'
    output.mkdir(parents=True, exist_ok=True)
    log = open(output / 'launch.log', 'w')
    launch = subprocess.Popen(['ros2', 'launch', 'robot_lab_bringup', 'tracked_simulator.launch.py',
                               'simulator:=' + simulator, 'map_name:=' + world, 'robot_model:=' + robot,
                               'task:=display', 'gui:=false', 'rviz:=false'],
                              stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('track_probe')
    samples, state = dict(truth=[], odom=[]), {}

    def keep(key):
        def store(msg):
            p = msg.pose.pose
            samples[key].append((time.monotonic(), p.position.x, p.position.y, p.position.z, yaw(p.orientation),
                                 msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9))
        return store
    node.create_subscription(Odometry, '/odom/ground_truth', keep('truth'), 50)
    node.create_subscription(Odometry, '/robot_lab_controller/odom', keep('odom'), 50)
    node.create_subscription(String, '/tracked/state', lambda m: state.update(json.loads(m.data)), 10)
    command = node.create_publisher(Twist, '/key_vel', 10)
    result = dict(simulator=simulator, map=world, robot=robot, phases=[])
    try:
        deadline = time.monotonic() + 180
        while time.monotonic() < deadline and not state.get('ready'):
            rclpy.spin_once(node, timeout_sec=0.1)
            if launch.poll() is not None:
                raise RuntimeError('launch exited early')
        if not state.get('ready'):
            raise RuntimeError('command owner never reported ready: %s' % state)
        settle = time.monotonic() + 3
        while time.monotonic() < settle:
            rclpy.spin_once(node, timeout_sec=0.05)
        for name, linear, angular, duration in PHASES:
            start_truth, start_odom = samples['truth'][-1], samples['odom'][-1]
            start = time.monotonic()
            while time.monotonic() - start < duration:
                msg = Twist()
                msg.linear.x, msg.angular.z = float(linear), float(angular)
                if name != 'settle':
                    command.publish(msg)
                rclpy.spin_once(node, timeout_sec=0.05)
            end_truth, end_odom = samples['truth'][-1], samples['odom'][-1]
            heading = start_truth[4]
            dx, dy = end_truth[1] - start_truth[1], end_truth[2] - start_truth[2]
            row = dict(phase=name, linear=linear, angular=angular, wall_s=duration,
                       sim_s=round(end_truth[5] - start_truth[5], 3),
                       truth_along_m=round(dx * math.cos(heading) + dy * math.sin(heading), 4),
                       truth_lateral_m=round(-dx * math.sin(heading) + dy * math.cos(heading), 4),
                       truth_yaw_rad=round(math.remainder(end_truth[4] - start_truth[4], math.tau), 4),
                       truth_z_m=round(end_truth[3], 4),
                       odom_distance_m=round(math.hypot(end_odom[1] - start_odom[1], end_odom[2] - start_odom[2]), 4),
                       odom_yaw_rad=round(math.remainder(end_odom[4] - start_odom[4], math.tau), 4))
            result['phases'].append(row)
            print(json.dumps(row), flush=True)
        result['final_state'] = {key: state.get(key) for key in ('ready', 'owner', 'command', 'tilt_rad')}
        result['passed_run'] = True
    except Exception as exc:
        result['error'] = str(exc)
        result['passed_run'] = False
    finally:
        (output / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
        node.destroy_node()
        rclpy.shutdown()
        try:
            os.killpg(launch.pid, signal.SIGINT)
            launch.wait(timeout=30)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            try:
                os.killpg(launch.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        log.close()
    print(json.dumps(result.get('error', 'ok')))


if __name__ == '__main__':
    main()
