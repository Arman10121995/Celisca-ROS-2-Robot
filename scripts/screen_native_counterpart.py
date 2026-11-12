#!/usr/bin/env python3
"""Finite GUI-input-path native locomotion screen, with actual ROS feedback.

Creates and cleans only its own launch process group. All outputs go to the
declared SSD directory. A completed screen is not a robot mission certificate.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot', default='asset_light_robotics_octopod')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if output.stat().st_dev == Path('/').stat().st_dev:
        parser.error('Use the mounted SSD')
    command = ['ros2', 'launch', 'robot_lab_bringup', 'simulated_robot.launch.py',
        'robot_model:='+args.robot, 'simulator:=mujoco', 'mode:=display', 'map_name:=none',
        'enable_native_locomotion:=true', 'gui:=false', 'start_rviz:=false']
    result = dict(command=command, states=[], joints=[], transforms=[], phases=[], error='',
                  scope='One experimental source-native locomotion screen; no terrain/mission qualification')
    (output/'command.json').write_text(json.dumps(command, indent=2))
    started = time.monotonic()
    rclpy.init()
    node = rclpy.create_node('native_counterpart_screen')
    pub = node.create_publisher(Twist, '/key_vel', 1)
    def state(msg):
        value = json.loads(msg.data)
        value['wall_elapsed'] = time.monotonic()-started
        result['states'].append(value)
    def joint(msg):
        if not result['joints'] or time.monotonic()-result['joints'][-1]['wall'] >= .1:
            result['joints'].append(dict(wall=time.monotonic(), name=list(msg.name),
                position=list(msg.position), velocity=list(msg.velocity), effort=list(msg.effort)))
    def tf(msg):
        for t in msg.transforms:
            if t.child_frame_id == 'native_body_1':
                result['transforms'].append(dict(wall_elapsed=time.monotonic()-started,
                    frame=t.header.frame_id, position=[t.transform.translation.x,
                    t.transform.translation.y, t.transform.translation.z],
                    quaternion=[t.transform.rotation.w, t.transform.rotation.x,
                                t.transform.rotation.y, t.transform.rotation.z]))
    node.create_subscription(String, '/robot_lab/locomotion/state', state, 10)
    node.create_subscription(JointState, '/joint_states', joint, 10)
    node.create_subscription(TFMessage, '/tf', tf, 10)
    process = None
    try:
        with (output/'launch.log').open('w') as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic()+30.
            while not result['states'] and time.monotonic() < deadline and process.poll() is None:
                rclpy.spin_once(node, timeout_sec=.05)
            if not result['states']:
                raise RuntimeError('Native state did not arrive in the startup budget')
            for name, vx, yaw, seconds in [('startup', 0., 0., 3.), ('forward', .12, 0., 6.),
                    ('neutral', 0., 0., 3.), ('reverse', -.12, 0., 6.), ('turn', 0., .25, 6.),
                    ('publisher_loss', None, None, 3.)]:
                before = result['states'][-1]
                deadline = time.monotonic()+seconds
                while time.monotonic() < deadline:
                    if process.poll() is not None:
                        raise RuntimeError('Launch exited during '+name)
                    if vx is not None:
                        msg = Twist(); msg.linear.x = vx; msg.angular.z = yaw
                        pub.publish(msg)
                    rclpy.spin_once(node, timeout_sec=.04)
                    if result['states'][-1]['fault']:
                        raise RuntimeError('Plant/controller fault: '+result['states'][-1]['fault'])
                after = result['states'][-1]
                a, b = before['root_pose_truth'], after['root_pose_truth']
                yaw_a = math.atan2(2*(a[3]*a[6]+a[4]*a[5]),1-2*(a[5]**2+a[6]**2))
                yaw_b = math.atan2(2*(b[3]*b[6]+b[4]*b[5]),1-2*(b[5]**2+b[6]**2))
                result['phases'].append(dict(name=name, requested=[vx, 0., yaw],
                    sim_s=after['sim_time']-before['sim_time'], wall_s=seconds,
                    displacement_world_m=[b[i]-a[i] for i in range(3)],
                    heading_change_rad=math.atan2(math.sin(yaw_b-yaw_a), math.cos(yaw_b-yaw_a)),
                    final_command=after['command']))
    except (Exception, KeyboardInterrupt) as exc:
        result['error'] = str(exc) or type(exc).__name__
    finally:
        pub.publish(Twist())
        if process and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3.)
        result['wall_s'] = time.monotonic()-started
        node.destroy_node(); rclpy.shutdown()
        (output/'screen.json').write_text(json.dumps(result, indent=2, allow_nan=False))
        summary = {k:v for k,v in result.items() if k not in ('states', 'joints', 'transforms')}
        summary.update(state_count=len(result['states']), joint_count=len(result['joints']),
            transform_count=len(result['transforms']))
        (output/'summary.json').write_text(json.dumps(summary, indent=2))
        print(json.dumps(summary, indent=2))
    return 1 if result['error'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
