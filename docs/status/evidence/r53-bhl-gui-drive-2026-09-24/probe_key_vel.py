#!/usr/bin/env python3
"""Record a bounded BHL Drive-pad command against an already running launch.

Use the same ROS_DOMAIN_ID as the simulator. Environment controls:
BHL_PROBE_TOPIC (/key_vel), BHL_PROBE_SPEED (0.25 m/s),
BHL_PROBE_YAW (0 rad/s), BHL_PROBE_START_T (1 simulated second: when the
speed/yaw command turns on; a zero twist is still published from t=1 so the
controller's settle ramp runs exactly as in a GUI drive),
BHL_PROBE_STOP_T (6 simulated seconds), BHL_PROBE_RATE_HZ (10 Hz), and
BHL_PROBE_MUX_TOPIC (/robot_lab_controller/cmd_vel_unstamped: the post-mux
twist the policy actually consumes). The probe observes five more seconds
after stop, and every record ends with the last post-mux (vx, wz) seen.
"""
import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray


def main():
    topic = os.getenv('BHL_PROBE_TOPIC', '/key_vel')
    mux_topic = os.getenv('BHL_PROBE_MUX_TOPIC',
                          '/robot_lab_controller/cmd_vel_unstamped')
    stop_t = float(os.getenv('BHL_PROBE_STOP_T', '6'))
    start_t = float(os.getenv('BHL_PROBE_START_T', '1'))
    speed = float(os.getenv('BHL_PROBE_SPEED', '.25'))
    yaw_rate = float(os.getenv('BHL_PROBE_YAW', '0'))
    rate = float(os.getenv('BHL_PROBE_RATE_HZ', '10'))
    effort_vector = os.getenv('BHL_PROBE_EFFORT_VECTOR', '0') not in ('0', '', 'false', 'no')

    if not all(math.isfinite(v)
               for v in (stop_t, start_t, speed, yaw_rate, rate)):
        raise ValueError('probe controls must be finite')
    if stop_t <= 1 or rate <= 0:
        raise ValueError('stop time must exceed 1 s and rate must be positive')
    if start_t < 0 or start_t >= stop_t:
        raise ValueError('start time must be non-negative and before stop')

    rclpy.init()
    node = Node('bhl_launch_probe')
    state = {
        't': None, 'xy': None, 'yaw': None, 'tilt': None,
        'cmd': [0.0, 0.0],
        'effort': None,
        'messages': {'odom': 0, 'imu': 0, 'joint': 0, 'effort': 0,
                     'clock': 0, 'mux': 0},
        'effort_times': [], 'records': [],
    }

    def on_clock(msg):
        state['t'] = msg.clock.sec + msg.clock.nanosec * 1e-9
        state['messages']['clock'] += 1

    def on_odom(msg):
        pose = msg.pose.pose
        state['xy'] = [pose.position.x, pose.position.y]
        q = pose.orientation
        state['yaw'] = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                  1 - 2 * (q.y * q.y + q.z * q.z))
        state['messages']['odom'] += 1

    def on_imu(msg):
        q = msg.orientation
        state['tilt'] = math.acos(max(-1, min(1, 1 - 2 * (q.x * q.x + q.y * q.y))))
        state['messages']['imu'] += 1

    def on_joint(_msg):
        state['messages']['joint'] += 1

    def on_effort(msg):
        state['messages']['effort'] += 1
        state['effort_times'].append(time.monotonic())
        if effort_vector:
            values = [float(v) for v in msg.data]
            if values and all(math.isfinite(v) for v in values):
                # Last full effort vector, for gait-activity analysis; the
                # scalar effort counter alone cannot tell a stepping policy
                # from one parked at a fixed target.
                state['effort'] = [round(v, 3) for v in values]

    def on_mux(msg):
        state['cmd'] = [msg.linear.x, msg.angular.z]
        state['messages']['mux'] += 1

    for msg_type, name, callback in (
        (Clock, '/clock', on_clock),
        (Odometry, '/odom/ground_truth', on_odom),
        (Imu, '/imu/out', on_imu),
        (JointState, '/joint_states', on_joint),
        (Float64MultiArray, '/bhl_standing_controller/commands', on_effort),
        (Twist, mux_topic, on_mux),
    ):
        node.create_subscription(msg_type, name, callback, 10)
    command_pub = node.create_publisher(Twist, topic, 10)

    try:
        wall_start = time.monotonic()
        while (state['t'] is None or state['xy'] is None or state['tilt'] is None):
            if time.monotonic() - wall_start > 30:
                raise RuntimeError('no clock, odometry, or IMU within 30 s')
            rclpy.spin_once(node, timeout_sec=.02)
        sim_start = state['t']
        start_xy = list(state['xy'])
        start_yaw = state['yaw']
        last_pub = float('-inf')
        last_record = float('-inf')
        while state['t'] - sim_start < stop_t + 5:
            if time.monotonic() - wall_start > 90:
                raise RuntimeError('simulation did not finish within 90 s')
            rclpy.spin_once(node, timeout_sec=.01)
            t = state['t'] - sim_start
            if t >= 1 and time.monotonic() - last_pub >= 1 / rate:
                command = Twist()
                if start_t <= t < stop_t:
                    command.linear.x = speed
                    command.angular.z = yaw_rate
                command_pub.publish(command)
                last_pub = time.monotonic()
            if t - last_record >= .04:
                state['records'].append([
                    round(t, 3), *state['xy'], state['tilt'],
                    state['messages']['effort'], state['yaw'],
                    round(state['cmd'][0], 3), round(state['cmd'][1], 3),
                    *( [state['effort']] if effort_vector else [] ),
                ])
                last_record = t
        command_pub.publish(Twist())
        records = state['records']
        walk = [r for r in records if stop_t - .2 < r[0] < stop_t + .1]
        effort_times = state['effort_times']
        report = {
            'duration': records[-1][0], 'start_xy': start_xy,
            'start_yaw': start_yaw,
            'command_window': [start_t, stop_t],
            'command': {'speed': speed, 'yaw_rate': yaw_rate},
            'walk_xy': walk[-1][1:3] if walk else None,
            'end_xy': records[-1][1:3], 'end_yaw': records[-1][5],
            'max_tilt': max(r[3] for r in records),
            'end_tilt': records[-1][3],
            'messages': state['messages'],
            'effort_vector': effort_vector,
            'mux_last_command': state['cmd'],
            'effort_rate_wall': (len(effort_times) /
                                 (effort_times[-1] - effort_times[0])),
            # The effort vector is a 25 Hz gait signal: it is only useful at
            # full rate, so the usual 10x decimation is skipped for it.
            'records': records if effort_vector else records[::10],
        }
        print(json.dumps(report, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
