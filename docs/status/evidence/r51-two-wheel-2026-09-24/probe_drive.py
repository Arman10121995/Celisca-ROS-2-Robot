#!/usr/bin/env python3
"""Live differential-drive, stop, and timeout probe for a running loc launch.

Run in the same ROS_DOMAIN_ID as a headless robot_lab_bringup MuJoCo launch.
This uses the GUI's /key_vel input and records physics truth separately from
EKF /odometry/filtered. One process controls a single robot on nav_obstacle.
"""
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


def wrapped_angle(end, start):
    return math.atan2(math.sin(end - start), math.cos(end - start))


def main():
    rclpy.init()
    node = Node('diff_drive_live_probe')
    state = {'sim': None, 'truth': None, 'truth_count': 0, 'estimate_count': 0,
             'clock_count': 0, 'mux_count': 0}

    def on_clock(msg):
        state['sim'] = msg.clock.sec + msg.clock.nanosec * 1e-9
        state['clock_count'] += 1

    def on_truth(msg):
        pose = msg.pose.pose
        q = pose.orientation
        state['truth'] = {
            'x': pose.position.x, 'y': pose.position.y,
            'yaw': math.atan2(2 * (q.w * q.z + q.x * q.y),
                              1 - 2 * (q.y * q.y + q.z * q.z)),
            'vx': msg.twist.twist.linear.x,
            'wz': msg.twist.twist.angular.z,
        }
        state['truth_count'] += 1

    def on_estimate(_msg):
        state['estimate_count'] += 1

    def on_mux(_msg):
        state['mux_count'] += 1

    node.create_subscription(Clock, '/clock', on_clock, 10)
    node.create_subscription(Odometry, '/odom/ground_truth', on_truth, 10)
    node.create_subscription(Odometry, '/odometry/filtered', on_estimate, 10)
    node.create_subscription(Twist, '/robot_lab_controller/cmd_vel_unstamped',
                             on_mux, 10)
    publisher = node.create_publisher(Twist, '/key_vel', 10)
    trace = []
    report = {'passed': False}
    try:
        deadline = time.monotonic() + 30
        while state['sim'] is None or state['truth'] is None:
            if time.monotonic() >= deadline:
                raise RuntimeError('no /clock or /odom/ground_truth')
            rclpy.spin_once(node, timeout_sec=.02)
        # Allow DDS discovery and the localization graph to settle.
        until = time.monotonic() + 1
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.02)

        def snapshot():
            return dict(state['truth'])

        def publish(vx, wz):
            command = Twist()
            command.linear.x = float(vx)
            command.angular.z = float(wz)
            publisher.publish(command)

        def phase(name, vx, wz, seconds):
            start_sim = state['sim']
            first = snapshot()
            last_pub = float('-inf')
            wall_deadline = time.monotonic() + max(20, seconds * 8)
            while state['sim'] - start_sim < seconds:
                if time.monotonic() >= wall_deadline:
                    raise RuntimeError(name + ' exceeded its wall-time budget')
                rclpy.spin_once(node, timeout_sec=.01)
                if time.monotonic() - last_pub >= .1:
                    publish(vx, wz)
                    last_pub = time.monotonic()
            last = snapshot()
            delta_x = last['x'] - first['x']
            delta_y = last['y'] - first['y']
            result = {
                'phase': name, 'command': [vx, wz],
                'sim_duration_s': state['sim'] - start_sim,
                'start': first, 'end': last,
                'forward_m': (delta_x * math.cos(first['yaw']) +
                              delta_y * math.sin(first['yaw'])),
                'yaw_change_rad': wrapped_angle(last['yaw'], first['yaw']),
            }
            trace.append(result)
            return result

        forward = phase('forward', .2, 0, 4)
        stop_forward = phase('stop_after_forward', 0, 0, 1)
        turn = phase('turn', 0, .5, 2)
        stop_turn = phase('stop_after_turn', 0, 0, 1)
        reverse = phase('reverse', -.15, 0, 2)
        stop_reverse = phase('stop_after_reverse', 0, 0, 1)
        # The first command expires at both the mux and simulator after
        # 0.5 wall seconds; deliberately send nothing after this phase.
        phase('watchdog_drive', .2, 0, 1)
        watchdog_start = time.monotonic()
        while time.monotonic() - watchdog_start < 1.5:
            rclpy.spin_once(node, timeout_sec=.02)
        watchdog_end = snapshot()
        publish(0, 0)
        criteria = {
            'forward_m_min': .45, 'turn_rad_min': .6,
            'reverse_m_max': -.18, 'stop_speed_max_m_s': .05,
            'stop_yaw_rate_max_rad_s': .08,
            'watchdog_speed_max_m_s': .05,
        }
        checks = {
            'forward': forward['forward_m'] >= criteria['forward_m_min'],
            'turn': turn['yaw_change_rad'] >= criteria['turn_rad_min'],
            'reverse': reverse['forward_m'] <= criteria['reverse_m_max'],
            'stop_forward': abs(stop_forward['end']['vx']) < .05,
            'stop_turn': abs(stop_turn['end']['wz']) < .08,
            'stop_reverse': abs(stop_reverse['end']['vx']) < .05,
            'watchdog': abs(watchdog_end['vx']) < .05,
            'mux': state['mux_count'] > 0,
            'estimate_and_truth': state['estimate_count'] > 0 and state['truth_count'] > 0,
        }
        report.update(
            passed=all(checks.values()), criteria=criteria, checks=checks,
            phases=trace, watchdog_end=watchdog_end,
            topic_counts={key: state[key] for key in
                          ('clock_count', 'truth_count', 'estimate_count', 'mux_count')},
        )
    except Exception as exc:
        report['error'] = repr(exc)
    finally:
        publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
        print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
