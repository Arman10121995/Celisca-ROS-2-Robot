#!/usr/bin/env python3
"""Bounded flight acceptance against ROS services and independent body truth."""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger


class Probe(Node):
    def __init__(self):
        super().__init__('px4_ros_flight_probe')
        self.state, self.truth, self.estimate = {}, None, None
        self.rotor_max = 0.0
        self.create_subscription(String, '/px4/status', self.status, 10)
        self.create_subscription(Odometry, '/px4/odometry_truth', self.body, 10)
        self.create_subscription(Odometry, '/px4/odometry', self.odom, 10)
        self.create_subscription(JointState, '/joint_states', self.joints, 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/px4/goal', 10)
        self.drive_pub = self.create_publisher(Twist, '/key_vel', 10)
        self.flight_clients = {a: self.create_client(Trigger, '/px4/'+a) for a in ('takeoff', 'hold', 'land')}

    def status(self, msg):
        self.state = json.loads(msg.data)

    def body(self, msg):
        p = msg.pose.pose.position
        self.truth = [p.x, p.y, p.z]

    def odom(self, msg):
        p = msg.pose.pose.position
        self.estimate = [p.x, p.y, p.z]

    def joints(self, msg):
        self.rotor_max = max([self.rotor_max]+[abs(v) for v in msg.velocity])

    def wait(self, condition, seconds, drive=None):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            if drive is not None:
                self.drive_pub.publish(drive)
            rclpy.spin_once(self, timeout_sec=0.05)
            if condition():
                return True
        return False

    def action(self, action):
        client = self.flight_clients[action]
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError('No ROS service /px4/'+action)
        future = client.call_async(Trigger.Request())
        if not self.wait(future.done, 5):
            raise RuntimeError('Flight service timed out: '+action)
        response = future.result()
        if not response.success:
            raise RuntimeError(response.message)
        return response.message


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    rclpy.init()
    probe = Probe()
    report = {'acceptance': {}, 'phases': {}}
    try:
        if not probe.wait(lambda: probe.state.get('position_ready') and probe.truth is not None, 60):
            raise RuntimeError('No fresh FCU estimate or independent Gazebo truth')
        initial = list(probe.truth)
        zero = Twist()
        probe.wait(lambda: False, 3, drive=zero)
        report['acceptance']['idle_unarmed_stationary'] = not probe.state.get('armed') and math.dist(initial, probe.truth) < 0.08
        report['phases']['idle'] = {'start_truth': initial, 'end_truth': probe.truth, 'state': dict(probe.state)}
        report['phases']['takeoff_request'] = probe.action('takeoff')
        if not probe.wait(lambda: probe.state.get('phase') == 'flying', 45):
            raise RuntimeError('Vehicle did not reach flying state: '+str(probe.state))
        probe.wait(lambda: False, 3)
        report['phases']['hover_truth'] = list(probe.truth)
        report['acceptance']['takeoff_hover_truth'] = abs(probe.truth[2]-initial[2]-3) < 0.35
        goal = [probe.estimate[0]+2, probe.estimate[1]+1, probe.estimate[2]]
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = goal
        msg.pose.orientation.w = 1.0
        probe.goal_pub.publish(msg)
        arrived = probe.wait(lambda: math.dist(probe.truth, goal) < 0.25, 20)
        report['phases']['goal'] = {'target_enu': goal, 'final_truth': list(probe.truth), 'final_estimate': probe.estimate}
        report['acceptance']['goal_truth'] = arrived
        probe.action('hold')
        probe.wait(lambda: False, 2)
        start = list(probe.truth)
        command = Twist()
        command.linear.x = 0.4
        probe.wait(lambda: False, 4, drive=command)
        probe.wait(lambda: False, 2, drive=zero)
        stopped = list(probe.truth)
        probe.wait(lambda: False, 3, drive=zero)
        report['phases']['drive'] = {'start_truth': start, 'stop_truth': stopped,
                                    'held_truth': list(probe.truth)}
        report['acceptance']['drive_moves_body'] = math.dist(start[:2], stopped[:2]) > 1.0
        report['acceptance']['released_drive_holds'] = math.dist(stopped, probe.truth) < 0.2
        report['phases']['land_request'] = probe.action('land')
        landed = probe.wait(lambda: probe.state.get('phase') == 'idle' and not probe.state.get('armed'), 35)
        report['phases']['land_truth'] = probe.truth
        report['acceptance']['landed_disarmed_truth'] = landed and abs(probe.truth[2]-initial[2]) < 0.1
        report['rotor_max_measured_rad_s'] = probe.rotor_max
        report['acceptance']['rotor_states_measured'] = probe.rotor_max > 10
    except Exception as exc:
        report['error'] = str(exc)
        if probe.state.get('armed'):
            try:
                probe.action('land')
                probe.wait(lambda: not probe.state.get('armed'), 30)
            except Exception:
                pass
    finally:
        report['passed'] = not report.get('error') and bool(report['acceptance']) and all(report['acceptance'].values())
        Path(args.out).write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2))
        probe.destroy_node()
        rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
