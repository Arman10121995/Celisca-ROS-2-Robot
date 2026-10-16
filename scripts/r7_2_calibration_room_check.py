#!/usr/bin/env python3
"""R7.2 camera_calibration in any simulator: turn in front of fixed chessboards.

Runs against a robot launched in the calibration room
(``scripts/r7_2_calibration_room_check.sh <simulator> <robot> <out>``),
with ``perception:=camera_calibration``.  The robot turns on the spot on
``/cmd_vel``; every colour frame goes to
robot_lab_algorithms.camera_calibration's ChessboardCalibrator (which keeps
views that differ in board position, size or tilt), and the node's own
latched result on /perception/calibrated_camera_info is read at the end.

Both are compared with the camera info and with a pinhole whose principal
point sits at the pixel centres, ((w - 1) / 2, (h - 1) / 2), as in
scripts/r7_2_calibration_check.py (Gazebo, one moving board).

  python3 scripts/r7_2_calibration_room_check.py --out <file.json> [--turns 2]
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src', 'robot_lab_algorithms'))

from robot_lab_algorithms.camera_calibration import ChessboardCalibrator, parse_pattern  # noqa: E402
from robot_lab_algorithms.visual_odometry import decode_gray  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--pattern', default='8x6')
    parser.add_argument('--square', type=float, default=0.04)
    parser.add_argument('--rate', type=float, default=0.15, help='turn rate, rad/s')
    parser.add_argument('--turns', type=float, default=2.0)
    parser.add_argument('--timeout', type=float, default=1800.0, help='wall seconds')
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    pattern = parse_pattern(args.pattern)

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    rclpy.init()
    node = Node('r7_2_calibration_room_check',
                parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    state = {'image': None, 'info': None, 'yaw': None, 'calibrated': None, 'images': 0}

    def on_image(msg):
        state['image'] = msg
        state['images'] += 1

    def on_truth(msg):
        q = msg.pose.pose.orientation
        state['yaw'] = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    node.create_subscription(Image, '/oakd/rgb/image_raw', on_image, qos_profile_sensor_data)
    node.create_subscription(CameraInfo, '/oakd/rgb/camera_info', lambda m: state.__setitem__('info', m),
                             qos_profile_sensor_data)
    node.create_subscription(Odometry, '/odom/ground_truth', on_truth, qos_profile_sensor_data)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(CameraInfo, '/perception/calibrated_camera_info',
                             lambda m: state.__setitem__('calibrated', m), latched)
    command = node.create_publisher(Twist, '/cmd_vel', 10)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    report = {'pattern': args.pattern, 'square_m': args.square, 'rate_rad_s': args.rate, 'turns': args.turns}
    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline and (state['image'] is None or state['info'] is None
                                           or state['yaw'] is None):
        spin(0.5)
    if state['image'] is None or state['info'] is None or state['yaw'] is None:
        report['error'] = 'no camera image, camera info or truth'
        json.dump(report, open(args.out, 'w'), indent=1)
        print(json.dumps(report))
        return 1
    info = state['info']
    width, height = info.width, info.height
    calibrator = ChessboardCalibrator(pattern, args.square, min_views=15, max_views=150, min_difference=0.05)
    outcomes, turned, last_yaw, last_stamp = [], 0.0, state['yaw'], None
    twist = Twist()
    twist.angular.z = args.rate
    started = time.monotonic()
    while turned < 2 * math.pi * args.turns and time.monotonic() < deadline:
        command.publish(twist)
        spin(0.05)
        turned += abs(math.atan2(math.sin(state['yaw'] - last_yaw), math.cos(state['yaw'] - last_yaw)))
        last_yaw = state['yaw']
        image = state['image']
        stamp = (image.header.stamp.sec, image.header.stamp.nanosec)
        if stamp != last_stamp:
            last_stamp = stamp
            outcomes.append(calibrator.add(decode_gray(image)))
    command.publish(Twist())
    spin(4.0)   # the node finishes its own views
    report.update(turned_rad=round(turned, 3), wall_s=round(time.monotonic() - started, 1),
                  frames=len(outcomes), images_received=state['images'])
    report['outcomes'] = {k: outcomes.count(k) for k in sorted(set(outcomes))}
    k_info = list(info.k)
    report['camera_info'] = {'fx': k_info[0], 'fy': k_info[4], 'cx': k_info[2], 'cy': k_info[5],
                             'width': width, 'height': height}
    report['pixel_centres'] = {'cx': (width - 1) / 2.0, 'cy': (height - 1) / 2.0}
    if calibrator.ready():
        for model in ('plumb_bob', 'pinhole'):
            result = calibrator.calibrate(model)
            k = result['camera_matrix']
            report['script_calibration_' + model] = {
                'views': result['views'], 'rms_px': round(result['rms_px'], 4),
                'fx': round(float(k[0, 0]), 3), 'fy': round(float(k[1, 1]), 3),
                'cx': round(float(k[0, 2]), 3), 'cy': round(float(k[1, 2]), 3),
                'distortion': [round(float(v), 5) for v in result['distortion']],
                'coverage': [round(v, 2) for v in calibrator.coverage()]}
    calibrated = state['calibrated']
    if calibrated is not None:
        report['node_calibration'] = {'fx': round(calibrated.k[0], 3), 'fy': round(calibrated.k[4], 3),
                                      'cx': round(calibrated.k[2], 3), 'cy': round(calibrated.k[5], 3),
                                      'distortion': [round(float(v), 5) for v in calibrated.d]}
    node.destroy_node()
    rclpy.shutdown()
    with open(args.out, 'w') as handle:
        json.dump(report, handle, indent=1)
    print(json.dumps(report, indent=1))
    return 0 if 'script_calibration_pinhole' in report else 1


if __name__ == '__main__':
    sys.exit(main())
