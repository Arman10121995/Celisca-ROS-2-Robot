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

``--stop-and-shoot DEG`` turns DEG at a time and uses only frames taken
after the robot has stopped and settled (no motion blur or latency while
turning).  Then every settled view also measures the principal point
directly: the image of each board's grid centre (through its corner
homography) against the board's world centre from the world file,
projected with the published camera info, the robot's true pose and the
camera mount from TF.  The mean difference is the principal point offset
(``centre_check``); MuJoCo and PyBullet, whose calibrations pass, are its
controls.
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

from robot_lab_algorithms.camera_calibration import (  # noqa: E402
    ChessboardCalibrator, find_board, grid_centre_pixel, parse_pattern, project_point)
from robot_lab_algorithms.visual_odometry import decode_gray  # noqa: E402


def default_world():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('robot_lab_maps'), 'maps', 'calibration_room', 'worlds',
                        'calibration_room.world')


def board_centres(world):
    """{name: world centre} of every calibration board included in *world*."""
    import xml.etree.ElementTree as ET
    centres = {}
    for include in ET.parse(world).getroot().iter('include'):
        uri, name, pose = include.findtext('uri', ''), include.findtext('name', ''), include.findtext('pose', '')
        if 'calibration_board' in uri and pose:
            centres[name] = np.array([float(v) for v in pose.split()[:3]])
    return centres


def centre_view(gray, pattern, square, info, truth, tf_buffer, boards, max_px=30.0):
    """(board, du, dv) for one settled frame, or None (no board, pose or TF)."""
    corners = find_board(gray, pattern)
    if corners is None or truth is None:
        return None
    measured = grid_centre_pixel(corners, pattern, square)
    try:
        mount = tf_buffer.lookup_transform(truth.child_frame_id, info.header.frame_id, rclpy_time_zero())
    except Exception:  # noqa: BLE001 - no TF yet
        return None
    q = truth.pose.pose.orientation
    base_rotation = quaternion_matrix(q.w, q.x, q.y, q.z)
    base_position = np.array([truth.pose.pose.position.x, truth.pose.pose.position.y, truth.pose.pose.position.z])
    r = mount.transform.rotation
    t = mount.transform.translation
    camera_rotation = base_rotation @ quaternion_matrix(r.w, r.x, r.y, r.z)    # optical axes in the world
    camera_position = base_position + base_rotation @ np.array([t.x, t.y, t.z])
    k = np.array(info.k).reshape(3, 3)
    best = None
    for name, centre in boards.items():
        predicted = project_point(k, camera_rotation.T, camera_position, centre)
        if predicted is None:
            continue
        offset = measured - predicted
        if best is None or np.hypot(*offset) < np.hypot(*best[1]):
            best = (name, offset)
    if best is None or np.hypot(*best[1]) > max_px:
        return None
    return best[0], float(best[1][0]), float(best[1][1])


def summarise_centres(views, info):
    if not views:
        return {'views': 0, 'note': 'no board centre matched within 30 px'}
    offsets = np.array([[du, dv] for _, du, dv in views])
    k = list(info.k)
    return {'views': len(views), 'boards': len({name for name, _, _ in views}),
            'du_px_mean': round(float(offsets[:, 0].mean()), 2), 'dv_px_mean': round(float(offsets[:, 1].mean()), 2),
            'du_px_std': round(float(offsets[:, 0].std()), 2), 'dv_px_std': round(float(offsets[:, 1].std()), 2),
            'implied_cx': round(k[2] + float(offsets[:, 0].mean()), 2),
            'implied_cy': round(k[5] + float(offsets[:, 1].mean()), 2)}


def quaternion_matrix(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rclpy_time_zero():
    from rclpy.time import Time
    return Time()


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--pattern', default='8x6')
    parser.add_argument('--square', type=float, default=0.04)
    parser.add_argument('--rate', type=float, default=0.15, help='turn rate, rad/s')
    parser.add_argument('--turns', type=float, default=2.0)
    parser.add_argument('--timeout', type=float, default=1800.0, help='wall seconds')
    parser.add_argument('--stop-and-shoot', type=float, default=0.0, metavar='DEG',
                        help='turn DEG, stop and settle before each view (0: turn continuously)')
    parser.add_argument('--settle', type=float, default=1.5, help='wall seconds to settle after each step')
    parser.add_argument('--world', default='', help='world file with the board poses (default: calibration_room)')
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
        state['truth'] = msg
    node.create_subscription(Image, '/oakd/rgb/image_raw', on_image, qos_profile_sensor_data)
    node.create_subscription(CameraInfo, '/oakd/rgb/camera_info', lambda m: state.__setitem__('info', m),
                             qos_profile_sensor_data)
    node.create_subscription(Odometry, '/odom/ground_truth', on_truth, qos_profile_sensor_data)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(CameraInfo, '/perception/calibrated_camera_info',
                             lambda m: state.__setitem__('calibrated', m), latched)
    command = node.create_publisher(Twist, '/cmd_vel', 10)
    import tf2_ros
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer, node)

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
    centre_views = []
    if args.stop_and_shoot > 0:
        boards = board_centres(args.world or default_world())
        report['stop_and_shoot_deg'] = args.stop_and_shoot
        step = math.radians(args.stop_and_shoot)
        while turned < 2 * math.pi * args.turns and time.monotonic() < deadline:
            moved = 0.0
            while moved < step and time.monotonic() < deadline:
                command.publish(twist)
                spin(0.05)
                delta = abs(math.atan2(math.sin(state['yaw'] - last_yaw), math.cos(state['yaw'] - last_yaw)))
                moved += delta; turned += delta
                last_yaw = state['yaw']
            command.publish(Twist())
            spin(args.settle)
            seen = state['images']
            while state['images'] == seen and time.monotonic() < deadline:
                spin(0.05)     # a frame rendered after the robot settled
            image = state['image']
            gray = decode_gray(image)
            outcomes.append(calibrator.add(gray))
            view = centre_view(gray, pattern, args.square, info, state.get('truth'), tf_buffer, boards)
            if view:
                centre_views.append(view)
        report['centre_check'] = summarise_centres(centre_views, info)
    while args.stop_and_shoot <= 0 and turned < 2 * math.pi * args.turns and time.monotonic() < deadline:
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
