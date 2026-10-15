#!/usr/bin/env python3
"""R7.2 camera_calibration: calibrate a Gazebo camera from a rendered chessboard.

Runs against a launched Gazebo robot (``scripts/r7_2_calibration_check.sh``
starts one with ``perception:=camera_calibration``).  The script:

1. draws a chessboard (``--pattern`` inner corners, ``--square`` metres,
   a one-square white margin) into a PNG and spawns it as a floating,
   self-lit plane (no gravity, no collision);
2. moves the board, with ``set_pose``, to random poses in front of the
   robot's camera (distance, offset and tilt drawn from a seeded RNG);
3. after each move waits for a camera image rendered with the board in
   place, and feeds it to robot_lab_algorithms.camera_calibration's
   ChessboardCalibrator;
4. reads the camera_calibration node's latched result on
   /perception/calibrated_camera_info, which collected its own views from
   the same stream.

Both calibrations are compared with the camera info and with a pinhole
whose principal point sits at the pixel centres, ((w - 1) / 2, (h - 1) / 2).

  python3 scripts/r7_2_calibration_check.py --world nav_empty --out <file.json>
"""
import argparse
import json
import math
import os
import subprocess
import sys
import tempfile
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src', 'robot_lab_algorithms'))

from robot_lab_algorithms.camera_calibration import ChessboardCalibrator, parse_pattern  # noqa: E402
from robot_lab_algorithms.visual_odometry import (  # noqa: E402
    decode_gray, matrix_quaternion, transform_matrix)


def board_png(path, pattern, pixels=40):
    """Square texture: a chessboard of (columns + 1) x (rows + 1) squares
    centred in white.  Square, so that however the renderer orients the
    texture on the square plane, the squares stay square (the first check
    stretched a 0.55 x 0.45 m board onto a transposed plane)."""
    import cv2
    columns, rows = pattern
    side = max(columns, rows) + 3   # the board plus at least one white square each side
    image = np.full((side * pixels, side * pixels), 255, np.uint8)
    left, top = (side - columns - 1) // 2, (side - rows - 1) // 2
    for j in range(rows + 1):
        for i in range(columns + 1):
            if (i + j) % 2 == 0:
                image[(top + j) * pixels:(top + j + 1) * pixels, (left + i) * pixels:(left + i + 1) * pixels] = 0
    cv2.imwrite(path, image)
    return side, side


def board_sdf(texture, width, height):
    return f"""<?xml version="1.0"?>
<sdf version="1.8">
  <model name="calibration_board">
    <static>false</static>
    <link name="link">
      <gravity>false</gravity>
      <inertial><mass>0.1</mass></inertial>
      <visual name="visual">
        <geometry><plane><normal>1 0 0</normal><size>{width} {height}</size></plane></geometry>
        <material>
          <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular>
          <pbr><metal>
            <albedo_map>{texture}</albedo_map>
            <emissive_map>{texture}</emissive_map>
            <roughness>1.0</roughness><metalness>0.0</metalness>
          </metal></pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def rotation(axis, angle):
    c, s = math.cos(angle), math.sin(angle)
    x, y, z = axis
    return np.array([[c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s],
                     [y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s],
                     [z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c)]])


def board_pose_in_camera(rng, fx, fy, cx, cy, width, height):
    """Board pose in the camera's optical frame: its visible +x face toward
    the camera, 0.6-1.0 m away, its centre on a random pixel of the middle
    60% of the image, tilted up to 30 deg."""
    distance = rng.uniform(0.6, 1.0)
    u = rng.uniform(0.2, 0.8) * width
    v = rng.uniform(0.2, 0.8) * height
    centre = np.array([(u - cx) * distance / fx, (v - cy) * distance / fy, distance])
    # Board x (its normal) toward the camera, board y along -x_camera, board z along +y_camera.
    base = np.column_stack([(0, 0, -1), (-1, 0, 0), (0, 1, 0)]).astype(float)
    tilt = (rotation((0, 1, 0), math.radians(rng.uniform(-30, 30)))
            @ rotation((0, 0, 1), math.radians(rng.uniform(-30, 30)))
            @ rotation((1, 0, 0), math.radians(rng.uniform(-20, 20))))
    pose = np.eye(4)
    pose[:3, :3] = base @ tilt
    pose[:3, 3] = centre
    return pose


def ign(*args, timeout=15):
    return subprocess.run(['ign', *args], capture_output=True, text=True, timeout=timeout)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--world', default='nav_empty', help='Gazebo world name')
    parser.add_argument('--pattern', default='8x6')
    parser.add_argument('--square', type=float, default=0.035,
                        help='square size of the spawned board (the intrinsics do not depend on it)')
    parser.add_argument('--poses', type=int, default=100)
    parser.add_argument('--seed', type=int, default=7)
    parser.add_argument('--camera-frame', default='oakd_rgb_camera_optical_frame')
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    pattern = parse_pattern(args.pattern)

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    node = Node('r7_2_calibration_check', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    state = {'image': None, 'info': None, 'truth': None, 'calibrated': None}
    node.create_subscription(Image, '/oakd/rgb/image_raw', lambda m: state.__setitem__('image', m),
                             qos_profile_sensor_data)
    node.create_subscription(CameraInfo, '/oakd/rgb/camera_info', lambda m: state.__setitem__('info', m),
                             qos_profile_sensor_data)
    node.create_subscription(Odometry, '/odom/ground_truth', lambda m: state.__setitem__('truth', m),
                             qos_profile_sensor_data)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(CameraInfo, '/perception/calibrated_camera_info',
                             lambda m: state.__setitem__('calibrated', m), latched)
    tf = Buffer()
    TransformListener(tf, node)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    report = {'world': args.world, 'pattern': args.pattern, 'square_m': args.square, 'seed': args.seed}
    deadline = time.monotonic() + 300
    base_from_camera = None
    while time.monotonic() < deadline:
        spin(0.5)
        if state['image'] is None or state['info'] is None or state['truth'] is None:
            continue
        try:
            t = tf.lookup_transform('base_footprint', args.camera_frame, Time())
        except Exception:
            continue
        q, p = t.transform.rotation, t.transform.translation
        base_from_camera = transform_matrix((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
        break
    if base_from_camera is None:
        report['error'] = 'no camera image, camera info, truth or camera transform'
        print(json.dumps(report))
        return 1
    info = state['info']
    width, height = info.width, info.height

    folder = tempfile.mkdtemp(prefix='calibration_board_', dir=os.environ.get('ROBOT_LAB_RUNTIME_ROOT'))
    texture = os.path.join(folder, 'board.png')
    squares_x, squares_y = board_png(texture, pattern)
    sdf = os.path.join(folder, 'board.sdf')
    with open(sdf, 'w') as handle:
        handle.write(board_sdf(texture, squares_x * args.square, squares_y * args.square))
    created = subprocess.run(['ros2', 'run', 'ros_gz_sim', 'create', '-world', args.world, '-file', sdf,
                              '-name', 'calibration_board', '-z', '-5'], capture_output=True, text=True, timeout=60)
    report['spawn'] = created.returncode
    spin(2.0)

    truth = state['truth'].pose.pose
    world_from_base = transform_matrix((truth.position.x, truth.position.y, truth.position.z),
                                       (truth.orientation.x, truth.orientation.y, truth.orientation.z,
                                        truth.orientation.w))
    world_from_camera = world_from_base @ base_from_camera
    rng = np.random.default_rng(args.seed)
    calibrator = ChessboardCalibrator(pattern, args.square, min_views=15, max_views=100, min_difference=0.05)
    outcomes = []
    k_info = list(info.k)
    misses = 0
    for _ in range(args.poses):
        pose = world_from_camera @ board_pose_in_camera(rng, k_info[0], k_info[4], k_info[2], k_info[5], width, height)
        x, y, z = pose[:3, 3]
        qx, qy, qz, qw = matrix_quaternion(pose[:3, :3])
        request = ('name: "calibration_board", position: {x: %.5f, y: %.5f, z: %.5f}, '
                   'orientation: {x: %.6f, y: %.6f, z: %.6f, w: %.6f}' % (x, y, z, qx, qy, qz, qw))
        moved = ign('service', '-s', '/world/%s/set_pose' % args.world, '--reqtype', 'ignition.msgs.Pose',
                    '--reptype', 'ignition.msgs.Boolean', '--timeout', '3000', '--req', request)
        if 'data: true' not in moved.stdout:
            outcomes.append('set_pose failed')
            continue
        # An image stamped at least 0.3 s of sim time after the move.
        moved_at = node.get_clock().now().nanoseconds * 1e-9
        image = None
        wait_end = time.monotonic() + 20
        while time.monotonic() < wait_end:
            spin(0.1)
            m = state['image']
            if m is not None and m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 >= moved_at + 0.3:
                image = m
                break
        if image is None:
            outcomes.append('no image')
            continue
        gray = decode_gray(image)
        outcome = calibrator.add(gray)
        outcomes.append(outcome)
        if outcome == 'no board' and misses < 4:
            import cv2
            cv2.imwrite(os.path.splitext(args.out)[0] + '_no_board_%d.png' % misses, gray)
            misses += 1
        spin(0.6)   # the node samples at 2 Hz of sim time
    spin(3.0)
    report['outcomes'] = {k: outcomes.count(k) for k in sorted(set(outcomes))}
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
    ign('service', '-s', '/world/%s/remove' % args.world, '--reqtype', 'ignition.msgs.Entity',
        '--reptype', 'ignition.msgs.Boolean', '--timeout', '3000', '--req', 'name: "calibration_board", type: MODEL')
    node.destroy_node()
    rclpy.shutdown()
    with open(args.out, 'w') as handle:
        json.dump(report, handle, indent=1)
    print(json.dumps(report, indent=1))
    return 0 if 'script_calibration_pinhole' in report else 1


if __name__ == '__main__':
    sys.exit(main())
