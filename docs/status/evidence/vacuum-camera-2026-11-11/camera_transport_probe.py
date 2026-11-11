#!/usr/bin/env python3
"""Actual installed ROS transport diagnostic with explicitly synthetic frames.

No robot, world, physics or learned checkpoint: this is not mission evidence.
Run with an unused ROS domain after sourcing the rebuilt workspace.
"""
import argparse
import base64
from io import BytesIO
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
from PIL import Image as PILImage
import rclpy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--capture', action='store_true', help='Also check the bounded dataset capture tool')
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    result = {'scope': 'installed ROS transport using synthetic RGB/depth/semantic images; no robot mission',
              'domain': os.environ.get('ROS_DOMAIN_ID'), 'passed': False}
    rclpy.init()
    node = rclpy.create_node('camera_receipt_probe')
    received = []
    sub = node.create_subscription(String, '/vacuum/objects', lambda m: received.append(json.loads(m.data)), 10)
    publishers = {key: node.create_publisher(kind, '/vacuum/camera/' + key, 10)
                  for key, kind in [('image', Image), ('depth', Image), ('labels', Image),
                                    ('camera_info', CameraInfo)]}
    broadcaster = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.frame_id = 'map'
    transform.child_frame_id = 'front_camera_optical_frame'
    transform.transform.rotation.w = 1.
    broadcaster.sendTransform(transform)
    start = time.monotonic()
    with (args.out / 'detector.log').open('w') as log:
        process = subprocess.Popen(['ros2', 'run', 'robot_lab_vacuum_cleaning', 'vacuum_object_detector'],
                                   stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        capture_process = None
        capture_log = None
        if args.capture:
            capture_log = (args.out / 'capture.log').open('w')
            capture_process = subprocess.Popen([
                '/usr/bin/python3', '-s', 'scripts/capture_vacuum_detector_data.py',
                '--out', str(args.out / 'synthetic-transport-dataset'), '--scene', 'synthetic-transport-only',
                '--split', 'train', '--frames', '2', '--interval', '.2', '--timeout', '12'],
                stdout=capture_log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = start + 20.
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError('Installed detector exited: %s' % process.returncode)
                stamp = node.get_clock().now().to_msg()
                header = TransformStamped().header
                header.stamp, header.frame_id = stamp, 'front_camera_optical_frame'
                rgb = np.zeros((24, 32, 3), dtype=np.uint8)
                rgb[:, :, 0] = 220
                labels = np.zeros((24, 32), dtype=np.uint8)
                labels[8:16, 12:20] = 22
                for key, encoding, array in [('image', 'rgb8', rgb),
                                              ('depth', '32FC1', np.ones((24, 32), dtype='<f4')),
                                              ('labels', 'mono8', labels)]:
                    msg = Image(header=header, height=24, width=32, encoding=encoding,
                                is_bigendian=0, step=int(array.strides[0]), data=array.tobytes())
                    publishers[key].publish(msg)
                info = CameraInfo(header=header, width=32, height=24,
                                  k=[100., 0., 16., 0., 100., 12., 0., 0., 1.])
                publishers['camera_info'].publish(info)
                for _ in range(10):
                    rclpy.spin_once(node, timeout_sec=.02)
                matches = [o for m in received for o in m.get('objects', []) if o.get('snapshot')]
                if matches and (capture_process is None or capture_process.poll() is not None):
                    obj = matches[-1]
                    snap = obj['snapshot']
                    image = PILImage.open(BytesIO(base64.b64decode(snap['data'], validate=True)))
                    assert obj['class'] == 'shoe' and obj['bbox'] == [12, 8, 20, 16]
                    assert image.size == (24, 24) and image.getpixel((5, 5))[0] > 180
                    assert any(m['ideal'] is True and m['detector'] == 'segmentation' for m in received)
                    result.update(passed=True, messages=len(received), object={k: v for k, v in obj.items()
                                                                            if k != 'snapshot'},
                                  snapshot_size=list(image.size), snapshot_stamp=snap['t'])
                    if capture_process is not None:
                        assert capture_process.returncode == 0, 'Dataset capture failed'
                        dataset = json.loads((args.out / 'synthetic-transport-dataset/annotations.json').read_text())
                        assert len(dataset['images']) == 2 and len(dataset['annotations']) == 2
                        result['captured_synthetic_frames'] = len(dataset['images'])
                    break
            else:
                raise RuntimeError('No actual object snapshot received within 20 wall seconds')
        except Exception as exc:
            result['passed'] = False
            result['error'] = str(exc)
        finally:
            os.killpg(process.pid, signal.SIGINT) if process.poll() is None else None
            try:
                process.wait(timeout=6)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
            if capture_process is not None:
                if capture_process.poll() is None:
                    os.killpg(capture_process.pid, signal.SIGINT)
                    try:
                        capture_process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        os.killpg(capture_process.pid, signal.SIGKILL)
                        capture_process.wait(timeout=3)
                capture_log.close()
            result['wall_s'] = time.monotonic() - start
            (args.out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
            node.destroy_subscription(sub)
            node.destroy_node()
            rclpy.shutdown()
    print(json.dumps(result))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
