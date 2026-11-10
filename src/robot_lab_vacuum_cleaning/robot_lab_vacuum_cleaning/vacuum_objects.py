"""Floor-object detector for the cleaning system (camera obstacle classes).

Commercial robot vacuums recognise cables, socks, shoes and pet mess with a
front camera and keep a class-specific distance from them.  This node turns
camera detections into floor objects in the map frame:

  sub  <labels_topic>       sensor_msgs/Image, per-pixel class label (mono8 or
                            rgb8 with the label in every channel)
  sub  <depth_topic>        sensor_msgs/Image, 32FC1 metres, pixel-aligned
  sub  <camera_info_topic>  sensor_msgs/CameraInfo
  pub  /vacuum/objects      std_msgs/String JSON {"detector", "objects": [...]}

The default detector is ``segmentation``: Gazebo's semantic segmentation
camera, whose labels come from the Label system on world models.  It is an
*ideal* detector (simulator ground truth for the class, not for the
position, which comes from depth and TF) and every message says so.  A
learned detector can publish the same labels image.
"""

import json
import time
from typing import Dict, List, Optional, Sequence

import numpy as np

DEFAULT_LABELS = {20: 'cable', 21: 'sock', 22: 'shoe', 23: 'pet_waste'}


def project_labels(labels: np.ndarray, depth: np.ndarray, k: Sequence[float], cam_to_map: np.ndarray,
                   classes: Dict[int, str], min_pixels: int = 12, max_range: float = 2.5,
                   cell: float = 0.1) -> List[Dict]:
    """Objects from a label image and its aligned depth image.

    ``k`` is the 3x3 intrinsic matrix (row-major, 9 values); ``cam_to_map``
    a 4x4 transform from the optical frame (z forward, x right, y down) to
    the map.  Pixels of one class are grouped by 2D floor cell (``cell``
    metres) and connected cells merge into one object: {class, x, y,
    radius, pixels}."""
    fx, cx, fy, cy = k[0], k[2], k[4], k[5]
    out: List[Dict] = []
    if labels.shape != depth.shape:
        return out
    h, w = labels.shape
    vs, us = np.mgrid[0:h, 0:w]
    for label, name in classes.items():
        mask = (labels == label) & np.isfinite(depth) & (depth > 0.05) & (depth < max_range)
        if int(mask.sum()) < min_pixels:
            continue
        z = depth[mask].astype(float)
        x = (us[mask] - cx) * z / fx
        y = (vs[mask] - cy) * z / fy
        pts = cam_to_map @ np.vstack([x, y, z, np.ones_like(z)])
        px, py = pts[0], pts[1]
        # Connected floor cells -> separate objects of the same class.
        keys = np.floor(np.column_stack([px, py]) / cell).astype(int)
        cells: Dict[tuple, List[int]] = {}
        for i, key in enumerate(map(tuple, keys)):
            cells.setdefault(key, []).append(i)
        seen = set()
        for start in cells:
            if start in seen:
                continue
            stack, members = [start], []
            seen.add(start)
            while stack:
                c = stack.pop()
                members.extend(cells[c])
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        n = (c[0] + dx, c[1] + dy)
                        if n in cells and n not in seen:
                            seen.add(n)
                            stack.append(n)
            if len(members) < min_pixels:
                continue
            mx, my = float(px[members].mean()), float(py[members].mean())
            radius = float(np.hypot(px[members] - mx, py[members] - my).max())
            out.append({'class': name, 'x': round(mx, 3), 'y': round(my, 3),
                        'radius': round(max(radius, 0.03), 3), 'pixels': len(members)})
    return out


def _image_array(msg) -> Optional[np.ndarray]:
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if msg.encoding in ('mono8', '8UC1'):
        return data.reshape(msg.height, msg.step)[:, :msg.width]
    if msg.encoding in ('rgb8', 'bgr8', '8UC3'):
        return data.reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)[:, :, 0]
    if msg.encoding == '32FC1':
        row = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(msg.height, msg.step // 4)
        return row[:, :msg.width]
    return None


def _matrix(tf) -> np.ndarray:
    t, q = tf.transform.translation, tf.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), t.x],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), t.y],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), t.z],
        [0.0, 0.0, 0.0, 1.0]])


def main(args=None):
    import rclpy
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener

    class ObjectDetector(Node):
        def __init__(self):
            super().__init__('vacuum_object_detector')
            p = self.declare_parameter
            p('detector', 'segmentation')
            p('labels_topic', '/vacuum/camera/labels')
            p('depth_topic', '/vacuum/camera/depth')
            p('camera_info_topic', '/vacuum/camera/camera_info')
            p('map_frame', 'map')
            p('camera_frame', '')     # overrides the image frame_id when set
            p('labels', json.dumps({str(k): v for k, v in DEFAULT_LABELS.items()}))
            p('rate_hz', 2.0)
            gp = lambda name: self.get_parameter(name).value  # noqa: E731
            self.classes = {int(k): v for k, v in json.loads(gp('labels')).items()}
            self.buffer = Buffer(cache_time=Duration(seconds=10.0))
            self.listener = TransformListener(self.buffer, self)
            self.info = self.depth = None
            self.last = 0.0
            self.period = 1.0 / max(0.1, float(gp('rate_hz')))
            self.pub = self.create_publisher(String, '/vacuum/objects', 10)
            self.create_subscription(CameraInfo, gp('camera_info_topic'), self._info, qos_profile_sensor_data)
            self.create_subscription(Image, gp('depth_topic'), self._depth, qos_profile_sensor_data)
            self.create_subscription(Image, gp('labels_topic'), self._labels, qos_profile_sensor_data)
            self.get_logger().info('Object detector (%s) for %s' % (gp('detector'), ', '.join(self.classes.values())))

        def _info(self, msg):
            self.info = msg

        def _depth(self, msg):
            self.depth = msg

        def _labels(self, msg):
            now = time.monotonic()
            if now - self.last < self.period or self.info is None or self.depth is None:
                return
            self.last = now
            labels, depth = _image_array(msg), _image_array(self.depth)
            if labels is None or depth is None:
                return
            frame = self.get_parameter('camera_frame').value or self.depth.header.frame_id or \
                self.info.header.frame_id
            try:
                tf = self.buffer.lookup_transform(self.get_parameter('map_frame').value, frame,
                                                  Time.from_msg(self.depth.header.stamp),
                                                  timeout=Duration(seconds=0.05))
            except Exception:
                return
            objects = project_labels(labels, depth, list(self.info.k), _matrix(tf), self.classes)
            stamp = self.depth.header.stamp
            self.pub.publish(String(data=json.dumps({
                'detector': self.get_parameter('detector').value,
                'ideal': self.get_parameter('detector').value == 'segmentation',
                't': stamp.sec + stamp.nanosec * 1e-9, 'objects': objects})))

    rclpy.init(args=args)
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
