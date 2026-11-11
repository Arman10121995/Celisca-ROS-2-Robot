#!/usr/bin/env python3
"""ROS 2 node: learned floor-object detector for the robot-vacuum camera.

Subscribes to the front-camera RGB/depth/camera_info and TF, runs the learned
torchvision detector, projects detections onto the floor in the map frame and
publishes them on ``/vacuum/objects`` with ``detector: 'learned'`` and
``ideal: false`` -- the same message shape as the ideal segmentation detector
in ``vacuum_objects.py``, so the cleaning mission can consume either.

Interfaces
  sub  /vacuum/camera/image        sensor_msgs/Image, rgb8
  sub  /vacuum/camera/depth        sensor_msgs/Image, 32FC1 metres
  sub  /vacuum/camera/camera_info  sensor_msgs/CameraInfo
  pub  /vacuum/objects             std_msgs/String JSON
"""
import json
import time
from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from .learned_detector import LearnedDetector, LABELS, CLASS_RADIUS, project_detections
from .vacuum_objects import _image_array, _matrix


class LearnedObjectDetector(Node):
    def __init__(self):
        super().__init__('vacuum_learned_object_detector')
        p = self.declare_parameter
        p('model_path', '')
        p('score_thresh', 0.4)
        p('labels_topic', '/vacuum/camera/labels')   # kept for interface parity
        p('depth_topic', '/vacuum/camera/depth')
        p('camera_info_topic', '/vacuum/camera/camera_info')
        p('map_frame', 'map')
        p('camera_frame', '')      # overrides the image frame_id when set
        p('labels', json.dumps({str(k): v for k, v in LABELS.items()}))
        p('rate_hz', 2.0)
        gp = lambda name: self.get_parameter(name).value
        self.classes = {int(k): v for k, v in json.loads(gp('labels')).items()}
        model_path = gp('model_path')
        self.detector = LearnedDetector(model_path, score_thresh=float(gp('score_thresh')))
        if not self.detector.ok:
            self.get_logger().warn('no learned detector checkpoint at %r; publishing no objects' % model_path)
        self.buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.listener = TransformListener(self.buffer, self)
        self.info = self.depth = None
        self.last = 0.0
        self.period = 1.0 / max(0.1, float(gp('rate_hz')))
        self.pub = self.create_publisher(String, '/vacuum/objects', 10)
        self.create_subscription(CameraInfo, gp('camera_info_topic'), self._info, qos_profile_sensor_data)
        self.create_subscription(Image, gp('depth_topic'), self._depth, qos_profile_sensor_data)
        self.create_subscription(Image, '/vacuum/camera/image', self._image, qos_profile_sensor_data)
        self.get_logger().info('Learned object detector (checkpoint %s, score_thresh %.2f)' %
                               (model_path, float(gp('score_thresh'))))

    def _info(self, msg):
        self.info = msg

    def _depth(self, msg):
        self.depth = msg

    def _image(self, msg):
        now = time.monotonic()
        if now - self.last < self.period:
            return
        self.last = now
        if self.info is None or self.depth is None:
            return
        rgb = _image_array(msg)
        depth = _image_array(self.depth)
        if rgb is None or depth is None:
            return
        frame = self.get_parameter('camera_frame').value or self.depth.header.frame_id or \
            self.info.header.frame_id
        try:
            tf = self.buffer.lookup_transform(self.get_parameter('map_frame').value, frame,
                                              Time.from_msg(self.depth.header.stamp),
                                              timeout=Duration(seconds=0.05))
        except Exception:
            return
        detections = self.detector.detect(rgb)
        objects = project_detections(detections, depth, list(self.info.k), _matrix(tf),
                                     radius_override=CLASS_RADIUS)
        stamp = self.depth.header.stamp
        self.pub.publish(String(data=json.dumps({
            'detector': 'learned',
            'ideal': False,
            't': stamp.sec + stamp.nanosec * 1e-9,
            'objects': objects,
        })))

    def destroy_node(self):
        try:
            self.pub.publish(String(data=json.dumps({'detector': 'learned', 'ideal': False, 't': 0.0, 'objects': []})))
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LearnedObjectDetector()
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
