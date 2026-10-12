"""Bridge a Gazebo Harmonic RGB-D sensor to ROS 2 Humble.

The PX4 flight runtime runs Gazebo Harmonic, whose transport the Fortress
ros_gz_bridge in Humble cannot read.  This process subscribes with the
Harmonic Python bindings and republishes ROS images and camera info.

Run it with the system interpreter in isolated mode (``python3 -s``): the
generated Harmonic messages need the distribution's protobuf, and a newer
per-user protobuf refuses them.  The runtime starts it that way.

Stamps are ROS wall time at receipt, like the flight runtime's other
relays (the flight graph does not use simulation time).
"""
import argparse
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

# gz.msgs PixelFormatType -> (ROS encoding, bytes per pixel)
_ENCODINGS = {3: ('rgb8', 3), 4: ('rgba8', 4), 5: ('bgra8', 4), 8: ('bgr8', 3),
              1: ('mono8', 1), 2: ('mono16', 2), 13: ('32FC1', 4)}


def ros_image(raw, frame_id, stamp):
    """sensor_msgs/Image from a gz.msgs.Image, or None for an unknown format."""
    encoding = _ENCODINGS.get(int(raw.pixel_format_type))
    if encoding is None or not raw.width or not raw.height:
        return None
    message = Image()
    message.header.stamp, message.header.frame_id = stamp, frame_id
    message.height, message.width = raw.height, raw.width
    message.encoding = encoding[0]
    message.is_bigendian = 0
    message.step = raw.step or raw.width*encoding[1]
    if len(raw.data) != message.step*message.height:
        return None
    message.data = raw.data
    return message


def ros_camera_info(raw, frame_id, stamp):
    """sensor_msgs/CameraInfo (plumb-bob, no distortion) from gz.msgs.CameraInfo."""
    message = CameraInfo()
    message.header.stamp, message.header.frame_id = stamp, frame_id
    message.height, message.width = raw.height, raw.width
    k = [float(v) for v in raw.intrinsics.k]
    if len(k) != 9:
        return None
    message.k = k
    projection = list(raw.projection.p) if raw.HasField('projection') else []
    message.p = [float(v) for v in projection] if len(projection) == 12 else \
        [k[0], 0.0, k[2], 0.0, 0.0, k[4], k[5], 0.0, 0.0, 0.0, 1.0, 0.0]
    message.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    message.distortion_model = 'plumb_bob'
    message.d = [0.0]*5
    return message


class GzCameraBridge(Node):
    def __init__(self, gz_topic, ros_prefix, frame_id):
        super().__init__('px4_gz_camera_bridge')
        from gz.msgs10.camera_info_pb2 import CameraInfo as GzCameraInfo
        from gz.msgs10.image_pb2 import Image as GzImage
        from gz.transport13 import Node as GzNode
        self.frame_id = frame_id
        self.rgb_pub = self.create_publisher(Image, ros_prefix+'/rgb/image_raw', 5)
        self.depth_pub = self.create_publisher(Image, ros_prefix+'/depth/image_raw', 5)
        self.info_pub = self.create_publisher(CameraInfo, ros_prefix+'/rgb/camera_info', 5)
        self.depth_info_pub = self.create_publisher(CameraInfo, ros_prefix+'/depth/camera_info', 5)
        self.lock = threading.Lock()
        self.counts = dict(rgb=0, depth=0, info=0, rejected=0)
        self.gz = GzNode()
        for suffix, kind, callback in (('/image', GzImage, self.on_rgb),
                                       ('/depth_image', GzImage, self.on_depth),
                                       ('/camera_info', GzCameraInfo, self.on_info)):
            if not self.gz.subscribe(kind, gz_topic+suffix, callback):
                raise RuntimeError('Cannot subscribe to Gazebo topic '+gz_topic+suffix)
        self.create_timer(10.0, self.report)
        self.get_logger().info('Bridging Gazebo %s/{image,depth_image,camera_info} to %s/...'
                               % (gz_topic, ros_prefix))

    def _publish(self, publisher, message, key):
        with self.lock:
            if message is None:
                self.counts['rejected'] += 1
                return
            self.counts[key] += 1
        publisher.publish(message)

    def on_rgb(self, raw):
        self._publish(self.rgb_pub, ros_image(raw, self.frame_id, self.get_clock().now().to_msg()), 'rgb')

    def on_depth(self, raw):
        self._publish(self.depth_pub, ros_image(raw, self.frame_id, self.get_clock().now().to_msg()), 'depth')

    def on_info(self, raw):
        info = ros_camera_info(raw, self.frame_id, self.get_clock().now().to_msg())
        self._publish(self.info_pub, info, 'info')
        if info is not None:
            self.depth_info_pub.publish(info)

    def report(self):
        with self.lock:
            counts = dict(self.counts)
        self.get_logger().info('camera bridge counts: %s' % counts)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gz-topic', required=True)
    parser.add_argument('--ros-prefix', default='/x500/camera')
    parser.add_argument('--frame-id', default='camera_optical_frame')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = GzCameraBridge(args.gz_topic, args.ros_prefix, args.frame_id)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
