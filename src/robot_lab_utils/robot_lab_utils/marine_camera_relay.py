#!/usr/bin/env python3
"""Use actual RGB-D bytes with source-fixed estimated camera extrinsics.

No map -> simulator body transform is consulted: that would leak truth pose
into the independent SLAM graph. Source images/calibration/timestamps persist.
"""
import copy
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import StaticTransformBroadcaster
from scipy.spatial.transform import Rotation
from robot_lab_utils.marine_model import rotation


def fixed_extrinsic(description, child, parent='base_link'):
    root = ET.fromstring(description)
    transforms = {parent: np.eye(4)}
    pending = list(root.findall('joint'))
    while pending:
        progress = False
        for joint in list(pending):
            if joint.get('type') != 'fixed':
                pending.remove(joint)
                continue
            base, tip = joint.find('parent').get('link'), joint.find('child').get('link')
            if base not in transforms:
                continue
            origin = joint.find('origin')
            matrix = np.eye(4)
            if origin is not None:
                matrix[:3, :3] = rotation(tuple(map(float, origin.get('rpy', '0 0 0').split())))
                matrix[:3, 3] = tuple(map(float, origin.get('xyz', '0 0 0').split()))
            transforms[tip] = transforms[base] @ matrix
            pending.remove(joint)
            progress = True
        if not progress:
            break
    if child not in transforms:
        raise ValueError('Camera must have a complete fixed source chain from base_link: ' + child)
    return transforms[child]


class CameraRelay(Node):
    def __init__(self):
        super().__init__('marine_estimated_camera')
        self.declare_parameter('description_file', '')
        for key,value in dict(source_camera_frame='oakd_rgb_camera_optical_frame',estimated_base_frame='marine_estimated_base_link',
                              target_camera_frame='marine_estimated_camera_optical_frame',output_prefix='/marine/camera').items():
            self.declare_parameter(key,value)
        description = Path(self.get_parameter('description_file').value).read_text()
        self.source_frame = self.get_parameter('source_camera_frame').value
        transform = fixed_extrinsic(description, self.source_frame)
        self.frame = self.get_parameter('target_camera_frame').value
        message = TransformStamped()
        message.header.frame_id, message.child_frame_id = self.get_parameter('estimated_base_frame').value, self.frame
        message.header.stamp = self.get_clock().now().to_msg()
        q = Rotation.from_matrix(transform[:3, :3]).as_quat()
        for index, key in enumerate('xyz'):
            setattr(message.transform.translation, key, float(transform[index, 3]))
        for index, key in enumerate('xyzw'):
            setattr(message.transform.rotation, key, float(q[index]))
        self.tf = StaticTransformBroadcaster(self)
        self.tf.sendTransform(message)
        prefix=self.get_parameter('output_prefix').value
        for kind, source, target in [(Image, '/oakd/rgb/image_raw', prefix+'/rgb'),
                                     (Image, '/oakd/depth/image_raw', prefix+'/depth'),
                                     (CameraInfo, '/oakd/rgb/camera_info', prefix+'/info')]:
            pub = self.create_publisher(kind, target, 10)
            self.create_subscription(kind, source, lambda msg, p=pub: self.relay(msg, p), qos_profile_sensor_data)

    def relay(self, msg, publisher):
        if msg.header.frame_id != self.source_frame:
            return
        output = copy.copy(msg)
        output.header = copy.copy(msg.header)
        output.header.frame_id = self.frame
        publisher.publish(output)


def main(args=None):
    from robot_lab_utils.process_lifetime import exit_with_parent
    exit_with_parent(); rclpy.init(args=args)
    node = CameraRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
