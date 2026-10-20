#!/usr/bin/env python3
"""Actual lidar samples with fixed source extrinsics on the estimated body."""
import copy
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from scipy.spatial.transform import Rotation
from robot_lab_utils.marine_camera_relay import fixed_extrinsic


class Relay(Node):
    def __init__(self):
        super().__init__('gait_estimated_lidar')
        self.declare_parameter('description_file','')
        source=Path(self.get_parameter('description_file').value).read_text()
        matrix=fixed_extrinsic(source,'gait_laser_link')
        transform=TransformStamped();transform.header.frame_id='gait/base_link';transform.child_frame_id='gait/laser_link'
        q=Rotation.from_matrix(matrix[:3,:3]).as_quat()
        for i,key in enumerate('xyz'):setattr(transform.transform.translation,key,float(matrix[i,3]))
        for i,key in enumerate('xyzw'):setattr(transform.transform.rotation,key,float(q[i]))
        self.tf=StaticTransformBroadcaster(self);self.tf.sendTransform(transform)
        self.pub=self.create_publisher(LaserScan,'/gait/scan',10)
        self.create_subscription(LaserScan,'/scan',self.observe,qos_profile_sensor_data)

    def observe(self,msg):
        if msg.header.frame_id!='gait_laser_link':return
        output=copy.copy(msg);output.header=copy.copy(msg.header);output.header.frame_id='gait/laser_link'
        self.pub.publish(output)


def main(args=None):
    from robot_lab_utils.process_lifetime import exit_with_parent
    exit_with_parent();rclpy.init(args=args);node=Relay()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
