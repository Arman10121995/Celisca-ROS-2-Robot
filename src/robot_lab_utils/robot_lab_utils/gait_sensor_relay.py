#!/usr/bin/env python3
"""Actual lidar samples with fixed source extrinsics on the estimated body."""
import copy
import math
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformBroadcaster, TransformException, TransformListener
from scipy.spatial.transform import Rotation
from robot_lab_utils.marine_camera_relay import fixed_extrinsic


class Relay(Node):
    def __init__(self):
        super().__init__('gait_estimated_lidar')
        self.declare_parameter('description_file','')
        self.declare_parameter('virtual_drive_scan',False)
        self.declare_parameter('base_height',0.)
        self.virtual_drive=self.get_parameter('virtual_drive_scan').value
        self.pending=None
        for key in ('visual_spawn_x','visual_spawn_y','visual_spawn_yaw'):self.declare_parameter(key,0.)
        source=Path(self.get_parameter('description_file').value).read_text()
        matrix=fixed_extrinsic(source,'gait_laser_link')
        transform=TransformStamped();transform.header.frame_id='gait/base_link';transform.child_frame_id='gait/laser_link'
        q=Rotation.from_matrix(matrix[:3,:3]).as_quat()
        for i,key in enumerate('xyz'):setattr(transform.transform.translation,key,float(matrix[i,3]))
        for i,key in enumerate('xyzw'):setattr(transform.transform.rotation,key,float(q[i]))
        self.tf=StaticTransformBroadcaster(self);transforms=[transform]
        if self.virtual_drive:
            if max(abs(float(matrix[i,3])) for i in (0,1))>1e-6:
                raise ValueError('Virtual drive scan requires a head-centred laser; nonzero XY needs point-cloud reprojection')
            virtual=TransformStamped();virtual.header.frame_id='gait/base_footprint';virtual.child_frame_id='gait/laser_drive_frame'
            virtual.transform.translation.z=float(self.get_parameter('base_height').value)+float(matrix[2,3])
            virtual.transform.rotation.w=1.;transforms.append(virtual)
        self.tf.sendTransform(transforms)
        self.pub=self.create_publisher(LaserScan,'/gait/scan',10)
        self.create_subscription(LaserScan,'/scan',self.observe,qos_profile_sensor_data)
        self.buffer=Buffer(node=self);self.listener=TransformListener(self.buffer,self)
        self.visual=TransformBroadcaster(self)
        self.create_timer(.1,self.visual_alignment)
        if self.virtual_drive:self.create_timer(.02,self.flush_pending)

    def visual_alignment(self):
        # Follow SLAM/AMCL's map correction on the separate truth-only display
        # branch. No measured body pose is used to correct estimated odometry.
        try:frame=self.buffer.lookup_transform('map','gait/odom',rclpy.time.Time())
        except TransformException:return
        x=float(self.get_parameter('visual_spawn_x').value);y=float(self.get_parameter('visual_spawn_y').value)
        heading=float(self.get_parameter('visual_spawn_yaw').value)
        q=frame.transform.rotation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        ix,iy=-math.cos(heading)*x-math.sin(heading)*y,math.sin(heading)*x-math.cos(heading)*y
        output=TransformStamped();output.header.frame_id='map';output.child_frame_id='odom'
        output.header.stamp=self.get_clock().now().to_msg()
        output.transform.translation.x=frame.transform.translation.x+math.cos(yaw)*ix-math.sin(yaw)*iy
        output.transform.translation.y=frame.transform.translation.y+math.sin(yaw)*ix+math.cos(yaw)*iy
        output.transform.rotation.z=math.sin((yaw-heading)/2);output.transform.rotation.w=math.cos((yaw-heading)/2)
        self.visual.sendTransform(output)

    def observe(self,msg):
        if msg.header.frame_id!='gait_laser_link':return
        if self.virtual_drive:
            self.pending=(msg,time.monotonic());self.flush_pending();return
        output=copy.copy(msg);output.header=copy.copy(msg.header);output.header.frame_id='gait/laser_link'
        self.pub.publish(output)

    def flush_pending(self):
        if self.pending is None:return
        msg,received=self.pending
        if time.monotonic()-received>.2:self.pending=None;return
        try:
            transform=self.buffer.lookup_transform('gait/base_footprint','gait/laser_link',rclpy.time.Time.from_msg(msg.header.stamp)).transform
        except TransformException:return
        q=transform.rotation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        output=copy.copy(msg);output.header=copy.copy(msg.header);output.header.frame_id='gait/laser_drive_frame'
        # The laser origin has zero head-plane XY offset. Rotate measured ray
        # angles into a fixed virtual mounting frame, retaining every range.
        # AMCL caches its mounting transform, so a moving head TF alone is
        # insufficient even though RViz can display that dynamic frame.
        output.angle_min+=yaw;output.angle_max+=yaw
        self.pub.publish(output);self.pending=None


def main(args=None):
    from robot_lab_utils.process_lifetime import exit_with_parent
    exit_with_parent();rclpy.init(args=args);node=Relay()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
