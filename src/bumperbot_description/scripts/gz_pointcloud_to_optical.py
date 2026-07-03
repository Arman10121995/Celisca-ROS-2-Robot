#!/usr/bin/env python3
import sys
import os
# Add user site-packages to path to find numpy
user_site_packages = os.path.expanduser('~/.local/lib/python3.10/site-packages')
if os.path.exists(user_site_packages) and user_site_packages not in sys.path:
    sys.path.insert(0, user_site_packages)

import struct

import numpy as np
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField


class GazeboPointCloudToOptical(Node):
    def __init__(self):
        super().__init__("gz_pointcloud_to_optical")
        self.declare_parameter("input_topic", "/oakd/points_gz")
        self.declare_parameter("output_topic", "/oakd/points")
        self.declare_parameter("frame_id", "oakd_rgb_camera_optical_frame")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.frame_id = self.get_parameter("frame_id").value

        self.publisher = self.create_publisher(PointCloud2, output_topic, 10)
        self.subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self.pointcloud_callback,
            10,
        )

    def pointcloud_callback(self, msg):
        field_offsets = {field.name: field.offset for field in msg.fields}
        if not {"x", "y", "z"}.issubset(field_offsets):
            self.get_logger().warn("PointCloud2 is missing x/y/z fields")
            return

        for axis in ("x", "y", "z"):
            field = next(field for field in msg.fields if field.name == axis)
            if field.datatype != PointField.FLOAT32 or field.count != 1:
                self.get_logger().warn("PointCloud2 x/y/z fields must be float32")
                return

        msg.header.frame_id = self.frame_id

        endian = ">" if msg.is_bigendian else "<"
        x_offset = field_offsets["x"]
        y_offset = field_offsets["y"]
        z_offset = field_offsets["z"]
        point_count = msg.width * msg.height

        if msg.row_step == msg.width * msg.point_step:
            cloud_dtype = np.dtype({
                "names": ["x", "y", "z"],
                "formats": [endian + "f4", endian + "f4", endian + "f4"],
                "offsets": [x_offset, y_offset, z_offset],
                "itemsize": msg.point_step,
            })
            points = np.ndarray(
                shape=(point_count,),
                dtype=cloud_dtype,
                buffer=msg.data,
            )
            gazebo_x = points["x"].copy()
            points["x"] = -points["y"]
            points["y"] = -points["z"]
            points["z"] = gazebo_x
            self.publish_pointcloud(msg)
            return

        float_format = endian + "f"
        for index in range(point_count):
            point_offset = index * msg.point_step
            gz_x = struct.unpack_from(float_format, msg.data, point_offset + x_offset)[0]
            gz_y = struct.unpack_from(float_format, msg.data, point_offset + y_offset)[0]
            gz_z = struct.unpack_from(float_format, msg.data, point_offset + z_offset)[0]

            struct.pack_into(float_format, msg.data, point_offset + x_offset, -gz_y)
            struct.pack_into(float_format, msg.data, point_offset + y_offset, -gz_z)
            struct.pack_into(float_format, msg.data, point_offset + z_offset, gz_x)

        self.publish_pointcloud(msg)

    def publish_pointcloud(self, msg):
        if not rclpy.ok():
            return

        try:
            self.publisher.publish(msg)
        except RCLError:
            if rclpy.ok():
                raise


def main():
    rclpy.init()
    node = GazeboPointCloudToOptical()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
