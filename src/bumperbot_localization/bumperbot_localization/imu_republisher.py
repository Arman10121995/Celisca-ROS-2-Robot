#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import Imu

imu_pub = None

def imuCallback(imu):
    global imu_pub
    imu.header.frame_id = "imu_link"

    if all(value == 0.0 for value in imu.angular_velocity_covariance):
        imu.angular_velocity_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01,
        ]

    if all(value == 0.0 for value in imu.linear_acceleration_covariance):
        imu.linear_acceleration_covariance = [
            0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.1,
        ]

    imu_pub.publish(imu)


def main(args=None):
    global imu_pub
    rclpy.init(args=args)
    node = Node('imu_republisher_node')
    time.sleep(1)
    imu_pub = node.create_publisher(Imu, "imu_ekf", 10)
    imu_sub = node.create_subscription(Imu, "imu/out", imuCallback, 10)
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
