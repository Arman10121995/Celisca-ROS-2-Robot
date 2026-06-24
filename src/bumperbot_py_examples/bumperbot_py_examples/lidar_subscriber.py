import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import pygame
import math
import numpy as np

class SimpleSubscriber(Node):

    def __init__(self):
        super().__init__("simple_subscriber")
        self.sub_ = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.sub_

    def scan_callback(self, msg):
        self.get_logger().info("I heard: %s" % msg.ranges)
        self.get_logger().info(f"First value: {msg.ranges[0]}")
        raise SystemExit


def main():
    rclpy.init()
    node = SimpleSubscriber()

    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        node.get_logger().info("Shutting down after receiving first scan.")

    # Clean up (crucial for finishing the process)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()