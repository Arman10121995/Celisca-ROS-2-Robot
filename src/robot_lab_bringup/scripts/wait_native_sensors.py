#!/usr/bin/env python3
"""Gate native experiment stacks on actual sensor readiness, or fail bounded."""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool


def main():
    rclpy.init()
    node = Node('native_sensor_readiness_gate')
    ready = []
    node.create_subscription(Bool, '/robot_lab/native_sensors_ready', lambda msg: ready.append(msg.data),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    deadline = time.monotonic()+180
    while rclpy.ok() and time.monotonic() < deadline and not any(ready):
        rclpy.spin_once(node, timeout_sec=.2)
    success = any(ready)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    raise SystemExit(0 if success else 1)


if __name__ == '__main__':
    main()
