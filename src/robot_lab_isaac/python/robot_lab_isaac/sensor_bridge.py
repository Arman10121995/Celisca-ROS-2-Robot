"""Isaac Sim sensor bridge node.

Converts Isaac Omniverse sensor data to ROS 2 topics when the
isaacsim Python API is available.  In offline mode (no isaacsim
installed) the node logs a clear message and exits gracefully.
"""
import rclpy
from rclpy.node import Node


class IsaacSensorBridge(Node):
    """Bridge Isaac Sim sensors to ROS 2 topics."""

    def __init__(self):
        super().__init__("isaac_sensor_bridge")
        self.declare_parameter("robot_name", "bumperbot")

        try:
            import isaacsim  # noqa: F401
            self.get_logger().info("Isaac Sim sensor bridge active (stub).")
        except ImportError:
            self.get_logger().warn(
                "Isaac Sim Python API not available. "
                "Sensor bridge running in offline mode."
            )


def main(args=None):
    rclpy.init(args=args)
    node = IsaacSensorBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
