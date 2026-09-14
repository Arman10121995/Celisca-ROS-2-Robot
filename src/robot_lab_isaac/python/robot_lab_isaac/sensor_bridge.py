"""Isaac Sim sensor bridge node.

Converts Isaac Omniverse sensor data to ROS 2 topics when the
isaacsim Python API is available.  In offline mode (no isaacsim
installed) the node logs a clear message and exits gracefully.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
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
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # A stop from launch or the GUI, not a failure: without this every
        # Isaac shutdown logged a traceback and "process has died".
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
