"""Isaac Sim sensor bridge node.

Converts Omniverse sensor data (LiDAR, camera, IMU) into ROS 2
standard message types so that downstream nodes (localization,
mapping, navigation) receive consistent topics regardless of the
underlying simulator.
"""
import rclpy
from rclpy.node import Node


class IsaacSensorBridge(Node):
    """Bridge Isaac Sim sensor topics to ROS 2."""

    def __init__(self):
        super().__init__("isaac_sensor_bridge")
        self.declare_parameter("robot_name", "bumperbot")
        self._robot_name = self.get_parameter("robot_name").value
        self.get_logger().info(
            f"Isaac sensor bridge started for robot '{self._robot_name}'."
        )
        self.get_logger().warn(
            "Isaac Sim sensor bridge is a stub. Full bridge logic "
            "will be connected when running inside an active "
            "Omniverse Kit session."
        )


def main(args=None):
    rclpy.init(args=args)
    node = IsaacSensorBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
