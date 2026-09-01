"""MuJoCo sensor bridge node."""
import rclpy
from rclpy.node import Node


class MuJoCoSensorBridge(Node):
    """Bridge MuJoCo sensor data to ROS 2 standard message types."""

    def __init__(self):
        super().__init__("mujoco_sensor_bridge")
        self.declare_parameter("robot_name", "bumperbot")
        self._robot_name = self.get_parameter("robot_name").value
        self.get_logger().info(
            "MuJoCo sensor bridge started for robot '{}'.".format(self._robot_name)
        )
        self.get_logger().warn(
            "MuJoCo sensor bridge is a stub. Connect to a running "
            "MuJoCo instance for sensor data."
        )


def main(args=None):
    rclpy.init(args=args)
    node = MuJoCoSensorBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
