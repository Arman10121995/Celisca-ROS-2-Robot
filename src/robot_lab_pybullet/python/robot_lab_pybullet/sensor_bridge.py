"""PyBullet sensor bridge node."""
import rclpy
from rclpy.node import Node


class PyBulletSensorBridge(Node):
    """Bridge PyBullet sensor data to ROS 2 standard message types."""

    def __init__(self):
        super().__init__("pybullet_sensor_bridge")
        self.declare_parameter("robot_name", "bumperbot")
        self._robot_name = self.get_parameter("robot_name").value
        self.get_logger().info(
            "PyBullet sensor bridge started for robot '{}'.".format(self._robot_name)
        )
        self.get_logger().warn(
            "PyBullet sensor bridge is a stub. Connect to a running "
            "PyBullet instance for sensor data."
        )


def main(args=None):
    rclpy.init(args=args)
    node = PyBulletSensorBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
