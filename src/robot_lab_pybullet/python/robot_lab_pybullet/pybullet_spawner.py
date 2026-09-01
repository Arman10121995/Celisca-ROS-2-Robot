"""PyBullet robot spawner node."""
import rclpy
from rclpy.node import Node


class PyBulletSpawner(Node):
    """Spawn robot URDF into a PyBullet physics world."""

    def __init__(self):
        super().__init__("pybullet_spawner")
        self.declare_parameter("robot_name", "bumperbot")
        self.declare_parameter("robot_package", "robot_lab_robots")
        self.declare_parameter("robot_xacro", "")
        self.declare_parameter("spawn_x", 0.0)
        self.declare_parameter("spawn_y", 0.0)
        self.declare_parameter("spawn_z", 0.0)
        self.declare_parameter("spawn_yaw", 0.0)

        self._spawned = False
        self._timer = self.create_timer(0.5, self._try_spawn)

    def _try_spawn(self):
        if self._spawned:
            return
        try:
            import pybullet as p  # noqa: F401
            import pybullet_data  # noqa: F401
            self.get_logger().info("PyBullet API available (stub).")
        except ImportError:
            self.get_logger().warn(
                "PyBullet not installed. Running in offline mode — "
                "robot will not be spawned visually. "
                "Install pybullet: pip install pybullet"
            )
        self._spawned = True
        self.get_logger().info(
            "Spawn request: name={}, x={}, y={}, z={}".format(
                self.get_parameter("robot_name").value,
                self.get_parameter("spawn_x").value,
                self.get_parameter("spawn_y").value,
                self.get_parameter("spawn_z").value,
            )
        )

    def __del__(self):
        try:
            import pybullet as p
            p.disconnect()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = PyBulletSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
