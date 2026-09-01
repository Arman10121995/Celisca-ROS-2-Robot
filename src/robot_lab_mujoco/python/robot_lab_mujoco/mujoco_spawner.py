"""MuJoCo robot spawner node."""
import os
import rclpy
from rclpy.node import Node


class MuJoCoSpawner(Node):
    """Spawn robot into a MuJoCo simulation."""

    def __init__(self):
        super().__init__("mujoco_spawner")
        self.declare_parameter("world_xml", "")
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
            import mujoco  # noqa: F401
            self.get_logger().info("MuJoCo API available (stub).")
        except ImportError:
            self.get_logger().warn(
                "mujoco package not installed. Running in offline mode. "
                "Install mujoco: pip install mujoco"
            )
        self._spawned = True
        self.get_logger().info(
            "Spawn request: name={}, xml={}, x={}, y={}, z={}".format(
                self.get_parameter("robot_name").value,
                self.get_parameter("world_xml").value,
                self.get_parameter("spawn_x").value,
                self.get_parameter("spawn_y").value,
                self.get_parameter("spawn_z").value,
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = MuJoCoSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
