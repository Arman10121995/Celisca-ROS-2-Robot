"""Humanoid Standing Controller node (P3.4).

Publishes a nominal standing-posture position command (Float64MultiArray) to a
forward_command_controller command topic at a fixed rate, sized to a humanoid's
joint set. This is the minimal commandable control interface for the Berkeley
Humanoid Lite simulated profile, and is reusable for other humanoid robots
(h1/g1) that use the same position-forward_command_controller wiring.

Subscribes to /joint_states (best-effort) to learn the live joint count so the
command vector always matches the controller's joint ordering.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class HumanoidStandingController(Node):
    """Publish a standing-pose position command for a humanoid's joints."""

    def __init__(self):
        super().__init__('humanoid_standing_controller')
        self.declare_parameter('command_topic', '/bhl_standing_controller/commands')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('command_rate_hz', 50.0)
        self.declare_parameter('default_num_joints', 22)
        # Nominal standing pose; empty means "all zeros" (upright reference pose).
        self.declare_parameter('standing_pose', [])

        command_topic = self.get_parameter('command_topic').value
        joint_states_topic = self.get_parameter('joint_states_topic').value
        command_rate = float(self.get_parameter('command_rate_hz').value)
        self._num_joints = int(self.get_parameter('default_num_joints').value)
        self._standing_pose = list(self.get_parameter('standing_pose').value)
        if len(self._standing_pose) < self._num_joints:
            self._standing_pose += [0.0] * (self._num_joints - len(self._standing_pose))
        self._standing_pose = self._standing_pose[: self._num_joints]

        self._last_num_joints = 0
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._joint_state_sub = self.create_subscription(
            JointState, joint_states_topic, self._on_joint_state, qos
        )
        self._command_pub = self.create_publisher(Float64MultiArray, command_topic, 10)
        self._timer = self.create_timer(1.0 / command_rate, self._publish_command)
        self.get_logger().info(
            f"HumanoidStandingController: {joint_states_topic} -> {command_topic} "
            f"at {command_rate} Hz over {self._num_joints} joints"
        )

    def _on_joint_state(self, msg):
        """Track the live joint count so the vector matches the observed state."""
        self._last_num_joints = len(msg.name)

    def _publish_command(self):
        n = self._last_num_joints if self._last_num_joints > 0 else self._num_joints
        msg = Float64MultiArray()
        msg.data = self._standing_pose[:n]
        self._command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HumanoidStandingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()