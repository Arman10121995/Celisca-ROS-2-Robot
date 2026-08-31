"""Joint Effort Commander node for legged and humanoid robots (P3.3).

Subscribes to /joint_states and publishes Float64MultiArray effort targets
to a forward_command_controller command topic at a fixed rate. This is the
minimal commandable control interface for the Go2 simulated legged profile.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class JointEffortCommander(Node):
    """Publish group effort commands for a legged robot's joints."""

    def __init__(self):
        super().__init__('joint_effort_commander')
        self.declare_parameter('command_topic', '/go2_group_effort_controller/commands')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('command_rate_hz', 100.0)
        self.declare_parameter('default_effort', 0.0)

        command_topic = self.get_parameter('command_topic').value
        joint_states_topic = self.get_parameter('joint_states_topic').value
        command_rate = float(self.get_parameter('command_rate_hz').value)
        self._default_effort = float(self.get_parameter('default_effort').value)

        self._last_num_joints = 0
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._joint_state_sub = self.create_subscription(
            JointState, joint_states_topic, self._on_joint_state, qos
        )
        self._command_pub = self.create_publisher(Float64MultiArray, command_topic, 10)
        self._timer = self.create_timer(1.0 / command_rate, self._publish_command)
        self.get_logger().info(
            f"JointEffortCommander: {joint_states_topic} -> {command_topic} "
            f"at {command_rate} Hz"
        )

    def _on_joint_state(self, msg):
        """Track joint count so effort vectors match the observed state."""
        self._last_num_joints = len(msg.name)

    def _publish_command(self):
        """Publish a zero/default effort command sized to the robot's joints."""
        n = self._last_num_joints if self._last_num_joints > 0 else 12
        msg = Float64MultiArray()
        msg.data = [self._default_effort] * n
        self._command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointEffortCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
