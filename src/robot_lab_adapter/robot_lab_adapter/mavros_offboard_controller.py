"""MAVROS Offboard Controller node (P3.5).

Publishes a nominal zero-thrust hold/attitude setpoint (AttitudeTarget) at
a fixed rate to the mavros setpoint /mavros/setpoint/attitude topic. This
serves as the minimal commandable interface for the Quadrotor SITL
simulated aerial profile, holding the multirotor at a stable hover in
offboard mode.

For a multirotor with zero command, AttitudeTarget defaults to a neutral
orientation with zero thrust, producing a stable hover. Realsense-style
MAVROS bridges forward this to the ArduPilot SITL FCU.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# mavros_msgs is a soft runtime dependency: imported lazily so the node can
# start (and be unit-tested) in environments without micro-ROS/mavros installed.
try:
    from mavros_msgs.msg import AttitudeTarget
    _HAS_MAVROS = True
except ImportError:
    AttitudeTarget = None
    _HAS_MAVROS = False


class MavrosOffboardController(Node):
    """Publish AttitudeTarget hold commands for a multirotor SITL profile."""

    def __init__(self):
        super().__init__('mavros_offboard_controller')
        self.declare_parameter('command_topic', '/mavros/setpoint/attitude')
        self.declare_parameter('command_rate_hz', 10.0)
        self.declare_parameter('default_throttle', 0.0)
        self.declare_parameter('mode', 'ALT_HOLD')

        self._command_topic = self.get_parameter('command_topic').value
        self._command_rate = float(self.get_parameter('command_rate_hz').value)
        self._throttle = float(self.get_parameter('default_throttle').value)
        self._mode = self.get_parameter('mode').value

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._command_pub = None
        if _HAS_MAVROS:
            self._command_pub = self.create_publisher(
                AttitudeTarget, self._command_topic, 10,
            )
            self._timer = self.create_timer(
                1.0 / self._command_rate, self._publish_command
            )
            self.get_logger().info(
                f"MavrosOffboardController: -> {self._command_topic} "
                f"at {self._command_rate} Hz, mode={self._mode}, "
                f"throttle={self._throttle}"
            )
        else:
            self.get_logger().warn(
                'mavros_msgs not available; mavros_offboard_controller running '
                'in fallback mode (no MAVLink setpoints emitted).'
            )

    def _publish_command(self):
        """Publish a zero-thrust attitude hold target (stable hover)."""
        if not _HAS_MAVROS:
            return
        msg = AttitudeTarget()
        # Timestamp and frame for the FCU; zero attitude / zero thrust = hold.
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0
        msg.thrust = self._throttle
        msg.type_mask = 0
        self._command_pub.publish(msg)
        _ = self._command_pub  # keep reference


def main(args=None):
    rclpy.init(args=args)
    node = MavrosOffboardController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
