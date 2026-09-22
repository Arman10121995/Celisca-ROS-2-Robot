"""R5.3: Berkeley Humanoid Lite humanoid standing controller node.

A thin ROS2 wrapper around the pure-logic closed-loop balance core
(:mod:`robot_lab_adapter.bhl_balance`). This node:

- Subscribes to ``/joint_states`` (best-effort) for measured positions and
  velocities (the D term of the effort loop needs the velocities).
- Subscribes to ``/imu/out`` (sensor_msgs/Imu, the topic the Gazebo bridge
  publishes) for the body attitude used by
  the ankle-strategy balance and the latched safety monitor.
- Publishes a ``Float64MultiArray`` of 22 values to the
  ``bhl_standing_controller`` command topic at the declared rate. With
  ``command_interface:=effort`` (the default, matching the description's
  effort command interface) those values are the joint-space PD efforts
  ``tau = Kp*(q* - q) + Kd*(0 - qdot)``, clamped to +/-20 N.m, that the
  ``effort_controllers/JointGroupEffortController`` forwards to the joints --
  the loop is closed *here*, not inside the controller (which forwards).
  ``command_interface:=position`` publishes the raw stance targets instead
  (see ``r53-bhl-effort-interface-2026-09-16``).

The balance law itself is tested without a live ROS graph (see
``test/test_r5_3_bhl_balance.py``). This node only marshals ROS messages to
and from that logic.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray

from robot_lab_adapter.bhl_balance import (
    BHL_JOINT_NAMES,
    BALANCE_RATE_HZ,
    BodyState,
    BhlBalanceController,
)


def quaternion_to_body_state(imu: Imu, invert_tilt: bool = False) -> BodyState:
    """Extract roll/pitch body state from an IMU message.

    Falls back to a zero-attitude BodyState if the orientation covariance
    is invalid (honest missing-attitude handling). ``invert_tilt`` negates
    both angles (diagnostic switch for sensor-convention mismatches).
    """
    q = imu.orientation
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if norm < 1e-6:
        return BodyState(roll_rad=0.0, pitch_rad=0.0)
    body = BodyState.from_quaternion(q.x, q.y, q.z, q.w)
    if invert_tilt:
        body = BodyState(roll_rad=-body.roll_rad, pitch_rad=-body.pitch_rad)
    return body


class HumanoidStandingController(Node):
    """Closed-loop standing/balance controller for the Berkeley Humanoid Lite."""

    def __init__(self):
        super().__init__("humanoid_standing_controller")
        self.declare_parameter("command_topic", "/bhl_standing_controller/commands")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("imu_topic", "/imu/out")
        self.declare_parameter("command_interface", "effort")
        self.declare_parameter("command_rate_hz", BALANCE_RATE_HZ)
        self.declare_parameter(
            "imu_invert_tilt", False)

        command_topic = self.get_parameter("command_topic").value
        joint_states_topic = self.get_parameter("joint_states_topic").value
        imu_topic = self.get_parameter("imu_topic").value
        rate = float(self.get_parameter("command_rate_hz").value)
        interface = str(self.get_parameter("command_interface").value)
        self._invert_tilt = bool(
            self.get_parameter("imu_invert_tilt").value)
        if interface not in ("effort", "position"):
            self.get_logger().warning(
                f"unknown command_interface '{interface}'; falling back to effort")
            interface = "effort"
        self._interface = interface

        self._controller = BhlBalanceController()
        self._measured_positions: dict = {}
        self._measured_velocities: dict = {}

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._joint_sub = self.create_subscription(
            JointState, joint_states_topic, self._on_joint_state, sensor_qos
        )
        self._imu_sub = self.create_subscription(
            Imu, imu_topic, self._on_imu, sensor_qos
        )
        self._command_pub = self.create_publisher(
            Float64MultiArray, command_topic,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST),
        )
        self._body: BodyState | None = None
        self._timer = self.create_timer(1.0 / rate, self._on_timer)
        self.get_logger().info(
            f"HumanoidStandingController: {joint_states_topic} + {imu_topic} "
            f"-> {command_topic} at {rate} Hz over {len(BHL_JOINT_NAMES)} joints "
            f"({interface} interface, closed-loop balance)"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        """Cache the latest measured joint positions and velocities by name."""
        self._measured_positions = dict(zip(msg.name, msg.position))
        if msg.velocity:
            self._measured_velocities = dict(zip(msg.name, msg.velocity))

    def _on_imu(self, msg: Imu) -> None:
        """Cache the latest body attitude from the IMU."""
        self._body = quaternion_to_body_state(msg, self._invert_tilt)

    def _on_timer(self) -> None:
        """One balance cycle: body state -> Float64MultiArray command."""
        dt = 1.0 / float(self.get_parameter("command_rate_hz").value)
        cycle = self._controller.update(
            dt=dt,
            measured_positions=self._measured_positions,
            measured_velocities=self._measured_velocities,
            body=self._body,
        )
        for issue in cycle.issues:
            if "warn" in issue:
                self.get_logger().warn(issue)
            elif "safe_stop" in issue:
                self.get_logger().warning(f"SAFE_STOP: {issue}")
        if self._interface == "effort":
            # The balance core already computes the clamped PD efforts
            # (zero-drive for unmeasured joints, zero on SAFE_STOP); the effort
            # controller forwards them unchanged.
            values = [cycle.efforts[j] for j in BHL_JOINT_NAMES]
        else:
            values = [cycle.position_targets[j] for j in BHL_JOINT_NAMES]
        msg = Float64MultiArray()
        msg.data = values
        self._command_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HumanoidStandingController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # A stop from launch or the GUI, not a failure: without this every
        # standing-controller shutdown logged a traceback and "process has died".
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
