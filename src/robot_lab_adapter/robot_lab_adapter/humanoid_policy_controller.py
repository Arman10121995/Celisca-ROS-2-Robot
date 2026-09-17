"""R5.3: Berkeley Humanoid Lite ONNX policy controller node.

A thin ROS2 wrapper around the pure-logic policy adapter
(:mod:`robot_lab_adapter.bhl_policy`). This node:

- Subscribes to ``/joint_states`` (best-effort) for measured positions and
  velocities (position targets are published either way; velocities feed the
  policy observation and are zeros until the first JointState with velocities
  arrives).
- Subscribes to ``/bhl/imu`` (sensor_msgs/Imu) for the body quaternion used
  in the observation (projected gravity) and the latched tilt safety monitor.
- Subscribes to ``cmd_vel`` (geometry_msgs/Twist) for the base-velocity
  command; the policy clamps it to the training command ranges.
- Publishes a ``Float64MultiArray`` of 22 values to the
  ``bhl_standing_controller`` command topic at the policy decision rate
  (25 Hz upstream). With ``command_interface:=effort`` (the default, matching
  the description's effort command interface) the node converts the policy's
  position targets into joint-space PD efforts
  ``tau = Kp*(q* - q) + Kd*(0 - qdot)``, clamped to +/-20 N.m, and the
  ``effort_controllers/JointGroupEffortController`` forwards them to the
  joints. The PD loop is closed *here*: that controller type is a pure effort
  forwarder and has no kp/kd of its own. ``command_interface:=position``
  publishes the raw position targets instead
  (see ``r53-bhl-effort-interface-2026-09-16``).

The observation assembly, inference, action conversion and safety latch all
live in ``bhl_policy`` and are unit-tested without a live ROS graph (see
``test/test_r5_3_bhl_policy_adapter.py``). This node only marshals ROS
messages to and from that logic.

The policy does not start driving until the first ``cmd_vel`` arrives. Before
that the node holds the *measured* pose (the straight-legged spawn pose,
proven stable under zero commands) instead of the config default pose: the
default pose bends the legs, and stepping there instantly into the standing
biped is the startup transient that toppled the earlier probes (evidence
``r53-bhl-actuation-2026-09-16``). On the first ``cmd_vel`` the node ramps
from the measured pose to the default pose over ``settle_duration_s``
(default 2 s, bounded), then hands over to the policy; a tilt at or beyond
the fall threshold mid-ramp ends the ramp immediately so the controller's
latched SAFE_STOP path takes over.
"""

from __future__ import annotations

from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray

from robot_lab_adapter.bhl_balance import (
    BHL_JOINT_NAMES,
    STANCE_PD_ARMS,
    STANCE_PD_LEGS,
    TILT_FALL_RAD,
    pd_effort_command,
)
from robot_lab_adapter.bhl_policy import (
    BhlPolicyController,
    POLICY_RATE_HZ,
    StartupSettle,
    load_policy_config,
    quaternion_tilt,
)


class HumanoidPolicyController(Node):
    """ONNX velocity-policy controller for the Berkeley Humanoid Lite."""

    def __init__(
        self,
        settle_duration_s: Optional[float] = None,
        command_interface: Optional[str] = None,
    ):
        super().__init__("humanoid_policy_controller")
        self.declare_parameter("command_topic", "/bhl_standing_controller/commands")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("imu_topic", "/bhl/imu")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("policy_name", "policy_humanoid")
        self.declare_parameter("command_rate_hz", POLICY_RATE_HZ)
        self.declare_parameter("settle_duration_s", 2.0)
        self.declare_parameter("command_interface", "effort")
        self.declare_parameter("kp_legs", STANCE_PD_LEGS[0])
        self.declare_parameter("kd_legs", STANCE_PD_LEGS[1])
        self.declare_parameter("kp_arms", STANCE_PD_ARMS[0])
        self.declare_parameter("kd_arms", STANCE_PD_ARMS[1])

        command_topic = self.get_parameter("command_topic").value
        joint_states_topic = self.get_parameter("joint_states_topic").value
        imu_topic = self.get_parameter("imu_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        policy_name = self.get_parameter("policy_name").value
        rate = float(self.get_parameter("command_rate_hz").value)
        if settle_duration_s is None:
            settle_duration = float(self.get_parameter("settle_duration_s").value)
        else:
            settle_duration = float(settle_duration_s)

        interface = str(
            self.get_parameter("command_interface").value
            if command_interface is None else command_interface
        )
        if interface not in ("effort", "position"):
            self.get_logger().warning(
                f"unknown command_interface '{interface}'; falling back to effort")
            interface = "effort"
        self._interface = interface
        self._leg_gains = (
            float(self.get_parameter("kp_legs").value),
            float(self.get_parameter("kd_legs").value))
        self._arm_gains = (
            float(self.get_parameter("kp_arms").value),
            float(self.get_parameter("kd_arms").value))

        self._controller = BhlPolicyController(load_policy_config(name=policy_name))
        self._settle = StartupSettle(
            self._controller.config.hold_pose(), duration_s=settle_duration)
        self._dt = 1.0 / rate
        self._measured_positions: Dict[str, float] = {}
        self._measured_velocities: Optional[Dict[str, float]] = None
        self._command = [0.0, 0.0, 0.0]
        self._orientation = (0.0, 0.0, 0.0, 1.0)
        self._gyro = (0.0, 0.0, 0.0)
        self._commanded = False

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._joint_sub = self.create_subscription(
            JointState, joint_states_topic, self._on_joint_state, sensor_qos
        )
        self._imu_sub = self.create_subscription(
            Imu, imu_topic, self._on_imu, sensor_qos
        )
        self._cmd_sub = self.create_subscription(
            Twist, cmd_vel_topic, self._on_cmd_vel, sensor_qos
        )
        self._command_pub = self.create_publisher(
            Float64MultiArray, command_topic,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST),
        )
        self._timer = self.create_timer(1.0 / rate, self._on_timer)
        self.get_logger().info(
            f"HumanoidPolicyController: policy={policy_name} "
            f"({joint_states_topic} + {imu_topic} + {cmd_vel_topic} -> "
            f"{command_topic}) at {rate} Hz over {len(BHL_JOINT_NAMES)} joints "
            f"({interface} interface)"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        """Cache the latest measured joint state keyed by name."""
        self._measured_positions = dict(zip(msg.name, msg.position))
        if msg.velocity:
            self._measured_velocities = dict(zip(msg.name, msg.velocity))

    def _on_imu(self, msg: Imu) -> None:
        """Cache the latest IMU attitude and body-frame angular velocity."""
        q = msg.orientation
        norm2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        if norm2 > 1e-12:
            self._orientation = (q.x, q.y, q.z, q.w)
        self._gyro = (
            msg.angular_velocity.x, msg.angular_velocity.y,
            msg.angular_velocity.z)

    def _on_cmd_vel(self, msg: Twist) -> None:
        """Cache the latest base-velocity command (policy clamps it)."""
        self._command = [msg.linear.x, msg.linear.y, msg.angular.z]
        self._commanded = True

    def _on_timer(self) -> None:
        """One policy cycle: onboard state -> Float64MultiArray targets."""
        if not self._measured_positions:
            # No measurement yet: publishing anything would command blind.
            return
        if not self._commanded:
            # Hold the measured pose until the first command arrives. The
            # spawn pose (straight-legged) is proven stable under zero
            # commands; stepping straight to the bent-leg default pose is
            # the startup transient that toppled the earlier probes.
            targets = self._settle.hold_targets(self._measured_positions)
            cycle_issues = []
        elif not self._settle.settled:
            if not self._settle.active:
                self._settle.start(self._measured_positions)
                self.get_logger().info(
                    "first cmd_vel: ramping from the measured pose to the "
                    f"policy default pose over {self._settle.duration_s:.1f} s")
            tilt = quaternion_tilt(*self._orientation)
            if tilt >= TILT_FALL_RAD:
                self.get_logger().error(
                    f"settle ramp aborted: tilt {tilt:.2f} rad exceeds the "
                    "fall threshold; handing over to the policy safety latch")
                self._settle.finish()
            targets, settled = self._settle.targets(
                self._measured_positions, self._dt)
            if settled:
                self.get_logger().info("settle ramp complete: policy driving")
            cycle_issues = []
        else:
            cycle = self._controller.update(
                command=self._command,
                gyro=self._gyro,
                orientation=self._orientation,
                measured_positions=self._measured_positions,
                measured_velocities=self._measured_velocities,
            )
            cycle_issues = cycle.issues
            targets = cycle.position_targets
        for issue in cycle_issues:
            if issue.startswith("safe_stop"):
                self.get_logger().error(issue)
            else:
                self.get_logger().warning(issue)
        if self._interface == "effort":
            # Close the PD-effort loop here: JointGroupEffortController is a
            # pure effort forwarder (no kp/kd of its own), so the node converts
            # the policy's position targets into clamped PD efforts. A joint
            # with no measurement gets zero effort (never a blind drive).
            efforts = pd_effort_command(
                targets,
                self._measured_positions,
                self._measured_velocities or {},
                self._leg_gains,
                self._arm_gains,
            )
            values = [efforts[j] for j in BHL_JOINT_NAMES]
        else:
            values = [targets[j] for j in BHL_JOINT_NAMES]
        msg = Float64MultiArray()
        msg.data = values
        self._command_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HumanoidPolicyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


