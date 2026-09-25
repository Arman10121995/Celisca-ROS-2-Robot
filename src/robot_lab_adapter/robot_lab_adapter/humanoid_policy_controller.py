"""R5.3: Berkeley Humanoid Lite ONNX policy controller node.

A thin ROS2 wrapper around the pure-logic policy adapter
(:mod:`robot_lab_adapter.bhl_policy`). This node:

- Subscribes to ``/joint_states`` (best-effort) for measured positions and
  velocities (position targets are published either way; velocities feed the
  policy observation and are zeros until the first JointState with velocities
  arrives).
- Subscribes to ``/imu/out`` (sensor_msgs/Imu, the topic the Gazebo bridge
  publishes) for the body quaternion used
  in the observation (projected gravity) and the latched tilt safety monitor.
- Subscribes to ``cmd_vel`` (geometry_msgs/Twist) for the base-velocity
  command; the policy clamps it to the training command ranges. With
  ``yaw_servo_gain`` > 0 (off by default) the yaw part of that command is
  additionally closed in the loop on the measured body yaw rate: a held
  pure-turn command was measured to collapse the policy's stepping limit
  cycle, and the boost-only servo (``bhl_policy.YawRateBoost``) raises the
  commanded rate toward the training limit while the robot fails to turn.
- Publishes a ``Float64MultiArray`` of 22 values to the
  ``bhl_standing_controller`` command topic at 250 Hz for effort servoing,
  with policy decisions at 25 Hz upstream. With ``command_interface:=effort`` (the default, matching
  the description's effort command interface) the node converts the policy's
  position targets into joint-space PD efforts
  ``tau = Kp*(q* - q) + Kd*(0 - qdot)`` using checkpoint gains and
  limits (4 N.m arms / 6 N.m legs), additionally bounded by the URDF, and the
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
(default 2 s, bounded), then hands over to the policy. The ramp itself is
*stabilized*: it is servoed with the stance PD gains and the ankle-strategy
tilt feedback from the balance core (:func:`bhl_balance.balance_targets`
applied to the ramp blend), because the checkpoint policy gains are
statically unstable for a standing biped (restoring stiffness
2 * kp * ankle_k ~= 40 N.m/rad against a ~71 N.m/rad gravity topple
stiffness) — the open-loop joint-space ramp toppled the robot in evidence
``r53-live-policy-servo-2026-09-22`` before the policy ever drove. Once the
ramp completes, the checkpoint gains take over on the policy phases where
they were verified (``r53-bhl-adapter-path-2026-09-15``). A tilt at or beyond
the fall threshold mid-ramp ends the ramp immediately and latches the
controller's SAFE_STOP path (zero effort), exactly as in the driving phases.
"""

from __future__ import annotations

import math
from typing import Dict, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray

from robot_lab_adapter.bhl_balance import (
    BHL_JOINT_NAMES,
    BALANCE_RATE_HZ,
    EFFORT_LIMIT,
    TILT_FALL_RAD,
    BodyState,
    balance_targets,
    pd_effort_command,
)
from robot_lab_adapter.bhl_policy import (
    BhlPolicyController,
    POLICY_RATE_HZ,
    StartupSettle,
    YAW_COMMAND_LIMIT,
    YawRateBoost,
    load_policy_config,
    quaternion_tilt,
)


class HumanoidPolicyController(Node):
    """ONNX velocity-policy controller for the Berkeley Humanoid Lite."""

    def __init__(
        self,
        settle_duration_s: Optional[float] = None,
        command_interface: Optional[str] = None,
        yaw_servo_gain: Optional[float] = None,
    ):
        super().__init__("humanoid_policy_controller")
        self.declare_parameter("command_topic", "/bhl_standing_controller/commands")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("imu_topic", "/imu/out")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("policy_name", "policy_humanoid")
        self.declare_parameter("command_rate_hz", POLICY_RATE_HZ)
        self.declare_parameter("settle_duration_s", 2.0)
        self.declare_parameter("yaw_servo_gain", 0.0)
        self.declare_parameter("yaw_servo_limit", YAW_COMMAND_LIMIT)
        self.declare_parameter("yaw_servo_filter_tau_s", 0.08)
        self.declare_parameter("command_interface", "effort")
        self.declare_parameter("effort_rate_hz", BALANCE_RATE_HZ)

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
        self._controller = BhlPolicyController(load_policy_config(name=policy_name))
        config = self._controller.config
        self._pd = dict(zip(config.joints, zip(config.kp, config.kd, config.effort_limits)))
        effort_rate = float(self.get_parameter("effort_rate_hz").value)
        if not math.isfinite(rate) or rate <= 0 or not math.isfinite(effort_rate) or effort_rate < rate:
            raise ValueError("positive policy rate and effort rate >= policy rate required")
        self._yaw_servo = YawRateBoost(
            gain=(float(self.get_parameter("yaw_servo_gain").value)
                  if yaw_servo_gain is None else float(yaw_servo_gain)),
            limit=float(self.get_parameter("yaw_servo_limit").value),
            filter_tau_s=float(
                self.get_parameter("yaw_servo_filter_tau_s").value))
        self._targets = {}
        self._settle = StartupSettle(
            self._controller.config.hold_pose(), duration_s=settle_duration)
        self._dt = 1.0 / rate
        self._measured_positions: Dict[str, float] = {}
        self._measured_velocities: Optional[Dict[str, float]] = None
        self._command = [0.0, 0.0, 0.0]
        self._orientation = (0.0, 0.0, 0.0, 1.0)
        self._gyro = (0.0, 0.0, 0.0)
        self._commanded = False

        # The policy closes a 250 Hz effort loop on live simulator state. A
        # ten-sample sensor queue can feed it measurements already 40 ms old
        # when the ROS graph is busy (e.g. localization + RViz); prefer the
        # newest sample over processing a backlog of obsolete joint/IMU data.
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
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
        self._effort_timer = (
            self.create_timer(1.0 / effort_rate, self._on_effort_timer)
            if interface == "effort" else None)

        self.get_logger().info(
            f"HumanoidPolicyController: policy={policy_name} "
            f"({joint_states_topic} + {imu_topic} + {cmd_vel_topic} -> "
            f"{command_topic}) at {rate} Hz over {len(BHL_JOINT_NAMES)} joints "
            f"({interface} interface; effort servo {effort_rate} Hz, "
            "stance PD/ankle-strategy settle ramp, checkpoint PD/limits "
            f"driving{self._servo_note()})"
        )

    def _servo_note(self) -> str:
        """Startup log fragment for the opt-in yaw servo (empty when off)."""
        if not self._yaw_servo.enabled:
            return ""
        return (f", boost-only yaw servo gain={self._yaw_servo.gain:g} "
                f"limit=+/-{self._yaw_servo.limit:g} rad/s "
                f"filter_tau={self._yaw_servo.filter_tau_s:g} s")

    def _on_joint_state(self, msg: JointState) -> None:
        """Cache the latest measured joint state keyed by name."""
        self._measured_positions = dict(zip(msg.name, msg.position))
        self._measured_velocities = dict(zip(msg.name, msg.velocity)) if msg.velocity else None

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

    def _policy_command(self):
        """Command for this policy cycle, with the opt-in yaw servo applied.

        The servo reads the same body-frame yaw rate the policy observation
        uses (``self._gyro[2]``), so no second measurement source is
        introduced. With ``yaw_servo_gain`` at its 0 default, and whenever
        the operator's yaw reference is zero, this is the operator's command
        unchanged.
        """
        return [self._command[0], self._command[1],
                self._yaw_servo.command(
                    self._command[2], self._gyro[2], self._dt)]

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
            elif not self._settle.active:
                self.get_logger().error(
                    "settle ramp aborted: handing over to the policy safety "
                    "latch; the policy drive path stays zero-effort")
            cycle_issues = []
        else:
            cycle = self._controller.update(
                command=self._policy_command(),
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
        self._targets = targets
        if self._interface == "position":
            self._publish([targets[j] for j in BHL_JOINT_NAMES])

    def _publish(self, values):
        msg = Float64MultiArray()
        msg.data = values
        self._command_pub.publish(msg)

    def _on_effort_timer(self):
        """Fresh-state PD between policy decisions, in checkpoint joint order."""
        if not self._measured_positions or not self._targets:
            return
        if quaternion_tilt(*self._orientation) >= TILT_FALL_RAD:
            # The public update checks tilt before inference and latches stop.
            self._controller.update(
                command=self._command, gyro=self._gyro, orientation=self._orientation,
                measured_positions=self._measured_positions,
                measured_velocities=self._measured_velocities)
        if self._controller.safety_state == self._controller.SAFE_STOP:
            self._publish([0.0] * len(BHL_JOINT_NAMES))
            return
        if not self._commanded or (
                self._settle.active and not self._settle.settled):
            # Settle/hold phase: the checkpoint policy gains are statically
            # unstable for a standing biped (restoring stiffness
            # 2 * kp * ankle_k ~= 40 N.m/rad against the ~71 N.m/rad gravity
            # topple stiffness), which is what toppled the first live
            # policy-servo run during the joint-space ramp (evidence
            # ``r53-live-policy-servo-2026-09-22``). The ramp is servoed with
            # the stance PD + ankle-strategy gains the balance core was
            # plant-swept with instead (docs/status/evidence/
            # r53-bhl-ankle-fix-2026-09-17): tilt feedback stabilizes the
            # bend-down, and the harder stance gains exceed topple stiffness.
            stance = balance_targets(self._targets, self._body_state())
            efforts = pd_effort_command(
                stance, self._measured_positions, self._measured_velocities)
            self._publish([efforts[j] for j in BHL_JOINT_NAMES])
            return
        values = []
        for joint in BHL_JOINT_NAMES:
            target = (self._targets.get(joint) if self._commanded
                      else self._measured_positions.get(joint))
            measured = self._measured_positions.get(joint)
            velocity = (self._measured_velocities or {}).get(joint, 0.0)
            if target is None or measured is None or not all(
                    math.isfinite(v) for v in (target, measured, velocity)):
                values.append(0.0)
                continue
            kp, kd, limit = self._pd[joint]
            bound = min(float(limit), EFFORT_LIMIT)
            effort = kp * (target - measured) - kd * velocity
            values.append(float(max(-bound, min(bound, effort))))
        self._publish(values)

    def _body_state(self):
        """Body roll/pitch from the cached IMU quaternion (balance-core type)."""
        x, y, z, w = self._orientation
        return BodyState.from_quaternion(x, y, z, w)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HumanoidPolicyController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # A stop from launch or the probe's process-group SIGINT, not a
        # failure: without this every SIGINT teardown logged an RCLError
        # "rcl_shutdown already called" traceback and a non-zero exit.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()


