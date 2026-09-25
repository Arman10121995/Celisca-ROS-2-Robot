"""ROS wrapper for the measured-joint Go2 stance and bounded trot core."""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import Trigger

from robot_lab_adapter.go2_locomotion import (
    BaseVelocity, BodyState, FallRecovery, Go2LocomotionCore, JOINT_NAMES,
)
from robot_lab_adapter.go2_velocity_policy import Go2VelocityPolicy
from robot_lab_adapter.go2_recovery_policy import Go2RecoveryPolicy, RECOVERY_POLICY_DT_S


def _param_float(node, name, default):
    """Read a float parameter tolerating a string-typed launch override."""
    value = node.get_parameter(name).value
    if isinstance(value, str):
        try:
            return float(value.strip())
        except ValueError:
            return default
    return float(value)


def _as_flag(node, name, default=False):
    """Read a boolean parameter tolerating a string-typed launch override."""
    value = node.get_parameter(name).value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in ("true", "1", "yes", "on"):
            return True
        if normalized in ("false", "0", "no", "off"):
            return False
        return default
    return bool(value)


class Go2StanceGaitController(Node):
    def __init__(self):
        super().__init__("go2_stance_gait_controller")
        self.declare_parameter("command_rate_hz", 250.0)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("imu_topic", "/imu/out")
        self.declare_parameter("cmd_vel_topic", "/robot_lab_controller/cmd_vel_unstamped")
        self.declare_parameter("effort_topic", "/go2_group_effort_controller/commands")
        # The imported URDF has zero motor armature. At 250 Hz the upstream
        # gains excite its light calf joints, so use gains qualified against
        # the sampled MuJoCo plant rather than the continuous-time model.
        self.declare_parameter("gain_scale", 0.2)
        self.declare_parameter("damping_scale", 0.2)
        self.declare_parameter("enable_experimental_gait", False)
        self.declare_parameter("policy_path", "")
        self.declare_parameter("reverse_command_map", "feedforward")
        # Fall recovery is opt-in and experimental. Off by default: a re-stand
        # attempt drives the nominal pose with elevated gains and has not been
        # shown to right the robot on this plant.
        self.declare_parameter("enable_fall_recovery", False)
        self.declare_parameter("fall_recovery_timeout_s", 4.0)
        self.declare_parameter("fall_recovery_start_delay_s", 0.0)
        self.declare_parameter("fall_recovery_gain_scale", 0.5)
        self.declare_parameter("fall_recovery_damping_scale", 0.5)
        self.declare_parameter("recovery_policy_path", "")
        rate = float(self.get_parameter("command_rate_hz").value)
        if rate <= 0.0:
            raise ValueError("command_rate_hz must be positive")
        self._period = 1.0 / rate
        self._timeout = float(self.get_parameter("command_timeout_s").value)
        self._gait_enabled = bool(self.get_parameter("enable_experimental_gait").value)
        policy_path = str(self.get_parameter("policy_path").value)
        reverse_command_map = str(self.get_parameter("reverse_command_map").value)
        self._policy = (Go2VelocityPolicy(policy_path, reverse_map=reverse_command_map)
                        if policy_path else None)
        self._last_policy_at = float("-inf")
        self._core = Go2LocomotionCore(
            gain_scale=float(self.get_parameter("gain_scale").value),
            damping_scale=float(self.get_parameter("damping_scale").value))
        self._positions = {}
        self._velocities = {}
        self._efforts = {}
        self._foot_forces = None
        self._foot_forces_at = float("-inf")
        self._body = None
        self._body_height = None
        self._body_height_at = float("-inf")
        self._orientation = None
        self._angular_velocity = None
        self._cmd = BaseVelocity()
        self._cmd_at = float("-inf")
        self._last_safety_state = self._core.safety.state
        self._recovery = (
            FallRecovery(
                timeout_s=_param_float(
                    self, "fall_recovery_timeout_s", 4.0),
                start_delay_s=_param_float(
                    self, "fall_recovery_start_delay_s", 0.0),
                gain_scale=_param_float(
                    self, "fall_recovery_gain_scale", 0.5),
                damping_scale=_param_float(
                    self, "fall_recovery_damping_scale", 0.5),
            )
            if _as_flag(self, "enable_fall_recovery", False)
            else None)
        recovery_path = str(self.get_parameter("recovery_policy_path").value)
        if recovery_path and self._recovery is None:
            raise ValueError("recovery_policy_path requires enable_fall_recovery:=true")
        self._recovery_policy = (Go2RecoveryPolicy(recovery_path)
                                 if recovery_path else None)
        self._last_recovery_policy_at = float("-inf")
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            JointState, self.get_parameter("joint_states_topic").value,
            self._on_joint_state, sensor_qos)
        self.create_subscription(
            Imu, self.get_parameter("imu_topic").value,
            self._on_imu, sensor_qos)
        self.create_subscription(
            Odometry, "/odom/ground_truth", self._on_ground_truth, sensor_qos)
        self.create_subscription(
            Float64MultiArray, "/go2/foot_contact_forces",
            self._on_foot_contacts, sensor_qos)
        self.create_subscription(
            Twist, self.get_parameter("cmd_vel_topic").value,
            self._on_twist, 10)
        self._pub = self.create_publisher(
            Float64MultiArray, self.get_parameter("effort_topic").value, 10)
        self._safety_pub = self.create_publisher(String, "/go2/safety_state", 10)
        self._fallen_pub = self.create_publisher(Bool, "/go2/fallen", 10)
        self._recovery_pub = self.create_publisher(String, "/go2/recovery_state", 10)
        self.create_service(Trigger, "/go2_controller/reset_safety", self._reset_safety)
        self.create_timer(self._period, self._on_timer)
        self.get_logger().info(
            "Go2 closed-loop stance: %d effort joints at %.1f Hz; "
            "controller %s" % (len(JOINT_NAMES), rate,
                                "ONNX velocity policy" if self._policy else
                                "experimental gait" if self._gait_enabled else "stance only"))

    def _on_joint_state(self, msg):
        self._positions = dict(zip(msg.name, msg.position))
        self._velocities = dict(zip(msg.name, msg.velocity))
        self._efforts = dict(zip(msg.name, msg.effort))

    def _on_imu(self, msg):
        q = msg.orientation
        if q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w > 1e-6:
            attitude = BodyState.from_quaternion(q.x, q.y, q.z, q.w)
            height = (self._body_height if time.monotonic() - self._body_height_at < 0.2
                      else None)
            self._body = BodyState(attitude.roll_rad, attitude.pitch_rad, height)
            self._orientation = (q.x, q.y, q.z, q.w)
            a = msg.angular_velocity
            self._angular_velocity = (a.x, a.y, a.z)

    def _on_ground_truth(self, msg):
        height = float(msg.pose.pose.position.z)
        if 0.0 <= height < 5.0:
            self._body_height = height
            self._body_height_at = time.monotonic()

    def _on_twist(self, msg):
        self._cmd = BaseVelocity(
            vx=float(msg.linear.x), vy=float(msg.linear.y), wz=float(msg.angular.z))
        self._cmd_at = time.monotonic()

    def _on_foot_contacts(self, msg):
        if len(msg.data) == 4:
            self._foot_forces = dict(zip(("FL", "FR", "RL", "RR"), msg.data))
            self._foot_forces_at = time.monotonic()

    def _reset_safety(self, _request, response):
        self._core.safety.reset()
        self._cmd = BaseVelocity()
        self._cmd_at = float("-inf")
        if self._policy:
            self._policy.reset()
            self._last_policy_at = float("-inf")
        if self._recovery is not None:
            self._recovery.reset()
        if self._recovery_policy is not None:
            self._recovery_policy.reset()
            self._last_recovery_policy_at = float("-inf")
        response.success = True
        response.message = "Go2 safety latch reset; command is zero"
        return response

    def _on_timer(self):
        command = self._cmd if time.monotonic() - self._cmd_at <= self._timeout \
            else BaseVelocity()
        if self._policy is not None:
            efforts = self._policy_efforts(command)
            state = self._core.safety.state
        else:
            if not self._gait_enabled:
                command = BaseVelocity()
            cycle = self._core.update(
                self._period, self._positions, self._velocities,
                self._efforts if self._efforts else None,
                body=self._body, velocity_command=command,
                measured_contact_forces=self._foot_forces)
            efforts, state = cycle.efforts, cycle.safety_state
        if self._recovery is not None and self._core.safety.fallen:
            now = time.monotonic()
            forces = (self._foot_forces if now - self._foot_forces_at < 0.1
                      else None)
            recovered = self._recovery.update(
                now, self._core.safety.fallen, self._body,
                self._positions, self._velocities, forces)
            efforts = {name: recovered.get(name, 0.0) for name in JOINT_NAMES}
            if (self._recovery_policy is not None
                    and self._recovery.status == FallRecovery.ATTEMPTING):
                if now - self._last_recovery_policy_at >= RECOVERY_POLICY_DT_S:
                    try:
                        self._recovery_policy.step(
                            self._positions, self._velocities,
                            self._angular_velocity, self._body, forces)
                        self._last_recovery_policy_at = now
                    except Exception as exc:
                        # The experimental inference path must fail closed even
                        # if ONNX Runtime raises outside our input checks.
                        self._recovery.status = FallRecovery.FAILED
                        self.get_logger().error("Go2 recovery policy failed: %s" % exc)
                efforts = (self._recovery_policy.efforts(
                    self._positions, self._velocities)
                    if self._recovery.status == FallRecovery.ATTEMPTING
                    else {name: 0.0 for name in JOINT_NAMES})
        if state != self._last_safety_state:
            self.get_logger().warning(
                "Go2 safety %s: %s" %
                (state, self._core.safety.reason or "attitude recovered"))
            self._last_safety_state = state
        safety_msg = String()
        safety_msg.data = state
        self._safety_pub.publish(safety_msg)
        fallen_msg = Bool()
        fallen_msg.data = self._core.safety.fallen
        self._fallen_pub.publish(fallen_msg)
        recovery_msg = String()
        recovery_msg.data = self._recovery.status if self._recovery else "disabled"
        self._recovery_pub.publish(recovery_msg)
        msg = Float64MultiArray()
        msg.data = [efforts[name] for name in JOINT_NAMES]
        self._pub.publish(msg)

    def _policy_efforts(self, command):
        zero = {name: 0.0 for name in JOINT_NAMES}
        if self._body is None or self._orientation is None or \
                self._angular_velocity is None or not all(
                    name in self._positions and name in self._velocities
                    for name in JOINT_NAMES):
            return zero
        safety = self._core.safety
        safety.observe_body(self._body)
        if not safety.gait_permitted():
            return zero
        limited = self._core.set_velocity(command, self._period)
        now = time.monotonic()
        if now - self._last_policy_at >= 0.02:
            try:
                self._policy.step(self._positions, self._velocities,
                                  self._angular_velocity, self._orientation,
                                  limited)
            except ValueError as exc:
                safety._enter_safe_stop("invalid Go2 policy data: " + str(exc))
                return zero
            self._last_policy_at = now
        efforts = self._policy.efforts(self._positions, self._velocities)
        if self._efforts:
            safety.observe_efforts(efforts, self._efforts)
        return efforts if safety.gait_permitted() else zero


def main(args=None):
    rclpy.init(args=args)
    node = Go2StanceGaitController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
