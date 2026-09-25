"""ROS wrapper for the measured-joint Go2 stance and bounded trot core."""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from robot_lab_adapter.go2_locomotion import (
    BaseVelocity, BodyState, Go2LocomotionCore, JOINT_NAMES,
)


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
        rate = float(self.get_parameter("command_rate_hz").value)
        if rate <= 0.0:
            raise ValueError("command_rate_hz must be positive")
        self._period = 1.0 / rate
        self._timeout = float(self.get_parameter("command_timeout_s").value)
        self._gait_enabled = bool(self.get_parameter("enable_experimental_gait").value)
        self._core = Go2LocomotionCore(
            gain_scale=float(self.get_parameter("gain_scale").value),
            damping_scale=float(self.get_parameter("damping_scale").value))
        self._positions = {}
        self._velocities = {}
        self._efforts = {}
        self._body = None
        self._cmd = BaseVelocity()
        self._cmd_at = float("-inf")
        self._last_safety_state = self._core.safety.state
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            JointState, self.get_parameter("joint_states_topic").value,
            self._on_joint_state, sensor_qos)
        self.create_subscription(
            Imu, self.get_parameter("imu_topic").value,
            self._on_imu, sensor_qos)
        self.create_subscription(
            Twist, self.get_parameter("cmd_vel_topic").value,
            self._on_twist, 10)
        self._pub = self.create_publisher(
            Float64MultiArray, self.get_parameter("effort_topic").value, 10)
        self.create_service(Trigger, "/go2_controller/reset_safety", self._reset_safety)
        self.create_timer(self._period, self._on_timer)
        self.get_logger().info(
            "Go2 closed-loop stance: %d effort joints at %.1f Hz; "
            "experimental gait %s" %
            (len(JOINT_NAMES), rate, "enabled" if self._gait_enabled else "disabled"))

    def _on_joint_state(self, msg):
        self._positions = dict(zip(msg.name, msg.position))
        self._velocities = dict(zip(msg.name, msg.velocity))
        self._efforts = dict(zip(msg.name, msg.effort))

    def _on_imu(self, msg):
        q = msg.orientation
        if q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w > 1e-6:
            self._body = BodyState.from_quaternion(q.x, q.y, q.z, q.w)

    def _on_twist(self, msg):
        self._cmd = BaseVelocity(
            vx=float(msg.linear.x), vy=float(msg.linear.y), wz=float(msg.angular.z))
        self._cmd_at = time.monotonic()

    def _reset_safety(self, _request, response):
        self._core.safety.reset()
        self._cmd = BaseVelocity()
        self._cmd_at = float("-inf")
        response.success = True
        response.message = "Go2 safety latch reset; command is zero"
        return response

    def _on_timer(self):
        command = self._cmd if time.monotonic() - self._cmd_at <= self._timeout \
            else BaseVelocity()
        if not self._gait_enabled:
            command = BaseVelocity()
        cycle = self._core.update(
            self._period, self._positions, self._velocities,
            self._efforts if self._efforts else None,
            body=self._body, velocity_command=command)
        if cycle.safety_state != self._last_safety_state:
            self.get_logger().warning(
                "Go2 safety %s: %s" %
                (cycle.safety_state, self._core.safety.reason or "attitude recovered"))
            self._last_safety_state = cycle.safety_state
        msg = Float64MultiArray()
        msg.data = [cycle.efforts[name] for name in JOINT_NAMES]
        self._pub.publish(msg)


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
