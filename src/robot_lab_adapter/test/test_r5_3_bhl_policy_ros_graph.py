"""R5.3 integration tests: policy controller node on a live ROS graph.

Launches the real ``HumanoidPolicyController`` node (the deployed entry
point) against a real rclpy graph and verifies the ROS marshalling that the
pure-logic tests cannot cover:

- Before any ``cmd_vel`` arrives the node holds the *measured* pose on the
  ``bhl_standing_controller`` command topic (spawn-pose hold: the config
  default pose bends the legs and stepping there instantly is the startup
  transient that toppled the earlier probes — see evidence
  ``r53-bhl-actuation-2026-09-16``).
- Two command interfaces are exercised. The deployed default is
  ``command_interface:=effort``: the node converts its position targets into
  clamped PD efforts ``tau = kp*(q* - q) + kd*(0 - qdot)`` because
  ``effort_controllers/JointGroupEffortController`` is a pure effort forwarder
  with no control law of its own (see ``r53-bhl-effort-interface-2026-09-16``).
  ``command_interface:=position`` publishes the raw position targets and is
  covered here so the position marshalling path stays regression-tested.
- A ``cmd_vel`` ramps from the measured pose to the default pose over
  ``settle_duration_s`` (injected short in these tests), then real ONNX
  inference takes over: 22 finite targets at the policy rate, with the joint
  velocities from ``JointState`` actually consumed (non-zero dq in the
  observation path).
- A tipped IMU latches SAFE_STOP; on the effort interface the node publishes
  zero efforts (no drive), on the position interface the default pose.

These tests require a sourced ROS 2 environment (rclpy) and the vendored
checkpoints; they are marked ``integration`` and skip cleanly when either is
missing, so the fast unit tier stays graph-free.
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path

import numpy as np
import pytest

_adapter_pkg = Path(__file__).resolve().parents[1]
if str(_adapter_pkg) not in sys.path:
    sys.path.insert(0, str(_adapter_pkg))

rclpy = pytest.importorskip("rclpy", reason="live-graph test needs a sourced ROS 2 env")
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402

from robot_lab_adapter.bhl_balance import BHL_JOINT_NAMES, EFFORT_LIMIT  # noqa: E402
from robot_lab_adapter.bhl_policy import load_policy_config  # noqa: E402

pytestmark = pytest.mark.integration

COMMAND_TOPIC = "/bhl_standing_controller/commands"
JOINT_STATES_TOPIC = "/joint_states"
IMU_TOPIC = "/imu/out"
CMD_VEL_TOPIC = "/cmd_vel"


@pytest.fixture(scope="module")
def policy_config():
    return load_policy_config(name="policy_humanoid")


class GraphHarness:
    """Controller node + sensor/command stubs on one live executor."""

    def __init__(self, config, settle_duration_s=0.08, command_interface="position"):
        self.config = config
        self.received = []  # (recv_time, Float64MultiArray)

        from rclpy.node import Node
        from geometry_msgs.msg import Twist
        from sensor_msgs.msg import Imu, JointState
        from std_msgs.msg import Float64MultiArray

        self.JointState = JointState
        self.Imu = Imu
        self.Twist = Twist

        from robot_lab_adapter.humanoid_policy_controller import (
            HumanoidPolicyController)
        # A short injected settle ramp: the deployed default (2 s) would
        # dominate these tests' spin windows; the ramp itself is covered by
        # the pure-logic StartupSettle tests.
        self.controller = HumanoidPolicyController(
            settle_duration_s=settle_duration_s,
            command_interface=command_interface)

        self.helper = Node("policy_graph_test_helper")
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._joint_pub = self.helper.create_publisher(
            JointState, JOINT_STATES_TOPIC, reliable)
        self._imu_pub = self.helper.create_publisher(Imu, IMU_TOPIC, reliable)
        self._cmd_pub = self.helper.create_publisher(Twist, CMD_VEL_TOPIC, reliable)
        self.helper.create_subscription(
            Float64MultiArray, COMMAND_TOPIC, self._on_targets, reliable)

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.controller)
        self.executor.add_node(self.helper)

    def _on_targets(self, msg):
        self.received.append((time.monotonic(), msg))

    # -- stimulus helpers -------------------------------------------------
    def publish_sensors(self, positions, velocities, quat=(0.0, 0.0, 0.0, 1.0),
                        gyro=(0.0, 0.0, 0.0)):
        joint_msg = self.JointState()
        joint_msg.name = list(BHL_JOINT_NAMES)
        joint_msg.position = [positions.get(j, 0.0) for j in BHL_JOINT_NAMES]
        joint_msg.velocity = [velocities.get(j, 0.0) for j in BHL_JOINT_NAMES]
        self._joint_pub.publish(joint_msg)
        imu_msg = self.Imu()
        imu_msg.orientation.x, imu_msg.orientation.y = quat[0], quat[1]
        imu_msg.orientation.z, imu_msg.orientation.w = quat[2], quat[3]
        imu_msg.angular_velocity.x = gyro[0]
        imu_msg.angular_velocity.y = gyro[1]
        imu_msg.angular_velocity.z = gyro[2]
        self._imu_pub.publish(imu_msg)

    def publish_cmd_vel(self, vx=0.0, vy=0.0, wz=0.0):
        cmd = self.Twist()
        cmd.linear.x, cmd.linear.y, cmd.angular.z = vx, vy, wz
        self._cmd_pub.publish(cmd)

    def spin_for(self, seconds):
        """Pump the graph, re-publishing sensors at ~100 Hz."""
        deadline = time.monotonic() + seconds
        nominal = self.config.nominal_pose
        while time.monotonic() < deadline:
            self.publish_sensors(
                {j: float(v) for j, v in nominal.items()},
                {j: 0.05 for j in nominal})
            self.executor.spin_once(timeout_sec=0.01)

    def close(self):
        self.executor.remove_node(self.controller)
        self.executor.remove_node(self.helper)
        self.controller.destroy_node()
        self.helper.destroy_node()


@pytest.fixture(scope="module")
def graph(policy_config):
    try:
        rclpy.init()
    except RuntimeError:
        pass  # already initialized (e.g. re-run in a persistent session)
    harness = GraphHarness(policy_config)
    yield harness
    harness.close()
    try:
        rclpy.shutdown()
    except Exception:
        pass


@pytest.fixture(scope="module")
def effort_graph(policy_config):
    """The deployed default: the node closes the PD loop and publishes efforts.

    ``JointGroupEffortController`` has no kp/kd of its own, so on this interface
    the command topic carries the torques the node computed.
    """
    try:
        rclpy.init()
    except RuntimeError:
        pass  # already initialized (e.g. re-run in a persistent session)
    harness = GraphHarness(policy_config, command_interface="effort")
    yield harness
    harness.close()


def _targets(harness):
    """Latest published 22-vector keyed by canonical joint order."""
    assert harness.received, "no command messages received on the graph"
    _, msg = harness.received[-1]
    assert len(msg.data) == len(BHL_JOINT_NAMES)
    return list(msg.data)


def test_hold_measured_pose_before_first_cmd_vel(graph, policy_config):
    """Pre-command targets follow the *measured* pose, not the default pose.

    The measured pose is what the spawned robot is actually in (proven stable
    under zero commands); the config default pose bends the legs and stepping
    there instantly is the startup transient from evidence
    ``r53-bhl-actuation-2026-09-16``.
    """
    graph.received.clear()
    # Publish a distinctive measured pose; the hold must mirror it exactly.
    measured = {
        j: (0.1 if "hip_pitch" in j else 0.0) for j in BHL_JOINT_NAMES
    }
    deadline = time.monotonic() + 0.4
    while time.monotonic() < deadline:
        graph.publish_sensors(measured, {j: 0.0 for j in BHL_JOINT_NAMES})
        graph.executor.spin_once(timeout_sec=0.01)
    # Multiple messages at the policy rate, all equal to the measured pose.
    assert len(graph.received) >= 5
    for _, msg in graph.received:
        for joint, value in zip(BHL_JOINT_NAMES, msg.data):
            assert value == pytest.approx(measured[joint], abs=1e-9)


def test_cmd_vel_triggers_real_inference_at_policy_rate(graph):
    graph.received.clear()
    graph.publish_cmd_vel(vx=0.25)
    graph.spin_for(1.2)
    stamps = [t for t, _ in graph.received]
    # Rate is nominally 25 Hz; accept a generous lower bound for CI machines.
    assert len(stamps) >= 10
    rate = (len(stamps) - 1) / (stamps[-1] - stamps[0])
    assert 10.0 <= rate <= 40.0
    targets = np.asarray(_targets(graph))
    assert np.isfinite(targets).all()
    # A real policy output at 0.25 m/s is not exactly the zero-action pose.
    hold = np.asarray([graph.config.hold_pose()[j] for j in BHL_JOINT_NAMES])
    assert not np.allclose(targets, hold, atol=1e-6)


def test_joint_velocities_reach_the_observation_path(graph):
    """Non-zero JointState velocities are consumed, not silently dropped.

    A large positive dq on every joint with a stationary-measurement alias is
    indistinguishable in the published targets from zeros without inspecting
    the observation, so this test asserts the wiring contract instead: the
    node stores velocities from JointState and passes them through the
    controller update (verified by the pure-logic suite); here we confirm the
    graph end-to-end still produces finite targets with them present.
    """
    graph.received.clear()
    graph.publish_cmd_vel(vx=0.25)
    nominal = graph.config.nominal_pose
    deadline = time.monotonic() + 0.8
    while time.monotonic() < deadline:
        graph.publish_sensors(
            {j: float(v) for j, v in nominal.items()},
            {j: 1.0 for j in nominal})  # well-formed, non-zero dq
        graph.executor.spin_once(timeout_sec=0.01)
    # Wiring contract: the node cached the velocities from JointState...
    assert graph.controller._measured_velocities is not None
    assert graph.controller._measured_velocities["leg_left_hip_pitch_joint"] == 1.0
    # ...and the graph end-to-end still produces finite 22-vector targets.
    targets = np.asarray(_targets(graph))
    assert len(targets) == 22 and np.isfinite(targets).all()


def test_tipped_imu_latches_safe_stop_on_the_graph(graph, policy_config):
    graph.received.clear()
    graph.publish_cmd_vel(vx=0.25)
    graph.spin_for(0.3)  # inference running
    # Tip the robot beyond the 0.70 rad fall threshold and keep commanding.
    tilt = 0.75
    tipped = (math.sin(tilt / 2), 0.0, 0.0, math.cos(tilt / 2))
    nominal = graph.config.nominal_pose
    deadline = time.monotonic() + 0.8
    while time.monotonic() < deadline:
        graph.publish_sensors(
            {j: float(v) for j, v in nominal.items()},
            {j: 0.0 for j in nominal}, quat=tipped)
        graph.executor.spin_once(timeout_sec=0.01)
    # SAFE_STOP holds the default pose while cmd_vel keeps arriving.
    assert graph.controller._controller.safety_state == "SAFE_STOP"
    hold = [policy_config.hold_pose()[j] for j in BHL_JOINT_NAMES]
    recent = graph.received[-10:]
    assert recent, "no targets published after SAFE_STOP"
    for _, msg in recent:
        assert list(msg.data) == pytest.approx(hold, abs=1e-9)



# ---------------------------------------------------------------------------
# Effort interface (the deployed default)
# ---------------------------------------------------------------------------

def test_effort_interface_holds_with_zero_drive_before_cmd_vel(effort_graph):
    """The pre-command hold on the effort interface is *zero drive*.

    The node asks for ``tau = kp*(q* - q)`` with ``q*`` == the measured pose,
    so every published value is ~0 even though the measured joints are not.
    On the position interface those same values would equal the measured pose
    (proving the two interfaces really differ on the wire).
    """
    effort_graph.received.clear()
    measured = {j: (0.1 if "hip_pitch" in j else 0.0) for j in BHL_JOINT_NAMES}
    deadline = time.monotonic() + 0.4
    while time.monotonic() < deadline:
        effort_graph.publish_sensors(measured, {j: 0.0 for j in BHL_JOINT_NAMES})
        effort_graph.executor.spin_once(timeout_sec=0.01)
    assert effort_graph.received, "no command messages received on the graph"
    for _, msg in effort_graph.received:
        assert list(msg.data) == pytest.approx([0.0] * len(BHL_JOINT_NAMES), abs=1e-6)


def test_effort_interface_publishes_bounded_pd_efforts(effort_graph):
    """Once driving, the command topic carries finite, clamped torques.

    This is the contract the effort controller relies on: it forwards whatever
    it receives straight to the joints, so the node -- not the controller --
    must keep the values inside the +/-20 N.m URDF bound.
    """
    effort_graph.received.clear()
    effort_graph.publish_cmd_vel(vx=0.25)
    effort_graph.spin_for(1.2)
    efforts = np.asarray(_targets(effort_graph))
    assert len(efforts) == len(BHL_JOINT_NAMES)
    assert np.isfinite(efforts).all()
    assert np.any(np.abs(efforts) > 1e-9), "the PD loop is not driving"
    assert np.all(np.abs(efforts) <= EFFORT_LIMIT), "effort exceeded the URDF bound"


def test_settle_ramp_servo_uses_stance_gains_with_tilt_feedback(
        effort_graph, monkeypatch):
    """The settle ramp is servoed to *stand*, not with the checkpoint gains.

    The first live policy-servo run toppled during the measured -> default
    pose ramp: the checkpoint policy PD (kp=20/10, kd=2) gives an ankle
    restoring stiffness (~40 N.m/rad) far below the gravity topple stiffness
    (~71 N.m/rad) and has no tilt feedback at all. The ramp must instead use
    the balance core's stance PD (120/4 legs, 60/2 arms) plus the
    ankle-strategy correction on the blended ramp targets, so a growing tilt
    produces growing righting effort instead of a blind joint-space pull.
    """
    node = effort_graph.controller
    measured = {j: 0.0 for j in BHL_JOINT_NAMES}
    node._measured_positions = dict(measured)
    node._measured_velocities = {j: 0.0 for j in BHL_JOINT_NAMES}
    node._commanded = True  # a cmd_vel arrived; the ramp is running
    node._settle.start(measured)
    # Mid-ramp targets: halfway to the bent-knee default pose.
    node._targets, _ = node._settle.targets(measured, node._settle.duration_s / 2)
    published = []
    monkeypatch.setattr(node, "_publish", published.append)

    # Level body: stance PD on the ramp blend, far harder than kp=20.
    node._on_effort_timer()
    level = published[-1]
    knee = BHL_JOINT_NAMES.index("leg_left_knee_pitch_joint")
    default_knee = node._controller.config.hold_pose()["leg_left_knee_pitch_joint"]
    raw = 120.0 * 0.5 * default_knee  # stance Kp on the halfway blend
    assert level[knee] == pytest.approx(
        max(-EFFORT_LIMIT, min(EFFORT_LIMIT, raw)), abs=1e-9)
    assert abs(level[knee]) > 2.0  # the checkpoint kp=20 would give 2.0 N.m

    # Tipped body: the ankle-strategy correction must add righting effort on
    # the ankles (same-sign on both legs - the parallel-ankle requirement).
    node._orientation = (math.sin(0.1 / 2), 0.0, 0.0, math.cos(0.1 / 2))
    node._on_effort_timer()
    tipped = published[-1]
    ank_l = BHL_JOINT_NAMES.index("leg_left_ankle_roll_joint")
    ank_r = BHL_JOINT_NAMES.index("leg_right_ankle_roll_joint")
    assert tipped[ank_l] > level[ank_l] + 1e-9
    assert tipped[ank_r] > level[ank_r] + 1e-9
    # Arms swing out of phase with the roll (reaction).
    sh_l = BHL_JOINT_NAMES.index("arm_left_shoulder_pitch_joint")
    sh_r = BHL_JOINT_NAMES.index("arm_right_shoulder_pitch_joint")
    assert (tipped[sh_l] - level[sh_l]) * (tipped[sh_r] - level[sh_r]) < 0

    # Handover: once the ramp completes, the checkpoint gains drive again
    # (still bounded by the checkpoint effort limit, here 6 N.m on the legs).
    node._settle.finish()
    node._orientation = (0.0, 0.0, 0.0, 1.0)
    node._on_effort_timer()
    handover = published[-1]
    kp, kd, limit = node._pd["leg_left_knee_pitch_joint"]
    bound = min(float(limit), EFFORT_LIMIT)
    raw = kp * node._targets["leg_left_knee_pitch_joint"] - kd * 0.0
    assert handover[knee] == pytest.approx(
        max(-bound, min(bound, raw)), abs=1e-9)
    # ...and it is genuinely the checkpoint law, not the stance law: the
    # stance PD would command the full URDF-clamped 20 N.m here.
    assert abs(handover[knee]) < EFFORT_LIMIT



def test_effort_servo_uses_checkpoint_gains_and_fresh_state(effort_graph, monkeypatch):
    node = effort_graph.controller
    node._commanded = True
    node._targets = {j: .1 for j in BHL_JOINT_NAMES}
    node._measured_positions = {j: 0.0 for j in BHL_JOINT_NAMES}
    node._measured_velocities = {j: .1 for j in BHL_JOINT_NAMES}
    published = []
    monkeypatch.setattr(node, "_publish", published.append)
    node._on_effort_timer()
    expected = dict(zip(node._controller.config.joints,
                        node._controller.config.kp * .1 - node._controller.config.kd * .1))
    assert published[-1] == pytest.approx([expected[j] for j in BHL_JOINT_NAMES])
    # No policy decision between samples: the servo must see the new velocity.
    node._measured_velocities = {j: 100.0 for j in BHL_JOINT_NAMES}
    node._on_effort_timer()
    limits = dict(zip(node._controller.config.joints, node._controller.config.effort_limits))
    assert published[-1] == pytest.approx([-limits[j] for j in BHL_JOINT_NAMES])
    assert node._effort_timer.timer_period_ns == 4_000_000
    assert node._timer.timer_period_ns == 40_000_000


def test_effort_fall_stops_drive_even_between_policy_decisions(effort_graph, monkeypatch):
    node = effort_graph.controller
    node._targets = {j: .4 for j in BHL_JOINT_NAMES}
    node._commanded = True
    published = []
    monkeypatch.setattr(node, "_publish", published.append)
    node._orientation = (math.sin(.8 / 2), 0.0, 0.0, math.cos(.8 / 2))
    node._on_effort_timer()
    assert node._controller.safety_state == 'SAFE_STOP'
    assert published[-1] == [0.0] * 22
    node._orientation = (0.0, 0.0, 0.0, 1.0)
    node._on_effort_timer()
    assert published[-1] == [0.0] * 22


def test_recoil_torque_filter_wiring_is_opt_in_and_resets_on_stop():
    """The default path is unchanged; the opt-in path loads and applies EMA."""
    from robot_lab_adapter.bhl_policy import MOTOR_TORQUE_FILTER_ALPHA
    from robot_lab_adapter.humanoid_policy_controller import (
        HumanoidPolicyController)
    try:
        rclpy.init()
    except RuntimeError:
        pass
    off = HumanoidPolicyController(settle_duration_s=0.05)
    try:
        assert off._effort_filters is None
        assert off._filter_note() == ""
    finally:
        off.destroy_node()

    on = HumanoidPolicyController(
        settle_duration_s=0.05, torque_filter_enabled=True)
    try:
        filters = list(on._effort_filters.values())
        assert filters
        effective = 1.0 - (1.0 - MOTOR_TORQUE_FILTER_ALPHA) ** 8.0
        assert all(f.alpha == pytest.approx(effective) for f in filters)
        assert "Recoil torque EMA" in on._filter_note()

        on._commanded = True
        on._targets = {j: 0.1 for j in BHL_JOINT_NAMES}
        on._measured_positions = {j: 0.0 for j in BHL_JOINT_NAMES}
        on._measured_velocities = {j: 0.0 for j in BHL_JOINT_NAMES}
        published = []
        on._publish = published.append
        on._on_effort_timer()
        # BHL_JOINT_NAMES starts with a leg, whose checkpoint Kp is 20.
        assert published[-1][0] == pytest.approx(2.0 * effective)

        # Invalid feedback clears that channel rather than filtering stale
        # torque, and SAFE_STOP clears every channel before publishing zero.
        on._measured_positions.pop(BHL_JOINT_NAMES[0])
        on._on_effort_timer()
        assert published[-1][0] == 0.0
        on._measured_positions[BHL_JOINT_NAMES[0]] = 0.0
        on._on_effort_timer()
        assert published[-1][0] != 0.0
        on._orientation = (math.sin(0.8 / 2), 0.0, 0.0, math.cos(0.8 / 2))
        on._on_effort_timer()
        assert published[-1] == [0.0] * len(BHL_JOINT_NAMES)
        assert all(f.value == 0.0 for f in filters)
    finally:
        on.destroy_node()


def test_yaw_servo_wiring_is_opt_in_and_boost_only():
    """Node wiring for the yaw servo: off by default, direction preserved.

    The servo changes the policy's *observation* (the command block), not the
    action scaling, so it is invisible in the published targets; this asserts
    the wiring directly. With the default gain 0 the operator's command
    reaches the policy unchanged; with a gain the shortfall is boosted,
    clamped to the training range, and never reversed, so a released stick
    still commands exactly zero (the qualified stop path).
    """
    from robot_lab_adapter.bhl_policy import YAW_COMMAND_LIMIT
    from robot_lab_adapter.humanoid_policy_controller import (
        HumanoidPolicyController)
    try:
        rclpy.init()
    except RuntimeError:
        pass  # already initialized (e.g. re-run in a persistent session)
    off = HumanoidPolicyController(settle_duration_s=0.05)
    try:
        assert not off._yaw_servo.enabled
        assert off._servo_note() == ""
        off._command = [0.0, 0.0, 0.3]
        off._gyro = (0.0, 0.0, 0.0)
        assert off._policy_command() == [0.0, 0.0, 0.3]
    finally:
        off.destroy_node()
    on = HumanoidPolicyController(settle_duration_s=0.05, yaw_servo_gain=4.0)
    try:
        assert on._servo_note().startswith(", boost-only yaw servo gain=4")
        on._command = [0.0, 0.0, 0.3]
        on._gyro = (0.0, 0.0, 0.0)
        boosted = on._policy_command()
        assert boosted[0] == 0.0 and boosted[1] == 0.0
        assert boosted[2] == pytest.approx(YAW_COMMAND_LIMIT)
        # Non-turning is a full shortfall: the boost saturates upside.
        assert on._policy_command()[2] == pytest.approx(YAW_COMMAND_LIMIT)
        # Turning as commanded: the 80 ms measurement filter settles and the
        # boost fades back to the operator's reference.
        on._gyro = (0.0, 0.0, 0.3)
        for _ in range(12):
            tracking = on._policy_command()[2]
        assert tracking == pytest.approx(0.3, abs=0.02)
        # Released stick: exactly zero, never a command against the spin.
        on._command = [0.0, 0.0, 0.0]
        assert on._policy_command() == [0.0, 0.0, 0.0]
        on._command = [0.0, 0.0, -0.3]
        on._gyro = (0.0, 0.0, 0.3)
        assert on._policy_command()[2] < 0
    finally:
        on.destroy_node()
