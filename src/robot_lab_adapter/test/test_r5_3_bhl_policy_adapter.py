"""R5.3 tests: Berkeley Humanoid Lite ONNX policy adapter (bhl_policy).

Covers the pure-logic policy adapter layer that turns the vendored upstream
ONNX velocity checkpoints into URDF-clamped joint-position targets:

- Config loading enforces the verified feed-forward layout (9 + 3 * n
  observation dims over the ``action_indices`` joints) for both vendored
  policies: ``policy_humanoid`` (22 actions, 75 dims) and
  ``policy_humanoid_legs`` (12 leg actions, 45 dims), and rejects configs
  that disagree with it.
- Observation assembly: command clamped to the training ranges; gravity
  vector identical to ``qualify_policy.rotation_state``; q - default, dq and
  previous-action blocks in policy joint order.
- Action conversion: ``default + action_scale * clip(action, limits)`` for
  action joints, default pose for non-action joints, every target clamped to
  the vendored URDF position limits.
- The opt-in boost-only yaw servo (``YawRateBoost``): disabled by default,
  raises ``|wz|`` toward the training limit only while the low-passed measured
  body yaw rate falls short, never commands against the reference, and
  returns the reference unchanged on non-finite input.
- Controller with a stub inference session: observation passthrough to the
  session, hold-and-warn on missing measurements (no fabricated inference),
  latched SAFE_STOP on excessive tilt or invalid (NaN/wrong-shape) policy
  output, and explicit ``reset()`` as the only recovery.

No ROS graph, no onnxruntime, no MuJoCo: the session is dependency-injected.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

# Make robot_lab_adapter importable (mirrors the R5.2 / R5.3 harnesses).
_adapter_pkg = Path(__file__).resolve().parents[1]
if str(_adapter_pkg) not in sys.path:
    sys.path.insert(0, str(_adapter_pkg))

from robot_lab_adapter.bhl_balance import POSITION_LIMITS, TILT_FALL_RAD  # noqa: E402
from robot_lab_adapter.bhl_policy import (  # noqa: E402
    BhlPolicyController,
    StartupSettle,
    YAW_COMMAND_LIMIT,
    YawRateBoost,
    action_to_targets,
    build_observation,
    clamp_command,
    load_policy_config,
    quaternion_gravity,
    quaternion_tilt,
    upstream_dir,
)


# ----------------------------------------------------------------------------
# Fixtures
# ----------------------------------------------------------------------------

@pytest.fixture(scope="module")
def full_config():
    """The full-body policy config (22 actions, 75-dim observation)."""
    return load_policy_config(name="policy_humanoid")


@pytest.fixture(scope="module")
def legs_config():
    """The legs-only policy config (12 actions, 45-dim observation)."""
    return load_policy_config(name="policy_humanoid_legs")


@pytest.fixture(scope="module")
def nominal_measurements(full_config):
    """Measured joint positions exactly at the config default pose."""
    return {j: float(v) for j, v in full_config.nominal_pose.items()}


class StubIO:
    """Minimal onnxruntime input-metadata stand-in."""

    def __init__(self, name="observations"):
        self.name = name
        self.shape = [1, "obs"]


class StubSession:
    """Records every observation fed to ``run`` and returns zeros."""

    def __init__(self, action_dim=22, output=None):
        self.action_dim = action_dim
        self.output = output
        self.fed = []

    def get_inputs(self):
        return [StubIO()]

    def run(self, _unused, feed):
        self.fed.append(feed)
        if self.output is None:
            return [np.zeros((1, self.action_dim), dtype=np.float32)]
        return [self.output]


# ----------------------------------------------------------------------------
# Config loading
# ----------------------------------------------------------------------------

def test_upstream_dir_resolves_vendored_tree():
    upstream = upstream_dir()
    assert (upstream / "configs" / "policy_humanoid.yaml").is_file()


def test_full_body_config_layout(full_config):
    assert full_config.name == "policy_humanoid"
    assert len(full_config.joints) == 22
    assert full_config.action_indices == tuple(range(22))
    assert full_config.num_observations == 75
    assert full_config.action_scale == 0.25
    assert full_config.checkpoint.is_file()


def test_legs_config_layout(legs_config):
    assert legs_config.name == "policy_humanoid_legs"
    assert len(legs_config.joints) == 22
    assert legs_config.action_indices == tuple(range(10, 22))
    assert legs_config.action_joints == legs_config.joints[10:]
    assert legs_config.num_observations == 45
    assert legs_config.checkpoint.is_file()


def _write_config(tmp_path, name, **overrides):
    upstream = tmp_path / "upstream"
    (upstream / "configs").mkdir(parents=True)
    fields = {
        "policy_checkpoint_path": "checkpoints/x.onnx",
        "num_joints": 1,
        "joints": ["a"],
        "joint_kp": [1],
        "joint_kd": [1],
        "effort_limits": [1],
        "default_joint_positions": [0],
        "num_observations": 9 + 3,
        "history_length": 0,
        "num_actions": 1,
        "action_indices": [0],
        "action_scale": 0.25,
        "action_limit_lower": -1,
        "action_limit_upper": 1,
        "policy_dt": 0.04,
        "physics_dt": 0.005,
    }
    fields.update(overrides)
    lines = [f"{k}: {v}" for k, v in fields.items()]
    (upstream / "configs" / f"{name}.yaml").write_text("\n".join(lines) + "\n")
    return upstream


def test_config_rejects_non_feedforward_policy(tmp_path):
    upstream = _write_config(tmp_path, "recurrent", history_length=4)
    with pytest.raises(ValueError, match="feed-forward"):
        load_policy_config(upstream=upstream, name="recurrent")


def test_config_rejects_wrong_observation_size(tmp_path):
    upstream = _write_config(tmp_path, "bogus", num_observations=11)
    with pytest.raises(ValueError, match="num_observations"):
        load_policy_config(upstream=upstream, name="bogus")


def test_hold_pose_is_urdf_clamped(full_config):
    for joint, value in full_config.hold_pose().items():
        lo, hi = POSITION_LIMITS[joint]
        assert lo <= value <= hi


# ----------------------------------------------------------------------------
# Command / quaternion helpers
# ----------------------------------------------------------------------------

def test_clamp_command_to_training_ranges():
    assert list(clamp_command([5.0, -2.0, 9.0])) == [1.0, -0.5, 1.5]
    assert list(clamp_command([-5.0, 2.0, -9.0])) == [-1.0, 0.5, -1.5]
    assert list(clamp_command([0.25, 0.0, 0.1])) == [0.25, 0.0, 0.1]


def test_clamp_command_rejects_wrong_size():
    with pytest.raises(ValueError):
        clamp_command([1.0, 2.0])


def test_gravity_upright_matches_qualify_policy_convention():
    # qualify_policy.rotation_state unpacks w-first: (w, x, y, z) = (1, 0, 0, 0)
    assert np.allclose(quaternion_gravity(0.0, 0.0, 0.0, 1.0), [0.0, 0.0, -1.0])


def test_gravity_pitch_and_roll_90_degrees():
    half = math.sin(math.pi / 4)
    # Pitch +90 (x-forward tips down): gravity points +x in body frame.
    assert np.allclose(
        quaternion_gravity(0.0, half, 0.0, half), [1.0, 0.0, 0.0], atol=1e-12)
    # Roll +90 (right side down): gravity points -y in body frame.
    assert np.allclose(
        quaternion_gravity(half, 0.0, 0.0, half), [0.0, -1.0, 0.0], atol=1e-12)


def test_gravity_is_unit_length_for_unit_quaternions():
    half = math.sin(math.pi / 4)
    for quat in [(0.1, 0.2, 0.3, 0.9), (0.0, half, 0.0, half), (0.5, 0.5, 0.5, 0.5)]:
        assert abs(np.linalg.norm(quaternion_gravity(*quat)) - 1.0) < 1e-12



def test_gravity_rejects_zero_norm():
    with pytest.raises(ValueError):
        quaternion_gravity(0.0, 0.0, 0.0, 0.0)


def test_tilt_matches_qualify_policy_convention(full_config):
    assert quaternion_tilt(0.0, 0.0, 0.0, 1.0) == pytest.approx(0.0)
    half = math.sin(math.pi / 4)
    assert quaternion_tilt(half, 0.0, 0.0, half) == pytest.approx(math.pi / 2)
    # The probe's worst recorded tilt (0.052 rad) stays far below the fall threshold.
    assert quaternion_tilt(math.sin(0.026), 0.0, 0.0, math.cos(0.026)) < TILT_FALL_RAD


# ----------------------------------------------------------------------------
# Observation assembly
# ----------------------------------------------------------------------------

def test_build_observation_layout_and_dtype(full_config):
    na = [full_config.nominal[i] for i in full_config.action_indices]
    obs = build_observation(
        [0.25, 0.0, 0.0], [0.01, 0.02, 0.03], [0.0, 0.0, -1.0],
        na, np.full(22, 0.5), np.full(22, -0.25), na)
    assert obs.shape == (75,)
    assert obs.dtype == np.float32
    assert list(obs[0:3]) == [0.25, 0.0, 0.0]
    assert list(obs[3:6]) == pytest.approx([0.01, 0.02, 0.03], abs=1e-7)
    assert list(obs[6:9]) == [0.0, 0.0, -1.0]
    # q - default block is zero at the default pose...
    assert np.allclose(obs[9:31], 0.0)
    # ...dq and previous-action blocks pass through unchanged.
    assert np.allclose(obs[31:53], 0.5)
    assert np.allclose(obs[53:75], -0.25)


def test_build_observation_subtracts_default(full_config):
    na = [full_config.nominal[i] for i in full_config.action_indices]
    offset = [v + 0.1 for v in na]
    obs = build_observation([0, 0, 0], [0, 0, 0], [0, 0, -1],
                            offset, np.zeros(22), np.zeros(22), na)
    assert np.allclose(obs[9:31], 0.1)


def test_build_observation_rejects_wrong_sizes(full_config):
    na = [full_config.nominal[i] for i in full_config.action_indices]
    with pytest.raises(ValueError):
        build_observation([0, 0], [0, 0, 0], [0, 0, -1], na, np.zeros(22), np.zeros(22), na)
    with pytest.raises(ValueError):
        build_observation([0, 0, 0], [0, 0, 0], [0, 0, -1], na, np.zeros(21), np.zeros(22), na)


def test_legs_observation_is_45_dim(legs_config):
    na = [legs_config.nominal[i] for i in legs_config.action_indices]
    obs = build_observation([0, 0, 0], [0, 0, 0], [0, 0, -1],
                            na, np.zeros(12), np.zeros(12), na)
    assert obs.shape == (45,)


# ----------------------------------------------------------------------------
# Action -> position targets
# ----------------------------------------------------------------------------

def test_zero_action_targets_equal_hold_pose(full_config):
    assert action_to_targets(np.zeros(22), full_config) == full_config.hold_pose()


def test_action_offset_math(full_config):
    scale = full_config.action_scale  # 0.25
    targets = action_to_targets(np.full(22, 0.6), full_config)
    for joint, index in zip(full_config.action_joints, full_config.action_indices):
        expected = full_config.nominal[index] + scale * 0.6
        expected = max(POSITION_LIMITS[joint][0],
                       min(POSITION_LIMITS[joint][1], expected))
        assert targets[joint] == pytest.approx(expected, abs=1e-12)


def test_action_clamped_to_config_action_limits(full_config):
    # Upstream limits are +-10000; a raw action beyond them must not move the
    # target beyond default + scale * limit.
    limit = full_config.action_limit_upper
    targets = action_to_targets(np.full(22, limit * 10.0), full_config)
    for joint, index in zip(full_config.action_joints, full_config.action_indices):
        expected = max(POSITION_LIMITS[joint][0],
                       min(POSITION_LIMITS[joint][1],
                           full_config.nominal[index] + full_config.action_scale * limit))
        assert targets[joint] == pytest.approx(expected, abs=1e-12)


def test_extreme_action_targets_stay_inside_urdf_limits(full_config):
    for joint, value in action_to_targets(np.full(22, 1e6), full_config).items():
        lo, hi = POSITION_LIMITS[joint]
        assert lo <= value <= hi


def test_legs_policy_holds_arms_at_default_pose(legs_config):
    targets = action_to_targets(np.full(12, 5.0), legs_config)
    arm_joints = set(legs_config.joints[:10])
    for joint in arm_joints:
        assert targets[joint] == pytest.approx(
            float(legs_config.nominal[legs_config.joints.index(joint)]))
    # While every leg joint moved off its default.
    assert all(
        targets[j] != pytest.approx(float(legs_config.nominal[i]))
        for i, j in zip(legs_config.action_indices, legs_config.action_joints))


def test_action_to_targets_rejects_wrong_shape(full_config):
    with pytest.raises(ValueError):
        action_to_targets(np.zeros(12), full_config)


# ----------------------------------------------------------------------------
# Controller (stub inference session)
# ----------------------------------------------------------------------------

UPRIGHT = (0.0, 0.0, 0.0, 1.0)


def test_controller_runs_inference_and_passes_command_through(full_config):
    session = StubSession()
    controller = BhlPolicyController(full_config, session=session)
    cycle = controller.update([0.2, 0.1, 0.3], [0.0, 0.0, 0.0], UPRIGHT,
                              {j: float(v) for j, v in full_config.nominal_pose.items()})
    assert cycle.safety_state == "NOMINAL"
    assert cycle.issues == []
    assert len(session.fed) == 1
    fed = session.fed[0]["observations"]
    assert fed.shape == (1, 75)
    assert list(fed[0][0:3]) == pytest.approx([0.2, 0.1, 0.3], abs=1e-7)
    assert cycle.observation.shape == (75,)
    assert len(cycle.position_targets) == 22


def test_controller_clamps_oversized_command_before_inference(full_config):
    session = StubSession()
    controller = BhlPolicyController(full_config, session=session)
    controller.update([9.0, -9.0, 9.0], [0.0, 0.0, 0.0], UPRIGHT,
                      {j: float(v) for j, v in full_config.nominal_pose.items()})
    assert list(session.fed[0]["observations"][0][0:3]) == [1.0, -0.5, 1.5]


def test_controller_zero_action_holds_default_pose(full_config, nominal_measurements):
    controller = BhlPolicyController(full_config, session=StubSession())
    cycle = controller.update([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], UPRIGHT,
                              nominal_measurements)
    assert cycle.position_targets == full_config.hold_pose()


def test_controller_missing_measurement_holds_and_skips_inference(full_config):
    session = StubSession()
    controller = BhlPolicyController(full_config, session=session)
    measurements = {j: float(v) for j, v in full_config.nominal_pose.items()}
    del measurements["leg_right_ankle_roll_joint"]
    first = controller.update([0.2, 0, 0], [0, 0, 0], UPRIGHT, measurements)
    assert len(session.fed) == 0
    assert "warn" in first.issues[0]
    assert "leg_right_ankle_roll_joint" in first.issues[0]

    # A further cycle still holds the same targets, still without inference.
    second = controller.update([0.2, 0, 0], [0, 0, 0], UPRIGHT, measurements)
    assert len(session.fed) == 0
    assert second.position_targets == first.position_targets


def test_controller_latches_safe_stop_on_excessive_tilt(full_config, nominal_measurements):
    controller = BhlPolicyController(full_config, session=StubSession())
    tilt = TILT_FALL_RAD + 0.05
    cycle = controller.update(
        [0.2, 0, 0], [0, 0, 0],
        (math.sin(tilt / 2), 0.0, 0.0, math.cos(tilt / 2)), nominal_measurements)
    assert cycle.safety_state == "SAFE_STOP"
    assert "safe_stop" in cycle.issues[0]
    assert cycle.position_targets == full_config.hold_pose()


def test_safe_stop_is_latched_until_explicit_reset(full_config, nominal_measurements):
    controller = BhlPolicyController(full_config, session=StubSession())
    tilt = TILT_FALL_RAD + 0.05
    controller.update([0.2, 0, 0], [0, 0, 0],
                      (math.sin(tilt / 2), 0.0, 0.0, math.cos(tilt / 2)),
                      nominal_measurements)
    # Upright again but still latched: no inference runs.
    session = controller.session
    held = controller.update([0.2, 0, 0], [0, 0, 0], UPRIGHT, nominal_measurements)
    assert held.safety_state == "SAFE_STOP"
    assert session.fed == []

    controller.reset()
    resumed = controller.update([0.2, 0, 0], [0, 0, 0], UPRIGHT, nominal_measurements)
    assert resumed.safety_state == "NOMINAL"
    assert len(session.fed) == 1


def test_controller_latches_safe_stop_on_invalid_policy_output(
        full_config, nominal_measurements):
    controller = BhlPolicyController(
        full_config, session=StubSession(output=np.full((1, 22), np.nan)))
    cycle = controller.update([0.2, 0, 0], [0, 0, 0], UPRIGHT, nominal_measurements)
    assert cycle.safety_state == "SAFE_STOP"
    assert cycle.issues[0] == "safe_stop: invalid policy output"
    assert cycle.position_targets == full_config.hold_pose()


def test_controller_latches_safe_stop_on_wrong_shape_output(
        full_config, nominal_measurements):
    controller = BhlPolicyController(
        full_config, session=StubSession(output=np.zeros((1, 12))))
    cycle = controller.update([0.2, 0, 0], [0, 0, 0], UPRIGHT, nominal_measurements)
    assert cycle.safety_state == "SAFE_STOP"


def test_controller_feeds_previous_action_back(full_config, nominal_measurements):
    session = StubSession(output=np.full((1, 22), 0.5, dtype=np.float32))
    controller = BhlPolicyController(full_config, session=session)
    controller.update([0.2, 0, 0], [0, 0, 0], UPRIGHT, nominal_measurements)
    controller.update([0.2, 0, 0], [0, 0, 0], UPRIGHT, nominal_measurements)
    # The second observation's previous-action block (last 22 dims) is the
    # first (clipped) action.
    assert np.allclose(session.fed[1]["observations"][0][53:75], 0.5)


def test_controller_works_with_legs_policy(legs_config):
    measurements = {j: float(v) for j, v in legs_config.nominal_pose.items()}
    controller = BhlPolicyController(legs_config, session=StubSession(action_dim=12))
    cycle = controller.update([0.25, 0.0, 0.0], [0.0, 0.0, 0.0], UPRIGHT, measurements)
    assert cycle.safety_state == "NOMINAL"
    assert cycle.observation.shape == (45,)
    assert len(cycle.action) == 12
    assert set(cycle.position_targets) == set(legs_config.joints)


def test_policy_rate_matches_upstream_policy_dt(full_config):
    assert full_config.policy_dt == pytest.approx(1.0 / 25.0)


# ----------------------------------------------------------------------------
# StartupSettle: measured-pose hold + bounded ramp to the default pose
# ----------------------------------------------------------------------------

def _measured_spawn_pose(config):
    """The straight-legged spawn pose (every joint at zero)."""
    return {j: 0.0 for j in config.joints}


def test_settle_rejects_nonpositive_duration(full_config):
    with pytest.raises(ValueError, match="settle duration"):
        StartupSettle(full_config.hold_pose(), duration_s=0.0)
    with pytest.raises(ValueError, match="settle duration"):
        StartupSettle(full_config.hold_pose(), duration_s=-1.0)


def test_settle_hold_follows_the_measured_pose(full_config):
    """Pre-command targets mirror the measured pose (URDF-clamped)."""
    settle = StartupSettle(full_config.hold_pose(), duration_s=2.0)
    measured = _measured_spawn_pose(full_config)
    targets = settle.hold_targets(measured)
    assert set(targets) == set(full_config.joints)
    for joint, value in targets.items():
        assert value == pytest.approx(measured[joint], abs=1e-12)

    # Unmeasured joints fall back to the configured hold pose (clamped).
    partial = {j: v for j, v in measured.items() if "arm_" not in j}
    targets = settle.hold_targets(partial)
    for joint, value in targets.items():
        expected = partial.get(joint, full_config.hold_pose()[joint])
        lo, hi = POSITION_LIMITS[joint]
        assert value == pytest.approx(max(lo, min(hi, expected)), abs=1e-12)


def test_settle_hold_clamps_to_urdf_limits(full_config):
    """A measured pose outside the URDF range cannot be commanded verbatim."""
    settle = StartupSettle(full_config.hold_pose(), duration_s=2.0)
    joint = "leg_left_knee_pitch_joint"
    lo, hi = POSITION_LIMITS[joint]
    targets = settle.hold_targets({joint: lo - 1.0})
    assert targets[joint] == pytest.approx(lo, abs=1e-12)
    targets = settle.hold_targets({joint: hi + 1.0})
    assert targets[joint] == pytest.approx(hi, abs=1e-12)


def test_settle_ramp_runs_from_measured_to_default_pose(full_config):
    """targets() blends measured -> default linearly over duration_s."""
    measured = _measured_spawn_pose(full_config)
    default = full_config.hold_pose()
    settle = StartupSettle(default, duration_s=2.0)

    # Before start: measured-pose hold, never settled.
    targets, settled = settle.targets(measured, dt=1.0)
    assert not settled and not settle.settled
    assert targets["leg_left_knee_pitch_joint"] == pytest.approx(0.0, abs=1e-12)

    settle.start(measured)
    assert settle.active
    # Halfway through the ramp, the knee is halfway to its +0.4 default.
    targets, settled = settle.targets(measured, dt=1.0)
    assert not settled
    assert targets["leg_left_knee_pitch_joint"] == pytest.approx(
        0.5 * default["leg_left_knee_pitch_joint"], abs=1e-12)
    # Completing the duration settles and reaches the default pose.
    targets, settled = settle.targets(measured, dt=1.0)
    assert settled and settle.settled
    for joint, value in targets.items():
        assert value == pytest.approx(default[joint], abs=1e-12)


def test_settle_ramp_does_not_advance_before_start(full_config):
    """Repeated pre-start targets() calls never consume the ramp budget."""
    settle = StartupSettle(full_config.hold_pose(), duration_s=1.0)
    measured = _measured_spawn_pose(full_config)
    for _ in range(10):
        settle.targets(measured, dt=0.1)
    assert not settle.settled
    settle.start(measured)
    targets, settled = settle.targets(measured, dt=0.5)
    assert not settled  # only 0.5 s of the 1.0 s ramp consumed


def test_settle_finish_ends_the_ramp_immediately(full_config):
    """finish() is the tilt-abort path: hold pose from then on."""
    settle = StartupSettle(full_config.hold_pose(), duration_s=10.0)
    measured = _measured_spawn_pose(full_config)
    settle.start(measured)
    settle.targets(measured, dt=0.1)
    settle.finish()
    assert settle.settled and not settle.active
    targets, settled = settle.targets(measured, dt=1.0)
    assert settled
    # The ramp aborts to the *hold* pose (the same default-pose hold the
    # controller's SAFE_STOP path uses), not a half-ramped blend.
    for joint, value in targets.items():
        assert value == pytest.approx(full_config.hold_pose()[joint], abs=1e-12)


def test_settle_targets_stay_urdf_clamped(full_config):
    """No ramp point may command past the vendored URDF position limits."""
    # An origin far beyond the limits exercises the clamping on every blend.
    wild = {j: 10.0 for j in full_config.joints}
    settle = StartupSettle(full_config.hold_pose(), duration_s=1.0)
    settle.start(wild)
    for _ in range(12):
        targets, _ = settle.targets(wild, dt=0.1)
        for joint, value in targets.items():
            lo, hi = POSITION_LIMITS[joint]
            assert lo - 1e-12 <= value <= hi + 1e-12


# ----------------------------------------------------------------------------
# Opt-in boost-only yaw servo
# ----------------------------------------------------------------------------

def test_yaw_servo_disabled_passes_every_command_through():
    """Default (gain 0): the qualified open-loop command path is unchanged."""
    servo = YawRateBoost()
    assert not servo.enabled
    assert servo.limit == YAW_COMMAND_LIMIT
    assert servo.command(0.3, 0.0, 0.04) == 0.3
    assert servo.command(-0.3, 0.0, 0.04) == -0.3
    # The measurement filter still runs, but it cannot leak into the output.
    assert servo.command(0.0, 5.0, 0.04) == 0.0
    assert servo.command(0.75, -5.0, 0.04) == 0.75


def test_yaw_servo_boosts_only_in_the_reference_direction():
    """Boost-only: no deficit means no change, over-tracking never reverses."""
    servo = YawRateBoost(gain=4.0, filter_tau_s=0.0)  # unfiltered: alpha = 1
    assert servo.command(0.3, 0.0, 0.04) == pytest.approx(1.5)
    assert servo.command(0.3, 0.15, 0.04) == pytest.approx(0.9)
    assert servo.command(0.3, 0.3, 0.04) == pytest.approx(0.3)
    assert servo.command(0.3, 0.5, 0.04) == pytest.approx(0.3)
    assert servo.command(0.3, -0.5, 0.04) == pytest.approx(1.5)
    assert servo.command(-0.3, 0.0, 0.04) == pytest.approx(-1.5)
    assert servo.command(-0.3, 0.5, 0.04) == pytest.approx(-1.5)


def test_yaw_servo_boost_never_leaves_the_training_range():
    """A huge gain still cannot command past the trained yaw command."""
    servo = YawRateBoost(gain=50.0, filter_tau_s=0.0)
    assert servo.command(1.0, -1.0, 0.04) == YAW_COMMAND_LIMIT
    assert servo.command(-1.0, 1.0, 0.04) == -YAW_COMMAND_LIMIT
    with pytest.raises(ValueError):
        YawRateBoost(gain=1.0, limit=YAW_COMMAND_LIMIT + 0.1)


def test_yaw_servo_rejects_invalid_parameters():
    with pytest.raises(ValueError):
        YawRateBoost(gain=-1.0)
    with pytest.raises(ValueError):
        YawRateBoost(limit=0.0)
    with pytest.raises(ValueError):
        YawRateBoost(filter_tau_s=-0.1)
    with pytest.raises(ValueError):
        YawRateBoost(gain=float('nan'))


def test_yaw_servo_filters_the_measured_rate_and_reset_clears_it():
    """25 Hz cycles at tau 80 ms: alpha 0.5, so the deficit halves each cycle."""
    servo = YawRateBoost(gain=1.0, filter_tau_s=0.08)
    assert servo.command(0.3, 0.3, 0.04) == pytest.approx(0.45)
    assert servo.command(0.3, 0.3, 0.04) == pytest.approx(0.375)
    assert servo.command(0.3, 0.3, 0.04) == pytest.approx(0.3375)
    assert servo.command(0.0, 0.0, 0.04) == 0.0
    servo.command(0.3, 0.3, 0.04)
    servo.reset()
    assert servo.command(0.3, 0.3, 0.04) == pytest.approx(0.45)


def test_yaw_servo_never_fabricates_a_correction():
    """Non-finite inputs fall back to the operator's command unchanged."""
    servo = YawRateBoost(gain=4.0)
    assert servo.command(0.3, float('nan'), 0.04) == 0.3
    assert servo.command(0.3, 0.0, float('inf')) == 0.3
    assert math.isnan(servo.command(float('nan'), 0.0, 0.04))



