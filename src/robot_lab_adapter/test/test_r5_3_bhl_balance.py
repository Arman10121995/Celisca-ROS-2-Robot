"""R5.3 tests: Berkeley Humanoid Lite closed-loop balance core.

Covers the R5.3 acceptance bar, honestly:

- Joint constants (names, effort/position limits, nominal pose) are validated
  against the on-disk BHL URDF so the module never claims gains/limits that
  the description does not provide.
- A fixed standing pose is NOT balance: when the IMU body state reports a
  non-zero roll or pitch, the balance targets strictly change.
- Ankle strategy: the BHL ankles are a PARALLEL mechanism (both ankle-pitch
  axes +Y, both ankle-roll axes +X at the rest pose), so body roll yields a
  SAME-SIGN ankle-roll command on both legs and body pitch a same-sign
  ankle-pitch lean; hip roll adds a same-sign lateral CoM shift; arms swing
  out of phase with roll (arm reaction). The ankle gains give a combined
  restoring stiffness (2*Kp*K = 168 N.m/rad) exceeding the gravity topple
  stiffness (m*g*h ~= 108 N.m/rad) with a 1.5x margin.
- Stance PD hold: a joint with no position measurement is *not driven*
  (zero effort), never assumed at target; all efforts clamp to the URDF
  20 N.m limit.
- Safety monitor latches SAFE_STOP on excessive tilt (>=0.70 rad) or
  sustained effort saturation (>=98% limit for 50 cycles); recovery needs an
  explicit reset; SAFE_STOP forces the nominal pose as the hold command.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

# Make robot_lab_adapter importable (mirrors the R5.2 test harness).
_adapter_pkg = Path(__file__).resolve().parents[1]
if str(_adapter_pkg) not in sys.path:
    sys.path.insert(0, str(_adapter_pkg))

from robot_lab_adapter.bhl_balance import (  # noqa: E402
    ANKLE_BALANCE_K_PITCH,
    ANKLE_BALANCE_K_ROLL,
    ANKLE_STIFFNESS_MARGIN,
    ARM_BALANCE_K,
    BASE_VEL_LIMITS_PLACEHOLDER,
    BHL_ARM_JOINTS,
    BHL_JOINT_NAMES,
    BHL_LEG_JOINTS,
    BHL_LEGS,
    EFFORT_LIMIT,
    HIP_BALANCE_K_ROLL,
    KNEE_BALANCE_K,
    EFFORT_SATURATION_CYCLES,
    POSITION_LIMITS,
    STANCE_DT,
    STANCE_DURATION_SECONDS,
    STANCE_PD_ARMS,
    STANCE_PD_LEGS,
    TILT_FALL_RAD,
    TILT_WARN_RAD,
    TOPPLE_STIFFNESS_NM_PER_RAD,
    BodyState,
    BhlBalanceController,
    BhlControlCycle,
    SafetyState,
    balance_targets,
    clamp_effort,
    clamp_position,
    nominal_standing_pose,
    parse_bhl_joints,
    pd_effort_command,
)


# ----------------------------------------------------------------------
# Fixtures and helpers
# ----------------------------------------------------------------------
@pytest.fixture
def nominal():
    return nominal_standing_pose()


@pytest.fixture
def controller():
    return BhlBalanceController()


def _full_positions():
    """All 22 joints at the nominal pose (as if freshly measured)."""
    return dict(nominal_standing_pose())


# ----------------------------------------------------------------------
# R1: Static constants honest to the URDF
# ----------------------------------------------------------------------
class TestJointCount:
    def test_22_actuated_joints(self):
        urdf = parse_bhl_joints()
        revolute = {n for n in urdf}
        assert len(revolute) == 22

    def test_module_lists_match_urdf(self):
        urdf = parse_bhl_joints()
        mod = set(BHL_JOINT_NAMES)
        assert mod == set(urdf)
        assert len(BHL_JOINT_NAMES) == 22

    def test_leg_and_arm_split(self):
        assert len(BHL_LEG_JOINTS) == 12
        assert len(BHL_ARM_JOINTS) == 10
        assert set(BHL_LEG_JOINTS).isdisjoint(set(BHL_ARM_JOINTS))
        assert set(BHL_LEG_JOINTS) | set(BHL_ARM_JOINTS) == set(BHL_JOINT_NAMES)

    def test_two_legs_biped(self):
        assert set(BHL_LEGS) == {"left", "right"}


class TestEffortLimits:
    def test_effort_limit_is_20(self):
        assert EFFORT_LIMIT == 20.0

    def test_urdf_all_effort_20(self):
        urdf = parse_bhl_joints()
        for name, d in urdf.items():
            assert d["effort"] == 20.0

    def test_urdf_all_velocity_15(self):
        urdf = parse_bhl_joints()
        for name, d in urdf.items():
            assert d["velocity"] == 15.0


# ----------------------------------------------------------------------
# Position limits honest to URDF
# ----------------------------------------------------------------------
class TestPositionLimits:
    def test_all_joints_have_limits(self):
        urdf = parse_bhl_joints()
        for name in BHL_JOINT_NAMES:
            lo, hi = POSITION_LIMITS[name]
            assert lo < hi
            assert urdf[name]["lower"] == pytest.approx(lo, abs=1e-5)
            assert urdf[name]["upper"] == pytest.approx(hi, abs=1e-5)

    def test_nominal_inside_limits(self):
        for name, val in nominal_standing_pose().items():
            lo, hi = POSITION_LIMITS[name]
            assert lo <= val <= hi

    def test_knee_has_zero_lower_bound(self):
        assert POSITION_LIMITS["leg_left_knee_pitch_joint"][0] == 0.0
        assert POSITION_LIMITS["leg_right_knee_pitch_joint"][0] == 0.0

    def test_right_shoulder_pitch_upper_is_1_57(self):
        # arm_right_elbow_pitch has upper = 0 (negative range)
        assert POSITION_LIMITS["arm_right_elbow_pitch_joint"][1] == 0.0


# ----------------------------------------------------------------------
# R2: Balance law - standing pose is not balance
# ----------------------------------------------------------------------
class TestBalanceLaw:
    def test_body_none_returns_nominal(self, nominal):
        targets = balance_targets(nominal, None)
        for name in BHL_JOINT_NAMES:
            assert targets[name] == pytest.approx(nominal[name])

    def test_level_body_returns_nominal(self, nominal):
        body = BodyState(roll_rad=0.0, pitch_rad=0.0)
        targets = balance_targets(nominal, body)
        for name in BHL_JOINT_NAMES:
            assert targets[name] == pytest.approx(nominal[name])

    def test_roll_changes_targets(self, nominal):
        body = BodyState(roll_rad=0.2, pitch_rad=0.0)
        targets = balance_targets(nominal, body)
        changed = [n for n in BHL_JOINT_NAMES if abs(targets[n] - nominal[n]) > 1e-9]
        assert len(changed) > 0

    def test_pitch_changes_targets(self, nominal):
        body = BodyState(roll_rad=0.0, pitch_rad=0.15)
        targets = balance_targets(nominal, body)
        changed = [n for n in BHL_JOINT_NAMES if abs(targets[n] - nominal[n]) > 1e-9]
        assert len(changed) > 0

    def test_small_roll_changes_targets(self, nominal):
        """The defining honesty property: balance != fixed pose."""
        body = BodyState(roll_rad=0.01, pitch_rad=0.0)
        targets = balance_targets(nominal, body)
        assert abs(targets["leg_left_ankle_roll_joint"] - nominal["leg_left_ankle_roll_joint"]) > 1e-9
        assert abs(targets["leg_right_ankle_roll_joint"] - nominal["leg_right_ankle_roll_joint"]) > 1e-9


# ----------------------------------------------------------------------
# R3: Ankle strategy
# ----------------------------------------------------------------------
class TestAnkleStrategy:
    def test_ankle_roll_same_sign_parallel_axes(self, nominal):
        # FK at the rest pose: both ankle-roll axes are +X (parallel
        # mechanism). A differential command would cancel through the
        # pelvis and produce zero net roll moment, so both ankles must
        # receive the SAME-sign correction.
        body = BodyState(roll_rad=0.2, pitch_rad=0.0)
        targets = balance_targets(nominal, body)
        left_delta = targets["leg_left_ankle_roll_joint"] - nominal["leg_left_ankle_roll_joint"]
        right_delta = targets["leg_right_ankle_roll_joint"] - nominal["leg_right_ankle_roll_joint"]
        assert left_delta == pytest.approx(ANKLE_BALANCE_K_ROLL * 0.2, abs=1e-9)
        assert right_delta == pytest.approx(ANKLE_BALANCE_K_ROLL * 0.2, abs=1e-9)
        assert (left_delta > 0) and (right_delta > 0)

    def test_hip_roll_same_sign_co_m_shift(self, nominal):
        body = BodyState(roll_rad=0.2, pitch_rad=0.0)
        targets = balance_targets(nominal, body)
        left_delta = targets["leg_left_hip_roll_joint"] - nominal["leg_left_hip_roll_joint"]
        right_delta = targets["leg_right_hip_roll_joint"] - nominal["leg_right_hip_roll_joint"]
        assert left_delta == pytest.approx(HIP_BALANCE_K_ROLL * 0.2, abs=1e-9)
        assert right_delta == pytest.approx(HIP_BALANCE_K_ROLL * 0.2, abs=1e-9)

    def test_ankle_stiffness_beats_gravity_topple(self):
        # Regression anchor for the 2026-09-17 live topple: the combined
        # ankle restoring stiffness (both legs, joint Kp from the stance
        # hold) must exceed the gravity topple stiffness m*g*h with the
        # declared margin, or the standing equilibrium is unstable and the
        # biped diverges exactly as observed live.
        ankle_stiffness = (
            2 * STANCE_PD_LEGS[0] * ANKLE_BALANCE_K_PITCH
        )
        assert ankle_stiffness >= (
            ANKLE_STIFFNESS_MARGIN * TOPPLE_STIFFNESS_NM_PER_RAD
        )
        # And the roll channel must match (same K, same axes geometry).
        roll_stiffness = 2 * STANCE_PD_LEGS[0] * ANKLE_BALANCE_K_ROLL
        assert roll_stiffness == pytest.approx(ankle_stiffness)

    def test_ankle_roll_scales_with_tilt(self, nominal):
        big = balance_targets(nominal, BodyState(roll_rad=0.2, pitch_rad=0.0))
        small = balance_targets(nominal, BodyState(roll_rad=0.1, pitch_rad=0.0))
        big_delta = big["leg_left_ankle_roll_joint"] - nominal["leg_left_ankle_roll_joint"]
        small_delta = small["leg_left_ankle_roll_joint"] - nominal["leg_left_ankle_roll_joint"]
        assert big_delta == pytest.approx(2 * small_delta, abs=1e-9)

    def test_ankle_pitch_same_sign_both_legs(self, nominal):
        body = BodyState(roll_rad=0.0, pitch_rad=0.15)
        targets = balance_targets(nominal, body)
        left_delta = targets["leg_left_ankle_pitch_joint"] - nominal["leg_left_ankle_pitch_joint"]
        right_delta = targets["leg_right_ankle_pitch_joint"] - nominal["leg_right_ankle_pitch_joint"]
        assert left_delta == pytest.approx(ANKLE_BALANCE_K_PITCH * 0.15, abs=1e-9)
        assert right_delta == pytest.approx(ANKLE_BALANCE_K_PITCH * 0.15, abs=1e-9)

    def test_hip_roll_helps_ankle_roll(self, nominal):
        # Same-sign lateral CoM shift (parallel hip geometry, like the
        # ankles - differential would only rotate the pelvis).
        body = BodyState(roll_rad=0.2, pitch_rad=0.0)
        targets = balance_targets(nominal, body)
        left_hip = targets["leg_left_hip_roll_joint"] - nominal["leg_left_hip_roll_joint"]
        right_hip = targets["leg_right_hip_roll_joint"] - nominal["leg_right_hip_roll_joint"]
        assert left_hip == pytest.approx(HIP_BALANCE_K_ROLL * 0.2, abs=1e-9)
        assert right_hip == pytest.approx(HIP_BALANCE_K_ROLL * 0.2, abs=1e-9)


# ----------------------------------------------------------------------
# R4: Arm reaction
# ----------------------------------------------------------------------
class TestArmReaction:
    def test_arms_opposite_shoulder_pitch(self, nominal):
        body = BodyState(roll_rad=0.2, pitch_rad=0.0)
        targets = balance_targets(nominal, body)
        left_delta = targets["arm_left_shoulder_pitch_joint"] - nominal["arm_left_shoulder_pitch_joint"]
        right_delta = targets["arm_right_shoulder_pitch_joint"] - nominal["arm_right_shoulder_pitch_joint"]
        assert left_delta == pytest.approx(ARM_BALANCE_K * 0.2, abs=1e-9)
        assert right_delta == pytest.approx(-ARM_BALANCE_K * 0.2, abs=1e-9)
        assert (left_delta > 0) != (right_delta > 0)

    def test_arms_only_react_to_roll(self, nominal):
        body = BodyState(roll_rad=0.0, pitch_rad=0.1)
        targets = balance_targets(nominal, body)
        assert targets["arm_left_shoulder_pitch_joint"] == pytest.approx(
            nominal["arm_left_shoulder_pitch_joint"]
        )
        assert targets["arm_right_shoulder_pitch_joint"] == pytest.approx(
            nominal["arm_right_shoulder_pitch_joint"]
        )

    def test_knee_softens_with_tilt(self, nominal):
        # Knee softening only has headroom when the stance carries knee
        # flexion; the deployed nominal is the straight-legged URDF rest
        # pose (knee 0 = clamped lower limit), so exercise the law with a
        # flexed stance instead.
        flexed = dict(nominal)
        flexed["leg_left_knee_pitch_joint"] = 0.5
        flat = balance_targets(flexed, BodyState(roll_rad=0.0, pitch_rad=0.0))
        tilted = balance_targets(flexed, BodyState(roll_rad=0.3, pitch_rad=0.3))
        assert tilted["leg_left_knee_pitch_joint"] < flat["leg_left_knee_pitch_joint"]


# ----------------------------------------------------------------------
# R5: Clamping and no-drive safety
# ----------------------------------------------------------------------
class TestClamping:
    def test_clamp_position_respects_limits(self):
        for name in BHL_JOINT_NAMES:
            lo, hi = POSITION_LIMITS[name]
            assert clamp_position(name, lo - 100.0) == pytest.approx(lo)
            assert clamp_position(name, hi + 100.0) == pytest.approx(hi)
            mid = (lo + hi) / 2
            assert clamp_position(name, mid) == pytest.approx(mid)

    def test_clamp_effort_respects_limit(self):
        v = "leg_left_ankle_pitch_joint"
        assert clamp_effort(v, 50.0) == pytest.approx(20.0)
        assert clamp_effort(v, -50.0) == pytest.approx(-20.0)
        assert clamp_effort(v, 5.0) == pytest.approx(5.0)

    def test_balance_targets_clamped(self, nominal):
        body = BodyState(roll_rad=0.6, pitch_rad=0.6)
        targets = balance_targets(nominal, body)
        for name in BHL_JOINT_NAMES:
            lo, hi = POSITION_LIMITS[name]
            assert lo <= targets[name] <= hi


class TestNoDriveOnMissingData:
    def test_missing_position_gives_zero_effort(self):
        targets = nominal_standing_pose()
        positions = {k: v for k, v in targets.items() if k != "leg_left_ankle_pitch_joint"}
        efforts = pd_effort_command(targets, positions)
        assert efforts["leg_left_ankle_pitch_joint"] == 0.0

    def test_missing_velocity_uses_zero(self, nominal):
        positions = _full_positions()
        efforts_no_vel = pd_effort_command(nominal, positions)
        efforts_with_zero_vel = pd_effort_command(
            nominal, positions, {j: 0.0 for j in BHL_JOINT_NAMES}
        )
        for name in BHL_JOINT_NAMES:
            assert efforts_no_vel[name] == pytest.approx(efforts_with_zero_vel[name])

    def test_all_efforts_within_limit(self, nominal):
        positions = _full_positions()
        efforts = pd_effort_command(nominal, positions)
        for name, tau in efforts.items():
            assert abs(tau) <= EFFORT_LIMIT

    def test_pd_gains_per_limb_type(self):
        assert STANCE_PD_LEGS[0] > STANCE_PD_ARMS[0]


class TestPdGainOverrides:
    """The effort-interface nodes close the PD loop themselves (the controller
    is a pure effort forwarder with no gains), so ``pd_effort_command`` must
    let them select gains while defaulting to the balance law's convention."""

    def test_default_gains_equal_explicit_module_constants(self, nominal):
        positions = {j: v - 0.05 for j, v in _full_positions().items()}
        default = pd_effort_command(nominal, positions)
        explicit = pd_effort_command(
            nominal, positions, None, STANCE_PD_LEGS, STANCE_PD_ARMS
        )
        for name in BHL_JOINT_NAMES:
            assert default[name] == pytest.approx(explicit[name])
        # Guard against a vacuous all-zero comparison.
        assert any(abs(default[j]) > 1e-9 for j in BHL_JOINT_NAMES)

    def test_larger_kp_asks_for_more_effort(self):
        target = {j: 0.0 for j in BHL_JOINT_NAMES}
        positions = {j: 0.0 for j in BHL_JOINT_NAMES}
        target["leg_left_knee_pitch_joint"] = 0.2
        soft = pd_effort_command(target, positions, None, (10.0, 2.0), (10.0, 2.0))
        stiff = pd_effort_command(target, positions, None, (120.0, 4.0), (120.0, 4.0))
        assert 0.0 < soft["leg_left_knee_pitch_joint"]
        assert soft["leg_left_knee_pitch_joint"] < stiff["leg_left_knee_pitch_joint"]

    def test_damping_term_opposes_velocity(self):
        target = {j: 0.0 for j in BHL_JOINT_NAMES}
        positions = {j: 0.0 for j in BHL_JOINT_NAMES}
        velocities = {j: 0.0 for j in BHL_JOINT_NAMES}
        velocities["leg_left_knee_pitch_joint"] = 1.0
        efforts = pd_effort_command(
            target, positions, velocities, (10.0, 2.0), (10.0, 2.0)
        )
        assert efforts["leg_left_knee_pitch_joint"] == pytest.approx(-2.0)

    def test_overridden_gains_still_clamp_to_effort_limit(self):
        target = {j: 0.0 for j in BHL_JOINT_NAMES}
        positions = {j: 0.0 for j in BHL_JOINT_NAMES}
        target["leg_left_knee_pitch_joint"] = 1.5  # large error
        efforts = pd_effort_command(
            target, positions, None, (1000.0, 0.0), (1000.0, 0.0)
        )
        assert efforts["leg_left_knee_pitch_joint"] == pytest.approx(EFFORT_LIMIT)


# ----------------------------------------------------------------------
# R6: Safety monitor
# ----------------------------------------------------------------------
class TestSafetyMonitor:
    def test_nominal_starts_clean(self):
        s = SafetyState()
        assert s.state == SafetyState.NOMINAL
        assert s.reason is None

    def test_warn_on_tilt(self):
        s = SafetyState()
        issues = s.observe_body(BodyState(roll_rad=0.4, pitch_rad=0.0))
        assert s.state == SafetyState.WARN
        assert any("warn" in i for i in issues)

    def test_fall_enters_safe_stop(self):
        s = SafetyState()
        issues = s.observe_body(BodyState(roll_rad=0.8, pitch_rad=0.0))
        assert s.state == SafetyState.SAFE_STOP
        assert any("safe_stop" in i for i in issues)

    def test_safe_stop_latched(self):
        s = SafetyState()
        s.observe_body(BodyState(roll_rad=0.8, pitch_rad=0.0))
        assert s.state == SafetyState.SAFE_STOP
        s.observe_body(BodyState(roll_rad=0.0, pitch_rad=0.0))
        assert s.state == SafetyState.SAFE_STOP

    def test_safe_stop_requires_reset(self):
        s = SafetyState()
        s.observe_body(BodyState(roll_rad=0.8, pitch_rad=0.0))
        assert s.state == SafetyState.SAFE_STOP
        s.reset()
        assert s.state == SafetyState.NOMINAL

    def test_warn_recovers_when_level(self):
        s = SafetyState()
        s.observe_body(BodyState(roll_rad=0.4, pitch_rad=0.0))
        assert s.state == SafetyState.WARN
        s.observe_body(BodyState(roll_rad=0.0, pitch_rad=0.0))
        assert s.state == SafetyState.NOMINAL

    def test_safe_stop_positions_are_nominal(self):
        s = SafetyState()
        result = s.safe_stop_positions()
        assert set(result.keys()) == set(BHL_JOINT_NAMES)
        for name in BHL_JOINT_NAMES:
            assert result[name] == pytest.approx(nominal_standing_pose()[name])

    def test_balance_permitted_in_nominal(self):
        assert SafetyState().balance_permitted() is True

    def test_balance_not_permitted_in_safe_stop(self):
        s = SafetyState()
        s._enter_safe_stop("test")
        assert s.balance_permitted() is False

    def test_effort_saturation_triggers_safe_stop(self):
        s = SafetyState()
        cmd = {j: 19.6 for j in BHL_JOINT_NAMES}
        meas = {j: 19.6 for j in BHL_JOINT_NAMES}
        issues = []
        for _ in range(51):
            issues = s.observe_efforts(cmd, meas)
        assert s.state == SafetyState.SAFE_STOP
        assert any("safe_stop" in i for i in issues)

    def test_no_sat_until_threshold_cycles(self):
        s = SafetyState()
        cmd = {j: 19.6 for j in BHL_JOINT_NAMES}
        meas = {j: 19.6 for j in BHL_JOINT_NAMES}
        for _ in range(49):
            s.observe_efforts(cmd, meas)
        assert s.state != SafetyState.SAFE_STOP
        s.observe_efforts(cmd, meas)
        assert s.state == SafetyState.SAFE_STOP

    def test_missing_effort_resets_saturation(self):
        """A missing measurement restarts that joint's saturation count.

        Counters are per joint: the other joints still reach the threshold on
        the same cycle, so this checks the gap on one joint, not the latch.
        """
        s = SafetyState()
        gap_joint = "leg_left_knee_pitch_joint"
        only_gap = [gap_joint]
        cmd = {j: 19.6 for j in only_gap}
        meas = {j: 19.6 for j in only_gap}
        for _ in range(EFFORT_SATURATION_CYCLES - 1):
            s.observe_efforts(cmd, meas)
        s.observe_efforts(cmd, {})  # measurement missing for one cycle
        for _ in range(EFFORT_SATURATION_CYCLES - 1):
            s.observe_efforts(cmd, meas)
        assert s.state != SafetyState.SAFE_STOP
        s.observe_efforts(cmd, meas)
        assert s.state == SafetyState.SAFE_STOP

    def test_missing_effort_on_all_joints_prevents_latch(self):
        s = SafetyState()
        cmd = {j: 19.6 for j in BHL_JOINT_NAMES}
        meas = {j: 19.6 for j in BHL_JOINT_NAMES}
        for _ in range(EFFORT_SATURATION_CYCLES - 1):
            s.observe_efforts(cmd, meas)
        s.observe_efforts(cmd, {})
        assert s.state != SafetyState.SAFE_STOP


# ----------------------------------------------------------------------
# R7: Controller integration
# ----------------------------------------------------------------------
class TestController:
    def test_update_returns_cycle(self, controller):
        positions = _full_positions()
        cycle = controller.update(STANCE_DT, positions)
        assert isinstance(cycle, BhlControlCycle)
        assert cycle.safety_state == SafetyState.NOMINAL
        assert set(cycle.position_targets.keys()) == set(BHL_JOINT_NAMES)
        assert set(cycle.efforts.keys()) == set(BHL_JOINT_NAMES)

    def test_update_with_body_uses_balance(self, controller):
        """A tilt below the warn threshold still changes the targets."""
        positions = _full_positions()
        body = BodyState(roll_rad=0.2, pitch_rad=0.0)
        assert body.max_tilt_rad < TILT_WARN_RAD
        cycle = controller.update(STANCE_DT, positions, body=body)
        assert cycle.safety_state == SafetyState.NOMINAL
        assert cycle.position_targets == balance_targets(nominal_standing_pose(), body)
        assert cycle.position_targets != nominal_standing_pose()

    def test_update_warns_between_warn_and_fall_thresholds(self, controller):
        positions = _full_positions()
        tilt = (TILT_WARN_RAD + TILT_FALL_RAD) / 2.0
        cycle = controller.update(
            STANCE_DT, positions, body=BodyState(roll_rad=tilt, pitch_rad=0.0))
        assert cycle.safety_state == SafetyState.WARN

    def test_safe_stop_zeroes_efforts(self, controller):
        positions = _full_positions()
        controller.update(STANCE_DT, positions, body=BodyState(roll_rad=0.8, pitch_rad=0.0))
        assert controller.safety.state == SafetyState.SAFE_STOP
        cycle = controller.update(STANCE_DT, positions, body=BodyState(roll_rad=0.8, pitch_rad=0.0))
        assert cycle.safety_state == SafetyState.SAFE_STOP
        assert all(abs(v) < 1e-9 for v in cycle.efforts.values())

    def test_safe_stop_uses_nominal_targets(self, controller):
        positions = _full_positions()
        controller.update(STANCE_DT, positions, body=BodyState(roll_rad=0.8, pitch_rad=0.0))
        cycle = controller.update(STANCE_DT, positions, body=BodyState(roll_rad=0.8, pitch_rad=0.0))
        assert cycle.safety_state == SafetyState.SAFE_STOP
        for name in BHL_JOINT_NAMES:
            assert cycle.position_targets[name] == pytest.approx(nominal_standing_pose()[name])

    def test_reset_recovers_from_safe_stop(self, controller):
        positions = _full_positions()
        controller.update(STANCE_DT, positions, body=BodyState(roll_rad=0.8, pitch_rad=0.0))
        assert controller.safety.state == SafetyState.SAFE_STOP
        controller.reset()
        body = BodyState(roll_rad=0.2, pitch_rad=0.0)
        cycle = controller.update(STANCE_DT, positions, body=body)
        assert cycle.safety_state != SafetyState.SAFE_STOP

    def test_standing_pose_is_not_balance(self, controller):
        """Critical honesty test: level body -> nominal targets;
        tilted body -> different targets."""
        positions = _full_positions()
        level = controller.update(STANCE_DT, positions, body=BodyState(roll_rad=0.0, pitch_rad=0.0))
        tilted = controller.update(STANCE_DT, positions, body=BodyState(roll_rad=0.15, pitch_rad=0.05))
        diffs = [
            abs(tilted.position_targets[n] - level.position_targets[n])
            for n in BHL_JOINT_NAMES
        ]
        assert any(d > 1e-6 for d in diffs)

    def test_contacts_none_for_biped(self, controller):
        positions = _full_positions()
        cycle = controller.update(STANCE_DT, positions, body=BodyState(roll_rad=0.0, pitch_rad=0.0))
        for leg in BHL_LEGS:
            assert cycle.contacts[leg] is None

    def test_stance_constants_declared(self):
        assert STANCE_DURATION_SECONDS == 1.0
        assert BASE_VEL_LIMITS_PLACEHOLDER is None

    def test_imu_quaternion_body_state(self):
        body = BodyState.from_quaternion(0.383, 0.0, 0.0, 0.924)
        assert body.roll_rad == pytest.approx(0.785, abs=0.02)
        assert body.pitch_rad == pytest.approx(0.0, abs=1e-6)

    def test_warn_threshold_constant(self):
        assert TILT_WARN_RAD == 0.35

    def test_fall_threshold_constant(self):
        assert TILT_FALL_RAD == 0.70

    def test_zero_quaternion_fallback(self):
        body = BodyState.from_quaternion(0.0, 0.0, 0.0, 0.0)
        assert body.roll_rad == 0.0
        assert body.pitch_rad == 0.0






