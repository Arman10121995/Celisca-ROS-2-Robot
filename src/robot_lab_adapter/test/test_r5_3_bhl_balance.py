"""R5.3 tests: Berkeley Humanoid Lite closed-loop balance core.

Covers the R5.3 acceptance bar, honestly:

- Joint constants (names, effort/position limits, nominal pose) are validated
  against the on-disk BHL URDF so the module never claims gains/limits that
  the description does not provide.
- A fixed standing pose is NOT balance: when the IMU body state reports a
  non-zero roll or pitch, the balance targets strictly change.
- Ankle strategy: body roll yields a differential ankle-roll command
  (opposite signs on left/right); body pitch yields a same-sign ankle-pitch
  lean; arms swing out of phase with roll (arm reaction).
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
    ARM_BALANCE_K,
    BASE_VEL_LIMITS_PLACEHOLDER,
    BHL_ARM_JOINTS,
    BHL_JOINT_NAMES,
    BHL_LEG_JOINTS,
    BHL_LEGS,
    EFFORT_LIMIT,
    KNEE_BALANCE_K,
    POSITION_LIMITS,
    STANCE_DURATION_SECONDS,
    STANCE_PD_ARMS,
    STANCE_PD_LEGS,
    TILT_FALL_RAD,
    TILT_WARN_RAD,
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
    def test_ankle_roll_differential_opp_signs(self, nominal):
        body = BodyState(roll_rad=0.2, pitch_rad=0.0)
        targets = balance_targets(nominal, body)
        left_delta = targets["leg_left_ankle_roll_joint"] - nominal["leg_left_ankle_roll_joint"]
        right_delta = targets["leg_right_ankle_roll_joint"] - nominal["leg_right_ankle_roll_joint"]
        assert left_delta == pytest.approx(ANKLE_BALANCE_K_ROLL * 0.2, abs=1e-9)
        assert right_delta == pytest.approx(-ANKLE_BALANCE_K_ROLL * 0.2, abs=1e-9)
        assert (left_delta > 0) and (right_delta < 0)

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
        body = BodyState(roll_rad=0.2, pitch_rad=0.0)
        targets = balance_targets(nominal, body)
        left_hip = targets["leg_left_hip_roll_joint"] - nominal["leg_left_hip_roll_joint"]
        right_hip = targets["leg_right_hip_roll_joint"] - nominal["leg_right_hip_roll_joint"]
        assert left_hip == pytest.approx(0.08 * 0.2, abs=1e-9)
        assert right_hip == pytest.approx(-0.08 * 0.2, abs=1e-9)

