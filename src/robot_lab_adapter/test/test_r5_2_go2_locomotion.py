"""R5.2 tests: Unitree Go2 locomotion core (stance, gait, estimation, safety).

Covers the R5.2 acceptance bar, honestly:

- Joint limits (effort/position) come from the Go2 description and every
  produced effort is clamped to them.
- Stance is a closed-loop joint-space PD hold; a joint with no position
  measurement is *not driven* (zero effort), never assumed at target.
- The gait is a bounded base-velocity interface: commanded twists are
  clamped and rate-limited; the trot is diagonal-pair stepping with
  stance/swing targets, explicitly *not* claimed to be MPC.
- Contact estimation is effort-residual based: stance legs with small
  residual are in contact, swing legs are not, missing data is None.
- The safety monitor latches SAFE_STOP on excessive tilt or sustained
  effort saturation; recovery needs an explicit reset.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

# Ensure the adapter package's *parent* dir is importable from the test tree.
# Let python find robot_lab_adapter under it (mirrors the R4.x benchmark tests):
# the package lives at <workspace>/src/robot_lab_adapter/robot_lab_adapter/.
_adapter_pkg = Path(__file__).resolve().parents[1]
if str(_adapter_pkg) not in sys.path:
    sys.path.insert(0, str(_adapter_pkg))

from robot_lab_adapter.go2_locomotion import (
    BASE_ACC_LIMITS,
    BASE_VEL_LIMITS,
    CONTACT_RESIDUAL_THRESHOLD_NM,
    EFFORT_LIMITS,
    EFFORT_SATURATION_CYCLES,
    JOINT_KINDS,
    JOINT_NAMES,
    LEG_PREFIXES,
    NOMINAL_STANCE,
    PD_GAINS,
    POSITION_LIMITS,
    SWING_CALF_OFFSET_RAD,
    SWING_THIGH_OFFSET_RAD,
    TILT_FALL_RAD,
    TILT_WARN_RAD,
    TROT_CYCLE_SECONDS,
    TROT_DUTY,
    BaseVelocity,
    BodyState,
    ControlCycle,
    Go2LocomotionCore,
    LegObservation,
    SafetyState,
    StanceController,
    clamp_base_velocity,
    clamp_effort,
    clamp_position,
    estimate_leg_contacts,
    joint_kind,
    leg_in_stance,
    leg_phase,
    nominal_stance_pose,
    rate_limit_base_velocity,
    trot_joint_targets,
)


# ------------------------------------------------------------------
# Constants honestly reflect the Go2 description
# ------------------------------------------------------------------


class TestConstants:
    def test_twelve_joints_four_legs(self):
        assert len(JOINT_NAMES) == 12
        assert set(LEG_PREFIXES) == {"FL", "FR", "RL", "RR"}
        assert set(JOINT_KINDS) == {"hip", "thigh", "calf"}

    def test_joint_ordering_matches_names(self):
        assert JOINT_NAMES[0] == "FL_hip_joint"
        assert JOINT_NAMES[-1] == "RR_calf_joint"

    def test_effort_limits_from_const_xacro(self):
        assert EFFORT_LIMITS == {"hip": 23.7, "thigh": 23.7, "calf": 35.55}

    def test_pd_gains_from_robot_control_yaml(self):
        assert PD_GAINS == {"hip": (100.0, 5.0),
                            "thigh": (300.0, 8.0), "calf": (300.0, 8.0)}

    def test_nominal_stance_inside_position_limits(self):
        for kind, value in NOMINAL_STANCE.items():
            lo, hi = POSITION_LIMITS[kind]
            assert lo <= value <= hi

    def test_joint_kind(self):
        assert joint_kind("FL_thigh_joint") == "thigh"
        with pytest.raises(ValueError):
            joint_kind("not_a_leg_joint")

    def test_clamp_effort(self):
        assert clamp_effort("FL_hip_joint", 100.0) == 23.7
        assert clamp_effort("FL_hip_joint", -100.0) == -23.7
        assert clamp_effort("FL_calf_joint", 40.0) == 35.55

    def test_clamp_position(self):
        assert clamp_position("FL_thigh_joint", 10.0) == POSITION_LIMITS["thigh"][1]
        assert clamp_position("FL_calf_joint", 0.0) == POSITION_LIMITS["calf"][1]


# ------------------------------------------------------------------
# Stance: closed-loop PD hold
# ------------------------------------------------------------------


class TestStance:
    def test_nominal_pose_has_twelve_entries(self):
        pose = nominal_stance_pose()
        assert set(pose) == set(JOINT_NAMES)

    def test_pd_hold_at_target_is_zero_effort(self):
        stance = StanceController()
        pose = nominal_stance_pose()
        command = stance.effort_command(pose, {j: 0.0 for j in JOINT_NAMES})
        for joint in JOINT_NAMES:
            assert command[joint] == 0.0

    def test_pd_pulls_toward_target(self):
        stance = StanceController()
        pose = nominal_stance_pose()
        pose["FL_thigh_joint"] = pose["FL_thigh_joint"] - 0.1
        command = stance.effort_command(pose, {j: 0.0 for j in JOINT_NAMES})
        assert command["FL_thigh_joint"] > 0.0  # pulls forward toward target

    def test_damping_opposes_velocity(self):
        stance = StanceController()
        pose = nominal_stance_pose()
        vel = {j: 0.0 for j in JOINT_NAMES}
        vel["FL_thigh_joint"] = 1.0
        command = stance.effort_command(pose, vel)
        assert command["FL_thigh_joint"] < 0.0

    def test_missing_position_means_no_drive(self):
        stance = StanceController()
        pose = {j: nominal_stance_pose()[j] for j in JOINT_NAMES if j != "RL_calf_joint"}
        command = stance.effort_command(pose, {})
        assert command["RL_calf_joint"] == 0.0

    def test_efforts_always_within_limits(self):
        stance = StanceController()
        pose = {j: 10.0 for j in JOINT_NAMES}
        command = stance.effort_command(pose, {j: 0.0 for j in JOINT_NAMES})
        for joint, effort in command.items():
            assert abs(effort) <= EFFORT_LIMITS[joint_kind(joint)]


# ------------------------------------------------------------------
# Bounded base-velocity interface
# ------------------------------------------------------------------


class TestBaseVelocity:
    def test_clamp_to_bounds(self):
        bounded = clamp_base_velocity(BaseVelocity(vx=10.0, vy=-10.0, wz=10.0))
        assert bounded.vx == BASE_VEL_LIMITS["vx"]
        assert bounded.vy == -BASE_VEL_LIMITS["vy"]
        assert bounded.wz == BASE_VEL_LIMITS["wz"]

    def test_rate_limit_first_order(self):
        prev = BaseVelocity()
        cmd = BaseVelocity(vx=10.0)
        nxt = rate_limit_base_velocity(prev, cmd, dt=0.1)
        assert nxt.vx == pytest.approx(BASE_ACC_LIMITS["vx"] * 0.1)

    def test_rate_limit_converges(self):
        prev = BaseVelocity()
        cmd = BaseVelocity(vx=0.5)
        for _ in range(100):
            prev = rate_limit_base_velocity(prev, cmd, dt=0.05)
        assert prev.vx == pytest.approx(0.5, abs=1e-6)

    def test_zero_dt_falls_back_to_clamp(self):
        nxt = rate_limit_base_velocity(BaseVelocity(), BaseVelocity(vx=10.0), dt=0.0)
        assert nxt.vx == BASE_VEL_LIMITS["vx"]

    def test_negative_dt_falls_back_to_clamp(self):
        nxt = rate_limit_base_velocity(BaseVelocity(), BaseVelocity(vx=10.0), dt=-1.0)
        assert nxt.vx == BASE_VEL_LIMITS["vx"]


# ------------------------------------------------------------------
# Trot gait: diagonal pairs, honest bounded stepping
# ------------------------------------------------------------------


class TestTrotGait:
    def test_diagonal_pairs_share_phase(self):
        assert leg_phase("FL", 0.25) == leg_phase("RR", 0.25)
        assert leg_phase("FR", 0.25) == leg_phase("RL", 0.25)

    def test_opposite_pairs_offset_half_cycle(self):
        assert leg_phase("FR", 0.1) == pytest.approx((leg_phase("FL", 0.1) + 0.5) % 1.0)

    def test_phase_wraps(self):
        assert leg_phase("FL", 1.25) == pytest.approx(0.25)

    def test_unknown_leg_raises(self):
        with pytest.raises(ValueError):
            leg_phase("XX", 0.0)

    def test_duty_half(self):
        assert TROT_DUTY == 0.5
        assert leg_in_stance("FL", 0.0) is True
        assert leg_in_stance("FL", 0.75) is False

    def test_exactly_one_pair_in_stance_at_mid_swing(self):
        # at phase 0.75 the first pair (FL/RR) is swinging
        assert leg_in_stance("FL", 0.75) is False
        assert leg_in_stance("FR", 0.75) is True

    def test_targets_cover_all_twelve_joints(self):
        targets = trot_joint_targets(0.3, BaseVelocity())
        assert set(targets) == set(JOINT_NAMES)

    def test_stance_legs_hold_nominal(self):
        targets = trot_joint_targets(0.1, BaseVelocity())  # all legs in stance
        for name in JOINT_NAMES:
            assert targets[name] == NOMINAL_STANCE[joint_kind(name)]

    def test_swing_leg_lifts_foot(self):
        # phase 0.75: FL/RR swing; swing thigh retracts (bounded offset)
        targets = trot_joint_targets(0.75, BaseVelocity(vx=BASE_VEL_LIMITS["vx"]))
        expected = clamp_position(
            "FL_thigh_joint", NOMINAL_STANCE["thigh"] + SWING_THIGH_OFFSET_RAD
        )
        assert targets["FL_thigh_joint"] == expected

    def test_zero_velocity_means_no_swing_offset(self):
        targets = trot_joint_targets(0.75, BaseVelocity())
        assert targets["FL_thigh_joint"] == NOMINAL_STANCE["thigh"]
        assert targets["FL_calf_joint"] == NOMINAL_STANCE["calf"]

    def test_swing_targets_within_position_limits(self):
        for phase in (0.0, 0.25, 0.5, 0.75):
            targets = trot_joint_targets(
                phase, BaseVelocity(vx=BASE_VEL_LIMITS["vx"], vy=1.0, wz=2.0)
            )
            for name, value in targets.items():
                lo, hi = POSITION_LIMITS[joint_kind(name)]
                assert lo <= value <= hi


# ------------------------------------------------------------------
# Effort-residual contact estimation
# ------------------------------------------------------------------


def _obs(measured, commanded):
    return LegObservation(
        measured_effort=measured, commanded_effort=commanded, position_error={}
    )


def _all_legs(obs_for):
    """Return {leg: LegObservation} for all four legs."""
    return {leg: obs_for.get(leg, _obs({}, {})) for leg in LEG_PREFIXES}


class TestContactEstimation:
    def test_swing_leg_is_never_in_contact(self):
        contacts = estimate_leg_contacts(
            _all_legs({}), {leg: False for leg in LEG_PREFIXES}
        )
        assert contacts == {leg: False for leg in LEG_PREFIXES}

    def test_loaded_stance_leg_in_contact(self):
        small = {"FL_thigh_joint": 10.0, "FL_calf_joint": 10.0, "FL_hip_joint": 10.0}
        contacts = estimate_leg_contacts(
            _all_legs({"FL": _obs(small, small)}),
            {leg: (leg == "FL") for leg in LEG_PREFIXES},
        )
        assert contacts["FL"] is True
        assert contacts["FR"] is False

    def test_large_residual_means_no_contact(self):
        measured = {"FL_thigh_joint": 0.0, "FL_calf_joint": 0.0, "FL_hip_joint": 0.0}
        commanded = {"FL_thigh_joint": 20.0, "FL_calf_joint": 20.0, "FL_hip_joint": 20.0}
        contacts = estimate_leg_contacts(
            _all_legs({"FL": _obs(measured, commanded)}),
            {leg: (leg == "FL") for leg in LEG_PREFIXES},
        )
        assert contacts["FL"] is False

    def test_missing_measurement_is_unknown_not_contact(self):
        commanded = {"FL_thigh_joint": 5.0, "FL_calf_joint": 5.0, "FL_hip_joint": 5.0}
        contacts = estimate_leg_contacts(
            _all_legs({"FL": _obs({}, commanded)}),
            {leg: (leg == "FL") for leg in LEG_PREFIXES},
        )
        assert contacts["FL"] is None

    def test_threshold_is_positive_and_used(self):
        assert CONTACT_RESIDUAL_THRESHOLD_NM == 8.0

    def test_boundary_residual(self):
        # residual exactly at threshold counts as contact
        measured = {"FL_thigh_joint": 8.0, "FL_calf_joint": 8.0, "FL_hip_joint": 8.0}
        commanded = {j: 0.0 for j in measured}
        contacts = estimate_leg_contacts(
            _all_legs({"FL": _obs(measured, commanded)}),
            {leg: (leg == "FL") for leg in LEG_PREFIXES},
        )
        assert contacts["FL"] is True


# ------------------------------------------------------------------
# Safety monitor: latched SAFE_STOP, no silent recovery
# ------------------------------------------------------------------


class TestSafetyMonitor:
    def test_starts_nominal(self):
        safety = SafetyState()
        assert safety.state == SafetyState.NOMINAL
        assert safety.gait_permitted() is True

    def test_mild_tilt_keeps_nominal(self):
        safety = SafetyState()
        issues = safety.observe_body(BodyState(roll_rad=0.05, pitch_rad=-0.05))
        assert safety.state == SafetyState.NOMINAL
        assert issues == []

    def test_warn_threshold_enters_warn(self):
        safety = SafetyState()
        issues = safety.observe_body(BodyState(roll_rad=0.40, pitch_rad=0.0))
        assert safety.state == SafetyState.WARN
        assert any(i.startswith("warn:") for i in issues)

    def test_fall_threshold_forces_safe_stop(self):
        safety = SafetyState()
        issues = safety.observe_body(BodyState(roll_rad=TILT_FALL_RAD, pitch_rad=0.0))
        assert safety.state == SafetyState.SAFE_STOP
        assert any(i.startswith("safe_stop:") for i in issues)
        assert safety.gait_permitted() is False

    def test_safe_stop_is_latched(self):
        safety = SafetyState()
        safety.observe_body(BodyState(roll_rad=TILT_FALL_RAD, pitch_rad=0.0))
        # even after the body recovers, the latch stays
        safety.observe_body(BodyState(roll_rad=0.0, pitch_rad=0.0))
        assert safety.state == SafetyState.SAFE_STOP
        assert safety.gait_permitted() is False

    def test_warn_recovers_to_nominal(self):
        safety = SafetyState()
        safety.observe_body(BodyState(roll_rad=0.40, pitch_rad=0.0))
        assert safety.state == SafetyState.WARN
        safety.observe_body(BodyState(roll_rad=0.0, pitch_rad=0.0))
        assert safety.state == SafetyState.NOMINAL

    def test_reset_clears_latch(self):
        safety = SafetyState()
        safety.observe_body(BodyState(roll_rad=TILT_FALL_RAD, pitch_rad=0.0))
        safety.reset()
        assert safety.state == SafetyState.NOMINAL
        assert safety.reason is None
        assert safety.gait_permitted() is True

    def test_safe_stop_efforts_are_zero(self):
        safety = SafetyState()
        assert safety.safe_stop_efforts() == {j: 0.0 for j in JOINT_NAMES}

    def test_sustained_saturation_enters_safe_stop(self):
        safety = SafetyState()
        limit = EFFORT_LIMITS["calf"]
        saturated = {"FL_calf_joint": limit}
        for _ in range(EFFORT_SATURATION_CYCLES):
            safety.observe_efforts(saturated, saturated)
        assert safety.state == SafetyState.SAFE_STOP
        assert "FL_calf_joint" in safety.reason

    def test_saturation_below_cycles_does_not_stop(self):
        safety = SafetyState()
        limit = EFFORT_LIMITS["calf"]
        saturated = {"FL_calf_joint": limit}
        for _ in range(EFFORT_SATURATION_CYCLES - 1):
            safety.observe_efforts(saturated, saturated)
        assert safety.state == SafetyState.NOMINAL

    def test_missing_measurement_resets_saturation_counter(self):
        safety = SafetyState()
        limit = EFFORT_LIMITS["calf"]
        saturated = {"FL_calf_joint": limit}
        for _ in range(EFFORT_SATURATION_CYCLES - 1):
            safety.observe_efforts(saturated, saturated)
        # missing measured effort for the joint resets the counter
        safety.observe_efforts(saturated, {})
        for _ in range(EFFORT_SATURATION_CYCLES - 1):
            safety.observe_efforts(saturated, saturated)
        assert safety.state == SafetyState.NOMINAL


class TestBodyState:
    def test_identity_quaternion_upright(self):
        body = BodyState.from_quaternion(0.0, 0.0, 0.0, 1.0)
        assert body.roll_rad == pytest.approx(0.0, abs=1e-9)
        assert body.pitch_rad == pytest.approx(0.0, abs=1e-9)

    def test_max_tilt_picks_larger(self):
        body = BodyState(roll_rad=0.2, pitch_rad=-0.45)
        assert body.max_tilt_rad == pytest.approx(0.45)


# ------------------------------------------------------------------
# Locomotion core: one honest update cycle
# ------------------------------------------------------------------


class TestLocomotionCore:
    def _positions(self):
        return {joint: 0.0 for joint in JOINT_NAMES}

    def test_single_update_produces_full_command(self):
        core = Go2LocomotionCore()
        cycle = core.update(
            0.05, self._positions(),
            measured_efforts={j: 0.0 for j in JOINT_NAMES},
        )
        assert set(cycle.efforts) == set(JOINT_NAMES)
        assert set(cycle.position_targets) == set(JOINT_NAMES)
        assert cycle.safety_state == SafetyState.NOMINAL
        assert cycle.issues == []

    def test_efforts_within_limits(self):
        core = Go2LocomotionCore()
        cycle = core.update(0.05, self._positions())
        for joint, effort in cycle.efforts.items():
            assert abs(effort) <= EFFORT_LIMITS[joint_kind(joint)]

    def test_phase_advances(self):
        core = Go2LocomotionCore()
        core.update(0.05, self._positions())
        assert core.phase == pytest.approx(0.05 / TROT_CYCLE_SECONDS)

    def test_phase_wraps_after_cycle(self):
        core = Go2LocomotionCore()
        core.update(TROT_CYCLE_SECONDS, self._positions())
        assert core.phase == pytest.approx(0.0, abs=1e-9)

    def test_missing_position_means_zero_drive(self):
        core = Go2LocomotionCore()
        positions = self._positions()
        del positions["RR_calf_joint"]
        cycle = core.update(0.05, positions)
        assert cycle.efforts["RR_calf_joint"] == 0.0

    def test_velocity_command_rate_limited(self):
        core = Go2LocomotionCore()
        cycle = core.update(
            0.01, self._positions(),
            velocity_command=BaseVelocity(vx=10.0),
        )
        # one 10 ms step at 1.0 m/s^2 reaches 0.01 m/s
        assert core.velocity.vx == pytest.approx(0.01, abs=1e-9)

    def test_velocity_command_bounded_at_limits(self):
        core = Go2LocomotionCore()
        for _ in range(500):
            core.update(
                0.05, self._positions(),
                velocity_command=BaseVelocity(vx=100.0, vy=100.0, wz=100.0),
            )
        assert core.velocity.vx == BASE_VEL_LIMITS["vx"]
        assert core.velocity.vy == BASE_VEL_LIMITS["vy"]
        assert core.velocity.wz == BASE_VEL_LIMITS["wz"]

    def test_fall_tilt_replaces_command_with_safe_stop(self):
        core = Go2LocomotionCore()
        cycle = core.update(
            0.05,
            self._positions(),
            body=BodyState(roll_rad=TILT_FALL_RAD, pitch_rad=0.0),
        )
        assert cycle.safety_state == SafetyState.SAFE_STOP
        assert all(e == 0.0 for e in cycle.efforts.values())

    def test_safe_stop_latches_across_cycles(self):
        core = Go2LocomotionCore()
        core.update(
            0.05,
            self._positions(),
            body=BodyState(roll_rad=TILT_FALL_RAD, pitch_rad=0.0),
        )
        cycle = core.update(0.05, self._positions())
        assert cycle.safety_state == SafetyState.SAFE_STOP
        assert all(e == 0.0 for e in cycle.efforts.values())

    def test_contacts_none_without_effort_measurements(self):
        core = Go2LocomotionCore()
        cycle = core.update(0.05, self._positions())
        assert all(value is None for value in cycle.contacts.values())

    def test_contacts_computed_with_effort_measurements(self):
        core = Go2LocomotionCore()
        measured = {j: 0.0 for j in JOINT_NAMES}
        cycle = core.update(0.05, self._positions(), measured_efforts=measured)
        assert set(cycle.contacts) == set(LEG_PREFIXES)
        assert all(value in (True, False, None) for value in cycle.contacts.values())

    def test_all_issues_flow_through_cycle(self):
        core = Go2LocomotionCore()
        cycle = core.update(
            0.05,
            self._positions(),
            body=BodyState(roll_rad=TILT_FALL_RAD, pitch_rad=0.0),
        )
        assert any("safe_stop" in issue for issue in cycle.issues)
