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
    BRACE_POSE,
    CONTACT_RESIDUAL_THRESHOLD_NM,
    CROUCH_POSE,
    crouch_phase_pose,
    EFFORT_LIMITS,
    EFFORT_SATURATION_CYCLES,
    FALL_CONFIRM_CYCLES,
    FALL_INVERTED_TILT_RAD,
    FALL_POSE_COLLAPSED,
    FALL_POSE_INVERTED,
    FALL_POSE_UNKNOWN,
    FALL_POSE_UPRIGHT,
    FALL_RECOVER_CROUCH_S,
    FALL_RECOVER_GATE_TILT_RAD,
    FALL_RECOVER_MAX_ROLL_CYCLES,
    FALL_RECOVER_ROLL_ENTRY_PITCH_RAD,
    FALL_RECOVER_ROLL_S,
    FALL_RECOVER_STAND_S,
    FALL_RECOVER_SUCCESS_DWELL_S,
    FALL_RECOVER_TUCK_S,
    JOINT_KINDS,
    JOINT_NAMES,
    LEG_PREFIXES,
    NOMINAL_STANCE,
    PD_GAINS,
    POSITION_LIMITS,
    RECOVERY_PHASES,
    RECOVERY_PHASE_ROLL,
    RECOVERY_PHASE_TUCK,
    SWING_CALF_OFFSET_RAD,
    SWING_THIGH_OFFSET_RAD,
    TILT_FALL_RAD,
    TILT_WARN_RAD,
    TROT_CYCLE_SECONDS,
    TROT_DUTY,
    TUCK_POSE,
    BaseVelocity,
    BodyState,
    ControlCycle,
    FallRecovery,
    Go2LocomotionCore,
    LegObservation,
    SafetyState,
    StanceController,
    brace_legs,
    clamp_base_velocity,
    clamp_effort,
    clamp_position,
    classify_fall_pose,
    estimate_leg_contacts,
    joint_kind,
    leg_in_stance,
    leg_phase,
    nominal_stance_pose,
    rate_limit_base_velocity,
    recovery_entry_phase,
    recovery_pose,
    roll_phase_pose,
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

    def test_gain_scale_reduces_simulator_startup_effort(self):
        pose = nominal_stance_pose()
        pose["FL_thigh_joint"] -= 0.1
        command = StanceController(gain_scale=0.15).effort_command(pose, {})
        assert command["FL_thigh_joint"] == pytest.approx(4.5)
        with pytest.raises(ValueError):
            Go2LocomotionCore(gain_scale=0.0)

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
        # At mid swing the calf folds while the thigh passes its center.
        targets = trot_joint_targets(0.75, BaseVelocity(vx=BASE_VEL_LIMITS["vx"]))
        expected = clamp_position(
            "FL_calf_joint", NOMINAL_STANCE["calf"] + SWING_CALF_OFFSET_RAD
        )
        assert targets["FL_calf_joint"] == expected

    def test_forward_stance_sweeps_foot_backward(self):
        start = trot_joint_targets(0.0, BaseVelocity(vx=0.25))["FL_thigh_joint"]
        end = trot_joint_targets(0.49, BaseVelocity(vx=0.25))["FL_thigh_joint"]
        assert start < NOMINAL_STANCE["thigh"] < end

    def test_yaw_gives_opposite_sides_opposite_stride(self):
        targets = trot_joint_targets(0.0, BaseVelocity(wz=0.5))
        assert targets["FL_thigh_joint"] > NOMINAL_STANCE["thigh"]
        assert targets["RR_thigh_joint"] < NOMINAL_STANCE["thigh"]

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


def test_direct_foot_forces_override_motor_effort_contact_heuristic():
    core = Go2LocomotionCore()
    pose = nominal_stance_pose()
    velocities = {name: 0.0 for name in JOINT_NAMES}
    cycle = core.update(
        0.004, pose, velocities, measured_efforts=velocities,
        measured_contact_forces={"FL": 12.0, "FR": 0.0,
                                 "RL": 15.0, "RR": 0.0})
    assert cycle.contacts == {"FL": True, "FR": False,
                              "RL": True, "RR": False}


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


# ----------------------------------------------------------------------
# Fall detection and the bounded re-stand attempt (R5.2 fall handling)
# ----------------------------------------------------------------------


class TestFallDetection:
    def test_single_tilt_spike_safe_stops_without_reporting_fall(self):
        safety = SafetyState()
        safety.observe_body(BodyState(roll_rad=TILT_FALL_RAD, pitch_rad=0.0))
        assert safety.state == SafetyState.SAFE_STOP
        assert safety.fallen is False

    def test_sustained_tilt_latches_fallen_and_reset_clears_it(self):
        safety = SafetyState()
        for _ in range(FALL_CONFIRM_CYCLES):
            safety.observe_body(BodyState(roll_rad=TILT_FALL_RAD, pitch_rad=0.0))
        assert safety.fallen is True
        assert "sustained tilt" in safety.fall_reason
        # Attitude recovering must NOT silently clear the latched fall.
        safety.observe_body(BodyState(roll_rad=0.0, pitch_rad=0.0))
        assert safety.fallen is True
        safety.reset()
        assert safety.fallen is False
        assert safety.fall_reason is None

    def test_interrupted_tilt_does_not_accumulate_to_a_fall(self):
        safety = SafetyState()
        for _ in range(FALL_CONFIRM_CYCLES - 1):
            safety.observe_body(BodyState(roll_rad=TILT_FALL_RAD, pitch_rad=0.0))
        safety.observe_body(BodyState(roll_rad=0.0, pitch_rad=0.0))
        for _ in range(FALL_CONFIRM_CYCLES - 1):
            safety.observe_body(BodyState(roll_rad=TILT_FALL_RAD, pitch_rad=0.0))
        assert safety.fallen is False

    def test_brief_trip_then_persistent_warning_latches_fall(self):
        safety = SafetyState()
        safety.observe_body(BodyState(roll_rad=TILT_FALL_RAD, pitch_rad=0.0))
        for _ in range(FALL_CONFIRM_CYCLES - 1):
            safety.observe_body(BodyState(roll_rad=0.52, pitch_rad=0.0))
        assert safety.fallen is True
        assert safety.state == SafetyState.SAFE_STOP

    def test_warning_without_fall_threshold_trip_does_not_latch_fall(self):
        safety = SafetyState()
        for _ in range(FALL_CONFIRM_CYCLES * 2):
            safety.observe_body(BodyState(roll_rad=0.52, pitch_rad=0.0))
        assert safety.fallen is False
        assert safety.state == SafetyState.WARN

    def test_fall_flag_never_permits_gait(self):
        safety = SafetyState()
        for _ in range(FALL_CONFIRM_CYCLES):
            safety.observe_body(BodyState(roll_rad=TILT_FALL_RAD, pitch_rad=0.0))
        assert safety.gait_permitted() is False


class TestFallPoseClassification:
    """Terminal-pose classification, checked against the recorded 60 N values.

    The 2026-09-25 60 N trials ended at 3.1416 rad and 0.057 m (inverted),
    while the 35 N trial ended at 0.056 rad and 0.373 m (upright) and the
    pre-recovery 60 N collapse at 0.518 rad and 0.139 m (collapsed).
    """

    def test_recorded_upright_35n_trial_is_upright(self):
        assert classify_fall_pose(
            BodyState(roll_rad=0.056, pitch_rad=0.0, body_height_m=0.373)
        ) == FALL_POSE_UPRIGHT

    def test_recorded_collapsed_60n_pose_is_collapsed(self):
        assert classify_fall_pose(
            BodyState(roll_rad=0.518, pitch_rad=0.0, body_height_m=0.139)
        ) == FALL_POSE_COLLAPSED

    def test_recorded_inverted_60n_terminal_pose_is_inverted(self):
        assert classify_fall_pose(
            BodyState(roll_rad=3.1416, pitch_rad=0.0, body_height_m=0.057)
        ) == FALL_POSE_INVERTED

    def test_inverted_is_detected_on_pitch_too(self):
        assert classify_fall_pose(
            BodyState(roll_rad=0.0, pitch_rad=math.pi, body_height_m=0.057)
        ) == FALL_POSE_INVERTED

    def test_inverted_takes_priority_over_standing_height(self):
        """A high body reading must not mask a rolled-over trunk: the first
        delayed trial briefly read 0.472 m airborne and looked recoverable."""
        assert classify_fall_pose(
            BodyState(roll_rad=FALL_INVERTED_TILT_RAD, pitch_rad=0.0,
                      body_height_m=0.47)
        ) == FALL_POSE_INVERTED

    def test_low_body_height_alone_is_not_inverted(self):
        assert classify_fall_pose(
            BodyState(roll_rad=0.05, pitch_rad=0.0, body_height_m=0.06)
        ) == FALL_POSE_COLLAPSED

    def test_tilt_above_warn_is_not_upright(self):
        assert classify_fall_pose(
            BodyState(roll_rad=TILT_WARN_RAD, pitch_rad=0.0, body_height_m=0.40)
        ) == FALL_POSE_COLLAPSED

    def test_missing_measurements_are_unknown_not_guessed(self):
        assert classify_fall_pose(None) == FALL_POSE_UNKNOWN
        assert classify_fall_pose(BodyState(0.5, 0.0)) == FALL_POSE_UNKNOWN
        assert classify_fall_pose(
            BodyState(0.05, 0.0, float("nan"))) == FALL_POSE_UNKNOWN
        assert classify_fall_pose(
            BodyState(float("nan"), 0.0, 0.30)) == FALL_POSE_UNKNOWN

    def test_inverted_threshold_is_past_vertical(self):
        assert FALL_INVERTED_TILT_RAD > TILT_FALL_RAD
        assert FALL_INVERTED_TILT_RAD < math.pi


class TestFallRecovery:
    def _positions(self, value=0.0):
        return {name: value for name in JOINT_NAMES}

    def test_idle_without_a_fall_commands_zero(self):
        recovery = FallRecovery()
        efforts = recovery.update(
            0.0, False, BodyState(0.0, 0.0), self._positions(), {})
        assert recovery.status == FallRecovery.IDLE
        assert all(e == 0.0 for e in efforts.values())

    def test_attempt_drives_measured_joints_toward_nominal_stance(self):
        recovery = FallRecovery()
        fallen_body = BodyState(roll_rad=TILT_FALL_RAD, pitch_rad=0.0)
        efforts = recovery.update(0.0, True, fallen_body, self._positions(), {})
        assert recovery.status == FallRecovery.ATTEMPTING
        assert recovery.attempts == 1
        thigh = "FL_thigh_joint"
        assert efforts[thigh] > 0.0  # target 0.72 rad is above the 0.0 pose
        assert efforts["FL_calf_joint"] < 0.0

    def test_unmeasured_joint_is_never_driven(self):
        recovery = FallRecovery()
        fallen_body = BodyState(roll_rad=TILT_FALL_RAD, pitch_rad=0.0)
        efforts = recovery.update(0.0, True, fallen_body, {}, {})
        assert all(e == 0.0 for e in efforts.values())

    def test_succeeds_only_after_sustained_upright_height_and_support(self):
        recovery = FallRecovery()
        fallen_body = BodyState(roll_rad=TILT_FALL_RAD, pitch_rad=0.0)
        recovery.update(0.0, True, fallen_body, self._positions(), {})
        forces = {leg: 15.0 for leg in LEG_PREFIXES}
        recovery.update(1.0, True, BodyState(0.01, 0.0, 0.35),
                        self._positions(), {}, forces)
        assert recovery.status == FallRecovery.ATTEMPTING
        efforts = recovery.update(
            1.6, True, BodyState(0.01, 0.0, 0.35),
            self._positions(), {}, forces)
        assert recovery.status == FallRecovery.SUCCEEDED
        assert all(e == 0.0 for e in efforts.values())

    def test_transient_airborne_height_is_not_recovery(self):
        recovery = FallRecovery()
        fallen_body = BodyState(roll_rad=TILT_FALL_RAD, pitch_rad=0.0)
        recovery.update(0.0, True, fallen_body, self._positions(), {})
        one_foot = {leg: (14.0 if leg == "RL" else 0.0) for leg in LEG_PREFIXES}
        recovery.update(1.0, True, BodyState(0.1, 0.0, 0.47),
                        self._positions(), {}, one_foot)
        recovery.update(1.6, True, BodyState(0.1, 0.0, 0.47),
                        self._positions(), {}, one_foot)
        assert recovery.status == FallRecovery.ATTEMPTING
        forces = {leg: 10.0 for leg in LEG_PREFIXES}
        recovery.update(2.0, True, BodyState(0.1, 0.0, 0.35),
                        self._positions(), {}, forces)
        recovery.update(2.2, True, BodyState(0.1, 0.0, 0.14),
                        self._positions(), {}, forces)
        recovery.update(2.8, True, BodyState(0.1, 0.0, 0.35),
                        self._positions(), {}, forces)
        assert recovery.status == FallRecovery.ATTEMPTING

    def test_upright_tilt_without_measured_standing_height_is_not_success(self):
        recovery = FallRecovery()
        fallen_body = BodyState(roll_rad=TILT_FALL_RAD, pitch_rad=0.0)
        recovery.update(0.0, True, fallen_body, self._positions(), {})
        forces = {leg: 15.0 for leg in LEG_PREFIXES}
        recovery.update(1.0, True, BodyState(0.01, 0.0, 0.14),
                        self._positions(), {}, forces)
        assert recovery.status == FallRecovery.ATTEMPTING
        recovery.update(2.0, True, BodyState(0.01, 0.0),
                        self._positions(), {}, forces)
        assert recovery.status == FallRecovery.ATTEMPTING

    def test_expired_window_fails_and_stops_driving(self):
        recovery = FallRecovery(timeout_s=1.0)
        fallen_body = BodyState(roll_rad=TILT_FALL_RAD, pitch_rad=0.0)
        recovery.update(0.0, True, fallen_body, self._positions(), {})
        efforts = recovery.update(2.0, True, fallen_body, self._positions(), {})
        assert recovery.status == FallRecovery.FAILED
        assert all(e == 0.0 for e in efforts.values())
        # A terminal state must not restart on its own.
        again = recovery.update(3.0, True, fallen_body, self._positions(), {})
        assert recovery.attempts == 1
        assert all(e == 0.0 for e in again.values())

    def test_collapsed_pose_expiry_is_failed_not_unrecoverable(self):
        """A recoverable terminal pose must report FAILED, so a real
        controller shortfall is not filed as an out-of-envelope pose."""
        recovery = FallRecovery(timeout_s=1.0)
        collapsed = BodyState(roll_rad=0.52, pitch_rad=0.0, body_height_m=0.139)
        recovery.update(0.0, True, collapsed, self._positions(), {})
        efforts = recovery.update(2.0, True, collapsed, self._positions(), {})
        assert recovery.status == FallRecovery.FAILED
        assert recovery.terminal_pose == FALL_POSE_COLLAPSED
        assert all(e == 0.0 for e in efforts.values())

    def test_inverted_pose_expiry_is_unrecoverable(self):
        """The recorded 60 N trials ended at tilt ~pi on the body. Standing
        effort cannot right that, so the verdict must not imply weak gains."""
        recovery = FallRecovery(timeout_s=1.0)
        inverted = BodyState(roll_rad=math.pi, pitch_rad=0.0, body_height_m=0.057)
        recovery.update(0.0, True, inverted, self._positions(), {})
        efforts = recovery.update(2.0, True, inverted, self._positions(), {})
        assert recovery.status == FallRecovery.UNRECOVERABLE
        assert recovery.terminal_pose == FALL_POSE_INVERTED
        assert all(e == 0.0 for e in efforts.values())
        # Like FAILED, it is terminal and must not restart on its own.
        again = recovery.update(3.0, True, inverted, self._positions(), {})
        assert recovery.attempts == 1
        assert all(e == 0.0 for e in again.values())

    def test_unrecoverable_is_a_terminal_state(self):
        assert FallRecovery.UNRECOVERABLE in FallRecovery.TERMINAL_STATES
        assert FallRecovery.FAILED in FallRecovery.TERMINAL_STATES
        assert FallRecovery.SUCCEEDED in FallRecovery.TERMINAL_STATES
        assert FallRecovery.ATTEMPTING not in FallRecovery.TERMINAL_STATES

    def test_success_records_the_measured_pose(self):
        recovery = FallRecovery()
        recovery.update(0.0, True, BodyState(roll_rad=TILT_FALL_RAD, pitch_rad=0.0),
                        self._positions(), {})
        forces = {leg: 15.0 for leg in LEG_PREFIXES}
        upright = BodyState(0.01, 0.0, 0.35)
        recovery.update(1.0, True, upright, self._positions(), {}, forces)
        recovery.update(1.6, True, upright, self._positions(), {}, forces)
        assert recovery.status == FallRecovery.SUCCEEDED
        assert recovery.terminal_pose == FALL_POSE_UPRIGHT

    def test_reset_clears_the_recorded_terminal_pose(self):
        recovery = FallRecovery(timeout_s=1.0)
        inverted = BodyState(roll_rad=math.pi, pitch_rad=0.0, body_height_m=0.057)
        recovery.update(0.0, True, inverted, self._positions(), {})
        recovery.update(2.0, True, inverted, self._positions(), {})
        assert recovery.terminal_pose == FALL_POSE_INVERTED
        recovery.reset()
        assert recovery.terminal_pose is None
        assert recovery.status == FallRecovery.IDLE

    def test_zero_effort_delay_starts_timeout_only_when_attempt_begins(self):
        recovery = FallRecovery(start_delay_s=1.0, timeout_s=2.0)
        fallen_body = BodyState(roll_rad=TILT_FALL_RAD, pitch_rad=0.0)
        waiting = recovery.update(0.0, True, fallen_body, self._positions(), {})
        assert recovery.status == FallRecovery.WAITING
        assert all(e == 0.0 for e in waiting.values())
        recovery.update(0.99, True, fallen_body, self._positions(), {})
        assert recovery.status == FallRecovery.WAITING
        driving = recovery.update(1.0, True, fallen_body, self._positions(), {})
        assert recovery.status == FallRecovery.ATTEMPTING
        assert driving["FL_thigh_joint"] > 0.0
        recovery.update(2.9, True, fallen_body, self._positions(), {})
        assert recovery.status == FallRecovery.ATTEMPTING
        stopped = recovery.update(3.1, True, fallen_body, self._positions(), {})
        assert recovery.status == FallRecovery.FAILED
        assert all(e == 0.0 for e in stopped.values())

    def test_rejects_unsafe_parameters(self):
        with pytest.raises(ValueError):
            FallRecovery(timeout_s=0.0)
        with pytest.raises(ValueError):
            FallRecovery(gain_scale=0.0)
        with pytest.raises(ValueError):
            FallRecovery(gain_scale=1.5)
        with pytest.raises(ValueError):
            FallRecovery(damping_scale=0.0)
        with pytest.raises(ValueError):
            FallRecovery(start_delay_s=-0.1)

    def test_reset_returns_to_idle(self):
        recovery = FallRecovery()
        fallen_body = BodyState(roll_rad=TILT_FALL_RAD, pitch_rad=0.0)
        recovery.update(0.0, True, fallen_body, self._positions(), {})
        recovery.reset()
        assert recovery.status == FallRecovery.IDLE


class TestFallRecoveryLadder:
    """The sequenced get-up ladder: tuck -> roll -> crouch -> stand.

    The recorded 60 N failure was a *single-phase* attempt: it drove the
    nominal stance pose from a down trunk at 0.14 m and ended inverted at tilt
    ~pi. These tests pin the invariants that failure motivated: the standing
    pose is withheld until measured tilt is under the gate, the roll phase
    braces the pair the trunk is measured to rest on, phases and roll cycles
    are bounded, and a missing attitude holds the phase instead of guessing.
    """

    #: The ladder tests run the same PD law at a small gain scale so the
    #: commanded effort stays inside the per-joint limits: at the runtime 0.5
    #: scale every joint saturates from a zero-position measurement and
    #: different waypoints then clamp to the same number, which would hide
    #: *which* waypoint is being driven.
    GAIN_SCALE = 0.02

    def _positions(self, value=0.0):
        return {name: value for name in JOINT_NAMES}

    def _stance(self, target, positions):
        return StanceController(target=target, gain_scale=self.GAIN_SCALE,
                                damping_scale=self.GAIN_SCALE
                                ).effort_command(positions, {})

    def _recovery(self, **kwargs):
        return FallRecovery(gain_scale=self.GAIN_SCALE,
                            damping_scale=self.GAIN_SCALE, **kwargs)

    def _collapsed(self, tilt=1.2, height=0.14, axis="pitch"):
        """A collapsed fallen trunk, nose-down -- the ladder's tuck-first path.

        The entry phase is read from measured attitude
        (:func:`recovery_entry_phase`): only a pitch-dominant trunk enters at
        the tuck, so the ladder-mechanics tests pose this fixture nose-down.
        The roll-axis cases ask for ``axis="roll"`` explicitly, which is the
        trunk the ladder enters at the braced push.
        """
        if axis == "roll":
            return BodyState(roll_rad=tilt, pitch_rad=0.0,
                             body_height_m=height)
        return BodyState(roll_rad=0.0, pitch_rad=tilt, body_height_m=height)

    def test_first_cycle_retracts_the_legs_instead_of_driving_stance(self):
        recovery = self._recovery()
        positions = self._positions()
        efforts = recovery.update(0.0, True, self._collapsed(), positions, {})
        assert recovery.status == FallRecovery.ATTEMPTING
        assert recovery.phase == FallRecovery.TUCK
        assert recovery.state_label == "attempting:tuck"
        assert recovery.roll_cycles == 0
        assert efforts == pytest.approx(
            self._stance(recovery_pose("tuck"), positions))
        standing = self._stance(nominal_stance_pose(), positions)
        assert any(abs(efforts[j] - standing[j]) > 1e-6 for j in JOINT_NAMES)

    def test_roll_dominant_side_rest_enters_at_the_braced_push(self):
        # Measured: the settled side rest (roll 0.518 / pitch 0.000) is the very
        # pose whose down-side pair folded out from under it in the recorded
        # 60 N trials -- one 198 N strike through a single foot, the other three
        # feet at 0.0 N, then inverted at 0.057 m. Its entry is the braced push.
        recovery = self._recovery()
        positions = self._positions()
        rest = BodyState(roll_rad=0.518, pitch_rad=0.0, body_height_m=0.139)
        efforts = recovery.update(0.0, True, rest, positions, {})
        assert recovery.phase == FallRecovery.ROLL
        assert recovery.state_label == "attempting:roll"
        assert recovery.roll_cycles == 1
        assert efforts == pytest.approx(
            self._stance(roll_phase_pose(0.518, 0.0), positions))
        # The pair the trunk rests on is driven *out* of the tuck fold -- thigh
        # back toward zero, calf opened toward the measured limit -- and the
        # free pair, already at its tuck waypoint, is left alone.
        assert brace_legs(0.518, 0.0) == ("FR", "RR")
        pushed = recovery.update(0.05, True, rest, recovery_pose("tuck"), {})
        for leg in ("FR", "RR"):
            assert pushed[f"{leg}_thigh_joint"] < 0.0
            assert pushed[f"{leg}_calf_joint"] > 0.0
        for leg in ("FL", "RL"):
            assert pushed[f"{leg}_thigh_joint"] == pytest.approx(0.0)
            assert pushed[f"{leg}_calf_joint"] == pytest.approx(0.0)

    def test_a_settled_side_rest_is_never_folded_by_the_ladder(self):
        # A rest that stays put (the null control's 0.52 rad / 0.139 m) is
        # already under the gate, so the ladder climbs the rest of the way --
        # roll -> crouch -> stand -- and ends on the bounded window. What it
        # must never do is fold the legs the trunk is lying on, which is the
        # measurement this dispatch exists for.
        recovery = self._recovery()
        positions = self._positions()
        rest = BodyState(roll_rad=0.518, pitch_rad=0.0, body_height_m=0.139)
        recovery.update(0.0, True, rest, positions, {})
        assert recovery.phase == FallRecovery.ROLL
        roll_end = FALL_RECOVER_ROLL_S + 0.01
        recovery.update(roll_end, True, rest, positions, {})
        assert recovery.phase == FallRecovery.CROUCH
        recovery.update(roll_end + FALL_RECOVER_CROUCH_S + 0.01, True, rest,
                        positions, {})
        assert recovery.phase == FallRecovery.STAND
        efforts = recovery.update(4.1, True, rest, positions, {})
        assert recovery.status == FallRecovery.FAILED
        assert recovery.last_reason.startswith("get-up window expired")
        assert recovery.terminal_pose == FALL_POSE_COLLAPSED
        assert recovery.state_label == "failed:stand"
        assert recovery.roll_cycles == 1
        assert all(e == 0.0 for e in efforts.values())


    def test_every_measured_recovering_pose_still_enters_at_the_tuck(self):
        # The dispatch must not disturb a pose that measurably got up: every
        # placed nose-down pose from 0.9 to 1.6 rad recovered through the
        # tuck-first ladder, and so did the roll-dominant 1.0+0.7, 1.2+1.0 and
        # 1.4+0.8. The 0.9+0.8 and 1.0+0.9 corner diagonals -- which never got
        # up -- stay on that path too: this rule does not claim to fix them.
        positions = self._positions()
        for body in (
                BodyState(roll_rad=0.0, pitch_rad=0.9, body_height_m=0.2),
                BodyState(roll_rad=0.0, pitch_rad=1.4, body_height_m=0.234),
                BodyState(roll_rad=0.0, pitch_rad=1.6, body_height_m=0.2),
                BodyState(roll_rad=0.7, pitch_rad=1.0, body_height_m=0.2),
                BodyState(roll_rad=1.0, pitch_rad=1.2, body_height_m=0.2),
                BodyState(roll_rad=0.8, pitch_rad=1.4, body_height_m=0.2),
                BodyState(roll_rad=0.8, pitch_rad=0.9, body_height_m=0.2),
                BodyState(roll_rad=0.9, pitch_rad=1.0, body_height_m=0.2)):
            recovery = self._recovery()
            recovery.update(0.0, True, body, positions, {})
            assert recovery.phase == FallRecovery.TUCK
            assert recovery.roll_cycles == 0

    def test_entry_phase_is_read_from_measurement_or_not_guessed(self):
        # No measurement, a non-finite one, and any trunk past the inverted tilt
        # (which has nothing to brace against): the tuck, exactly as before.
        assert recovery_entry_phase(None) == RECOVERY_PHASE_TUCK
        assert recovery_entry_phase(
            BodyState(roll_rad=float("nan"), pitch_rad=0.0)) == RECOVERY_PHASE_TUCK
        assert recovery_entry_phase(
            BodyState(roll_rad=0.0, pitch_rad=float("inf"))) == RECOVERY_PHASE_TUCK
        assert recovery_entry_phase(
            BodyState(roll_rad=FALL_INVERTED_TILT_RAD + 0.1, pitch_rad=0.0,
                      body_height_m=0.057)) == RECOVERY_PHASE_TUCK
        # Roll-dominant either side, and the 0.9+0.9 tie (which brace_legs reads
        # as roll) enter at the braced push; with the pitch margin met, not.
        assert recovery_entry_phase(
            BodyState(roll_rad=0.9, pitch_rad=0.0)) == RECOVERY_PHASE_ROLL
        assert recovery_entry_phase(
            BodyState(roll_rad=-0.9, pitch_rad=0.0)) == RECOVERY_PHASE_ROLL
        assert recovery_entry_phase(
            BodyState(roll_rad=0.9, pitch_rad=0.9)) == RECOVERY_PHASE_ROLL
        assert recovery_entry_phase(
            BodyState(roll_rad=0.8, pitch_rad=1.0)) == RECOVERY_PHASE_TUCK
        # The threshold sits inside the ladder's own envelope.
        assert 0.0 < FALL_RECOVER_ROLL_ENTRY_PITCH_RAD < FALL_INVERTED_TILT_RAD

    def test_standing_pose_is_withheld_while_the_trunk_stays_past_the_gate(self):
        recovery = self._recovery()
        positions = self._positions()
        standing = self._stance(nominal_stance_pose(), positions)
        collapsed = self._collapsed()
        for t in (0.0, 0.5, 1.2, 1.9, 2.6):
            efforts = recovery.update(t, True, collapsed, positions, {})
            if recovery.status != FallRecovery.ATTEMPTING:
                break
            assert recovery.phase in (FallRecovery.TUCK, FallRecovery.ROLL)
            assert any(abs(efforts[j] - standing[j]) > 1e-6 for j in JOINT_NAMES)
        # A trunk that does not move is not worth repeating: the attempt stops on
        # the measured *progress* rule -- which is what the fixed count was
        # standing in for -- with the pose class still read from the measurement.
        assert recovery.status == FallRecovery.FAILED
        assert recovery.roll_cycles == 1
        assert recovery.last_reason.startswith("roll cycle bought less than")
        assert recovery.terminal_pose == FALL_POSE_COLLAPSED

    def test_roll_phase_braces_the_pair_the_trunk_rests_on(self):
        recovery = self._recovery()
        positions = self._positions()
        # A roll-dominant trunk is the case the ladder enters at the braced
        # push (recovery_entry_phase), so the roll phase is already running from
        # the first cycle.
        collapsed = self._collapsed(axis="roll")
        recovery.update(0.0, True, collapsed, positions, {})
        rolled = recovery.update(FALL_RECOVER_TUCK_S, True, collapsed, positions, {})
        assert recovery.phase == FallRecovery.ROLL
        assert recovery.roll_cycles == 1
        assert recovery.state_label == "attempting:roll"
        assert rolled == pytest.approx(
            self._stance(roll_phase_pose(1.2, 0.0), positions))
        # With the legs measured at the tuck waypoint, the loaded pair is
        # driven toward the straight waypoint -- thigh back toward zero
        # (negative effort) and calf opened toward the limit (positive
        # effort) -- while the other pair is already home.
        tuck_positions = recovery_pose("tuck")
        efforts = recovery.update(FALL_RECOVER_TUCK_S + 0.05, True, collapsed,
                                  tuck_positions, {})
        assert efforts["FR_thigh_joint"] < 0.0
        assert efforts["FR_calf_joint"] > 0.0
        assert efforts["FL_thigh_joint"] == pytest.approx(0.0)
        assert efforts["FL_calf_joint"] == pytest.approx(0.0)

    def test_roll_phase_follows_the_measured_attitude(self):
        assert brace_legs(1.2, 0.0) == ("FR", "RR")
        assert brace_legs(-1.2, 0.0) == ("FL", "RL")
        assert brace_legs(0.0, 1.2) == ("RL", "RR")
        assert brace_legs(-0.1, -1.2) == ("FL", "FR")
        left = roll_phase_pose(-1.2, 0.0)
        right = roll_phase_pose(1.2, 0.0)
        assert left["FL_thigh_joint"] == pytest.approx(BRACE_POSE["thigh"])
        assert left["FR_thigh_joint"] == pytest.approx(TUCK_POSE["thigh"])
        assert right["FL_thigh_joint"] == pytest.approx(TUCK_POSE["thigh"])
        assert right["FR_thigh_joint"] == pytest.approx(BRACE_POSE["thigh"])

    def test_ladder_reaches_stand_only_after_a_measured_gate_pass(self):
        recovery = self._recovery()
        positions = self._positions()
        collapsed = self._collapsed()
        recovery.update(0.0, True, collapsed, positions, {})
        assert recovery.phase == FallRecovery.TUCK
        recovery.update(FALL_RECOVER_TUCK_S, True, collapsed, positions, {})
        assert recovery.phase == FallRecovery.ROLL
        # The roll primitive brings the trunk back under the gate.
        recovered = BodyState(roll_rad=0.45, pitch_rad=0.0, body_height_m=0.14)
        recovery.update(FALL_RECOVER_TUCK_S + FALL_RECOVER_ROLL_S, True,
                        recovered, positions, {})
        assert recovery.phase == FallRecovery.CROUCH
        assert recovery.state_label == "attempting:crouch"
        efforts = recovery.update(
            FALL_RECOVER_TUCK_S + FALL_RECOVER_ROLL_S + FALL_RECOVER_CROUCH_S,
            True, recovered, positions, {})
        assert recovery.phase == FallRecovery.STAND
        assert recovery.state_label == "attempting:stand"
        # The standing pose is slewed in over the bounded stand window and is
        # reached exactly at its end; the slew itself is pinned by
        # test_stand_phase_slews_the_pose_instead_of_stepping_to_it.
        stand_at = (FALL_RECOVER_TUCK_S + FALL_RECOVER_ROLL_S
                    + FALL_RECOVER_CROUCH_S)
        recovery.update(stand_at + FALL_RECOVER_STAND_S, True,
                        recovered, positions, {})
        assert recovery.phase == FallRecovery.STAND
        efforts = recovery.update(stand_at + FALL_RECOVER_STAND_S + 0.1, True,
                                  recovered, positions, {})
        assert efforts == pytest.approx(
            self._stance(nominal_stance_pose(), positions))

    def test_a_trunk_that_climbs_back_out_of_the_gate_is_not_levered_further(self):
        recovery = self._recovery()
        positions = self._positions()
        nearly_up = BodyState(roll_rad=0.4, pitch_rad=0.0, body_height_m=0.14)
        recovery.update(0.0, True, self._collapsed(), positions, {})
        recovery.update(FALL_RECOVER_TUCK_S, True, nearly_up, positions, {})
        assert recovery.phase == FallRecovery.CROUCH
        recovery.update(FALL_RECOVER_TUCK_S + FALL_RECOVER_CROUCH_S, True,
                        nearly_up, positions, {})
        assert recovery.phase == FallRecovery.STAND
        # The trunk rolls back past the gate: back down the ladder it goes.
        efforts = recovery.update(
            FALL_RECOVER_TUCK_S + FALL_RECOVER_CROUCH_S + 0.1, True,
            self._collapsed(tilt=1.0), positions, {})
        assert recovery.phase == FallRecovery.TUCK
        assert efforts == pytest.approx(
            self._stance(recovery_pose("tuck"), positions))

    def test_inverted_trunk_never_enters_the_roll_phase(self):
        recovery = self._recovery(timeout_s=1.0)
        positions = self._positions()
        inverted = BodyState(roll_rad=FALL_INVERTED_TILT_RAD + 0.1,
                             pitch_rad=0.0, body_height_m=0.057)
        recovery.update(0.0, True, inverted, positions, {})
        assert recovery.phase == FallRecovery.TUCK
        recovery.update(FALL_RECOVER_TUCK_S, True, inverted, positions, {})
        assert recovery.phase == FallRecovery.TUCK       # re-retracted, not rolled
        assert recovery.roll_cycles == 0
        recovery.update(1.5, True, inverted, positions, {})
        assert recovery.status == FallRecovery.UNRECOVERABLE
        assert recovery.terminal_pose == FALL_POSE_INVERTED
        assert recovery.state_label == "unrecoverable:tuck"

    def test_unmeasured_attitude_holds_the_phase_instead_of_advancing(self):
        recovery = self._recovery()
        positions = self._positions()
        recovery.update(0.0, True, self._collapsed(), positions, {})
        efforts = recovery.update(2.0, True, None, positions, {})
        assert recovery.phase == FallRecovery.TUCK
        assert recovery.status == FallRecovery.ATTEMPTING
        assert efforts == pytest.approx(
            self._stance(recovery_pose("tuck"), positions))

    def _succeed(self, recovery, positions, forces, standing):
        """Drive the ladder to a *measured* success; returns that time."""
        recovery.update(0.0, True, self._collapsed(), positions, {})
        recovery.update(1.0, True, standing, positions, {}, forces)
        recovery.update(1.0 + FALL_RECOVER_SUCCESS_DWELL_S, True, standing,
                        positions, {}, forces)
        return 1.0 + FALL_RECOVER_SUCCESS_DWELL_S

    def test_completed_get_up_holds_the_nominal_stance(self):
        # Measured motivation: the placed 1.4 rad nose-down trial reached
        # 0.33 m with four feet loaded, reported `succeeded:stand`, and was flat
        # on its belly 0.24 s later. The node calls update() for as long as
        # `fallen` is latched, so a zero-effort return at success is a command
        # that drops the robot the ladder just stood up.
        recovery = self._recovery()
        positions = self._positions()
        forces = {leg: 25.0 for leg in LEG_PREFIXES}
        standing = BodyState(roll_rad=0.01, pitch_rad=0.0, body_height_m=0.33)
        now = self._succeed(recovery, positions, forces, standing)
        assert recovery.status == FallRecovery.SUCCEEDED
        held = recovery.update(now + 0.1, True, standing, positions, {}, forces)
        assert held == pytest.approx(
            self._stance(nominal_stance_pose(), positions))
        assert any(abs(value) > 0.0 for value in held.values())
        assert recovery.state_label.startswith("succeeded")

    def test_completed_get_up_stops_driving_when_the_evidence_is_gone(self):
        recovery = self._recovery()
        positions = self._positions()
        forces = {leg: 25.0 for leg in LEG_PREFIXES}
        standing = BodyState(roll_rad=0.01, pitch_rad=0.0, body_height_m=0.33)
        now = self._succeed(recovery, positions, forces, standing)
        # Height lost: the robot is back down, so the hold must stop.
        dropped = BodyState(roll_rad=0.05, pitch_rad=0.0, body_height_m=0.14)
        efforts = recovery.update(now + 0.1, True, dropped, positions, {}, forces)
        assert all(value == 0.0 for value in efforts.values())
        # Support lost -- airborne, or tipped onto its side -- stops it too.
        one_foot = {leg: (25.0 if leg == "RR" else 0.0) for leg in LEG_PREFIXES}
        tipped = BodyState(roll_rad=0.4, pitch_rad=0.0, body_height_m=0.33)
        efforts = recovery.update(now + 0.2, True, tipped, positions, {}, one_foot)
        assert all(value == 0.0 for value in efforts.values())
        # The decision is measured, never latched: standing again re-enables
        # the hold, and no measurement at all keeps it stopped.
        again = recovery.update(now + 0.3, True, standing, positions, {}, forces)
        assert any(abs(value) > 0.0 for value in again.values())
        unknown = recovery.update(now + 0.4, True, None, positions, {}, forces)
        assert all(value == 0.0 for value in unknown.values())
        assert recovery.status == FallRecovery.SUCCEEDED

    def test_a_bounded_stop_never_holds_the_stance(self):
        # The hold belongs to a measured success only: a bounded stop keeps
        # publishing nothing even when the robot is back inside the gate.
        recovery = self._recovery()
        positions = self._positions()
        collapsed = self._collapsed()
        for t in (0.0, FALL_RECOVER_TUCK_S,
                  FALL_RECOVER_TUCK_S + FALL_RECOVER_ROLL_S, 1.0, 2.0, 3.0):
            recovery.update(t, True, collapsed, positions, {})
            if recovery.status != FallRecovery.ATTEMPTING:
                break
        assert recovery.status == FallRecovery.FAILED
        forces = {leg: 25.0 for leg in LEG_PREFIXES}
        standing = BodyState(roll_rad=0.01, pitch_rad=0.0, body_height_m=0.33)
        efforts = recovery.update(3.2, True, standing, positions, {}, forces)
        assert all(value == 0.0 for value in efforts.values())
        assert recovery.status == FallRecovery.FAILED

    def test_stand_phase_slews_the_pose_in_instead_of_stepping_to_it(self):
        # Measured: stepping to the nominal stance out of the crouch over-drives
        # the legs -- the placed nose-down trial catapulted the trunk to 0.56 m
        # with no foot loaded and tilt to 0.81 rad, and the ladder then spent
        # three retries undoing its own stand-up. The stand phase must slew.
        recovery = self._recovery(timeout_s=20.0)
        positions = self._positions()
        nearly_up = BodyState(roll_rad=0.45, pitch_rad=0.0, body_height_m=0.14)
        recovery.update(0.0, True, self._collapsed(), positions, {})
        recovery.update(FALL_RECOVER_TUCK_S, True, nearly_up, positions, {})
        assert recovery.phase == FallRecovery.CROUCH
        crouch_effort = self._stance(recovery_pose("crouch"), positions)
        stand_effort = self._stance(nominal_stance_pose(), positions)
        stand_at = FALL_RECOVER_TUCK_S + FALL_RECOVER_CROUCH_S
        # Entering the phase drives the crouch waypoint, not a step to standing.
        entry = recovery.update(stand_at, True, nearly_up, positions, {})
        assert recovery.phase == FallRecovery.STAND
        assert entry == pytest.approx(crouch_effort)
        assert any(abs(entry[j] - stand_effort[j]) > 1e-3 for j in JOINT_NAMES)
        # Halfway through the bounded slew the command is strictly between the
        # two waypoints -- progress, never a jump.
        half = recovery.update(stand_at + FALL_RECOVER_STAND_S / 2.0, True,
                               nearly_up, positions, {})
        assert any(abs(half[j] - crouch_effort[j]) > 1e-6 for j in JOINT_NAMES)
        assert any(abs(half[j] - stand_effort[j]) > 1e-6 for j in JOINT_NAMES)
        # At the end of the slew the target *is* the nominal stance, and it
        # stays there for as long as the phase runs.
        done = recovery.update(stand_at + FALL_RECOVER_STAND_S, True,
                               nearly_up, positions, {})
        assert done == pytest.approx(stand_effort)
        later = recovery.update(stand_at + 5.0, True, nearly_up, positions, {})
        assert later == pytest.approx(stand_effort)

    def test_roll_phase_lateral_hip_input_is_opt_in_and_mirrored(self):
        # The default stays the qualified sagittal-only brace: hips at zero.
        default = roll_phase_pose(1.2, 0.0)
        assert all(default[f"{leg}_hip_joint"] == pytest.approx(0.0)
                   for leg in LEG_PREFIXES)
        # A nonzero input reaches the *braced* pair only, mirrored per side,
        # because the Go2's hip roll axes are opposed.
        braced = roll_phase_pose(1.2, 0.0, 0.4)
        assert braced["FR_hip_joint"] == pytest.approx(0.4)
        assert braced["RR_hip_joint"] == pytest.approx(0.4)
        assert braced["FL_hip_joint"] == pytest.approx(0.0)
        assert braced["RL_hip_joint"] == pytest.approx(0.0)
        left = roll_phase_pose(-1.2, 0.0, 0.4)
        assert left["FL_hip_joint"] == pytest.approx(-0.4)
        assert left["FR_hip_joint"] == pytest.approx(0.0)
        # The rest of the braced waypoint is unchanged, and the input is
        # clamped to the measured hip limits.
        assert braced["FR_thigh_joint"] == pytest.approx(BRACE_POSE["thigh"])
        assert roll_phase_pose(1.2, 0.0, 9.0)[f"{'FR'}_hip_joint"] == pytest.approx(
            clamp_position("FR_hip_joint", 9.0))
        # The recovery passes it through, and refuses a non-finite value.
        recovery = self._recovery(roll_brace_hip_rad=0.4)
        recovery.update(0.0, True, self._collapsed(axis="roll"),
                        positions := self._positions(), {})
        efforts = recovery.update(FALL_RECOVER_TUCK_S, True,
                                  self._collapsed(axis="roll"),
                                  positions, {})
        assert recovery.phase == FallRecovery.ROLL
        assert efforts == pytest.approx(
            self._stance(roll_phase_pose(1.2, 0.0, 0.4), positions))
        with pytest.raises(ValueError):
            FallRecovery(roll_brace_hip_rad=float("nan"))

    def test_crouch_keeps_the_measured_splay_only_while_roll_dominates(self):
        # The plain crouch waypoint is the pitch path, which measurably works,
        # so it must be untouched by the lateral input.
        plain = crouch_phase_pose(1.4, 0.0)
        assert plain == pytest.approx(recovery_pose("crouch"))
        assert crouch_phase_pose(0.0, 1.4, 0.8) == pytest.approx(
            recovery_pose("crouch"))
        # Roll-dominated: the braced pair keeps the splay, the other pair does
        # not, and the knees are still the crouch waypoint.
        rolled = crouch_phase_pose(1.4, 0.0, 0.8)
        assert rolled["FR_hip_joint"] == pytest.approx(0.8)
        assert rolled["RR_hip_joint"] == pytest.approx(0.8)
        assert rolled["FL_hip_joint"] == pytest.approx(0.0)
        assert rolled["RL_hip_joint"] == pytest.approx(0.0)
        assert rolled["FR_calf_joint"] == pytest.approx(CROUCH_POSE["calf"])
        left = crouch_phase_pose(-1.4, 0.0, 0.8)
        assert left["FL_hip_joint"] == pytest.approx(-0.8)
        assert left["FR_hip_joint"] == pytest.approx(0.0)

    def test_stand_releases_the_splay_with_the_trunks_remaining_roll(self):
        # Measured: dropping the splay at the stand entry put the trunk on its
        # back, so the stand target releases it with the measured roll instead.
        recovery = self._recovery(timeout_s=20.0, roll_brace_hip_rad=0.8)
        positions = self._positions()
        stand_at = FALL_RECOVER_TUCK_S + FALL_RECOVER_CROUCH_S
        recovery.update(0.0, True, self._collapsed(), positions, {})
        recovery.update(FALL_RECOVER_TUCK_S, True,
                        BodyState(roll_rad=0.6, pitch_rad=0.0, body_height_m=0.14),
                        positions, {})
        recovery.update(stand_at, True,
                        BodyState(roll_rad=0.6, pitch_rad=0.0, body_height_m=0.14),
                        positions, {})
        assert recovery.phase == FallRecovery.STAND

        def hips_at(roll, pitch=0.0):
            recovery.phase_started_at = 99.0   # past the slew: the nominal pose
            target = recovery._stand_target(
                99.0 + FALL_RECOVER_STAND_S, BodyState(roll, pitch, 0.33))
            return {leg: target[f"{leg}_hip_joint"] for leg in LEG_PREFIXES}

        # Full splay while the trunk is still well rolled, on the braced pair.
        rolled = hips_at(0.6)
        assert rolled["FR"] == pytest.approx(0.8)
        assert rolled["RR"] == pytest.approx(0.8)
        assert rolled["FL"] == pytest.approx(0.0)
        # Graded: it tapers with the remaining roll rather than switching off.
        assert hips_at(0.2)["FR"] == pytest.approx(0.8 * 0.2 / 0.35)
        # Upright, and pitch-dominated, both mean exactly the nominal stance.
        assert hips_at(0.0)["FR"] == pytest.approx(0.0)
        assert hips_at(0.0, 0.4)["FR"] == pytest.approx(0.0)
        # Without the opt-in input nothing changes at all.
        plain = self._recovery(timeout_s=20.0)
        plain.update(0.0, True, self._collapsed(), positions, {})
        plain.update(FALL_RECOVER_TUCK_S, True,
                     BodyState(roll_rad=0.6, pitch_rad=0.0, body_height_m=0.14),
                     positions, {})
        plain.update(stand_at, True,
                     BodyState(roll_rad=0.6, pitch_rad=0.0, body_height_m=0.14),
                     positions, {})
        plain.phase_started_at = 99.0
        target = plain._stand_target(
            99.0 + FALL_RECOVER_STAND_S,
            BodyState(0.6, 0.0, 0.33))
        assert target == pytest.approx(nominal_stance_pose())

    def test_a_roll_cycle_is_repeated_only_while_the_trunk_keeps_improving(self):
        # Measured: the free-pair splay walks a 1.4 rad flank down to 0.49 rad but
        # needs longer than the old fixed two cycles, and a primitive that was
        # still measurably working used to be reported as a failure.
        def body(tilt):
            # The progress rule reads tilt only, and the axis under test here
            # is the cycle bookkeeping: pose the fixture nose-down so the entry
            # is the tuck and the roll cycle is the first one.
            return BodyState(roll_rad=0.0, pitch_rad=tilt, body_height_m=0.14)

        recovery = self._recovery(roll_progress_rad=0.15, max_roll_cycles=4)
        positions = self._positions()
        recovery.update(0.0, True, body(1.4), positions, {})
        assert recovery.phase == FallRecovery.TUCK
        # Times are just past each phase boundary, never exactly on it.
        roll_1 = FALL_RECOVER_TUCK_S + 0.01
        recovery.update(roll_1, True, body(1.4), positions, {})
        assert recovery.phase == FallRecovery.ROLL
        assert recovery.roll_cycles == 1
        # A cycle that bought real progress buys another one.
        tuck_2 = roll_1 + FALL_RECOVER_ROLL_S + 0.01
        recovery.update(tuck_2, True, body(1.0), positions, {})
        assert recovery.status == FallRecovery.ATTEMPTING
        assert recovery.phase == FallRecovery.TUCK
        roll_2 = tuck_2 + FALL_RECOVER_TUCK_S + 0.01
        recovery.update(roll_2, True, body(1.0), positions, {})
        assert recovery.phase == FallRecovery.ROLL
        assert recovery.roll_cycles == 2
        # One that does not ends the attempt at once, and says why.
        recovery.update(roll_2 + FALL_RECOVER_ROLL_S + 0.01, True,
                        body(0.95), positions, {})
        assert recovery.status == FallRecovery.FAILED
        assert recovery.last_reason.startswith("roll cycle bought less than")
        # The absolute cap still bounds an attempt that keeps improving.
        capped = self._recovery(roll_progress_rad=0.05, max_roll_cycles=2)
        now = 0.0
        for step in range(8):
            now += FALL_RECOVER_TUCK_S + 0.01
            capped.update(now, True, body(1.4 - 0.2 * step), positions, {})
            now += FALL_RECOVER_ROLL_S + 0.01
            capped.update(now, True, body(1.4 - 0.2 * step), positions, {})
            if capped.status != FallRecovery.ATTEMPTING:
                break
        assert capped.status == FallRecovery.FAILED
        assert capped.roll_cycles == 2
        assert capped.last_reason.startswith("roll cycles exhausted")

    def test_roll_phase_can_splay_the_free_pair_instead(self):
        # The second, unqualified hypothesis: keep the braced pair straight (it is
        # the support) and splay the *other* pair for the moment. Opt-in, and the
        # pair selection is independent of the braced-pair input.
        both = roll_phase_pose(1.2, 0.0, 0.4, 0.8)
        assert both["FR_hip_joint"] == pytest.approx(0.4)    # braced pair
        assert both["RR_hip_joint"] == pytest.approx(0.4)
        assert both["FL_hip_joint"] == pytest.approx(-0.8)   # free pair, mirrored
        assert both["RL_hip_joint"] == pytest.approx(-0.8)
        # Either input alone leaves the other pair at its waypoint's zero.
        assert roll_phase_pose(1.2, 0.0, 0.0, 0.8)["FR_hip_joint"] == pytest.approx(0.0)
        assert roll_phase_pose(1.2, 0.0, 0.4)["FL_hip_joint"] == pytest.approx(0.0)
        # The recovery passes it through and refuses a non-finite value.
        recovery = self._recovery(roll_free_hip_rad=0.8)
        positions = self._positions()
        recovery.update(0.0, True, self._collapsed(axis="roll"), positions, {})
        efforts = recovery.update(FALL_RECOVER_TUCK_S, True,
                                  self._collapsed(axis="roll"),
                                  positions, {})
        assert recovery.phase == FallRecovery.ROLL
        assert efforts == pytest.approx(
            self._stance(roll_phase_pose(1.2, 0.0, 0.0, 0.8), positions))
        with pytest.raises(ValueError):
            FallRecovery(roll_free_hip_rad=float("inf"))

    def test_crouch_holds_the_measured_splay_for_the_whole_phase(self):
        # Slewing the splay out over the crouch was tried, to let the legs gather
        # under the hips, and it measured *worse*: the trunk drops back out of the
        # gate mid-phase, the ladder re-tucks and the trials end inverted again
        # (fall_ladder_gather_20260928T*). The crouch therefore holds it, and the
        # 0.70-0.76 rad pose that leaves is a stable stop, not a trap.
        recovery = self._recovery(timeout_s=20.0, roll_brace_hip_rad=0.8)
        positions = self._positions()
        recovery.update(0.0, True, self._collapsed(), positions, {})
        # Still past the gate at the tuck exit, so the ladder takes the roll
        # route; under it at the roll exit, so it arrives in the crouch.
        recovery.update(FALL_RECOVER_TUCK_S, True, self._collapsed(), positions, {})
        assert recovery.phase == FallRecovery.ROLL
        rolled = BodyState(roll_rad=0.6, pitch_rad=0.0, body_height_m=0.14)
        crouch_at = FALL_RECOVER_TUCK_S + FALL_RECOVER_ROLL_S
        recovery.update(crouch_at, True, rolled, positions, {})
        assert recovery.phase == FallRecovery.CROUCH
        held = self._stance(crouch_phase_pose(0.6, 0.0, 0.8), positions)
        for offset in (0.01, FALL_RECOVER_CROUCH_S / 2.0,
                       FALL_RECOVER_CROUCH_S - 0.01):
            efforts = recovery.update(crouch_at + offset, True, rolled,
                                      positions, {})
            assert efforts == pytest.approx(held)
        assert efforts["FR_hip_joint"] != pytest.approx(0.0)
        # At the window's end the ladder hands over to the stand phase.
        recovery.update(crouch_at + FALL_RECOVER_CROUCH_S + 0.01, True,
                        rolled, positions, {})
        assert recovery.phase == FallRecovery.STAND

    def test_bounds_waypoints_and_gate_are_predeclared_and_consistent(self):
        assert FallRecovery.PHASES == (FallRecovery.TUCK, FallRecovery.ROLL,
                                       FallRecovery.CROUCH, FallRecovery.STAND)
        assert RECOVERY_PHASES == FallRecovery.PHASES
        # The gate sits inside the fall band: at or above the tilt a stand-up
        # is refused for anyway, and below the roll-over threshold.
        assert TILT_FALL_RAD <= FALL_RECOVER_GATE_TILT_RAD < FALL_INVERTED_TILT_RAD
        assert FALL_RECOVER_MAX_ROLL_CYCLES >= 1
        tuck, crouch, stand = (recovery_pose("tuck"), recovery_pose("crouch"),
                               recovery_pose("stand"))
        assert stand == nominal_stance_pose()
        thigh, calf = "FL_thigh_joint", "FL_calf_joint"
        assert tuck[thigh] > crouch[thigh] > stand[thigh]
        assert tuck[calf] < crouch[calf] < stand[calf]
        assert recovery_pose("brace")[thigh] < stand[thigh]

    def test_ladder_waypoints_stay_inside_the_measured_position_limits(self):
        poses = [recovery_pose(kind) for kind in ("tuck", "brace", "crouch")]
        poses += [roll_phase_pose(roll, pitch) for roll, pitch in
                  ((1.2, 0.0), (-1.2, 0.0), (0.0, 1.2), (0.0, -1.2))]
        for pose in poses:
            assert set(pose) == set(JOINT_NAMES)
            for name, value in pose.items():
                assert value == pytest.approx(clamp_position(name, value))
        with pytest.raises(ValueError):
            recovery_pose("somersault")

    def test_ladder_rejects_unsafe_parameters(self):
        with pytest.raises(ValueError):
            FallRecovery(gate_tilt_rad=0.0)
        with pytest.raises(ValueError):
            FallRecovery(gate_tilt_rad=FALL_INVERTED_TILT_RAD)
        with pytest.raises(ValueError):
            FallRecovery(tuck_s=0.0)
        with pytest.raises(ValueError):
            FallRecovery(roll_s=-0.1)
        with pytest.raises(ValueError):
            FallRecovery(crouch_s=0.0)
        with pytest.raises(ValueError):
            FallRecovery(max_roll_cycles=0)

    def test_reset_clears_the_ladder_state(self):
        recovery = self._recovery()
        positions = self._positions()
        recovery.update(0.0, True, self._collapsed(), positions, {})
        recovery.update(FALL_RECOVER_TUCK_S, True, self._collapsed(), positions, {})
        assert recovery.roll_cycles == 1
        recovery.reset()
        assert recovery.phase is None
        assert recovery.roll_cycles == 0
        assert recovery.last_reason is None
        assert recovery.state_label == FallRecovery.IDLE
