"""R5.2: Unitree Go2 locomotion core (stance, gait, estimation, safety).

Pure-logic locomotion layer for the Unitree Go2 quadruped. The ROS node
(``go2_stance_gait_controller.py``) is a thin wrapper around this module,
so every control law here is unit-testable without a live ROS graph.

Honest scope (R5.2 acceptance bar):

- Stance is a **closed-loop joint-space PD hold** around a nominal
  standing pose, using the measured PID gains from the Go2 description
  (robot_control.yaml: hip p=100/d=5, thigh p=300/d=8, calf p=300/d=8).
- Gait is a **bounded base-velocity interface**: commanded twist values
  are clamped to fixed bounds before use; a diagonal-pair trot generator
  turns the bounded twist into per-leg joint-space swing/stance targets.
  It is a bounded stepping gait, *not* a claim of model-predictive or
  full-body dynamics control.
- Contact output prefers measured MuJoCo foot-to-world force, falling back to
  the motor-effort residual heuristic when direct force is unavailable. Body state
  (roll/pitch) comes from the IMU quaternion; simulator body height comes from
  ground-truth odometry for the experimental recovery success check. Falls and excessive tilt
  force a SAFE_STOP (damped zero effort), never silent continuation.
- The opt-in get-up attempt is a bounded ladder of predeclared joint-space
    waypoints (tuck -> roll -> crouch -> stand) whose transitions read measured
    attitude; a roll entered from hip-supported contact additionally requires a
    bounded, dwelled transfer of load to the feet. The standing pose is commanded
    only once measured tilt is back under the stand-up gate, and the roll count is
    bounded. It is a measured-feedback sequence, not a trained get-up policy.
- All 12 joints are effort-commandable with per-joint limits taken from
  the Go2 description (const.xacro): hip/thigh 23.7 N·m, calf 35.55 N·m.
  **Raw effort publishing is not gait control**: every effort command is
  produced by this layer and clamped to the per-joint limits.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from robot_lab_utils.go2_support import GO2_SUPPORT_GEOM_NAMES

# ----------------------------------------------------------------------
# Robot constants (from go2_description/xacro/const.xacro and
# config/robot_control.yaml — measured, not invented)
# ----------------------------------------------------------------------

#: Leg prefixes in ros2_control joint ordering.
LEG_PREFIXES: Tuple[str, ...] = ("FL", "FR", "RL", "RR")
#: Joint kinds within each leg.
JOINT_KINDS: Tuple[str, ...] = ("hip", "thigh", "calf")

#: Canonical 12-joint ordering (matches go2_controllers.yaml).
JOINT_NAMES: Tuple[str, ...] = tuple(
    f"{leg}_{kind}_joint" for leg in LEG_PREFIXES for kind in JOINT_KINDS
)

#: Per-joint effort limits [N·m] (const.xacro: *_torque_max).
EFFORT_LIMITS: Dict[str, float] = {
    "hip": 23.7,
    "thigh": 23.7,
    "calf": 35.55,
}

#: Per-joint position limits [rad] (const.xacro: *_position_min/max).
POSITION_LIMITS: Dict[str, Tuple[float, float]] = {
    "hip": (-1.0472, 1.0472),
    "thigh": (-1.5708, 3.4907),
    "calf": (-2.7227, -0.83776),
}

#: Per-joint PD gains from config/robot_control.yaml.
PD_GAINS: Dict[str, Tuple[float, float]] = {
    # kind: (Kp, Kd)
    "hip": (100.0, 5.0),
    "thigh": (300.0, 8.0),
    "calf": (300.0, 8.0),
}

#: Nominal standing pose [rad] (hip, thigh, calf per leg) — the Go2
#: stand-up posture, inside the const.xacro position limits.
NOMINAL_STANCE: Dict[str, float] = {
    "hip": 0.0,
    "thigh": 0.72,
    "calf": -1.45,
}

#: Leg link lengths [m] (const.xacro: thigh_offset_y, calf_offset_z).
HIP_LATERAL_OFFSET_M = 0.0955
THIGH_LENGTH_M = 0.213
CALF_LENGTH_M = 0.213

#: Bounded base-velocity interface limits (fixed, predeclared).
BASE_VEL_LIMITS = {
    "vx": 0.8,    # m/s forward
    "vy": 0.4,    # m/s lateral
    "wz": 1.0,    # rad/s yaw rate
}
#: Base-velocity acceleration limits (per second).
BASE_ACC_LIMITS = {"vx": 1.0, "vy": 0.5, "wz": 2.0}

#: Safety thresholds (predeclared).
TILT_WARN_RAD = 0.35      # ~20 degrees: gait requests are refused
TILT_FALL_RAD = 0.70      # ~40 degrees: force SAFE_STOP
EFFORT_SATURATION_FRACTION = 0.98
EFFORT_SATURATION_CYCLES = 50  # sustained saturation at control rate

#: A single tilt spike must not be reported as a fall. After a fall-threshold
#: trip, the flag needs this many consecutive warn-or-higher observations.
FALL_CONFIRM_CYCLES = 25

#: A re-stand attempt succeeds only with measured upright tilt and standing
#: body height, and is abandoned after this bounded window.
FALL_RECOVER_TIMEOUT_S = 4.0
FALL_RECOVER_TILT_RAD = TILT_WARN_RAD
FALL_RECOVER_MIN_HEIGHT_M = 0.25
FALL_RECOVER_MIN_LOADED_FEET = 3
FALL_RECOVER_CONTACT_N = 2.0
FALL_RECOVER_SUCCESS_DWELL_S = 0.5
GO2_SUPPORT_WEIGHT_N = 126.53
FALL_RECOVER_SUPPORT_MIN_FOOT_N = 0.75 * GO2_SUPPORT_WEIGHT_N
FALL_RECOVER_SUPPORT_MAX_FOOT_N = 1.25 * GO2_SUPPORT_WEIGHT_N
FALL_RECOVER_SUPPORT_MAX_HIP_N = 0.25 * GO2_SUPPORT_WEIGHT_N
FALL_RECOVER_SUPPORT_MAX_TRUNK_N = 0.1 * GO2_SUPPORT_WEIGHT_N
FALL_RECOVER_SUPPORT_MAX_TOTAL_N = 1.4 * GO2_SUPPORT_WEIGHT_N
FALL_RECOVER_SUPPORT_DWELL_S = 0.25

#: Sequenced get-up ladder (R5.2 recovery attempt). Recorded 60 N trials drove
#: straight to the nominal stance pose from a down trunk at 0.14 m body height
#: and ended *inverted* at tilt ~pi and 0.057 m: from a fallen pose the stance
#: drive's thigh/calf torques lever the trunk over its feet instead of raising
#: it. The ladder stages the same bounded joint-space PD effort through
#: measured waypoints -- fold the legs in (retract the lever), brace the legs
#: the trunk rests on (roll), put the feet under the hips (crouch), then stand
#: -- and commands the standing pose only from the ``stand`` phase, which is
#: entered only when *measured* tilt is below
#: :data:`FALL_RECOVER_GATE_TILT_RAD`. This is a bounded, measured-feedback
#: sequence, not a trained get-up policy: the gate, the phase bounds and the
#: bounded roll count are invariants a test can check, whether the roll
#: primitive rights this plant is a measurement question.
FALL_RECOVER_GATE_TILT_RAD = 0.8
FALL_RECOVER_TUCK_S = 0.4
FALL_RECOVER_ROLL_S = 0.7
FALL_RECOVER_CROUCH_S = 0.6
#: The standing pose is *slewed* in from the crouch waypoint over this time
#: rather than stepped to. A step was measured to over-drive the legs out of
#: the crouch: the placed 1.4 rad nose-down trial left the ground at 0.56 m
#: with no foot loaded and the trunk pitching to 0.81 rad, and the ladder then
#: needed three retries to undo its own stand-up.
FALL_RECOVER_STAND_S = 0.6
FALL_RECOVER_MAX_ROLL_CYCLES = 4
#: A roll cycle is only worth repeating if it *measurably* brought the trunk
#: closer to upright. Measured: the free-pair splay walks a 1.4 rad flank down to
#: 0.49 rad, but slowly -- the fixed two-cycle budget expired at 1.81 s, just
#: before the trunk was under the gate, so a primitive that was working got
#: reported as a failure. The absolute cap above still bounds the attempt; this
#: is the rule that ends it as soon as a cycle stops making progress, which is
#: what the fixed count was standing in for.
FALL_RECOVER_ROLL_PROGRESS_RAD = 0.15

#: Ladder waypoints [rad] per joint kind, inside the measured position limits
#: (const.xacro). Forward kinematics with L1 = L2 = 0.213 m places the foot
#: this far below its hip in each pose:
#:
#:   tuck   (1.35, -2.70): 0.09 m, directly under the hip (leg folded flat)
#:   crouch (1.10, -2.20): 0.19 m, directly under the hip
#:   brace  (0.10, -0.90): 0.36 m, 0.13 m behind the hip (straightest the
#:          measured calf limit allows: 0.36 m is longer than the 0.32 m
#:          standing drop, so a braced leg can push the trunk up past it)
TUCK_POSE: Dict[str, float] = {"hip": 0.0, "thigh": 1.35, "calf": -2.70}
BRACE_POSE: Dict[str, float] = {"hip": 0.0, "thigh": 0.10, "calf": -0.90}
CROUCH_POSE: Dict[str, float] = {"hip": 0.0, "thigh": 1.10, "calf": -2.20}

#: Ladder waypoints by name, indexed by joint kind.
RECOVERY_WAYPOINTS: Dict[str, Dict[str, float]] = {
    "tuck": TUCK_POSE,
    "brace": BRACE_POSE,
    "crouch": CROUCH_POSE,
    "stand": NOMINAL_STANCE,
}

#: Ladder phase names, in the order :class:`FallRecovery` enters them.
RECOVERY_PHASE_TUCK = "tuck"
RECOVERY_PHASE_ROLL = "roll"
RECOVERY_PHASE_CROUCH = "crouch"
RECOVERY_PHASE_STAND = "stand"
RECOVERY_PHASES: Tuple[str, ...] = (
    RECOVERY_PHASE_TUCK, RECOVERY_PHASE_ROLL,
    RECOVERY_PHASE_CROUCH, RECOVERY_PHASE_STAND,
)

#: Beyond this tilt the trunk has rolled *past* the point a standing controller
#: can act from. :func:`classify_fall_pose` reports ``inverted`` here: the body
#: is on its back, and righting it needs a roll-over primitive, not a stand-up
#: one. Recorded 60 N trials ended at tilt 3.142 rad (pi) and 0.057 m, so this
#: separates "the controller failed" from "the pose was never recoverable".
FALL_INVERTED_TILT_RAD = 2.4


#: Terminal-pose classes returned by :func:`classify_fall_pose`.
FALL_POSE_UPRIGHT = "upright"
FALL_POSE_COLLAPSED = "collapsed"
FALL_POSE_INVERTED = "inverted"
FALL_POSE_UNKNOWN = "unknown"


def classify_fall_pose(body: Optional["BodyState"]) -> str:
    """Classify where the body ended up, from measured attitude and height.

    Distinguishes the two reasons a re-stand attempt can fail, which the
    bounded attempt must not conflate:

    - ``collapsed``: down but the trunk is not rolled past vertical. This is
      the pose a stand-up controller is meant to act from.
    - ``inverted``: the body is on its back or side, past
      :data:`FALL_INVERTED_TILT_RAD`. Standing effort cannot right it.

    Missing attitude or height reports ``unknown`` rather than guessing, so an
    unqualified result is never mistaken for a measured one.
    """
    if body is None:
        return FALL_POSE_UNKNOWN
    tilt = body.max_tilt_rad
    if not math.isfinite(tilt):
        return FALL_POSE_UNKNOWN
    if tilt >= FALL_INVERTED_TILT_RAD:
        return FALL_POSE_INVERTED
    if body.body_height_m is None or not math.isfinite(body.body_height_m):
        return FALL_POSE_UNKNOWN
    if body.body_height_m >= FALL_RECOVER_MIN_HEIGHT_M and \
            tilt < TILT_WARN_RAD:
        return FALL_POSE_UPRIGHT
    return FALL_POSE_COLLAPSED

#: Duty factor and cycle time for the bounded trot.
TROT_CYCLE_SECONDS = 0.7
TROT_DUTY = 0.5


def joint_kind(joint_name: str) -> str:
    """Return the joint kind ('hip'/'thigh'/'calf') for a joint name."""
    for kind in JOINT_KINDS:
        if joint_name.endswith(f"_{kind}_joint"):
            return kind
    raise ValueError(f"not a Go2 leg joint: {joint_name!r}")


def clamp_effort(joint_name: str, effort: float) -> float:
    """Clamp an effort command to the joint's measured effort limit."""
    limit = EFFORT_LIMITS[joint_kind(joint_name)]
    return max(-limit, min(limit, effort))


def clamp_position(joint_name: str, position: float) -> float:
    """Clamp a joint-space target to the joint's measured position range."""
    lo, hi = POSITION_LIMITS[joint_kind(joint_name)]
    return max(lo, min(hi, position))


# ----------------------------------------------------------------------
# Sequenced get-up waypoints (recovery ladder)
# ----------------------------------------------------------------------


def recovery_pose(kind: str) -> Dict[str, float]:
    """All 12 clamped joint targets for a predeclared ladder waypoint.

    ``kind`` indexes the measured waypoint table: ``tuck``, ``brace``,
    ``crouch`` or ``stand`` (the nominal stance pose).
    """
    if kind not in RECOVERY_WAYPOINTS:
        raise ValueError(f"not a recovery waypoint: {kind!r}")
    return {
        name: clamp_position(name, RECOVERY_WAYPOINTS[kind][joint_kind(name)])
        for name in JOINT_NAMES
    }


def brace_legs(roll_rad: float, pitch_rad: float) -> Tuple[str, ...]:
    """The leg pair the trunk is resting on, from *measured* attitude.

    A positive roll lifts the left flank, so the right legs are the ones
    trapped under the trunk; a positive pitch tips the trunk back onto its
    rear legs. The larger of the two measured angles decides, so a level
    trunk is never silently assumed to be lying on one side.
    """
    if abs(roll_rad) >= abs(pitch_rad):
        return ("FR", "RR") if roll_rad > 0.0 else ("FL", "RL")
    return ("RL", "RR") if pitch_rad > 0.0 else ("FL", "FR")


def roll_phase_pose(roll_rad: float, pitch_rad: float,
                    hip_rad: float = 0.0,
                    free_hip_rad: float = 0.0) -> Dict[str, float]:
    """Joint targets for the roll phase: brace the loaded pair, retract the rest.

    The pair the trunk rests on is driven toward the straight waypoint (the
    longest push the measured calf limit allows), while the other pair folds to
    the tuck waypoint, so the trunk's mass moves over the braced side instead
    of being levered over the feet.

    ``hip_rad`` and ``free_hip_rad`` are **opt-in, unqualified** lateral inputs,
    signed like the spawn pose's splayed stance (right leg positive, left leg
    mirrored) and clamped to the measured hip limits. Both are 0.0 by default
    because they encode two different hypotheses about the roll axis, and only
    the first has been measured:

    * ``hip_rad`` splays the *braced* pair. Measured: the sign is positive, and
      +0.8 rad is the magnitude that brings a flank-lying trunk under the
      0.8 rad gate (negative drives the trunk onto its back).
    * ``free_hip_rad`` splays the *other* pair instead, leaving the braced pair
      straight -- the classic "plant the upper legs for the moment, push with
      the lower ones" split. It exists to be measured on this plant, not because
      it is known to be right; the splayed-support pose that ``hip_rad`` leaves
      behind (the robot balanced at 0.70-0.76 rad) is stable but not standing,
      so a primitive that separates the support from the moment is the next
      thing to try.

    Placed-pose trials are how both are measured; neither is a default.
    """
    braced = brace_legs(roll_rad, pitch_rad)
    pose: Dict[str, float] = {}
    for name in JOINT_NAMES:
        leg = name.split("_")[0]
        table = BRACE_POSE if leg in braced else TUCK_POSE
        if joint_kind(name) == "hip":
            splay = hip_rad if leg in braced else free_hip_rad
            if splay:
                # The Go2's hip roll axes are opposed, so a splay is mirrored.
                pose[name] = clamp_position(
                    name, splay if leg.endswith("R") else -splay)
                continue
        pose[name] = clamp_position(name, table[joint_kind(name)])
    return pose


def crouch_phase_pose(roll_rad: float, pitch_rad: float,
                      hip_rad: float = 0.0) -> Dict[str, float]:
    """The crouch waypoint, keeping the lateral input while roll dominates.

    Measured: with the measured +0.8 rad splay the ladder got a flank-lying
    trunk under the gate for the first time and then failed *here*, because the
    plain crouch waypoint puts the hips back to zero and levers the trunk over
    again mid-transition. So the crouch keeps the same input on the same
    measured pair -- and only while roll dominates. A pitch-dominated pose gets
    the plain crouch waypoint, which is the path that measurably works.
    """
    pose = recovery_pose("crouch")
    if not hip_rad or abs(roll_rad) < abs(pitch_rad):
        return pose
    for name in JOINT_NAMES:
        leg = name.split("_")[0]
        if joint_kind(name) != "hip":
            continue
        if leg in brace_legs(roll_rad, pitch_rad):
            pose[name] = clamp_position(
                name, hip_rad if leg.endswith("R") else -hip_rad)
    return pose


# ----------------------------------------------------------------------
# Stance: closed-loop joint-space PD hold
# ----------------------------------------------------------------------


def nominal_stance_pose() -> Dict[str, float]:
    """The nominal standing pose for all 12 joints (clamped to limits)."""
    return {
        name: clamp_position(name, NOMINAL_STANCE[joint_kind(name)])
        for name in JOINT_NAMES
    }


@dataclass
class StanceController:
    """Closed-loop stance: PD hold around the nominal standing pose.

    tau = Kp * (q* - q) + Kd * (qdot* - qdot), clamped per joint.
    This is the "closed-loop stance" the R5.2 acceptance requires;
    open-loop raw effort publication is deliberately not offered.
    """

    target: Dict[str, float] = field(default_factory=nominal_stance_pose)
    gain_scale: float = 1.0
    damping_scale: float = 1.0

    def __post_init__(self) -> None:
        # Targets are always clamped to the measured position limits.
        self.target = {
            name: clamp_position(name, value) for name, value in self.target.items()
        }

    def effort_command(
        self,
        positions: Dict[str, float],
        velocities: Dict[str, float],
    ) -> Dict[str, float]:
        """Compute clamped PD efforts for all 12 joints.

        A joint missing from ``positions`` produces **zero effort** (no
        measurement -> no drive, never an assumed pose); a joint with a
        position but no velocity uses zero joint velocity for the damping
        term.
        """
        command: Dict[str, float] = {}
        for name in JOINT_NAMES:
            if name not in positions:
                command[name] = 0.0
                continue
            q_star = self.target[name]
            q = positions[name]
            qdot = velocities.get(name, 0.0)
            kp, kd = PD_GAINS[joint_kind(name)]
            tau = self.gain_scale * kp * (q_star - q) \
                + self.damping_scale * kd * (0.0 - qdot)
            command[name] = clamp_effort(name, tau)
        return command


# ----------------------------------------------------------------------
# Bounded base-velocity interface
# ----------------------------------------------------------------------


@dataclass(frozen=True)
class BaseVelocity:
    """A commanded base twist (m/s, m/s, rad/s)."""

    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0

    def is_zero(self) -> bool:
        return abs(self.vx) < 1e-9 and abs(self.vy) < 1e-9 and abs(self.wz) < 1e-9


def clamp_base_velocity(command: BaseVelocity) -> BaseVelocity:
    """Clamp a twist to the bounded interface limits."""
    return BaseVelocity(
        vx=max(-BASE_VEL_LIMITS["vx"], min(BASE_VEL_LIMITS["vx"], command.vx)),
        vy=max(-BASE_VEL_LIMITS["vy"], min(BASE_VEL_LIMITS["vy"], command.vy)),
        wz=max(-BASE_VEL_LIMITS["wz"], min(BASE_VEL_LIMITS["wz"], command.wz)),
    )


def rate_limit_base_velocity(
    previous: BaseVelocity, command: BaseVelocity, dt: float
) -> BaseVelocity:
    """Apply acceleration limits to a commanded twist over dt seconds."""
    if dt <= 0.0:
        return clamp_base_velocity(command)
    bounded = clamp_base_velocity(command)
    prev = clamp_base_velocity(previous)

    def _limit(axis: str, target: float, current: float) -> float:
        max_step = BASE_ACC_LIMITS[axis] * dt
        return current + max(-max_step, min(max_step, target - current))

    return BaseVelocity(
        vx=_limit("vx", bounded.vx, prev.vx),
        vy=_limit("vy", bounded.vy, prev.vy),
        wz=_limit("wz", bounded.wz, prev.wz),
    )


# ----------------------------------------------------------------------
# Bounded trot gait: diagonal-pair stepping from a bounded twist
# ----------------------------------------------------------------------

#: Diagonal pairs of the trot: (FL, RR) and (FR, RL).
TROT_DIAGONAL_PAIRS: Tuple[Tuple[str, str], Tuple[str, str]] = (("FL", "RR"), ("FR", "RL"))

#: Swing lift [rad]; the thigh sweeps forward while the folded calf clears
#: the floor. A stance leg sweeps backward relative to the trunk.
SWING_THIGH_OFFSET_RAD = 0.35
SWING_CALF_OFFSET_RAD = -0.5
MAX_STRIDE_RAD = SWING_THIGH_OFFSET_RAD
STRIDE_RAD_PER_MPS = 0.8


def leg_phase(leg: str, phase: float) -> float:
    """Normalized phase (0..1) of *leg* within the trot cycle.

    Diagonal pairs move together: FL/RR share phase 0, FR/RL lead by 0.5.
    """
    if leg not in LEG_PREFIXES:
        raise ValueError(f"unknown leg {leg!r}")
    lead = 0.0 if leg in TROT_DIAGONAL_PAIRS[0] else 0.5
    return (phase + lead) % 1.0


def leg_in_stance(leg: str, phase: float) -> bool:
    """True while *leg* is in stance (duty factor of the cycle)."""
    return leg_phase(leg, phase) < TROT_DUTY


def trot_joint_targets(phase: float, velocity: BaseVelocity) -> Dict[str, float]:
    """Joint-space swing/stance targets for all 12 joints at *phase*.

    Stance feet sweep backward relative to the trunk; swing feet return
    forward with a folded calf for clearance. Yaw commands give left and
    right legs different fore-aft speeds. This is a bounded joint-space
    stepping law, not a model-predictive dynamics controller.
    """
    targets: Dict[str, float] = {}
    for name in JOINT_NAMES:
        leg = name.split("_")[0]
        kind = joint_kind(name)
        p = leg_phase(leg, phase)
        side = 1.0 if leg in ("FL", "RL") else -1.0
        fore_aft_speed = velocity.vx - side * velocity.wz * (HIP_LATERAL_OFFSET_M + 0.0465)
        amplitude = max(-MAX_STRIDE_RAD, min(MAX_STRIDE_RAD,
                                            STRIDE_RAD_PER_MPS * fore_aft_speed))
        if p < TROT_DUTY:
            sweep = -amplitude + 2.0 * amplitude * p / TROT_DUTY
            lift = 0.0
        else:
            swing_p = (p - TROT_DUTY) / (1.0 - TROT_DUTY)
            sweep = amplitude * (1.0 - 2.0 * swing_p)
            lift = math.sin(math.pi * swing_p) * min(
                1.0, abs(fore_aft_speed) / BASE_VEL_LIMITS["vx"])
        raw = NOMINAL_STANCE[kind]
        if kind == "thigh":
            raw += sweep
        elif kind == "calf":
            raw += SWING_CALF_OFFSET_RAD * lift
        targets[name] = clamp_position(name, raw)
    return targets


# ----------------------------------------------------------------------
# Effort-residual contact estimation (per leg, honest and local)
# ----------------------------------------------------------------------

#: Residual below which a stance leg is considered loaded/in contact [N·m].
CONTACT_RESIDUAL_THRESHOLD_NM = 8.0


@dataclass(frozen=True)
class LegObservation:
    """Measured joint state and the command actually sent for one leg."""

    measured_effort: Dict[str, float]      # joint -> measured effort
    commanded_effort: Dict[str, float]     # joint -> effort this cycle sent
    position_error: Dict[str, float]       # joint -> |measured - target|


def leg_contact_residual(leg: str, observation: LegObservation) -> float:
    """Mean absolute motor-effort residual across the leg's joints [N·m].

    This is a command-tracking heuristic. An ideal effort motor reports
    actuator torque almost equal to the command even without ground contact;
    physics foot-contact truth is still required for contact qualification.
    """
    residuals = []
    for kind in JOINT_KINDS:
        joint = f"{leg}_{kind}_joint"
        measured = observation.measured_effort.get(joint)
        commanded = observation.commanded_effort.get(joint)
        if measured is None or commanded is None:
            return float("nan")  # missing data -> unknown, never zero
        residuals.append(abs(measured - commanded))
    return sum(residuals) / len(residuals)


def estimate_leg_contacts(
    observations: Dict[str, LegObservation], stance_flags: Dict[str, bool]
) -> Dict[str, Optional[bool]]:
    """Per-leg contact estimate: True (contact), False (swing), None (unknown).

    Only stance legs can be in contact; a swing leg is reported False. A
    stance leg with missing/NaN data reports None rather than guessing.
    """
    result: Dict[str, Optional[bool]] = {}
    for leg in LEG_PREFIXES:
        if not stance_flags.get(leg, False):
            result[leg] = False
            continue
        residual = leg_contact_residual(leg, observations[leg])
        if math.isnan(residual):
            result[leg] = None
        else:
            result[leg] = residual <= CONTACT_RESIDUAL_THRESHOLD_NM
    return result


# ----------------------------------------------------------------------
# Body state from IMU (roll/pitch) and latched safety monitor
# ----------------------------------------------------------------------


@dataclass(frozen=True)
class BodyState:
    """Attitude snapshot used by the safety monitor."""

    roll_rad: float
    pitch_rad: float
    body_height_m: Optional[float] = None  # None if unavailable

    @classmethod
    def from_quaternion(cls, x: float, y: float, z: float, w: float) -> "BodyState":
        """Roll/pitch from an IMU quaternion (ZYX convention)."""
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
        return cls(roll_rad=roll, pitch_rad=pitch)

    @property
    def max_tilt_rad(self) -> float:
        return max(abs(self.roll_rad), abs(self.pitch_rad))


class SafetyState:
    """Latched safety monitor: NOMINAL -> WARN -> SAFE_STOP.

    SAFE_STOP is entered on excessive tilt or sustained effort saturation
    and is *latched*: recovery requires an explicit ``reset()`` by the
    operator, never silent continuation.
    """

    NOMINAL = "nominal"
    WARN = "warn"
    SAFE_STOP = "safe_stop"

    def __init__(self) -> None:
        self.state = self.NOMINAL
        self.saturation_counters: Dict[str, int] = {j: 0 for j in JOINT_NAMES}
        self.reason: Optional[str] = None
        #: Latched "the robot is down" flag, distinct from SAFE_STOP. SAFE_STOP
        #: means "stop driving"; this means "a get-up attempt is required". It
        #: is deliberately *not* cleared by attitude recovering, only by reset().
        self.fallen = False
        self.fall_reason: Optional[str] = None
        self._fall_cycles = 0
        self._tilt_trip_seen = False

    def reset(self) -> None:
        self.state = self.NOMINAL
        self.saturation_counters = {j: 0 for j in JOINT_NAMES}
        self.reason = None
        self.fallen = False
        self.fall_reason = None
        self._fall_cycles = 0
        self._tilt_trip_seen = False

    def observe_body(self, body: BodyState) -> List[str]:
        """Update state from attitude; returns any new issue strings."""
        issues: List[str] = []
        tilt = body.max_tilt_rad
        if tilt >= TILT_FALL_RAD:
            self._tilt_trip_seen = True
            self._enter_safe_stop(f"tilt {tilt:.2f} rad exceeds fall threshold")
            issues.append(f"safe_stop: tilt {tilt:.2f} rad exceeds fall threshold")
        if self._tilt_trip_seen and tilt >= TILT_WARN_RAD:
            self._fall_cycles += 1
            if self._fall_cycles >= FALL_CONFIRM_CYCLES and not self.fallen:
                self.fallen = True
                self.fall_reason = (
                    "sustained tilt %.2f rad after a fall-threshold trip" % tilt)
                issues.append("fallen: " + self.fall_reason)
        elif not self.fallen:
            self._fall_cycles = 0
            self._tilt_trip_seen = False
        if tilt < TILT_FALL_RAD:
            if tilt >= TILT_WARN_RAD:
                if self.state == self.NOMINAL:
                    self.state = self.WARN
                    self.reason = f"tilt {tilt:.2f} rad exceeds warn threshold"
                    issues.append(f"warn: {self.reason}")
            elif self.state == self.WARN:
                self.state = self.NOMINAL
                self.reason = None
        return issues

    def observe_efforts(
        self, commanded: Dict[str, float], measured: Dict[str, float]
    ) -> List[str]:
        """Track sustained per-joint effort saturation; returns issues."""
        issues: List[str] = []
        for joint in JOINT_NAMES:
            measured_value = measured.get(joint)
            commanded_value = commanded.get(joint)
            if measured_value is None or commanded_value is None:
                self.saturation_counters[joint] = 0
                continue
            limit = EFFORT_LIMITS[joint_kind(joint)]
            saturated = (
                abs(commanded_value) >= EFFORT_SATURATION_FRACTION * limit
                and abs(measured_value) >= EFFORT_SATURATION_FRACTION * limit
            )
            if saturated:
                self.saturation_counters[joint] += 1
            else:
                self.saturation_counters[joint] = 0
            if self.saturation_counters[joint] >= EFFORT_SATURATION_CYCLES:
                self._enter_safe_stop(f"sustained effort saturation on {joint}")
                issues.append(f"safe_stop: sustained effort saturation on {joint}")
        return issues

    def _enter_safe_stop(self, reason: str) -> None:
        if self.state != self.SAFE_STOP:
            self.state = self.SAFE_STOP
            self.reason = reason

    def gait_permitted(self) -> bool:
        return self.state != self.SAFE_STOP

    def safe_stop_efforts(self) -> Dict[str, float]:
        """Damped zero effort for every joint: the SAFE_STOP command."""
        return {joint: 0.0 for joint in JOINT_NAMES}


class FallRecovery:
    """Bounded, opt-in sequenced get-up attempt after a latched fall.

    This is a get-up **attempt**, not a demonstrated get-up. It advances a
    bounded ladder of predeclared joint-space waypoints, using measured state
    only:

    ``tuck`` (fold the legs in) -> ``roll`` (brace the legs the trunk rests
    on, retract the others) -> ``crouch`` (feet under the hips) -> ``stand``
    (the nominal stance pose).

    The ladder replaces the single-phase version of this class, which drove
    the nominal stance pose straight from a fallen trunk and *rolled the trunk
    over*: the recorded 60 N trials ended inverted at tilt ~pi and 0.057 m
    body height, the opposite of standing up. Two rules follow from that
    evidence, and they are what this class guarantees:

    - the nominal standing pose is commanded only from the ``stand`` phase,
      and that phase is entered only once *measured* tilt is under
      ``gate_tilt_rad``;
    - every phase is bounded, the roll count is bounded, and the attempt
      window ends the attempt anyway — a trunk that stays past the gate is
      never levered further over, and the ladder never restarts by itself.

    Success is reported only from measured evidence: tilt below
    ``success_tilt_rad``, body height at or above the standing threshold, at
    least three feet loaded above the contact threshold, held for a measured
    dwell. A phase entered without a measured attitude is held, never guessed.
    A succeeded attempt then *keeps* that evidence by holding the nominal
    stance: the node calls this method for as long as ``fallen`` is latched, so
    a zero-effort return at success is a command that drops the robot it just
    stood up. The hold stops the moment the same evidence is gone, and neither
    the other terminal states nor the latched :attr:`SafetyState.fallen` flag
    are ever cleared from here.
    When the attempt stops, a terminal pose rolled past
    :data:`FALL_INVERTED_TILT_RAD` is reported as :attr:`UNRECOVERABLE` rather
    than :attr:`FAILED`, so the evidence does not imply the gains were merely
    too weak. It never silently resumes walking, and it never clears the
    latched :attr:`SafetyState.fallen` flag on its own.
    """

    IDLE = "idle"
    WAITING = "waiting"
    ATTEMPTING = "attempting"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    #: The attempt ended with the body inverted; standing effort cannot right it.
    UNRECOVERABLE = "unrecoverable"

    #: Terminal states: the attempt is over and will not restart by itself.
    TERMINAL_STATES = (SUCCEEDED, FAILED, UNRECOVERABLE)

    #: Ladder phases; :attr:`PHASES` is the order they are entered in.
    TUCK = RECOVERY_PHASE_TUCK
    ROLL = RECOVERY_PHASE_ROLL
    CROUCH = RECOVERY_PHASE_CROUCH
    STAND = RECOVERY_PHASE_STAND
    PHASES = RECOVERY_PHASES

    def __init__(
        self,
        timeout_s: float = FALL_RECOVER_TIMEOUT_S,
        start_delay_s: float = 0.0,
        success_tilt_rad: float = FALL_RECOVER_TILT_RAD,
        gain_scale: float = 0.5,
        damping_scale: float = 0.5,
        gate_tilt_rad: float = FALL_RECOVER_GATE_TILT_RAD,
        tuck_s: float = FALL_RECOVER_TUCK_S,
        roll_s: float = FALL_RECOVER_ROLL_S,
        crouch_s: float = FALL_RECOVER_CROUCH_S,
        stand_s: float = FALL_RECOVER_STAND_S,
        roll_brace_hip_rad: float = 0.0,
        roll_free_hip_rad: float = 0.0,
        max_roll_cycles: int = FALL_RECOVER_MAX_ROLL_CYCLES,
        roll_progress_rad: float = FALL_RECOVER_ROLL_PROGRESS_RAD,
    ) -> None:
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive")
        if not math.isfinite(start_delay_s) or start_delay_s < 0.0:
            raise ValueError("start_delay_s must be nonnegative")
        if not math.isfinite(success_tilt_rad) or success_tilt_rad <= 0.0:
            raise ValueError("success_tilt_rad must be positive")
        if not 0.0 < gain_scale <= 1.0:
            raise ValueError("gain_scale must be within (0, 1]")
        if damping_scale <= 0.0:
            raise ValueError("damping_scale must be positive")
        # The gate is the tilt a stand-up may still be driven from: above zero
        # and below the roll-over threshold, or the ladder would never stand
        # (gate 0) or would stand from an inverted trunk (gate past vertical).
        if not 0.0 < gate_tilt_rad < FALL_INVERTED_TILT_RAD:
            raise ValueError(
                "gate_tilt_rad must be within (0, FALL_INVERTED_TILT_RAD)")
        for name, value in (("tuck_s", tuck_s), ("roll_s", roll_s),
                            ("crouch_s", crouch_s), ("stand_s", stand_s)):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive")
        if max_roll_cycles < 1:
            raise ValueError("max_roll_cycles must be at least one")
        if not math.isfinite(roll_progress_rad) or roll_progress_rad < 0.0:
            raise ValueError("roll_progress_rad must be finite and nonnegative")
        if (not math.isfinite(roll_brace_hip_rad)
                or not math.isfinite(roll_free_hip_rad)):
            raise ValueError("roll hip inputs must be finite")
        self.timeout_s = timeout_s
        self.start_delay_s = start_delay_s
        self.success_tilt_rad = success_tilt_rad
        self.gate_tilt_rad = gate_tilt_rad
        self.tuck_s = tuck_s
        self.roll_s = roll_s
        self.crouch_s = crouch_s
        self.stand_s = stand_s
        self.roll_brace_hip_rad = roll_brace_hip_rad
        self.roll_free_hip_rad = roll_free_hip_rad
        self.max_roll_cycles = max_roll_cycles
        self.roll_progress_rad = roll_progress_rad
        self.status = self.IDLE
        self.started_at: Optional[float] = None
        self.active_started_at: Optional[float] = None
        self.success_candidate_at: Optional[float] = None
        self._waiting_stance: Optional[StanceController] = None
        #: Ladder phase, or None while no attempt is running.
        self.phase: Optional[str] = None
        self.phase_started_at: Optional[float] = None
        #: Roll (brace-and-push) cycles spent inside this attempt.
        self.roll_cycles = 0
        #: Measured tilt when the current roll cycle began, for the progress rule.
        self._roll_entry_tilt: Optional[float] = None
        self._support_transfer_required = False
        self._support_transfer_since: Optional[float] = None
        #: Why the attempt stopped, when it did.
        self.last_reason: Optional[str] = None
        self.attempts = 0
        #: Pose class measured when the attempt reached a terminal state.
        self.terminal_pose: Optional[str] = None
        self._gain_scale = gain_scale
        self._damping_scale = damping_scale
        self._stance = StanceController(
            target=nominal_stance_pose(),
            gain_scale=gain_scale,
            damping_scale=damping_scale,
        )

    @property
    def state_label(self) -> str:
        """``status`` with the ladder phase appended once the attempt begins.

        The ROS node publishes this on ``/go2/recovery_state`` so a trial
        trace shows *where* in the ladder the attempt was: the recorded traces
        before the ladder existed only ever showed ``attempting``.
        """
        return self.status if self.phase is None else f"{self.status}:{self.phase}"

    def reset(self) -> None:
        self.status = self.IDLE
        self.started_at = None
        self.active_started_at = None
        self.success_candidate_at = None
        self._waiting_stance = None
        self.terminal_pose = None
        self.phase = None
        self.phase_started_at = None
        self.roll_cycles = 0
        self._roll_entry_tilt = None
        self._support_transfer_required = False
        self._support_transfer_since = None
        self.last_reason = None

    def update(
        self,
        now_s: float,
        fallen: bool,
        body: Optional[BodyState],
        positions: Dict[str, float],
        velocities: Dict[str, float],
        measured_contact_forces: Optional[Dict[str, float]] = None,
        measured_support_forces: Optional[Dict[str, float]] = None,
    ) -> Dict[str, float]:
        """Return the re-stand effort for this cycle (``{}`` means zero drive).

        ``body`` and ``positions`` may be missing: without measurements this
        degrades to zero effort rather than assuming a pose.
        """
        if self.status == self.SUCCEEDED:
            # A completed get-up must keep driving the pose it just reached.
            # Releasing the drive here drops a robot that is *standing*: the
            # placed 1.4 rad nose-down trial reached 0.33 m with four feet
            # loaded, reported ``succeeded:stand`` at 4.948 s, and was flat on
            # its belly at 0.057 m 0.24 s later — the node keeps calling this
            # method while ``fallen`` is latched, so zero effort *is* the
            # command. The hold is conditional on the same measured evidence
            # that granted the success, so leaving the standing envelope stops
            # the drive instead of masking a fall.
            if self._standing_evidence(body, measured_contact_forces):
                return self._hold_efforts(positions, velocities)
            return {joint: 0.0 for joint in JOINT_NAMES}
        if self.status in self.TERMINAL_STATES or not fallen:
            return {joint: 0.0 for joint in JOINT_NAMES}
        if self.status == self.IDLE:
            self.status = self.WAITING if self.start_delay_s > 0.0 else self.ATTEMPTING
            self.started_at = now_s
            if self.status == self.ATTEMPTING:
                self.active_started_at = now_s
                self._enter_phase(self.TUCK, now_s)
            else:
                self._waiting_stance = self._capture_waiting_stance(positions)
            self.attempts += 1
        if self.status == self.WAITING:
            if self.started_at is not None and now_s - self.started_at < self.start_delay_s:
                return self._waiting_efforts(positions, velocities)
            self.status = self.ATTEMPTING
            self.active_started_at = now_s
            self._waiting_stance = None
            self._enter_phase(self.TUCK, now_s)
        if self.active_started_at is not None and now_s - self.active_started_at > self.timeout_s:
            self._end_attempt(body, "get-up window expired")
            return {joint: 0.0 for joint in JOINT_NAMES}
        if self._standing_evidence(body, measured_contact_forces):
            if self.success_candidate_at is None:
                self.success_candidate_at = now_s
            elif now_s - self.success_candidate_at >= FALL_RECOVER_SUCCESS_DWELL_S:
                self.status = self.SUCCEEDED
                self.terminal_pose = classify_fall_pose(body)
                return {joint: 0.0 for joint in JOINT_NAMES}
        else:
            self.success_candidate_at = None
        # The ladder is advanced only after the standing check: measured
        # evidence that the trunk is already up wins over phase timing, so a
        # robot that came up is never re-retracted by a stale phase.
        self._advance_ladder(now_s, body, measured_support_forces)
        if self.status in self.TERMINAL_STATES:
            return {joint: 0.0 for joint in JOINT_NAMES}
        efforts = self._phase_efforts(
            positions, velocities, body, now_s, measured_support_forces)
        for joint in JOINT_NAMES:
            if joint not in positions:
                efforts[joint] = 0.0
        return efforts

    def _capture_waiting_stance(
        self, positions: Dict[str, float]
    ) -> Optional[StanceController]:
        if not all(
            name in positions
            and math.isfinite(positions[name])
            and POSITION_LIMITS[joint_kind(name)][0]
            <= positions[name] <= POSITION_LIMITS[joint_kind(name)][1]
            for name in JOINT_NAMES
        ):
            return None
        return StanceController(
            target={name: positions[name] for name in JOINT_NAMES},
            gain_scale=self._gain_scale,
            damping_scale=self._damping_scale,
        )

    def _waiting_efforts(
        self, positions: Dict[str, float], velocities: Dict[str, float]
    ) -> Dict[str, float]:
        if self._waiting_stance is None:
            return {joint: 0.0 for joint in JOINT_NAMES}
        return self._waiting_stance.effort_command(positions, velocities)

    def _standing_evidence(
        self,
        body: Optional[BodyState],
        measured_contact_forces: Optional[Dict[str, float]],
    ) -> bool:
        """Measured evidence of standing, exactly as the success gate defines it.

        Shared by the success dwell and the post-success hold, so a get-up can
        never be declared on evidence its own hold would reject: tilt under
        :attr:`success_tilt_rad`, body height at or above
        :data:`FALL_RECOVER_MIN_HEIGHT_M`, and at least
        :data:`FALL_RECOVER_MIN_LOADED_FEET` feet above
        :data:`FALL_RECOVER_CONTACT_N`. A missing measurement is never
        evidence: no body or no contact data means ``False``.
        """
        loaded_feet = sum(
            1 for leg in LEG_PREFIXES
            if measured_contact_forces is not None
            and math.isfinite(measured_contact_forces.get(leg, float("nan")))
            and measured_contact_forces[leg] >= FALL_RECOVER_CONTACT_N)
        return (
            body is not None
            and body.max_tilt_rad < self.success_tilt_rad
            and body.body_height_m is not None
            and body.body_height_m >= FALL_RECOVER_MIN_HEIGHT_M
            and loaded_feet >= FALL_RECOVER_MIN_LOADED_FEET)

    def _hold_efforts(
        self,
        positions: Dict[str, float],
        velocities: Dict[str, float],
    ) -> Dict[str, float]:
        """Nominal-stance PD that keeps a *completed* get-up standing.

        The same stance law the ``stand`` phase drives, at the recovery's own
        gain scales, with the missing-measurement rule used everywhere in this
        file: a joint without a measurement gets zero effort, never an assumed
        pose.
        """
        efforts = self._stance.effort_command(positions, velocities)
        for joint in JOINT_NAMES:
            if joint not in positions:
                efforts[joint] = 0.0
        return efforts

    # -- ladder internals (all transitions read *measured* state) --------

    @staticmethod
    def _measured_tilt(body: Optional[BodyState]) -> Optional[float]:
        """Measured tilt [rad], or None when it is missing or not finite."""
        if body is None:
            return None
        tilt = body.max_tilt_rad
        return tilt if math.isfinite(tilt) else None

    def _enter_phase(self, phase: str, now_s: float) -> None:
        if phase not in self.PHASES:
            raise ValueError(f"not a recovery phase: {phase!r}")
        self.phase = phase
        self.phase_started_at = now_s

    def _enter_roll(
        self,
        now_s: float,
        body: Optional[BodyState] = None,
        measured_support_forces: Optional[Dict[str, float]] = None,
    ) -> None:
        self.roll_cycles += 1
        self._roll_entry_tilt = self._measured_tilt(body)
        self._support_transfer_required = self._roll_starts_hip_supported(
            body, measured_support_forces)
        self._support_transfer_since = None
        self._enter_phase(self.ROLL, now_s)

    def _end_attempt(self, body: Optional[BodyState], reason: str) -> None:
        """Stop driving and record why, from the *measured* terminal pose.

        An inverted terminal pose is reported as UNRECOVERABLE rather than
        FAILED so the evidence does not imply the gains were merely too weak;
        a collapsed one stays FAILED, a controller shortfall.
        """
        self.terminal_pose = classify_fall_pose(body)
        self.status = (self.UNRECOVERABLE
                       if self.terminal_pose == FALL_POSE_INVERTED
                       else self.FAILED)
        self.last_reason = reason

    def _advance_ladder(
        self,
        now_s: float,
        body: Optional[BodyState],
        measured_support_forces: Optional[Dict[str, float]] = None,
    ) -> None:
        """Advance the bounded ladder on measured attitude and support transfer.

        Without a measured attitude the phase is held (and the attempt window
        still bounds it): the ladder never guesses a transition. ``stand`` is
        entered only once the trunk is measured under ``gate_tilt_rad``, and a
        trunk that climbs back out of the gate goes back down the ladder
        instead of being levered further over. A roll phase that begins from a
        hip-supported flank must also establish a bounded, dwelled foot-load
        transfer before it can advance.
        """
        if self.phase_started_at is None:
            return
        tilt = self._measured_tilt(body)
        if tilt is None:
            return
        elapsed = now_s - self.phase_started_at
        if self.phase == self.TUCK:
            if elapsed < self.tuck_s:
                return
            if tilt >= FALL_INVERTED_TILT_RAD:
                # Legs in the air: there is nothing to brace against, so the
                # bounded answer is to keep the legs retracted and let the
                # window decide, never to drive a standing pose from here.
                self._enter_phase(self.TUCK, now_s)
            elif tilt >= self.gate_tilt_rad:
                self._enter_roll(now_s, body, measured_support_forces)
            else:
                self._enter_phase(self.CROUCH, now_s)
            return
        if self.phase == self.ROLL:
            if self._support_transfer_required:
                if self._has_support_transfer(body, measured_support_forces):
                    if self._support_transfer_since is None:
                        self._support_transfer_since = now_s
                    elif now_s - self._support_transfer_since >= \
                            FALL_RECOVER_SUPPORT_DWELL_S:
                        self._support_transfer_required = False
                        self._support_transfer_since = None
                        self._roll_entry_tilt = None
                        self._enter_phase(self.CROUCH, now_s)
                        return
                else:
                    self._support_transfer_since = None
            if elapsed < self.roll_s:
                return
            if self._support_transfer_required:
                self._end_attempt(
                    body, "roll stroke ended without settled foot support transfer")
            elif tilt >= FALL_INVERTED_TILT_RAD:
                self._end_attempt(
                    body, "trunk rolled past %.1f rad during the roll phase"
                    % FALL_INVERTED_TILT_RAD)
            elif tilt >= self.gate_tilt_rad:
                self._retry_or_end(body, now_s)
            else:
                # The gate is passed, so no roll cycle is in progress any more: a
                # later climb-back re-tucks on the count, not the progress rule.
                self._roll_entry_tilt = None
                self._enter_phase(self.CROUCH, now_s)
            return
        if self.phase == self.CROUCH:
            if elapsed < self.crouch_s:
                return
            if tilt >= FALL_INVERTED_TILT_RAD:
                self._end_attempt(
                    body, "trunk rolled past %.1f rad during the crouch phase"
                    % FALL_INVERTED_TILT_RAD)
            elif tilt >= self.gate_tilt_rad:
                self._retry_or_end(body, now_s)
            else:
                self._enter_phase(self.STAND, now_s)
            return
        if tilt >= FALL_INVERTED_TILT_RAD:
            self._end_attempt(
                body, "standing pose drove the trunk past %.1f rad"
                % FALL_INVERTED_TILT_RAD)
        elif tilt >= self.gate_tilt_rad:
            self._retry_or_end(body, now_s)

    def _retry_or_end(self, body: Optional[BodyState], now_s: float) -> None:
        """Spend another roll cycle, or stop once the attempt stops progressing.

        The fixed cycle count used to decide this, and it ended attempts that
        were still measurably working: the free-pair splay takes a 1.4 rad flank
        down to 0.49 rad, but needs longer than two cycles to get there. A cycle
        is now only repeated while the measured tilt keeps improving by
        :data:`FALL_RECOVER_ROLL_PROGRESS_RAD`, and the attempt ends as soon as
        one does not -- which is the flail guard the count was standing in for.
        :data:`FALL_RECOVER_MAX_ROLL_CYCLES` remains as the absolute cap.
        """
        tilt = self._measured_tilt(body)
        if self.roll_cycles >= self.max_roll_cycles:
            self._end_attempt(
                body, "roll cycles exhausted above the %.2f rad gate"
                % self.gate_tilt_rad)
            return
        if self._roll_entry_tilt is not None and (
                tilt is None
                or tilt > self._roll_entry_tilt - self.roll_progress_rad):
            # A roll cycle ran and bought less than the progress margin: stop
            # flailing. A trunk that merely *climbed back* out of the gate after
            # the roll succeeded has no cycle in progress, so it still re-tucks.
            self._end_attempt(
                body, "roll cycle bought less than %.2f rad of tilt"
                % self.roll_progress_rad)
            return
        self._enter_phase(self.TUCK, now_s)

    def _stand_target(self, now_s: float,
                      body: Optional[BodyState] = None) -> Dict[str, float]:
        """The standing pose, slewed in and with the lateral splay released.

        Stepping straight to the nominal stance was measured to over-drive the
        legs out of the crouch: the placed 1.4 rad nose-down trial left the
        ground at 0.56 m with no foot loaded, pitched the trunk to 0.81 rad and
        then cost the ladder three retries to undo its own stand-up. Slewing the
        target over :attr:`stand_s` keeps the drive bounded, and the end of the
        slew *is* the nominal stance, so the standing pose is still reached (and
        still only from this phase).

        The measured lateral splay is *released* here rather than dropped, for
        the same reason it was kept through the crouch: with the hips already at
        zero, all three placed-flank trials tipped onto their back within 0.4 s
        of the stand entry while the trunk still carried 0.5–0.7 rad of roll. The
        splay scales with how far the trunk still is from :attr:`success_tilt_rad`
        — full above it, tapering to exactly the nominal stance as the trunk comes
        upright — and only while roll dominates, so the pitch path is untouched.
        """
        stance = nominal_stance_pose()
        if self.phase_started_at is None:
            return stance
        alpha = (now_s - self.phase_started_at) / self.stand_s
        if alpha >= 1.0:
            target = dict(stance)
        else:
            crouch = recovery_pose("crouch")
            alpha = max(0.0, alpha)
            target = {
                name: clamp_position(
                    name, crouch[name] + alpha * (stance[name] - crouch[name]))
                for name in JOINT_NAMES
            }
        self._release_lateral_splay(target, body)
        return target

    def _release_lateral_splay(self, target: Dict[str, float],
                               body: Optional[BodyState]) -> None:
        """Scale the measured splay down with the trunk's remaining roll.

        Full splay at or beyond :attr:`success_tilt_rad` of roll, zero when the
        trunk is upright, and nothing at all unless roll still dominates -- a
        missing attitude or a pitch-dominated pose keeps the target as built.
        """
        if not self.roll_brace_hip_rad or body is None:
            return
        if abs(body.roll_rad) < abs(body.pitch_rad):
            return
        scale = min(1.0, abs(body.roll_rad) / self.success_tilt_rad)
        if scale <= 0.0:
            return
        splay = self.roll_brace_hip_rad * scale
        for name in JOINT_NAMES:
            leg = name.split("_")[0]
            if joint_kind(name) != "hip":
                continue
            if leg in brace_legs(body.roll_rad, body.pitch_rad):
                target[name] = clamp_position(
                    name, splay if leg.endswith("R") else -splay)

    def _support_creation_hip_rad(
        self,
        body: Optional[BodyState],
        measured_support_forces: Optional[Dict[str, float]],
        now_s: float,
    ) -> float:
        """Ramp braced-hip splay in, holding it through transient contacts.

        The roll transfer gate owns release: it advances only after valid foot
        support persists for its dwell. Tapering on a single force sample would
        retract the support-creation target on the same contact slam the gate
        rejects.
        """
        if self.roll_brace_hip_rad == 0.0 or body is None:
            return 0.0
        if abs(body.roll_rad) < abs(body.pitch_rad):
            return 0.0
        if measured_support_forces is None:
            return 0.0
        braced = brace_legs(body.roll_rad, body.pitch_rad)
        support_names = [f"{leg}_hip_contact_0" for leg in braced]
        support_names.extend(f"{leg}_foot_contact_0" for leg in LEG_PREFIXES)
        loads = [measured_support_forces.get(name, float("nan"))
                 for name in support_names]
        if not all(math.isfinite(load) and load >= 0.0 for load in loads):
            return 0.0
        if self.phase_started_at is None or not math.isfinite(now_s):
            return 0.0
        elapsed = max(0.0, now_s - self.phase_started_at)
        scale = min(1.0, elapsed / self.roll_s)
        return self.roll_brace_hip_rad * scale

    def _phase_efforts(
        self,
        positions: Dict[str, float],
        velocities: Dict[str, float],
        body: Optional[BodyState],
        now_s: float,
        measured_support_forces: Optional[Dict[str, float]] = None,
    ) -> Dict[str, float]:
        """Clamped joint-space PD effort toward this phase's waypoint."""
        if self.phase == self.STAND:
            target = self._stand_target(now_s, body)
            stance = StanceController(target=target, gain_scale=self._gain_scale,
                                      damping_scale=self._damping_scale)
            return stance.effort_command(positions, velocities)
        if self.phase == self.CROUCH:
            # The crouch holds the measured splay for the whole phase. Slewing it
            # out over the crouch was tried and measured worse: the trunk drops
            # back out of the gate mid-phase, the ladder re-tucks and the trials
            # end inverted again (fall_ladder_gather_20260928T*). The 0.70-0.76
            # rad pose this leaves is a *stable* stop, not a trap.
            target = (crouch_phase_pose(body.roll_rad, body.pitch_rad,
                                        self.roll_brace_hip_rad)
                      if body is not None else recovery_pose("crouch"))
        elif self.phase == self.ROLL and body is not None:
            support_splay = self._support_creation_hip_rad(
                body, measured_support_forces, now_s)
            target = roll_phase_pose(
                body.roll_rad, body.pitch_rad,
                support_splay,
                self.roll_free_hip_rad)
        else:
            target = recovery_pose("tuck")
        stance = StanceController(target=target, gain_scale=self._gain_scale,
                                  damping_scale=self._damping_scale)
        return stance.effort_command(positions, velocities)

    @staticmethod
    def _roll_starts_hip_supported(
        body: Optional[BodyState],
        measured_support_forces: Optional[Dict[str, float]],
    ) -> bool:
        if body is None or abs(body.roll_rad) < abs(body.pitch_rad):
            return False
        if measured_support_forces is None:
            return False
        braced = brace_legs(body.roll_rad, body.pitch_rad)
        hip_load = sum(measured_support_forces.get(
            f"{leg}_hip_contact_0", 0.0) for leg in braced)
        foot_load = sum(measured_support_forces.get(
            f"{leg}_foot_contact_0", 0.0) for leg in LEG_PREFIXES)
        return hip_load >= 0.5 * GO2_SUPPORT_WEIGHT_N \
            and foot_load < 0.25 * GO2_SUPPORT_WEIGHT_N

    @staticmethod
    def _has_support_transfer(
        body: Optional[BodyState],
        measured_support_forces: Optional[Dict[str, float]],
    ) -> bool:
        if body is None or measured_support_forces is None:
            return False
        braced = brace_legs(body.roll_rad, body.pitch_rad)
        names = [f"{leg}_hip_contact_0" for leg in braced]
        names.extend(f"{leg}_foot_contact_0" for leg in LEG_PREFIXES)
        names.append("trunk_contact_0")
        forces = [measured_support_forces.get(name, float("nan")) for name in names]
        if not all(math.isfinite(force) and force >= 0.0 for force in forces):
            return False
        hip_end = len(braced)
        foot_end = hip_end + len(LEG_PREFIXES)
        hip_load = sum(forces[:hip_end])
        foot_load = sum(forces[hip_end:foot_end])
        trunk_load = forces[foot_end]
        return (
            FALL_RECOVER_SUPPORT_MIN_FOOT_N <= foot_load
            <= FALL_RECOVER_SUPPORT_MAX_FOOT_N
            and hip_load <= FALL_RECOVER_SUPPORT_MAX_HIP_N
            and trunk_load <= FALL_RECOVER_SUPPORT_MAX_TRUNK_N
            and hip_load + foot_load + trunk_load
            <= FALL_RECOVER_SUPPORT_MAX_TOTAL_N
        )


# ----------------------------------------------------------------------
# Top-level locomotion core: one testable update cycle
# ----------------------------------------------------------------------


@dataclass
class ControlCycle:
    """Everything one update cycle produced (command + estimates + safety)."""

    efforts: Dict[str, float]
    position_targets: Dict[str, float]
    contacts: Dict[str, Optional[bool]]
    safety_state: str
    phase: float
    issues: List[str] = field(default_factory=list)


class Go2LocomotionCore:
    """Pure-logic locomotion core for the Go2.

    One call to :meth:`update` advances the trot phase, computes PD efforts
    toward the gait targets, estimates per-leg contacts, runs the safety
    monitor, and — if SAFE_STOP is latched — replaces the command with the
    damped zero-effort output. No ROS types are involved, so every law is
    unit-testable.
    """

    def __init__(self, initial_velocity: Optional[BaseVelocity] = None,
                 gain_scale: float = 1.0, damping_scale: float = 1.0) -> None:
        self.phase = 0.0
        if not math.isfinite(gain_scale) or gain_scale <= 0.0 or gain_scale > 1.0:
            raise ValueError("gain_scale must be within (0, 1]")
        if not math.isfinite(damping_scale) or damping_scale <= 0.0:
            raise ValueError("damping_scale must be positive")
        self.gain_scale = gain_scale
        self.damping_scale = damping_scale
        self.velocity = clamp_base_velocity(initial_velocity or BaseVelocity())
        self.safety = SafetyState()
        self._last_commanded: Dict[str, float] = {j: 0.0 for j in JOINT_NAMES}

    def set_velocity(self, command: BaseVelocity, dt: float) -> BaseVelocity:
        """Rate-limit a new base-velocity command onto the gait."""
        self.velocity = rate_limit_base_velocity(self.velocity, command, dt)
        return self.velocity

    def advance_phase(self, dt: float) -> float:
        """Advance the trot phase by *dt* seconds (wraps at cycle time)."""
        self.phase = (self.phase + dt / TROT_CYCLE_SECONDS) % 1.0
        return self.phase

    def stance_flags(self) -> Dict[str, bool]:
        return {leg: leg_in_stance(leg, self.phase) for leg in LEG_PREFIXES}

    def update(
        self,
        dt: float,
        measured_positions: Dict[str, float],
        measured_velocities: Optional[Dict[str, float]] = None,
        measured_efforts: Optional[Dict[str, float]] = None,
        body: Optional[BodyState] = None,
        velocity_command: Optional[BaseVelocity] = None,
        measured_contact_forces: Optional[Dict[str, float]] = None,
    ) -> ControlCycle:
        """Run one control cycle.

        ``measured_positions`` should contain the joints that were actually
        measured; joints missing from it receive **zero drive** for this
        cycle (never a fabricated effort from an assumed pose).
        """
        issues: List[str] = []
        if velocity_command is not None:
            self.set_velocity(velocity_command, dt)
        self.advance_phase(dt)

        if body is not None:
            issues.extend(self.safety.observe_body(body))

        targets = trot_joint_targets(self.phase, self.velocity)
        stance = StanceController(target=targets, gain_scale=self.gain_scale,
                                  damping_scale=self.damping_scale)
        efforts = stance.effort_command(
            measured_positions, measured_velocities or {}
        )
        # Honest no-data handling: a joint with no position measurement is
        # not driven at all this cycle (StanceController would otherwise
        # treat the missing measurement as "already at target").
        for joint in JOINT_NAMES:
            if joint not in measured_positions:
                efforts[joint] = 0.0
        position_targets = dict(targets)

        if measured_efforts is not None:
            issues.extend(self.safety.observe_efforts(efforts, measured_efforts))

        contacts: Dict[str, Optional[bool]] = {leg: None for leg in LEG_PREFIXES}
        if measured_efforts is not None:
            observations = {}
            for leg in LEG_PREFIXES:
                observations[leg] = LegObservation(
                    measured_effort={
                        f"{leg}_{kind}_joint": measured_efforts.get(
                            f"{leg}_{kind}_joint"
                        )
                        for kind in JOINT_KINDS
                    },
                    commanded_effort={
                        f"{leg}_{kind}_joint": efforts[f"{leg}_{kind}_joint"]
                        for kind in JOINT_KINDS
                    },
                    position_error={},
                )
            contacts = estimate_leg_contacts(observations, self.stance_flags())
        if measured_contact_forces is not None:
            contacts = {
                leg: (force >= 2.0 if force is not None and math.isfinite(force)
                      else None)
                for leg in LEG_PREFIXES
                for force in (measured_contact_forces.get(leg),)
            }

        if not self.safety.gait_permitted():
            efforts = self.safety.safe_stop_efforts()
            position_targets = {j: measured_positions.get(j, 0.0) for j in JOINT_NAMES}

        self._last_commanded = dict(efforts)
        return ControlCycle(
            efforts=efforts,
            position_targets=position_targets,
            contacts=contacts,
            safety_state=self.safety.state,
            phase=self.phase,
            issues=issues,
        )
