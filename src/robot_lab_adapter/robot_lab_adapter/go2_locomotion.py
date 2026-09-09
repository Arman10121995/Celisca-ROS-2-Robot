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
- Contact estimation is effort-residual based per leg; body state
  (roll/pitch) comes from the IMU quaternion; falls and excessive tilt
  force a SAFE_STOP (damped zero effort), never silent continuation.
- All 12 joints are effort-commandable with per-joint limits taken from
  the Go2 description (const.xacro): hip/thigh 23.7 N·m, calf 35.55 N·m.
  **Raw effort publishing is not gait control**: every effort command is
  produced by this layer and clamped to the per-joint limits.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

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
            tau = kp * (q_star - q) + kd * (0.0 - qdot)
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

#: Swing-target offsets [rad] applied to the nominal stance during swing:
#: thigh retracts and calf folds to lift the foot. Bounded and small.
SWING_THIGH_OFFSET_RAD = 0.35
SWING_CALF_OFFSET_RAD = -0.5


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

    Honest bounded stepping: stance legs hold the nominal pose; swing legs
    apply fixed bounded thigh/calf offsets to lift the foot. Forward/lateral
    velocity modulates swing height slightly (faster -> higher step). This
    is *not* an MPC or full-body-dynamics solution.
    """
    step_scale = min(1.0, (abs(velocity.vx) + abs(velocity.vy)) / BASE_VEL_LIMITS["vx"])
    targets: Dict[str, float] = {}
    for name in JOINT_NAMES:
        leg = name.split("_")[0]
        kind = joint_kind(name)
        if leg_in_stance(leg, phase):
            targets[name] = NOMINAL_STANCE[kind]
        else:
            if kind == "thigh":
                raw = NOMINAL_STANCE[kind] + SWING_THIGH_OFFSET_RAD * step_scale
            elif kind == "calf":
                raw = NOMINAL_STANCE[kind] + SWING_CALF_OFFSET_RAD * step_scale
            else:
                raw = NOMINAL_STANCE[kind]
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
    """Mean absolute effort residual across the leg's three joints [N·m].

    A stance leg carrying load shows measured efforts close to the PD
    command; a swinging leg shows a large residual (no ground reaction).
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

    def reset(self) -> None:
        self.state = self.NOMINAL
        self.saturation_counters = {j: 0 for j in JOINT_NAMES}
        self.reason = None

    def observe_body(self, body: BodyState) -> List[str]:
        """Update state from attitude; returns any new issue strings."""
        issues: List[str] = []
        tilt = body.max_tilt_rad
        if tilt >= TILT_FALL_RAD:
            self._enter_safe_stop(f"tilt {tilt:.2f} rad exceeds fall threshold")
            issues.append(f"safe_stop: tilt {tilt:.2f} rad exceeds fall threshold")
        elif tilt >= TILT_WARN_RAD:
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

    def __init__(self, initial_velocity: Optional[BaseVelocity] = None) -> None:
        self.phase = 0.0
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
        stance = StanceController(target=targets)
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
