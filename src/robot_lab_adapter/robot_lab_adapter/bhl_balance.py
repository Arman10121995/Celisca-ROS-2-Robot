"""R5.3: Berkeley Humanoid Lite closed-loop balance core.

Pure-logic balance layer for the Berkeley Humanoid Lite (BHL) biped. The ROS
node (``humanoid_standing_controller.py``) is a thin wrapper around this
module, so every control law here is unit-testable without a live ROS graph.

Honest scope (R5.3 acceptance bar):

- This is a **closed-loop** balance controller, not an open-loop pose hold.
  A fixed standing pose is deliberately NOT balance: when the IMU body state
  reports a non-zero roll or pitch, the controller changes its ankle / hip /
  arm commands to produce a righting moment. ``update(body=None)`` returns
  the nominal pose only as a safe open-loop fallback; with a measured body
  state the output differs, which is the balance signal.
- **Ankle strategy**: the BHL ankles are a *parallel* mechanism (FK at the
  rest pose: both ankle-pitch axes on +Y, both ankle-roll axes on +X), so
  body roll commands a SAME-SIGN ankle-roll correction on both legs and
  body pitch a same-sign ankle-pitch lean; a differential roll command
  would cancel through the pelvis and produce no net righting moment.
  Hip roll adds a same-sign lateral CoM shift. The ankle gains are sized
  so the combined restoring stiffness (2 * Kp * K = 168 N.m/rad) exceeds
  the gravity topple stiffness (m*g*h ~= 108 N.m/rad) with a 1.5x margin
  - a stable equilibrium, not a slowly diverging one.
- **Arm reaction**: arms swing (shoulder-pitch) out of phase with roll to
  add a corrective inertial moment.
- The balance target is converted to an estimated joint effort through a
  joint-space PD law (stance hold around the balance-reactive target),
  clamped to the *measured* per-joint effort limit (20 N.m from the BHL
  URDF ``<limit>``). Raw effort publication is not balance control; every
  effort is produced by this layer and clamped.
- **Safety monitor**: a latched ``SAFE_STOP`` is entered on excessive tilt
  (>= 0.70 rad, ~40 deg) or sustained effort saturation (>=98% limit for 50
  cycles); recovery requires an explicit ``reset()``.
- **No measurement -> no drive**: a joint missing from ``positions`` receives
  zero effort this cycle (never a fabricated command from an assumed pose).
- **Biped stance**: both legs are in contact during single-support balance.
  A ``stance_duration`` constant is declared for the balance hold; stepping
  / walking gait generation is *not* part of this layer (flat-ground
  stepping and stop remain future work).

All joint constants (names, limits, nominal pose) are extracted from the
vendored static URDF and are asserted against it by the qualification tests.
"""

from __future__ import annotations

import math
import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

# ----------------------------------------------------------------------
# Robot constants - honest to berkeley_humanoid_lite.urdf
# ----------------------------------------------------------------------
# The BHL URDF is a vendored static Onshape-to-URDF export. Every actuated
# revolute joint has effort limit 20.0 N.m and velocity limit 15.0 rad/s.
# The 22 actuated joints are 12 leg + 10 arm joints; 3 fixed joints
# (imu, arm_left_hand_l, arm_right_hand_l) are excluded from control.

#: Canonical 22-joint ordering (matches bhl_controllers.yaml command list).
BHL_JOINT_NAMES: Tuple[str, ...] = (
    # 12 leg joints (left then right)
    "leg_left_hip_roll_joint",
    "leg_left_hip_yaw_joint",
    "leg_left_hip_pitch_joint",
    "leg_left_knee_pitch_joint",
    "leg_left_ankle_pitch_joint",
    "leg_left_ankle_roll_joint",
    "leg_right_hip_roll_joint",
    "leg_right_hip_yaw_joint",
    "leg_right_hip_pitch_joint",
    "leg_right_knee_pitch_joint",
    "leg_right_ankle_pitch_joint",
    "leg_right_ankle_roll_joint",
    # 10 arm joints (left then right)
    "arm_left_shoulder_pitch_joint",
    "arm_left_shoulder_roll_joint",
    "arm_left_shoulder_yaw_joint",
    "arm_left_elbow_roll_joint",
    "arm_left_elbow_pitch_joint",
    "arm_right_shoulder_pitch_joint",
    "arm_right_shoulder_roll_joint",
    "arm_right_shoulder_yaw_joint",
    "arm_right_elbow_roll_joint",
    "arm_right_elbow_pitch_joint",
)

#: Leg joint names (12).
BHL_LEG_JOINTS: Tuple[str, ...] = BHL_JOINT_NAMES[:12]
#: Arm joint names (10).
BHL_ARM_JOINTS: Tuple[str, ...] = BHL_JOINT_NAMES[12:]

#: Sides handled for balance (biped: both legs contact).
BHL_LEGS: Tuple[str, ...] = ("left", "right")

#: Per-joint effort limit [N.m] for all BHL actuated joints (URDF <limit>).
EFFORT_LIMIT = 20.0
#: Per-joint velocity limit [rad/s] (URDF <limit>).
VELOCITY_LIMIT = 15.0

#: Per-joint position limits [rad] (lower, upper) - read from the URDF.
POSITION_LIMITS: Dict[str, Tuple[float, float]] = {
    "leg_left_hip_roll_joint": (-0.174533, 1.570800),
    "leg_left_hip_yaw_joint": (-0.981748, 0.589049),
    "leg_left_hip_pitch_joint": (-1.898050, 0.981748),
    "leg_left_knee_pitch_joint": (0.0, 2.443460),
    "leg_left_ankle_pitch_joint": (-0.785398, 0.785398),
    "leg_left_ankle_roll_joint": (-0.261799, 0.261799),
    "leg_right_hip_roll_joint": (-1.570800, 0.174533),
    "leg_right_hip_yaw_joint": (-0.589049, 0.981748),
    "leg_right_hip_pitch_joint": (-1.898050, 0.981748),
    "leg_right_knee_pitch_joint": (0.0, 2.443460),
    "leg_right_ankle_pitch_joint": (-0.785398, 0.785398),
    "leg_right_ankle_roll_joint": (-0.261799, 0.261799),
    "arm_left_shoulder_pitch_joint": (-1.570800, 0.785398),
    "arm_left_shoulder_roll_joint": (-0.261799, 1.309000),
    "arm_left_shoulder_yaw_joint": (-0.785398, 0.785398),
    "arm_left_elbow_roll_joint": (-0.785398, 0.785398),
    "arm_left_elbow_pitch_joint": (0.0, 1.570800),
    "arm_right_shoulder_pitch_joint": (-0.785398, 1.570800),
    "arm_right_shoulder_roll_joint": (-1.309000, 0.261799),
    "arm_right_shoulder_yaw_joint": (-0.785398, 0.785398),
    "arm_right_elbow_roll_joint": (-0.785398, 0.785398),
    "arm_right_elbow_pitch_joint": (-1.570800, 0.0),
}

#: Nominal standing pose [rad] - inside every joint's position limits.
#: Knees are bent (~0.8 rad) so the legs carry a restoring moment; ankle
#: pitch has a small toe-up bias; arms hang at the sides.
# R5.3 (2026-09-17, live-validated): the nominal stance is the URDF rest pose
# (every actuated joint at 0). The earlier draft stance (knees 0.8, ankle
# pitch 0.1) actively dragged the biped out of its balanced rest pose at
# spawn: the live tilt trace showed the robot standing at tilt 0.000 until the
# PD began pulling the knees toward 0.8, after which the knees buckled (20
# N.m is far too little to hold that crouch) and the biped fell through the
# 0.70 rad fall threshold in ~1-2 s. At the all-zero rest pose the legs are
# straight, gravity passes through the joint axes, and the PD hold is
# statically stable - balance corrections (ankle strategy, arm reaction)
# then act on perturbations instead of having to fight the stance itself.
NOMINAL_STANDING_POSE: Dict[str, float] = {
    "leg_left_hip_roll_joint": 0.0,
    "leg_left_hip_yaw_joint": 0.0,
    "leg_left_hip_pitch_joint": 0.0,
    "leg_left_knee_pitch_joint": 0.0,
    "leg_left_ankle_pitch_joint": 0.0,
    "leg_left_ankle_roll_joint": 0.0,
    "leg_right_hip_roll_joint": 0.0,
    "leg_right_hip_yaw_joint": 0.0,
    "leg_right_hip_pitch_joint": 0.0,
    "leg_right_knee_pitch_joint": 0.0,
    "leg_right_ankle_pitch_joint": 0.0,
    "leg_right_ankle_roll_joint": 0.0,
    "arm_left_shoulder_pitch_joint": 0.0,
    "arm_left_shoulder_roll_joint": 0.0,
    "arm_left_shoulder_yaw_joint": 0.0,
    "arm_left_elbow_roll_joint": 0.0,
    "arm_left_elbow_pitch_joint": 0.0,
    "arm_right_shoulder_pitch_joint": 0.0,
    "arm_right_shoulder_roll_joint": 0.0,
    "arm_right_shoulder_yaw_joint": 0.0,
    "arm_right_elbow_roll_joint": 0.0,
    "arm_right_elbow_pitch_joint": 0.0,
}

# ----------------------------------------------------------------------
# Balance gains (ankle strategy + arm reaction)
# ----------------------------------------------------------------------
# Each maps a unit of body tilt (rad) into a unit of joint-angle target
# correction (rad). All are bounded so balance-reactive targets stay inside
# the BHL position limits for tilts up to the warn threshold (~0.35 rad).

#: Joint-space PD gains for the stance hold estimate. These are *estimated*
#: gains used only for effort estimation / safety monitoring; the live BHL
#: description does not ship a robot_control.yaml, so these are not claimed
#: to match any on-disk file. Honesty: do not market as description gains.
STANCE_PD_LEGS: Tuple[float, float] = (120.0, 4.0)   # (Kp, Kd) N.m/rad
STANCE_PD_ARMS: Tuple[float, float] = (60.0, 2.0)    # (Kp, Kd) N.m/rad

#: Ankle-roll modulation per rad of body roll (both legs, SAME sign - the
#: BHL ankle-roll axes are parallel (+X/+X), so differential commands
#: cancel and produce no net roll moment; see balance_targets).
#: 0.7 -> combined restoring stiffness 2*120*0.7 = 168 N.m/rad, ~2.4x the
#: gravity topple stiffness (TOPPLE_STIFFNESS_NM_PER_RAD).
ANKLE_BALANCE_K_ROLL = 0.7
#: Hip-roll modulation per rad of body roll (same-sign lateral CoM shift).
HIP_BALANCE_K_ROLL = 0.08
#: Ankle-pitch modulation per rad of body pitch (both legs, same sign).
ANKLE_BALANCE_K_PITCH = 0.7
#: Shoulder-pitch modulation per rad of body roll (arm reaction).
ARM_BALANCE_K = 0.25
#: Knee flex modulation per rad of combined tilt (softens stance under load).
KNEE_BALANCE_K = 0.30

# Gravity topple stiffness about the foot edge, from the vendored URDF:
# total mass 16.3312 kg, whole-body CoM at the rest pose z=0.4823 m in the
# base frame, soles at z=+0.0400 m (verified against pybullet collision AABBs
# and `gz sdf -p`; see docs/status/evidence/r53-bhl-ankle-fix-2026-09-17/
# sole_height_probe.txt), so the CoM height above the sole is 0.4423 m.
# The earlier estimate (h=0.675 m, the base inertial origin) measured the CoM
# above the base ORIGIN, not above the sole, and overestimated the topple
# stiffness by ~1.5x. The ankle strategy must exceed this to make the standing
# equilibrium stable; pinned by TestAnkleStrategy regression tests.
TOPPLE_STIFFNESS_NM_PER_RAD = 16.3312 * 9.81 * 0.4423  # ~= 70.9 N.m/rad
#: Required safety margin of the ankle restoring stiffness over gravity.
ANKLE_STIFFNESS_MARGIN = 1.5

# ----------------------------------------------------------------------
# Safety thresholds (predeclared)
# ----------------------------------------------------------------------
TILT_WARN_RAD = 0.35    # ~20 deg: balance controller is struggling
TILT_FALL_RAD = 0.70    # ~40 deg: force SAFE_STOP (latched)
EFFORT_SATURATION_FRACTION = 0.98
EFFORT_SATURATION_CYCLES = 50  # sustained saturation at the control rate

#: Declared biped stance hold duration [s]. Both legs are in contact during
#: balance; stepping/walking gait generation is outside this layer.
STANCE_DURATION_SECONDS = 1.0
#: Balance control rate [Hz] (matches the standing-controller timer).
#: R5.3 (2026-09-18, plant-swept in MuJoCo, see docs/status/evidence/
#: r53-bhl-ankle-fix-2026-09-17/plant_rate_probe.txt): the SAME PD stance
#: hold (Kp=120, Kd=4, +/-20 N.m) is stable at >= 125 Hz and DIVERGES at
#: 50 Hz - the zero-order-hold torque staleness is discretely unstable for
#: the low-inertia ankle/foot links, which is the root cause of the live
#: ~5.7 s pivot topple (the biped stands still while the instability grows
#: from numerical noise, then breaks away in ~0.6 s). 250 Hz gives margin
#: over the measured 125 Hz stability edge; ign-gazebo's 100 Hz physics
#: still sees a torque at most one physics step stale at this rate.
#: The balance law itself was re-validated at 250 Hz with the SAME sign it
#: already had (+K on same-sign ankles): perturbed-stance max tilt 0.008 rad
#: with the reaction vs 0.129 rad with a flipped sign - the sign is right,
#: only the rate was wrong.
BALANCE_RATE_HZ = 250.0
#: Effective control period [s].
STANCE_DT = 1.0 / BALANCE_RATE_HZ
#: Reserved for the future base-velocity (stepping) interface. Left as None
#: on purpose: R5.3 balance has no base-velocity command channel; the standing
#: controller holds a fixed stance. A non-None value would falsely claim a
#: bounded stepping interface that is not implemented here.
BASE_VEL_LIMITS_PLACEHOLDER: Optional[Dict[str, float]] = None


def bhl_joint_kind(joint_name: str) -> str:
    """Return the functional kind of a BHL joint name.

    Returns one of: ``ankle_roll``, ``ankle_pitch``, ``hip_roll``,
    ``hip_yaw``, ``hip_pitch``, ``knee_pitch``, ``shoulder_pitch``,
    ``shoulder_roll``, ``shoulder_yaw``, ``elbow_roll``, ``elbow_pitch``.
    Raises ``ValueError`` if the name is not a recognised BHL actuated joint.
    """
    if joint_name in POSITION_LIMITS:
        if joint_name.endswith("_ankle_roll_joint"):
            return "ankle_roll"
        if joint_name.endswith("_ankle_pitch_joint"):
            return "ankle_pitch"
        if joint_name.endswith("_hip_roll_joint"):
            return "hip_roll"
        if joint_name.endswith("_hip_yaw_joint"):
            return "hip_yaw"
        if joint_name.endswith("_hip_pitch_joint"):
            return "hip_pitch"
        if joint_name.endswith("_knee_pitch_joint"):
            return "knee_pitch"
        if joint_name.endswith("_shoulder_pitch_joint"):
            return "shoulder_pitch"
        if joint_name.endswith("_shoulder_roll_joint"):
            return "shoulder_roll"
        if joint_name.endswith("_shoulder_yaw_joint"):
            return "shoulder_yaw"
        if joint_name.endswith("_elbow_roll_joint"):
            return "elbow_roll"
        if joint_name.endswith("_elbow_pitch_joint"):
            return "elbow_pitch"
    raise ValueError(f"not a BHL joint: {joint_name!r}")


def clamp_effort(joint_name: str, effort: float) -> float:
    """Clamp an effort command to the BHL joint effort limit (20 N.m)."""
    return max(-EFFORT_LIMIT, min(EFFORT_LIMIT, effort))


def clamp_position(joint_name: str, position: float) -> float:
    """Clamp a joint-space target to the joint's measured position range."""
    lo, hi = POSITION_LIMITS[joint_name]
    return max(lo, min(hi, position))


def nominal_standing_pose() -> Dict[str, float]:
    """The nominal standing pose for all 22 joints (clamped to limits)."""
    return {name: clamp_position(name, value) for name, value in NOMINAL_STANDING_POSE.items()}


# ----------------------------------------------------------------------
# Body state from IMU (roll/pitch) - mirrors the R5.2 BodyState contract
# ----------------------------------------------------------------------
@dataclass(frozen=True)
class BodyState:
    """Attitude snapshot used by the balance and safety layers.

    ``body_height_m`` is None if unavailable (honest missing-data handling).
    """

    roll_rad: float
    pitch_rad: float
    body_height_m: Optional[float] = None

    @classmethod
    def from_quaternion(cls, x: float, y: float, z: float, w: float) -> "BodyState":
        """Roll/pitch from an IMU quaternion (ZYX/Tait-Bryan convention)."""
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
        return cls(roll_rad=roll, pitch_rad=pitch)

    @property
    def max_tilt_rad(self) -> float:
        return max(abs(self.roll_rad), abs(self.pitch_rad))


# ----------------------------------------------------------------------
# Latched safety monitor - mirrors R5.2 SafetyState
# ----------------------------------------------------------------------
class SafetyState:
    """Latched safety monitor: NOMINAL -> WARN -> SAFE_STOP.

    SAFE_STOP is entered on excessive tilt or sustained effort saturation
    and is *latched*: recovery requires an explicit ``reset()`` by the
    operator, never automatic re-enablement mid-fall.
    """

    NOMINAL = "NOMINAL"
    WARN = "WARN"
    SAFE_STOP = "SAFE_STOP"

    def __init__(self) -> None:
        self.state: str = self.NOMINAL
        self.reason: Optional[str] = None
        self._saturation_counters: Dict[str, int] = {j: 0 for j in BHL_JOINT_NAMES}

    def reset(self) -> None:
        self.state = self.NOMINAL
        self.reason = None
        self._saturation_counters = {j: 0 for j in BHL_JOINT_NAMES}

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
        for joint in BHL_JOINT_NAMES:
            cmd = commanded.get(joint)
            meas = measured.get(joint)
            if cmd is None or meas is None:
                self._saturation_counters[joint] = 0
                continue
            saturated = (
                abs(cmd) >= EFFORT_SATURATION_FRACTION * EFFORT_LIMIT
                and abs(meas) >= EFFORT_SATURATION_FRACTION * EFFORT_LIMIT
            )
            if saturated:
                self._saturation_counters[joint] += 1
            else:
                self._saturation_counters[joint] = 0
            if self._saturation_counters[joint] >= EFFORT_SATURATION_CYCLES:
                self._enter_safe_stop(f"sustained effort saturation on {joint}")
                issues.append(f"safe_stop: sustained effort saturation on {joint}")
        return issues

    def _enter_safe_stop(self, reason: str) -> None:
        if self.state != self.SAFE_STOP:
            self.state = self.SAFE_STOP
            self.reason = reason

    def balance_permitted(self) -> bool:
        """True if balance commands may be issued (not latched in SAFE_STOP)."""
        return self.state != self.SAFE_STOP

    def safe_stop_positions(self) -> Dict[str, float]:
        """Safe hold pose for SAFE_STOP: the nominal pose (inside limits).

        Position controllers cannot command zero torque, so holding the nominal
        pose is the safest stop. (A truly passive stop would require effort
                        the BHL standing controller does not expose.)
        """
        return nominal_standing_pose()


# ----------------------------------------------------------------------
# Closed-loop balance: ankle strategy + arm reaction + PD stance hold
# ----------------------------------------------------------------------
def balance_targets(
    nominal: Dict[str, float],
    body: Optional[BodyState],
) -> Dict[str, float]:
    """Compute balance-reactive joint-position targets.

    This is the core balance law. When ``body`` tilts, ankle / hip / arm
    targets shift to produce a righting moment. When ``body`` is None or
    level, targets equal the nominal pose. Therefore a fixed pose is NOT
    balance: tilting the body strictly changes the output.
    """
    targets = dict(nominal)
    if body is None:
        return {name: clamp_position(name, value) for name, value in targets.items()}

    roll = body.roll_rad
    pitch = body.pitch_rad
    tilt_mag = body.max_tilt_rad

    # R5.3 (2026-09-17, live-validated + FK-checked): the BHL ankles are a
    # PARALLEL mechanism - forward kinematics at the rest pose puts BOTH
    # ankle-pitch axes on +Y and BOTH ankle-roll axes on +X. A differential
    # ankle-roll command (the earlier draft) therefore produces *zero net
    # roll moment*: the two internal torques cancel through the pelvis and
    # the feet are twisted in opposite directions without righting the body.
    # Both ankle pairs must be commanded same-sign.
    #
    # Gain justification: gravity topple stiffness about the foot edge is
    # m*g*h ~= 16.3312 kg * 9.81 * 0.4423 m ~= 70.9 N.m/rad (total mass and
    # whole-body CoM z=0.4823 m from the URDF inertials; sole at z=+0.0400 m,
    # see TOPPLE_STIFFNESS_NM_PER_RAD and the sole_height_probe evidence).
    # With joint Kp = 120
    # N.m/rad on each ankle and both legs contributing, K = 0.7 gives a
    # combined restoring stiffness of 2 * 120 * 0.7 = 168 N.m/rad - a ~2.4x
    # margin over gravity, so small perturbations DECAY instead of growing
    # (the earlier K = 0.12/0.15 gave ~29 N.m/rad: unstable by design, which
    # is exactly the observed divergence). Per-ankle saturation then occurs
    # at tilt 20 / (120 * 0.7) ~= 0.24 rad - inside the warn band, where the
    # safety monitor takes over. TOPPLE_STIFFNESS_NM_PER_RAD below pins the
    # estimate for a regression test.
    targets["leg_left_ankle_roll_joint"] += ANKLE_BALANCE_K_ROLL * roll
    targets["leg_right_ankle_roll_joint"] += ANKLE_BALANCE_K_ROLL * roll
    # Hip roll assists with a lateral CoM shift (same-sign pelvis lean).
    targets["leg_left_hip_roll_joint"] += HIP_BALANCE_K_ROLL * roll
    targets["leg_right_hip_roll_joint"] += HIP_BALANCE_K_ROLL * roll
    # Pitch -> ankle pitch lean (same sign on both feet, parallel axes).
    targets["leg_left_ankle_pitch_joint"] += ANKLE_BALANCE_K_PITCH * pitch
    targets["leg_right_ankle_pitch_joint"] += ANKLE_BALANCE_K_PITCH * pitch
    # Arm reaction: arms swing against the roll direction.
    targets["arm_left_shoulder_pitch_joint"] += ARM_BALANCE_K * roll
    targets["arm_right_shoulder_pitch_joint"] -= ARM_BALANCE_K * roll
    # Soften knee flexion as tilt grows (reduces stance stiffness under load).
    knee_shift = KNEE_BALANCE_K * tilt_mag
    targets["leg_left_knee_pitch_joint"] -= knee_shift
    targets["leg_right_knee_pitch_joint"] -= knee_shift

    return {name: clamp_position(name, value) for name, value in targets.items()}


def pd_effort_command(
    targets: Dict[str, float],
    positions: Dict[str, float],
    velocities: Optional[Dict[str, float]] = None,
    leg_gains: Optional[Tuple[float, float]] = None,
    arm_gains: Optional[Tuple[float, float]] = None,
) -> Dict[str, float]:
    """Estimate joint-space PD efforts toward *targets* from measured state.

    Mirrors the R5.2 stance PD law: tau = Kp*(q* - q) + Kd*(0 - qdot), clamped
    per joint. A joint missing from ``positions`` receives zero effort (no
    measurement -> no drive).

    ``leg_gains`` / ``arm_gains`` override the default ``(Kp, Kd)`` pairs so a
    caller that drives the joints through an effort interface (e.g. the policy
    node) can select its own gains without changing the balance law.
    """
    velocities = velocities or {}
    leg = STANCE_PD_LEGS if leg_gains is None else leg_gains
    arm = STANCE_PD_ARMS if arm_gains is None else arm_gains
    command: Dict[str, float] = {}
    for name in BHL_JOINT_NAMES:
        if name not in positions:
            command[name] = 0.0
            continue
        kp, kd = leg if name in BHL_LEG_JOINTS else arm
        tau = kp * (targets[name] - positions[name]) + kd * (0.0 - velocities.get(name, 0.0))
        command[name] = clamp_effort(name, tau)
    return command


@dataclass
class BhlControlCycle:
    """Everything one balance update produced (command + estimates + safety)."""

    position_targets: Dict[str, float]
    efforts: Dict[str, float]
    contacts: Dict[str, Optional[bool]]
    safety_state: str
    issues: List[str] = field(default_factory=list)


class BhlBalanceController:
    """Pure-logic closed-loop balance controller for the BHL biped.

    One call to :meth:`update` computes balance-reactive position targets
    from the IMU body state, estimates per-joint PD efforts for safety
    monitoring, runs the latched safety monitor, and -- if SAFE_STOP is
    latched -- replaces the command with the safe nominal pose. No ROS
    types are involved, so every control law is unit-testable.
    """

    def __init__(self) -> None:
        self.nominal = nominal_standing_pose()
        self.safety = SafetyState()
        self._last_commanded: Dict[str, float] = {j: 0.0 for j in BHL_JOINT_NAMES}

    def reset(self) -> None:
        """Reset the latched safety monitor (operator-initiated recovery)."""
        self.safety.reset()

    def stance_targets(self, body: Optional[BodyState]) -> Dict[str, float]:
        """Balance-reactive stance targets (ankle strategy + arm reaction)."""
        return balance_targets(self.nominal, body)

    def update(
        self,
        dt: float,
        measured_positions: Dict[str, float],
        measured_velocities: Optional[Dict[str, float]] = None,
        measured_efforts: Optional[Dict[str, float]] = None,
        body: Optional[BodyState] = None,
    ) -> BhlControlCycle:
        """Run one balance cycle.

        ``measured_positions`` should contain the joints that were actually
        measured; joints missing from it receive **zero drive** (never a
        fabricated command from an assumed pose).
        """
        issues: List[str] = []

        if body is not None:
            issues.extend(self.safety.observe_body(body))

        targets = self.stance_targets(body)
        efforts = pd_effort_command(targets, measured_positions, measured_velocities)

        # Honest no-data handling: a joint with no position measurement is
        # not driven at all this cycle.
        for joint in BHL_JOINT_NAMES:
            if joint not in measured_positions:
                efforts[joint] = 0.0

        if measured_efforts is not None:
            issues.extend(self.safety.observe_efforts(efforts, measured_efforts))

        # Biped balance: both legs in contact; no per-leg contact estimate is
        # available from position commands alone, so contacts are None.
        contacts: Dict[str, Optional[bool]] = {leg: None for leg in BHL_LEGS}

        if not self.safety.balance_permitted():
            targets = self.safety.safe_stop_positions()
            efforts = {j: 0.0 for j in BHL_JOINT_NAMES}
            if not any("safe_stop" in i for i in issues):
                issues.append(f"safe_stop: {self.safety.reason}")

        self._last_commanded = dict(efforts)
        return BhlControlCycle(
            position_targets=targets,
            efforts=efforts,
            contacts=contacts,
            safety_state=self.safety.state,
            issues=issues,
        )


# ----------------------------------------------------------------------
# URDF static validation (used by the qualification tests)
# ----------------------------------------------------------------------
_BHL_URDF_RELATIVE = os.path.join(
    "berkeley_humanoid_lite", "urdf", "berkeley_humanoid_lite.urdf")


def _urdf_path() -> str:
    """Resolve the BHL URDF, whether running from source or an install.

    A fixed number of parent hops from this file only works for one layout
    and silently produced a path outside the workspace; the package share
    directory is authoritative when the workspace is sourced, and otherwise
    the source tree is searched upward for robot_lab_robots.
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        candidate = os.path.join(
            get_package_share_directory("robot_lab_robots"),
            _BHL_URDF_RELATIVE)
        if os.path.isfile(candidate):
            return candidate
    except Exception:
        pass

    directory = os.path.dirname(os.path.abspath(__file__))
    while True:
        candidate = os.path.join(
            directory, "robot_lab_robots", _BHL_URDF_RELATIVE)
        if os.path.isfile(candidate):
            return candidate
        parent = os.path.dirname(directory)
        if parent == directory:
            # Return the conventional source-tree location so the failure
            # names a path a reader can act on.
            return os.path.join(
                os.path.dirname(os.path.abspath(__file__)),
                "..", "..", "robot_lab_robots", _BHL_URDF_RELATIVE)
        directory = parent


def bhl_urdf_path() -> str:
    """Public accessor for the resolved BHL URDF path (see :func:`_urdf_path`).

    Exposed so tests and tools can cross-check the description files (for
    example against the sibling MJCF) without importing a private name.
    """
    return _urdf_path()


#: Joint friction [N.m] every actuated BHL joint must declare in a *standard*
#: ``<dynamics friction="...">`` URDF tag, matching the robot's own MJCF
#: (mjcf/berkeley_humanoid_lite.xml ``frictionloss="0.1"``). The vendored
#: export carried this in a non-standard ``<joint_properties>`` tag that
#: urdfdom, sdformat and pybullet all silently drop, so the simulated biped
#: ran with ZERO joint friction; the MuJoCo A/B probe
#: (docs/status/evidence/r53-bhl-ankle-fix-2026-09-17/) shows the committed
#: balance law holds tilt at 0.0028 rad with this friction and falls at 0.5 s
#: without it. Pinned by TestJointFrictionDeclaration.
BHL_JOINT_FRICTION_NM = 0.1


def parse_bhl_joint_dynamics() -> Dict[str, Dict[str, float]]:
    """Parse the standard ``<dynamics>`` block of every revolute BHL joint.

    Returns ``{joint_name: {"friction": float, "damping": float}}`` reading
    only the standard tags that urdfdom / sdformat / pybullet actually load.
    A joint whose friction is declared in a non-standard tag therefore shows
    up as absent here, which is what makes the declaration regression-testable.
    """
    tree = ET.parse(_urdf_path())
    root = tree.getroot()
    out: Dict[str, Dict[str, float]] = {}
    for j in root.iter("joint"):
        if j.get("type") != "revolute":
            continue
        dyn = j.find("dynamics")
        if dyn is None:
            continue
        out[j.get("name")] = {
            "friction": float(dyn.get("friction", 0.0)),
            "damping": float(dyn.get("damping", 0.0)),
        }
    return out


def parse_bhl_joints() -> Dict[str, Dict[str, Optional[float]]]:
    """Parse actuated revolute joints from the BHL URDF.

    Returns a mapping ``{joint_name: {lower, upper, effort, velocity}}``
    for every revolute joint. Joints with no position limits return ``None``
    bounds. Used to verify the module constants are honest to the on-disk
    description.
    """
    tree = ET.parse(_urdf_path())
    root = tree.getroot()
    joints: Dict[str, Dict[str, Optional[float]]] = {}
    for j in root.iter("joint"):
        if j.get("type") != "revolute":
            continue
        name = j.get("name")
        limit = j.find("limit")
        if limit is None:
            continue
        joints[name] = {
            "lower": float(limit.get("lower")) if limit.get("lower") is not None else None,
            "upper": float(limit.get("upper")) if limit.get("upper") is not None else None,
            "effort": float(limit.get("effort")),
            "velocity": float(limit.get("velocity")),
        }
    return joints






