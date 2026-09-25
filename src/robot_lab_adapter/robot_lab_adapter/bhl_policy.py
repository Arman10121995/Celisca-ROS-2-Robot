"""R5.3: Berkeley Humanoid Lite ONNX locomotion-policy adapter (pure logic).

Pure-logic layer that runs the vendored upstream Berkeley Humanoid Lite ONNX
velocity policies and turns them into joint-position targets. The ROS node
(``humanoid_policy_controller.py``) is a thin wrapper around this module, so
every conversion here is unit-testable without a live ROS graph and without
onnxruntime installed (the inference session is dependency-injected).

Conventions are verified against the vendored upstream sources at pinned
commits (see ``docs/status/evidence/r53-bhl-policy-probe-2026-09-15``):

- Observation layout (feed-forward, ``history_length == 0``): velocity
  command (3), base angular velocity (3), projected gravity (3), joint
  positions minus default, joint velocities, previous action - the last
  three blocks over the policy's ``action_indices`` joints. This matches
  both vendored checkpoints: ``policy_humanoid`` (22 joints, 75 dims) and
  ``policy_humanoid_legs`` (12 leg joints, 45 dims). The config's declared
  ``num_observations`` is enforced against 9 + 3 * len(action_indices).
- Command ranges from the training ``CommandsCfg``: lin_vel_x [-1, 1],
  lin_vel_y [-0.5, 0.5], ang_vel_z [-1.5, 1.5]. Commands are clamped to
  these training ranges; they are not claims of verified robot limits.
- Actions: joint-position offsets scaled by ``action_scale`` (0.25) around
  the config default pose, exactly as the training ``JointPositionAction``
  (``use_default_offset``) and the headless probe apply them. Joints the
  policy does not act on (e.g. arms under ``policy_humanoid_legs``) are held
  at the config default pose.
- Joint ordering: the config ``joints`` list (arms first, then legs), which
  is the ``HUMANOID_LITE_JOINTS`` order the checkpoints were trained on.
  Measured ``joint_states`` are mapped by name into this order; nothing is
  assumed from index position.

Honest scope:

- This layer publishes joint-position targets. Joint PD gains and effort
  clamping happen in the backend controller; the policy config's per-joint
  effort limits (4 N.m arms / 6 N.m legs) are the training-time clamps and
  are exposed here so a backend can configure itself faithfully - the URDF
  20 N.m limit is a hardware bound, not the policy's operating envelope.
- A missing joint measurement means the 75-dim observation cannot be built;
  this layer skips inference that cycle, holds its previous targets and
  reports an issue. It never fabricates a measurement. Joint velocities are
  the caller's responsibility (the node differentiates consecutive
  ``joint_states``); when unavailable, zeros are used and the degraded
  tracking that follows is real, not hidden.
- Body tilt at or beyond the fall threshold (0.70 rad, shared with the
  balance core) latches SAFE_STOP; recovery requires an explicit ``reset()``.
- The opt-in boost-only yaw servo (:class:`YawRateBoost`) lives here too: it
  is the node's ``yaw_servo_gain > 0`` path, is off by default, and its class
  docstring carries the measured defect (a held pure-turn command collapsing
  the policy's stepping limit cycle) and its honest scope.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import yaml

from robot_lab_adapter.bhl_balance import POSITION_LIMITS, TILT_FALL_RAD

#: Base-velocity command width in the observation (vx, vy, wz).
COMMAND_WIDTH = 3

#: Training command ranges (vx, vy, wz); commands are clamped to these.
COMMAND_LIMITS: Tuple[float, float, float] = (1.0, 0.5, 1.5)

#: Training command limit for the yaw rate [rad/s] (``ang_vel_z`` range).
YAW_COMMAND_LIMIT = COMMAND_LIMITS[2]

#: Policy decision rate [Hz] (upstream policy_dt = 0.04 s).
POLICY_RATE_HZ = 25.0
#: Policy decision period [s].
POLICY_DT = 1.0 / POLICY_RATE_HZ

_UPSTREAM_RELATIVE = os.path.join("_upstream", "Berkeley-Humanoid-Lite")



def _usable_upstream(candidate: Path) -> bool:
    """A candidate counts only if the pinned policy configs are really there.

    A stale install-share copy of ``_upstream`` (built before the configs
    were vendored) must not silently shadow the source tree.
    """
    return ((candidate / "configs" / "policy_humanoid.yaml").is_file()
            and (candidate / "configs" / "policy_humanoid_legs.yaml").is_file())


def upstream_dir() -> Path:
    """Resolve the vendored upstream tree: source tree first, install share second.

    The source-tree copy is the pinned reference (submodule commits recorded
    in the R5.3 evidence); an install-share copy is a build artifact that can
    be stale (one was observed missing the ``policy_humanoid_legs`` config),
    so it is only used when no source tree is found. Both candidates are
    required to actually contain the policy configs.
    """
    directory = Path(__file__).resolve().parent
    while True:
        candidate = directory / "robot_lab_robots" / _UPSTREAM_RELATIVE
        if _usable_upstream(candidate):
            return candidate
        parent = directory.parent
        if parent == directory:
            break
        directory = parent

    try:
        from ament_index_python.packages import get_package_share_directory
        candidate = Path(get_package_share_directory("robot_lab_robots")) / _UPSTREAM_RELATIVE
        if _usable_upstream(candidate):
            return candidate
    except Exception:
        pass

    return Path(__file__).resolve().parents[2] / "robot_lab_robots" / _UPSTREAM_RELATIVE


@dataclass(frozen=True)
class PolicyConfig:
    """Everything the adapter needs from one upstream policy config YAML."""

    name: str
    checkpoint: Path
    #: Full 22-joint config ordering (arms first, then legs).
    joints: Tuple[str, ...]
    #: Joint indices the policy observes and acts on (upstream action_indices).
    action_indices: Tuple[int, ...]
    #: Full-config default pose [rad] (one entry per ``joints`` entry).
    nominal: np.ndarray
    kp: np.ndarray
    kd: np.ndarray
    effort_limits: np.ndarray
    action_scale: float
    action_limit_lower: float
    action_limit_upper: float
    policy_dt: float
    physics_dt: float
    num_observations: int
    default_base_position: np.ndarray

    @property
    def action_joints(self) -> Tuple[str, ...]:
        """The joints this policy observes and drives, in policy order."""
        return tuple(self.joints[i] for i in self.action_indices)

    @property
    def nominal_pose(self) -> Dict[str, float]:
        """The policy default pose keyed by joint name (all 22)."""
        return dict(zip(self.joints, (float(v) for v in self.nominal)))

    def hold_pose(self) -> Dict[str, float]:
        """Targets when idle: defaults for all 22 joints, URDF-clamped.

        Non-action joints (e.g. arms under ``policy_humanoid_legs``) hold the
        default pose; action joints do too until the first decision.
        """
        pose = dict(self.nominal_pose)
        for joint, value in pose.items():
            if joint in POSITION_LIMITS:
                lo, hi = POSITION_LIMITS[joint]
                pose[joint] = max(lo, min(hi, value))
        return pose


def load_policy_config(
    upstream: Optional[Path] = None, name: str = "policy_humanoid"
) -> PolicyConfig:
    """Load one upstream policy config plus its checkpoint path.

    Raises ``ValueError`` if the config is not a feed-forward policy whose
    observation size is exactly 9 + 3 * len(action_indices) - the layout the
    vendored checkpoints were trained with. ``policy_humanoid`` (22 actions,
    75 dims) and ``policy_humanoid_legs`` (12 actions, 45 dims) both satisfy
    this; anything else is not what this adapter was verified against.
    """
    base = Path(upstream) if upstream is not None else upstream_dir()
    cfg_path = base / "configs" / f"{name}.yaml"
    cfg = yaml.safe_load(cfg_path.read_text())
    if cfg["history_length"] != 0:
        raise ValueError(
            f"{name}: expected a feed-forward policy (history_length == 0), "
            f"got {cfg['history_length']}")
    indices = tuple(int(i) for i in cfg["action_indices"])
    expected_obs = 9 + 3 * len(indices)
    if cfg["num_observations"] != expected_obs:
        raise ValueError(
            f"{name}: num_observations={cfg['num_observations']} does not "
            f"match 9 + 3 * len(action_indices) = {expected_obs}")
    if cfg["num_actions"] != len(indices) or cfg["num_joints"] != len(cfg["joints"]):
        raise ValueError(f"{name}: joints/actions/config disagree")
    if len(cfg["default_joint_positions"]) != len(cfg["joints"]):
        raise ValueError(f"{name}: default pose does not cover every joint")
    for arrays in ("joint_kp", "joint_kd", "effort_limits"):
        if not all(math.isfinite(float(v)) and float(v) >= 0 for v in cfg[arrays]):
            raise ValueError(f"{name}: {arrays} must be finite and nonnegative")
        if arrays == "effort_limits" and any(float(v) == 0 for v in cfg[arrays]):
            raise ValueError(f"{name}: effort limits must be positive")
        if len(cfg[arrays]) != len(cfg["joints"]):
            raise ValueError(f"{name}: {arrays} does not cover every joint")
    return PolicyConfig(
        name=name,
        checkpoint=base / cfg["policy_checkpoint_path"],
        joints=tuple(cfg["joints"]),
        action_indices=indices,
        nominal=np.asarray(cfg["default_joint_positions"], dtype=float),
        kp=np.asarray(cfg["joint_kp"], dtype=float),
        kd=np.asarray(cfg["joint_kd"], dtype=float),
        effort_limits=np.asarray(cfg["effort_limits"], dtype=float),
        action_scale=float(cfg["action_scale"]),
        action_limit_lower=float(cfg["action_limit_lower"]),
        action_limit_upper=float(cfg["action_limit_upper"]),
        policy_dt=float(cfg["policy_dt"]),
        physics_dt=float(cfg["physics_dt"]),
        num_observations=int(cfg["num_observations"]),
        default_base_position=np.asarray(cfg["default_base_position"], dtype=float),
    )



def clamp_command(command: Sequence[float]) -> np.ndarray:
    """Clamp a base-velocity command to the training command ranges."""
    if len(command) != 3:
        raise ValueError("command must be (vx, vy, wz)")
    return np.clip(np.asarray(command, dtype=float),
                   [-COMMAND_LIMITS[0], -COMMAND_LIMITS[1], -COMMAND_LIMITS[2]],
                   COMMAND_LIMITS)


class YawRateBoost:
    """Boost-only closed-loop correction of the policy's yaw command.

    The vendored checkpoint is a memoryless feed-forward network, and on this
    plant it can converge to a *standing* action while a pure-turn command is
    still held. Measured 2026-09-25 (``turn_pos03_gait_25hz_b``, held
    +0.3 rad/s from 3 s): the summed-|joint effort| spread inside a window
    drops from 9.4 N.m (3-4 s, yaw +0.35 rad/s) to 0.16-0.20 N.m (5-13 s,
    yaw +0.001 rad/s) - the policy parks at a constant target, it does not
    step and slip. The action is a function of the command input, so raising
    the commanded rate is the one lever that can move the policy off that
    fixed point without retraining.

    The servo is deliberately one-sided:

    - it only ever *raises* ``|wz|``, and only while the low-passed measured
      body-frame yaw rate falls short of the reference in the reference's
      direction; it never commands against the reference, so a released stick
      still commands exactly zero and the qualified stop behaviour is
      unchanged;
    - the boosted command is clamped to the training command range, so the
      policy never sees a yaw command outside what it was trained on;
    - a non-finite input returns the reference unchanged (the qualified
      open-loop behaviour) rather than a fabricated correction.

    Honest scope: this is an opt-in deployment aid for a real defect, not a
    fix of the root cause (the policy's own limit cycle); ``yaw_servo_gain``
    defaults to 0 (disabled) and the class has no effect while the reference
    is zero. It is unit-tested and, like the rest of this layer, has no ROS
    dependency.
    """

    def __init__(self, gain: float = 0.0, limit: float = YAW_COMMAND_LIMIT,
                 filter_tau_s: float = 0.08):
        for name, value in (("gain", gain), ("limit", limit),
                            ("filter_tau_s", filter_tau_s)):
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
        if gain < 0:
            raise ValueError("gain must be non-negative (boost-only servo)")
        if limit <= 0:
            raise ValueError("limit must be positive")
        if limit > YAW_COMMAND_LIMIT:
            raise ValueError(
                f"limit {limit} exceeds the training yaw range "
                f"(+/-{YAW_COMMAND_LIMIT})")
        if filter_tau_s < 0:
            raise ValueError("filter_tau_s must be non-negative")
        self.gain = float(gain)
        self.limit = float(limit)
        self.filter_tau_s = float(filter_tau_s)
        self._measured = 0.0

    @property
    def enabled(self) -> bool:
        """True when the servo can change a command (``gain > 0``)."""
        return self.gain > 0.0

    def reset(self) -> None:
        """Forget the filtered measurement."""
        self._measured = 0.0

    def command(self, reference: float, measured_yaw_rate: float,
                dt: float) -> float:
        """Yaw command for one policy cycle.

        ``reference`` is the operator's yaw-rate command [rad/s],
        ``measured_yaw_rate`` the latest body-frame yaw rate [rad/s] and
        ``dt`` the cycle period [s] used to low-pass the measurement.
        """
        if not all(math.isfinite(v) for v in
                   (reference, measured_yaw_rate, dt)):
            return float(reference)
        if dt > 0:
            alpha = (1.0 if self.filter_tau_s <= 0
                     else min(1.0, dt / self.filter_tau_s))
            self._measured += alpha * (float(measured_yaw_rate)
                                       - self._measured)
        if not self.enabled or reference == 0.0:
            return float(reference)
        direction = 1.0 if reference > 0 else -1.0
        deficit = max(0.0, abs(reference) - direction * self._measured)
        boosted = abs(reference) + self.gain * deficit
        return direction * min(self.limit, boosted)


def quaternion_gravity(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Projected gravity ``R(q).T @ [0, 0, -1]`` from a ROS (x, y, z, w) quat.

    Identical to ``qualify_policy.rotation_state``'s gravity vector (there
    the quaternion is unpacked w-first); asserted equal by the unit tests.
    Returns a unit vector for a unit quaternion.
    """
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        raise ValueError("zero-norm quaternion has no attitude")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([2 * (w * y - x * z), -2 * (w * x + y * z),
                     2 * (x * x + y * y) - 1])


def quaternion_tilt(x: float, y: float, z: float, w: float) -> float:
    """Tilt away from upright (the qualify_policy tilt convention) [rad]."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        raise ValueError("zero-norm quaternion has no attitude")
    x, y = x / norm, y / norm
    return math.acos(max(-1.0, min(1.0, 1.0 - 2 * (x * x + y * y))))


def build_observation(
    command: Sequence[float],
    gyro: Sequence[float],
    gravity: Sequence[float],
    joint_positions: Sequence[float],
    joint_velocities: Sequence[float],
    previous_action: Sequence[float],
    nominal_action_joints: Sequence[float],
) -> np.ndarray:
    """Assemble the policy observation vector in verified order.

    Order: command (3), gyro (3), gravity (3), q - default (n), dq (n),
    previous action (n), all over the policy's action joints (n = 12 legs or
    22 full body). All joint inputs must already be in policy order; nothing
    here reorders.
    """
    parts = (command, gyro, gravity, joint_positions, joint_velocities,
             previous_action)
    sizes = (COMMAND_WIDTH, 3, 3, len(nominal_action_joints),
             len(nominal_action_joints), len(nominal_action_joints))
    if any(len(p) != n for p, n in zip(parts, sizes)):
        raise ValueError("observation inputs have wrong sizes")
    return np.concatenate([
        np.asarray(command, dtype=float),
        np.asarray(gyro, dtype=float),
        np.asarray(gravity, dtype=float),
        np.asarray(joint_positions, dtype=float)
        - np.asarray(nominal_action_joints, dtype=float),
        np.asarray(joint_velocities, dtype=float),
        np.asarray(previous_action, dtype=float),
    ]).astype(np.float32)


def action_to_targets(
    raw_action: np.ndarray, config: PolicyConfig
) -> Dict[str, float]:
    """Policy output -> URDF-clamped position targets for all 22 joints.

    ``target = default + action_scale * clip(action, action_limits)`` for the
    action joints; joints the policy does not drive hold the default pose.
    Every target is clamped to the vendored URDF position limits so a
    runaway policy cannot command past the hardware range.
    """
    raw = np.asarray(raw_action, dtype=float)
    action_joints = config.action_joints
    if raw.shape != (len(action_joints),):
        raise ValueError("action shape does not match the configured joints")
    clipped = np.clip(raw, config.action_limit_lower, config.action_limit_upper)
    targets = config.hold_pose()
    nominal_action = np.asarray(
        [config.nominal[i] for i in config.action_indices], dtype=float)
    for joint, target in zip(action_joints, nominal_action + config.action_scale * clipped):
        lo, hi = POSITION_LIMITS[joint]
        targets[joint] = float(max(lo, min(hi, target)))
    return targets


class StartupSettle:
    """Hold the measured spawn pose, then ramp to the default pose on demand.

    The BHL spawns straight-legged (every measured joint ~0 rad) while the
    policy default pose bends the legs (hip_pitch -0.2, knee +0.4, ankle
    -0.3 rad per side). Commanding that bent-leg pose instantly into the
    standing biped destabilizes it: tilt rises past the 0.70 rad fall
    threshold and the policy latches SAFE_STOP within seconds (decisively
    reproduced in evidence ``r53-bhl-actuation-2026-09-16``, whose phase 1
    also proves the measured straight-legged pose is stable under zero
    commands). This is the startup transient that respects the spawn pose:

    - before :meth:`start`, :meth:`hold_targets` returns the *measured*
      pose, URDF-clamped (falling back to the policy pose for joints that
      were not measured) - effectively zero commands on a settled robot;
    - :meth:`start` captures that measured pose as the ramp origin;
    - :meth:`targets` blends origin -> default pose linearly over
      ``duration_s`` of accumulated ``dt``, clamped to the URDF limits;
    - :meth:`finish` ends the ramp immediately (used when the tilt safety
      threshold is crossed mid-ramp, so the controller's own SAFE_STOP path
      takes over on the next update).
    """

    def __init__(self, hold_pose: Dict[str, float], duration_s: float) -> None:
        duration_s = float(duration_s)
        if duration_s <= 0.0:
            raise ValueError(f"settle duration must be positive, got {duration_s}")
        self._hold = {joint: float(value) for joint, value in hold_pose.items()}
        self._duration = duration_s
        self._origin: Optional[Dict[str, float]] = None
        self._elapsed = 0.0
        self._started = False

    @property
    def duration_s(self) -> float:
        return self._duration

    @property
    def active(self) -> bool:
        """True while a ramp is in progress (started, not yet settled)."""
        return self._started and self._elapsed < self._duration

    @property
    def settled(self) -> bool:
        """True once the ramp has run to completion (or been finished)."""
        return self._started and self._elapsed >= self._duration

    def hold_targets(self, measured: Optional[Dict[str, float]]) -> Dict[str, float]:
        """Pre-command targets: the measured pose, URDF-clamped."""
        targets: Dict[str, float] = {}
        for joint, default in self._hold.items():
            value = default
            if measured and joint in measured:
                value = float(measured[joint])
            lo, hi = POSITION_LIMITS[joint]
            targets[joint] = float(max(lo, min(hi, value)))
        return targets

    def start(self, measured: Optional[Dict[str, float]]) -> None:
        """Capture the ramp origin from the latest measurement."""
        self._origin = self.hold_targets(measured)
        self._elapsed = 0.0
        self._started = True

    def finish(self) -> None:
        """End the ramp immediately; subsequent targets are the hold pose."""
        self._elapsed = self._duration

    def targets(
        self, measured: Optional[Dict[str, float]], dt: float
    ) -> Tuple[Dict[str, float], bool]:
        """Advance the ramp by ``dt``; return (targets, settled).

        Before :meth:`start` this returns the measured-pose hold and
        ``settled=False``. Every target is clamped to the URDF limits.
        """
        if not self._started:
            return self.hold_targets(measured), False
        self._elapsed += float(dt)
        alpha = min(1.0, self._elapsed / self._duration)
        origin = self._origin or {}
        targets: Dict[str, float] = {}
        for joint, default in self._hold.items():
            start = origin.get(joint, default)
            value = start + alpha * (default - start)
            lo, hi = POSITION_LIMITS[joint]
            targets[joint] = float(max(lo, min(hi, value)))
        return targets, alpha >= 1.0


@dataclass
class BhlPolicyCycle:
    """One policy decision, mirroring ``BhlControlCycle``'s contract."""

    position_targets: Dict[str, float]
    issues: List[str] = field(default_factory=list)
    safety_state: str = "NOMINAL"
    observation: Optional[np.ndarray] = None
    action: Optional[np.ndarray] = None


class BhlPolicyController:
    """Closed-loop ONNX policy over onboard state, with a latched SAFE_STOP.

    ``session`` is anything with ``run(None, {input_name: obs}) -> [action]``
    (an onnxruntime session in production, a stub in tests). It is created
    lazily by :meth:`ensure_session` so importing this module never requires
    onnxruntime.
    """

    NOMINAL = "NOMINAL"
    SAFE_STOP = "SAFE_STOP"

    def __init__(self, config: PolicyConfig, session=None) -> None:
        self.config = config
        self.session = session
        self.safety_state = self.NOMINAL
        self.reason: Optional[str] = None
        self._action_joints = config.action_joints
        self._nominal_action = config.nominal[list(config.action_indices)].copy()
        self._previous_action = np.zeros(len(self._action_joints))
        self._targets = config.hold_pose()

    # -- session handling ------------------------------------------------
    def ensure_session(self):
        """Create the onnxruntime session on first use (CPU provider)."""
        if self.session is None:
            import onnxruntime as ort

            options = ort.SessionOptions()
            options.intra_op_num_threads = options.inter_op_num_threads = 1
            self.session = ort.InferenceSession(
                str(self.config.checkpoint), options,
                providers=["CPUExecutionProvider"])
        return self.session

    def reset(self) -> None:
        """Explicit operator reset of the latched SAFE_STOP."""
        self.safety_state = self.NOMINAL
        self.reason = None
        self._previous_action = np.zeros(len(self._action_joints))
        self._targets = self.config.hold_pose()

    def _hold(self, issues: List[str]) -> BhlPolicyCycle:
        """Hold the previous targets (missing data or latched stop)."""
        return BhlPolicyCycle(
            position_targets=dict(self._targets),
            issues=issues, safety_state=self.safety_state)

    def _safe_stop(self, reason: str, issues: List[str]) -> BhlPolicyCycle:
        """Latch SAFE_STOP and return the default-pose hold."""
        self.safety_state = self.SAFE_STOP
        self.reason = reason
        issues.append(f"safe_stop: {reason}")
        self._targets = self.config.hold_pose()
        self._previous_action = np.zeros(len(self._action_joints))
        return self._hold(issues)


    # -- one decision ----------------------------------------------------
    def update(
        self,
        command: Sequence[float],
        gyro: Sequence[float],
        orientation: Sequence[float],
        measured_positions: Dict[str, float],
        measured_velocities: Optional[Dict[str, float]] = None,
    ) -> BhlPolicyCycle:
        """One policy cycle from onboard state.

        ``orientation`` is the IMU quaternion in ROS (x, y, z, w) order;
        ``gyro`` is the body-frame angular velocity [rad/s];
        ``measured_positions`` maps joint name -> measured position [rad].
        ``measured_velocities`` maps joint name -> measured velocity [rad/s]
        and may be omitted, in which case zeros are used (honest degraded
        tracking, never a fabricated dq).
        """
        issues: List[str] = []
        x, y, z, w = (float(v) for v in orientation)
        tilt = quaternion_tilt(x, y, z, w)

        if self.safety_state == self.SAFE_STOP:
            return self._hold(issues)

        if tilt >= TILT_FALL_RAD:
            return self._safe_stop(
                f"tilt {tilt:.2f} rad exceeds fall threshold", issues)

        action_joints = self._action_joints
        missing = [j for j in action_joints if j not in measured_positions]
        if missing:
            # No measurement -> no inference. Hold the previous targets and
            # say so; a fabricated observation would be a lie about state.
            preview = ", ".join(missing[:3]) + ("..." if len(missing) > 3 else "")
            issues.append(
                f"warn: {len(missing)} joint measurement(s) missing, "
                f"holding previous targets ({preview})")
            return self._hold(issues)

        if len(gyro) != 3:
            raise ValueError("gyro must be body-frame (wx, wy, wz)")

        positions = np.asarray(
            [measured_positions[j] for j in action_joints], dtype=float)
        if measured_velocities is not None:
            velocities = np.asarray(
                [float(measured_velocities.get(j, 0.0))
                 for j in action_joints], dtype=float)
        else:
            velocities = np.zeros(len(action_joints))

        obs = build_observation(
            clamp_command(command), gyro,
            quaternion_gravity(x, y, z, w),
            positions, velocities, self._previous_action,
            self._nominal_action)

        session = self.ensure_session()
        input_info = session.get_inputs()[0]
        raw = np.asarray(session.run(None, {input_info.name: obs[None, :]})[0])
        if raw.shape != (1, len(action_joints)) or not np.isfinite(raw).all():
            return self._safe_stop("invalid policy output", issues)

        action = np.clip(raw[0], self.config.action_limit_lower,
                         self.config.action_limit_upper)
        self._previous_action = action
        self._targets = action_to_targets(action, self.config)
        return BhlPolicyCycle(
            position_targets=dict(self._targets),
            issues=issues, safety_state=self.safety_state,
            observation=obs, action=action)





