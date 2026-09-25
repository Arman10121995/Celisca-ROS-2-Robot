"""Opt-in NJU-RLC Go2 get-up actor adapted to measured ROS observations.

The deployed actor consumes 57 proprioceptive values and ten prior frames.
Its use remains experimental until a live collapse/get-up trial passes.

Plant-parity notes (R5.2, from the recorded 60 N trials):

- The learned action is mapped through the **measured per-joint** Go2 gains
  (hip 100/5, thigh 300/8, calf 300/8 from ``robot_control.yaml``) rather than
  one flat gain. A single flat gain left the load-bearing thigh and calf cold
  relative to the hips.
- The joint target is **rate-limited** (:data:`RECOVERY_TARGET_SLEW_RAD_S`).
  The actor is stepped at 50 Hz against 12 light, zero-armature joints; without
  a slew limit a single large action change threw the measured joint velocity to
  19.4 rad/s and flipped the body onto its back.
"""

from __future__ import annotations

import math

import numpy as np

from robot_lab_adapter.go2_locomotion import (
    BodyState, JOINT_NAMES, LEG_PREFIXES, PD_GAINS, clamp_effort,
    clamp_position, joint_kind,
)


RECOVERY_DEFAULT = np.array([0.0, 0.9, -1.8] * 4, dtype=np.float32)
RECOVERY_ACTION_SCALE = np.array([0.075, 0.25, 0.25] * 4, dtype=np.float32)
RECOVERY_CONTACT_THRESHOLD_N = 14.0
RECOVERY_POLICY_DT_S = 0.02
# Keep both the physical target and the recurrent previous-action observation
# within a bounded range when the learned actor leaves its training envelope.
RECOVERY_ACTION_ABS_LIMIT = 4.0
#: Per-joint target slew [rad/s]. Bounds how fast the commanded pose may move
#: at the 50 Hz policy rate; a full-range joint takes >= 0.5 s to traverse.
RECOVERY_TARGET_SLEW_RAD_S = 3.0
#: Fraction of the measured per-joint PD gains used while re-standing. Matches
#: the 0.2 scale qualified for stance on this MuJoCo plant.
RECOVERY_GAIN_SCALE = 0.2
#: Fraction of the measured per-joint damping gains used while re-standing.
RECOVERY_DAMPING_SCALE = 0.2


def _scaled_gains():
    """Per-joint (kp, kd) from the measured Go2 gains, in JOINT_NAMES order."""
    kp = np.array([PD_GAINS[joint_kind(name)][0] * RECOVERY_GAIN_SCALE
                   for name in JOINT_NAMES], dtype=np.float32)
    kd = np.array([PD_GAINS[joint_kind(name)][1] * RECOVERY_DAMPING_SCALE
                   for name in JOINT_NAMES], dtype=np.float32)
    return kp, kd


RECOVERY_KP, RECOVERY_KD = _scaled_gains()


class Go2RecoveryPolicy:
    """Generate bounded 12-joint get-up efforts from the pretrained actor."""

    def __init__(self, path: str, session=None):
        if session is None:
            import onnxruntime as ort
            session = ort.InferenceSession(path, providers=["CPUExecutionProvider"])
        inputs, outputs = session.get_inputs(), session.get_outputs()
        if ({item.name: item.shape for item in inputs}
                != {"proprio": [1, 57], "history": [1, 570]}
                or len(outputs) != 1 or outputs[0].shape != [1, 12]):
            raise ValueError("Go2 recovery policy requires [1,57] proprio, "
                             "[1,570] history and [1,12] output")
        self.session = session
        self.reset()

    def reset(self):
        self.last_action = np.zeros(12, dtype=np.float32)
        self.last_proprio = None
        self.history = np.zeros((10, 57), dtype=np.float32)
        self.target = RECOVERY_DEFAULT.copy()

    def step(self, positions, velocities, angular_velocity,
             body: BodyState, foot_forces):
        if body is None or foot_forces is None or angular_velocity is None:
            raise ValueError("Go2 recovery requires body attitude, angular velocity "
                             "and direct foot forces")
        if not all(name in positions and name in velocities for name in JOINT_NAMES):
            raise ValueError("Go2 recovery requires all 12 measured joints")
        if not all(leg in foot_forces for leg in LEG_PREFIXES):
            raise ValueError("Go2 recovery requires all four foot forces")
        q = np.asarray([positions[name] for name in JOINT_NAMES], dtype=np.float32)
        dq = np.asarray([velocities[name] for name in JOINT_NAMES], dtype=np.float32)
        contacts = np.asarray([
            0.5 if foot_forces[leg] > RECOVERY_CONTACT_THRESHOLD_N else -0.5
            for leg in LEG_PREFIXES], dtype=np.float32)
        proprio = np.concatenate((
            (body.roll_rad, body.pitch_rad),
            np.asarray(angular_velocity, dtype=np.float32) * 0.25,
            q - RECOVERY_DEFAULT, dq * 0.05, self.last_action, contacts,
            np.zeros(12, dtype=np.float32)), dtype=np.float32)
        if proprio.shape != (57,) or not np.isfinite(proprio).all():
            raise ValueError("non-finite Go2 recovery observation")
        proprio = np.clip(proprio, -100.0, 100.0)
        if self.last_proprio is not None:
            self.history[:-1] = self.history[1:]
            self.history[-1] = self.last_proprio
        self.last_proprio = proprio.copy()
        action = np.asarray(self.session.run(None, {
            "proprio": proprio.reshape(1, 57),
            "history": self.history.reshape(1, 570),
        })[0], dtype=np.float32).reshape(12)
        if not np.isfinite(action).all():
            raise ValueError(
                "non-finite Go2 recovery action (max |proprio| %.3g, "
                "max |history| %.3g, max |q| %.3g, max |dq| %.3g)" % (
                    np.max(np.abs(proprio)), np.max(np.abs(self.history)),
                    np.max(np.abs(q)), np.max(np.abs(dq))))
        action = np.clip(action, -RECOVERY_ACTION_ABS_LIMIT,
                         RECOVERY_ACTION_ABS_LIMIT)
        self.last_action = action.copy()
        desired = np.asarray([
            clamp_position(name, float(value))
            for name, value in zip(
                JOINT_NAMES, RECOVERY_DEFAULT + RECOVERY_ACTION_SCALE * action)
        ], dtype=np.float32)
        # Rate-limit the commanded pose: the actor runs at 50 Hz against light
        # zero-armature joints, so an unbounded target step is what threw the
        # body onto its back in the recorded trials.
        max_step = RECOVERY_TARGET_SLEW_RAD_S * RECOVERY_POLICY_DT_S
        delta = np.clip(desired - self.target, -max_step, max_step)
        self.target = np.asarray([
            clamp_position(name, float(value))
            for name, value in zip(JOINT_NAMES, self.target + delta)
        ], dtype=np.float32)
        return self.target.copy()

    def efforts(self, positions, velocities):
        values = {}
        for i, name in enumerate(JOINT_NAMES):
            if name not in positions or name not in velocities:
                values[name] = 0.0
                continue
            tau = float(RECOVERY_KP[i]) * (float(self.target[i])
                                           - positions[name]) \
                - float(RECOVERY_KD[i]) * velocities[name]
            # Cast to a plain float: a numpy scalar here fails the std_msgs
            # Float64MultiArray type assertion and kills the controller.
            values[name] = clamp_effort(name, float(tau)) \
                if math.isfinite(tau) else 0.0
        return values
