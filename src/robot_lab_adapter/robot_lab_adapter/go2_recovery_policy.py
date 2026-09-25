"""Opt-in NJU-RLC Go2 get-up actor adapted to measured ROS observations.

The deployed actor consumes 57 proprioceptive values and ten prior frames.
Its use remains experimental until a live collapse/get-up trial passes.
"""

from __future__ import annotations

import math

import numpy as np

from robot_lab_adapter.go2_locomotion import (
    BodyState, JOINT_NAMES, LEG_PREFIXES, clamp_effort, clamp_position,
)


RECOVERY_DEFAULT = np.array([0.0, 0.9, -1.8] * 4, dtype=np.float32)
RECOVERY_ACTION_SCALE = np.array([0.075, 0.25, 0.25] * 4, dtype=np.float32)
RECOVERY_KP = 40.0
RECOVERY_KD = 1.0
RECOVERY_CONTACT_THRESHOLD_N = 14.0
RECOVERY_POLICY_DT_S = 0.02
# Keep both the physical target and the recurrent previous-action observation
# within a bounded range when the learned actor leaves its training envelope.
RECOVERY_ACTION_ABS_LIMIT = 4.0


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
        self.target = np.asarray([
            clamp_position(name, float(value))
            for name, value in zip(
                JOINT_NAMES, RECOVERY_DEFAULT + RECOVERY_ACTION_SCALE * action)
        ], dtype=np.float32)
        return self.target.copy()

    def efforts(self, positions, velocities):
        values = {}
        for i, name in enumerate(JOINT_NAMES):
            if name not in positions or name not in velocities:
                values[name] = 0.0
                continue
            tau = RECOVERY_KP * (self.target[i] - positions[name]) \
                - RECOVERY_KD * velocities[name]
            values[name] = clamp_effort(name, float(tau)) if math.isfinite(tau) else 0.0
        return values
