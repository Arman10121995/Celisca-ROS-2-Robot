"""Optional Go2 ONNX velocity policy adapter for measured ROS joint/IMU data.

The model file is deliberately supplied by path: the policy must be validated
on this simulator plant before it can replace the guarded stance default.
"""

from __future__ import annotations

import math

import numpy as np

from robot_lab_adapter.go2_locomotion import JOINT_NAMES, clamp_effort, clamp_position


POLICY_DEFAULT = np.array(
    [-0.1, 0.9, -1.8, 0.1, 0.9, -1.8] * 2, dtype=np.float32)
POLICY_KP = np.array([20.0, 20.0, 40.0] * 4, dtype=np.float32)
POLICY_KD = np.array([1.0, 1.0, 2.0] * 4, dtype=np.float32)
POLICY_ACTION_SCALE = 0.5
POLICY_DT = 0.02
REVERSE_COMMAND_MAPS = ("feedforward", "inverse")
# Fitted from the 2026-09-25 four-command R5.2 sweep: the measured reverse
# speed is approximately 1.0367 * policy_command_magnitude - 0.2291 m/s.
# The inverse candidate ramps from zero over a 0.20 m/s requested deadband so
# a zero command cannot inject a finite reverse step.
REVERSE_INVERSE_SLOPE = 1.0367
REVERSE_INVERSE_OFFSET = 0.2291
REVERSE_INVERSE_DEADBAND_MPS = 0.20


def policy_forward_command(vx: float, reverse_map: str = "feedforward") -> float:
    """Map a requested forward/reverse velocity to the policy observation.

    The default feed-forward map bypasses the measured reverse dead zone. The
    opt-in inverse map is fitted to the R5.2 sweep and ramps continuously from
    zero through a bounded low-speed deadband.
    """
    if reverse_map not in REVERSE_COMMAND_MAPS:
        raise ValueError("reverse_map must be 'feedforward' or 'inverse'")
    if vx >= 0:
        return vx
    magnitude = abs(vx)
    if reverse_map == "feedforward":
        return -min(1.0, 0.8 * magnitude + 0.35 * min(1.0, magnitude / 0.1))
    # Keep zero command at zero, then use the measured inverse relation.
    if magnitude <= REVERSE_INVERSE_DEADBAND_MPS:
        boundary = (REVERSE_INVERSE_DEADBAND_MPS + REVERSE_INVERSE_OFFSET) \
            / REVERSE_INVERSE_SLOPE
        return -magnitude / REVERSE_INVERSE_DEADBAND_MPS * boundary
    policy_magnitude = (magnitude + REVERSE_INVERSE_OFFSET) / REVERSE_INVERSE_SLOPE
    return -min(1.0, policy_magnitude)


def projected_gravity(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Unit world down direction expressed in the IMU/body frame."""
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm < 1e-9:
        raise ValueError("missing IMU orientation")
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    return np.asarray((-2*(x*z-w*y), -2*(y*z+w*x),
                       -(1-2*(x*x+y*y))), dtype=np.float32)


class Go2VelocityPolicy:
    """Run a 45-observation/12-action ONNX Go2 policy at 50 Hz."""

    def __init__(self, path: str, session=None, reverse_map: str = "feedforward"):
        if reverse_map not in REVERSE_COMMAND_MAPS:
            raise ValueError("reverse_map must be 'feedforward' or 'inverse'")
        if session is None:
            import onnxruntime as ort
            session = ort.InferenceSession(path, providers=["CPUExecutionProvider"])
        inputs, outputs = session.get_inputs(), session.get_outputs()
        if len(inputs) != 1 or inputs[0].shape != [1, 45] \
                or len(outputs) != 1 or outputs[0].shape != [1, 12]:
            raise ValueError("Go2 policy requires one [1,45] input and one [1,12] output")
        self.session = session
        self.reverse_map = reverse_map
        self.input_name = inputs[0].name
        self.last_action = np.zeros(12, dtype=np.float32)
        self.target = POLICY_DEFAULT.copy()

    def reset(self):
        self.last_action.fill(0.0)
        self.target = POLICY_DEFAULT.copy()

    def step(self, positions, velocities, angular_velocity, orientation, command):
        if not all(name in positions and name in velocities for name in JOINT_NAMES):
            raise ValueError("Go2 policy requires all 12 measured joints")
        q = np.asarray([positions[name] for name in JOINT_NAMES], dtype=np.float32)
        dq = np.asarray([velocities[name] for name in JOINT_NAMES], dtype=np.float32)
        grav = projected_gravity(*orientation)
        obs = np.concatenate((angular_velocity, grav,
                              (policy_forward_command(command.vx, self.reverse_map),
                               command.vy, command.wz),
                              q - POLICY_DEFAULT, dq, self.last_action),
                             dtype=np.float32).reshape(1, 45)
        if not np.isfinite(obs).all():
            raise ValueError("non-finite Go2 policy observation")
        action = np.asarray(self.session.run(None, {self.input_name: obs})[0],
                            dtype=np.float32).reshape(12)
        if not np.isfinite(action).all():
            raise ValueError("non-finite Go2 policy action")
        self.last_action = action.copy()
        target = POLICY_DEFAULT + POLICY_ACTION_SCALE * action
        self.target = np.asarray([
            clamp_position(name, float(value)) for name, value in zip(JOINT_NAMES, target)
        ], dtype=np.float32)
        return self.target.copy()

    def efforts(self, positions, velocities):
        values = []
        for i, name in enumerate(JOINT_NAMES):
            if name not in positions or name not in velocities:
                values.append(0.0)
            else:
                tau = POLICY_KP[i]*(self.target[i]-positions[name]) \
                    - POLICY_KD[i]*velocities[name]
                values.append(clamp_effort(name, float(tau)))
        return dict(zip(JOINT_NAMES, values))
