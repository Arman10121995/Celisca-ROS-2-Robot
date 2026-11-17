"""Backend-neutral conveyor-belt force law (R11 payload transfer).

A body resting on a belt deck is driven so that its horizontal velocity
relative to the deck approaches the belt velocity (belt speed along the deck
axis, zero across it), with a force bounded by Coulomb friction. A stopped
belt therefore still grips. Used by the MuJoCo and PyBullet belts; Gazebo
uses its TrackController.
"""
import numpy as np


def belt_force(belt_speed, axis, relative_velocity, mass, normal_force, mu=0.8, gain=20.0):
    """World horizontal force [fx, fy, 0] on one body; axis is the deck's horizontal direction."""
    axis = np.array(axis, dtype=float)
    axis[2] = 0.
    norm = float(np.linalg.norm(axis))
    if norm < 1e-6 or normal_force <= 0. or mass <= 0.:
        return np.zeros(3)
    axis /= norm
    relative = np.array(relative_velocity, dtype=float)
    relative[2] = 0.
    force = gain * mass * (belt_speed * axis - relative)
    limit = mu * normal_force
    size = float(np.linalg.norm(force))
    if size > limit:
        force *= limit / size
    return force
