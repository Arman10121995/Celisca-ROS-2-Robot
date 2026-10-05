"""Bounded position trajectories; these produce targets, never robot poses."""
from dataclasses import dataclass

import numpy as np


@dataclass
class PositionTrajectory:
    times: np.ndarray
    positions: np.ndarray

    def sample(self, elapsed):
        segment = min(len(self.times)-2,
                      max(0, int(np.searchsorted(self.times, elapsed, side='right'))-1))
        duration = self.times[segment+1]-self.times[segment]
        fraction = np.clip((elapsed-self.times[segment])/duration, 0., 1.)
        delta = self.positions[segment+1]-self.positions[segment]
        position = self.positions[segment] + fraction*fraction*(3-2*fraction)*delta
        velocity = 6*fraction*(1-fraction)*delta/duration
        return position, velocity


def position_trajectory(names, expected_names, points, actual, limits,
                        max_velocity=.5, max_duration=15.):
    """Validate before reserving a goal; reorder every waypoint to model order.

    Cubic segments start and finish at rest. Their maximum velocity is 1.5
    times the average; checking that bound also checks the initial segment.
    """
    if len(names) != len(expected_names) or set(names) != set(expected_names):
        raise ValueError('Supply each of the seven arm joints exactly once')
    if not points:
        raise ValueError('At least one timed position waypoint is required')
    order = [names.index(name) for name in expected_names]
    positions = [np.asarray(actual, dtype=float)]
    times = [0.]
    if not np.all(np.isfinite(positions[0])):
        raise ValueError('Measured starting position is not finite')
    for duration, values in points:
        if len(values) != len(expected_names):
            raise ValueError('Every waypoint needs seven positions')
        position = np.asarray(values, dtype=float)[order]
        if not np.isfinite(duration) or not np.all(np.isfinite(position)):
            raise ValueError('Times and positions must be finite')
        if duration <= times[-1] or duration > max_duration:
            raise ValueError('Waypoint times must increase and finish within 15 seconds')
        if np.any(position < limits[:, 0]+.005) or np.any(position > limits[:, 1]-.005):
            raise ValueError('Waypoint exceeds the model joint limits with 0.005 rad margin')
        if np.any(1.5*np.abs(position-positions[-1])/(duration-times[-1]) > max_velocity+1e-9):
            raise ValueError('Trajectory exceeds the 0.5 rad/s target velocity limit')
        positions.append(position)
        times.append(duration)
    return PositionTrajectory(np.asarray(times), np.asarray(positions))
