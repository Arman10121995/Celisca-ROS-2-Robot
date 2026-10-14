"""Position-only, source-bounded paths for the experimental native arm adapter."""
import numpy as np

from .arm_trajectory import PositionTrajectory


def checked_arm_path(names, contract, points, actual, max_duration=120.):
    expected = contract['joint_names']
    if len(names) != len(expected) or set(names) != set(expected):
        raise ValueError('Supply every selected arm joint exactly once')
    if not 1 <= len(points) <= 256:
        raise ValueError('Supply 1–256 position waypoints')
    order = [names.index(name) for name in expected]
    limits = np.asarray(contract['limits'])
    margins, rates = np.asarray(contract['margins']), np.asarray(contract['max_velocity'])
    accelerations = np.where(np.asarray(contract['units']) == 'm', .06, .6)
    positions, times = [np.asarray(actual, dtype=float)], [0.]
    if positions[0].shape != (len(expected),) or not np.all(np.isfinite(positions[0])):
        raise ValueError('Finite measured starting joint positions required')
    for duration, values in points:
        if len(values) != len(expected):
            raise ValueError('Every waypoint needs every selected arm position')
        target = np.asarray(values, dtype=float)[order]
        if not np.isfinite(duration) or not np.all(np.isfinite(target)) or not times[-1] < duration <= max_duration:
            raise ValueError('Finite increasing waypoint times must finish within 120 simulation seconds')
        if np.any(target < limits[:, 0]+margins) or np.any(target > limits[:, 1]-margins):
            raise ValueError('Arm waypoint exceeds the source bounds and planning margin')
        delta, dt = np.abs(target-positions[-1]), duration-times[-1]
        if np.any(1.5*delta/dt > rates+1e-9) or np.any(6*delta/dt**2 > accelerations+1e-9):
            raise ValueError('Arm path exceeds bounded cubic target velocity/acceleration')
        times.append(duration)
        positions.append(target)
    return PositionTrajectory(np.asarray(times), np.asarray(positions))


def retime_arm_path(names, contract, positions, actual):
    """Keep every collision-checked straight joint-space edge, including start."""
    expected = contract['joint_names']
    if len(names) != len(expected) or set(names) != set(expected):
        raise ValueError('Planner returned a different arm joint set')
    order = [names.index(name) for name in expected]
    rates = np.asarray(contract['max_velocity'])
    acceleration = np.where(np.asarray(contract['units']) == 'm', .06, .6)
    previous, elapsed, points = np.asarray(actual, dtype=float), 0., []
    for values in positions:
        if len(values) != len(expected):
            raise ValueError('Planner returned an incomplete waypoint')
        target = np.asarray(values, dtype=float)[order]
        if not np.all(np.isfinite(target)):
            raise ValueError('Planner returned a non-finite waypoint')
        delta = np.abs(target-previous)
        if np.max(delta) < 1e-10:
            continue
        elapsed += max(.05, float(np.max(1.5*delta/rates)), float(np.max(np.sqrt(6*delta/acceleration))))
        points.append((elapsed, target.tolist()))
        previous = target
    checked_arm_path(expected, contract, points, actual)
    return points
