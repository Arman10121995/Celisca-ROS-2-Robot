"""Bicycle source limits, closed-loop correction and whole-footprint geometry.

These are numerical implementation checks, not simulator mission certificates.
"""
import math

import numpy as np
import pytest

from robot_lab_algorithms.bicycle_control import BicycleTracker, LAWS, convex_footprint_blocked, swept_footprint_blocked
from robot_lab_algorithms.grid_planners import GridView


def drive(axle='front'):
    return dict(type='ackermann', wheelbase=.32, max_steer=.58, max_steer_rate=2.,
        max_accel=1.5, max_speed=1., steer_axle=axle)


@pytest.mark.parametrize('law', LAWS)
@pytest.mark.parametrize('axle', ('front', 'rear'))
def test_lateral_error_recovers_with_steering_slew_and_curvature_limits(law, axle):
    control = BicycleTracker(law, drive(axle), max_speed=.25, deadline_s=.08)
    reference = control.reference([(0., 0.), (5., 0.)])
    pose, velocity = np.array([0., .2, 0.]), (0., 0.)
    last_delta = 0.
    dt = .05
    for _ in range(100):
        v, w = control.command(pose, velocity, reference, dt)
        assert 0 <= v <= .25
        assert abs(w) <= abs(v)*math.tan(.58)/.32+1e-9
        assert abs(control.delta-last_delta) <= 2*dt+1e-9
        last_delta = control.delta
        # Independent exact constant-curvature arc integration at fixed axle.
        if abs(w) > 1e-9:
            pose[0] += v/w*(math.sin(pose[2]+w*dt)-math.sin(pose[2]))
            pose[1] -= v/w*(math.cos(pose[2]+w*dt)-math.cos(pose[2]))
        else:
            pose[:2] += v*dt*np.array([math.cos(pose[2]), math.sin(pose[2])])
        pose[2] += w*dt
        velocity = (v, w)
    assert abs(pose[1]) < .12


def test_mpc_budget_uses_bounded_fallback():
    control = BicycleTracker('bicycle_mpc', drive(), deadline_s=1e-12)
    command = control.command([0., .2, 0.], [0., 0.], control.reference([(0., 0.), (2., 0.)]), .05)
    assert control.deadline_misses == 1
    assert 'fallback' in control.solver_state
    assert all(map(math.isfinite, command)) and abs(command[1]) <= command[0]*math.tan(.58)/.32


def test_invalid_state_rejected_and_car_never_spins_when_stopped():
    control = BicycleTracker('bicycle_stanley', drive())
    reference = control.reference([(0., 0.), (2., 0.)])
    with pytest.raises(ValueError):
        control.command([math.nan, 0., 0.], [0., 0.], reference, .05)
    v, w = control.command([0., 0., math.pi], [0., 0.], reference, .05)
    assert v > 0 and abs(w) <= v*math.tan(.58)/.32


def grid():
    return GridView(np.zeros((80, 80), dtype=int), (-2., -2.), .05)


def test_side_corner_hits_a_cell_that_the_centerline_misses():
    view = grid()
    row, column = view.cells([[.39, .15]])
    view.data[row[0], column[0]] = 100
    footprint = np.array([[.42, .17], [.42, -.17], [-.10, -.17], [-.10, .17]])
    assert convex_footprint_blocked(view, footprint)
    assert swept_footprint_blocked(view, footprint, (0., 0., 0.), (.3, 0.), 1.5)


def test_open_grid_passes_but_outside_and_unknown_are_blocked():
    view = grid()
    footprint = np.array([[.42, .17], [.42, -.17], [-.10, -.17], [-.10, .17]])
    assert not swept_footprint_blocked(view, footprint, (0., 0., 0.), (.3, .15), 1.5)
    assert convex_footprint_blocked(view, footprint+[2., 0.])
    row, column = view.cells([[0., 0.]])
    view.data[row[0], column[0]] = -1
    assert convex_footprint_blocked(view, footprint)
