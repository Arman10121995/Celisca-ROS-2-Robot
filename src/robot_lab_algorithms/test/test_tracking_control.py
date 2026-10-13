"""R7.8 path-tracking controllers on the shared differential-drive plant."""
import math

import numpy as np
import pytest

from robot_lab_algorithms import nav2_adapters
from robot_lab_algorithms.tracking_control import (
    CONTROLLERS, ActuatorPlant, BacksteppingTracker, Limits, LinearMpcTracker, LqrTracker,
    NonlinearMpcTracker, PidTracker, Reference, _lateral_model, make_controller)

DT = 0.05


def straight(length=8.0):
    return [(x, 0.0) for x in np.linspace(0.0, length, int(length / 0.05) + 1)]


def simulate(controller, reference, pose, limits=None, duration=20.0, tau=0.1):
    limits = limits or controller.limits
    plant = ActuatorPlant(limits=limits, tau=tau, pose=pose)
    hint = reference.project(pose[0], pose[1], 0.0, back=0.0, ahead=2.0)
    trace = []
    for _ in range(int(duration / DT)):
        p = plant.pose
        hint = reference.project(p[0], p[1], hint)
        along, lateral, heading = reference.errors(p, hint)
        v, w = controller.command(tuple(p), tuple(plant.velocity), reference, hint, DT)
        trace.append((hint, lateral, heading, v, w, tuple(p)))
        plant.step((v, w), DT)
    return trace


def test_reference_profile_and_projection():
    reference = Reference([(0, 0), (2, 0), (2, 2)], v_max=0.3)
    assert reference.length == pytest.approx(4.0, abs=0.05)
    speeds = [reference.speed(s) for s in np.linspace(0, reference.length, 50)]
    assert max(speeds) <= 0.3 + 1e-9
    assert reference.speed(reference.length) == 0.0
    # The corner is slower than the straights (lateral-acceleration limit).
    assert reference.speed(2.0) < reference.speed(0.8)
    assert reference.project(1.0, 0.1, 0.0) == pytest.approx(1.0, abs=0.02)
    # Past the end the extended reference continues along the final heading.
    x, y, theta, kappa = reference.state(reference.length + 0.2, extend=True)
    assert (x, y) == pytest.approx((2.0, 2.2), abs=0.03) and kappa == 0.0


@pytest.mark.parametrize('name', sorted(CONTROLLERS))
def test_every_controller_converges_from_a_lateral_offset(name):
    reference = Reference(straight(), v_max=0.3)
    trace = simulate(make_controller(name), reference, (0.3, 0.3, 0.0), duration=15.0)
    final = trace[-20:]
    assert all(abs(lat) < 0.03 and abs(head) < 0.05 for _, lat, head, *_ in final), final[-1]
    assert all(0.0 <= v <= 0.3 + 1e-9 and abs(w) <= 1.0 + 1e-9 for _, _, _, v, w, _ in trace)


def test_nonlinear_mpc_adjoint_gradient_matches_finite_differences():
    controller = NonlinearMpcTracker()
    n = controller.horizon
    rng = np.random.default_rng(3)
    state = np.array([0.1, -0.2, 0.3])
    ref = rng.normal(size=(n, 3))
    u_ref = rng.normal(size=(n, 2)) * 0.2
    u_prev = np.array([[0.1, 0.2]])
    u = rng.normal(size=2 * n) * 0.2
    _, gradient = controller._cost_and_gradient(u, state, ref, u_ref, u_prev)
    numeric = np.zeros_like(u)
    for i in range(len(u)):
        e = np.zeros_like(u)
        e[i] = 1e-6
        numeric[i] = (controller._cost_and_gradient(u + e, state, ref, u_ref, u_prev)[0]
                      - controller._cost_and_gradient(u - e, state, ref, u_ref, u_prev)[0]) / 2e-6
    assert np.max(np.abs(numeric - gradient)) < 1e-5


@pytest.mark.parametrize('speed', [0.05, 0.15, 0.3])
def test_lqr_closed_loop_is_stable(speed):
    controller = LqrTracker()
    a, b = _lateral_model(speed, controller.dt)
    eigenvalues = np.linalg.eigvals(a - b @ controller.gain(speed))
    assert np.all(np.abs(eigenvalues) < 1.0)


def test_linear_mpc_respects_the_turn_rate_bound():
    limits = Limits(w_max=0.2)
    controller = LinearMpcTracker(limits=limits)
    reference = Reference(straight(), v_max=0.3)
    # A 1 m offset wants far more than 0.2 rad/s; the QP solution must stay inside.
    v, w = controller._command((0.5, 1.0, 0.0), (0.0, 0.0), reference, 0.5, DT)
    assert abs(w) <= 0.2 + 1e-9 and w < 0


def test_pid_anti_windup_bounds_the_integral_and_reduces_overshoot():
    limits = Limits(w_max=0.3)
    reference = Reference(straight(12.0), v_max=0.3)
    results = {}
    for anti_windup in (True, False):
        controller = PidTracker(limits=limits, ki=0.6, anti_windup=anti_windup)
        trace = simulate(controller, reference, (0.5, 0.8, 0.0), limits=limits, duration=30.0)
        results[anti_windup] = (max(0.0, -min(lat for _, lat, *_ in trace)), abs(controller.integral))
    assert results[True][0] < results[False][0]
    assert results[True][1] <= 1.0 + 1e-9


def test_backstepping_lyapunov_function_does_not_grow():
    controller = BacksteppingTracker()
    reference = Reference(straight(10.0), v_max=0.3)
    plant = ActuatorPlant(limits=Limits(a_max=100.0, alpha_max=100.0), tau=1e-3, pose=(0.5, 0.4, 0.3))
    hint, values = 0.5, []
    for _ in range(200):
        p = plant.pose
        hint = reference.project(p[0], p[1], hint)
        rx, ry, rtheta, _ = reference.state(hint)
        c, s = math.cos(p[2]), math.sin(p[2])
        e_x = c * (rx - p[0]) + s * (ry - p[1])
        e_y = -s * (rx - p[0]) + c * (ry - p[1])
        e_th = rtheta - p[2]
        values.append(0.5 * (e_x ** 2 + e_y ** 2) + e_th ** 2 / (2 * controller.gamma))
        plant.step(controller.command(tuple(p), tuple(plant.velocity), reference, hint, DT), DT)
    assert values[-1] < 0.05 * values[0]
    assert all(b <= a + 1e-3 for a, b in zip(values, values[1:]))


def test_feedback_linearization_brings_the_axle_to_the_path_end():
    reference = Reference(straight(3.0), v_max=0.3)
    trace = simulate(make_controller('feedback_linearization'), reference, (0.0, 0.0, 0.0), duration=25.0)
    x, y, _ = trace[-1][5]
    assert math.hypot(x - 3.0, y) < 0.08


def test_tracking_laws_are_served_with_one_supervisor():
    assert set(CONTROLLERS) <= nav2_adapters.LOCAL_LAWS
    limits = nav2_adapters.TrackingLimits()
    assert nav2_adapters.supervise_heading(0.3, limits) is None
    v, w = nav2_adapters.supervise_heading(2.0, limits)
    assert v == 0.0 and w < 0  # heading left of the path -> turn right
