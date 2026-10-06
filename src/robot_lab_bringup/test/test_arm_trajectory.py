"""Counterexamples for target limits, timing and cubic velocity bounds."""
import numpy as np
import pytest

from robot_lab_utils.arm_trajectory import position_trajectory, retime_collision_checked_path


NAMES = ['joint'+str(i) for i in range(1, 8)]
LIMITS = np.tile([-2., 2.], (7, 1))


def test_joint_order_and_rest_to_rest_velocity_bound():
    endpoint = np.arange(7)/10.
    plan = position_trajectory(list(reversed(NAMES)), NAMES,
                               [(2., list(reversed(endpoint)))], np.zeros(7), LIMITS)
    np.testing.assert_allclose(plan.sample(2.)[0], endpoint)
    np.testing.assert_allclose(plan.sample(1.)[0], endpoint/2)
    np.testing.assert_allclose(plan.sample(0.)[1], np.zeros(7))
    np.testing.assert_allclose(plan.sample(2.)[1], np.zeros(7))
    np.testing.assert_allclose(plan.sample(3.)[0], endpoint)
    assert max(np.max(np.abs(plan.sample(t)[1])) for t in np.linspace(0, 2, 101)) <= .5


@pytest.mark.parametrize('names,points', [
    (NAMES[:-1], [(1., [0.]*7)]),
    (NAMES[:-1]+['joint1'], [(1., [0.]*7)]),
    (NAMES, []),
    (NAMES, [(1., [0.]*6)]),
    (NAMES, [(0., [0.]*7)]),
    (NAMES, [(1., [0.]*7), (1., [0.]*7)]),
    (NAMES, [(float('nan'), [0.]*7)]),
    (NAMES, [(1., [float('nan')]+[0.]*6)]),
    (NAMES, [(16., [0.]*7)]),
    (NAMES, [(8., [1.998]+[0.]*6)]),
    # Average velocity is .4, but a rest-to-rest cubic peaks at .6.
    (NAMES, [(1., [.4]+[0.]*6)]),
    # A safe later segment does not forgive an unsafe initial jump.
    (NAMES, [(.1, [.1]+[0.]*6), (2., [.1]+[0.]*6)]),
])
def test_rejects_unbounded_or_ambiguous_trajectories(names, points):
    with pytest.raises(ValueError):
        position_trajectory(names, NAMES, points, np.zeros(7), LIMITS)


def test_retimed_path_preserves_checked_edges_with_bounded_cubic_speed():
    first = np.array([.1, .2]+[0.]*5)
    second = np.array([.2, -.1]+[0.]*5)
    points = retime_collision_checked_path(list(reversed(NAMES)), NAMES,
        [list(reversed(q)) for q in [np.zeros(7), first, second]], np.zeros(7), LIMITS)
    plan = position_trajectory(NAMES, NAMES, points, np.zeros(7), LIMITS)
    np.testing.assert_allclose(plan.sample(points[0][0])[0], first)
    np.testing.assert_allclose(plan.sample(points[-1][0])[0], second)
    # Each common cubic scalar stays on its collision-checked straight edge.
    midpoint = plan.sample((points[0][0]+points[1][0])/2)[0]
    np.testing.assert_allclose(midpoint, (first+second)/2)
    assert max(np.max(np.abs(plan.sample(t)[1])) for t in np.linspace(0, points[-1][0], 301)) <= .350001


def test_retime_rejects_a_path_that_cannot_fit_the_native_time_contract():
    with pytest.raises(ValueError, match='15 seconds'):
        retime_collision_checked_path(NAMES, NAMES,
            [[1.9]*7, [-1.9]*7, [1.9]*7], np.zeros(7), LIMITS)


def test_retime_accepts_the_json_measured_limits_from_the_gui():
    import json
    measured = json.loads(json.dumps(dict(positions=[0.]*7, limits=LIMITS.tolist())))
    points = retime_collision_checked_path(NAMES, NAMES,
        [[0.]*7, [.1]+[0.]*6], measured['positions'], measured['limits'])
    assert points[-1][1] == [.1]+[0.]*6
    assert points[-1][0] >= 1.5*.1/.35
