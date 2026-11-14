"""Seeded sensor-fault relay: repeatable traces, windows, delay/dropout, truth refused (R6.2)."""
import copy
import math
from types import SimpleNamespace

import numpy as np
import pytest

from robot_lab_utils.sensor_fault_relay import Relay, validate


def scan(n=360, value=2.):
    return SimpleNamespace(ranges=[value] * (n - 4) + [math.inf] * 4, range_min=.12, range_max=12.)


SPEC = dict(input='/scan_clean', output='/scan', type='LaserScan', faults=[
    dict(kind='noise', std=.05, start_s=1.), dict(kind='dropout', probability=.2),
    dict(kind='occlusion', fraction=.25), dict(kind='outlier', probability=.02, scale=3.),
    dict(kind='delay', seconds=.3, start_s=2.)])


def run(seed):
    relay, rows = Relay(SPEC, seed, 0), []
    for k in range(50):
        rows.append(relay.receive(scan(), k * .1))
    return rows, relay


def test_same_seed_repeats_and_another_seed_differs():
    first, _ = run(7)
    again, _ = run(7)
    other, _ = run(8)
    assert first == again
    assert [r.get('digest') for r in first] != [r.get('digest') for r in other]


def test_faults_respect_windows_occlusion_sector_and_inf_rays():
    relay = Relay(dict(SPEC, faults=[dict(kind='noise', std=.05, start_s=1.), dict(kind='occlusion', fraction=.25)]), 1, 0)
    early = scan()
    relay.receive(early, .5)
    ranges = np.array(early.ranges)
    finite = ranges[np.isfinite(ranges)]
    assert np.allclose(finite, 2.)                               # noise window not open yet
    assert np.sum(np.isinf(ranges)) == 4 + 90                    # occluded sector returns nothing; inf rays kept
    late = scan()
    relay.receive(late, 1.5)
    values = np.array(late.ranges)
    assert np.std(values[np.isfinite(values)]) > .02


def test_dropout_and_delay_release_on_scenario_time():
    rows, relay = run(3)
    dropped = sum(r['dropped'] for r in rows)
    assert 3 <= dropped <= 20
    assert all(r['release_t'] == pytest.approx(r['t'] + .3) for r in rows if not r['dropped'] and r['t'] >= 2.)
    relay2 = Relay(dict(SPEC, faults=[dict(kind='delay', seconds=.3)]), 0, 0)
    relay2.receive(scan(), 1.)
    assert relay2.due(1.2) == [] and len(relay2.due(1.31)) == 1


def test_truth_topics_and_bad_specs_are_refused():
    with pytest.raises(ValueError, match='Truth'):
        validate(dict(relays=[dict(SPEC, input='/odom/ground_truth', output='/odom', type='Odometry')]))
    with pytest.raises(ValueError):
        validate(dict(relays=[dict(SPEC, faults=[dict(kind='teleport')])]))
    with pytest.raises(ValueError):
        validate(dict(relays=[dict(SPEC, type='PointCloud2')]))


def test_imu_and_depth_image_faults():
    imu_relay = Relay(dict(input='/imu_clean', output='/imu/out', type='Imu', faults=[dict(kind='bias', value=.1)]), 0, 0)
    vector = lambda: SimpleNamespace(x=0., y=0., z=0.)
    imu = SimpleNamespace(angular_velocity=vector(), linear_acceleration=vector())
    imu_relay.receive(imu, 0.)
    assert imu.angular_velocity.x == pytest.approx(.1) and imu.linear_acceleration.z == pytest.approx(.1)
    depth = SimpleNamespace(encoding='32FC1', height=4, width=5, data=np.ones(20, dtype=np.float32).tobytes())
    Relay(dict(input='/d_clean', output='/d', type='Image', faults=[dict(kind='occlusion', fraction=.5)]), 0, 0).receive(depth, 0.)
    values = np.frombuffer(depth.data, dtype=np.float32).reshape(4, 5)
    assert np.isnan(values).sum() == 10


@pytest.mark.parametrize('relays',[
    [dict(SPEC,input='scan_clean')],
    [SPEC,dict(SPEC)],
    [SPEC,dict(SPEC,input='/scan',output='/scan_clean')],
])
def test_ambiguous_duplicate_and_feedback_topic_routes_are_refused(relays):
    with pytest.raises(ValueError):validate(dict(relays=copy.deepcopy(relays)))
