"""R7.4 state estimators on noisy position fixes (robot_lab_algorithms.tracking_filters)."""
import math

import numpy as np
import pytest

from robot_lab_algorithms.tracking_filters import ESTIMATORS, ImmFilter, canonical, ctrv, wrap

DT = 0.1


def track(seconds=60.0, turn=0.0, speed=0.5, seed=0, noise=0.3):
    rng = np.random.default_rng(seed)
    state = np.array([0.0, 0.0, 0.3, speed, turn])
    rows = []
    for _ in range(int(seconds / DT)):
        state = ctrv(state, DT)
        rows.append((state.copy(), state[:2] + rng.normal(0, noise, 2)))
    return rows


@pytest.mark.parametrize('name', sorted(ESTIMATORS))
def test_estimators_beat_the_raw_fixes_and_find_speed_and_heading(name):
    rows = track(turn=0.2)
    flt = ESTIMATORS[name]()
    estimates = []
    for _, z in rows:
        flt.predict(DT)
        flt.update(z)
        estimates.append(flt.estimate())
    truth = np.array([r[0] for r in rows])[100:]
    fixes = np.array([r[1] for r in rows])[100:]
    e = np.array(estimates)[100:]
    raw = np.sqrt(np.mean(np.sum((fixes - truth[:, :2]) ** 2, axis=1)))
    position = np.sqrt(np.mean(np.sum((e[:, :2] - truth[:, :2]) ** 2, axis=1)))
    assert position < 0.6 * raw
    assert np.median(np.abs(e[:, 3] - 0.5)) < 0.1
    assert np.median([abs(wrap(a - b)) for a, b in zip(e[:, 2], truth[:, 2])]) < math.radians(15)


def test_canonical_folds_reverse_speed():
    x, y, th, v, w = canonical((1.0, 2.0, 0.1, -0.5, 0.2))
    assert v == pytest.approx(0.5) and th == pytest.approx(wrap(0.1 + math.pi))


def test_imm_moves_toward_the_turning_model_in_a_turn():
    flt = ImmFilter()
    for _, z in track(seconds=20.0, turn=0.0, seed=1):
        flt.predict(DT)
        flt.update(z)
    straight = flt.mu[0]
    for _, z in track(seconds=20.0, turn=0.6, seed=2):
        flt.predict(DT)
        flt.update(z)
    assert flt.mu[1] > 1.0 - straight or flt.mu[1] > 0.5
