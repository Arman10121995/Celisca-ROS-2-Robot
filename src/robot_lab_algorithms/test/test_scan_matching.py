"""R7.3 scan-to-map localizers (robot_lab_algorithms.scan_matching)."""
import math

import numpy as np
import pytest

from robot_lab_algorithms.scan_matching import (
    MATCHERS, OccupancyMap, ScanLocalizer, compose, relative, transform)

RES = 0.05


def room():
    """8 x 6 m room with 0.2 m walls, a pillar and an L-shaped partition."""
    w, h = 180, 140
    grid = np.zeros((h, w), dtype=np.int16)
    grid[:4, :] = grid[-4:, :] = grid[:, :4] = grid[:, -4:] = 100
    grid[60:80, 50:70] = 100            # pillar
    grid[90:94, 100:160] = 100          # partition
    grid[30:94, 156:160] = 100
    return OccupancyMap(grid.ravel(), w, h, RES, (0.0, 0.0))


def scan(occ, pose, rng, beams=180):
    angles = np.linspace(-math.pi, math.pi, beams, endpoint=False)
    distances = np.arange(0.05, 10.0, 0.01)
    a = pose[2] + angles
    xs, ys = pose[0] + np.outer(np.cos(a), distances), pose[1] + np.outer(np.sin(a), distances)
    cols = np.clip(((xs - occ.origin[0]) / RES).astype(int), 0, occ.grid.shape[1] - 1)
    rows = np.clip(((ys - occ.origin[1]) / RES).astype(int), 0, occ.grid.shape[0] - 1)
    hit = occ.grid[rows, cols] >= 65
    r = distances[hit.argmax(axis=1)] + rng.normal(0, 0.01, beams)
    return np.column_stack([r * np.cos(angles), r * np.sin(angles)])


def test_compose_and_relative_are_inverse():
    a, b = np.array([1.0, 2.0, 0.5]), np.array([-0.3, 4.0, -2.0])
    assert compose(a, relative(a, b)) == pytest.approx(b)


@pytest.mark.parametrize('name', [n for n in MATCHERS if n != 'dead_reckoning'])
def test_matchers_recover_a_small_pose_error(name):
    occ = room()
    rng = np.random.default_rng(0)
    truth = np.array([3.0, 2.5, 0.4])
    points = scan(occ, truth, rng)
    matcher = MATCHERS[name](occ)
    estimate = matcher.match(points, truth + np.array([0.08, -0.06, 0.04]))
    assert np.hypot(*(estimate[:2] - truth[:2])) < 0.04
    assert abs(estimate[2] - truth[2]) < math.radians(1.5)


def test_scan_localizer_removes_odometry_drift():
    occ = room()
    rng = np.random.default_rng(1)
    poses = [np.array([2.0 + 0.05 * k, 2.0 + 0.02 * k, 0.3]) for k in range(60)]
    drift = {name: ScanLocalizer(name, occ, poses[0]) for name in ('dead_reckoning', 'icp_point_to_line')}
    for a, b in zip(poses[:-1], poses[1:]):
        delta = relative(a, b) * np.array([1.05, 1.05, 1.0]) + np.array([0.0, 0.0, 0.004])
        points = scan(occ, b, rng)
        for localizer in drift.values():
            localizer.step(delta, points)
    final = poses[-1]
    dead = np.hypot(*(drift['dead_reckoning'].pose[:2] - final[:2]))
    icp = np.hypot(*(drift['icp_point_to_line'].pose[:2] - final[:2]))
    assert dead > 0.15 and icp < 0.05


def test_surface_cells_only():
    occ = room()
    # The 0.2 m walls are 4 cells thick: only the inner face is kept.
    assert np.all(transform(np.zeros((1, 2)), np.zeros(3)) == 0)
    assert occ.points[:, 1].min() >= 3 * RES
