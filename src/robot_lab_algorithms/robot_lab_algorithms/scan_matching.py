"""Scan-to-map localization (R7.3).

Each localizer tracks a planar pose (x, y, theta) in a known occupancy map:
it predicts with the odometry increment, then aligns the 2D laser scan to
the map.  All share :class:`OccupancyMap` (occupied-cell points, a KD-tree,
a distance field and an NDT grid built once from ``nav_msgs/OccupancyGrid``
data).

* :class:`IcpPointToPoint` - iterative closest point with the closed-form
  2D rigid alignment (SVD) and a shrinking correspondence gate.
* :class:`IcpPointToLine` - Gauss-Newton on point-to-line distances using
  map normals (Censi's PLICP cost, without its closed form).
* :class:`NdtMatcher` - 2D normal distributions transform: per-cell
  Gaussians of the map points, Newton steps on the summed score (Biber and
  Strasser 2003).
* :class:`CorrelativeMatcher` - exhaustive coarse-to-fine search of a pose
  window on a Gaussian likelihood field (Olson 2009).
* :class:`DeadReckoning` - the odometry alone, as the baseline.

``ScanLocalizer`` wraps any of them: ``step(odometry_delta, scan_points)``.
"""
from __future__ import annotations

import math

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree


def wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def compose(pose, delta):
    """pose (+) delta, delta in the pose's frame."""
    x, y, th = pose
    dx, dy, dth = delta
    c, s = math.cos(th), math.sin(th)
    return np.array([x + c * dx - s * dy, y + s * dx + c * dy, wrap(th + dth)])


def relative(a, b):
    """Delta that takes pose a to pose b, in a's frame."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.array([c * dx + s * dy, -s * dx + c * dy, wrap(b[2] - a[2])])


def transform(points, pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return points @ np.array([[c, s], [-s, c]]) + pose[:2]


def scan_points(ranges, angle_min, angle_increment, range_min=0.1, range_max=12.0, step=2,
                sensor_offset=(0.0, 0.0, 0.0)):
    """Valid beams as points in the robot frame (every ``step``-th beam)."""
    r = np.asarray(ranges, dtype=float)[::step]
    a = angle_min + angle_increment * np.arange(len(ranges))[::step] + sensor_offset[2]
    ok = np.isfinite(r) & (r > range_min) & (r < range_max * 0.99)
    pts = np.column_stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])])
    return pts + np.array(sensor_offset[:2])


class OccupancyMap:
    def __init__(self, data, width, height, resolution, origin, occupied=65, ndt_cell=0.5):
        grid = np.asarray(data, dtype=np.int16).reshape(height, width)
        self.grid, self.resolution = grid, float(resolution)
        self.origin = np.array(origin[:2], dtype=float)
        # Only occupied cells that touch free space: the surfaces a laser can
        # see.  Interior cells of thick walls would bias every matcher inward.
        occupied_mask = grid >= occupied
        free_mask = (grid >= 0) & (grid < occupied)
        touches_free = np.zeros_like(occupied_mask)
        touches_free[1:, :] |= free_mask[:-1, :]
        touches_free[:-1, :] |= free_mask[1:, :]
        touches_free[:, 1:] |= free_mask[:, :-1]
        touches_free[:, :-1] |= free_mask[:, 1:]
        rows, cols = np.nonzero(occupied_mask & touches_free)
        self.points = np.column_stack([self.origin[0] + (cols + 0.5) * resolution,
                                       self.origin[1] + (rows + 0.5) * resolution])
        self.tree = cKDTree(self.points)
        free = grid < occupied
        self.distance = ndimage.distance_transform_edt(free) * resolution
        self._normals = None
        self._ndt = None
        self.ndt_cell = ndt_cell

    @classmethod
    def from_msg(cls, msg, **kwargs):
        info = msg.info
        return cls(msg.data, info.width, info.height, info.resolution,
                   (info.origin.position.x, info.origin.position.y), **kwargs)

    def distance_at(self, xy):
        cols = np.floor((xy[:, 0] - self.origin[0]) / self.resolution).astype(int)
        rows = np.floor((xy[:, 1] - self.origin[1]) / self.resolution).astype(int)
        inside = (rows >= 0) & (rows < self.grid.shape[0]) & (cols >= 0) & (cols < self.grid.shape[1])
        out = np.full(len(xy), 1.0)
        out[inside] = self.distance[rows[inside], cols[inside]]
        return out

    def normals(self):
        if self._normals is None:
            _, idx = self.tree.query(self.points, k=min(8, len(self.points)))
            normals = np.zeros_like(self.points)
            for i, neighbours in enumerate(idx):
                q = self.points[neighbours] - self.points[neighbours].mean(axis=0)
                _, vectors = np.linalg.eigh(q.T @ q)
                normals[i] = vectors[:, 0]
            self._normals = normals
        return self._normals

    def ndt(self):
        """{cell: (mean, inverse covariance)} on an ndt_cell grid."""
        if self._ndt is None:
            cells = {}
            keys = np.floor(self.points / self.ndt_cell).astype(int)
            for key in np.unique(keys, axis=0):
                pts = self.points[(keys == key).all(axis=1)]
                if len(pts) < 3:
                    continue
                cov = np.cov(pts.T) + np.eye(2) * (self.resolution ** 2)
                cells[tuple(key)] = (pts.mean(axis=0), np.linalg.inv(cov))
            self._ndt = cells
        return self._ndt


class _Matcher:
    name = 'matcher'

    def __init__(self, occupancy):
        self.map = occupancy

    def match(self, points, guess):  # pragma: no cover - abstract
        raise NotImplementedError


class DeadReckoning(_Matcher):
    name = 'dead_reckoning'

    def match(self, points, guess):
        return guess


class IcpPointToPoint(_Matcher):
    name = 'icp_point_to_point'

    def __init__(self, occupancy, iterations=25, gate=0.5, min_gate=0.1, tolerance=1e-4):
        super().__init__(occupancy)
        self.iterations, self.gate, self.min_gate, self.tolerance = iterations, gate, min_gate, tolerance

    def match(self, points, guess):
        pose = np.array(guess, dtype=float)
        gate = self.gate
        for _ in range(self.iterations):
            world = transform(points, pose)
            d, idx = self.map.tree.query(world, distance_upper_bound=gate)
            ok = np.isfinite(d)
            if ok.sum() < 10:
                break
            src, dst = world[ok], self.map.points[idx[ok]]
            ms, md = src.mean(axis=0), dst.mean(axis=0)
            H = (src - ms).T @ (dst - md)
            U, _, Vt = np.linalg.svd(H)
            R = Vt.T @ U.T
            if np.linalg.det(R) < 0:
                Vt[1] *= -1
                R = Vt.T @ U.T
            dth = math.atan2(R[1, 0], R[0, 0])
            t = md - R @ ms
            new = np.array([R[0, 0] * pose[0] + R[0, 1] * pose[1] + t[0],
                            R[1, 0] * pose[0] + R[1, 1] * pose[1] + t[1], wrap(pose[2] + dth)])
            moved = float(np.linalg.norm(new[:2] - pose[:2]))
            pose = new
            gate = max(self.min_gate, gate * 0.8)
            if abs(dth) < self.tolerance and moved < self.tolerance:
                break
        return pose


class IcpPointToLine(_Matcher):
    name = 'icp_point_to_line'

    def __init__(self, occupancy, iterations=20, gate=0.5, min_gate=0.1):
        super().__init__(occupancy)
        self.iterations, self.gate, self.min_gate = iterations, gate, min_gate
        self.normals = occupancy.normals()

    def match(self, points, guess):
        pose = np.array(guess, dtype=float)
        gate = self.gate
        for _ in range(self.iterations):
            c, s = math.cos(pose[2]), math.sin(pose[2])
            world = transform(points, pose)
            d, idx = self.map.tree.query(world, distance_upper_bound=gate)
            ok = np.isfinite(d)
            if ok.sum() < 10:
                break
            q, n, p = self.map.points[idx[ok]], self.normals[idx[ok]], points[ok]
            residual = np.sum(n * (world[ok] - q), axis=1)
            dworld_dth = np.column_stack([-s * p[:, 0] - c * p[:, 1], c * p[:, 0] - s * p[:, 1]])
            J = np.column_stack([n[:, 0], n[:, 1], np.sum(n * dworld_dth, axis=1)])
            weights = 1.0 / (1.0 + (residual / 0.05) ** 2)  # Cauchy-like robust weights
            A = J.T @ (J * weights[:, None]) + np.eye(3) * 1e-6
            step = -np.linalg.solve(A, J.T @ (residual * weights))
            pose = np.array([pose[0] + step[0], pose[1] + step[1], wrap(pose[2] + step[2])])
            gate = max(self.min_gate, gate * 0.8)
            if np.linalg.norm(step[:2]) < 1e-4 and abs(step[2]) < 1e-4:
                break
        return pose


class NdtMatcher(_Matcher):
    """2D NDT: each scan point scores against the Gaussian of its nearest map cell.

    Score = sum exp(-d' S^-1 d / 2).  Iteratively reweighted Gauss-Newton:
    with weights w = exp(-d' S^-1 d / 2) held per iteration, minimise
    sum w d' S^-1 d; a step is kept only if it raises the score (halved
    otherwise).
    """

    name = 'ndt'

    def __init__(self, occupancy, iterations=20, association=0.75):
        super().__init__(occupancy)
        self.iterations = iterations
        cells = occupancy.ndt()
        self.means = np.array([m for m, _ in cells.values()])
        self.inverses = np.array([inv for _, inv in cells.values()])
        self.tree = cKDTree(self.means)
        self.association = association

    def _terms(self, points, pose):
        world = transform(points, pose)
        dist, idx = self.tree.query(world, distance_upper_bound=self.association)
        ok = np.isfinite(dist)
        d = world[ok] - self.means[idx[ok]]
        inv = self.inverses[idx[ok]]
        q = np.einsum('ni,nij,nj->n', d, inv, d)
        return ok, d, inv, q

    def score(self, points, pose):
        _, _, _, q = self._terms(points, pose)
        return float(np.sum(np.exp(-0.5 * q)))

    def match(self, points, guess):
        pose = np.array(guess, dtype=float)
        current = self.score(points, pose)
        for _ in range(self.iterations):
            c, s = math.cos(pose[2]), math.sin(pose[2])
            ok, d, inv, q = self._terms(points, pose)
            if ok.sum() < 10:
                break
            p = points[ok]
            w = np.maximum(np.exp(-0.5 * q), 1e-3)
            J = np.zeros((len(p), 2, 3))
            J[:, 0, 0] = J[:, 1, 1] = 1.0
            J[:, 0, 2] = -s * p[:, 0] - c * p[:, 1]
            J[:, 1, 2] = c * p[:, 0] - s * p[:, 1]
            JtS = np.einsum('nki,nkj->nij', J, inv)              # J' S^-1, (n, 3, 2)
            H = np.einsum('n,nij,njk->ik', w, JtS, J) + np.eye(3) * 1e-6
            g = np.einsum('n,nij,nj->i', w, JtS, d)
            step = -np.linalg.solve(H, g)
            accepted = False
            for _ in range(5):
                trial = np.array([pose[0] + step[0], pose[1] + step[1], wrap(pose[2] + step[2])])
                value = self.score(points, trial)
                if value >= current:
                    pose, current, accepted = trial, value, True
                    break
                step = step * 0.5
            if not accepted or (np.linalg.norm(step[:2]) < 1e-4 and abs(step[2]) < 1e-4):
                break
        return pose


class CorrelativeMatcher(_Matcher):
    name = 'correlative'

    def __init__(self, occupancy, window=0.3, angle_window=math.radians(10), sigma=0.05):
        super().__init__(occupancy)
        self.window, self.angle_window, self.sigma = window, angle_window, sigma

    def _search(self, points, center, half, step, half_angle, angle_step):
        offsets = np.arange(-half, half + 1e-9, step)
        grid = np.array([(dx, dy) for dx in offsets for dy in offsets])
        best, best_score = center, -1.0
        for dth in np.arange(-half_angle, half_angle + 1e-9, angle_step):
            th = center[2] + dth
            c, s = math.cos(th), math.sin(th)
            rotated = points @ np.array([[c, s], [-s, c]]) + center[:2]
            xy = (rotated[None, :, :] + grid[:, None, :]).reshape(-1, 2)
            likelihood = np.exp(-0.5 * (self.map.distance_at(xy) / self.sigma) ** 2)
            scores = likelihood.reshape(len(grid), len(points)).mean(axis=1)
            k = int(np.argmax(scores))
            if scores[k] > best_score:
                best_score = float(scores[k])
                best = np.array([center[0] + grid[k, 0], center[1] + grid[k, 1], wrap(th)])
        return best

    def match(self, points, guess):
        pose = np.array(guess, dtype=float)
        sub = points[::2] if len(points) > 90 else points
        pose = self._search(sub, pose, self.window, 0.1, self.angle_window, math.radians(2.5))
        pose = self._search(sub, pose, 0.1, 0.025, math.radians(2.5), math.radians(0.5))
        return pose


MATCHERS = {
    'dead_reckoning': DeadReckoning,
    'icp_point_to_point': IcpPointToPoint,
    'icp_point_to_line': IcpPointToLine,
    'ndt': NdtMatcher,
    'correlative': CorrelativeMatcher,
}


class ScanLocalizer:
    """Predict with odometry increments, correct by scan-to-map alignment."""

    def __init__(self, method, occupancy, initial_pose, max_correction=0.5):
        self.matcher = MATCHERS[method](occupancy)
        self.pose = np.array(initial_pose, dtype=float)
        self.max_correction = max_correction
        self.rejected = 0

    def step(self, odometry_delta, points):
        guess = compose(self.pose, odometry_delta)
        corrected = self.matcher.match(points, guess) if len(points) >= 10 else guess
        # A match that jumps further than the odometry could be wrong is refused.
        if np.linalg.norm(corrected[:2] - guess[:2]) > self.max_correction:
            corrected = guess
            self.rejected += 1
        self.pose = corrected
        return self.pose
