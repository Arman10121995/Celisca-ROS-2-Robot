"""Moving-obstacle tracking and prediction for drone routes (ROADMAP R5.11).

Each depth frame's points (already in the FCU map frame) are grouped into
clusters on a voxel grid; clusters are associated with tracks by gated
nearest-neighbour matching, and each track runs a constant-velocity Kalman
filter. A track is *moving* once it has been seen in enough frames with a
speed above ``MOVING_SPEED``. Moving tracks become:

* swept capsules (``PredictedObstacles``) that the 3D A* planner treats as
  obstacles over the prediction horizon (conservative, time-free), and
* time-indexed positions that ``time_conflict`` compares with the drone's own
  position along its route at the same time.

Static clusters are not predicted; they stay in the observed voxel layer.
This is constant-velocity prediction, not intent prediction: a sudden turn
is caught only by the next frames.
"""
import itertools
import math

import numpy as np

MOVING_SPEED = .3          # m/s; slower clusters are treated as static
MIN_FRAMES = 3
MAX_MISSES = 4
HORIZON_S = 3.
GATE_M = 1.2


def cluster_points(points, voxel=.25, min_points=12, max_clusters=40, floor_z=None):
    """[(centroid, radius, count)] of connected voxel groups (26-neighbour)."""
    from scipy import ndimage
    points = np.asarray(points, dtype=float)
    if floor_z is not None:
        points = points[points[:, 2] > floor_z + .2]       # the ground is not an object
    if len(points) < min_points:
        return []
    cells = np.floor(points / voxel).astype(np.int64)
    origin = cells.min(axis=0)
    shape = cells.max(axis=0) - origin + 1
    if np.prod(shape) > 4e6:
        raise ValueError('depth cluster grid too large; limit the depth range')
    grid = np.zeros(shape, dtype=bool)
    local = cells - origin
    grid[tuple(local.T)] = True
    labels, count = ndimage.label(grid, structure=np.ones((3, 3, 3)))
    point_labels = labels[tuple(local.T)]
    clusters = []
    for label in range(1, count + 1):
        members = points[point_labels == label]
        if len(members) < min_points:
            continue
        centroid = members.mean(axis=0)
        radius = float(np.max(np.linalg.norm(members - centroid, axis=1))) + voxel / 2
        clusters.append((centroid, radius, len(members)))
    clusters.sort(key=lambda c: -c[2])
    return clusters[:max_clusters]


class Track:
    _ids = itertools.count(1)

    def __init__(self, centroid, radius, stamp):
        self.id = next(Track._ids)
        self.x = np.concatenate([centroid, np.zeros(3)])
        self.P = np.diag([.1, .1, .1, 4., 4., 4.])
        self.radius, self.stamp, self.frames, self.misses = radius, stamp, 1, 0

    def predict(self, stamp, q=1.5):
        dt = max(0., stamp - self.stamp)
        F = np.eye(6)
        F[:3, 3:] = np.eye(3) * dt
        Q = np.zeros((6, 6))
        Q[:3, :3] = np.eye(3) * q * dt ** 3 / 3
        Q[:3, 3:] = Q[3:, :3] = np.eye(3) * q * dt ** 2 / 2
        Q[3:, 3:] = np.eye(3) * q * dt
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self.stamp = stamp

    def correct(self, centroid, radius, r=.05):
        H = np.hstack([np.eye(3), np.zeros((3, 3))])
        S = H @ self.P @ H.T + np.eye(3) * r
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ (np.asarray(centroid) - H @ self.x)
        self.P = (np.eye(6) - K @ H) @ self.P
        self.radius = .7 * self.radius + .3 * radius
        self.frames += 1
        self.misses = 0

    @property
    def position(self):
        return self.x[:3]

    @property
    def velocity(self):
        return self.x[3:]

    @property
    def speed(self):
        return float(np.linalg.norm(self.velocity[:2]))

    def moving(self):
        return self.frames >= MIN_FRAMES and self.speed >= MOVING_SPEED

    def at(self, t):
        """Predicted centre ``t`` seconds after the last update."""
        return self.position + self.velocity * t

    def state(self):
        return dict(id=self.id, position=[round(float(v), 3) for v in self.position],
                    velocity=[round(float(v), 3) for v in self.velocity], radius=round(self.radius, 3),
                    frames=self.frames, moving=self.moving())


class Tracker:
    def __init__(self):
        self.tracks = []

    def update(self, clusters, stamp):
        for track in self.tracks:
            track.predict(stamp)
        unmatched = list(range(len(clusters)))
        # Greedy nearest-neighbour association inside a gate that grows with the track's speed.
        pairs = sorted(((np.linalg.norm(track.position - clusters[i][0]), t, i)
                        for t, track in enumerate(self.tracks) for i in unmatched), key=lambda p: p[0])
        used_tracks = set()
        for distance, t, i in pairs:
            if t in used_tracks or i not in unmatched:
                continue
            if distance > GATE_M + self.tracks[t].speed * .5:
                continue
            self.tracks[t].correct(clusters[i][0], clusters[i][1])
            used_tracks.add(t)
            unmatched.remove(i)
        for t, track in enumerate(self.tracks):
            if t not in used_tracks:
                track.misses += 1
        self.tracks = [track for track in self.tracks if track.misses <= MAX_MISSES]
        for i in unmatched:
            self.tracks.append(Track(clusters[i][0], clusters[i][1], stamp))
        return self.tracks

    def moving(self):
        return [track for track in self.tracks if track.moving()]


def _segment_distance(p, a, b):
    ab = b - a
    t = 0. if not ab.any() else float(np.clip(np.dot(p - a, ab) / np.dot(ab, ab), 0., 1.))
    return float(np.linalg.norm(p - (a + t * ab)))


class PredictedObstacles:
    """Swept capsules of moving tracks over the horizon, as a planning layer."""

    def __init__(self, tracks, horizon=HORIZON_S, margin=.3):
        self.capsules = [(track.position.copy(), track.at(horizon), track.radius + margin) for track in tracks]

    def clearance(self, position):
        p = np.asarray(position, dtype=float)
        if not self.capsules:
            return math.inf
        return min(_segment_distance(p, a, b) - radius for a, b, radius in self.capsules)

    def segment_clear(self, start, end, radius=.7):
        a, b = np.asarray(start, dtype=float), np.asarray(end, dtype=float)
        steps = max(2, int(math.ceil(np.linalg.norm(b - a) / .1)))
        return all(self.clearance(a + (b - a) * s / steps) > radius for s in range(steps + 1))


def time_conflict(position, route, speed, tracks, radius=.7, horizon=HORIZON_S, step=.1):
    """First (time, track) at which the drone, flying ``route`` at ``speed`` from ``position``,
    comes within ``radius`` + track radius of a moving track's predicted centre; None if clear."""
    points = [np.asarray(position, dtype=float)] + [np.asarray(p, dtype=float) for p in route]
    if not tracks or speed <= 0:
        return None
    legs = [(a, b, np.linalg.norm(b - a)) for a, b in zip(points[:-1], points[1:])]
    for k in range(int(horizon / step) + 1):
        t = k * step
        travelled, drone = speed * t, points[-1]
        for a, b, length in legs:
            if travelled <= length:
                drone = a + (b - a) * (travelled / length if length else 0.)
                break
            travelled -= length
        for track in tracks:
            if np.linalg.norm(drone - track.at(t)) < radius + track.radius:
                return t, track
    return None
