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
# A moving object that leaves the camera's view is still there: moving tracks coast on their prediction
# for this long, with an uncertainty radius growing by COAST_SPREAD x speed x time unseen. A forward
# camera loses crossing traffic at the drone's own position (2026-11-13).
COAST_S = 4.
COAST_SPREAD = .3
HORIZON_S = 3.
GATE_M = 1.2
# Larger clusters are scenery seen partially: their centroid shifts with the viewpoint (2026-11-13).
MAX_OBJECT_RADIUS = 1.5
# A camera at drone height sees only part of a person (tracks 0.5-0.8 m tall, 2026-11-13), and a drone
# then passed 0.4 m over a walker's head. An object that stands on the ground (its seen bottom within
# GROUNDED_M of it) is assumed to reach at least MIN_STANDING_HEIGHT_M: a stated prior, not a measurement.
GROUNDED_M = .8
MIN_STANDING_HEIGHT_M = 2.


def cluster_points(points, voxel=.25, min_points=12, max_clusters=40, floor_z=None):
    """[(centroid, radius, count, z_low, z_high)] of connected voxel groups (26-neighbour).

    ``radius`` is horizontal and ``z_low``/``z_high`` the vertical extent: a person is a column from
    the ground to the head, not a sphere around its centroid (a drone passed 0.3 m over a walker's head
    when tracks were spheres, 2026-11-13).
    """
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
        radius = float(np.max(np.linalg.norm(members[:, :2] - centroid[:2], axis=1))) + voxel / 2
        clusters.append((centroid, radius, len(members), float(members[:, 2].min()) - voxel / 2,
                         float(members[:, 2].max()) + voxel / 2))
    clusters.sort(key=lambda c: -c[2])
    return clusters[:max_clusters]


class Track:
    _ids = itertools.count(1)

    def __init__(self, centroid, radius, stamp, z_low=None, z_high=None):
        self.id = next(Track._ids)
        self.z_low = float(centroid[2] - radius) if z_low is None else z_low
        self.z_high = float(centroid[2] + radius) if z_high is None else z_high
        self.x = np.concatenate([centroid, np.zeros(3)])
        self.P = np.diag([.1, .1, .1, 4., 4., 4.])
        self.radius, self.stamp, self.frames, self.misses = radius, stamp, 1, 0
        self.seen = stamp

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

    def correct(self, centroid, radius, r=.05, z_low=None, z_high=None):
        if z_low is not None:
            self.z_low, self.z_high = min(self.z_low, z_low), max(self.z_high, z_high)   # the extent seen so far
        H = np.hstack([np.eye(3), np.zeros((3, 3))])
        S = H @ self.P @ H.T + np.eye(3) * r
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ (np.asarray(centroid) - H @ self.x)
        self.P = (np.eye(6) - K @ H) @ self.P
        self.radius = .7 * self.radius + .3 * radius
        self.frames += 1
        self.misses = 0
        self.seen = self.stamp

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
        return self.frames >= MIN_FRAMES and self.speed >= MOVING_SPEED and self.radius <= MAX_OBJECT_RADIUS

    @property
    def unseen_s(self):
        return max(0., self.stamp - self.seen)

    @property
    def effective_radius(self):
        """Radius plus the uncertainty of coasting unseen."""
        return self.radius + COAST_SPREAD * self.speed * self.unseen_s

    def at(self, t):
        """Predicted centre ``t`` seconds after the last update."""
        return self.position + self.velocity * t

    def state(self):
        return dict(id=self.id, position=[round(float(v), 3) for v in self.position],
                    z_extent=[round(self.z_low, 2), round(self.z_high, 2)], unseen_s=round(self.unseen_s, 2),
                    velocity=[round(float(v), 3) for v in self.velocity], radius=round(self.radius, 3),
                    frames=self.frames, moving=self.moving())


class Tracker:
    def __init__(self, ground_z=None):
        self.tracks = []
        self.ground_z = ground_z

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
            self.tracks[t].correct(clusters[i][0], clusters[i][1], z_low=_low(clusters[i]), z_high=_high(clusters[i]))
            used_tracks.add(t)
            unmatched.remove(i)
        for t, track in enumerate(self.tracks):
            if t not in used_tracks:
                track.misses += 1
        self.tracks = [track for track in self.tracks if track.misses <= MAX_MISSES
                       or (track.moving() and track.unseen_s <= COAST_S)]
        for i in unmatched:
            self.tracks.append(Track(clusters[i][0], clusters[i][1], stamp, _low(clusters[i]), _high(clusters[i])))
        if self.ground_z is not None:
            for track in self.tracks:
                if track.z_low <= self.ground_z + GROUNDED_M:
                    track.z_low = min(track.z_low, self.ground_z)
                    track.z_high = max(track.z_high, self.ground_z + MIN_STANDING_HEIGHT_M)
        return self.tracks

    def moving(self):
        return [track for track in self.tracks if track.moving()]


def _low(cluster):
    return cluster[3] if len(cluster) > 3 else None


def _high(cluster):
    return cluster[4] if len(cluster) > 4 else None


def _segment_distance(p, a, b):
    ab = b - a
    t = 0. if not ab.any() else float(np.clip(np.dot(p - a, ab) / np.dot(ab, ab), 0., 1.))
    return float(np.linalg.norm(p - (a + t * ab)))


def column_distance(p, a, b, z_low, z_high):
    """Distance from p to a vertical column (z_low..z_high) whose axis sweeps a -> b horizontally."""
    horizontal = _segment_distance(np.array([p[0], p[1], 0.]), np.array([a[0], a[1], 0.]), np.array([b[0], b[1], 0.]))
    vertical = max(0., z_low - p[2], p[2] - z_high)
    return math.hypot(horizontal, vertical)


class SweptMemory:
    """Columns that moving tracks have passed through, for removing their ghosts from an accumulated map.

    A mapping layer that keeps every depth frame (RTAB-Map adds a node only after the camera moves)
    keeps a walking person as a wall of occupied cells along their path. Cells inside a column swept
    by a moving track are dropped from that layer; the live depth layer still reports what is there now.
    """

    def __init__(self, cell=.3, pad=.3, memory_s=300., max_columns=4000):
        self.cell, self.pad, self.memory_s, self.max_columns = cell, pad, memory_s, max_columns
        self.columns = {}
        self.version = 0

    def record(self, tracks, now):
        for track in tracks:
            x, y = float(track.position[0]), float(track.position[1])
            key = (round(x / self.cell), round(y / self.cell))
            radius, low, high = track.radius + self.pad, track.z_low - self.pad, track.z_high + self.pad
            old = self.columns.get(key)
            if old:
                radius, low, high = max(radius, old[2]), min(low, old[3]), max(high, old[4])
            self.columns[key] = (x, y, radius, low, high, now)
        self.columns = {k: v for k, v in self.columns.items() if now - v[5] <= self.memory_s}
        if len(self.columns) > self.max_columns:
            newest = sorted(self.columns.items(), key=lambda item: item[1][5])[-self.max_columns:]
            self.columns = dict(newest)
        if tracks:
            self.version += 1

    def snapshot(self):
        return self.version, [column[:5] for column in self.columns.values()]


def remove_swept(points, columns):
    """Drop points inside swept columns (x, y, radius, z_low, z_high); returns (kept points, removed count)."""
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    if not len(points) or not columns:
        return points, 0
    keep = np.ones(len(points), dtype=bool)
    for x, y, radius, low, high in columns:
        keep &= ~(((points[:, 0] - x) ** 2 + (points[:, 1] - y) ** 2 <= radius * radius)
                  & (points[:, 2] >= low) & (points[:, 2] <= high))
    return points[keep], int(len(points) - keep.sum())


class PredictedObstacles:
    """Swept columns (horizontal radius, vertical extent) of moving tracks over the horizon."""

    def __init__(self, tracks, horizon=HORIZON_S, margin=.3):
        self.capsules = [(track.position.copy(), track.at(horizon), track.effective_radius + margin, track.z_low,
                          track.z_high) for track in tracks]

    def clearance(self, position):
        p = np.asarray(position, dtype=float)
        if not self.capsules:
            return math.inf
        return min(column_distance(p, a, b, low, high) - radius for a, b, radius, low, high in self.capsules)

    def segment_clear(self, start, end, radius=.7):
        a, b = np.asarray(start, dtype=float), np.asarray(end, dtype=float)
        steps = max(2, int(math.ceil(np.linalg.norm(b - a) / .1)))
        return all(self.clearance(a + (b - a) * s / steps) > radius for s in range(steps + 1))


def time_conflict(position, route, speed, tracks, radius=.7, horizon=HORIZON_S, step=.1):
    """First (time, track) at which the drone, flying ``route`` at ``speed`` from ``position``,
    comes within ``radius`` + track radius of a moving track's predicted centre; None if clear.
    ``speed`` 0 (or no route) checks a drone holding at ``position``."""
    points = [np.asarray(position, dtype=float)] + [np.asarray(p, dtype=float) for p in route]
    if not tracks:
        return None
    speed = max(0., speed)
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
            centre = track.at(t)
            if column_distance(drone, centre, centre, track.z_low, track.z_high) < radius + track.effective_radius:
                return t, track
    return None
