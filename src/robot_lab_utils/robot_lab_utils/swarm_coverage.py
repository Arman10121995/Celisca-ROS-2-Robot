"""Cooperative coverage measured from what the members' scans actually saw (R10.3).

The area's free occupancy cells are the target. Every fresh scan marks the
free cells its rays cross (up to the sensor range) as seen, from the
member's estimated pose. Coverage is the seen fraction of the target, not a
count of reached waypoints. Idle members are sent to the nearest unseen cells
that are safe for their body, away from goals other members already hold.
"""
import math

import numpy as np

SENSOR_RANGE_M = 2.
GOAL_SEPARATION_M = 1.5


class SensedCoverage:
    def __init__(self, grid, center, width, height, sensor_range=SENSOR_RANGE_M):
        cx, cy = (float(v) for v in center)
        width, height, sensor_range = float(width), float(height), float(sensor_range)
        if not all(map(math.isfinite, (cx, cy, width, height, sensor_range))):
            raise ValueError('Finite coverage area and sensor range required')
        if not .5 <= width <= 100 or not .5 <= height <= 100 or not .5 <= sensor_range <= 10:
            raise ValueError('Coverage width/height must be 0.5–100 m and sensor range 0.5–10 m')
        self.grid, self.sensor_range = grid, sensor_range
        rows, cols = grid.free.shape
        ys, xs = np.mgrid[0:rows, 0:cols]
        wx, wy = self._world(xs + .5, ys + .5)
        inside = (np.abs(wx - cx) <= width / 2) & (np.abs(wy - cy) <= height / 2)
        self.target = grid.free & inside
        if not self.target.any():
            raise ValueError('The coverage area contains no known free cells')
        self.seen = np.zeros_like(self.target)
        self.area = dict(x=cx, y=cy, width=width, height=height)
        self.wx, self.wy = wx, wy

    def _world(self, col, row):
        g = self.grid
        c, s = math.cos(g.origin[2]), math.sin(g.origin[2])
        u, v = col * g.resolution, row * g.resolution
        return g.origin[0] + c * u - s * v, g.origin[1] + s * u + c * v

    def integrate(self, x, y, yaw, ranges, angle_min, angle_increment, range_min=0., range_max=math.inf):
        """Mark free cells crossed by each valid ray (and to sensor range for max-range rays)."""
        if not all(map(math.isfinite, (x, y, yaw, angle_min, angle_increment))):
            return 0
        g = self.grid
        step = g.resolution
        # Thin the beams so neighbouring rays are about one cell apart at full range.
        stride = max(1, int(step / max(abs(angle_increment) * self.sensor_range, 1e-6)))
        before = int(self.seen.sum())
        rows, cols = self.seen.shape
        for index in range(0, len(ranges), stride):
            value = ranges[index]
            if value is None or math.isnan(value) or value < range_min:
                continue
            length = min(self.sensor_range, value if math.isfinite(value) else range_max)
            if not math.isfinite(length) or length <= 0:
                continue
            angle = yaw + angle_min + index * angle_increment
            for d in np.arange(0., length, step):
                col, row = g.cell(x + d * math.cos(angle), y + d * math.sin(angle))
                if not (0 <= row < rows and 0 <= col < cols) or not g.free[row, col]:
                    break
                self.seen[row, col] = True
        return int(self.seen.sum()) - before

    def fraction(self):
        return float((self.seen & self.target).sum() / self.target.sum())

    def next_goal(self, position, radius, held=(), reachable=None):
        """Nearest unseen target cell with body clearance, away from held goals."""
        g = self.grid
        candidates = self.target & ~self.seen & (g.clearance > radius + .1)
        if reachable is not None:
            candidates &= reachable
        rows, cols = np.nonzero(candidates)
        if not len(rows):
            return None
        xs, ys = self.wx[rows, cols], self.wy[rows, cols]
        distance = np.hypot(xs - position[0], ys - position[1])
        for hx, hy in held:
            distance = np.where(np.hypot(xs - hx, ys - hy) < GOAL_SEPARATION_M, np.inf, distance)
        best = int(np.argmin(distance))
        if not math.isfinite(distance[best]):
            return None
        x, y = float(xs[best]), float(ys[best])
        return dict(x=x, y=y, yaw=math.atan2(y - position[1], x - position[0]))

    def state(self):
        return dict(area=self.area, fraction=round(self.fraction(), 4), target_cells=int(self.target.sum()),
                    seen_cells=int((self.seen & self.target).sum()), sensor_range_m=self.sensor_range,
                    source='Free cells crossed by each member\'s own scan rays from its estimated pose')


def reachable_from(grid, position, radius):
    """Cells connected to the member through body-clear space (4-connected)."""
    from scipy.ndimage import label
    safe = grid.clearance > radius + .05
    labels, _ = label(safe)
    col, row = grid.cell(*position)
    if not (0 <= row < safe.shape[0] and 0 <= col < safe.shape[1]) or not labels[row, col]:
        return np.zeros_like(safe)
    return labels == labels[row, col]


def redistribute(goals, members):
    """Give a lost member's remaining goals to the nearest remaining member each."""
    if not members:
        return {}
    plan = {name: [] for name in members}
    for goal in goals:
        nearest = min(members, key=lambda name: math.hypot(members[name][0] - goal['x'], members[name][1] - goal['y']))
        plan[nearest].append(goal)
    return plan
