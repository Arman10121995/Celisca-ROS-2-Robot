"""Swept circular-footprint checks for the box-built navigation fixtures.

The visibility graph repairs reference fixtures; it is not a runtime planner
implementation or evidence for an algorithm comparison.
"""
from dataclasses import dataclass
import heapq
import math


def _point_segment_distance(point, start, end):
    dx, dy = end[0] - start[0], end[1] - start[1]
    length2 = dx * dx + dy * dy
    t = (sum((p - s) * d for p, s, d in zip(point, start, (dx, dy)))
         / length2) if length2 else 0.0
    t = min(1.0, max(0.0, t))
    return math.hypot(point[0] - start[0] - t * dx,
                      point[1] - start[1] - t * dy)


@dataclass(frozen=True)
class Box2D:
    name: str
    x: float
    y: float
    hx: float
    hy: float
    yaw: float = 0.0

    def local(self, point):
        dx, dy = point[0] - self.x, point[1] - self.y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return c * dx + s * dy, -s * dx + c * dy

    def point_distance(self, point):
        x, y = self.local(point)
        dx, dy = abs(x) - self.hx, abs(y) - self.hy
        return math.hypot(max(dx, 0.0), max(dy, 0.0)) + min(max(dx, dy), 0.0)

    def segment_distance(self, start, end):
        """Exact distance to the whole segment, including between waypoints."""
        start, end = self.local(start), self.local(end)
        lower, upper = 0.0, 1.0
        for a, b, half in zip(start, end, (self.hx, self.hy)):
            delta = b - a
            if abs(delta) < 1e-15:
                if abs(a) > half:
                    break
            else:
                enter, leave = sorted(((-half - a) / delta, (half - a) / delta))
                lower, upper = max(lower, enter), min(upper, leave)
                if lower > upper:
                    break
        else:
            return 0.0

        def outside(point):
            return math.hypot(max(abs(point[0]) - self.hx, 0.0),
                              max(abs(point[1]) - self.hy, 0.0))

        return min(outside(start), outside(end), *(
            _point_segment_distance((x, y), start, end)
            for x in (-self.hx, self.hx) for y in (-self.hy, self.hy)))

    def corners(self, padding=0.0):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return [(self.x + c * x - s * y, self.y + s * x + c * y)
                for x in (-self.hx - padding, self.hx + padding)
                for y in (-self.hy - padding, self.hy + padding)]


def point_clearance(boxes, point, radius):
    return min(box.point_distance(point) for box in boxes) - radius


def segment_clearance(boxes, start, end, radius):
    return min(box.segment_distance(start, end) for box in boxes) - radius


def reference_route(boxes, start, goal, bounds, radius=0.14, margin=0.20):
    """Deterministic shortest visibility route with extra tracking clearance."""
    if not boxes or not math.isfinite(radius) or radius <= 0 or margin < 0:
        raise ValueError("nonempty geometry, positive radius and nonnegative margin required")

    def valid(point):
        return (all(math.isfinite(v) for v in point)
                and all(lo + radius <= v <= hi - radius
                        for v, lo, hi in zip(point, *bounds))
                and point_clearance(boxes, point, radius) >= margin)

    if not valid(start) or not valid(goal):
        raise ValueError("spawn or goal lacks requested clearance")
    points = [tuple(start), tuple(goal)]
    points += [p for box in boxes for p in box.corners(radius + margin + 1e-6)
               if valid(p)]
    edges = [[] for _ in points]
    for i, start_point in enumerate(points):
        for j in range(i):
            if segment_clearance(boxes, start_point, points[j], radius) >= margin:
                distance = math.dist(start_point, points[j])
                edges[i].append((j, distance))
                edges[j].append((i, distance))
    distances, previous, queue = {0: 0.0}, {}, [(0.0, 0)]
    while queue:
        distance, index = heapq.heappop(queue)
        if distance != distances[index]:
            continue
        if index == 1:
            route = [points[index]]
            while index:
                index = previous[index]
                route.append(points[index])
            return route[::-1]
        for neighbor, cost in edges[index]:
            candidate = distance + cost
            if candidate < distances.get(neighbor, math.inf):
                distances[neighbor], previous[neighbor] = candidate, index
                heapq.heappush(queue, (candidate, neighbor))
    raise ValueError("no reference route with requested footprint and margin")
