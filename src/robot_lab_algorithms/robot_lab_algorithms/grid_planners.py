"""Sampling and roadmap global planners on an occupancy grid (R7.6).

Pure numpy/scipy implementations used by the Nav2 ComputePathToPose adapter
(robot_lab_algorithms.nav2_adapters), so each method plans on the same
inflated Nav2 global costmap as the plugin planners and drives the robot
through the same navigator:

* ``rrt``       - RRT (LaValle 1998): goal-biased tree, every edge
  collision-checked at half the grid resolution.
* ``rrt_star``  - RRT* (Karaman & Frazzoli 2011): best collision-free parent
  in a shrinking-ball neighbourhood, rewiring with cost propagation, and
  anytime improvement until the iteration/time budget ends.
* ``prm``       - PRM (Kavraki et al. 1996): free-space samples joined to
  their k nearest collision-free neighbours, Dijkstra on the roadmap.
* ``voronoi``   - Generalized-Voronoi-diagram roadmap: cells equidistant to
  two different obstacles (labelled nearest-obstacle transform), searched
  with Dijkstra where leaving the diagram costs ``off_diagram_cost`` per
  metre, so the path follows maximum-clearance corridors and joins the
  diagram only near start and goal.

Every planner returns ``(x, y)`` waypoints from start to goal (both exact)
or ``[]`` when it finds no path.  Coordinates are metres in the grid frame.
"""
from __future__ import annotations

import heapq
import math
import time

import numpy as np


class GridView:
    """Free/blocked view of an occupancy grid (row = y, column = x).

    ``data`` holds nav_msgs/OccupancyGrid values (-1 unknown, 0..100).  A cell
    is blocked at ``blocked_at`` or above; with Nav2's costmap translation
    99 is the inscribed radius and 100 lethal, so the default keeps the
    robot's centre out of every cell its footprint would hit.
    """

    def __init__(self, data, origin, resolution, blocked_at=99, allow_unknown=True):
        self.data = np.asarray(data, dtype=np.int16)
        if self.data.ndim != 2:
            raise ValueError('grid data must be 2-D (rows, columns)')
        self.origin = (float(origin[0]), float(origin[1]))
        self.resolution = float(resolution)
        unknown = self.data < 0
        self.free = (self.data < blocked_at) & (~unknown | bool(allow_unknown))
        self.height, self.width = self.data.shape

    @classmethod
    def from_msg(cls, msg, **kwargs):
        data = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        return cls(data, (msg.info.origin.position.x, msg.info.origin.position.y),
                   msg.info.resolution, **kwargs)

    @property
    def bounds(self):
        return (self.origin[0], self.origin[1],
                self.origin[0] + self.width * self.resolution,
                self.origin[1] + self.height * self.resolution)

    def cells(self, xy):
        """(rows, cols) for an (N, 2) array of points; -1 where outside."""
        xy = np.atleast_2d(np.asarray(xy, dtype=float))
        cols = np.floor((xy[:, 0] - self.origin[0]) / self.resolution).astype(int)
        rows = np.floor((xy[:, 1] - self.origin[1]) / self.resolution).astype(int)
        outside = (cols < 0) | (rows < 0) | (cols >= self.width) | (rows >= self.height)
        rows[outside] = -1
        cols[outside] = -1
        return rows, cols

    def points_free(self, xy):
        rows, cols = self.cells(xy)
        inside = rows >= 0
        result = np.zeros(len(rows), dtype=bool)
        result[inside] = self.free[rows[inside], cols[inside]]
        return result

    def is_free(self, x, y):
        return bool(self.points_free([[x, y]])[0])

    def segment_free(self, a, b):
        """Every point of segment a-b, sampled at half the grid resolution, is free."""
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        count = max(2, int(math.ceil(length / (0.5 * self.resolution))) + 1)
        t = np.linspace(0.0, 1.0, count)[:, None]
        points = np.asarray(a, dtype=float) * (1 - t) + np.asarray(b, dtype=float) * t
        return bool(np.all(self.points_free(points)))

    def cell_center(self, row, col):
        return (self.origin[0] + (col + 0.5) * self.resolution,
                self.origin[1] + (row + 0.5) * self.resolution)

    def sample_free(self, rng, count):
        rows, cols = np.nonzero(self.free)
        if not len(rows):
            return np.zeros((0, 2))
        pick = rng.integers(len(rows), size=count)
        jitter = rng.random((count, 2))
        return np.stack([self.origin[0] + (cols[pick] + jitter[:, 0]) * self.resolution,
                         self.origin[1] + (rows[pick] + jitter[:, 1]) * self.resolution], axis=1)


def _endpoints_ok(view, start, goal):
    return view.is_free(*start) and view.is_free(*goal)


def _trace(parents, positions, index):
    path = []
    while index is not None and index >= 0:
        path.append((float(positions[index][0]), float(positions[index][1])))
        index = parents[index]
    path.reverse()
    return path


def rrt(view, start, goal, rng=None, step=0.5, max_iter=6000, goal_bias=0.1, time_budget=1.0):
    """RRT with collision-checked edges; the goal is joined exactly."""
    rng = rng or np.random.default_rng(0)
    if not _endpoints_ok(view, start, goal):
        return []
    positions = np.zeros((max_iter + 2, 2))
    parents = [-1]
    positions[0] = start
    count = 1
    deadline = time.monotonic() + time_budget
    goal_arr = np.asarray(goal, dtype=float)
    for _ in range(max_iter):
        if time.monotonic() > deadline:
            break
        sample = goal_arr if rng.random() < goal_bias else view.sample_free(rng, 1)[0]
        d = np.hypot(*(positions[:count] - sample).T)
        nearest = int(np.argmin(d))
        if d[nearest] < 1e-9:
            continue
        new = positions[nearest] + (sample - positions[nearest]) * min(1.0, step / d[nearest])
        if not view.segment_free(positions[nearest], new):
            continue
        positions[count] = new
        parents.append(nearest)
        count += 1
        if math.hypot(*(goal_arr - new)) <= step and view.segment_free(new, goal_arr):
            positions[count] = goal_arr
            parents.append(count - 1)
            return _trace(parents, positions, count)
    return []


def rrt_star(view, start, goal, rng=None, step=0.5, max_iter=3000, gamma=3.0,
             goal_bias=0.05, time_budget=1.0):
    """RRT*: best collision-free parent, rewiring, anytime until the budget ends."""
    rng = rng or np.random.default_rng(0)
    if not _endpoints_ok(view, start, goal):
        return []
    capacity = max_iter + 1
    positions = np.zeros((capacity, 2))
    costs = np.full(capacity, np.inf)
    parents = np.full(capacity, -1, dtype=int)
    children = [[] for _ in range(capacity)]
    positions[0], costs[0] = start, 0.0
    count = 1
    goal_arr = np.asarray(goal, dtype=float)
    best_goal_parent, best_goal_cost = -1, np.inf
    deadline = time.monotonic() + time_budget

    def propagate(index, delta):
        stack = list(children[index])
        while stack:
            child = stack.pop()
            costs[child] -= delta
            stack.extend(children[child])

    for _ in range(max_iter):
        if time.monotonic() > deadline:
            break
        sample = goal_arr if rng.random() < goal_bias else view.sample_free(rng, 1)[0]
        d = np.hypot(*(positions[:count] - sample).T)
        nearest = int(np.argmin(d))
        if d[nearest] < 1e-9:
            continue
        new = positions[nearest] + (sample - positions[nearest]) * min(1.0, step / d[nearest])
        if not view.segment_free(positions[nearest], new):
            continue
        radius = max(step, min(gamma * math.sqrt(math.log(count + 1) / (count + 1)) * 4.0, 3.0 * step))
        dn = np.hypot(*(positions[:count] - new).T)
        near = np.nonzero(dn <= radius)[0]
        parent, parent_cost = nearest, costs[nearest] + d[nearest] * min(1.0, step / d[nearest])
        for candidate in near[np.argsort(costs[near] + dn[near])]:
            cost = costs[candidate] + dn[candidate]
            if cost >= parent_cost:
                break
            if view.segment_free(positions[candidate], new):
                parent, parent_cost = int(candidate), cost
                break
        index = count
        positions[index], costs[index], parents[index] = new, parent_cost, parent
        children[parent].append(index)
        count += 1
        for candidate in near:
            candidate = int(candidate)
            if candidate == parent:
                continue
            via = parent_cost + dn[candidate]
            if via + 1e-9 < costs[candidate] and view.segment_free(new, positions[candidate]):
                old_parent = parents[candidate]
                if old_parent >= 0:
                    children[old_parent].remove(candidate)
                delta = costs[candidate] - via
                parents[candidate], costs[candidate] = index, via
                children[index].append(candidate)
                propagate(candidate, delta)
        to_goal = math.hypot(*(goal_arr - new))
        if to_goal <= step and parent_cost + to_goal < best_goal_cost \
                and view.segment_free(new, goal_arr):
            best_goal_parent, best_goal_cost = index, parent_cost + to_goal
    if best_goal_parent < 0:
        return []
    # Costs may have dropped by rewiring since the goal link was recorded.
    path = _trace(list(parents[:count]), positions, best_goal_parent)
    return path + [(float(goal[0]), float(goal[1]))]


def _graph_shortest_path(points, edges, weights, source, target):
    """Dijkstra over an undirected weighted edge list; node indices -> path indices."""
    adjacency = [[] for _ in range(len(points))]
    for (a, b), w in zip(edges, weights):
        adjacency[a].append((b, w))
        adjacency[b].append((a, w))
    dist = [math.inf] * len(points)
    prev = [-1] * len(points)
    dist[source] = 0.0
    heap = [(0.0, source)]
    while heap:
        d, node = heapq.heappop(heap)
        if node == target:
            break
        if d > dist[node]:
            continue
        for other, w in adjacency[node]:
            nd = d + w
            if nd < dist[other]:
                dist[other], prev[other] = nd, node
                heapq.heappush(heap, (nd, other))
    if not math.isfinite(dist[target]):
        return []
    path, node = [], target
    while node >= 0:
        path.append(node)
        node = prev[node]
    return path[::-1]


def prm(view, start, goal, rng=None, samples=600, k=12, max_edge=2.5):
    """PRM: k-nearest collision-free roadmap over free-space samples, Dijkstra."""
    from scipy.spatial import cKDTree
    rng = rng or np.random.default_rng(0)
    if not _endpoints_ok(view, start, goal):
        return []
    points = np.vstack([np.asarray([start, goal], dtype=float), view.sample_free(rng, samples)])
    tree = cKDTree(points)
    distances, neighbours = tree.query(points, k=min(k + 1, len(points)))
    edges, weights, seen = [], [], set()
    for a in range(len(points)):
        for distance, b in zip(distances[a][1:], neighbours[a][1:]):
            b = int(b)
            key = (min(a, b), max(a, b))
            if key in seen or not math.isfinite(distance) or distance > max_edge:
                continue
            seen.add(key)
            if view.segment_free(points[a], points[b]):
                edges.append(key)
                weights.append(float(distance))
    indices = _graph_shortest_path(points, edges, weights, 0, 1)
    return [(float(points[i][0]), float(points[i][1])) for i in indices]


def voronoi_diagram(view, separation=0.75):
    """(on_diagram mask, clearance in metres) for the view's free space.

    A free cell is on the generalized Voronoi diagram when a neighbouring
    cell's nearest obstacle point lies on another side of it: the two
    nearest points are further apart than ``separation`` x the local
    clearance (and at least 2.5 cells).  Unlike component labels this also
    finds the diagram between two parts of one connected wall, which is
    every corridor of a maze.
    """
    from scipy import ndimage
    blocked = ~view.free
    # The map border bounds free space like an obstacle.
    padded = np.pad(blocked, 1, constant_values=True)
    distance, (near_r, near_c) = ndimage.distance_transform_edt(~padded, return_indices=True)
    near_r = near_r[1:-1, 1:-1].astype(float)
    near_c = near_c[1:-1, 1:-1].astype(float)
    distance = distance[1:-1, 1:-1]
    diagram = np.zeros_like(view.free)
    threshold = np.maximum(2.5, separation * distance)
    for dr, dc in ((0, 1), (1, 0), (1, 1), (1, -1)):
        rs = slice(max(dr, 0), view.height + min(dr, 0))
        rd = slice(max(-dr, 0), view.height + min(-dr, 0))
        cs = slice(max(dc, 0), view.width + min(dc, 0))
        cd = slice(max(-dc, 0), view.width + min(-dc, 0))
        apart = np.zeros_like(view.free)
        gap = np.hypot(near_r[rd, cd] - near_r[rs, cs], near_c[rd, cd] - near_c[rs, cs])
        apart[rd, cd] = gap > threshold[rd, cd]
        diagram |= apart
    diagram &= view.free
    return diagram, distance * view.resolution


def voronoi(view, start, goal, off_diagram_cost=8.0):
    """Shortest path that follows the generalized Voronoi diagram where it can."""
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import dijkstra
    if not _endpoints_ok(view, start, goal):
        return []
    diagram, _ = voronoi_diagram(view)
    free = view.free
    index = -np.ones(free.shape, dtype=np.int64)
    rows, cols = np.nonzero(free)
    index[rows, cols] = np.arange(len(rows))
    step_cost = np.where(diagram, 1.0, float(off_diagram_cost))
    sources, targets, weights = [], [], []
    for dr, dc in ((0, 1), (1, 0), (1, 1), (1, -1)):
        r2, c2 = rows + dr, cols + dc
        ok = (r2 >= 0) & (r2 < view.height) & (c2 >= 0) & (c2 < view.width)
        r1, c1, r2, c2 = rows[ok], cols[ok], r2[ok], c2[ok]
        ok = free[r2, c2]
        if dr and dc:
            # No diagonal step past a blocked corner.
            ok &= free[r1 + dr, c1] & free[r1, c1 + dc]
        r1, c1, r2, c2 = r1[ok], c1[ok], r2[ok], c2[ok]
        length = math.hypot(dr, dc) * view.resolution
        cost = 0.5 * (step_cost[r1, c1] + step_cost[r2, c2]) * length
        sources.append(index[r1, c1])
        targets.append(index[r2, c2])
        weights.append(cost)
    sources, targets, weights = (np.concatenate(v) for v in (sources, targets, weights))
    size = len(rows)
    graph = coo_matrix((np.concatenate([weights, weights]),
                        (np.concatenate([sources, targets]), np.concatenate([targets, sources]))),
                       shape=(size, size)).tocsr()
    (sr,), (sc,) = (a for a in view.cells([start]))
    (gr,), (gc,) = (a for a in view.cells([goal]))
    source, target = int(index[sr, sc]), int(index[gr, gc])
    _, predecessors = dijkstra(graph, directed=False, indices=source, return_predecessors=True)
    if predecessors[target] < 0 and target != source:
        return []
    chain, node = [], target
    while node >= 0 and node != source:
        chain.append(node)
        node = predecessors[node]
    chain.append(source)
    chain.reverse()
    path = [(float(start[0]), float(start[1]))]
    path += [view.cell_center(rows[i], cols[i]) for i in chain[1:-1]]
    path.append((float(goal[0]), float(goal[1])))
    return path


PLANNERS = {
    'rrt_planner': rrt,
    'rrt_star_planner': rrt_star,
    'prm_planner': prm,
    'voronoi_planner': voronoi,
}


def densify(waypoints, spacing):
    """Insert points so consecutive waypoints are at most ``spacing`` apart."""
    if len(waypoints) < 2:
        return list(waypoints)
    out = [tuple(waypoints[0])]
    for a, b in zip(waypoints[:-1], waypoints[1:]):
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        pieces = max(1, int(math.ceil(length / spacing)))
        for i in range(1, pieces + 1):
            t = i / pieces
            out.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
    return out


def path_length(waypoints):
    return float(sum(math.hypot(b[0] - a[0], b[1] - a[1])
                     for a, b in zip(waypoints[:-1], waypoints[1:])))
