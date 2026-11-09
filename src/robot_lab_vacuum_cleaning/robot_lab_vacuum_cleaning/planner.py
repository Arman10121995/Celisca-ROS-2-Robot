"""Cleaning path planning on an occupancy grid (no ROS imports).

PlanningMap turns the map, the operator's keep-out areas and obstacles seen
during the run into the set of cells the robot centre may occupy.  On it:
A* transit paths, boustrophedon lanes (optionally cross-hatched), wall/edge
contours, a square spot spiral and exploration frontiers.
"""

import heapq
import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree

from .grid import Grid, Point, clearance, dilate, line_of_sight

Cell = Tuple[int, int]
SQRT2 = math.sqrt(2.0)


@dataclass
class Keepout:
    """Operator keep-out geometry: polygons (no-go) and segments (walls)."""
    polygons: List[Sequence[Point]] = field(default_factory=list)
    walls: List[Tuple[Point, Point]] = field(default_factory=list)

    def mask(self, grid: Grid) -> np.ndarray:
        mask = np.zeros(grid.data.shape, dtype=bool)
        for polygon in self.polygons:
            mask |= grid.polygon_mask(polygon)
        for p1, p2 in self.walls:
            # One cell either side so a diagonal step cannot slip through.
            mask |= grid.segment_mask(p1, p2, thickness_m=grid.resolution)
        return mask


class PlanningMap:
    """Traversable cells for a disc robot of ``robot_radius`` metres."""

    def __init__(self, grid: Grid, robot_radius: float, margin: float = 0.04,
                 keepout: Optional[Keepout] = None,
                 extra_blocked: Optional[np.ndarray] = None,
                 unknown_is_blocked: bool = True, keepout_margin: float = 0.12):
        self.grid = grid
        self.robot_radius = float(robot_radius)
        self.margin = float(margin)
        obstacles = grid.occupied_mask()
        if unknown_is_blocked:
            obstacles = obstacles | grid.unknown_mask()
        if extra_blocked is not None and extra_blocked.shape == obstacles.shape:
            obstacles = obstacles | extra_blocked
        self.keepout_mask = (keepout.mask(grid) if keepout is not None
                             else np.zeros(obstacles.shape, dtype=bool))
        blocked = obstacles | self.keepout_mask
        res = grid.resolution
        self.obstacle_clearance_m = clearance(obstacles) * res
        self.clearance_m = clearance(blocked) * res
        self.required = self.robot_radius + self.margin
        # Operator keep-outs get a wider berth than walls: localisation and
        # path-following errors must not carry the body into a no-go area.
        keep_clear = clearance(self.keepout_mask) * res
        self.traversable = (self.obstacle_clearance_m > self.required) & \
            (keep_clear > self.required + keepout_margin)
        # Prefer the middle of free space: cost rises inside ``comfort``.
        comfort = self.required + 0.25
        excess = np.clip(comfort - self.clearance_m, 0.0, None) / 0.25
        self.step_cost = 1.0 + 4.0 * excess
        self._components = None

    # -- helpers ---------------------------------------------------------
    @property
    def resolution(self) -> float:
        return self.grid.resolution

    def is_traversable(self, cell: Cell) -> bool:
        ix, iy = cell
        return self.grid.inside(ix, iy) and bool(self.traversable[iy, ix])

    def snap(self, point: Point, max_distance: float = 0.6) -> Optional[Cell]:
        """Nearest traversable cell to ``point`` within ``max_distance``."""
        ix, iy = self.grid.world_to_cell(*point)
        if self.is_traversable((ix, iy)):
            return ix, iy
        r = int(math.ceil(max_distance / self.resolution))
        x0, x1 = max(ix - r, 0), min(ix + r + 1, self.grid.width)
        y0, y1 = max(iy - r, 0), min(iy + r + 1, self.grid.height)
        if x0 >= x1 or y0 >= y1:
            return None
        window = self.traversable[y0:y1, x0:x1]
        if not window.any():
            return None
        ys, xs = np.nonzero(window)
        d2 = (xs + x0 - ix) ** 2 + (ys + y0 - iy) ** 2
        k = int(np.argmin(d2))
        if d2[k] > r * r:
            return None
        return int(xs[k] + x0), int(ys[k] + y0)

    def components(self) -> np.ndarray:
        if self._components is None:
            self._components, _ = ndimage.label(self.traversable, structure=np.ones((3, 3)))
        return self._components

    def reachable_from(self, point: Point) -> np.ndarray:
        cell = self.snap(point)
        if cell is None:
            return np.zeros(self.traversable.shape, dtype=bool)
        labels = self.components()
        return labels == labels[cell[1], cell[0]]

    # -- A* --------------------------------------------------------------
    def astar(self, start: Point, goal: Point, max_expansions: int = 600000
              ) -> Optional[List[Point]]:
        """Shortest safe path (world points, simplified) or None."""
        s = self.snap(start)
        g = self.snap(goal)
        if s is None or g is None:
            return None
        if s == g:
            return [start, goal]
        w = self.grid.width
        trav = self.traversable
        cost = self.step_cost
        if line_of_sight(trav, s, g):
            return self._densify([start, self.grid.cell_to_world(*g)])
        best = {s[1] * w + s[0]: 0.0}
        parent = {}
        gx, gy = g

        def h(ix, iy):
            dx, dy = abs(ix - gx), abs(iy - gy)
            return (dx + dy) + (SQRT2 - 2.0) * min(dx, dy)

        heap = [(h(*s), 0.0, s[0], s[1])]
        neighbours = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                      (1, 1, SQRT2), (1, -1, SQRT2), (-1, 1, SQRT2), (-1, -1, SQRT2))
        height = self.grid.height
        expansions = 0
        found = False
        while heap:
            f, gcost, ix, iy = heapq.heappop(heap)
            key = iy * w + ix
            if gcost > best.get(key, math.inf):
                continue
            if ix == gx and iy == gy:
                found = True
                break
            expansions += 1
            if expansions > max_expansions:
                return None
            c0 = cost[iy, ix]
            for dx, dy, step in neighbours:
                nx, ny = ix + dx, iy + dy
                if nx < 0 or ny < 0 or nx >= w or ny >= height or not trav[ny, nx]:
                    continue
                if dx and dy and not (trav[iy, nx] and trav[ny, ix]):
                    continue
                ng = gcost + step * 0.5 * (c0 + cost[ny, nx])
                nkey = ny * w + nx
                if ng < best.get(nkey, math.inf):
                    best[nkey] = ng
                    parent[nkey] = key
                    heapq.heappush(heap, (ng + h(nx, ny), ng, nx, ny))
        if not found:
            return None
        cells = []
        key = gy * w + gx
        while True:
            cells.append((key % w, key // w))
            if key not in parent:
                break
            key = parent[key]
        cells.reverse()
        cells = self._shortcut(cells)
        points = [start] + [self.grid.cell_to_world(*c) for c in cells[1:]]
        return self._densify(points)

    def _shortcut(self, cells: List[Cell]) -> List[Cell]:
        """String-pulling: drop cells visible from an earlier kept cell,
        unless the shortcut leaves the comfortable middle of free space."""
        if len(cells) <= 2:
            return cells
        out = [cells[0]]
        i = 0
        while i < len(cells) - 1:
            j = len(cells) - 1
            while j > i + 1:
                if line_of_sight(self.traversable, cells[i], cells[j]) and \
                        self._line_cost_ok(cells[i], cells[j]):
                    break
                j -= 1
            out.append(cells[j])
            i = j
        return out

    def _line_cost_ok(self, a: Cell, b: Cell) -> bool:
        n = max(abs(b[0] - a[0]), abs(b[1] - a[1]), 1)
        xs = np.rint(np.linspace(a[0], b[0], n + 1)).astype(int)
        ys = np.rint(np.linspace(a[1], b[1], n + 1)).astype(int)
        endpoints = max(self.step_cost[a[1], a[0]], self.step_cost[b[1], b[0]])
        return float(self.step_cost[ys, xs].max()) <= endpoints + 0.5

    def _densify(self, points: List[Point], spacing: float = 0.15) -> List[Point]:
        out = [points[0]]
        for p, q in zip(points, points[1:]):
            d = math.hypot(q[0] - p[0], q[1] - p[1])
            n = max(1, int(d / spacing))
            for k in range(1, n + 1):
                t = k / n
                out.append((p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])))
        return out

    def path_clear(self, points: Sequence[Point]) -> bool:
        cells = [self.grid.world_to_cell(*p) for p in points]
        return all(self.is_traversable(c) for c in cells)


# ---------------------------------------------------------------------------
# Coverage
# ---------------------------------------------------------------------------

def sweep_mask(grid: Grid, segments: Sequence[Tuple[Point, Point]], radius: float) -> np.ndarray:
    """Cells within ``radius`` of any of ``segments``."""
    seed = np.zeros(grid.data.shape, dtype=bool)
    for p, q in segments:
        seed |= grid.segment_mask(p, q)
    return dilate(seed, radius / grid.resolution)


def _runs(row: np.ndarray, min_len: int) -> List[Tuple[int, int]]:
    """[start, end] index pairs of True runs at least ``min_len`` long."""
    padded = np.concatenate(([False], row, [False]))
    edges = np.flatnonzero(padded[1:] != padded[:-1])
    starts, ends = edges[0::2], edges[1::2] - 1
    return [(int(s), int(e)) for s, e in zip(starts, ends) if e - s + 1 >= min_len]


def lane_segments(grid: Grid, region: np.ndarray, spacing: float, axis: str = 'x',
                  sweep_radius: Optional[float] = None) -> List[Tuple[Point, Point]]:
    """Straight lanes ``spacing`` metres apart over ``region``.

    ``axis`` 'x' gives lanes parallel to x.  Region parts that fall between
    two lane rows (narrow strips) get extra lanes so no part is skipped.
    """
    if not region.any():
        return []
    step = max(1, int(round(spacing / grid.resolution)))
    min_len = 1
    mask = region if axis == 'x' else region.T
    rows = np.flatnonzero(mask.any(axis=1))
    first, last = int(rows[0]), int(rows[-1])
    chosen = list(range(first + step // 2 if last - first >= step else first, last + 1, step))
    if not chosen:
        chosen = [(first + last) // 2]
    if last - chosen[-1] > step // 2:
        chosen.append(last)
    segs_rc = []
    for r in chosen:
        for c0, c1 in _runs(mask[r], min_len):
            segs_rc.append((r, c0, c1))
    segments = [_rc_to_world(grid, r, c0, c1, axis) for r, c0, c1 in segs_rc]
    # Fill gaps: region cells farther than the sweep from every lane.
    radius = sweep_radius if sweep_radius is not None else spacing * 0.6
    covered = sweep_mask(grid, segments, radius)
    missed = region & ~covered
    if missed.any():
        labels, n = ndimage.label(missed if axis == 'x' else missed.T)
        for k in range(1, n + 1):
            ys, xs = np.nonzero(labels == k)
            if len(ys) < 3:
                continue
            r = int(np.median(ys))
            for c0, c1 in _runs(mask[r], min_len):
                if c0 <= xs.max() and c1 >= xs.min():
                    segments.append(_rc_to_world(grid, r, c0, c1, axis))
    return segments


def _rc_to_world(grid, r, c0, c1, axis):
    if axis == 'x':
        return grid.cell_to_world(c0, r), grid.cell_to_world(c1, r)
    return grid.cell_to_world(r, c0), grid.cell_to_world(r, c1)


def best_axis(grid: Grid, region: np.ndarray, spacing: float) -> str:
    """Lane direction with fewer segments (fewer turns)."""
    nx = len(lane_segments(grid, region, spacing, 'x', sweep_radius=1e9))
    ny = len(lane_segments(grid, region, spacing, 'y', sweep_radius=1e9))
    return 'x' if nx <= ny else 'y'


def order_segments(segments: Sequence[Tuple[Point, Point]], start: Point
                   ) -> List[Tuple[Point, Point]]:
    """Greedy nearest-endpoint order; each segment flipped to start near the
    previous end, which gives the serpentine of a boustrophedon in open rooms."""
    remaining = list(segments)
    ordered = []
    here = start
    while remaining:
        best_i, best_d, flip = 0, math.inf, False
        for i, (p, q) in enumerate(remaining):
            dp = math.hypot(p[0] - here[0], p[1] - here[1])
            dq = math.hypot(q[0] - here[0], q[1] - here[1])
            if dp < best_d:
                best_i, best_d, flip = i, dp, False
            if dq < best_d:
                best_i, best_d, flip = i, dq, True
        p, q = remaining.pop(best_i)
        seg = (q, p) if flip else (p, q)
        ordered.append(seg)
        here = seg[1]
    return ordered


def segment_points(p: Point, q: Point, spacing: float = 0.2) -> List[Point]:
    d = math.hypot(q[0] - p[0], q[1] - p[1])
    n = max(1, int(d / spacing))
    return [(p[0] + (q[0] - p[0]) * k / n, p[1] + (q[1] - p[1]) * k / n) for k in range(n + 1)]


def edge_contours(pmap: PlanningMap, region: np.ndarray, min_length: float = 0.4,
                  spacing: float = 0.12) -> List[List[Point]]:
    """Polylines that follow obstacles/walls at the closest safe distance.

    Uses the outermost ring of traversable cells that borders a real obstacle
    (not a doorway into another room, nor a keep-out area)."""
    res = pmap.resolution
    band = pmap.required + 1.5 * res
    edge = (region & pmap.traversable & (pmap.obstacle_clearance_m <= band)
            & (pmap.clearance_m <= band))
    if not edge.any():
        return []
    ys, xs = np.nonzero(edge)
    cells = np.column_stack((xs, ys)).astype(float)
    tree = cKDTree(cells)
    unvisited = np.ones(len(cells), dtype=bool)
    contours = []
    while unvisited.any():
        i = int(np.flatnonzero(unvisited)[0])
        chain = [i]
        unvisited[i] = False
        # Grow forward then backward from the seed.
        for direction in (0, 1):
            current = chain[-1] if direction == 0 else chain[0]
            while True:
                found = None
                for radius in (1.01, 1.5, 2.3):
                    near = [j for j in tree.query_ball_point(cells[current], radius) if unvisited[j]]
                    if near:
                        found = min(near, key=lambda j: np.sum((cells[j] - cells[current]) ** 2))
                        break
                if found is None:
                    break
                # Cells passed over by a 2-cell jump are consumed too.
                for j in tree.query_ball_point(cells[found], 1.01):
                    if unvisited[j] and j != found and np.sum((cells[j] - cells[current]) ** 2) <= 2.1:
                        unvisited[j] = False
                unvisited[found] = False
                if direction == 0:
                    chain.append(found)
                else:
                    chain.insert(0, found)
                current = found
        length = len(chain) * res
        if length < min_length:
            continue
        step = max(1, int(round(spacing / res)))
        idx = chain[::step]
        if idx[-1] != chain[-1]:
            idx.append(chain[-1])
        contours.append([pmap.grid.cell_to_world(*cells[j]) for j in idx])
    return contours


def order_polylines(polylines: Sequence[List[Point]], start: Point) -> List[List[Point]]:
    remaining = [list(p) for p in polylines]
    ordered = []
    here = start
    while remaining:
        best = min(range(len(remaining)), key=lambda i: min(
            math.dist(here, remaining[i][0]), math.dist(here, remaining[i][-1])))
        line = remaining.pop(best)
        if math.dist(here, line[-1]) < math.dist(here, line[0]):
            line.reverse()
        ordered.append(line)
        here = line[-1]
    return ordered


def spot_spiral(pmap: PlanningMap, centre: Point, side: float = 1.6,
                spacing: float = 0.22) -> List[List[Point]]:
    """Square spiral out from ``centre``; split where it is not traversable."""
    pts = [centre]
    x, y = centre
    leg = spacing
    dirs = ((1, 0), (0, 1), (-1, 0), (0, -1))
    k = 0
    h = side / 2.0
    # One turn past the square, clamped to it, so the last ring runs the
    # whole edge instead of stopping short on two sides.
    while leg <= side + 2 * spacing + 1e-9:
        for _ in range(2):
            dx, dy = dirs[k % 4]
            x, y = x + dx * leg, y + dy * leg
            p = (min(max(x, centre[0] - h), centre[0] + h), min(max(y, centre[1] - h), centre[1] + h))
            if math.dist(p, pts[-1]) > 1e-6:
                pts.append(p)
            k += 1
        leg += spacing
    dense = []
    for p, q in zip(pts, pts[1:]):
        dense.extend(segment_points(p, q, 0.08)[:-1])
    dense.append(pts[-1])
    runs, current = [], []
    for p in dense:
        if pmap.is_traversable(pmap.grid.world_to_cell(*p)):
            current.append(p)
        elif current:
            runs.append(current)
            current = []
    if current:
        runs.append(current)
    return [_thin(r, 0.15) for r in runs if len(r) >= 2]


def _thin(points: List[Point], spacing: float) -> List[Point]:
    out = [points[0]]
    for p in points[1:-1]:
        if math.dist(p, out[-1]) >= spacing:
            out.append(p)
    if len(points) > 1:
        out.append(points[-1])
    return out


def frontiers(grid: Grid, pmap: PlanningMap, min_size_m: float = 0.3
              ) -> List[Tuple[Point, int]]:
    """(target point, size in cells) for each frontier cluster that has a
    reachable traversable cell nearby."""
    free = grid.free_mask()
    unknown = grid.unknown_mask()
    near_unknown = ndimage.binary_dilation(unknown, structure=np.array(
        [[0, 1, 0], [1, 1, 1], [0, 1, 0]], dtype=bool))
    frontier = free & near_unknown
    labels, n = ndimage.label(frontier, structure=np.ones((3, 3)))
    out = []
    min_cells = max(2, int(min_size_m / grid.resolution))
    for k in range(1, n + 1):
        ys, xs = np.nonzero(labels == k)
        if len(xs) < min_cells:
            continue
        cx, cy = grid.cell_to_world(float(np.mean(xs)), float(np.mean(ys)))
        cell = pmap.snap((cx, cy), max_distance=1.2)
        if cell is None:
            # Try the cluster's own cells.
            for j in np.linspace(0, len(xs) - 1, min(len(xs), 8)).astype(int):
                cell = pmap.snap(grid.cell_to_world(xs[j], ys[j]), max_distance=1.2)
                if cell is not None:
                    break
        if cell is None:
            continue
        out.append((grid.cell_to_world(*cell), len(xs)))
    return out
