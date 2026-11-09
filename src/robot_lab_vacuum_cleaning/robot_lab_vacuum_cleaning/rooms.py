"""Automatic room division of an occupancy grid, plus merge/split edits.

Free space is split where it narrows below ``door_width`` (doorways and
narrow passages): the cores wider than that are labelled and then grown back
over all free space by geodesic (through-free-space) breadth-first search.
Tiny rooms are merged into the neighbour they share most boundary with.
"""

import heapq
from collections import deque
from typing import Dict, List, Tuple

import numpy as np
from scipy import ndimage

from .grid import Grid, Point

N4 = ((1, 0), (-1, 0), (0, 1), (0, -1))


def segment_rooms(grid: Grid, door_width: float = 0.9, min_room_area: float = 1.5,
                  min_free_area: float = 0.25, min_width: float = 0.45) -> np.ndarray:
    """Room label per cell (0 = not a room).  Labels are 1..N in order of
    decreasing area."""
    free = grid.free_mask()
    res = grid.resolution
    # Drop isolated specks of free space (tops of furniture, noise).
    comp, n = ndimage.label(free)
    if n:
        sizes = ndimage.sum(free, comp, index=np.arange(1, n + 1)) * res * res
        keep = np.zeros(n + 1, dtype=bool)
        keep[1:] = sizes >= min_free_area
        free = keep[comp]
    if not free.any():
        return np.zeros(free.shape, dtype=np.int32)
    dist = ndimage.distance_transform_edt(free) * res
    thresholds = [t for t in (0.3, 0.45, 0.6, 0.75, 0.9, 1.05, 1.25, 1.5)
                  if t >= door_width / 2.0 - 1e-9] or [door_width / 2.0]
    cores = hierarchical_cores(free, dist, thresholds, int(0.6 / (res * res)))
    if not cores.any():
        cores, _ = ndimage.label(free)
    labels = grow_labels(cores, free, priority=dist)
    labels = merge_small(labels, min_room_area / (res * res))
    # Strips no robot fits into (behind furniture, between double walls).
    for rid in [int(i) for i in np.unique(labels) if i > 0]:
        if dist[labels == rid].max() < min_width / 2.0:
            labels[labels == rid] = 0
    return relabel_by_area(labels)


def hierarchical_cores(free: np.ndarray, dist: np.ndarray, thresholds: List[float],
                       min_core_cells: int) -> np.ndarray:
    """Room seeds: a region that, at a higher clearance threshold, breaks
    into two or more substantial cores is a set of rooms joined by openings
    narrower than the rooms themselves; recurse into each core."""
    seeds = np.zeros(free.shape, dtype=np.int32)
    counter = [0]

    def emit(mask):
        counter[0] += 1
        seeds[mask] = counter[0]

    stack = [(free, 0)]
    while stack:
        region, level = stack.pop()
        if level >= len(thresholds):
            emit(region)
            continue
        sub = region & (dist > thresholds[level])
        lab, n = ndimage.label(sub)
        if n == 0:
            emit(region)
            continue
        sizes = ndimage.sum(sub, lab, index=np.arange(1, n + 1))
        big = [k + 1 for k in range(n) if sizes[k] >= min_core_cells]
        if not big:
            emit(region)
        else:
            for k in big:
                stack.append((lab == k, level + 1))
    return seeds


def grow_labels(seeds: np.ndarray, free: np.ndarray, priority: np.ndarray = None) -> np.ndarray:
    """Multi-source flood of seed labels across ``free``.

    With ``priority`` (clearance), cells are claimed in order of decreasing
    clearance - a watershed - so borders settle at the narrow points
    (doorways) instead of half-way between seeds."""
    labels = np.where(free, seeds, 0).astype(np.int32)
    h, w = labels.shape
    if priority is None:
        queue = deque(zip(*np.nonzero(labels)))
        while queue:
            y, x = queue.popleft()
            lab = labels[y, x]
            for dx, dy in N4:
                nx, ny = x + dx, y + dy
                if 0 <= nx < w and 0 <= ny < h and free[ny, nx] and labels[ny, nx] == 0:
                    labels[ny, nx] = lab
                    queue.append((ny, nx))
    else:
        heap = []
        counter = 0
        for y, x in zip(*np.nonzero(labels)):
            heap.append((-float(priority[y, x]), counter, int(y), int(x)))
            counter += 1
        heapq.heapify(heap)
        while heap:
            _, _, y, x = heapq.heappop(heap)
            lab = labels[y, x]
            for dx, dy in N4:
                nx, ny = x + dx, y + dy
                if 0 <= nx < w and 0 <= ny < h and free[ny, nx] and labels[ny, nx] == 0:
                    labels[ny, nx] = lab
                    counter += 1
                    heapq.heappush(heap, (-float(priority[ny, nx]), counter, ny, nx))
    # Free pockets no seed reached become rooms of their own (merged later).
    rest = free & (labels == 0)
    if rest.any():
        extra, m = ndimage.label(rest)
        labels = np.where(rest, extra + labels.max(), labels)
    return labels


def adjacency(labels: np.ndarray) -> Dict[Tuple[int, int], int]:
    """Shared boundary length (cell edges) between touching labels."""
    out: Dict[Tuple[int, int], int] = {}
    for a, b in ((labels[:, :-1], labels[:, 1:]), (labels[:-1, :], labels[1:, :])):
        touch = (a != b) & (a > 0) & (b > 0)
        pairs = np.stack((np.minimum(a[touch], b[touch]), np.maximum(a[touch], b[touch])), axis=1)
        if len(pairs):
            uniq, counts = np.unique(pairs, axis=0, return_counts=True)
            for (p, q), c in zip(uniq, counts):
                out[(int(p), int(q))] = out.get((int(p), int(q)), 0) + int(c)
    return out


def merge_small(labels: np.ndarray, min_cells: float) -> np.ndarray:
    labels = labels.copy()
    while True:
        ids, counts = np.unique(labels[labels > 0], return_counts=True)
        small = [(c, i) for i, c in zip(ids, counts) if c < min_cells]
        if not small or len(ids) <= 1:
            return labels
        small.sort()
        adj = adjacency(labels)
        merged = False
        for _, sid in small:
            neighbours = [(n, (p if q == sid else q)) for (p, q), n in adj.items() if sid in (p, q)]
            if neighbours:
                target = max(neighbours)[1]
                labels[labels == sid] = target
                merged = True
                break
        if not merged:
            # Small, isolated pockets: not part of any room.
            for _, sid in small:
                labels[labels == sid] = 0
            return labels


def relabel_by_area(labels: np.ndarray) -> np.ndarray:
    ids, counts = np.unique(labels[labels > 0], return_counts=True)
    order = ids[np.argsort(-counts, kind='stable')]
    lut = np.zeros(int(labels.max()) + 1, dtype=np.int32)
    for new, old in enumerate(order, start=1):
        lut[old] = new
    return lut[labels]


def merge_rooms(labels: np.ndarray, ids: List[int]) -> Tuple[np.ndarray, int]:
    """Merge rooms ``ids`` into the first one; returns (labels, kept id)."""
    keep = int(ids[0])
    out = labels.copy()
    for rid in ids[1:]:
        out[out == int(rid)] = keep
    return out, keep


def split_room(grid: Grid, labels: np.ndarray, room_id: int, p1: Point, p2: Point
               ) -> Tuple[np.ndarray, List[int]]:
    """Cut room ``room_id`` along the line p1-p2.  Returns the new labels and
    the ids of the resulting parts (the largest keeps ``room_id``)."""
    room = labels == room_id
    cut = grid.segment_mask(p1, p2, thickness_m=grid.resolution)
    parts, n = ndimage.label(room & ~cut)
    if n < 2:
        return labels, [room_id]
    sizes = ndimage.sum(room, parts, index=np.arange(1, n + 1))
    order = list(np.argsort(-sizes) + 1)
    seeds = np.zeros(labels.shape, dtype=np.int32)
    next_id = int(labels.max()) + 1
    ids = [room_id]
    for rank, part in enumerate(order):
        if rank == 0:
            seeds[parts == part] = room_id
        elif sizes[part - 1] * grid.resolution ** 2 >= 0.5:
            seeds[parts == part] = next_id
            ids.append(next_id)
            next_id += 1
    # Cut cells and slivers join the part they touch.
    grown = grow_labels(seeds, room)
    out = labels.copy()
    out[room] = grown[room]
    return out, ids


def room_info(grid: Grid, labels: np.ndarray) -> Dict[int, Dict]:
    """Area and label anchor (a cell well inside the room) per room."""
    out = {}
    res = grid.resolution
    for rid in [int(i) for i in np.unique(labels) if i > 0]:
        mask = labels == rid
        dist = ndimage.distance_transform_edt(mask)
        iy, ix = np.unravel_index(int(np.argmax(dist)), dist.shape)
        ys, xs = np.nonzero(mask)
        x0, y0 = grid.cell_to_world(xs.min(), ys.min())
        x1, y1 = grid.cell_to_world(xs.max(), ys.max())
        out[rid] = {
            'area_m2': round(float(mask.sum()) * res * res, 2),
            'anchor': [round(v, 3) for v in grid.cell_to_world(ix, iy)],
            'bbox': [round(x0, 3), round(y0, 3), round(x1, 3), round(y1, 3)],
        }
    return out


def match_names(old_info: Dict[int, Dict], new_labels: np.ndarray, grid: Grid,
                old_meta: Dict[int, Dict]) -> Dict[int, Dict]:
    """Carry room names/settings over to a new segmentation: each old room's
    anchor point picks the new room it falls in."""
    out = {}
    for rid, info in old_info.items():
        ax, ay = info.get('anchor', (None, None))
        if ax is None:
            continue
        ix, iy = grid.world_to_cell(ax, ay)
        if grid.inside(ix, iy):
            new = int(new_labels[iy, ix])
            if new > 0 and new not in out and rid in old_meta:
                out[new] = dict(old_meta[rid])
    return out
