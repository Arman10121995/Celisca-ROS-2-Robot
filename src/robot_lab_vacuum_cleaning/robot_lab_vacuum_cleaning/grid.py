"""Occupancy-grid helpers for the cleaning planner (no ROS imports).

Cells are indexed ``(ix, iy)`` = (column, row); arrays are ``[iy, ix]``.
World coordinates are metres in the map frame; ``origin`` is the world
position of the lower-left corner of cell (0, 0), as in nav_msgs/MapMetaData.
"""

import math
from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple

import numpy as np
from scipy import ndimage

FREE_THRESHOLD = 25      # occupancy values below this (and >= 0) are free
OCCUPIED_THRESHOLD = 65  # values at or above this are obstacles

Point = Tuple[float, float]


@dataclass
class Grid:
    resolution: float
    origin: Point
    data: np.ndarray  # int8/int16 [h, w]; -1 unknown, 0..100 occupancy

    @property
    def width(self) -> int:
        return int(self.data.shape[1])

    @property
    def height(self) -> int:
        return int(self.data.shape[0])

    @classmethod
    def from_occupancy(cls, width, height, resolution, origin, values):
        data = np.asarray(values, dtype=np.int16).reshape(int(height), int(width))
        return cls(float(resolution), (float(origin[0]), float(origin[1])), data)

    def signature(self) -> str:
        """Identity of the grid geometry (not its contents)."""
        return '%dx%d@%.3f:%.3f,%.3f' % (self.width, self.height, self.resolution,
                                          self.origin[0], self.origin[1])

    # -- coordinates ---------------------------------------------------
    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        return (int(math.floor((x - self.origin[0]) / self.resolution)),
                int(math.floor((y - self.origin[1]) / self.resolution)))

    def cell_to_world(self, ix: float, iy: float) -> Point:
        return (self.origin[0] + (ix + 0.5) * self.resolution,
                self.origin[1] + (iy + 0.5) * self.resolution)

    def inside(self, ix: int, iy: int) -> bool:
        return 0 <= ix < self.width and 0 <= iy < self.height

    def cells_to_world(self, cells: Iterable[Tuple[int, int]]) -> List[Point]:
        return [self.cell_to_world(ix, iy) for ix, iy in cells]

    # -- masks ---------------------------------------------------------
    def free_mask(self) -> np.ndarray:
        return (self.data >= 0) & (self.data < FREE_THRESHOLD)

    def occupied_mask(self) -> np.ndarray:
        return self.data >= OCCUPIED_THRESHOLD

    def unknown_mask(self) -> np.ndarray:
        return self.data < 0

    def cell_centres(self) -> Tuple[np.ndarray, np.ndarray]:
        xs = self.origin[0] + (np.arange(self.width) + 0.5) * self.resolution
        ys = self.origin[1] + (np.arange(self.height) + 0.5) * self.resolution
        return np.meshgrid(xs, ys)

    def polygon_mask(self, polygon: Sequence[Point]) -> np.ndarray:
        """Cells whose centres lie inside ``polygon`` (even-odd rule)."""
        mask = np.zeros(self.data.shape, dtype=bool)
        if len(polygon) < 3:
            return mask
        xs = [p[0] for p in polygon]
        ys = [p[1] for p in polygon]
        x0, y0 = self.world_to_cell(min(xs), min(ys))
        x1, y1 = self.world_to_cell(max(xs), max(ys))
        x0, y0 = max(x0, 0), max(y0, 0)
        x1, y1 = min(x1, self.width - 1), min(y1, self.height - 1)
        if x1 < x0 or y1 < y0:
            return mask
        px = self.origin[0] + (np.arange(x0, x1 + 1) + 0.5) * self.resolution
        py = self.origin[1] + (np.arange(y0, y1 + 1) + 0.5) * self.resolution
        gx, gy = np.meshgrid(px, py)
        inside = np.zeros(gx.shape, dtype=bool)
        n = len(polygon)
        for i in range(n):
            xi, yi = polygon[i]
            xj, yj = polygon[i - 1]
            crosses = (yi > gy) != (yj > gy)
            with np.errstate(divide='ignore', invalid='ignore'):
                xcross = (xj - xi) * (gy - yi) / (yj - yi) + xi
            inside ^= crosses & (gx < xcross)
        mask[y0:y1 + 1, x0:x1 + 1] = inside
        return mask

    def segment_mask(self, p1: Point, p2: Point, thickness_m: float = 0.0) -> np.ndarray:
        """Cells touched by the segment p1-p2, optionally thickened."""
        mask = np.zeros(self.data.shape, dtype=bool)
        length = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        steps = max(2, int(length / (self.resolution * 0.25)) + 1)
        for t in np.linspace(0.0, 1.0, steps):
            ix, iy = self.world_to_cell(p1[0] + t * (p2[0] - p1[0]),
                                        p1[1] + t * (p2[1] - p1[1]))
            if self.inside(ix, iy):
                mask[iy, ix] = True
        if thickness_m > 0:
            mask = dilate(mask, thickness_m / self.resolution)
        return mask


def dilate(mask: np.ndarray, radius_cells: float) -> np.ndarray:
    """Disc dilation by ``radius_cells`` (Euclidean)."""
    if radius_cells <= 0 or not mask.any():
        return mask.copy()
    distance = ndimage.distance_transform_edt(~mask)
    return distance <= radius_cells


def clearance(blocked: np.ndarray) -> np.ndarray:
    """Distance (cells) from each cell to the nearest blocked cell."""
    if not blocked.any():
        return np.full(blocked.shape, 1e6)
    return ndimage.distance_transform_edt(~blocked)


def line_of_sight(traversable: np.ndarray, a: Tuple[int, int], b: Tuple[int, int]) -> bool:
    """True when every cell on the straight line a-b is traversable."""
    (x0, y0), (x1, y1) = a, b
    n = max(abs(x1 - x0), abs(y1 - y0))
    if n == 0:
        return bool(traversable[y0, x0])
    xs = np.rint(np.linspace(x0, x1, 2 * n + 1)).astype(int)
    ys = np.rint(np.linspace(y0, y1, 2 * n + 1)).astype(int)
    return bool(traversable[ys, xs].all())
