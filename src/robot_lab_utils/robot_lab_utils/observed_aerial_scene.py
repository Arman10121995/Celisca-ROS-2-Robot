"""Observed occupancy supplements a complete canonical static flight scene.

An absent observed point never proves free space. Voxel centres carry a
half-diagonal inflation so quantisation does not create holes in observations.
This is current occupancy, not a moving-object prediction or flight guarantee.
"""
import hashlib
import math

import numpy as np


class VoxelObstacles:
    def __init__(self, points, voxel=.15, max_cells=100000):
        from scipy.spatial import cKDTree
        points = np.asarray(points, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3 or len(points) > 300000:
            raise ValueError('Observed cloud needs at most 300000 XYZ points')
        if not .02 <= voxel <= .3 or not np.all(np.isfinite(points)) or np.any(np.abs(points) > 10000):
            raise ValueError('Finite bounded observed coordinates and voxel size required')
        cells = np.unique(np.floor(points/voxel).astype(np.int64), axis=0)
        if len(cells) > max_cells:
            raise ValueError('Observed occupancy exceeds the cell budget; refusing to drop obstacles')
        self.centres = (cells+.5)*voxel
        self.centres.flags.writeable = False
        self.tree = cKDTree(self.centres) if len(cells) else None
        self.padding = math.sqrt(3)*voxel/2
        self.cell_count = len(cells)
        self.source_points = len(points)
        self.sha256 = hashlib.sha256(str(float(voxel)).encode()+cells.tobytes()).hexdigest()

    def clearance(self, position):
        if self.tree is None:
            return math.inf
        distance, _ = self.tree.query(np.asarray(position), k=1)
        return float(distance)-self.padding

    def segment_clear(self, start, end, radius=.7):
        """Continuous clearance via conservative nearest-distance stepping."""
        a, b = np.asarray(start, dtype=float), np.asarray(end, dtype=float)
        if a.shape != (3,) or b.shape != (3,) or not np.all(np.isfinite([a, b])) or radius <= 0:
            raise ValueError('Finite segment endpoints and positive envelope required')
        length, travelled, iterations = float(np.linalg.norm(b-a)), 0., 0
        while True:
            position = a if not length else a+(b-a)*(travelled/length)
            clearance = self.clearance(position)-radius
            if clearance <= 1e-5:
                return False
            if travelled >= length:
                return True
            iterations += 1
            if iterations > 500:
                return False  # A near-contact query never stalls FCU ticks.
            travelled = min(length, travelled+min(.25, .8*clearance))


class ObservedScene:
    def __init__(self, static_scene, *observations):
        self.static = static_scene
        self.observations = observations

    def clearance(self, position):
        return min(self.static.clearance(position), *(observation.clearance(position) for observation in self.observations))

    def segment(self, start, end, radius, deadline, canceled):
        from .aerial_planning import StaticScene
        return StaticScene.segment(self, start, end, radius, deadline, canceled)


def depth_points(depth, intrinsic, min_range=.2, max_range=8.):
    """Project every valid registered pinhole depth pixel; no sparse sampling."""
    depth, k = np.asarray(depth, dtype=float), np.asarray(intrinsic, dtype=float).reshape(3, 3)
    if depth.ndim != 2 or depth.size > 300000 or not np.all(np.isfinite(k)) or k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError('Bounded depth image and finite pinhole intrinsics required')
    valid = np.isfinite(depth) & (depth >= min_range) & (depth <= max_range)
    v, u = np.nonzero(valid)
    z = depth[v, u]
    if len(z) < 32:
        raise ValueError('Insufficient valid depth; unseen space is not certified clear')
    y = (v-k[1, 2])*z/k[1, 1]
    x = (u-k[0, 2])*z/k[0, 0]-k[0, 1]*y/k[0, 0]
    return np.column_stack((x, y, z)), dict(valid_pixels=len(z), pixels=depth.size,
        valid_fraction=len(z)/depth.size, min_range_m=min_range, max_range_m=max_range)


def transform_points(points, translation, quaternion_xyzw):
    from .aerial_planning import rotation
    translation, q = np.asarray(translation), np.asarray(quaternion_xyzw)
    if translation.shape != (3,) or q.shape != (4,) or not np.all(np.isfinite([*translation, *q])) or not .5 < np.linalg.norm(q) < 1.5:
        raise ValueError('Finite bounded transform translation/quaternion required')
    return np.asarray(points) @ rotation(q[[3, 0, 1, 2]]).T+translation
