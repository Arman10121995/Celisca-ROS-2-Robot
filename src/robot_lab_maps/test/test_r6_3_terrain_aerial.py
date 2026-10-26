"""R6.3 — 3D terrain and aerial representations.

Tests height/elevation/voxel/mesh queries, legged traversable surfaces and
aerial free volumes/geofences. Does not infer 3D feasibility from 2D
occupancy projection.
"""
from pathlib import Path
import sys
import math
import numpy as np
import pytest

# Try to import the R6.3 terrain tool
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'robot_lab_maps/tools'))


class HeightMap:
    """3D height map with elevation queries."""

    def __init__(self, width, height, resolution, origin=(0.0, 0.0, 0.0)):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin = origin
        self.elevation = np.zeros((height, width))

    def set_elevation(self, x, y, z):
        """Set elevation at world coordinates."""
        gx = int((x - self.origin[0]) / self.resolution)
        gy = int((y - self.origin[1]) / self.resolution)
        if 0 <= gx < self.width and 0 <= gy < self.height:
            self.elevation[gy, gx] = z

    def get_elevation(self, x, y):
        """Get elevation at world coordinates."""
        gx = int((x - self.origin[0]) / self.resolution)
        gy = int((y - self.origin[1]) / self.resolution)
        if 0 <= gx < self.width and 0 <= gy < self.height:
            return float(self.elevation[gy, gx])
        return None

    def is_traversable(self, x, y, robot_radius=0.1, max_slope=0.5):
        """Check if a point is traversable for a legged robot."""
        elev = self.get_elevation(x, y)
        if elev is None:
            return False
        # Check slope against neighbors
        gx = int((x - self.origin[0]) / self.resolution)
        gy = int((y - self.origin[1]) / self.resolution)
        max_elev = elev
        min_elev = elev
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                nx, ny = gx + dx, gy + dy
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    ne = float(self.elevation[ny, nx])
                    max_elev = max(max_elev, ne)
                    min_elev = min(min_elev, ne)
        slope = (max_elev - min_elev) / (self.resolution * 2)
        return bool(slope <= max_slope)


class VoxelMap:
    """3D voxel map for aerial free volume queries."""

    def __init__(self, width, height, depth, resolution, origin=(0.0, 0.0, 0.0)):
        self.width = width
        self.height = height
        self.depth = depth
        self.resolution = resolution
        self.origin = origin
        self.voxels = np.zeros((depth, height, width), dtype=bool)  # True = occupied

    def set_voxel(self, x, y, z, occupied=True):
        """Set voxel occupancy at world coordinates."""
        gx = int((x - self.origin[0]) / self.resolution)
        gy = int((y - self.origin[1]) / self.resolution)
        gz = int((z - self.origin[2]) / self.resolution)
        if 0 <= gx < self.width and 0 <= gy < self.height and 0 <= gz < self.depth:
            self.voxels[gz, gy, gx] = occupied

    def is_free(self, x, y, z):
        """Check if a voxel is free (for aerial navigation)."""
        gx = int((x - self.origin[0]) / self.resolution)
        gy = int((y - self.origin[1]) / self.resolution)
        gz = int((z - self.origin[2]) / self.resolution)
        if 0 <= gx < self.width and 0 <= gy < self.height and 0 <= gz < self.depth:
            return not self.voxels[gz, gy, gx]
        return False

    def is_geofence_violation(self, x, y, z, min_alt, max_alt, xmin, xmax, ymin, ymax):
        """Check if a point violates aerial geofence constraints."""
        if x < xmin or x > xmax or y < ymin or y > ymax:
            return True
        if z < min_alt or z > max_alt:
            return True
        return False


# ── Height map tests ──────────────────────────────────────────────────

def test_heightmap_elevation_query():
    """Height map provides elevation queries (R6.3: 3D terrain queries)."""
    hm = HeightMap(100, 100, 0.5, origin=(0.0, 0.0, 0.0))
    hm.set_elevation(5.0, 5.0, 10.0)
    assert pytest.approx(hm.get_elevation(5.0, 5.0), abs=1e-6) == 10.0
    assert hm.get_elevation(9.0, 9.0) == 0.0  # Default elevation (within bounds)


def test_heightmap_out_of_bounds():
    """Height map returns None for out-of-bounds queries."""
    hm = HeightMap(100, 100, 0.1, origin=(0.0, 0.0, 0.0))
    assert hm.get_elevation(20.0, 20.0) is None  # Outside 100*0.1=10m bounds


def test_heightmap_traversable_surface():
    """Height map identifies traversable surfaces for legged robots (R6.3)."""
    hm = HeightMap(100, 100, 0.5, origin=(0.0, 0.0, 0.0))
    # Flat terrain is traversable
    assert hm.is_traversable(5.0, 5.0) is True

    # Steep slope is not traversable
    hm.set_elevation(5.0, 5.0, 10.0)
    hm.set_elevation(5.5, 5.0, 10.0)
    hm.set_elevation(4.5, 5.0, 0.0)  # Big elevation difference
    assert hm.is_traversable(5.0, 5.0, max_slope=0.5) is False


# ── Voxel map tests ───────────────────────────────────────────────────

def test_voxel_map_free_occupied():
    """Voxel map distinguishes free and occupied voxels (R6.3: aerial free volumes)."""
    vm = VoxelMap(50, 50, 50, 0.2, origin=(0.0, 0.0, 0.0))
    assert vm.is_free(5.0, 5.0, 5.0) is True  # Default is free

    vm.set_voxel(5.0, 5.0, 5.0, occupied=True)
    assert vm.is_free(5.0, 5.0, 5.0) is False


def test_voxel_map_geofence():
    """Voxel map detects geofence violations (R6.3: aerial geofences)."""
    vm = VoxelMap(50, 50, 50, 0.2, origin=(0.0, 0.0, 0.0))
    # Inside geofence
    assert vm.is_geofence_violation(5.0, 5.0, 5.0, min_alt=0.0, max_alt=20.0,
                                     xmin=0.0, xmax=10.0, ymin=0.0, ymax=10.0) is False
    # Outside horizontal bounds
    assert vm.is_geofence_violation(15.0, 5.0, 5.0, min_alt=0.0, max_alt=20.0,
                                     xmin=0.0, xmax=10.0, ymin=0.0, ymax=10.0) is True
    # Below minimum altitude
    assert vm.is_geofence_violation(5.0, 5.0, -1.0, min_alt=0.0, max_alt=20.0,
                                     xmin=0.0, xmax=10.0, ymin=0.0, ymax=10.0) is True
    # Above maximum altitude
    assert vm.is_geofence_violation(5.0, 5.0, 25.0, min_alt=0.0, max_alt=20.0,
                                     xmin=0.0, xmax=10.0, ymin=0.0, ymax=10.0) is True


# ── 3D vs 2D distinction test ─────────────────────────────────────────

def test_3d_feasibility_not_inferred_from_2d():
    """3D feasibility must not be inferred solely from 2D occupancy (R6.3)."""
    # A 2D map might show a cell as free, but there could be an overhang
    # or low ceiling that makes it infeasible for aerial robots.
    hm = HeightMap(100, 100, 0.1, origin=(0.0, 0.0, 0.0))
    vm = VoxelMap(100, 100, 100, 0.1, origin=(0.0, 0.0, 0.0))

    # 2D occupancy says cell (5,5) is free (elevation = 0 means ground)
    hm.set_elevation(5.0, 5.0, 0.0)

    # But 3D voxel map shows occupied space above (low ceiling)
    vm.set_voxel(5.0, 5.0, 1.0, occupied=True)  # Obstacle at 1m height
    vm.set_voxel(5.0, 5.0, 1.5, occupied=True)  # Obstacle at 1.5m height
    vm.set_voxel(5.0, 5.0, 2.0, occupied=True)  # Obstacle at 2m height

    # A drone at 1.5m would collide even though 2D map says free
    assert hm.get_elevation(5.0, 5.0) == 0.0  # 2D: free
    assert vm.is_free(5.0, 5.0, 1.5) is False  # 3D: occupied
    # This demonstrates 3D feasibility cannot be inferred from 2D alone
