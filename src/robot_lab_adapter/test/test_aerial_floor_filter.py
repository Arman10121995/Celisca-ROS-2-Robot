"""Floor removal before moving-obstacle clustering survives a biased height estimate (R5.11)."""
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip('rclpy')
from robot_lab_adapter.aerial_obstacles import AerialObstacles


def clusters_for(centres, ground_z=.24):
    layer = SimpleNamespace(centres=centres)
    obstacles = SimpleNamespace(ground_z=ground_z, static_scene=None)
    return AerialObstacles.unexplained_clusters(obstacles, layer)


def test_a_floor_seen_too_high_does_not_swallow_a_walker():
    grid = np.arange(.5, 8., .15)
    gx, gy = np.meshgrid(grid, np.arange(-3., 3., .15))
    # The floor ahead reads up to 0.5 m too high (height estimate and attitude errors), 0.15 m cells.
    floor = np.column_stack([gx.ravel(), gy.ravel(), .24 + .5 * gx.ravel() / 8.])
    walker = np.array([[4. + dx, dy, z] for dx in (-.15, 0., .15) for dy in (-.15, 0., .15)
                       for z in np.arange(.5, 1.9, .15)])
    clusters = clusters_for(np.vstack([floor, walker]))
    assert len(clusters) == 1
    centre, radius = clusters[0][0], clusters[0][1]
    assert abs(centre[0] - 4.) < .3 and abs(centre[1]) < .3 and radius < 1.


def test_a_raised_platform_is_not_taken_for_floor():
    deck = np.column_stack([np.full(40, 3.), np.linspace(-1, 1, 40), np.full(40, 1.5)])   # 1.26 m above ground
    clusters = clusters_for(deck)
    assert len(clusters) == 1                     # kept as an obstacle, not removed as floor
