"""Goal B heading for robots whose final heading is not scored (scripts/sim_nav_mission.py)."""
import math
import sys
import types
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).parent))
pytest.importorskip("scipy")
mission = pytest.importorskip("sim_nav_mission")


def _grid():
    data = np.zeros((40, 40), dtype=np.int16)
    data[:2, :] = 100                                   # a wall along y in [0, 0.1) m
    info = types.SimpleNamespace(height=40, width=40, resolution=.05,
                                 origin=types.SimpleNamespace(position=types.SimpleNamespace(x=0., y=0.)))
    return types.SimpleNamespace(data=data.flatten().tolist(), info=info)


def test_goal_heading_runs_along_the_wall_nearest_the_approach():
    goal = dict(x=1.0, y=.45)
    # Approaching from above, heading straight into the wall (-90 deg).
    yaw = mission.along_obstacle_heading(_grid(), goal, -math.pi/2+.3)
    assert abs(math.sin(yaw)) < 1e-6 and math.cos(yaw) > 0      # along +x, the side the approach leans to
    yaw = mission.along_obstacle_heading(_grid(), goal, -math.pi/2-.3)
    assert abs(math.sin(yaw)) < 1e-6 and math.cos(yaw) < 0


def test_goal_heading_falls_back_to_the_approach_off_the_map():
    assert mission.along_obstacle_heading(_grid(), dict(x=9., y=9.), .7) == .7
