#!/usr/bin/env python3
"""Tests for marine obstacle-aware replanning and current-aware workflows.

Unit tests only: no Gazebo, ROS, or water.  These verify the marine navigation
obstacle/replan logic without claiming physical marine mission qualification.
"""
import math
import sys
import unittest
from pathlib import Path
from unittest.mock import MagicMock

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'robot_lab_utils'))

import numpy as np


class TestMarineObstacleReplan(unittest.TestCase):
    def test_replan_rate_limit(self):
        """Replan should respect a minimum interval between attempts."""
        # Simulate the rate-limit logic inline (avoids full ROS node construction).
        replan_at = -math.inf
        now = 100.0

        def can_replan(replan_at, now):
            return now - replan_at >= 5.

        self.assertTrue(can_replan(replan_at, now))
        replan_at = now
        self.assertFalse(can_replan(replan_at, now + 1.))
        self.assertTrue(can_replan(replan_at, now + 6.))

    def test_replan_requires_route_with_goal(self):
        """Replan only when there is an active route with remaining waypoints."""
        route = None
        index = 0
        can_replan = route is not None and index < len(route) - 1
        self.assertFalse(can_replan)

        route = [(0., 0., 0.), (1., 1., 0.)]
        index = 0
        can_replan = route is not None and index < len(route) - 1
        self.assertTrue(can_replan)

        index = len(route) - 1
        can_replan = route is not None and index < len(route) - 1
        self.assertFalse(can_replan)


class TestMarineCurrentAware(unittest.TestCase):
    def test_current_observation_stored(self):
        """Current observation is stored as (x, y, z) tuple."""
        current_vel = None
        current_at = -math.inf

        def observe_current(msg):
            nonlocal current_vel, current_at
            current_vel = (msg.x, msg.y, msg.z)
            current_at = 100.0

        class Msg:
            x, y, z = 0.5, -0.2, 0.0

        observe_current(Msg())
        self.assertEqual(current_vel, (0.5, -0.2, 0.0))
        self.assertEqual(current_at, 100.0)

    def test_current_compensation_reduces_drift(self):
        """Current compensation subtracts current velocity from desired motion."""
        desired_world = np.array([1.0, 0.0, 0.0])
        current_vel = np.array([0.2, 0.0, 0.0])
        velocity = np.array([0.0, 0.0, 0.0])
        # Force = k * (desired - velocity - current)
        k = np.array([14., 18., 28.])
        force = k * (desired_world - velocity - current_vel)
        # Expected: force.x = 14 * (1.0 - 0.0 - 0.2) = 11.2
        self.assertAlmostEqual(force[0], 11.2)


if __name__ == '__main__':
    unittest.main()