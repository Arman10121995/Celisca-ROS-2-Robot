from __future__ import annotations

import math
from typing import Any, Dict, Optional, Tuple

try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
except ImportError:
    rclpy = None
    Node = object


class GroundTruthAdapter:
    """Extract ground-truth metrics from sensor data and simulation state."""

    def __init__(self) -> None:
        self.poses = []
        self.scan_distances = []
        self.odometry_data = []

    def add_odometry(self, msg: Any) -> None:
        """Record odometry data (as dict or Odometry msg)."""
        if isinstance(msg, dict):
            self.odometry_data.append(msg)
            x = msg.get('pose', {}).get('pose', {}).get('position', {}).get('x', 0.0)
            y = msg.get('pose', {}).get('pose', {}).get('position', {}).get('y', 0.0)
        else:
            self.odometry_data.append(msg)
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y

        self.poses.append((x, y))

    def add_scan(self, msg: Any) -> None:
        """Record laser scan data."""
        if isinstance(msg, dict):
            ranges = msg.get('ranges', [])
        else:
            ranges = msg.ranges

        valid_ranges = [r for r in ranges if 0 < r < 30.0]
        if valid_ranges:
            self.scan_distances.append(min(valid_ranges))

    def compute_path_length(self) -> float:
        """Compute total path length from poses."""
        if len(self.poses) < 2:
            return 0.0

        length = 0.0
        for i in range(1, len(self.poses)):
            dx = self.poses[i][0] - self.poses[i - 1][0]
            dy = self.poses[i][1] - self.poses[i - 1][1]
            length += math.sqrt(dx * dx + dy * dy)

        return length

    def compute_collision_count(self, collision_threshold: float = 0.3) -> int:
        """Estimate collision count from minimum clearance."""
        if not self.scan_distances:
            return 0

        collisions = sum(1 for d in self.scan_distances if d < collision_threshold)
        return collisions

    def compute_min_clearance(self) -> float:
        """Compute minimum clearance from sensor data."""
        if not self.scan_distances:
            return 0.0

        return min(self.scan_distances)

    def extract_metrics(self) -> Dict[str, float]:
        """Extract all computed metrics."""
        return {
            'path_length_m': self.compute_path_length(),
            'collision_count': self.compute_collision_count(),
            'min_clearance_m': self.compute_min_clearance(),
        }


class LiveSensorAdapter(Node if rclpy else object):
    """Adapter for live sensor streams during benchmark execution."""

    def __init__(self, node_name: str = 'live_sensor_adapter'):
        if rclpy:
            rclpy.init()
            super().__init__(node_name)
            self.ground_truth = GroundTruthAdapter()
            self.create_subscription(
                Odometry,
                '/odom',
                lambda msg: self.ground_truth.add_odometry(msg),
                10,
            )
            self.create_subscription(
                LaserScan,
                '/scan',
                lambda msg: self.ground_truth.add_scan(msg),
                10,
            )
        else:
            self.ground_truth = GroundTruthAdapter()

    def get_metrics(self) -> Dict[str, float]:
        """Get current metrics."""
        return self.ground_truth.extract_metrics()

    def reset(self) -> None:
        """Reset metrics collection."""
        self.ground_truth = GroundTruthAdapter()

    def shutdown(self) -> None:
        """Clean up resources."""
        try:
            if rclpy:
                rclpy.shutdown()
        except Exception:
            pass
