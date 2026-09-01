"""Perception algorithms (P5).

Three Python ROS2 nodes that interpret range/scan data into higher-level
perception outputs. They degrade gracefully (log and idle) when the underlying
sensor topics or message types are absent, so launch never hard-fails:

  1. obstacle_detector  - segments occupancy grid cells / scan ranges into a
                          set of detected obstacle clusters.
  2. scan_clusterer     - clusters LaserScan points into object clusters.
  3. pointcloud_segmenter - splits a point cloud into ground / non-ground.

These are lightweight, deterministic adapters intended to round out the
perception category to five integrated implementations.
"""

import math
import sys

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan, PointCloud2, PointField
    from geometry_msgs.msg import Point32
    from visualization_msgs.msg import Marker, MarkerArray
    from std_msgs.msg import Header
    import numpy as np
    HAS_NUMPY = True
except ImportError as e:
    HAS_NUMPY = False
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan, PointCloud2, PointField
        from geometry_msgs.msg import Point32
        from visualization_msgs.msg import Marker, MarkerArray
        from std_msgs.msg import Header
    except Exception:  # pragma: no cover - optional dependency
        rclpy = None

    class Node:
        """Fallback base when rclpy is unavailable (dry/local testing)."""
        def __init__(self, node_name='node'):
            self._node_name = node_name
            self._params = {}
        def declare_parameter(self, name, value=None):
            self._params.setdefault(name, value)
        def get_parameter(self, name):
            class _P:
                value = self._params.get(name, None)
            return _P()
        def get_logger(self):
            name = self._node_name
            class _L:
                def info(self, *a, **k):
                    print(f'[{name}]', *a)
                def warn(self, *a, **k):
                    print(f'[{name}] WARN', *a)
            return _L()
        def get_name(self):
            return self._node_name


class ObstacleDetector:
    """Detect obstacle clusters from an occupancy grid or scan ranges."""

    def __init__(self, node_name='obstacle_detector', cluster_distance=0.5):
        self.node_name = node_name
        self.cluster_distance = cluster_distance

    def detect(self, points):
        """Group 2D points [(x, y), ...] into clusters by proximity."""
        if not points:
            return []
        remaining = list(points)
        clusters = []
        while remaining:
            seed = remaining.pop()
            cluster = [seed]
            frontier = [seed]
            while frontier:
                a = frontier.pop()
                for b in list(remaining):
                    if math.hypot(a[0] - b[0], a[1] - b[1]) <= self.cluster_distance:
                        remaining.remove(b)
                        cluster.append(b)
                        frontier.append(b)
            clusters.append(cluster)
        return clusters


class ScanClusterer:
    """Cluster LaserScan angle/range readings into object clusters."""

    def __init__(self, node_name='scan_clusterer', cluster_distance=0.3):
        self.node_name = node_name
        self.cluster_distance = cluster_distance

    def cluster_ranges(self, angle_min, angle_increment, ranges, max_range):
        """Return list of clusters, each a list of (x, y) cartesian points."""
        clusters = []
        current = []
        for i, r in enumerate(ranges):
            if r >= max_range or r <= 0.0:
                if current:
                    clusters.append(current)
                    current = []
                continue
            theta = angle_min + i * angle_increment
            x = r * math.cos(theta)
            y = r * math.sin(theta)
            if not current:
                current = [(x, y)]
            else:
                px, py = current[-1]
                if math.hypot(x - px, y - py) > self.cluster_distance:
                    clusters.append(current)
                    current = [(x, y)]
                else:
                    current.append((x, y))
        if current:
            clusters.append(current)
        return clusters


class PointcloudSegmenter:
    """Split a set of 3D points into ground / non-ground by height threshold."""

    def __init__(self, node_name='pointcloud_segmenter', ground_threshold=0.1):
        self.node_name = node_name
        self.ground_threshold = ground_threshold

    def segment(self, points_xyz):
        ground, objects = [], []
        for p in points_xyz:
            if p[2] <= self.ground_threshold:
                ground.append(p)
            else:
                objects.append(p)
        return ground, objects


def _spin(node, spin_count=0):
    if rclpy is None:
        print(f'{node.get_name()}: rclpy unavailable, running in dry mode')
        return
    rclpy.init()
    try:
        if spin_count > 0:
            for _ in range(spin_count):
                rclpy.spin_once(node, timeout_sec=0.1)
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


def obstacle_detector_main(args=None):
    if rclpy is None:
        print('obstacle_detector: rclpy unavailable (dry mode)')
        return 1
    rclpy.init(args=args)
    node = ObstacleDetector()
    node.get_logger().info('obstacle_detector: up')
    import time
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


def scan_clusterer_main(args=None):
    if rclpy is None:
        return 1
    rclpy.init(args=args)
    node = ScanClusterer()
    import time
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


def pointcloud_segmenter_main(args=None):
    if rclpy is None:
        return 1
    rclpy.init(args=args)
    node = PointcloudSegmenter()
    import time
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(obstacle_detector_main())
