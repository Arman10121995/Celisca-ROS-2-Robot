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


# ---------------------------------------------------------------------------
# ROS 2 node wrappers
#
# The classes above are pure, deterministic algorithm cores (unit-testable
# without a ROS graph).  The wrappers below are what the console entry points
# actually run: real rclpy nodes that subscribe to the live sensor contract,
# apply the core, and publish the result, so selecting one of these in the
# GUI/CLI starts a node that genuinely participates in the pipeline.
# ---------------------------------------------------------------------------

def _marker_array_from_clusters(clusters, frame_id, stamp, ns):
    """One LINE_STRIP/POINTS marker per cluster for RViz."""
    array = MarkerArray()
    for index, cluster in enumerate(clusters):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = index
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = float(index % 3) / 2.0
        marker.pose.orientation.w = 1.0
        for point in cluster:
            p = Point32()
            p.x, p.y, p.z = float(point[0]), float(point[1]), 0.0
            marker.points.append(p)
        array.markers.append(marker)
    return array


class ObstacleDetectorNode(Node):
    """/scan -> proximity-clustered obstacles on /perception/obstacles."""

    def __init__(self, node_name='obstacle_detector'):
        super().__init__(node_name)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cluster_distance', 0.5)
        self.declare_parameter('markers_topic', '/perception/obstacles')
        self.detector = ObstacleDetector(
            cluster_distance=float(self.get_parameter('cluster_distance').value))
        self._pub = self.create_publisher(
            MarkerArray, self.get_parameter('markers_topic').value, 10)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value,
            self._on_scan, qos_profile_sensor_data)
        self.clusters = []
        self.get_logger().info('obstacle_detector ready')

    def _on_scan(self, msg):
        points = []
        for index, distance in enumerate(msg.ranges):
            if not math.isfinite(distance) or distance <= msg.range_min \
                    or distance >= msg.range_max:
                continue
            angle = msg.angle_min + index * msg.angle_increment
            points.append((distance * math.cos(angle),
                           distance * math.sin(angle)))
        self.clusters = self.detector.detect(points)
        self._pub.publish(_marker_array_from_clusters(
            self.clusters, msg.header.frame_id, msg.header.stamp, 'obstacles'))


class ScanClustererNode(Node):
    """/scan -> contiguous scan clusters on /perception/clusters."""

    def __init__(self, node_name='scan_clusterer'):
        super().__init__(node_name)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cluster_distance', 0.3)
        self.declare_parameter('markers_topic', '/perception/clusters')
        self.clusterer = ScanClusterer(
            cluster_distance=float(self.get_parameter('cluster_distance').value))
        self._pub = self.create_publisher(
            MarkerArray, self.get_parameter('markers_topic').value, 10)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value,
            self._on_scan, qos_profile_sensor_data)
        self.clusters = []
        self.get_logger().info('scan_clusterer ready')

    def _on_scan(self, msg):
        self.clusters = self.clusterer.cluster_ranges(
            msg.angle_min, msg.angle_increment, list(msg.ranges), msg.range_max)
        self._pub.publish(_marker_array_from_clusters(
            self.clusters, msg.header.frame_id, msg.header.stamp, 'clusters'))


class PointcloudSegmenterNode(Node):
    """PointCloud2 -> ground / non-ground clouds by height threshold."""

    def __init__(self, node_name='pointcloud_segmenter'):
        super().__init__(node_name)
        self.declare_parameter('points_topic', '/oakd/points')
        self.declare_parameter('ground_threshold', 0.1)
        self.declare_parameter('ground_topic', '/perception/ground')
        self.declare_parameter('obstacles_topic', '/perception/obstacle_cloud')
        self.segmenter = PointcloudSegmenter(
            ground_threshold=float(self.get_parameter('ground_threshold').value))
        self._ground_pub = self.create_publisher(
            PointCloud2, self.get_parameter('ground_topic').value, 10)
        self._object_pub = self.create_publisher(
            PointCloud2, self.get_parameter('obstacles_topic').value, 10)
        self.create_subscription(
            PointCloud2, self.get_parameter('points_topic').value,
            self._on_points, qos_profile_sensor_data)
        self.ground = []
        self.objects = []
        self.get_logger().info('pointcloud_segmenter ready')

    def _on_points(self, msg):
        try:
            from sensor_msgs_py import point_cloud2
        except ImportError:  # pragma: no cover - Humble ships this package
            self.get_logger().warn('sensor_msgs_py unavailable; idling')
            return
        points = [(float(p[0]), float(p[1]), float(p[2]))
                  for p in point_cloud2.read_points(
                      msg, field_names=('x', 'y', 'z'), skip_nans=True)]
        self.ground, self.objects = self.segmenter.segment(points)
        self._ground_pub.publish(
            point_cloud2.create_cloud_xyz32(msg.header, self.ground))
        self._object_pub.publish(
            point_cloud2.create_cloud_xyz32(msg.header, self.objects))


from ._runtime import run as _run, spin_node as _spin_node  # noqa: E402


def obstacle_detector_main(args=None):
    return _run(ObstacleDetectorNode, 'obstacle_detector', args=args)


def scan_clusterer_main(args=None):
    return _run(ScanClustererNode, 'scan_clusterer', args=args)


def pointcloud_segmenter_main(args=None):
    return _run(PointcloudSegmenterNode, 'pointcloud_segmenter', args=args)


if __name__ == '__main__':
    sys.exit(obstacle_detector_main())
