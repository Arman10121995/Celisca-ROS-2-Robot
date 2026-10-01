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


class EuclideanClusterer:
    """Euclidean distance-based clustering for 3D point clouds."""

    def __init__(self, node_name='euclidean_clusterer', tolerance=0.1, min_points=5):
        self.node_name = node_name
        self.tolerance = tolerance
        self.min_points = min_points

    def cluster(self, points_xyz):
        """Cluster 3D points using Euclidean distance threshold.
        
        Returns list of clusters, each a list of (x, y, z) points.
        """
        if not points_xyz:
            return []
        
        clusters = []
        visited = set()
        
        for i, point in enumerate(points_xyz):
            if i in visited:
                continue
            
            # Start new cluster with this point
            cluster = [point]
            queue = [i]
            visited.add(i)
            
            while queue:
                current_idx = queue.pop(0)
                current_point = points_xyz[current_idx]
                
                # Find neighbors within tolerance
                for j, other_point in enumerate(points_xyz):
                    if j in visited:
                        continue
                    
                    distance = math.sqrt(
                        (current_point[0] - other_point[0])**2 + 
                        (current_point[1] - other_point[1])**2 + 
                        (current_point[2] - other_point[2])**2
                    )
                    
                    if distance <= self.tolerance:
                        cluster.append(other_point)
                        queue.append(j)
                        visited.add(j)
            
            # Only keep clusters with enough points
            if len(cluster) >= self.min_points:
                clusters.append(cluster)
        
        return clusters


class DBSCANClusterer:
    """DBSCAN (Density-Based Spatial Clustering of Applications with Noise) clustering."""

    def __init__(self, node_name='dbscan_clusterer', eps=0.1, min_samples=5):
        self.node_name = node_name
        self.eps = eps
        self.min_samples = min_samples

    def cluster(self, points_xyz):
        """Cluster 3D points using DBSCAN algorithm.
        
        Returns tuple of (clusters, noise) where clusters is a list of point clusters
        and noise is a list of points not belonging to any cluster.
        """
        if not points_xyz:
            return [], []
        
        # Find neighbors for each point
        neighbors = []
        for i, point in enumerate(points_xyz):
            point_neighbors = []
            for j, other_point in enumerate(points_xyz):
                if i == j:
                    continue
                distance = math.sqrt(
                    (point[0] - other_point[0])**2 + 
                    (point[1] - other_point[1])**2 + 
                    (point[2] - other_point[2])**2
                )
                if distance <= self.eps:
                    point_neighbors.append(j)
            neighbors.append(point_neighbors)
        
        # DBSCAN algorithm
        cluster_id = 0
        clusters = []
        noise = []
        labels = [-1] * len(points_xyz)  # -1 means unclassified
        
        for i in range(len(points_xyz)):
            if labels[i] != -1:  # Already classified
                continue
            
            # Check if this is a core point
            if len(neighbors[i]) >= self.min_samples:
                # Start a new cluster
                cluster = []
                queue = [i]
                labels[i] = cluster_id
                
                while queue:
                    current_idx = queue.pop(0)
                    cluster.append(points_xyz[current_idx])
                    
                    # Expand to all density-reachable points
                    for neighbor_idx in neighbors[current_idx]:
                        if labels[neighbor_idx] == -1:  # Unclassified
                            labels[neighbor_idx] = cluster_id
                            queue.append(neighbor_idx)
                        elif labels[neighbor_idx] == -2:  # Noise point that becomes border point
                            labels[neighbor_idx] = cluster_id
                            queue.append(neighbor_idx)
                
                clusters.append(cluster)
                cluster_id += 1
            else:
                # This is a noise point (for now)
                labels[i] = -2  # Mark as potential noise
        
        # Collect noise points
        for i, label in enumerate(labels):
            if label == -2:  # Noise
                noise.append(points_xyz[i])
        
        return clusters, noise


class RANSACGroundRemoval:
    """RANSAC-based ground removal and obstacle segmentation from point clouds."""

    def __init__(self, node_name='ransac_ground_removal', max_iterations=100, 
                 distance_threshold=0.05, normal_threshold=0.85):
        self.node_name = node_name
        self.max_iterations = max_iterations
        self.distance_threshold = distance_threshold
        self.normal_threshold = normal_threshold

    def segment(self, points_xyz):
        """Segment 3D points into ground and obstacles using RANSAC plane fitting.
        
        Returns tuple of (ground_points, obstacle_points).
        """
        if not points_xyz or len(points_xyz) < 3:
            return [], points_xyz
        
        best_inliers = []
        best_plane = None
        
        # RANSAC plane fitting
        for _ in range(self.max_iterations):
            # Randomly select 3 points to define a plane
            import random
            rng = random.Random()
            indices = rng.sample(range(len(points_xyz)), min(3, len(points_xyz)))
            
            if len(indices) < 3:
                continue
                
            p1, p2, p3 = [points_xyz[i] for i in indices]
            
            # Calculate plane normal using cross product
            v1 = (p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2])
            v2 = (p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2])
            
            # Cross product
            normal_x = v1[1] * v2[2] - v1[2] * v2[1]
            normal_y = v1[2] * v2[0] - v1[0] * v2[2] 
            normal_z = v1[0] * v2[1] - v1[1] * v2[0]
            
            normal_length = math.sqrt(normal_x**2 + normal_y**2 + normal_z**2)
            if normal_length < 1e-10:
                continue  # Degenerate plane
            
            # Normalize normal
            normal = (normal_x / normal_length, normal_y / normal_length, normal_z / normal_length)
            
            # Plane equation: normal · (p - p1) = 0 => normal · p = normal · p1
            plane_d = normal[0] * p1[0] + normal[1] * p1[1] + normal[2] * p1[2]
            
            # Count inliers
            inliers = []
            for j, point in enumerate(points_xyz):
                distance = abs(normal[0] * point[0] + normal[1] * point[1] + normal[2] * point[2] - plane_d)
                if distance <= self.distance_threshold:
                    inliers.append(j)
            
            # Check if this is the best plane so far
            if len(inliers) > len(best_inliers):
                best_inliers = inliers
                best_plane = (normal, plane_d)
        
        if best_plane is None:
            # No plane found, return all as obstacles
            return [], points_xyz
        
        # Check if the best plane is likely ground (normal should be close to (0,0,1))
        normal, plane_d = best_plane
        if abs(normal[2]) < self.normal_threshold:  # Not a horizontal plane
            return [], points_xyz
        
        # Separate ground and obstacles
        ground_points = []
        obstacle_points = []
        
        for i, point in enumerate(points_xyz):
            if i in best_inliers:
                ground_points.append(point)
            else:
                obstacle_points.append(point)
        
        return ground_points, obstacle_points


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


class EuclideanClustererNode(Node):
    """PointCloud2 -> Euclidean clustered obstacles on /perception/euclidean_clusters."""

    def __init__(self, node_name='euclidean_clusterer'):
        super().__init__(node_name)
        self.declare_parameter('points_topic', '/oakd/points')
        self.declare_parameter('tolerance', 0.1)
        self.declare_parameter('min_points', 5)
        self.declare_parameter('markers_topic', '/perception/euclidean_clusters')
        self.clusterer = EuclideanClusterer(
            tolerance=float(self.get_parameter('tolerance').value),
            min_points=int(self.get_parameter('min_points').value))
        self._pub = self.create_publisher(
            MarkerArray, self.get_parameter('markers_topic').value, 10)
        self.create_subscription(
            PointCloud2, self.get_parameter('points_topic').value,
            self._on_points, qos_profile_sensor_data)
        self.clusters = []
        self.get_logger().info('euclidean_clusterer ready')

    def _on_points(self, msg):
        try:
            from sensor_msgs_py import point_cloud2
        except ImportError:  # pragma: no cover - Humble ships this package
            self.get_logger().warn('sensor_msgs_py unavailable; idling')
            return
        points = [(float(p[0]), float(p[1]), float(p[2]))
                  for p in point_cloud2.read_points(
                      msg, field_names=('x', 'y', 'z'), skip_nans=True)]
        self.clusters = self.clusterer.cluster(points)
        self._pub.publish(_marker_array_from_clusters(
            self.clusters, msg.header.frame_id, msg.header.stamp, 'euclidean'))


class DBSCANClustererNode(Node):
    """PointCloud2 -> DBSCAN clustered obstacles on /perception/dbscan_clusters."""

    def __init__(self, node_name='dbscan_clusterer'):
        super().__init__(node_name)
        self.declare_parameter('points_topic', '/oakd/points')
        self.declare_parameter('eps', 0.1)
        self.declare_parameter('min_samples', 5)
        self.declare_parameter('markers_topic', '/perception/dbscan_clusters')
        self.clusterer = DBSCANClusterer(
            eps=float(self.get_parameter('eps').value),
            min_samples=int(self.get_parameter('min_samples').value))
        self._pub = self.create_publisher(
            MarkerArray, self.get_parameter('markers_topic').value, 10)
        self.create_subscription(
            PointCloud2, self.get_parameter('points_topic').value,
            self._on_points, qos_profile_sensor_data)
        self.clusters = []
        self.get_logger().info('dbscan_clusterer ready')

    def _on_points(self, msg):
        try:
            from sensor_msgs_py import point_cloud2
        except ImportError:  # pragma: no cover - Humble ships this package
            self.get_logger().warn('sensor_msgs_py unavailable; idling')
            return
        points = [(float(p[0]), float(p[1]), float(p[2]))
                  for p in point_cloud2.read_points(
                      msg, field_names=('x', 'y', 'z'), skip_nans=True)]
        clusters, _ = self.clusterer.cluster(points)
        self.clusters = clusters
        self._pub.publish(_marker_array_from_clusters(
            self.clusters, msg.header.frame_id, msg.header.stamp, 'dbscan'))


class RANSACGroundRemovalNode(Node):
    """PointCloud2 -> RANSAC ground/obstacle segmentation on /perception/ransac_ground and /perception/ransac_obstacles."""

    def __init__(self, node_name='ransac_ground_removal'):
        super().__init__(node_name)
        self.declare_parameter('points_topic', '/oakd/points')
        self.declare_parameter('max_iterations', 100)
        self.declare_parameter('distance_threshold', 0.05)
        self.declare_parameter('normal_threshold', 0.85)
        self.declare_parameter('ground_topic', '/perception/ransac_ground')
        self.declare_parameter('obstacles_topic', '/perception/ransac_obstacles')
        self.segmenter = RANSACGroundRemoval(
            max_iterations=int(self.get_parameter('max_iterations').value),
            distance_threshold=float(self.get_parameter('distance_threshold').value),
            normal_threshold=float(self.get_parameter('normal_threshold').value))
        self._ground_pub = self.create_publisher(
            PointCloud2, self.get_parameter('ground_topic').value, 10)
        self._object_pub = self.create_publisher(
            PointCloud2, self.get_parameter('obstacles_topic').value, 10)
        self.create_subscription(
            PointCloud2, self.get_parameter('points_topic').value,
            self._on_points, qos_profile_sensor_data)
        self.ground = []
        self.objects = []
        self.get_logger().info('ransac_ground_removal ready')

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


def euclidean_clusterer_main(args=None):
    return _run(EuclideanClustererNode, 'euclidean_clusterer', args=args)


def dbscan_clusterer_main(args=None):
    return _run(DBSCANClustererNode, 'dbscan_clusterer', args=args)


def ransac_ground_removal_main(args=None):
    return _run(RANSACGroundRemovalNode, 'ransac_ground_removal', args=args)


if __name__ == '__main__':
    sys.exit(obstacle_detector_main())
