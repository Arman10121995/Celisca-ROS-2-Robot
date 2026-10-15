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


def _components(points, radius):
    """Connected components of points under a distance threshold (KD-tree BFS)."""
    import numpy as np
    from scipy.spatial import cKDTree
    pts = np.asarray(points, dtype=float)
    if len(pts) == 0:
        return []
    tree = cKDTree(pts)
    neighbours = tree.query_ball_point(pts, radius)
    label = np.full(len(pts), -1)
    groups = []
    for i in range(len(pts)):
        if label[i] >= 0:
            continue
        label[i] = len(groups)
        frontier, members = [i], [i]
        while frontier:
            j = frontier.pop()
            for k in neighbours[j]:
                if label[k] < 0:
                    label[k] = label[i]
                    frontier.append(k)
                    members.append(k)
        groups.append(members)
    return groups


class ObstacleDetector:
    """Group 2D points into clusters by proximity (single linkage, KD-tree)."""

    def __init__(self, node_name='obstacle_detector', cluster_distance=0.5):
        self.node_name = node_name
        self.cluster_distance = cluster_distance

    def detect(self, points):
        """Group 2D points [(x, y), ...] into clusters by proximity."""
        if not points:
            return []
        return [[tuple(points[i]) for i in group] for group in _components(points, self.cluster_distance)]


class ScanClusterer:
    """Segment a LaserScan into clusters at range breakpoints.

    Consecutive beams belong to one cluster while their points are closer
    than ``cluster_distance``; with ``adaptive=True`` the threshold grows
    with range (Borges and Aldon's adaptive breakpoint detector), so far
    surfaces seen at grazing angles are not split.
    """

    def __init__(self, node_name='scan_clusterer', cluster_distance=0.3, adaptive=False,
                 lambda_angle=math.radians(10), sigma=0.03):
        self.node_name = node_name
        self.cluster_distance = cluster_distance
        self.adaptive = adaptive
        self.lambda_angle = lambda_angle
        self.sigma = sigma

    def cluster_ranges(self, angle_min, angle_increment, ranges, max_range):
        """Return list of clusters, each a list of (x, y) cartesian points."""
        clusters = []
        current = []
        last_r = None
        for i, r in enumerate(ranges):
            if r >= max_range or r <= 0.0 or r != r:
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
                limit = self.cluster_distance
                if self.adaptive and last_r is not None:
                    limit = max(limit, last_r * math.sin(abs(angle_increment)) /
                                math.sin(self.lambda_angle - abs(angle_increment)) + 3 * self.sigma)
                if math.hypot(x - px, y - py) > limit:
                    clusters.append(current)
                    current = [(x, y)]
                else:
                    current.append((x, y))
            last_r = r
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
    """Euclidean clustering (PCL's region growing): connected components under
    a distance tolerance, keeping clusters of at least ``min_points``."""

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
        return [[tuple(points_xyz[i]) for i in group] for group in _components(points_xyz, self.tolerance)
                if len(group) >= self.min_points]


class DBSCANClusterer:
    """DBSCAN (Ester et al. 1996) with KD-tree neighbourhoods."""

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
        import numpy as np
        from scipy.spatial import cKDTree
        pts = np.asarray(points_xyz, dtype=float)
        neighbours = cKDTree(pts).query_ball_point(pts, self.eps)
        # A core point has at least min_samples points, itself included, within
        # eps; clusters grow only through core points (border points join but
        # do not extend the cluster).
        core = np.array([len(n) >= self.min_samples for n in neighbours])
        labels = np.full(len(pts), -1)
        clusters = []
        for i in range(len(pts)):
            if labels[i] != -1 or not core[i]:
                continue
            cluster_id = len(clusters)
            labels[i] = cluster_id
            queue, members = [i], []
            while queue:
                j = queue.pop()
                members.append(j)
                if not core[j]:
                    continue
                for k in neighbours[j]:
                    if labels[k] == -1:
                        labels[k] = cluster_id
                        queue.append(k)
            clusters.append([tuple(points_xyz[j]) for j in sorted(members)])
        noise = [tuple(points_xyz[i]) for i in range(len(pts)) if labels[i] == -1]
        return clusters, noise


class RANSACGroundRemoval:
    """RANSAC plane fit for the ground, refined by least squares on its inliers.

    Hypotheses whose normal is further than ``normal_threshold`` (cosine) from
    vertical are skipped, so walls are not taken for the floor.
    """

    def __init__(self, node_name='ransac_ground_removal', max_iterations=100,
                 distance_threshold=0.05, normal_threshold=0.85, seed=0):
        self.node_name = node_name
        self.max_iterations = max_iterations
        self.distance_threshold = distance_threshold
        self.normal_threshold = normal_threshold
        self.seed = seed

    def segment(self, points_xyz):
        """Segment 3D points into ground and obstacles using RANSAC plane fitting.

        Returns tuple of (ground_points, obstacle_points).
        """
        if not points_xyz or len(points_xyz) < 3:
            return [], list(points_xyz)
        import numpy as np
        pts = np.asarray(points_xyz, dtype=float)
        rng = np.random.default_rng(self.seed)
        best = None
        best_count = 0
        for _ in range(self.max_iterations):
            p1, p2, p3 = pts[rng.choice(len(pts), 3, replace=False)]
            normal = np.cross(p2 - p1, p3 - p1)
            length = np.linalg.norm(normal)
            if length < 1e-10:
                continue
            normal = normal / length
            if abs(normal[2]) < self.normal_threshold:
                continue
            inliers = np.abs((pts - p1) @ normal) <= self.distance_threshold
            count = int(inliers.sum())
            if count > best_count:
                best, best_count = inliers, count
        if best is None or best_count < 3:
            return [], list(points_xyz)
        # Least-squares refinement z = a x + b y + c on the inliers.
        inlier_pts = pts[best]
        A = np.column_stack([inlier_pts[:, 0], inlier_pts[:, 1], np.ones(len(inlier_pts))])
        a, b, c = np.linalg.lstsq(A, inlier_pts[:, 2], rcond=None)[0]
        normal = np.array([-a, -b, 1.0]) / math.sqrt(a * a + b * b + 1.0)
        distance = np.abs((pts - np.array([0.0, 0.0, c])) @ normal)
        ground = distance <= self.distance_threshold
        return ([tuple(points_xyz[i]) for i in np.nonzero(ground)[0]],
                [tuple(points_xyz[i]) for i in np.nonzero(~ground)[0]])


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
        self.declare_parameter('adaptive', False)
        self.declare_parameter('markers_topic', '/perception/clusters')
        self.clusterer = ScanClusterer(
            cluster_distance=float(self.get_parameter('cluster_distance').value),
            adaptive=bool(self.get_parameter('adaptive').value))
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


def quaternion_matrix(x, y, z, w):
    """3x3 rotation of a unit quaternion."""
    import numpy as np
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def upright_points(points, rotation, translation, max_points=0):
    """Points moved into a z-up frame (``p @ R.T + t``), every k-th point
    kept so at most ``max_points`` remain (0: all)."""
    import numpy as np
    pts = np.asarray(points, dtype=float).reshape(-1, 3)
    if max_points and len(pts) > max_points:
        pts = pts[::int(math.ceil(len(pts) / float(max_points)))]
    return pts @ np.asarray(rotation, dtype=float).T + np.asarray(translation, dtype=float)


class _UprightCloud:
    """Ground segmentation needs z up.  A depth camera publishes in its
    optical frame (z forward, y down): the height threshold then cut by
    depth and RANSAC's near-vertical normal test picked walls facing the
    camera (found 2026-10-15).  Clouds are moved into ``target_frame``
    (default base_footprint) first, and thinned to ``max_points``."""

    def _setup_upright(self, default_frame='base_footprint'):
        self.declare_parameter('target_frame', default_frame)
        self.declare_parameter('max_points', 8000)
        self.target_frame = str(self.get_parameter('target_frame').value)
        self.max_points = int(self.get_parameter('max_points').value)
        from tf2_ros import Buffer, TransformListener
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    def _upright(self, msg, points):
        """(points as a list of tuples in target_frame, header) or (None, None)."""
        from rclpy.time import Time
        header = msg.header
        if not self.target_frame or header.frame_id == self.target_frame:
            return [tuple(p) for p in upright_points(points, [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                                                     [0, 0, 0], self.max_points)], header
        try:
            transform = self._tf_buffer.lookup_transform(self.target_frame, header.frame_id, Time())
        except Exception as exc:
            self.get_logger().warn('no transform %s -> %s yet: %s' % (
                header.frame_id, self.target_frame, exc), throttle_duration_sec=5.0)
            return None, None
        q, t = transform.transform.rotation, transform.transform.translation
        moved = upright_points(points, quaternion_matrix(q.x, q.y, q.z, q.w), (t.x, t.y, t.z),
                               self.max_points)
        from std_msgs.msg import Header
        out = Header()
        out.stamp = header.stamp
        out.frame_id = self.target_frame
        return [tuple(p) for p in moved], out


class PointcloudSegmenterNode(_UprightCloud, Node):
    """PointCloud2 -> ground / non-ground clouds by height threshold."""

    def __init__(self, node_name='pointcloud_segmenter'):
        super().__init__(node_name)
        self.declare_parameter('points_topic', '/oakd/points')
        self.declare_parameter('ground_threshold', 0.1)
        self.declare_parameter('ground_topic', '/perception/ground')
        self.declare_parameter('obstacles_topic', '/perception/obstacle_cloud')
        self.segmenter = PointcloudSegmenter(
            ground_threshold=float(self.get_parameter('ground_threshold').value))
        self._setup_upright()
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
        points, header = self._upright(msg, points)
        if points is None:
            return
        self.ground, self.objects = self.segmenter.segment(points)
        self._ground_pub.publish(
            point_cloud2.create_cloud_xyz32(header, self.ground))
        self._object_pub.publish(
            point_cloud2.create_cloud_xyz32(header, self.objects))


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


class RANSACGroundRemovalNode(_UprightCloud, Node):
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
        self._setup_upright()
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
        points, header = self._upright(msg, points)
        if points is None:
            return
        self.ground, self.objects = self.segmenter.segment(points)
        self._ground_pub.publish(
            point_cloud2.create_cloud_xyz32(header, self.ground))
        self._object_pub.publish(
            point_cloud2.create_cloud_xyz32(header, self.objects))


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
