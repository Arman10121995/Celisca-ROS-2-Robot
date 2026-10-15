"""Feature-based RGB-D visual odometry (R7.2 feature_detection).

The registry's feature_detection entry named ORB-SLAM3, whose build here
carries an OpenCV ABI mismatch, so the id was cataloged only.  This module is
the lab's own front end, with OpenCV and NumPy only:

1. detect and describe features in each grey frame;
2. match them with a keyframe's (k = 2 nearest neighbours, Lowe ratio test);
3. lift the keyframe's matched features to 3D with its depth image;
4. solve the camera motion with PnP inside RANSAC.

The keyframe moves on when the camera has moved ``keyframe_m`` or turned
``keyframe_deg`` from it, or when fewer than ``keyframe_inliers`` matches
agree; at a standstill the pose therefore stays still instead of adding
frame-to-frame noise.  Five detectors are selectable:

  orb       ORB: FAST corners with oriented rBRIEF, Hamming distance
  akaze     AKAZE: nonlinear scale space with M-LDB, Hamming
  brisk     BRISK, Hamming
  sift      SIFT: difference of Gaussians, L2
  gftt_orb  Shi-Tomasi corners described with ORB, Hamming

The node publishes the base pose as nav_msgs/Odometry in its own frame
(``visual_odom``, identity at the first frame) and the keyframe's lifted
features as a PointCloud2; it does not publish TF.

Limits: frame-to-keyframe only (no bundle adjustment, no loop closure), so
drift grows with distance; textureless views lose tracking, and the pose
then holds until features return.
"""
import math

import numpy as np

DETECTORS = ('orb', 'akaze', 'brisk', 'sift', 'gftt_orb')


def make_frontend(name, max_features=1000):
    """(detector, extractor, norm) for one of DETECTORS."""
    import cv2
    if name == 'orb':
        orb = cv2.ORB_create(nfeatures=max_features)
        return orb, orb, cv2.NORM_HAMMING
    if name == 'akaze':
        akaze = cv2.AKAZE_create(threshold=0.0005)
        return akaze, akaze, cv2.NORM_HAMMING
    if name == 'brisk':
        brisk = cv2.BRISK_create(thresh=20)
        return brisk, brisk, cv2.NORM_HAMMING
    if name == 'sift':
        sift = cv2.SIFT_create(nfeatures=max_features)
        return sift, sift, cv2.NORM_L2
    if name == 'gftt_orb':
        return (cv2.GFTTDetector_create(maxCorners=max_features, qualityLevel=0.01, minDistance=5),
                cv2.ORB_create(nfeatures=max_features), cv2.NORM_HAMMING)
    raise ValueError('unknown detector %r (choose from %s)' % (name, ', '.join(DETECTORS)))


class FeatureTracker:
    """Detect, describe and match features with one detector."""

    def __init__(self, detector='orb', max_features=1000, ratio=0.8):
        import cv2
        self.name = detector
        self.max_features = int(max_features)
        self.ratio = float(ratio)
        self.detector, self.extractor, norm = make_frontend(detector, self.max_features)
        self.matcher = cv2.BFMatcher(norm)

    def features(self, gray):
        """(N x 2 pixel coordinates, descriptors or None)."""
        keypoints = self.detector.detect(gray, None)
        if len(keypoints) > self.max_features:
            keypoints = sorted(keypoints, key=lambda k: -k.response)[:self.max_features]
        keypoints, descriptors = self.extractor.compute(gray, keypoints)
        points = np.array([k.pt for k in keypoints], dtype=np.float64).reshape(-1, 2)
        return points, descriptors

    def match(self, query, train):
        """Index pairs (query, train) that pass the ratio test."""
        if query is None or train is None or len(query) < 2 or len(train) < 2:
            return np.empty((0, 2), dtype=int)
        good = []
        for pair in self.matcher.knnMatch(query, train, k=2):
            if len(pair) == 2 and pair[0].distance < self.ratio * pair[1].distance:
                good.append((pair[0].queryIdx, pair[0].trainIdx))
        return np.array(good, dtype=int).reshape(-1, 2)


def lift(points, depth, fx, fy, cx, cy, min_depth=0.1, max_depth=8.0):
    """Camera-frame 3D points of pixel coordinates (nearest depth pixel) and a
    mask of those with a valid depth."""
    u = np.round(points[:, 0]).astype(int)
    v = np.round(points[:, 1]).astype(int)
    inside = (u >= 0) & (u < depth.shape[1]) & (v >= 0) & (v < depth.shape[0])
    z = np.full(len(points), np.nan)
    z[inside] = depth[v[inside], u[inside]]
    valid = np.isfinite(z) & (z >= min_depth) & (z <= max_depth)
    z = np.where(valid, z, 0.0)
    xyz = np.column_stack([(points[:, 0] - cx) * z / fx, (points[:, 1] - cy) * z / fy, z])
    return xyz, valid


def rotation_angle(rotation):
    return math.acos(max(-1.0, min(1.0, (np.trace(rotation) - 1.0) / 2.0)))


class RgbdOdometry:
    """Camera pose from consecutive grey + depth frames (first frame = identity).

    ``step(gray, depth)`` returns (4 x 4 pose of the camera in the first
    camera frame, stats).  Stats: features, matches with depth, PnP inliers,
    tracked (bool) and keyframe (bool, a new keyframe was taken).
    """

    def __init__(self, intrinsics, detector='orb', max_features=1000, ratio=0.8, min_inliers=12,
                 reprojection_px=2.0, min_depth=0.1, max_depth=8.0, keyframe_m=0.25, keyframe_deg=10.0,
                 keyframe_inliers=40, max_step_m=0.5, max_step_deg=30.0):
        import cv2
        self._cv2 = cv2
        self.fx, self.fy, self.cx, self.cy = (float(v) for v in intrinsics)
        self.camera_matrix = np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]], dtype=np.float64)
        self.tracker = FeatureTracker(detector, max_features, ratio)
        self.min_inliers = int(min_inliers)
        self.reprojection_px = float(reprojection_px)
        self.min_depth, self.max_depth = float(min_depth), float(max_depth)
        self.keyframe_m, self.keyframe_rad = float(keyframe_m), math.radians(keyframe_deg)
        self.keyframe_inliers = int(keyframe_inliers)
        self.max_step_m, self.max_step_rad = float(max_step_m), math.radians(max_step_deg)
        self.pose = np.eye(4)
        self.keyframe = None   # (pose, xyz, valid, descriptors)
        self.since_keyframe = np.eye(4)
        self.frames = self.lost = 0

    def _set_keyframe(self, xyz, valid, descriptors):
        self.keyframe = (self.pose.copy(), xyz, valid, descriptors)
        self.since_keyframe = np.eye(4)

    def step(self, gray, depth):
        cv2 = self._cv2
        self.frames += 1
        points, descriptors = self.tracker.features(gray)
        xyz, valid = lift(points, depth, self.fx, self.fy, self.cx, self.cy, self.min_depth, self.max_depth)
        stats = {'features': len(points), 'matches': 0, 'inliers': 0, 'tracked': False, 'keyframe': False}
        if self.keyframe is None:
            self._set_keyframe(xyz, valid, descriptors)
            stats.update(tracked=True, keyframe=True)
            return self.pose.copy(), stats
        key_pose, key_xyz, key_valid, key_descriptors = self.keyframe
        pairs = self.tracker.match(key_descriptors, descriptors)
        if len(pairs):
            pairs = pairs[key_valid[pairs[:, 0]]]
        stats['matches'] = len(pairs)
        motion = None
        if len(pairs) >= self.min_inliers:
            found, rvec, tvec, inliers = cv2.solvePnPRansac(
                key_xyz[pairs[:, 0]], points[pairs[:, 1]], self.camera_matrix, None,
                iterationsCount=200, reprojectionError=self.reprojection_px, confidence=0.999)
            count = 0 if inliers is None else len(inliers)
            stats['inliers'] = count
            if found and count >= self.min_inliers:
                rotation, _ = cv2.Rodrigues(rvec)
                current_from_key = np.eye(4)
                current_from_key[:3, :3], current_from_key[:3, 3] = rotation, tvec.ravel()
                motion = np.linalg.inv(current_from_key)   # the camera's motion since the keyframe
                step = np.linalg.inv(self.since_keyframe) @ motion
                if (np.linalg.norm(step[:3, 3]) > self.max_step_m
                        or rotation_angle(step[:3, :3]) > self.max_step_rad):
                    motion = None   # implausible jump: a bad RANSAC consensus
        if motion is None:
            # Lost: hold the pose and start again from this frame.
            self.lost += 1
            self._set_keyframe(xyz, valid, descriptors)
            stats['keyframe'] = True
            return self.pose.copy(), stats
        self.pose = key_pose @ motion
        self.since_keyframe = motion
        stats['tracked'] = True
        if (np.linalg.norm(motion[:3, 3]) > self.keyframe_m or rotation_angle(motion[:3, :3]) > self.keyframe_rad
                or stats['inliers'] < self.keyframe_inliers):
            self._set_keyframe(xyz, valid, descriptors)
            stats['keyframe'] = True
        return self.pose.copy(), stats

    def keyframe_points(self):
        """The keyframe's lifted features (camera frame)."""
        if self.keyframe is None:
            return np.empty((0, 3))
        return self.keyframe[1][self.keyframe[2]]


def base_motion(camera_pose, base_from_camera):
    """Pose of the base in its first-frame base frame, from the camera's pose
    in its first frame and the fixed base <- camera transform."""
    return base_from_camera @ camera_pose @ np.linalg.inv(base_from_camera)


def transform_matrix(translation, quaternion_xyzw):
    """4 x 4 matrix of a translation and an (x, y, z, w) quaternion."""
    x, y, z, w = quaternion_xyzw
    matrix = np.eye(4)
    matrix[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    matrix[:3, 3] = translation
    return matrix


def matrix_quaternion(rotation):
    """(x, y, z, w) of a rotation matrix."""
    m = rotation
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0:
        s = 2.0 * math.sqrt(trace + 1.0)
        return ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s)
    i = int(np.argmax([m[0, 0], m[1, 1], m[2, 2]]))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * math.sqrt(max(1e-12, 1.0 + m[i, i] - m[j, j] - m[k, k]))
    q = [0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (m[j, i] + m[i, j]) / s
    q[k] = (m[k, i] + m[i, k]) / s
    return (q[0], q[1], q[2], (m[k, j] - m[j, k]) / s)


def decode_gray(msg):
    """sensor_msgs/Image (rgb8, bgr8, rgba8, bgra8, mono8) as a uint8 grey image."""
    channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}.get(msg.encoding)
    if channels is None:
        raise ValueError('unsupported colour encoding %s' % msg.encoding)
    rows = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)
    image = rows[:, :msg.width * channels].reshape(msg.height, msg.width, channels).astype(np.float32)
    if channels == 1:
        return image[:, :, 0].astype(np.uint8)
    red, blue = (0, 2) if msg.encoding.startswith('rgb') else (2, 0)
    gray = 0.299 * image[:, :, red] + 0.587 * image[:, :, 1] + 0.114 * image[:, :, blue]
    return np.clip(gray + 0.5, 0, 255).astype(np.uint8)


try:
    from rclpy.node import Node
except ImportError:  # pragma: no cover - the cores above work without ROS
    Node = object


class FeatureOdometryNode(Node):
    """RGB + depth + camera info -> visual odometry of the base."""

    def __init__(self, node_name='feature_detection'):
        super().__init__(node_name)
        from nav_msgs.msg import Odometry
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, Image, PointCloud2
        from tf2_ros import Buffer, TransformListener
        for name, value in (('rgb_topic', '/oakd/rgb/image_raw'), ('depth_topic', '/oakd/depth/image_raw'),
                            ('info_topic', '/oakd/rgb/camera_info'), ('detector', 'orb'),
                            ('max_features', 1000), ('base_frame', 'base_footprint'),
                            ('odom_frame', 'visual_odom'), ('odom_topic', '/perception/visual_odometry'),
                            ('features_topic', '/perception/features')):
            self.declare_parameter(name, value)
        param = lambda name: self.get_parameter(name).value  # noqa: E731
        self.detector, self.max_features = str(param('detector')), int(param('max_features'))
        if self.detector not in DETECTORS:
            raise ValueError('detector must be one of %s' % ', '.join(DETECTORS))
        self.base_frame, self.odom_frame = str(param('base_frame')), str(param('odom_frame'))
        self.odometry = None
        self.intrinsics = None
        self.base_from_camera = None
        self.pending = {}
        self.frames = 0
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._odom_pub = self.create_publisher(Odometry, param('odom_topic'), 10)
        self._features_pub = self.create_publisher(PointCloud2, param('features_topic'), 5)
        self.create_subscription(CameraInfo, param('info_topic'), self._on_info, qos_profile_sensor_data)
        self.create_subscription(Image, param('rgb_topic'), lambda m: self._on_image('rgb', m),
                                 qos_profile_sensor_data)
        self.create_subscription(Image, param('depth_topic'), lambda m: self._on_image('depth', m),
                                 qos_profile_sensor_data)
        self.get_logger().info('feature_detection ready (%s, %d features)' % (self.detector, self.max_features))

    def _on_info(self, msg):
        if msg.k[0] > 0 and msg.k[4] > 0:
            self.intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def _on_image(self, kind, msg):
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        entry = self.pending.setdefault(key, {})
        entry[kind] = msg
        for old in sorted(self.pending)[:-10]:
            self.pending.pop(old)
        if 'rgb' in entry and 'depth' in entry:
            self.pending.pop(key)
            self._process(entry['rgb'], entry['depth'])

    def _camera_transform(self, frame):
        from rclpy.time import Time
        try:
            t = self._tf_buffer.lookup_transform(self.base_frame, frame, Time())
        except Exception as exc:
            self.get_logger().warn('no transform %s -> %s yet: %s' % (frame, self.base_frame, exc),
                                   throttle_duration_sec=5.0)
            return None
        q, p = t.transform.rotation, t.transform.translation
        return transform_matrix((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))

    def _process(self, rgb, depth_msg):
        from nav_msgs.msg import Odometry
        from sensor_msgs_py import point_cloud2
        from .perception import decode_depth
        if self.intrinsics is None:
            self.get_logger().warn('no camera info yet', throttle_duration_sec=5.0)
            return
        if self.base_from_camera is None:
            self.base_from_camera = self._camera_transform(depth_msg.header.frame_id)
            if self.base_from_camera is None:
                return
        if self.odometry is None:
            self.odometry = RgbdOdometry(self.intrinsics, self.detector, self.max_features)
        try:
            gray, depth = decode_gray(rgb), decode_depth(depth_msg)
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=5.0)
            return
        camera_pose, stats = self.odometry.step(gray, depth)
        self.frames += 1
        base = base_motion(camera_pose, self.base_from_camera)
        out = Odometry()
        out.header.stamp = depth_msg.header.stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = (float(v) for v in base[:3, 3])
        q = matrix_quaternion(base[:3, :3])
        o = out.pose.pose.orientation
        o.x, o.y, o.z, o.w = (float(v) for v in q)
        if not stats['tracked']:
            out.pose.covariance[0] = out.pose.covariance[7] = out.pose.covariance[35] = 1e3
        self._odom_pub.publish(out)
        if stats['keyframe']:
            self._features_pub.publish(point_cloud2.create_cloud_xyz32(
                depth_msg.header, self.odometry.keyframe_points().astype(np.float32)))
        if self.frames % 300 == 0:
            self.get_logger().info('%d frames, %d lost; last: %d features, %d matches, %d inliers' % (
                self.odometry.frames, self.odometry.lost, stats['features'], stats['matches'], stats['inliers']))


def feature_detection_main(args=None):
    from ._runtime import run
    return run(FeatureOdometryNode, 'feature_detection', args=args)
