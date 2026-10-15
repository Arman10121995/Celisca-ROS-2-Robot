"""Chessboard camera calibration (R7.2 camera_calibration).

The registry's camera_calibration entry named ROS's interactive
cameracalibrator.py, which is not installed here, so the id was cataloged
only.  This module is the same method with OpenCV and NumPy only (Zhang's
planar-target calibration, as cv2.calibrateCamera implements it):

1. find the inner corners of a chessboard in each grey frame and refine them
   to sub-pixel accuracy;
2. keep a view only when it differs from every kept view in board position,
   size or tilt (the coverage that cameracalibrator.py asks the operator
   for), up to ``max_views``;
3. once ``min_views`` are kept, solve for the camera matrix and the
   plumb_bob distortion and report the RMS reprojection error.

The node runs beside any camera, publishes the calibrated
sensor_msgs/CameraInfo (latched) and writes it as a ROS camera-info YAML,
and logs how it differs from the camera's own camera info.  A board must be
in view: in simulation that needs a chessboard model in the world.
"""
import math

import numpy as np


def parse_pattern(text):
    """'8x6' -> (8, 6) inner corners per row and column."""
    columns, rows = (int(v) for v in str(text).lower().split('x'))
    if columns < 3 or rows < 3:
        raise ValueError('a chessboard needs at least 3 x 3 inner corners')
    return columns, rows


def board_points(pattern, square):
    """Inner-corner positions on the board plane (z = 0), row by row."""
    columns, rows = pattern
    grid = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2) * float(square)
    return np.column_stack([grid, np.zeros(len(grid))]).astype(np.float32)


def find_board(gray, pattern):
    """Sub-pixel inner corners (N x 2) of the chessboard, or None."""
    import cv2
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
    found, corners = cv2.findChessboardCorners(gray, pattern, flags=flags)
    if not found:
        return None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.001)
    corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
    return corners.reshape(-1, 2)


def view_signature(corners, pattern, image_size):
    """(x, y, size, skew) in [0, 1] of a detected board, as cameracalibrator.py
    measures coverage: centre position, apparent size and tilt."""
    width, height = image_size
    columns, rows = pattern
    grid = corners.reshape(rows, columns, 2)
    outer = np.array([grid[0, 0], grid[0, -1], grid[-1, -1], grid[-1, 0]])
    centre = outer.mean(axis=0)
    d1, d2 = outer[2] - outer[0], outer[3] - outer[1]
    area = 0.5 * abs(float(d1[0] * d2[1] - d1[1] * d2[0]))
    size = math.sqrt(area / float(width * height))
    # Skew: how far the top-left corner angle is from a right angle.
    a, b = outer[1] - outer[0], outer[3] - outer[0]
    angle = math.acos(max(-1.0, min(1.0, float(a @ b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-12))))
    skew = min(1.0, 2.0 * abs(math.pi / 2 - angle))
    return np.array([centre[0] / width, centre[1] / height, size, skew])


class ChessboardCalibrator:
    """Collect distinct chessboard views and calibrate from them."""

    def __init__(self, pattern=(8, 6), square=0.05, min_views=15, max_views=40, min_difference=0.1):
        self.pattern = tuple(pattern)
        self.square = float(square)
        self.min_views, self.max_views = int(min_views), int(max_views)
        self.min_difference = float(min_difference)
        self.views, self.signatures = [], []
        self.image_size = None
        self.result = None

    def add(self, gray):
        """Look for the board; keep the view if it is new.  Returns
        'kept', 'similar', 'no board' or 'full'."""
        if len(self.views) >= self.max_views:
            return 'full'
        corners = find_board(gray, self.pattern)
        if corners is None:
            return 'no board'
        size = (gray.shape[1], gray.shape[0])
        if self.image_size is not None and size != self.image_size:
            raise ValueError('image size changed from %s to %s' % (self.image_size, size))
        self.image_size = size
        signature = view_signature(corners, self.pattern, size)
        if self.signatures and min(np.abs(signature - s).max() for s in self.signatures) < self.min_difference:
            return 'similar'
        self.views.append(corners.astype(np.float32))
        self.signatures.append(signature)
        return 'kept'

    def ready(self):
        return len(self.views) >= self.min_views

    def coverage(self):
        """Spread of the kept views: (x, y, size, skew) ranges in [0, 1]."""
        if not self.signatures:
            return (0.0, 0.0, 0.0, 0.0)
        s = np.array(self.signatures)
        return tuple(float(v) for v in s.max(axis=0) - s.min(axis=0))

    def calibrate(self):
        """Camera matrix, distortion (k1, k2, p1, p2, k3) and RMS reprojection error (px)."""
        import cv2
        if not self.views:
            raise ValueError('no chessboard views')
        objects = [board_points(self.pattern, self.square)] * len(self.views)
        images = [v.reshape(-1, 1, 2) for v in self.views]
        rms, matrix, distortion, _, _ = cv2.calibrateCamera(objects, images, self.image_size, None, None)
        self.result = {'camera_matrix': matrix, 'distortion': distortion.ravel()[:5], 'rms_px': float(rms),
                       'views': len(self.views), 'image_size': self.image_size}
        return self.result


def camera_info_yaml(name, result):
    """A ROS camera-info YAML document (camera_calibration_parsers format)."""
    width, height = result['image_size']
    k = [float(v) for v in result['camera_matrix'].ravel()]
    d = [float(v) for v in result['distortion']]
    p = [k[0], k[1], k[2], 0.0, k[3], k[4], k[5], 0.0, 0.0, 0.0, 1.0, 0.0]

    def matrix(rows, cols, data):
        return 'rows: %d\n  cols: %d\n  data: [%s]' % (rows, cols, ', '.join('%.8g' % v for v in data))
    return ('image_width: %d\nimage_height: %d\ncamera_name: %s\ncamera_matrix:\n  %s\n'
            'distortion_model: plumb_bob\ndistortion_coefficients:\n  %s\nrectification_matrix:\n  %s\n'
            'projection_matrix:\n  %s\n' % (width, height, name, matrix(3, 3, k), matrix(1, 5, d),
                                           matrix(3, 3, [1, 0, 0, 0, 1, 0, 0, 0, 1]), matrix(3, 4, p)))


def decode_gray(msg):
    from .visual_odometry import decode_gray as decode
    return decode(msg)


try:
    from rclpy.node import Node
except ImportError:  # pragma: no cover - the cores above work without ROS
    Node = object


class CameraCalibrationNode(Node):
    """Camera images -> calibrated CameraInfo once enough board views are kept."""

    def __init__(self, node_name='camera_calibration'):
        super().__init__(node_name)
        from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, Image
        for name, value in (('image_topic', '/oakd/rgb/image_raw'), ('info_topic', '/oakd/rgb/camera_info'),
                            ('output_topic', '/perception/calibrated_camera_info'), ('pattern', '8x6'),
                            ('square', 0.05), ('min_views', 15), ('max_views', 40), ('rate', 2.0),
                            ('output_path', '')):
            self.declare_parameter(name, value)
        param = lambda name: self.get_parameter(name).value  # noqa: E731
        self.calibrator = ChessboardCalibrator(parse_pattern(param('pattern')), float(param('square')),
                                               int(param('min_views')), int(param('max_views')))
        self.period = 1.0 / max(0.1, float(param('rate')))
        self.output_path = str(param('output_path'))
        self.published_info = None
        self.last_try = -math.inf
        self.calibrated = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(CameraInfo, param('output_topic'), latched)
        self.create_subscription(CameraInfo, param('info_topic'), self._on_info, qos_profile_sensor_data)
        self.create_subscription(Image, param('image_topic'), self._on_image, qos_profile_sensor_data)
        self.get_logger().info('camera_calibration ready: %s board, %.3f m squares, %d-%d views' % (
            param('pattern'), float(param('square')), self.calibrator.min_views, self.calibrator.max_views))

    def _on_info(self, msg):
        self.published_info = msg

    def _on_image(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.calibrated is not None or stamp - self.last_try < self.period:
            return
        self.last_try = stamp
        try:
            outcome = self.calibrator.add(decode_gray(msg))
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=5.0)
            return
        if outcome == 'kept':
            self.get_logger().info('view %d kept; coverage x %.2f y %.2f size %.2f skew %.2f' % (
                (len(self.calibrator.views),) + self.calibrator.coverage()))
        elif outcome == 'no board':
            self.get_logger().info('no %dx%d chessboard in view' % self.calibrator.pattern,
                                   throttle_duration_sec=30.0)
        if self.calibrator.ready():
            self._finish(msg)

    def _finish(self, image):
        from sensor_msgs.msg import CameraInfo
        result = self.calibrator.calibrate()
        k = result['camera_matrix']
        info = CameraInfo()
        info.header.frame_id = image.header.frame_id
        info.header.stamp = image.header.stamp
        info.width, info.height = result['image_size']
        info.distortion_model = 'plumb_bob'
        info.d = [float(v) for v in result['distortion']]
        info.k = [float(v) for v in k.ravel()]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [k[0, 0], k[0, 1], k[0, 2], 0.0, k[1, 0], k[1, 1], k[1, 2], 0.0, 0.0, 0.0, 1.0, 0.0]
        self._pub.publish(info)
        self.calibrated = info
        line = 'calibrated from %d views: fx %.2f fy %.2f cx %.2f cy %.2f, RMS %.3f px' % (
            result['views'], k[0, 0], k[1, 1], k[0, 2], k[1, 2], result['rms_px'])
        if self.published_info is not None:
            pk = self.published_info.k
            line += '; camera info says fx %.2f fy %.2f cx %.2f cy %.2f' % (pk[0], pk[4], pk[2], pk[5])
        self.get_logger().info(line)
        if self.output_path:
            with open(self.output_path, 'w') as handle:
                handle.write(camera_info_yaml(self.get_name(), result))
            self.get_logger().info('wrote %s' % self.output_path)


def camera_calibration_main(args=None):
    from ._runtime import run
    return run(CameraCalibrationNode, 'camera_calibration', args=args)
