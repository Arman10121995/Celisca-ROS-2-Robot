"""Localization nodes (R7.3).

The console scripts run scan-to-map localizers from
robot_lab_algorithms.scan_matching (rewritten 2026-10-13):

* icp_localization - ICP, point to point (SVD alignment);
* plicp_localization - ICP, point to line (Gauss-Newton on map normals);
* ndt_localization - 2D normal distributions transform;
* correlative_localization - coarse-to-fine correlative scan matching;
* dead_reckoning - the odometry alone.

Each node subscribes to /map, /scan and the odom -> base TF; it predicts
with the odometry increment between scans, aligns the scan, and publishes
geometry_msgs/PoseWithCovarianceStamped on /robot_lab/<method>_pose beside
AMCL (it does not publish TF).  The initial pose comes from /initialpose,
else the first /amcl_pose, else the initial_* parameters.

The classes defined below (DeadReckoning, AMCLLocalization, ICPLocalization,
NDTLocalization, RGBDSLAMLocalization) are the earlier versions, kept for
their tests only: the dead_reckoning node had no subscriptions, ICP
estimated rotation about the map origin with brute-force matching, and the
"AMCL" and "RGB-D SLAM" nodes were simplified stand-ins.
"""

import math
import sys

import random
from ._runtime import run as _run


try:
    import rclpy
    from rclpy.node import Node as _RclpyNode
    Node = _RclpyNode
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


class DeadReckoning:
    """Integrate body twist into an odometry pose (dead reckoning).

    Pure 2D math with no ROS dependency so numerical tests run without a
    ROS context (R1.2: numerical tests must not require rclpy.init()).
    """

    def __init__(self, initial_x=0.0, initial_y=0.0, initial_theta=0.0):
        self.x = float(initial_x)
        self.y = float(initial_y)
        self.theta = float(initial_theta)

    def integrate(self, vx, wz, dt):
        """Advance the pose by (linear x, angular z) over dt seconds (2D)."""
        if dt <= 0.0:
            return (self.x, self.y, self.theta)
        if abs(wz) < 1e-6:
            self.x += vx * dt * math.cos(self.theta)
            self.y += vx * dt * math.sin(self.theta)
        else:
            radius = vx / wz
            self.x += radius * (math.sin(self.theta + wz * dt) - math.sin(self.theta))
            self.y += -radius * (math.cos(self.theta + wz * dt) - math.cos(self.theta))
        self.theta += wz * dt
        return (self.x, self.y, self.theta)


class AMCLLocalization:
    """Adaptive Monte Carlo Localization (AMCL) - simplified particle filter.
    
    This is a basic particle filter implementation for demonstration purposes.
    Real AMCL would be more sophisticated with KLD sampling, etc.
    """

    def __init__(self, num_particles=100, initial_pose=(0.0, 0.0, 0.0), 
                 initial_covariance=(0.1, 0.1, 0.05), motion_noise=(0.2, 0.2, 0.1),
                 measurement_noise=(0.3, 0.3)):
        self.num_particles = num_particles
        self.motion_noise = motion_noise
        self.measurement_noise = measurement_noise
        
        # Initialize particles
        self.particles = []
        self.weights = []
        for _ in range(num_particles):
            self.particles.append(list(initial_pose))
            self.weights.append(1.0 / num_particles)

    def motion_update(self, vx, wz, dt):
        """Update particle positions based on motion model."""
        for i in range(len(self.particles)):
            particle = self.particles[i]
            x, y, theta = particle
            
            # Add Gaussian noise to motion
            vx_noisy = vx + self._gaussian_noise(0, self.motion_noise[0])
            wz_noisy = wz + self._gaussian_noise(0, self.motion_noise[2])
            
            # Update position (same kinematics as DeadReckoning)
            if abs(wz_noisy) < 1e-6:
                x += vx_noisy * dt * math.cos(theta)
                y += vx_noisy * dt * math.sin(theta)
            else:
                radius = vx_noisy / wz_noisy
                x += radius * (math.sin(theta + wz_noisy * dt) - math.sin(theta))
                y += -radius * (math.cos(theta + wz_noisy * dt) - math.cos(theta))
            theta += wz_noisy * dt
            
            # Normalize angle
            theta = self._normalize_angle(theta)
            self.particles[i] = [x, y, theta]

    def measurement_update(self, landmarks):
        """Update particle weights based on sensor measurements.
        
        landmarks: list of (x, y, expected_distance) tuples
        """
        total_weight = 0.0
        
        for i in range(len(self.particles)):
            particle = self.particles[i]
            px, py, ptheta = particle
            
            # Calculate measurement likelihood
            weight = 1.0
            for lx, ly, expected_dist in landmarks:
                actual_dist = math.sqrt((px - lx)**2 + (py - ly)**2)
                # Gaussian likelihood
                variance = self.measurement_noise[0]**2
                likelihood = math.exp(-0.5 * (actual_dist - expected_dist)**2 / variance)
                weight *= likelihood
            
            self.weights[i] = weight
            total_weight += weight
        
        # Normalize weights
        if total_weight > 0:
            self.weights = [w / total_weight for w in self.weights]
        else:
            # Reset to uniform if all weights are zero
            self.weights = [1.0 / len(self.weights)] * len(self.weights)

    def resample(self):
        """Resample particles based on weights (systematic resampling)."""
        # Simple systematic resampling
        new_particles = []
        cumulative_weights = self._cumulative_weights()
        
        step = 1.0 / len(self.particles)
        u = random.uniform(0, step)
        
        i = 0
        for j in range(len(self.particles)):
            while u > cumulative_weights[i] and i < len(cumulative_weights) - 1:
                i += 1
            new_particles.append(list(self.particles[i]))
            u += step
        
        self.particles = new_particles
        self.weights = [1.0 / len(self.particles)] * len(self.particles)

    def estimate_pose(self):
        """Calculate weighted mean pose estimate."""
        if not self.particles:
            return (0.0, 0.0, 0.0)
        
        x_sum, y_sum, theta_sum = 0.0, 0.0, 0.0
        weight_sum = 0.0
        
        for i in range(len(self.particles)):
            particle = self.particles[i]
            weight = self.weights[i]
            x_sum += particle[0] * weight
            y_sum += particle[1] * weight
            # Handle angle wrapping
            theta_sum += self._normalize_angle(particle[2]) * weight
            weight_sum += weight
        
        if weight_sum > 0:
            x = x_sum / weight_sum
            y = y_sum / weight_sum
            theta = theta_sum / weight_sum
            return (x, y, self._normalize_angle(theta))
        else:
            return (0.0, 0.0, 0.0)

    def _gaussian_noise(self, mean, std_dev):
        """Generate Gaussian noise (simplified - use uniform for now)."""
        import random
        return random.gauss(mean, std_dev)

    def _normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _cumulative_weights(self):
        """Calculate cumulative weights."""
        cumulative = []
        running_sum = 0.0
        for w in self.weights:
            running_sum += w
            cumulative.append(running_sum)
        return cumulative


class ICPLocalization:
    """Iterative Closest Point (ICP) scan matching for localization.
    
    Simplified ICP implementation for laser scan matching.
    """

    def __init__(self, initial_pose=(0.0, 0.0, 0.0), max_iterations=10, 
                 distance_threshold=1.0, convergence_threshold=0.01):
        self.pose = list(initial_pose)  # x, y, theta
        self.max_iterations = max_iterations
        self.distance_threshold = distance_threshold
        self.convergence_threshold = convergence_threshold

    def match_scan(self, current_scan, reference_scan):
        """Match current scan to reference scan using ICP.
        
        current_scan: list of (range, angle) tuples in polar coordinates
        reference_scan: list of (x, y) tuples in Cartesian coordinates (map frame)
        
        Returns estimated pose (x, y, theta) after matching.
        """
        if not current_scan or not reference_scan:
            return self.pose
        
        # Convert current scan to Cartesian coordinates relative to robot
        current_points = []
        for r, angle in current_scan:
            if r > 0 and r < self.distance_threshold:
                x_rel = r * math.cos(angle)
                y_rel = r * math.sin(angle)
                current_points.append((x_rel, y_rel))
        
        if not current_points:
            return self.pose
        
        # Iterative ICP
        for _ in range(self.max_iterations):
            # Transform current points to map frame
            transformed_points = self._transform_points(current_points)
            
            # Find closest points
            matches = []
            for tp in transformed_points:
                closest_dist = float('inf')
                closest_rp = None
                for rp in reference_scan:
                    dist = math.sqrt((tp[0] - rp[0])**2 + (tp[1] - rp[1])**2)
                    if dist < closest_dist:
                        closest_dist = dist
                        closest_rp = rp
                
                if closest_rp and closest_dist < self.distance_threshold:
                    matches.append((tp, closest_rp))
            
            if not matches:
                break
            
            # Calculate transformation that minimizes error
            new_pose = self._calculate_transformation(matches)
            
            # Check for convergence
            dx = new_pose[0] - self.pose[0]
            dy = new_pose[1] - self.pose[1]
            dtheta = self._normalize_angle(new_pose[2] - self.pose[2])
            
            if math.sqrt(dx**2 + dy**2) < self.convergence_threshold and abs(dtheta) < self.convergence_threshold:
                self.pose = new_pose
                break
            
            self.pose = new_pose
        
        return self.pose

    def _transform_points(self, points):
        """Transform points from robot frame to map frame."""
        x, y, theta = self.pose
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        
        transformed = []
        for px, py in points:
            # Apply rotation and translation
            tx = x + px * cos_theta - py * sin_theta
            ty = y + px * sin_theta + py * cos_theta
            transformed.append((tx, ty))
        return transformed

    def _calculate_transformation(self, matches):
        """Calculate transformation that minimizes error between matched points."""
        # Simple least squares approach
        tx_sum, ty_sum, theta_sum = 0.0, 0.0, 0.0
        count = 0
        
        for (tp_x, tp_y), (rp_x, rp_y) in matches:
            # Translation error
            tx_sum += rp_x - tp_x
            ty_sum += rp_y - tp_y
            
            # Rotation error (simplified)
            # This is a basic implementation - real ICP would use more sophisticated methods
            cross = tp_x * rp_y - tp_y * rp_x
            dot = tp_x * rp_x + tp_y * rp_y
            if dot > 0:
                theta_sum += math.atan2(cross, dot)
            
            count += 1
        
        if count > 0:
            new_x = self.pose[0] + tx_sum / count
            new_y = self.pose[1] + ty_sum / count
            new_theta = self.pose[2] + theta_sum / count
            return [new_x, new_y, self._normalize_angle(new_theta)]
        else:
            return self.pose

    def _normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


class NDTLocalization:
    """Normal Distributions Transform (NDT) registration for localization.
    
    Simplified NDT implementation for demonstration purposes.
    """

    def __init__(self, initial_pose=(0.0, 0.0, 0.0), resolution=0.5, 
                 max_iterations=10, convergence_threshold=0.01):
        self.pose = list(initial_pose)
        self.resolution = resolution
        self.max_iterations = max_iterations
        self.convergence_threshold = convergence_threshold

    def match_scan(self, current_scan, reference_grid):
        """Match current scan to reference grid using NDT.
        
        current_scan: list of (range, angle) tuples
        reference_grid: dict mapping grid cells to normal distributions (mean, covariance)
        
        Returns estimated pose after matching.
        """
        if not current_scan:
            return self.pose
        
        # Convert current scan to Cartesian points
        current_points = []
        for r, angle in current_scan:
            if r > 0:
                x_rel = r * math.cos(angle)
                y_rel = r * math.sin(angle)
                current_points.append((x_rel, y_rel))
        
        if not current_points:
            return self.pose
        
        # NDT registration
        for _ in range(self.max_iterations):
            # Transform current points to map frame
            transformed_points = self._transform_points(current_points)
            
            # Calculate score and gradients
            score, gradient_x, gradient_y, gradient_theta = self._calculate_ndt_score(
                transformed_points, reference_grid)
            
            # Update pose using gradient descent
            learning_rate = 0.1
            new_x = self.pose[0] + learning_rate * gradient_x
            new_y = self.pose[1] + learning_rate * gradient_y
            new_theta = self.pose[2] + learning_rate * gradient_theta
            
            # Check convergence
            dx = new_x - self.pose[0]
            dy = new_y - self.pose[1]
            dtheta = self._normalize_angle(new_theta - self.pose[2])
            
            if math.sqrt(dx**2 + dy**2) < self.convergence_threshold and abs(dtheta) < self.convergence_threshold:
                self.pose = [new_x, new_y, new_theta]
                break
            
            self.pose = [new_x, new_y, new_theta]
        
        return self.pose

    def _transform_points(self, points):
        """Transform points from robot frame to map frame."""
        x, y, theta = self.pose
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        
        transformed = []
        for px, py in points:
            tx = x + px * cos_theta - py * sin_theta
            ty = y + px * sin_theta + py * cos_theta
            transformed.append((tx, ty))
        return transformed

    def _calculate_ndt_score(self, points, reference_grid):
        """Calculate NDT score and gradients."""
        score = 0.0
        gradient_x = 0.0
        gradient_y = 0.0
        gradient_theta = 0.0
        
        for point in points:
            # Find grid cell for this point
            cell_x = int(point[0] // self.resolution)
            cell_y = int(point[1] // self.resolution)
            cell_key = (cell_x, cell_y)
            
            if cell_key in reference_grid:
                mean, cov = reference_grid[cell_key]
                # Simple Gaussian probability
                dx = point[0] - mean[0]
                dy = point[1] - mean[1]
                
                # Inverse covariance (simplified)
                if cov[0] > 0 and cov[1] > 0:
                    det = cov[0] * cov[1] - cov[2] * cov[2]
                    if det > 0:
                        inv_cov = ([cov[1] / det, -cov[2] / det], 
                                  [-cov[2] / det, cov[0] / det])
                        
                        # Calculate gradient contribution
                        prob = math.exp(-0.5 * (dx**2 * inv_cov[0][0] + 2 * dx * dy * inv_cov[0][1] + dy**2 * inv_cov[1][1]))
                        score += prob
                        
                        # Approximate gradients
                        gradient_x += dx * prob
                        gradient_y += dy * prob
                        # Theta gradient would require derivative w.r.t. rotation
        
        return score, gradient_x, gradient_y, gradient_theta

    def _normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


class RGBDSLAMLocalization:
    """RGB-D SLAM-based localization using visual features.
    
    Simplified implementation that simulates RGB-D feature matching.
    """

    def __init__(self, initial_pose=(0.0, 0.0, 0.0), camera_fov=(60.0, 45.0), 
                 max_features=100, matching_threshold=0.7):
        self.pose = list(initial_pose)  # x, y, theta
        self.camera_fov = camera_fov  # horizontal, vertical in degrees
        self.max_features = max_features
        self.matching_threshold = matching_threshold
        self.feature_map = {}  # Map features by location

    def process_frame(self, depth_image, rgb_image):
        """Process RGB-D frame and update pose.
        
        depth_image: 2D array of depth values
        rgb_image: 2D array of RGB values
        
        Returns updated pose estimate.
        """
        # Simplified implementation
        # In a real system, this would extract features and match them
        
        # For now, we'll just return the current pose
        # This is a placeholder that would be replaced with actual RGB-D SLAM
        return self.pose

    def add_to_map(self, features, pose):
        """Add features to the map at the given pose."""
        # Store features in the map
        if pose not in self.feature_map:
            self.feature_map[pose] = []
        self.feature_map[pose].extend(features)

    def estimate_pose_from_features(self, current_features):
        """Estimate pose by matching current features to map."""
        # This would implement feature matching and pose estimation
        # For now, return current pose
        return self.pose


class ScanLocalizerNode(Node):
    """Track the robot pose in /map with one scan_matching localizer."""

    def __init__(self, method, node_name):
        super().__init__(node_name)
        import numpy as np
        import tf2_ros
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan
        from robot_lab_algorithms import scan_matching
        self._np, self._sm = np, scan_matching
        self.method = method
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('output_topic', '/robot_lab/%s_pose' % method)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_theta', 0.0)
        self.declare_parameter('beam_step', 2)
        g = lambda name: self.get_parameter(name).value  # noqa: E731
        self._pose_type = PoseWithCovarianceStamped
        self.localizer = None
        self.map = None
        self.initial = None
        self.last_odom = None
        self.sensor = None
        self.tf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf, self)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, g('output_topic'), 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, g('map_topic'), self.on_map, latched)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_initial, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, 10)
        self.create_subscription(LaserScan, g('scan_topic'), self.on_scan, qos_profile_sensor_data)
        self.get_logger().info('%s localizer -> %s' % (method, g('output_topic')))

    @staticmethod
    def _pose_of(msg_pose):
        q = msg_pose.orientation
        return (msg_pose.position.x, msg_pose.position.y,
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def on_map(self, msg):
        self.map = self._sm.OccupancyMap.from_msg(msg)
        self.localizer = None  # rebuilt on the next scan with the new map

    def on_initial(self, msg):
        self.initial = self._pose_of(msg.pose.pose)
        self.localizer = None

    def on_amcl(self, msg):
        if self.initial is None:
            self.initial = self._pose_of(msg.pose.pose)

    def _odom_pose(self, stamp):
        from rclpy.time import Time
        t = self.tf.lookup_transform(self.get_parameter('odom_frame').value,
                                     self.get_parameter('base_frame').value, Time())
        q = t.transform.rotation
        return self._np.array([t.transform.translation.x, t.transform.translation.y,
                               math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))])

    def _sensor_offset(self, frame):
        if self.sensor is None:
            from rclpy.time import Time
            t = self.tf.lookup_transform(self.get_parameter('base_frame').value, frame, Time())
            q = t.transform.rotation
            self.sensor = (t.transform.translation.x, t.transform.translation.y,
                           math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        return self.sensor

    def on_scan(self, msg):
        if self.map is None:
            return
        try:
            odom = self._odom_pose(msg.header.stamp)
            offset = self._sensor_offset(msg.header.frame_id)
        except Exception:  # noqa: BLE001 - TF not ready yet
            return
        if self.localizer is None:
            start = self.initial or tuple(float(self.get_parameter(n).value)
                                          for n in ('initial_x', 'initial_y', 'initial_theta'))
            self.localizer = self._sm.ScanLocalizer(self.method, self.map, start)
            self.last_odom = odom
        delta = self._sm.relative(self.last_odom, odom)
        self.last_odom = odom
        points = self._sm.scan_points(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min,
                                      msg.range_max, int(self.get_parameter('beam_step').value), offset)
        x, y, th = self.localizer.step(delta, points)
        out = self._pose_type()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'map'
        out.pose.pose.position.x, out.pose.pose.position.y = float(x), float(y)
        out.pose.pose.orientation.z, out.pose.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
        self.pub.publish(out)


def _localizer_main(method, node_name, args=None):
    return _run(lambda: ScanLocalizerNode(method, node_name), node_name, args=args)


def dead_reckoning_main(args=None):
    return _localizer_main('dead_reckoning', 'dead_reckoning', args)


def icp_localization_main(args=None):
    return _localizer_main('icp_point_to_point', 'icp_localization', args)


def plicp_localization_main(args=None):
    return _localizer_main('icp_point_to_line', 'plicp_localization', args)


def ndt_localization_main(args=None):
    return _localizer_main('ndt', 'ndt_localization', args)


def correlative_localization_main(args=None):
    return _localizer_main('correlative', 'correlative_localization', args)


def amcl_main(args=None):
    print('amcl (robot_lab_algorithms): retired - the lab runs Nav2 AMCL (localization:=amcl)')
    return 1


def rgbd_slam_localization_main(args=None):
    print('rgbd_slam_localization: retired - use RTAB-Map (localization:=rtabmap_localization)')
    return 1


if __name__ == '__main__':
    sys.exit(dead_reckoning_main())
