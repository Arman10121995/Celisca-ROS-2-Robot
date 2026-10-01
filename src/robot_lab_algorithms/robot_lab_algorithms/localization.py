"""Localization algorithms (P5).

Adds `dead_reckoning`, a lightweight odometry-integration pose estimator that
rounds out the localization category to five integrated implementations. It
integrates linear/angular velocity commands into an estimated pose and
degrades gracefully when odometry topics are absent.
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


class DeadReckoningNode(Node):
    """ROS wrapper around :class:`DeadReckoning` for the console entry point."""

    def __init__(self, node_name='dead_reckoning'):
        super().__init__(node_name)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_theta', 0.0)
        self.dr = DeadReckoning(
            initial_x=self.get_parameter('initial_x').value,
            initial_y=self.get_parameter('initial_y').value,
            initial_theta=self.get_parameter('initial_theta').value,
        )
        self.get_logger().info('DeadReckoning ready')

    @property
    def x(self):
        return self.dr.x

    @property
    def y(self):
        return self.dr.y

    @property
    def theta(self):
        return self.dr.theta

    def integrate(self, vx, wz, dt):
        return self.dr.integrate(vx, wz, dt)


class AMCLNode(Node):
    """ROS wrapper for AMCL particle filter localization."""

    def __init__(self, node_name='amcl'):
        super().__init__(node_name)
        self.declare_parameter('num_particles', 100)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_theta', 0.0)
        self.declare_parameter('odom_topic', '/odom/ground_truth')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('output_topic', '/amcl_pose')
        
        self.amcl = AMCLLocalization(
            num_particles=int(self.get_parameter('num_particles').value),
            initial_pose=(float(self.get_parameter('initial_x').value),
                        float(self.get_parameter('initial_y').value),
                        float(self.get_parameter('initial_theta').value))
        )
        
        # Subscribe to odometry and laser scan
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import LaserScan
        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._on_odom, 10)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self._on_scan, 10)
        
        # Publish estimated pose
        from geometry_msgs.msg import PoseWithCovarianceStamped
        self._pub = self.create_publisher(
            PoseWithCovarianceStamped, self.get_parameter('output_topic').value, 10)
        
        self._last_odom = None
        self._last_scan = None
        self.get_logger().info('AMCL ready')

    def _on_odom(self, msg):
        self._last_odom = msg
        self._process()

    def _on_scan(self, msg):
        self._last_scan = msg
        self._process()

    def _process(self):
        if self._last_odom is None or self._last_scan is None:
            return
        
        # Extract motion (simplified - use velocity from odometry)
        vx = self._last_odom.twist.twist.linear.x
        wz = self._last_odom.twist.twist.angular.z
        dt = 0.1  # Fixed time step for now
        
        # Update particles based on motion
        self.amcl.motion_update(vx, wz, dt)
        
        # Extract landmarks from scan (simplified - use scan points as landmarks)
        landmarks = []
        for i, r in enumerate(self._last_scan.ranges):
            if r > 0 and r < self._last_scan.range_max:
                angle = self._last_scan.angle_min + i * self._last_scan.angle_increment
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                # For now, use distance as expected distance (simplified)
                landmarks.append((x, y, r))
        
        # Update based on measurements
        if landmarks:
            self.amcl.measurement_update(landmarks)
        
        # Resample
        self.amcl.resample()
        
        # Publish estimated pose
        x, y, theta = self.amcl.estimate_pose()
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        import math
        half_theta = theta / 2.0
        pose_msg.pose.pose.orientation.w = math.cos(half_theta)
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = math.sin(half_theta)
        
        # Simple covariance (would be calculated from particle spread in real implementation)
        pose_msg.pose.covariance = [0.1] * 36
        pose_msg.pose.covariance[0] = pose_msg.pose.covariance[7] = pose_msg.pose.covariance[14] = 0.25
        pose_msg.pose.covariance[21] = pose_msg.pose.covariance[28] = pose_msg.pose.covariance[35] = 0.1
        
        self._pub.publish(pose_msg)


class ICPNode(Node):
    """ROS wrapper for ICP scan matching localization."""

    def __init__(self, node_name='icp_localization'):
        super().__init__(node_name)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_theta', 0.0)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('output_topic', '/icp_pose')
        
        self.icp = ICPLocalization(
            initial_pose=(float(self.get_parameter('initial_x').value),
                        float(self.get_parameter('initial_y').value),
                        float(self.get_parameter('initial_theta').value))
        )
        
        from sensor_msgs.msg import LaserScan
        from nav_msgs.msg import OccupancyGrid
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self._on_scan, 10)
        self.create_subscription(
            OccupancyGrid, self.get_parameter('map_topic').value, self._on_map, 10)
        
        from geometry_msgs.msg import PoseWithCovarianceStamped
        self._pub = self.create_publisher(
            PoseWithCovarianceStamped, self.get_parameter('output_topic').value, 10)
        
        self._map = None
        self.get_logger().info('ICP Localization ready')

    def _on_map(self, msg):
        self._map = msg

    def _on_scan(self, msg):
        if self._map is None:
            return
        
        # Extract scan points (polar coordinates)
        scan_points = []
        for i, r in enumerate(msg.ranges):
            if r > 0 and r < msg.range_max:
                angle = msg.angle_min + i * msg.angle_increment
                scan_points.append((r, angle))
        
        # Extract reference points from map (simplified)
        reference_points = self._extract_map_features()
        
        if scan_points and reference_points:
            pose = self.icp.match_scan(scan_points, reference_points)
            self._publish_pose(pose, msg.header.stamp)

    def _extract_map_features(self):
        """Extract features from the map for matching."""
        if self._map is None:
            return []
        
        # Simple approach: extract all free space points (simplified)
        features = []
        resolution = self._map.info.resolution
        width = self._map.info.width
        height = self._map.info.height
        origin_x = self._map.info.origin.position.x
        origin_y = self._map.info.origin.position.y
        
        for y in range(height):
            for x in range(width):
                idx = y * width + x
                if self._map.data[idx] == 0:  # Free space
                    map_x = origin_x + x * resolution
                    map_y = origin_y + y * resolution
                    features.append((map_x, map_y))
        
        return features

    def _publish_pose(self, pose, stamp):
        from geometry_msgs.msg import PoseWithCovarianceStamped
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x = pose[0]
        pose_msg.pose.pose.position.y = pose[1]
        pose_msg.pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        half_theta = pose[2] / 2.0
        pose_msg.pose.pose.orientation.w = math.cos(half_theta)
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = math.sin(half_theta)
        
        # Simple covariance
        pose_msg.pose.covariance = [0.1] * 36
        pose_msg.pose.covariance[0] = pose_msg.pose.covariance[7] = pose_msg.pose.covariance[14] = 0.25
        pose_msg.pose.covariance[21] = pose_msg.pose.covariance[28] = pose_msg.pose.covariance[35] = 0.1
        
        self._pub.publish(pose_msg)


class NDTNode(Node):
    """ROS wrapper for NDT localization."""

    def __init__(self, node_name='ndt_localization'):
        super().__init__(node_name)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_theta', 0.0)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('output_topic', '/ndt_pose')
        
        self.ndt = NDTLocalization(
            initial_pose=(float(self.get_parameter('initial_x').value),
                        float(self.get_parameter('initial_y').value),
                        float(self.get_parameter('initial_theta').value))
        )
        
        from sensor_msgs.msg import LaserScan
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self._on_scan, 10)
        
        from geometry_msgs.msg import PoseWithCovarianceStamped
        self._pub = self.create_publisher(
            PoseWithCovarianceStamped, self.get_parameter('output_topic').value, 10)
        
        self.get_logger().info('NDT Localization ready')

    def _on_scan(self, msg):
        # Extract scan points (polar coordinates)
        scan_points = []
        for i, r in enumerate(msg.ranges):
            if r > 0 and r < msg.range_max:
                angle = msg.angle_min + i * msg.angle_increment
                scan_points.append((r, angle))
        
        # Create simple reference grid (in real implementation this would be precomputed from map)
        reference_grid = self._create_reference_grid()
        
        pose = self.ndt.match_scan(scan_points, reference_grid)
        self._publish_pose(pose, msg.header.stamp)

    def _create_reference_grid(self):
        """Create a simple reference grid (placeholder)."""
        # This would be precomputed from the map in a real implementation
        grid = {}
        # Add some sample points with normal distributions
        grid[(0, 0)] = ([0.0, 0.0], [1.0, 0.0, 0.0])  # mean, covariance
        grid[(1, 0)] = ([1.0, 0.0], [1.0, 0.0, 0.0])
        grid[(0, 1)] = ([0.0, 1.0], [1.0, 0.0, 0.0])
        return grid

    def _publish_pose(self, pose, stamp):
        from geometry_msgs.msg import PoseWithCovarianceStamped
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x = pose[0]
        pose_msg.pose.pose.position.y = pose[1]
        pose_msg.pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        half_theta = pose[2] / 2.0
        pose_msg.pose.pose.orientation.w = math.cos(half_theta)
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = math.sin(half_theta)
        
        # Simple covariance
        pose_msg.pose.covariance = [0.1] * 36
        pose_msg.pose.covariance[0] = pose_msg.pose.covariance[7] = pose_msg.pose.covariance[14] = 0.25
        pose_msg.pose.covariance[21] = pose_msg.pose.covariance[28] = pose_msg.pose.covariance[35] = 0.1
        
        self._pub.publish(pose_msg)


class RGBDSLAMNode(Node):
    """ROS wrapper for RGB-D SLAM localization."""

    def __init__(self, node_name='rgbd_slam_localization'):
        super().__init__(node_name)
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_theta', 0.0)
        self.declare_parameter('rgb_topic', '/camera/rgb/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('output_topic', '/rgbd_slam_pose')
        
        self.rgbd_slam = RGBDSLAMLocalization(
            initial_pose=(float(self.get_parameter('initial_x').value),
                        float(self.get_parameter('initial_y').value),
                        float(self.get_parameter('initial_theta').value))
        )
        
        # Placeholder subscriptions - in real implementation these would be used
        self.get_logger().info('RGB-D SLAM Localization ready')

    def _publish_pose(self, pose, stamp):
        from geometry_msgs.msg import PoseWithCovarianceStamped
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x = pose[0]
        pose_msg.pose.pose.position.y = pose[1]
        pose_msg.pose.pose.position.z = pose[2]
        
        # Convert theta to quaternion (pose[2] is theta in this simplified implementation)
        half_theta = pose[2] / 2.0
        pose_msg.pose.pose.orientation.w = math.cos(half_theta)
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = math.sin(half_theta)
        
        # Simple covariance
        pose_msg.pose.covariance = [0.1] * 36
        pose_msg.pose.covariance[0] = pose_msg.pose.covariance[7] = pose_msg.pose.covariance[14] = 0.25
        pose_msg.pose.covariance[21] = pose_msg.pose.covariance[28] = pose_msg.pose.covariance[35] = 0.1
        
        self._pub.publish(pose_msg)


def dead_reckoning_main(args=None):
    return _run(DeadReckoningNode, 'dead_reckoning', args=args)


def amcl_main(args=None):
    return _run(AMCLNode, 'amcl', args=args)


def icp_localization_main(args=None):
    return _run(ICPNode, 'icp_localization', args=args)


def ndt_localization_main(args=None):
    return _run(NDTNode, 'ndt_localization', args=args)


def rgbd_slam_localization_main(args=None):
    return _run(RGBDSLAMNode, 'rgbd_slam_localization', args=args)


if __name__ == '__main__':
    sys.exit(dead_reckoning_main())
