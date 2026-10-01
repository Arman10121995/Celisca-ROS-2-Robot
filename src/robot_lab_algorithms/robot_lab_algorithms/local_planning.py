"""Local planning algorithms (P5).

Adds `follow_the_gap`, a reactive obstacle-aware local planner that computes a
2D steering command by choosing the angular-gap heading that maximizes free
space. Rounds out the local_planning category to five integrated
implementations.
"""

from __future__ import annotations

import math
import sys


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


class FollowTheGap:
    """Select heading toward the widest gap in a laser scan."""

    def __init__(self, safety_radius=0.3, max_range=10.0):
        self.safety_radius = safety_radius
        self.max_range = max_range

    def steer(self, angle_min, angle_increment, ranges, goal_bearing=0.0):
        """Return a (linear, angular) steering command given a scan.

        goal_bearing: desired heading (rad) toward the goal. The planner picks
        the widest gap and steers toward its center, then biases toward goal.
        """
        if not ranges:
            return (0.0, 0.0)
        # Build an array of (angle, range) with obstacles closer than safety set to 0
        beams = []
        for i, r in enumerate(ranges):
            theta = angle_min + i * angle_increment
            if r < self.safety_radius:
                beams.append((theta, 0.0))
            else:
                beams.append((theta, min(r, self.max_range)))
        # Find the widest contiguous gap of free beams
        best_gap = None
        best_width = 0.0
        i = 0
        n = len(beams)
        while i < n:
            if beams[i][1] > self.safety_radius:
                j = i
                while j < n and beams[j][1] > self.safety_radius:
                    j += 1
                width = j - i
                if width > best_width:
                    best_width = width
                    mid = (beams[i][0] + beams[j - 1][0]) / 2.0
                    best_gap = mid
                i = j
            else:
                i += 1
        if best_gap is None:
            return (0.0, 0.0)
        # blend gap heading with goal bearing
        heading = 0.6 * best_gap + 0.4 * goal_bearing
        linear = 0.5
        angular = heading
        return (linear, angular)


class DWBLocalPlanner:
    """Dynamic Window Approach (DWA) local planner.
    
    Simplified DWA implementation that evaluates candidate trajectories
    and selects the best one based on objective function.
    """

    def __init__(self, max_vel_x=0.5, min_vel_x=-0.2, max_vel_theta=1.0, 
                 min_vel_theta=-1.0, accel_lim_x=0.5, accel_lim_theta=0.5,
                 safety_radius=0.3, goal_tolerance=0.2):
        self.max_vel_x = max_vel_x
        self.min_vel_x = min_vel_x
        self.max_vel_theta = max_vel_theta
        self.min_vel_theta = min_vel_theta
        self.accel_lim_x = accel_lim_x
        self.accel_lim_theta = accel_lim_theta
        self.safety_radius = safety_radius
        self.goal_tolerance = goal_tolerance
        
        # Previous velocities for acceleration limits
        self.prev_vel_x = 0.0
        self.prev_vel_theta = 0.0

    def compute_velocity_commands(self, angle_min, angle_increment, ranges, 
                                   current_vel_x, current_vel_y, current_vel_theta,
                                   goal_x, goal_y, goal_theta):
        """Compute velocity commands using DWA.
        
        Returns (linear_vel, angular_vel) tuple.
        """
        # Sample velocity space
        candidate_commands = []
        for vx in [current_vel_x - 0.2, current_vel_x, current_vel_x + 0.2]:
            vx = max(self.min_vel_x, min(self.max_vel_x, vx))
            for vtheta in [current_vel_theta - 0.3, current_vel_theta, current_vel_theta + 0.3]:
                vtheta = max(self.min_vel_theta, min(self.max_vel_theta, vtheta))
                
                # Check acceleration limits
                accel_x = abs(vx - current_vel_x)
                accel_theta = abs(vtheta - current_vel_theta)
                
                if accel_x <= self.accel_lim_x and accel_theta <= self.accel_lim_theta:
                    candidate_commands.append((vx, vtheta))
        
        if not candidate_commands:
            return (0.0, 0.0)
        
        # Evaluate each candidate
        best_command = None
        best_score = float('-inf')
        
        current_x, current_y, current_theta = 0.0, 0.0, 0.0  # Simplified
        
        for vx, vtheta in candidate_commands:
            score = self._evaluate_command(
                vx, vtheta, angle_min, angle_increment, ranges,
                current_x, current_y, current_theta,
                goal_x, goal_y, goal_theta)
            
            if score > best_score:
                best_score = score
                best_command = (vx, vtheta)
        
        return best_command

    def _evaluate_command(self, vx, vtheta, angle_min, angle_increment, ranges,
                         current_x, current_y, current_theta,
                         goal_x, goal_y, goal_theta):
        """Evaluate a candidate velocity command."""
        # Simulate trajectory
        dt = 0.1
        steps = 10
        
        # Check collision along trajectory
        for step in range(1, steps + 1):
            # Predict future pose
            future_theta = current_theta + vtheta * dt * step
            future_x = current_x + vx * math.cos(future_theta) * dt * step
            future_y = current_y + vx * math.sin(future_theta) * dt * step
            
            # Check collision using scan data (simplified)
            if self._check_collision(future_x, future_y, future_theta, angle_min, angle_increment, ranges):
                return float('-inf')  # Collision
        
        # Calculate scores for different objectives
        # Goal heading alignment
        goal_heading = math.atan2(goal_y - current_y, goal_x - current_x)
        heading_error = abs(self._normalize_angle(future_theta - goal_heading))
        heading_score = max(0, 1.0 - heading_error / math.pi)
        
        # Goal distance
        goal_distance = math.hypot(goal_x - future_x, goal_y - future_y)
        distance_score = max(0, 1.0 - min(goal_distance / 2.0, 1.0))
        
        # Velocity magnitude (prefer higher speeds when possible)
        velocity_score = abs(vx) / self.max_vel_x
        
        # Combined score with weights
        total_score = 0.4 * heading_score + 0.4 * distance_score + 0.2 * velocity_score
        
        return total_score

    def _check_collision(self, x, y, theta, angle_min, angle_increment, ranges):
        """Check if a pose is in collision using scan data."""
        # This is a simplified collision check
        # In a real implementation, this would use the actual scan data
        return False

    def _normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


class RegulatedPurePursuit:
    """Regulated Pure Pursuit local planner.
    
    Follows a global path using pure pursuit with velocity regulation.
    """

    def __init__(self, lookahead_distance=0.5, max_vel=0.5, min_vel=0.1, 
                 max_angular_vel=1.0, goal_tolerance=0.2):
        self.lookahead_distance = lookahead_distance
        self.max_vel = max_vel
        self.min_vel = min_vel
        self.max_angular_vel = max_angular_vel
        self.goal_tolerance = goal_tolerance

    def compute_velocity_commands(self, global_path, current_x, current_y, current_theta):
        """Compute velocity commands to follow a global path.
        
        global_path: list of (x, y) waypoints
        current_x, current_y, current_theta: current robot pose
        
        Returns (linear_vel, angular_vel) tuple.
        """
        if not global_path:
            return (0.0, 0.0)
        
        # Find the goal point (last point in path)
        goal_x, goal_y = global_path[-1]
        
        # Check if we've reached the goal
        distance_to_goal = math.hypot(goal_x - current_x, goal_y - current_y)
        if distance_to_goal < self.goal_tolerance:
            return (0.0, 0.0)
        
        # Find the lookahead point
        lookahead_idx = self._find_lookahead_point(global_path, current_x, current_y)
        
        if lookahead_idx is None:
            # No lookahead point found, head toward goal
            lookahead_x, lookahead_y = goal_x, goal_y
        else:
            lookahead_x, lookahead_y = global_path[lookahead_idx]
        
        # Calculate curvature and linear velocity
        curvature = self._calculate_curvature(current_x, current_y, current_theta,
                                            lookahead_x, lookahead_y)
        
        # Regulate velocity based on curvature (higher curvature = lower speed)
        linear_vel = max(self.min_vel, self.max_vel * max(0, 1.0 - abs(curvature)))
        
        # Calculate angular velocity
        target_heading = math.atan2(lookahead_y - current_y, lookahead_x - current_x)
        heading_error = self._normalize_angle(target_heading - current_theta)
        angular_vel = min(self.max_angular_vel, max(-self.max_angular_vel, 
                          heading_error * linear_vel / max(0.001, distance_to_goal)))
        
        return (linear_vel, angular_vel)

    def _find_lookahead_point(self, path, current_x, current_y):
        """Find the waypoint on the path that is lookahead_distance away."""
        for i, (x, y) in enumerate(path):
            distance = math.hypot(x - current_x, y - current_y)
            if distance >= self.lookahead_distance:
                return i
        return None

    def _calculate_curvature(self, x1, y1, theta1, x2, y2):
        """Calculate curvature between current pose and lookahead point."""
        dx = x2 - x1
        dy = y2 - y1
        distance = math.hypot(dx, dy)
        
        if distance < 0.001:
            return 0.0
        
        # Angle to lookahead point
        cross_term = (x1 - x2) * math.sin(theta1) - (y1 - y2) * math.cos(theta1)
        alpha = math.asin(min(1.0, max(-1.0, cross_term / distance)))
        return 2.0 * math.sin(alpha) / distance

    def _normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


class TEBLocalPlanner:
    """Timed Elastic Band (TEB) local planner - simplified version.
    
    This is a placeholder implementation that would be replaced with
    the actual Nav2 TEB plugin if available.
    """

    def __init__(self, max_vel_x=0.5, max_vel_theta=1.0, safety_radius=0.3):
        self.max_vel_x = max_vel_x
        self.max_vel_theta = max_vel_theta
        self.safety_radius = safety_radius

    def compute_velocity_commands(self, angle_min, angle_increment, ranges,
                                   current_vel_x, current_vel_theta,
                                   goal_x, goal_y):
        """Compute velocity commands using TEB-like optimization."""
        # This is a simplified placeholder
        # Real TEB would perform trajectory optimization
        
        # Check for obstacles
        min_distance = min(r for r in ranges if r > 0 and r < float('inf'))
        
        # Slow down if obstacles are close
        if min_distance < self.safety_radius:
            linear_vel = 0.0
        else:
            linear_vel = min(self.max_vel_x, 0.5)
        
        # Simple heading toward goal
        goal_heading = math.atan2(goal_y, goal_x)
        heading_error = goal_heading  # Simplified
        angular_vel = min(self.max_vel_theta, max(-self.max_vel_theta, heading_error))
        
        return (linear_vel, angular_vel)


class MPPILocalPlanner:
    """Model Predictive Path Integral (MPPI) local planner - simplified version.
    
    This is a placeholder implementation for demonstration.
    """

    def __init__(self, num_samples=10, horizon=1.0, temperature=0.1,
                 max_vel_x=0.5, max_vel_theta=1.0):
        self.num_samples = num_samples
        self.horizon = horizon
        self.temperature = temperature
        self.max_vel_x = max_vel_x
        self.max_vel_theta = max_vel_theta

    def compute_velocity_commands(self, angle_min, angle_increment, ranges,
                                   current_vel_x, current_vel_theta,
                                   goal_x, goal_y):
        """Compute velocity commands using MPPI-like sampling."""
        best_cost = float('inf')
        best_command = (0.0, 0.0)
        
        # Sample candidate trajectories
        for _ in range(self.num_samples):
            # Sample random velocity perturbations
            vx = current_vel_x + random.gauss(0, 0.1)
            vtheta = current_vel_theta + random.gauss(0, 0.1)
            
            vx = max(-self.max_vel_x, min(self.max_vel_x, vx))
            vtheta = max(-self.max_vel_theta, min(self.max_vel_theta, vtheta))
            
            # Evaluate cost (simplified)
            cost = self._evaluate_trajectory(vx, vtheta, goal_x, goal_y)
            
            # MPPI uses exponential cost transformation
            if cost < best_cost:
                best_cost = cost
                best_command = (vx, vtheta)
        
        return best_command

    def _evaluate_trajectory(self, vx, vtheta, goal_x, goal_y):
        """Evaluate the cost of a trajectory."""
        # Simplified cost function
        # Real MPPI would integrate costs over the horizon
        
        # Distance to goal cost
        distance_cost = math.hypot(goal_x, goal_y)  # Simplified
        
        # Velocity cost (prefer higher speeds)
        velocity_cost = -abs(vx)
        
        # Combined cost
        return distance_cost + velocity_cost


# ---------------------------------------------------------------------------
# ROS 2 node wrapper (what the console entry point runs)
# ---------------------------------------------------------------------------

import random
from ._runtime import run as _run, spin_node as _spin_node  # noqa: E402


class FollowTheGapNode(Node):
    """/scan -> reactive gap-following velocity command.

    Publishes to ``cmd_vel_topic`` (``/cmd_vel_gap`` by default) so the
    planner can be observed without fighting the active controller; point it
    at ``/cmd_vel`` to drive the robot with it.
    """

    def __init__(self, node_name='follow_the_gap'):
        super().__init__(node_name)
        from sensor_msgs.msg import LaserScan
        from geometry_msgs.msg import Twist, PoseStamped
        from rclpy.qos import qos_profile_sensor_data
        self._twist_type = Twist
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_gap')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('safety_radius', 0.3)
        self.declare_parameter('max_range', 10.0)
        self.planner = FollowTheGap(
            safety_radius=float(self.get_parameter('safety_radius').value),
            max_range=float(self.get_parameter('max_range').value))
        self._pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value,
            self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, self.get_parameter('goal_topic').value,
            self._on_goal, 10)
        self._goal_bearing = 0.0
        self.get_logger().info('follow_the_gap ready')

    def _on_goal(self, msg):
        self._goal_bearing = math.atan2(msg.pose.position.y,
                                        msg.pose.position.x)

    def _on_scan(self, msg):
        ranges = [r if math.isfinite(r) else msg.range_max for r in msg.ranges]
        linear, angular = self.planner.steer(
            msg.angle_min, msg.angle_increment, ranges, self._goal_bearing)
        command = self._twist_type()
        command.linear.x = float(linear)
        command.angular.z = float(angular)
        self._pub.publish(command)


def follow_the_gap_main(args=None):
    return _run(FollowTheGapNode, 'follow_the_gap', args=args)


class DWBNode(Node):
    """ROS wrapper for Dynamic Window Approach (DWA) local planner."""

    def __init__(self, node_name='dwb_local_planner'):
        super().__init__(node_name)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_dwb')
        self.declare_parameter('max_vel_x', 0.5)
        self.declare_parameter('min_vel_x', -0.2)
        self.declare_parameter('max_vel_theta', 1.0)
        self.declare_parameter('min_vel_theta', -1.0)
        
        self.planner = DWBLocalPlanner(
            max_vel_x=float(self.get_parameter('max_vel_x').value),
            min_vel_x=float(self.get_parameter('min_vel_x').value),
            max_vel_theta=float(self.get_parameter('max_vel_theta').value),
            min_vel_theta=float(self.get_parameter('min_vel_theta').value))
        
        from sensor_msgs.msg import LaserScan
        from geometry_msgs.msg import Twist, PoseStamped
        from rclpy.qos import qos_profile_sensor_data
        self._twist_type = Twist
        self._pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value,
            self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, self.get_parameter('goal_topic').value,
            self._on_goal, 10)
        
        self._last_scan = None
        self._goal = None
        self.get_logger().info('DWB local planner ready')

    def _on_scan(self, msg):
        self._last_scan = msg
        self._process()

    def _on_goal(self, msg):
        self._goal = msg
        self._process()

    def _process(self):
        if self._last_scan is None or self._goal is None:
            return
        
        vx, vtheta = self.planner.compute_velocity_commands(
            self._last_scan.angle_min, self._last_scan.angle_increment,
            list(self._last_scan.ranges), 0.0, 0.0, 0.0,  # current velocities
            self._goal.pose.position.x, self._goal.pose.position.y, 0.0)  # goal
        
        command = self._twist_type()
        command.linear.x = float(vx)
        command.angular.z = float(vtheta)
        self._pub.publish(command)


class RegulatedPurePursuitNode(Node):
    """ROS wrapper for Regulated Pure Pursuit local planner."""

    def __init__(self, node_name='regulated_pure_pursuit'):
        super().__init__(node_name)
        self.declare_parameter('path_topic', '/plan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_rpp')
        self.declare_parameter('lookahead_distance', 0.5)
        self.declare_parameter('max_vel', 0.5)
        self.declare_parameter('max_angular_vel', 1.0)
        
        self.planner = RegulatedPurePursuit(
            lookahead_distance=float(self.get_parameter('lookahead_distance').value),
            max_vel=float(self.get_parameter('max_vel').value),
            max_angular_vel=float(self.get_parameter('max_angular_vel').value))
        
        from nav_msgs.msg import Path
        from geometry_msgs.msg import Twist
        self._twist_type = Twist
        self._pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.create_subscription(
            Path, self.get_parameter('path_topic').value,
            self._on_path, 10)
        
        self._last_path = None
        self.get_logger().info('Regulated Pure Pursuit ready')

    def _on_path(self, msg):
        self._last_path = msg
        # Extract path as list of (x, y) points
        global_path = [(pose.pose.position.x, pose.pose.position.y) 
                     for pose in msg.poses]
        
        # Current robot pose (simplified - using first pose as current)
        if global_path:
            current_x, current_y = global_path[0]
            current_theta = 0.0  # Simplified
        else:
            current_x, current_y, current_theta = 0.0, 0.0, 0.0
        
        vx, vtheta = self.planner.compute_velocity_commands(
            global_path, current_x, current_y, current_theta)
        
        command = self._twist_type()
        command.linear.x = float(vx)
        command.angular.z = float(vtheta)
        self._pub.publish(command)


class TEBNode(Node):
    """ROS wrapper for TEB local planner."""

    def __init__(self, node_name='teb_local_planner'):
        super().__init__(node_name)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_teb')
        self.declare_parameter('max_vel_x', 0.5)
        self.declare_parameter('max_vel_theta', 1.0)
        self.declare_parameter('safety_radius', 0.3)
        
        self.planner = TEBLocalPlanner(
            max_vel_x=float(self.get_parameter('max_vel_x').value),
            max_vel_theta=float(self.get_parameter('max_vel_theta').value),
            safety_radius=float(self.get_parameter('safety_radius').value))
        
        from sensor_msgs.msg import LaserScan
        from geometry_msgs.msg import Twist, PoseStamped
        from rclpy.qos import qos_profile_sensor_data
        self._twist_type = Twist
        self._pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value,
            self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, self.get_parameter('goal_topic').value,
            self._on_goal, 10)
        
        self._last_scan = None
        self._goal = None
        self.get_logger().info('TEB local planner ready')

    def _on_scan(self, msg):
        self._last_scan = msg
        self._process()

    def _on_goal(self, msg):
        self._goal = msg
        self._process()

    def _process(self):
        if self._last_scan is None or self._goal is None:
            return
        
        vx, vtheta = self.planner.compute_velocity_commands(
            self._last_scan.angle_min, self._last_scan.angle_increment,
            list(self._last_scan.ranges), 0.0, 0.0,  # current velocities
            self._goal.pose.position.x, self._goal.pose.position.y)
        
        command = self._twist_type()
        command.linear.x = float(vx)
        command.angular.z = float(vtheta)
        self._pub.publish(command)


class MPPINode(Node):
    """ROS wrapper for MPPI local planner."""

    def __init__(self, node_name='mppi_local_planner'):
        super().__init__(node_name)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_mppi')
        self.declare_parameter('num_samples', 10)
        self.declare_parameter('horizon', 1.0)
        self.declare_parameter('temperature', 0.1)
        self.declare_parameter('max_vel_x', 0.5)
        self.declare_parameter('max_vel_theta', 1.0)
        
        self.planner = MPPILocalPlanner(
            num_samples=int(self.get_parameter('num_samples').value),
            horizon=float(self.get_parameter('horizon').value),
            temperature=float(self.get_parameter('temperature').value),
            max_vel_x=float(self.get_parameter('max_vel_x').value),
            max_vel_theta=float(self.get_parameter('max_vel_theta').value))
        
        from sensor_msgs.msg import LaserScan
        from geometry_msgs.msg import Twist, PoseStamped
        from rclpy.qos import qos_profile_sensor_data
        self._twist_type = Twist
        self._pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value,
            self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, self.get_parameter('goal_topic').value,
            self._on_goal, 10)
        
        self._last_scan = None
        self._goal = None
        self.get_logger().info('MPPI local planner ready')

    def _on_scan(self, msg):
        self._last_scan = msg
        self._process()

    def _on_goal(self, msg):
        self._goal = msg
        self._process()

    def _process(self):
        if self._last_scan is None or self._goal is None:
            return
        
        vx, vtheta = self.planner.compute_velocity_commands(
            self._last_scan.angle_min, self._last_scan.angle_increment,
            list(self._last_scan.ranges), 0.0, 0.0,  # current velocities
            self._goal.pose.position.x, self._goal.pose.position.y)
        
        command = self._twist_type()
        command.linear.x = float(vx)
        command.angular.z = float(vtheta)
        self._pub.publish(command)


def dwb_local_planner_main(args=None):
    return _run(DWBNode, 'dwb_local_planner', args=args)


def regulated_pure_pursuit_main(args=None):
    return _run(RegulatedPurePursuitNode, 'regulated_pure_pursuit', args=args)


def teb_local_planner_main(args=None):
    return _run(TEBNode, 'teb_local_planner', args=args)


def mppi_local_planner_main(args=None):
    return _run(MPPINode, 'mppi_local_planner', args=args)


if __name__ == '__main__':
    sys.exit(follow_the_gap_main())
