#!/usr/bin/env python3
import math
from enum import Enum

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ExplorerState(Enum):
    WAITING_FOR_SCAN = 'waiting_for_scan'
    WAITING_FOR_ODOM = 'waiting_for_odom'
    STARTUP_SCAN = 'startup_scan'
    DRIVE = 'drive'
    TURN = 'turn'
    EMERGENCY_BACKUP = 'emergency_backup'
    COMPLETE = 'complete'


class AutonomousScanExplorer(Node):
    def __init__(self):
        super().__init__('autonomous_scan_explorer')

        self.declare_parameter('cmd_vel_topic', '/key_vel')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/bumperbot_controller/odom')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('forward_speed', 0.08)
        self.declare_parameter('slow_speed', 0.03)
        self.declare_parameter('reverse_speed', -0.04)
        self.declare_parameter('turn_speed', 0.5)
        self.declare_parameter('startup_spin_time', 2.0)
        self.declare_parameter('emergency_distance', 0.15)
        self.declare_parameter('stop_distance', 0.25)
        self.declare_parameter('slow_distance', 0.55)
        self.declare_parameter('scan_complete_distance', 1.0)
        self.declare_parameter('candidate_width_degrees', 8.0)
        self.declare_parameter('side_clearance', 0.8)
        self.declare_parameter('visited_cell_size', 0.35)
        self.declare_parameter('visited_lookahead', 0.9)
        self.declare_parameter('heading_tolerance', 0.2)
        self.declare_parameter('scan_timeout', 1.0)
        self.declare_parameter('min_exploration_time', 300.0)
        self.declare_parameter('min_unknown_improvement', 25)
        self.declare_parameter('enable_map_completion', False)

        self.cmd_vel_topic = self.get_parameter_value('cmd_vel_topic').string_value
        self.scan_topic = self.get_parameter_value('scan_topic').string_value
        self.odom_topic = self.get_parameter_value('odom_topic').string_value
        self.map_topic = self.get_parameter_value('map_topic').string_value
        self.forward_speed = self.get_parameter_value('forward_speed').double_value
        self.slow_speed = self.get_parameter_value('slow_speed').double_value
        self.reverse_speed = self.get_parameter_value('reverse_speed').double_value
        self.turn_speed = self.get_parameter_value('turn_speed').double_value
        self.startup_spin_time = self.get_parameter_value('startup_spin_time').double_value
        self.emergency_distance = self.get_parameter_value(
            'emergency_distance'
        ).double_value
        self.stop_distance = self.get_parameter_value('stop_distance').double_value
        self.slow_distance = self.get_parameter_value('slow_distance').double_value
        self.scan_complete_distance = self.get_parameter_value(
            'scan_complete_distance'
        ).double_value
        self.candidate_width = math.radians(
            self.get_parameter_value('candidate_width_degrees').double_value
        )
        self.side_clearance = self.get_parameter_value('side_clearance').double_value
        self.visited_cell_size = self.get_parameter_value(
            'visited_cell_size'
        ).double_value
        self.visited_lookahead = self.get_parameter_value(
            'visited_lookahead'
        ).double_value
        self.heading_tolerance = self.get_parameter_value(
            'heading_tolerance'
        ).double_value
        self.scan_timeout = self.get_parameter_value('scan_timeout').double_value
        self.min_exploration_time = self.get_parameter_value(
            'min_exploration_time'
        ).double_value
        self.min_unknown_improvement = self.get_parameter_value(
            'min_unknown_improvement'
        ).integer_value
        self.enable_map_completion = self.get_parameter_value(
            'enable_map_completion'
        ).bool_value

        self.publisher_ = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, qos_profile_sensor_data
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_callback, 10
        )

        self.latest_scan = None
        self.last_scan_time = None
        self.current_pose = None
        self.visited_cells = {}
        self.target_heading = 0.0
        self.state = ExplorerState.WAITING_FOR_SCAN
        self.start_time = self.now_seconds()
        self.best_unknown_count = None
        self.completion_reported = False
        self.last_log_time = 0.0

        self.timer = self.create_timer(0.1, self.control_callback)

        self.get_logger().info(
            'Autonomous scan explorer publishing %s, reading %s, reading %s'
            % (self.cmd_vel_topic, self.scan_topic, self.odom_topic)
        )

    def get_parameter_value(self, name):
        return self.get_parameter(name).get_parameter_value()

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.last_scan_time = self.now_seconds()

    def odom_callback(self, msg):
        pose = msg.pose.pose
        yaw = self.yaw_from_quaternion(pose.orientation)
        self.current_pose = (pose.position.x, pose.position.y, yaw)
        cell = self.cell_for_position(pose.position.x, pose.position.y)
        self.visited_cells[cell] = self.visited_cells.get(cell, 0) + 1

    def map_callback(self, msg):
        if not self.enable_map_completion:
            return

        unknown_count = msg.data.count(-1)
        now = self.now_seconds()

        if self.best_unknown_count is None:
            self.best_unknown_count = unknown_count
            return

        improvement = self.best_unknown_count - unknown_count
        if improvement >= self.min_unknown_improvement:
            self.best_unknown_count = unknown_count

        ran_long_enough = now - self.start_time >= self.min_exploration_time
        visited_enough_places = len(self.visited_cells) >= 25
        if unknown_count == 0 and ran_long_enough and visited_enough_places:
            self.state = ExplorerState.COMPLETE

    def control_callback(self):
        now = self.now_seconds()

        if self.state == ExplorerState.COMPLETE:
            self.publish_velocity(0.0, 0.0)
            if not self.completion_reported:
                self.get_logger().info('Map exploration complete; robot stopped.')
                self.completion_reported = True
            return

        if self.latest_scan is None:
            self.state = ExplorerState.WAITING_FOR_SCAN
            self.publish_velocity(0.0, 0.0)
            self.throttled_log('Waiting for LaserScan data before moving.')
            return

        if now - self.last_scan_time > self.scan_timeout:
            self.state = ExplorerState.WAITING_FOR_SCAN
            self.publish_velocity(0.0, 0.0)
            self.throttled_log('LaserScan timed out; stopping robot.')
            return

        if self.current_pose is None:
            self.state = ExplorerState.WAITING_FOR_ODOM
            self.publish_velocity(0.0, 0.0)
            self.throttled_log('Waiting for odometry before moving.')
            return

        if now - self.start_time < self.startup_spin_time:
            self.state = ExplorerState.STARTUP_SCAN
            self.publish_velocity(0.0, self.turn_speed * 0.35)
            return

        sectors = self.scan_sectors(self.latest_scan)
        linear_x, angular_z = self.choose_velocity(sectors)
        self.publish_velocity(linear_x, angular_z)

    def choose_velocity(self, sectors):
        front = sectors['front']
        front_left = sectors['front_left']
        front_right = sectors['front_right']
        closest_front = min(front, front_left, front_right)

        forward_already_scanned = (
            front < self.scan_complete_distance
            and self.projected_visit_count(0.0) > 0
        )
        best_heading, best_clearance = self.best_open_heading(
            allow_forward=not forward_already_scanned
        )
        self.target_heading = best_heading

        if closest_front < self.emergency_distance:
            self.state = ExplorerState.EMERGENCY_BACKUP
            avoid_heading = self.wall_avoidance_heading(best_heading, sectors)
            return self.reverse_speed, self.turn_command(avoid_heading)

        if closest_front < self.stop_distance:
            self.state = ExplorerState.TURN
            avoid_heading = self.wall_avoidance_heading(best_heading, sectors)
            return 0.0, self.turn_command(avoid_heading)

        if forward_already_scanned and abs(best_heading) > self.heading_tolerance:
            self.state = ExplorerState.TURN
            return 0.0, self.turn_command(best_heading)

        if abs(best_heading) > self.heading_tolerance:
            self.state = ExplorerState.TURN
            return 0.0, self.turn_command(best_heading)

        self.state = ExplorerState.DRIVE
        speed = self.forward_speed
        if front < self.slow_distance or best_clearance < self.slow_distance:
            speed = self.slow_speed

        return speed, self.turn_command(best_heading) * 0.4

    def best_open_heading(self, allow_forward=True):
        candidate_degrees = (-150, -120, -90, -60, -30, 0, 30, 60, 90, 120, 150, 180)
        best_heading = math.radians(90)
        best_score = -float('inf')
        best_clearance = 0.0

        for degrees in candidate_degrees:
            if degrees == 0 and not allow_forward:
                continue

            heading = math.radians(degrees)
            clearance = self.min_range_around(
                self.latest_scan,
                heading,
                self.candidate_width,
            )
            if clearance < self.stop_distance:
                continue

            visit_count = self.projected_visit_count(heading)
            score = min(clearance, self.scan_complete_distance)
            score -= abs(heading) * 0.15
            score -= visit_count * 2.5

            if visit_count == 0:
                score += 1.0

            if degrees == 0 and visit_count == 0:
                score += 0.4

            if score > best_score:
                best_score = score
                best_heading = heading
                best_clearance = clearance

        if best_score == -float('inf'):
            sectors = self.scan_sectors(self.latest_scan)
            best_heading = math.radians(90 if sectors['left'] >= sectors['right'] else -90)
            best_clearance = max(sectors['left'], sectors['right'])

        return best_heading, best_clearance

    def turn_command(self, heading):
        if abs(heading) < 0.05:
            return 0.0
        return self.clamp(heading * 1.3, -self.turn_speed, self.turn_speed)

    def wall_avoidance_heading(self, heading, sectors):
        if abs(heading) >= self.heading_tolerance:
            return heading
        return math.radians(90 if sectors['left'] >= sectors['right'] else -90)

    def scan_sectors(self, scan):
        return {
            'front': self.min_range(scan, -25.0, 25.0),
            'front_left': self.min_range(scan, 15.0, 75.0),
            'front_right': self.min_range(scan, -75.0, -15.0),
            'left': self.min_range(scan, 60.0, 120.0),
            'right': self.min_range(scan, -120.0, -60.0),
        }

    def min_range_around(self, scan, center_angle, width):
        values = []

        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance):
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue

            angle = self.normalize_angle(scan.angle_min + index * scan.angle_increment)
            error = abs(self.normalize_angle(angle - center_angle))
            if error <= width:
                values.append(distance)

        if not values:
            return float('inf')
        return min(values)

    def min_range(self, scan, start_degrees, end_degrees):
        start = math.radians(start_degrees)
        end = math.radians(end_degrees)
        values = []

        for index, distance in enumerate(scan.ranges):
            if not math.isfinite(distance):
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue

            angle = self.normalize_angle(scan.angle_min + index * scan.angle_increment)
            if start <= angle <= end:
                values.append(distance)

        if not values:
            return float('inf')
        return min(values)

    def publish_velocity(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.publisher_.publish(twist)
        self.log_motion(linear_x, angular_z)

    def projected_visit_count(self, relative_heading):
        if self.current_pose is None:
            return 0

        x, y, yaw = self.current_pose
        heading = yaw + relative_heading
        projected_x = x + math.cos(heading) * self.visited_lookahead
        projected_y = y + math.sin(heading) * self.visited_lookahead
        return self.visited_cells.get(
            self.cell_for_position(projected_x, projected_y),
            0,
        )

    def cell_for_position(self, x, y):
        return (
            int(math.floor(x / self.visited_cell_size)),
            int(math.floor(y / self.visited_cell_size)),
        )

    def throttled_log(self, message):
        now = self.now_seconds()
        if now - self.last_log_time > 2.0:
            self.get_logger().warn(message)
            self.last_log_time = now

    def log_motion(self, linear_x, angular_z):
        now = self.now_seconds()
        if now - self.last_log_time <= 2.0:
            return

        if self.latest_scan is None:
            return

        sectors = self.scan_sectors(self.latest_scan)
        self.get_logger().info(
            'state=%s cmd=(%.2f, %.2f) front=%.2f left=%.2f right=%.2f visited=%d'
            % (
                self.state.value,
                linear_x,
                angular_z,
                sectors['front'],
                sectors['left'],
                sectors['right'],
                len(self.visited_cells),
            )
        )
        self.last_log_time = now

    @staticmethod
    def clamp(value, minimum, maximum):
        return max(minimum, min(value, maximum))

    @staticmethod
    def normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def stop(self):
        self.publish_velocity(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousScanExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
