#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class BumperbotWallFollower(Node):
    RECOVERY_ROTATE = 'rotate'
    RECOVERY_WAIT = 'wait'
    RECOVERY_REVERSE = 'reverse'

    def __init__(self):
        super().__init__('bumperbot_wall_follower')

        self.cmd_vel_pub = self.create_publisher(Twist, '/key_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # SETTINGS
        self.speed_multiplier = 1.5
        self.target_dist = 0.8
        self.forward_speed = 0.15
        self.p_gain = 1.2
        self.angle_gain = 2.0
        self.safety_dist = 0.4
        self.wall_detection_threshold = 1.0
        self.min_points_for_line_fit = 5
        self.stuck_timeout = 2.0
        self.recovery_rotate_time = 1.0
        self.recovery_wait_time = 0.5
        self.recovery_reverse_time = 1.0
        self.recovery_rotate_speed = 0.7
        self.recovery_reverse_speed = -0.08
        self.rear_safety_dist = 0.25

        self.blocked_since = None
        self.recovery_state = None
        self.recovery_step_started_at = None
        self.recovery_turn_direction = 1.0
        self.last_log_time = 0.0

        self.get_logger().info(f"Wall Follower Started. Speed Multiplier: {self.speed_multiplier}x")

    def scan_callback(self, msg):
        cmd = Twist()
        now = self.now_seconds()

        # 1. EMERGENCY TURN
        dist_front = self.distance_in_sector(msg, 170.0, -170.0)

        if self.recovery_state is not None:
            self.publish_recovery_command(msg, now)
            return

        if dist_front < self.safety_dist:
            self.update_blocked_state(now)

            if now - self.blocked_since >= self.stuck_timeout:
                self.start_recovery(msg, now, dist_front)
                self.publish_recovery_command(msg, now)
                return

            # Multiplier applied here for snappy emergency response
            cmd.linear.x = 0.05 * self.speed_multiplier
            cmd.angular.z = 0.8 * self.speed_multiplier
            self.throttled_warn(f"Emergency turn! Front distance: {dist_front:.2f}m")
            self.cmd_vel_pub.publish(cmd)
            return
        self.blocked_since = None

        # 2. Collect points for line fitting
        wall_x_robot = []
        wall_y_robot = []
        min_lidar_angle_deg = 45
        max_lidar_angle_deg = 135

        for i, dist in enumerate(msg.ranges):
            if not (math.isfinite(dist) and dist > msg.range_min and dist < self.wall_detection_threshold):
                continue

            lidar_angle_deg = math.degrees(msg.angle_min + (i * msg.angle_increment))
            if lidar_angle_deg > 180: lidar_angle_deg -= 360

            if min_lidar_angle_deg <= lidar_angle_deg <= max_lidar_angle_deg:
                angle_from_forward_rad = math.radians(lidar_angle_deg - 180)
                x_temp = dist * math.cos(angle_from_forward_rad)
                y_temp = dist * math.sin(angle_from_forward_rad)

                wall_x_robot.append(x_temp)
                wall_y_robot.append(-y_temp)

                # 3. Line Fitting and Wall Following Logic
        if len(wall_x_robot) >= self.min_points_for_line_fit:
            m, c = np.polyfit(wall_x_robot, wall_y_robot, 1)
            wall_angle_rad = math.atan2(m, 1)

            if c < 0:
                self.get_logger().warn(f"Wall on wrong side, moving straight.")
                cmd.linear.x = self.forward_speed * self.speed_multiplier
                cmd.angular.z = 0.0
            else:
                current_dist_to_wall = c / math.sqrt(m ** 2 + 1)
                error_dist = self.target_dist - current_dist_to_wall

                # Apply multiplier to the final linear and angular results
                cmd.linear.x = self.forward_speed * self.speed_multiplier

                angular_z_dist_correction = error_dist * self.p_gain
                angular_z_angle_correction = -wall_angle_rad * self.angle_gain
                cmd.angular.z = (angular_z_dist_correction + angular_z_angle_correction) * self.speed_multiplier

                self.get_logger().info(f"Following wall at {self.speed_multiplier}x speed")

        else:
            cmd.linear.x = self.forward_speed * self.speed_multiplier
            cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)

    def update_blocked_state(self, now):
        if self.blocked_since is None:
            self.blocked_since = now

    def start_recovery(self, msg, now, dist_front):
        self.recovery_state = self.RECOVERY_ROTATE
        self.recovery_step_started_at = now
        self.recovery_turn_direction = self.best_recovery_turn_direction(msg)
        self.blocked_since = None
        self.get_logger().warn(
            f"Stuck detected at {dist_front:.2f}m. Starting rotate-wait-reverse recovery."
        )

    def publish_recovery_command(self, msg, now):
        cmd = Twist()
        elapsed = now - self.recovery_step_started_at

        if self.recovery_state == self.RECOVERY_ROTATE:
            if elapsed >= self.recovery_rotate_time:
                self.set_recovery_state(self.RECOVERY_WAIT, now)
            else:
                cmd.angular.z = (
                    self.recovery_turn_direction
                    * self.recovery_rotate_speed
                    * self.speed_multiplier
                )

        if self.recovery_state == self.RECOVERY_WAIT:
            if now - self.recovery_step_started_at >= self.recovery_wait_time:
                self.set_recovery_state(self.RECOVERY_REVERSE, now)

        if self.recovery_state == self.RECOVERY_REVERSE:
            rear_dist = self.distance_in_sector(msg, -10.0, 10.0)
            if rear_dist < self.rear_safety_dist:
                self.finish_recovery("rear path blocked")
            elif now - self.recovery_step_started_at >= self.recovery_reverse_time:
                self.finish_recovery("complete")
            else:
                cmd.linear.x = self.recovery_reverse_speed * self.speed_multiplier

        self.cmd_vel_pub.publish(cmd)

    def set_recovery_state(self, state, now):
        self.recovery_state = state
        self.recovery_step_started_at = now
        self.get_logger().warn(f"Recovery step: {state}")

    def finish_recovery(self, reason):
        self.get_logger().warn(f"Recovery finished: {reason}. Resuming wall follow.")
        self.recovery_state = None
        self.recovery_step_started_at = None

    def best_recovery_turn_direction(self, msg):
        left_dist = self.distance_in_sector(msg, 60.0, 120.0)
        right_dist = self.distance_in_sector(msg, -120.0, -60.0)
        return 1.0 if left_dist >= right_dist else -1.0

    def distance_in_sector(self, msg, start_deg, end_deg):
        min_dist = float('inf')

        for i, dist in enumerate(msg.ranges):
            if not (math.isfinite(dist) and msg.range_min < dist <= msg.range_max):
                continue

            angle_deg = math.degrees(msg.angle_min + (i * msg.angle_increment))
            angle_deg = self.normalize_degrees(angle_deg)
            if self.angle_in_sector(angle_deg, start_deg, end_deg):
                min_dist = min(min_dist, dist)

        return min_dist

    def throttled_warn(self, message):
        now = self.now_seconds()
        if now - self.last_log_time >= 1.0:
            self.get_logger().warn(message)
            self.last_log_time = now

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def angle_in_sector(angle_deg, start_deg, end_deg):
        start_deg = BumperbotWallFollower.normalize_degrees(start_deg)
        end_deg = BumperbotWallFollower.normalize_degrees(end_deg)

        if start_deg <= end_deg:
            return start_deg <= angle_deg <= end_deg
        return angle_deg >= start_deg or angle_deg <= end_deg

    @staticmethod
    def normalize_degrees(angle_deg):
        return (angle_deg + 180.0) % 360.0 - 180.0


def main(args=None):
    rclpy.init(args=args)
    node = BumperbotWallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cmd_vel_pub.publish(Twist())
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
