#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

def euler_from_quaternion(quaternion):
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

class MapCoverageController(Node):
    class State:
        NAVIGATE_TO_WALL = "NAVIGATE_TO_WALL"
        ALIGN_TO_WALL = "ALIGN_TO_WALL"
        FOLLOW_WALL = "FOLLOW_WALL"
        TURN_TO_CROSS = "TURN_TO_CROSS"
        CROSS_ROOM = "CROSS_ROOM"
        SHIFT_LANE_TURN = "SHIFT_LANE_TURN"
        SHIFT_LANE_MOVE = "SHIFT_LANE_MOVE"
        TURN_BACK = "TURN_BACK"

    def __init__(self):
        super().__init__('map_coverage_controller')

        self.cmd_vel_pub = self.create_publisher(Twist, '/key_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Settings
        self.target_dist = 0.5 
        self.forward_speed = 0.15
        self.rotate_speed = 0.7
        self.robot_size = 0.45 # Shift distance
        self.wall_threshold = 0.8
        self.infinity_threshold = 2.5
        
        # State variables
        self.state = self.State.NAVIGATE_TO_WALL
        self.current_pose = None # (x, y, yaw)
        self.initial_wall_yaw = None # The "Imaginary Line" orientation
        self.target_heading = 0.0
        self.start_lane_pos = None
        self.turn_direction = -1 # -1 for right, 1 for left

        self.get_logger().info("Map Coverage Controller (Imaginary Line Alignment)")

    def odom_callback(self, msg):
        yaw = euler_from_quaternion(msg.pose.pose.orientation)
        self.current_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def scan_callback(self, msg):
        if self.current_pose is None:
            return

        cmd = Twist()
        
        # Sector distances (LiDAR pi is front)
        dist_front = self.get_min_dist_fixed(msg, -0.2, 0.2)
        dist_left = self.get_min_dist_fixed(msg, 1.3, 1.8)
        
        current_state = self.state

        if self.state == self.State.NAVIGATE_TO_WALL:
            if dist_front < self.target_dist + 0.2:
                self.get_logger().info("Wall found. Establishing Imaginary Line...")
                self.state = self.State.ALIGN_TO_WALL
                # Align parallel to wall
                self.target_heading = self.normalize_angle(self.current_pose[2] - math.pi/2)
            else:
                cmd.linear.x = self.forward_speed

        elif self.state == self.State.ALIGN_TO_WALL:
            if self.rotate_to_target(cmd):
                # Save this yaw as our global reference (Imaginary Line)
                self.initial_wall_yaw = self.current_pose[2]
                self.get_logger().info(f"Imaginary Line set at {math.degrees(self.initial_wall_yaw):.1f} deg.")
                self.state = self.State.FOLLOW_WALL

        elif self.state == self.State.FOLLOW_WALL:
            # Follow physical wall but keep checking for "infinite" space to our left
            if dist_left > self.infinity_threshold:
                self.get_logger().info("Wall ended. Turning PERPENDICULAR to Imaginary Line.")
                self.state = self.State.TURN_TO_CROSS
                # Turn 90 deg relative to the INITIAL line, not current heading
                self.target_heading = self.normalize_angle(self.initial_wall_yaw - math.pi/2)
            elif dist_front < self.target_dist:
                self.get_logger().info("Corner hit. Turning PERPENDICULAR to Imaginary Line.")
                self.state = self.State.TURN_TO_CROSS
                self.target_heading = self.normalize_angle(self.initial_wall_yaw - math.pi/2)
            else:
                # Follow physical wall (PID)
                error = self.target_dist - dist_left
                cmd.linear.x = self.forward_speed
                cmd.angular.z = -error * 2.0

        elif self.state == self.State.TURN_TO_CROSS:
            if self.rotate_to_target(cmd):
                self.get_logger().info("Crossing room on fixed axis...")
                self.state = self.State.CROSS_ROOM

        elif self.state == self.State.CROSS_ROOM:
            # Maintain orientation relative to initial wall line while crossing
            angle_error = self.normalize_angle(self.target_heading - self.current_pose[2])
            cmd.angular.z = angle_error * 1.5
            
            if dist_front < self.target_dist:
                self.get_logger().info("Reached opposite wall. Shifting lane along Imaginary Axis.")
                self.state = self.State.SHIFT_LANE_TURN
                # Turn to face parallel to the Imaginary Line (forward or backward)
                self.target_heading = self.normalize_angle(self.initial_wall_yaw if self.turn_direction == -1 else self.initial_wall_yaw + math.pi)
            else:
                cmd.linear.x = self.forward_speed

        elif self.state == self.State.SHIFT_LANE_TURN:
            if self.rotate_to_target(cmd):
                self.get_logger().info("Shifting...")
                self.state = self.State.SHIFT_LANE_MOVE
                self.start_lane_pos = self.current_pose

        elif self.state == self.State.SHIFT_LANE_MOVE:
            # Maintain orientation during move
            angle_error = self.normalize_angle(self.target_heading - self.current_pose[2])
            cmd.angular.z = angle_error * 1.5
            
            dist_moved = self.get_distance_from_start()
            if dist_moved >= self.robot_size or dist_front < self.target_dist:
                self.get_logger().info("Shift complete. Turning back.")
                self.state = self.State.TURN_BACK
                # Turn to face opposite direction of previous cross
                # If we were at initial_yaw - pi/2, now we go to initial_yaw + pi/2 (or vice versa)
                # Basically, we alternate the cross direction
                self.target_heading = self.normalize_angle(self.initial_wall_yaw + math.pi/2) if self.turn_direction == -1 else self.normalize_angle(self.initial_wall_yaw - math.pi/2)
                self.turn_direction *= -1 
            else:
                cmd.linear.x = self.forward_speed

        elif self.state == self.State.TURN_BACK:
            if self.rotate_to_target(cmd):
                self.get_logger().info("Crossing back on fixed axis...")
                self.state = self.State.CROSS_ROOM

        if self.state != current_state:
            self.get_logger().info(f"Transitioned to {self.state}")

        self.cmd_vel_pub.publish(cmd)

    def rotate_to_target(self, cmd):
        angle_diff = self.normalize_angle(self.target_heading - self.current_pose[2])
        if abs(angle_diff) < 0.05:
            return True
        cmd.angular.z = self.rotate_speed if angle_diff > 0 else -self.rotate_speed
        return False

    def get_distance_from_start(self):
        return math.sqrt((self.current_pose[0] - self.start_lane_pos[0])**2 + 
                         (self.current_pose[1] - self.start_lane_pos[1])**2)

    def get_min_dist_fixed(self, msg, start_rad, end_rad):
        min_dist = float('inf')
        for i, dist in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            angle = self.normalize_angle(angle - math.pi)
            if start_rad <= angle <= end_rad:
                if math.isfinite(dist) and dist > msg.range_min:
                    min_dist = min(min_dist, dist)
        return min_dist

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

def main(args=None):
    rclpy.init(args=args)
    node = MapCoverageController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cmd_vel_pub.publish(Twist())
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
