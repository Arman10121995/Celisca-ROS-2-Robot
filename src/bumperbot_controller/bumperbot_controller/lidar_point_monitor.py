#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped  # For Nav2 goals
from geometry_msgs.msg import PointStamped  # For transforming lidar points

import math
import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs  # This import helps ensure PointStamped is registered with tf2_ros

# Nav2 Action Client imports
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from rclpy.task import Future  # For handling async action client futures

import threading  # For listening to user input
import pygame  # For visualization
import sys  # For pygame exit


class LidarPointMonitor(Node):
    GRID_RESOLUTION = 2.0  # meters for each side of the square grid cell

    def __init__(self):
        super().__init__('lidar_point_monitor')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)

        self.subscription  # prevent unused variable warning

        self.get_logger().info('Lidar Point Monitor Node has been started.')

        # Initialize min/max coordinates to extreme values
        self.min_x = float('inf')
        self.max_x = float('-inf')
        self.min_y = float('inf')
        self.max_y = float('-inf')

        # Store unique lidar points as (x, y) tuples in the target_frame
        self.unique_lidar_points = set()

        # Store all potential grid points (before filtering for emptiness)
        self.initial_grid_points_all = set()
        # Store generated grid points (these will be the *navigable* points after filtering)
        self.grid_points = set()  # This will contain (gx, gy) of empty cells

        # TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # The target frame for transformations (e.g., 'odom' or 'map')
        # 'map' is preferred for globally consistent mapping
        self.target_frame = 'map'
        # The robot's base frame (e.g., 'base_link')
        self.robot_base_frame = 'base_link'

        # Timer will be created later, after user input
        self.timer = None
        self.monitoring_active = False  # Flag to control grid generation and navigation

        # Navigation state variables
        self.is_navigating = False  # True if we have an active goal with Nav2
        self.current_goal_point = None  # The final goal point we are trying to reach (x,y tuple)
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self._send_goal_future = None
        self._get_result_future = None
        self.goal_handle = None

    def listener_callback(self, msg):
        # self.get_logger().info(f'Received LaserScan message from frame: {msg.header.frame_id}')

        # Iterate through the ranges and calculate (x, y) for each point
        for i, range_value in enumerate(msg.ranges):
            # Check if the range value is valid (not infinite and within min/max)
            if msg.range_min <= range_value <= msg.range_max and not math.isinf(range_value):

                # Calculate the angle for this specific range measurement
                angle = msg.angle_min + i * msg.angle_increment

                # Create a PointStamped message for the lidar point in its own frame
                point_in_laser_frame = PointStamped()
                point_in_laser_frame.header.frame_id = msg.header.frame_id
                point_in_laser_frame.header.stamp = msg.header.stamp
                point_in_laser_frame.point.x = range_value * math.cos(angle)
                point_in_laser_frame.point.y = range_value * math.sin(angle)
                point_in_laser_frame.point.z = 0.0  # Lidar is typically 2D, so z is 0

                try:
                    # Transform the point to the target_frame (e.g., 'map')
                    transformed_point = self.tf_buffer.transform(
                        point_in_laser_frame,
                        self.target_frame,
                        timeout=rclpy.duration.Duration(seconds=0.1)
                    )

                    x_global = transformed_point.point.x
                    y_global = transformed_point.point.y

                    # Round coordinates for practical uniqueness in the set
                    # Adjust rounding precision as needed. 3 decimal places (mm) is usually sufficient.
                    rounded_x = round(x_global, 3)
                    rounded_y = round(y_global, 3)

                    # Add to the set of unique points
                    self.unique_lidar_points.add((rounded_x, rounded_y))

                    # Update overall min/max coordinates
                    if x_global < self.min_x:
                        self.min_x = x_global
                    if x_global > self.max_x:
                        self.max_x = x_global
                    if y_global < self.min_y:
                        self.min_y = y_global
                    if y_global > self.max_y:
                        self.max_y = y_global

                except TransformException as ex:
                    # self.get_logger().warn(f'Could not transform point from {point_in_laser_frame.header.frame_id} to {self.target_frame}: {ex}')
                    pass  # Suppress frequent warnings if transforms are temporarily unavailable

    def get_overall_min_max_coordinates(self):
        """
        Returns the highest and lowest x and y values observed from lidar points
        in the target_frame (e.g., 'map').
        Returns (min_x, max_x, min_y, max_y).
        If no valid points have been received, returns (inf, -inf, inf, -inf).
        """
        return (self.min_x, self.max_x, self.min_y, self.max_y)

    def get_unique_lidar_points(self):
        """
        Returns a set of all unique (x, y) lidar points observed so far,
        transformed into the target_frame (e.g., 'map').
        """
        return self.unique_lidar_points

    def generate_grid_points(self):
        """
        Generates a grid of points covering the bounding box of the observed lidar points.
        Each point represents the bottom-left corner of a GRID_RESOLUTION x GRID_RESOLUTION square meter area.
        The grid points are stored in self.initial_grid_points_all.
        """
        # Only generate grid if we have valid min/max coordinates
        if math.isinf(self.min_x):
            self.initial_grid_points_all.clear()  # Ensure grid is empty if no points
            return

        grid_size = self.GRID_RESOLUTION
        new_initial_grid_points = set()

        # Calculate the starting points for the grid, aligned to multiples of grid_size
        # This ensures the grid points are at 0, 2, 4, ... or -2, 0, 2, ...
        # The floor ensures we start at or before min_x
        start_x = math.floor(self.min_x / grid_size) * grid_size
        start_y = math.floor(self.min_y / grid_size) * grid_size

        # The ceiling ensures we end at or after max_x
        end_x = math.ceil(self.max_x / grid_size) * grid_size
        end_y = math.ceil(self.max_y / grid_size) * grid_size

        x = start_x
        while x <= end_x:
            y = start_y
            while y <= end_y:
                new_initial_grid_points.add((round(x, 3), round(y, 3)))
                y += grid_size
            x += grid_size

        self.initial_grid_points_all = new_initial_grid_points

    def _get_occupied_cell_indices(self):
        """Helper to get current occupied cell indices from unique_lidar_points."""
        grid_size = self.GRID_RESOLUTION
        occupied_cell_indices = set()
        for lx, ly in self.unique_lidar_points:
            cell_x_idx = math.floor(lx / grid_size)
            cell_y_idx = math.floor(ly / grid_size)
            occupied_cell_indices.add((cell_x_idx, cell_y_idx))
        return occupied_cell_indices

    def filter_empty_grid_points(self):
        """
        Filters the generated grid points, keeping only those that represent
        empty cells (i.e., cells that do NOT contain any unique lidar points).
        A grid point (gx, gy) represents the bottom-left corner of the cell [gx, gx+res) x [gy, gy+res).
        Returns the set of occupied cell indices for use in navigation checks.
        """
        if not self.initial_grid_points_all:
            self.grid_points.clear()
            return set()  # Return empty set of occupied cells

        grid_size = self.GRID_RESOLUTION
        occupied_cell_indices = self._get_occupied_cell_indices()

        filtered_grid_points = set()
        for gx, gy in self.initial_grid_points_all:
            # Determine the cell index for the current grid point (gx, gy)
            cell_x_idx = math.floor(gx / grid_size)
            cell_y_idx = math.floor(gy / grid_size)

            # If this cell is NOT marked as occupied by a lidar point, keep the grid point
            if (cell_x_idx, cell_y_idx) not in occupied_cell_indices:
                filtered_grid_points.add((gx, gy))

        self.grid_points = filtered_grid_points  # self.grid_points now holds only empty cells
        return occupied_cell_indices  # Return for use in process_navigation_points

    def get_grid_points(self):
        """
        Returns a set of (x, y) tuples representing the filtered grid points (empty cells).
        """
        return self.grid_points

    def _get_robot_current_pose_in_target_frame(self):
        """
        Looks up the robot's current pose (x, y) in the target_frame.
        Returns (x, y) tuple or None if transform is unavailable.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.robot_base_frame,
                rclpy.time.Time()  # Get the latest available transform
            )
            # Assuming 2D navigation, we only care about x, y
            return (round(transform.transform.translation.x, 3),
                    round(transform.transform.translation.y, 3))
        except TransformException as ex:
            self.get_logger().warn(f"Could not get transform from {self.robot_base_frame} to {self.target_frame}: {ex}")
            return None

    def _goal_response_callback(self, future: Future):
        goal_handle = future.result()
        if not goal_handle:
            self.get_logger().error('Goal was rejected by action server!')
            self.is_navigating = False
            self.current_goal_point = None
            return

        self.get_logger().info('Goal accepted by action server.')
        self.goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future: Future):
        result = future.result().result
        status = future.result().status

        if status == rclpy.action.GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Navigation succeeded! Result: {result}')
        else:
            self.get_logger().warn(f'Navigation failed with status: {status}')

        self.is_navigating = False
        self.current_goal_point = None
        self.goal_handle = None

    def navigate_to(self, goal_point_xy):
        """
        Sends a navigation goal to the Nav2 stack.
        """
        self.get_logger().info(f"Sending navigation goal to Nav2: {goal_point_xy}")

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available!')
            self.is_navigating = False
            self.current_goal_point = None
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.target_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_point_xy[0]
        goal_msg.pose.pose.position.y = goal_point_xy[1]
        goal_msg.pose.pose.position.z = 0.0

        # Default orientation (facing positive X)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.is_navigating = True
        self.current_goal_point = goal_point_xy
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self._goal_response_callback)

    def process_navigation_points(self, occupied_cell_indices):
        """
        Manages the selection and monitoring of the next navigation point.
        Checks for new obstacles at the target point and follows the planned path.
        """
        grid_size = self.GRID_RESOLUTION

        # Case 1: Currently navigating to a point
        if self.is_navigating:
            if self.goal_handle is None:
                # Goal was sent but handle not yet received, wait
                self.get_logger().info("Waiting for Nav2 goal response...")
                return

            status = self.goal_handle.get_status()
            if status == rclpy.action.GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f"Nav2 reports goal {self.current_goal_point} SUCCEEDED.")
                self.is_navigating = False
                self.current_goal_point = None
                self.goal_handle = None
            elif status in [rclpy.action.GoalStatus.STATUS_ABORTED, rclpy.action.GoalStatus.STATUS_CANCELED]:
                self.get_logger().warn(f"Nav2 reports goal {self.current_goal_point} {status}.")
                self.is_navigating = False
                self.current_goal_point = None
                self.goal_handle = None
            else:  # ACTIVE, PENDING, EXECUTING, etc.
                self.get_logger().info(f"Nav2 goal {self.current_goal_point} is {status}.")
                # Check if the current goal point itself has become occupied
                if self.current_goal_point:
                    goal_gx, goal_gy = self.current_goal_point
                    goal_cell_x_idx = math.floor(goal_gx / grid_size)
                    goal_cell_y_idx = math.floor(goal_gy / grid_size)
                    if (goal_cell_x_idx, goal_cell_y_idx) in occupied_cell_indices:
                        self.get_logger().warn(
                            f"Obstacle detected at current Nav2 goal {self.current_goal_point}. Cancelling Nav2 goal.")
                        # Cancel the Nav2 goal
                        if self.goal_handle:
                            cancel_future = self.goal_handle.cancel_goal_async()
                            cancel_future.add_done_callback(
                                lambda future: self.get_logger().info("Nav2 goal cancellation requested."))
                        self.is_navigating = False
                        self.current_goal_point = None
                        self.goal_handle = None
                return  # Don't try to find a new point if already navigating

        # Case 2: Not currently navigating, need to find a new goal
        if not self.grid_points:
            self.current_goal_point = None
            self.is_navigating = False
            self.get_logger().info("No navigable grid points available to choose from for a new goal.")
            return

        # Pick a new goal point (e.g., the first available empty cell)
        # In a real application, you'd have a more sophisticated goal selection strategy
        sorted_navigable_points = sorted(list(self.grid_points))
        if not sorted_navigable_points:
            self.get_logger().info("No navigable grid points available after sorting for a new goal.")
            self.current_goal_point = None
            self.is_navigating = False
            return

        new_goal_candidate = sorted_navigable_points[0]  # Pick the first one as a new goal

        # Final check before sending goal to Nav2
        candidate_gx, candidate_gy = new_goal_candidate
        candidate_cell_x_idx = math.floor(candidate_gx / grid_size)
        candidate_cell_y_idx = math.floor(candidate_gy / grid_size)

        if (candidate_cell_x_idx, candidate_cell_y_idx) in occupied_cell_indices:
            self.get_logger().warn(
                f"Candidate goal point {new_goal_candidate} is unexpectedly occupied. Skipping this point.")
            # Remove this point from grid_points so we don't try it again immediately
            self.grid_points.discard(new_goal_candidate)
            self.is_navigating = False  # Ensure we try to find another goal next cycle
            self.current_goal_point = None
        else:
            self.navigate_to(new_goal_candidate)

    def _start_monitoring_and_navigation(self):
        """
        Activates the timer to start grid generation and navigation.
        Called after user input.
        """
        self.monitoring_active = True
        self.timer = self.create_timer(5.0, self.monitor_and_grid_update)
        self.get_logger().info("Monitoring and navigation activated!")

    def monitor_and_grid_update(self):
        """
        Prints the current overall min/max coordinates, unique point count,
        regenerates/reports on the grid points, and processes navigation points.
        This function only runs if self.monitoring_active is True.
        """
        if not self.monitoring_active:
            return  # Do nothing until activated by user input

        min_x, max_x, min_y, max_y = self.get_overall_min_max_coordinates()
        num_unique_lidar_points = len(self.unique_lidar_points)

        if math.isinf(min_x):  # Check if any valid points have been processed
            self.get_logger().info('No valid lidar points processed yet. Grid not generated.')
            self.initial_grid_points_all.clear()
            self.grid_points.clear()  # Clear grid if no points
            self.current_goal_point = None
            self.is_navigating = False
            # Also cancel any pending Nav2 goals if no map data
            if self.goal_handle:
                cancel_future = self.goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(
                    lambda future: self.get_logger().info("Nav2 goal cancellation requested due to no map data."))
                self.goal_handle = None
        else:
            self.get_logger().info(
                f'Monitor Status (in {self.target_frame} frame): '
                f'Unique Lidar Points Count: {num_unique_lidar_points}, '
                f'X: [{min_x:.2f}, {max_x:.2f}], Y: [{min_y:.2f}, {max_y:.2f}]'
            )

            self.generate_grid_points()  # This now populates self.initial_grid_points_all
            self.get_logger().info(f'Initial Grid Points Count (before filtering): {len(self.initial_grid_points_all)}')

            # Filter empty grid points and get occupied cell indices
            occupied_cell_indices = self.filter_empty_grid_points()  # This populates self.grid_points with empty cells
            self.get_logger().info(
                f'Filtered Navigable Grid Points Count ({self.GRID_RESOLUTION}m resolution): {len(self.grid_points)}')

            self.process_navigation_points(occupied_cell_indices)  # Pass occupied cells for dynamic checks


class LidarVisualizer:
    SCREEN_WIDTH = 800
    SCREEN_HEIGHT = 800
    SCALE = 20  # Pixels per meter (e.g., 20 pixels for 1 meter)

    # Colors
    BLACK = (0, 0, 0)
    WHITE = (255, 255, 255)
    GREEN = (0, 255, 0)
    RED = (255, 0, 0)
    BLUE = (0, 0, 255)

    def __init__(self, node: LidarPointMonitor):
        self.node = node
        pygame.init()
        self.screen = pygame.display.set_mode((self.SCREEN_WIDTH, self.SCREEN_HEIGHT))
        pygame.display.set_caption("Lidar Map Visualizer")
        self.clock = pygame.time.Clock()

        # Calculate offset to center ROS (0,0) and flip Y-axis
        self.offset_x = self.SCREEN_WIDTH // 2
        self.offset_y = self.SCREEN_HEIGHT // 2

    def _ros_to_pygame(self, x_ros, y_ros):
        """Converts ROS (x,y) meters to Pygame (pixel_x, pixel_y) coordinates."""
        pixel_x = int(self.offset_x + x_ros * self.SCALE)
        pixel_y = int(self.offset_y - y_ros * self.SCALE)  # Subtract to flip Y-axis
        return pixel_x, pixel_y

    def draw_points(self, surface, points_set, color, radius=1):
        """Draws a set of (x,y) points onto the Pygame surface."""
        # Take a copy to avoid issues if the set is modified by the ROS thread during iteration
        points_copy = list(points_set)
        for x, y in points_copy:
            pixel_x, pixel_y = self._ros_to_pygame(x, y)
            if 0 <= pixel_x < self.SCREEN_WIDTH and 0 <= pixel_y < self.SCREEN_HEIGHT:
                pygame.draw.circle(surface, color, (pixel_x, pixel_y), radius)

    def run(self):
        """Main Pygame loop."""
        self.node.get_logger().info(
            "Pygame visualizer started. Press Enter in the Pygame window to activate navigation.")
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_RETURN:
                        if not self.node.monitoring_active:
                            self.node._start_monitoring_and_navigation()
                            self.node.get_logger().info(
                                "Enter pressed in Pygame window. Activating ROS node monitoring.")
                        else:
                            self.node.get_logger().info("Monitoring already active.")

            self.screen.fill(self.WHITE)  # Clear screen

            # Draw lidar points (black)
            self.draw_points(self.screen, self.node.unique_lidar_points, self.BLACK, radius=1)

            # Draw grid points (green) if monitoring is active
            if self.node.monitoring_active:
                self.draw_points(self.screen, self.node.grid_points, self.GREEN, radius=2)

                # Optionally draw current goal point (red)
                if self.node.current_goal_point:
                    gx, gy = self.node.current_goal_point
                    pixel_x, pixel_y = self._ros_to_pygame(gx, gy)
                    pygame.draw.circle(self.screen, self.RED, (pixel_x, pixel_y), 5)

                # Optionally draw robot's current position (blue)
                robot_pose = self.node._get_robot_current_pose_in_target_frame()
                if robot_pose:
                    rx, ry = robot_pose
                    pixel_x, pixel_y = self._ros_to_pygame(rx, ry)
                    pygame.draw.circle(self.screen, self.BLUE, (pixel_x, pixel_y), 7)

            pygame.display.flip()  # Update the full display Surface to the screen
            self.clock.tick(60)  # Limit to 60 FPS

        pygame.quit()
        sys.exit()  # Ensure Pygame thread exits cleanly


def main(args=None):
    rclpy.init(args=args)

    lidar_monitor = LidarPointMonitor()

    # Start Pygame visualizer in a separate thread
    visualizer = LidarVisualizer(lidar_monitor)
    visualizer_thread = threading.Thread(target=visualizer.run)
    visualizer_thread.start()

    try:
        rclpy.spin(lidar_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure the node is destroyed and ROS 2 shutdown
        lidar_monitor.get_logger().info("Shutting down LidarPointMonitor node.")
        lidar_monitor.destroy_node()
        rclpy.shutdown()
        # The visualizer thread will exit when its loop finishes (e.g., window closed)
        # We don't need to explicitly join it here as sys.exit() will be called from within it.


if __name__ == '__main__':
    main()