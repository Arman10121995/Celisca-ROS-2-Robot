import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped

import math
import tf2_ros
from tf2_ros import TransformException
from collections import deque  # For efficient queue operations in BFS


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

        # Timer to periodically print status and regenerate grid
        self.timer = self.create_timer(5.0, self.monitor_and_grid_update)  # Combined function

        # Navigation state variables
        self.next_point = None  # The immediate next step in the path
        self.is_navigating = False  # True if we are currently following a path
        self.current_goal_point = None  # The final goal point we are trying to reach
        self.current_path = deque()  # Stores the sequence of grid points to follow

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
                        timeout=rclpy.duration.Duration(seconds=0.1)  # Small timeout
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
                    # Log the exception, but don't stop processing
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

    def _find_path_bfs(self, start_point_xy, goal_point_xy, navigable_grid_points_xy, occupied_cell_indices):
        """
        Finds the shortest path from start_point_xy to goal_point_xy using BFS
        through navigable_grid_points_xy, avoiding occupied_cell_indices.
        Returns a deque of (x,y) points representing the path, or None if no path.
        """
        grid_size = self.GRID_RESOLUTION

        # Convert start and goal points to their respective grid cell indices
        start_cell_idx = (math.floor(start_point_xy[0] / grid_size), math.floor(start_point_xy[1] / grid_size))
        goal_cell_idx = (math.floor(goal_point_xy[0] / grid_size), math.floor(goal_point_xy[1] / grid_size))

        # Check if start or goal are in occupied cells
        if start_cell_idx in occupied_cell_indices:
            self.get_logger().warn(
                f"Start point {start_point_xy} is in an occupied cell {start_cell_idx}. Cannot plan path.")
            return None
        if goal_cell_idx in occupied_cell_indices:
            self.get_logger().warn(
                f"Goal point {goal_point_xy} is in an occupied cell {goal_cell_idx}. Cannot plan path.")
            return None

        # Convert navigable_grid_points_xy to cell indices for BFS
        navigable_cell_indices = set()
        for gx, gy in navigable_grid_points_xy:
            navigable_cell_indices.add((math.floor(gx / grid_size), math.floor(gy / grid_size)))

        if start_cell_idx not in navigable_cell_indices and start_cell_idx not in occupied_cell_indices:
            self.get_logger().warn(
                f"Start cell {start_cell_idx} is not explicitly marked as navigable or occupied. Assuming navigable for path planning.")
            # For BFS, we can temporarily add it if it's not occupied
            navigable_cell_indices.add(start_cell_idx)

        if goal_cell_idx not in navigable_cell_indices:
            self.get_logger().warn(
                f"Goal cell {goal_cell_idx} is not explicitly marked as navigable. Cannot plan path.")
            return None

        queue = deque([(start_cell_idx)])
        visited = {start_cell_idx}
        parent = {start_cell_idx: None}  # To reconstruct path

        # 8-directional movement
        # dx = [-1, -1, -1, 0, 0, 1, 1, 1]
        # dy = [-1, 0, 1, -1, 1, -1, 0, 1]
        # 4-directional movement (often preferred for grid pathfinding to avoid diagonal "cutting corners")
        dx = [0, 0, 1, -1]
        dy = [1, -1, 0, 0]

        while queue:
            current_cell = queue.popleft()

            if current_cell == goal_cell_idx:
                # Path found, reconstruct it
                path = deque()
                while current_cell != start_cell_idx:
                    # Convert cell index back to grid point (bottom-left corner)
                    path.appendleft((round(current_cell[0] * grid_size, 3), round(current_cell[1] * grid_size, 3)))
                    current_cell = parent[current_cell]
                return path

            for i in range(len(dx)):
                neighbor_cell = (current_cell[0] + dx[i], current_cell[1] + dy[i])

                if neighbor_cell not in visited and \
                        neighbor_cell not in occupied_cell_indices and \
                        neighbor_cell in navigable_cell_indices:  # Ensure neighbor is navigable
                    visited.add(neighbor_cell)
                    parent[neighbor_cell] = current_cell
                    queue.append(neighbor_cell)

        self.get_logger().warn(f"No path found from {start_point_xy} to {goal_point_xy}.")
        return None  # No path found

    def navigate_to(self, goal_point_xy):
        """
        Initiates navigation to a specific goal point (x, y) in the target_frame.
        This function calculates a path and sets up the navigation state.
        """
        robot_current_pose_xy = self._get_robot_current_pose_in_target_frame()
        if robot_current_pose_xy is None:
            self.get_logger().error("Cannot navigate: Robot's current pose is unknown.")
            self.is_navigating = False
            self.current_goal_point = None
            self.current_path.clear()
            self.next_point = None
            return

        self.get_logger().info(f"Attempting to plan path from {robot_current_pose_xy} to goal {goal_point_xy}")

        # Find a path using BFS
        path = self._find_path_bfs(
            robot_current_pose_xy,
            goal_point_xy,
            self.grid_points,  # These are the empty cells
            self._get_occupied_cell_indices()  # Current occupied cells
        )

        if path:
            self.current_path = path
            self.current_goal_point = goal_point_xy
            self.is_navigating = True
            self.next_point = self.current_path.popleft() if self.current_path else self.current_goal_point
            self.get_logger().info(
                f"Path found to {goal_point_xy}. First step: {self.next_point}. Path length: {len(self.current_path) + 1} steps.")
            # Here you would send the first navigation command to the robot
            # For dry run, we just log it.
            # self.send_robot_command(self.next_point)
        else:
            self.get_logger().warn(f"Failed to find a path to {goal_point_xy}. Cannot navigate.")
            self.is_navigating = False
            self.current_goal_point = None
            self.current_path.clear()
            self.next_point = None

    def process_navigation_points(self, occupied_cell_indices):
        """
        Manages the selection and monitoring of the next navigation point.
        Checks for new obstacles at the target point and follows the planned path.
        """
        grid_size = self.GRID_RESOLUTION

        # Case 1: Currently navigating and following a path
        if self.is_navigating and self.current_goal_point is not None:
            if not self.current_path:  # Path is empty, means we reached the goal
                self.get_logger().info(f"Reached goal point: {self.current_goal_point}")
                self.is_navigating = False
                self.current_goal_point = None
                self.next_point = None
                return  # Goal reached, stop processing navigation for this cycle

            # Check if the *next step* in the path has become occupied
            next_step_gx, next_step_gy = self.next_point
            next_step_cell_x_idx = math.floor(next_step_gx / grid_size)
            next_step_cell_y_idx = math.floor(next_step_gy / grid_size)

            if (next_step_cell_x_idx, next_step_cell_y_idx) in occupied_cell_indices:
                self.get_logger().warn(
                    f"Obstacle detected at next step {self.next_point} on path to {self.current_goal_point}. Replanning needed.")
                self.is_navigating = False  # Cancel current navigation
                self.current_path.clear()
                self.next_point = None
                # The next monitor_and_grid_update cycle will attempt to find a new goal/path
                return
            else:
                # Simulate moving to the next point
                self.get_logger().info(f"Simulating move to: {self.next_point} towards goal {self.current_goal_point}")
                # In a real robot, you'd send a command to move to self.next_point

                # After "moving", update next_point to the next step in the path
                self.next_point = self.current_path.popleft() if self.current_path else self.current_goal_point
                return  # Continue navigating on the next cycle

        # Case 2: Not currently navigating, need to find a new goal and path
        if not self.grid_points:
            self.next_point = None
            self.is_navigating = False
            self.current_goal_point = None
            self.current_path.clear()
            self.get_logger().info("No navigable grid points available to choose from for a new goal.")
            return

        # Pick a new goal point (e.g., the first available empty cell)
        # In a real application, you'd have a more sophisticated goal selection strategy
        sorted_navigable_points = sorted(list(self.grid_points))
        if not sorted_navigable_points:
            self.get_logger().info("No navigable grid points available after sorting for a new goal.")
            self.next_point = None
            self.is_navigating = False
            self.current_goal_point = None
            self.current_path.clear()
            return

        new_goal_candidate = sorted_navigable_points[0]  # Pick the first one as a new goal

        # Attempt to navigate to this new goal
        self.navigate_to(new_goal_candidate)

    def monitor_and_grid_update(self):
        """
        Prints the current overall min/max coordinates, unique point count,
        regenerates/reports on the grid points, and processes navigation points.
        """
        min_x, max_x, min_y, max_y = self.get_overall_min_max_coordinates()
        num_unique_lidar_points = len(self.unique_lidar_points)

        if math.isinf(min_x):  # Check if any valid points have been processed
            self.get_logger().info('No valid lidar points processed yet. Grid not generated.')
            self.initial_grid_points_all.clear()
            self.grid_points.clear()  # Clear grid if no points
            self.next_point = None
            self.is_navigating = False
            self.current_goal_point = None
            self.current_path.clear()
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


def main(args=None):
    rclpy.init(args=args)

    lidar_monitor = LidarPointMonitor()

    rclpy.spin(lidar_monitor)

    # Destroy the node explicitly
    lidar_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()