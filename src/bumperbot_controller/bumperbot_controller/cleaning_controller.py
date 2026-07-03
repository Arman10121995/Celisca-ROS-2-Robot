#!/usr/bin/env python3
"""
Room Cleaner Module

This module contains the cleaning-specific logic extracted from room_vacuum_controller.py.
It can be used as a standalone cleaner or integrated with a mapper.

Usage:
    from room_cleaner import RoomCleaner
    cleaner = RoomCleaner(map_data, config)
    cleaner.plan_cleaning()
    # ... use cleaner methods
"""

import math
import os
import sys
from pathlib import Path
from collections import deque
from typing import Dict, List, Set, Tuple, Optional
from dataclasses import dataclass, field
from enum import Enum
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# Pygame is optional (for visualization only)
try:
    import pygame
    HAS_PYGAME = True
except Exception:
    pygame = None
    HAS_PYGAME = False


def get_generated_map_path():
    """Get the absolute path to the generated_map directory.
    
    Returns the absolute path to WORKSPACE/src/maps/generated_map directory
    by navigating up from the current module's location.
    Handles both source and install directory paths.
    """
    # Get the directory containing this module
    module_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Check if we're in an install directory (contains 'install' in the path)
    if '/install/' in module_dir:
        # We're in the install directory. Navigate to the source workspace.
        # From: WORKSPACE/install/bumperbot_controller/local/lib/python3.10/dist-packages/bumperbot_controller/
        # To:   WORKSPACE/src/maps/generated_map/
        # Find the workspace root by going up until we find 'install' directory
        parts = module_dir.split('/install/')
        if len(parts) >= 2:
            workspace_root = parts[0]
            # Navigate to WORKSPACE/src/maps/generated_map
            generated_map_path = os.path.abspath(os.path.join(workspace_root, 'src', 'maps', 'generated_map'))
        else:
            # Fallback: use relative path from install location
            generated_map_path = os.path.abspath(os.path.join(module_dir, '..', '..', '..', '..', '..', 'src', 'maps', 'generated_map'))
    else:
        # We're in the source directory.
        # From: WORKSPACE/src/bumperbot_controller/bumperbot_controller/
        # To:   WORKSPACE/src/maps/generated_map/
        generated_map_path = os.path.abspath(os.path.join(module_dir, '..', '..', 'maps', 'generated_map'))
    
    return generated_map_path


class CleanerState(Enum):
    """States for the cleaning process."""
    CLEAN_PLAN = "CLEAN_PLAN"
    CLEAN_APPROACH_LANE = "CLEAN_APPROACH_LANE"
    CLEAN_SWEEP_LANE = "CLEAN_SWEEP_LANE"
    CLEAN_DONE = "CLEAN_DONE"


@dataclass
class CleanerConfig:
    """Configuration for the room cleaner."""
    cleaning_grid_spacing: float = 0.40
    map_resolution: float = 0.15
    robot_obstacle_clearance: float = 0.30
    cleaning_goal_dist: float = 0.25
    lane_width: float = 0.40
    
    # Derived settings
    cleaning_step_cells: int = 1
    rejected_goal_radius_cells: int = 1
    
    def __post_init__(self):
        self.cleaning_step_cells = max(1, int(round(self.cleaning_grid_spacing / self.map_resolution)))
        self.rejected_goal_radius_cells = max(1, int(round(0.20 / self.map_resolution)))


class RoomCleaner:
    """
    Handles the cleaning logic for a room vacuum robot.
    
    This class manages:
    - Cleaning goal generation
    - Cleaning lane planning
    - Cleaning path following
    - Tracking of cleaned areas
    """
    
    def __init__(self, config: CleanerConfig = None):
        """
        Initialize the room cleaner.
        
        Args:
            config: CleanerConfig instance with cleaning parameters
        """
        self.config = config or CleanerConfig()
        
        # Cleaning state
        self.state = CleanerState.CLEAN_PLAN
        self.cleaning_goals: List[Tuple[int, int]] = []
        self.current_clean_goal_index: int = 0
        self.current_cleaning_lane_cells: List[Tuple[int, int]] = []
        self.current_lane_start_cell: Optional[Tuple[int, int]] = None
        self.current_lane_endpoint_cell: Optional[Tuple[int, int]] = None
        self.current_goal_cell: Optional[Tuple[int, int]] = None
        self.cleaned_cells: Set[Tuple[int, int]] = set()
        self.temporarily_skipped_cleaning_cells: Set[Tuple[int, int]] = set()
        self.rejected_cleaning_cells: Set[Tuple[int, int]] = set()
        
        # Map data (to be provided by mapper)
        self.free_counts: Dict[Tuple[int, int], int] = {}
        self.occupied_counts: Dict[Tuple[int, int], int] = {}
        self.temporary_obstacle_counts: Dict[Tuple[int, int], int] = {}
        self.map_version: int = 0
        self.last_cleaning_goal_version: int = -1
        self.map_origin: Tuple[float, float] = (0.0, 0.0)
        
        # Callbacks for integration with mapper
        self.is_cell_occupied_callback = None
        self.is_cell_free_callback = None
        self.is_cell_blocked_callback = None
        self.plan_path_to_cell_callback = None
        self.reached_cell_callback = None
    
    def set_map_data(self, free_counts, occupied_counts, temporary_obstacle_counts=None):
        """Set the map data from the mapper."""
        self.free_counts = free_counts
        self.occupied_counts = occupied_counts
        if temporary_obstacle_counts:
            self.temporary_obstacle_counts = temporary_obstacle_counts
        self.map_version += 1
    
    def set_callbacks(self, is_occupied, is_free, is_blocked, plan_path, reached_cell):
        """Set callback functions for integration with mapper."""
        self.is_cell_occupied_callback = is_occupied
        self.is_cell_free_callback = is_free
        self.is_cell_blocked_callback = is_blocked
        self.plan_path_to_cell_callback = plan_path
        self.reached_cell_callback = reached_cell

    def load_map_from_pgm(self, pgm_path=None, yaml_path=None, map_dir=None):
        """Load map data from PGM and YAML files.
        
        Args:
            pgm_path: Path to PGM file (optional if map_dir is provided)
            yaml_path: Path to YAML file (optional if map_dir is provided)
            map_dir: Directory containing map.pgm and map.yaml files
            
        Returns:
            True if map was loaded successfully, False otherwise
        """
        try:
            # Determine file paths
            if map_dir is None:
                # Use default workspace generated_map directory
                map_dir = get_generated_map_path()
            
            if map_dir:
                pgm_path = pgm_path or os.path.join(map_dir, "map.pgm")
                yaml_path = yaml_path or os.path.join(map_dir, "map.yaml")
            
            if not pgm_path or not yaml_path:
                return False
            
            if not os.path.exists(pgm_path) or not os.path.exists(yaml_path):
                return False
            
            # Load YAML metadata to get resolution and origin
            yaml_data = {}
            with open(yaml_path, 'r') as f:
                for line in f:
                    line = line.strip()
                    if ':' in line and not line.startswith('#'):
                        key, value = line.split(':', 1)
                        yaml_data[key.strip()] = value.strip()
            
            resolution = float(yaml_data.get('resolution', 0.05))
            origin_str = yaml_data.get('origin', '[0.0, 0.0, 0.0]')
            origin_str = origin_str.strip('[]').replace(',', ' ')
            origin_parts = [float(x) for x in origin_str.split()]
            origin_x, origin_y = origin_parts[0], origin_parts[1]
            
            # Update config with loaded resolution
            self.config.map_resolution = resolution
            self.config.__post_init__()  # Recalculate derived settings
            
            # Store origin for coordinate conversion
            self.map_origin = (origin_x, origin_y)
            
            # Load PGM data
            free_counts = {}
            occupied_counts = {}
            
            with open(pgm_path, 'r') as f:
                lines = f.readlines()
                
            # Skip comments and find PGM header
            line_idx = 0
            while line_idx < len(lines) and lines[line_idx].startswith('#'):
                line_idx += 1
            
            if line_idx >= len(lines) or lines[line_idx].strip() != 'P2':
                return False
            
            line_idx += 1
            
            # Get width and height
            while line_idx < len(lines) and lines[line_idx].strip() == '':
                line_idx += 1
            
            if line_idx >= len(lines):
                return False
                
            width, height = map(int, lines[line_idx].strip().split())
            line_idx += 1
            
            # Get max value
            while line_idx < len(lines) and lines[line_idx].strip() == '':
                line_idx += 1
            
            if line_idx >= len(lines):
                return False
                
            max_val = int(lines[line_idx].strip())
            line_idx += 1
            
            # Parse pixel data
            row_idx = 0
            while line_idx < len(lines) and row_idx < height:
                line = lines[line_idx].strip()
                if not line:
                    line_idx += 1
                    continue
                
                pixel_values = line.split()
                
                for col_idx, pixel_str in enumerate(pixel_values):
                    if row_idx >= height:
                        break
                        
                    pixel_val = int(pixel_str)
                    
                    # Convert pixel coordinate to cell coordinate
                    # PGM: row 0 is top, but we want row 0 to be bottom
                    pgm_row = row_idx
                    pgm_col = col_idx
                    
                    # Flip Y axis: PGM origin is top-left, we want bottom-left
                    cell_y = height - 1 - pgm_row
                    cell_x = pgm_col
                    
                    if pixel_val == 0:  # Free space
                        free_counts[(cell_x, cell_y)] = 1
                    elif pixel_val == 100:  # Occupied
                        occupied_counts[(cell_x, cell_y)] = 1
                    # -1 or other values are unknown, we ignore them
                
                row_idx += 1
                line_idx += 1
            
            # Set the loaded map data
            self.set_map_data(free_counts, occupied_counts)
            return True
            
        except Exception as e:
            print(f"Error loading map: {e}")
            return False
    
    def is_cell_occupied(self, cell):
        """Check if a cell is occupied."""
        if self.is_cell_occupied_callback:
            return self.is_cell_occupied_callback(cell)
        return cell in self.occupied_counts and self.occupied_counts[cell] > 0
    
    def is_cell_free(self, cell):
        """Check if a cell is free."""
        if self.is_cell_free_callback:
            return self.is_cell_free_callback(cell)
        return cell in self.free_counts and self.free_counts[cell] > 0
    
    def is_cell_blocked(self, cell):
        """Check if a cell is blocked."""
        if self.is_cell_blocked_callback:
            return self.is_cell_blocked_callback(cell)
        return self.is_cell_occupied(cell)
    
    def plan_path_to_cell(self, goal):
        """Plan a path to a cell."""
        if self.plan_path_to_cell_callback:
            return self.plan_path_to_cell_callback(goal)
        return False
    
    def reached_cell(self, cell, tolerance):
        """Check if a cell has been reached."""
        if self.reached_cell_callback:
            return self.reached_cell_callback(cell, tolerance)
        return False
    
    def build_cleaning_goals(self):
        """
        Build a list of cleaning goals from free cells.
        
        Returns:
            List of (x, y) cell coordinates to clean
        """
        cleaning_goals = []
        added_cells = set()
        
        # Get all free cells
        free_cells = [cell for cell, count in self.free_counts.items() if count > 0]
        
        # Sort cells for systematic cleaning (left to right, bottom to top)
        free_cells.sort(key=lambda cell: (cell[1], cell[0]))
        
        # Add cells spaced by cleaning grid
        for cell in free_cells:
            cx, cy = cell
            # Check if this cell or nearby cells are already in goals
            nearby = False
            for gx, gy in cleaning_goals:
                if abs(cx - gx) < self.config.cleaning_step_cells and abs(cy - gy) < self.config.cleaning_step_cells:
                    nearby = True
                    break
            if not nearby:
                cleaning_goals.append(cell)
        
        return cleaning_goals
    
    def mark_history_cleaned(self):
        """Mark cells from history as cleaned (to be implemented with history)."""
        # This would be implemented with access to robot's history
        pass
    
    def is_cell_cleaned(self, cell):
        """Check if a cell has been cleaned."""
        return cell in self.cleaned_cells
    
    def mark_cleaned_area(self, center_cell, radius=1):
        """Mark an area around a cell as cleaned."""
        cx, cy = center_cell
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                cell = (cx + dx, cy + dy)
                if cell in self.cleaning_goals or cell in self.free_counts:
                    self.cleaned_cells.add(cell)
    
    def plan_cleaning(self):
        """Plan the cleaning process."""
        self.temporarily_skipped_cleaning_cells.clear()
        self.mark_history_cleaned()
        self.cleaning_goals = self.build_cleaning_goals()
        return self.plan_next_cleaning_lane("cleaning started")
    
    def refresh_cleaning_goals_if_needed(self, map_changed, reason):
        """Refresh cleaning goals if map has changed."""
        if not map_changed and self.last_cleaning_goal_version == self.map_version:
            return None
        return self.refresh_cleaning_goals(reason)
    
    def refresh_cleaning_goals(self, reason):
        """Refresh the list of cleaning goals."""
        old_remaining = set(self.cleaning_goals[self.current_clean_goal_index:])
        self.temporarily_skipped_cleaning_cells.clear()
        self.mark_history_cleaned()
        new_goals = self.build_cleaning_goals()
        new_goal_set = set(new_goals)
        added_count = len(new_goal_set - old_remaining)
        removed_count = len(old_remaining - new_goal_set)
        self.cleaning_goals = new_goals
        note = self.plan_next_cleaning_lane(reason)
        return f"cleaning_goals_refreshed_added_{added_count}_removed_{removed_count}; {note}"
    
    def plan_next_cleaning_lane(self, reason):
        """Plan the next cleaning lane."""
        self.clear_cleaning_lane_state()
        self.current_path = []
        self.current_path_direct = False
        self.current_clean_goal_index = 0
        self.last_cleaning_goal_version = self.map_version
        
        if not self.cleaning_goals:
            self.current_goal_cell = None
            self.state = CleanerState.CLEAN_DONE
            return "cleaning_no_goals"
        
        while self.current_clean_goal_index < len(self.cleaning_goals):
            start_cell = self.cleaning_goals[self.current_clean_goal_index]
            if (
                self.is_cell_cleaned(start_cell)
                or self.is_rejected_goal(start_cell)
                or start_cell in self.temporarily_skipped_cleaning_cells
            ):
                self.current_clean_goal_index += 1
                continue
            
            lane_cells = self.get_cleaning_lane_cells(self.current_clean_goal_index)
            if not lane_cells:
                self.current_clean_goal_index += 1
                continue
            
            self.current_cleaning_lane_cells = lane_cells
            self.current_lane_start_cell = lane_cells[0]
            self.current_lane_endpoint_cell = lane_cells[-1]
            self.current_goal_cell = self.current_lane_start_cell
            
            if self.current_cleaning_lane_start_reached():
                return self.start_cleaning_lane_sweep()
            
            if self.plan_path_to_cell(self.current_lane_start_cell):
                self.current_goal_cell = self.current_lane_start_cell
                self.state = CleanerState.CLEAN_APPROACH_LANE
                return f"cleaning_approach_lane_{self.current_lane_start_cell}_to_{self.current_lane_endpoint_cell}"
            
            self.skip_cleaning_lane_cells()
            self.current_clean_goal_index += len(lane_cells)
        
        self.clear_cleaning_lane_state()
        self.current_goal_cell = None
        self.current_path = []
        self.state = CleanerState.CLEAN_DONE
        return "cleaning_lanes_unreachable"
    
    def clear_cleaning_lane_state(self):
        """Clear the current cleaning lane state."""
        self.current_cleaning_lane_cells = []
        self.current_lane_start_cell = None
        self.current_lane_endpoint_cell = None
    
    def get_cleaning_lane_cells(self, start_index):
        """Get cells for a cleaning lane starting at index."""
        if start_index >= len(self.cleaning_goals):
            return []
        
        start_cell = self.cleaning_goals[start_index]
        lane_cells = [start_cell]
        max_gap_cells = max(self.config.cleaning_step_cells + 1, 
                          int(math.ceil(self.config.cleaning_step_cells * 1.5)))
        
        for cell in self.cleaning_goals[start_index + 1:]:
            previous = lane_cells[-1]
            if cell[1] != start_cell[1]:
                break
            if abs(cell[0] - previous[0]) > max_gap_cells:
                break
            if (
                self.is_cell_cleaned(cell)
                or self.is_rejected_goal(cell)
                or cell in self.temporarily_skipped_cleaning_cells
            ):
                break
            lane_cells.append(cell)
        
        return lane_cells
    
    def current_cleaning_lane_start_reached(self):
        """Check if the start of the current cleaning lane has been reached."""
        if self.current_lane_start_cell is None:
            return False
        return self.reached_cell(self.current_lane_start_cell, self.config.cleaning_goal_dist)
    
    def current_cleaning_lane_finished(self):
        """Check if the current cleaning lane is finished."""
        if self.current_lane_endpoint_cell is None:
            return False
        if self.reached_cell(self.current_lane_endpoint_cell, self.config.cleaning_goal_dist):
            return True
        return all(self.is_cell_cleaned(cell) for cell in self.current_cleaning_lane_cells)
    
    def start_cleaning_lane_sweep(self):
        """Start sweeping the current cleaning lane."""
        self.mark_cleaned_area(self.current_lane_start_cell)
        
        remaining_lane_cells = [
            cell for cell in self.current_cleaning_lane_cells
            if not self.is_cell_cleaned(cell) and not self.is_cell_blocked(cell)
        ]
        if not remaining_lane_cells:
            return self.finish_current_cleaning_lane()
        
        for endpoint in reversed(remaining_lane_cells):
            # For now, just mark the endpoint (path planning would be done by mapper)
            self.current_lane_endpoint_cell = endpoint
            self.current_goal_cell = endpoint
            self.state = CleanerState.CLEAN_SWEEP_LANE
            return f"cleaning_sweep_lane_to_{endpoint}"
        
        self.skip_cleaning_lane_cells()
        self.current_clean_goal_index += len(self.current_cleaning_lane_cells)
        return self.plan_next_cleaning_lane("skipped cleaning lane without clear straight sweep")
    
    def finish_current_cleaning_lane(self):
        """Finish the current cleaning lane."""
        # Mark current position as cleaned (if we have a position callback)
        self.current_clean_goal_index += len(self.current_cleaning_lane_cells)
        self.cleaning_goals = self.build_cleaning_goals()
        return self.plan_next_cleaning_lane("finished cleaning lane")
    
    def skip_cleaning_lane_cells(self):
        """Skip the cells in the current cleaning lane."""
        for cell in self.current_cleaning_lane_cells:
            self.temporarily_skipped_cleaning_cells.add(cell)
    
    def is_rejected_goal(self, cell):
        """Check if a cell is in the rejected goals set."""
        return cell in self.rejected_cleaning_cells
    
    def reject_goal_cell(self, goal, reason):
        """Reject a goal cell and add nearby cells to rejected set."""
        radius = self.config.rejected_goal_radius_cells
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if dx * dx + dy * dy > radius * radius:
                    continue
                self.rejected_cleaning_cells.add((goal[0] + dx, goal[1] + dy))
    
    def remaining_cleaning_goals(self):
        """Count remaining cleaning goals."""
        return sum(1 for cell in self.cleaning_goals[self.current_clean_goal_index:] 
                   if not self.is_cell_cleaned(cell))


class CleaningController(Node):
    """ROS2 node for standalone cleaning controller."""
    
    def __init__(self):
        super().__init__('cleaning_controller')
        
        # Configuration
        self.cmd_vel_topic = '/key_vel'
        self.scan_topic = '/scan'
        self.odom_topic = '/bumperbot_controller/odom'
        self.map_resolution = 0.15
        
        # Movement parameters
        self.forward_speed = 0.2
        self.rotate_speed = 0.5
        self.goal_tolerance = 0.2  # meters
        self.angle_tolerance = 0.1  # radians
        
        # Setup ROS2 interfaces
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        
        # Initialize cleaner
        config = CleanerConfig(
            cleaning_grid_spacing=0.40,
            map_resolution=self.map_resolution,
            robot_obstacle_clearance=0.30
        )
        self.cleaner = RoomCleaner(config)
        
        # State variables - must be initialized before callbacks are used
        self.current_pose = None
        self.current_goal_cell = None
        self.current_path = []
        self.target_position = None
        self.target_yaw = None
        
        # Try to load map from generated_map directory
        generated_map_dir = get_generated_map_path()
        self.get_logger().info(f"Looking for map in: {generated_map_dir}")
        
        if self.cleaner.load_map_from_pgm(map_dir=generated_map_dir):
            self.get_logger().info("Successfully loaded map from generated_map directory")
            # Set up callbacks for the cleaner
            self.cleaner.set_callbacks(
                is_occupied=self.is_cell_occupied,
                is_free=self.is_cell_free,
                is_blocked=self.is_cell_blocked,
                plan_path=self.cleaner_plan_path_callback,
                reached_cell=self.cleaner_reached_cell_callback
            )
            # Plan cleaning
            self.cleaner.plan_cleaning()
            self.get_logger().info(f"Cleaning planned with {len(self.cleaner.cleaning_goals)} goals")
        else:
            self.get_logger().warn(f"No map found in {generated_map_dir}, using empty map")
        
        # Pygame Visualization (optional for headless operation)
        self.enable_viz = HAS_PYGAME
        self.first_pose_received = False
        if HAS_PYGAME:
            pygame.init()
            self.screen_size = 600
            self.screen = pygame.display.set_mode((self.screen_size, self.screen_size))
            pygame.display.set_caption("Bumperbot Cleaning Monitor")
            self.status_font = self.load_blocky_font(18)
            self.default_scale = 40.0
            self.scale = self.default_scale
            self.min_scale = 8.0
            self.max_scale = 220.0
            # Initialize view center to map origin, will be updated to robot position on first odom
            self.view_center_x = self.cleaner.map_origin[0]
            self.view_center_y = self.cleaner.map_origin[1]
            self.view_locked = False
            self.show_current_scan = True
            self.view_menu_open = False
            self.dragging_view = False
            self.drag_start_mouse = None
            self.drag_start_center = None
            self.viz_timer = self.create_timer(0.1, self.pygame_loop)
        
        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # Start with first cleaning goal if available
        if self.cleaner.cleaning_goals:
            self.current_goal_cell = self.cleaner.cleaning_goals[0]
            self.update_target_from_cell()
    
    def scan_callback(self, msg):
        """Handle laser scan messages."""
        pass  # Not needed for basic cleaning
    
    def odom_callback(self, msg):
        """Handle odometry messages."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        _, _, yaw = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.current_pose = (x, y, yaw)
        # On first pose, center the view on the robot
        if self.enable_viz and not self.first_pose_received:
            self.view_center_x = x
            self.view_center_y = y
            self.first_pose_received = True
    
    def update_target_from_cell(self):
        """Update target position from current goal cell."""
        if self.current_goal_cell is None:
            self.target_position = None
            self.target_yaw = None
            return
        
        # Convert cell coordinates to world coordinates
        # Assuming cell coordinates are in map grid units
        cell_x, cell_y = self.current_goal_cell
        self.target_position = (cell_x * self.map_resolution, cell_y * self.map_resolution)
        self.target_yaw = None  # For now, don't specify target yaw
    
    def control_loop(self):
        """Main control loop."""
        if self.current_pose is None or self.target_position is None:
            # Publish stop command if we don't have pose or target
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            return
        
        # Check if we've reached the current target
        if self.reached_target():
            self.get_logger().info(f"Reached target at {self.target_position}")
            # Mark this cell as cleaned
            if self.current_goal_cell:
                self.cleaner.mark_cleaned_area(self.current_goal_cell, radius=1)
            
            # Move to next cleaning goal
            self.move_to_next_goal()
            return
        
        # Calculate movement commands to reach target
        twist = self.calculate_movement_to_target()
        self.cmd_vel_pub.publish(twist)
    
    def move_to_next_goal(self):
        """Move to the next cleaning goal."""
        if not self.cleaner.cleaning_goals:
            self.get_logger().info("All cleaning goals completed")
            self.current_goal_cell = None
            self.target_position = None
            return
        
        # Find next uncleaned goal
        for i, goal in enumerate(self.cleaner.cleaning_goals):
            if not self.cleaner.is_cell_cleaned(goal):
                self.current_goal_cell = goal
                self.update_target_from_cell()
                self.get_logger().info(f"Moving to next goal: {self.current_goal_cell}")
                return
        
        # All goals are cleaned
        self.get_logger().info("All cleaning goals are cleaned")
        self.current_goal_cell = None
        self.target_position = None
    
    def reached_target(self):
        """Check if we've reached the current target position."""
        if self.current_pose is None or self.target_position is None:
            return False
        
        # Check position tolerance
        dx = self.current_pose[0] - self.target_position[0]
        dy = self.current_pose[1] - self.target_position[1]
        distance = math.sqrt(dx*dx + dy*dy)
        
        return distance <= self.goal_tolerance
    
    def calculate_movement_to_target(self):
        """Calculate movement commands to reach the target position."""
        twist = Twist()
        
        if self.current_pose is None or self.target_position is None:
            return twist
        
        x, y, yaw = self.current_pose
        target_x, target_y = self.target_position
        
        # Calculate distance and angle to target
        dx = target_x - x
        dy = target_y - y
        distance = math.sqrt(dx*dx + dy*dy)
        target_angle = math.atan2(dy, dx)
        
        # Calculate angle difference
        angle_diff = self.normalize_angle(target_angle - yaw)
        
        if distance < self.goal_tolerance:
            # We're close enough, just stop
            return twist
        
        if abs(angle_diff) > self.angle_tolerance:
            # Need to rotate to face target
            twist.angular.z = self.rotate_speed if angle_diff > 0 else -self.rotate_speed
        else:
            # Face the right direction, move forward
            twist.linear.x = self.forward_speed
        
        return twist
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi] range."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def is_cell_occupied(self, cell):
        """Check if a cell is occupied."""
        return cell in self.cleaner.occupied_counts and self.cleaner.occupied_counts[cell] > 0
    
    def is_cell_free(self, cell):
        """Check if a cell is free."""
        return cell in self.cleaner.free_counts and self.cleaner.free_counts[cell] > 0
    
    def is_cell_blocked(self, cell):
        """Check if a cell is blocked."""
        return self.is_cell_occupied(cell)
    
    def plan_path_to_cell(self, target_cell):
        """Plan a path to a target cell."""
        # For now, just return a direct path
        return [target_cell]
    
    def cleaner_reached_cell_callback(self, cell, tolerance):
        """Callback for RoomCleaner to check if a cell has been reached."""
        return self.reached_cell(cell, tolerance)
    
    def cleaner_plan_path_callback(self, goal):
        """Callback for RoomCleaner to plan a path to a goal."""
        return self.plan_path_to_cell(goal)
    
    def reached_cell(self, cell, tolerance=None):
        """Check if we've reached a cell."""
        if self.current_pose is None:
            return False
        
        # Use tolerance if provided, otherwise use default
        tolerance = tolerance or self.goal_tolerance
        
        # Convert cell to world coordinates
        cell_x, cell_y = cell
        cell_world_x = cell_x * self.map_resolution
        cell_world_y = cell_y * self.map_resolution
        
        # Check distance to cell center
        dx = self.current_pose[0] - cell_world_x
        dy = self.current_pose[1] - cell_world_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        return distance <= tolerance
    
    def euler_from_quaternion(self, quaternion):
        """Convert quaternion to Euler angles."""
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return 0, 0, math.atan2(siny_cosp, cosy_cosp)

    # GUI Helper Methods
    def load_blocky_font(self, size):
        for font_name in ("dejavusansmono", "liberationmono", "consolas", "couriernew", "monospace"):
            font_path = pygame.font.match_font(font_name, bold=True)
            if font_path:
                return pygame.font.Font(font_path, size)
        return pygame.font.SysFont("monospace", size, bold=True)

    def cell_to_world(self, cell):
        map_x = cell[0] * self.map_resolution + self.cleaner.map_origin[0]
        map_y = cell[1] * self.map_resolution + self.cleaner.map_origin[1]
        return map_x, map_y

    def world_to_screen(self, x, y):
        view_center_x, view_center_y = self.get_view_center()
        screen_x = int(self.screen_size / 2 + (x - view_center_x) * self.scale)
        screen_y = int(self.screen_size / 2 - (y - view_center_y) * self.scale)
        return screen_x, screen_y

    def get_view_center(self):
        if self.view_locked and self.current_pose:
            return self.current_pose[0], self.current_pose[1]
        return self.view_center_x, self.view_center_y

    def handle_pygame_event(self, event):
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

        if event.type == pygame.KEYDOWN:
            if event.key in (pygame.K_l, pygame.K_f):
                self.toggle_view_lock()

        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:  # Left click
                self.dragging_view = True
                self.drag_start_mouse = event.pos
                self.drag_start_center = (self.view_center_x, self.view_center_y)

        if event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1:  # Left click release
                self.dragging_view = False
                self.drag_start_mouse = None
                self.drag_start_center = None

        if event.type == pygame.MOUSEMOTION and self.dragging_view:
            mouse_dx = event.pos[0] - self.drag_start_mouse[0]
            mouse_dy = event.pos[1] - self.drag_start_mouse[1]
            center_x = self.drag_start_center[0] - mouse_dx / self.scale
            center_y = self.drag_start_center[1] + mouse_dy / self.scale
            self.view_center_x = center_x
            self.view_center_y = center_y

        if event.type == pygame.MOUSEWHEEL:
            old_scale = self.scale
            if event.y > 0:
                self.scale = min(self.scale * 1.1, self.max_scale)
            elif event.y < 0:
                self.scale = max(self.scale * 0.9, self.min_scale)

            if self.scale != old_scale:
                # Get current mouse position
                mouse_x, mouse_y = pygame.mouse.get_pos()
                view_x, view_y = self.get_view_center()
                world_x, world_y = self.screen_to_world(mouse_x, mouse_y)
                self.view_center_x = world_x + (view_x - world_x) * (self.scale / old_scale)
                self.view_center_y = world_y + (view_y - world_y) * (self.scale / old_scale)

    def toggle_view_lock(self):
        self.view_locked = not self.view_locked

    def screen_to_world(self, screen_x, screen_y):
        view_center_x, view_center_y = self.get_view_center()
        world_x = view_center_x + (screen_x - self.screen_size / 2) / self.scale
        world_y = view_center_y - (screen_y - self.screen_size / 2) / self.scale
        return world_x, world_y

    def draw_map_point(self, cell, color):
        sx, sy = self.world_to_screen(*self.cell_to_world(cell))
        rect = pygame.Rect(sx - 2, sy - 2, 4, 4)
        if rect.right < 0 or rect.left >= self.screen_size or rect.bottom < 0 or rect.top >= self.screen_size:
            return
        pygame.draw.rect(self.screen, color, rect)

    def draw_status_row(self, y, fields):
        x = 10
        label_color = (255, 145, 45)
        value_color = (255, 255, 255)
        for label, value in fields:
            label_surface = self.status_font.render(f"{label} ", True, label_color)
            value_surface = self.status_font.render(str(value), True, value_color)
            self.screen.blit(label_surface, (x, y))
            self.screen.blit(value_surface, (x + label_surface.get_width(), y))
            x += label_surface.get_width() + value_surface.get_width() + 10

    def pygame_loop(self):
        if not HAS_PYGAME:
            return

        for event in pygame.event.get():
            self.handle_pygame_event(event)

        self.screen.fill((30, 30, 30))  # Dark background

        # Draw occupied cells (walls)
        for cell in self.cleaner.occupied_counts:
            if self.cleaner.occupied_counts[cell] > 0:
                sx, sy = self.world_to_screen(*self.cell_to_world(cell))
                if 0 <= sx < self.screen_size and 0 <= sy < self.screen_size:
                    pygame.draw.rect(self.screen, (0, 100, 255), (sx - 2, sy - 2, 4, 4))

        # Draw free cells
        for cell in self.cleaner.free_counts:
            if self.cleaner.free_counts[cell] > 0:
                sx, sy = self.world_to_screen(*self.cell_to_world(cell))
                if 0 <= sx < self.screen_size and 0 <= sy < self.screen_size:
                    pygame.draw.rect(self.screen, (45, 45, 45), (sx - 1, sy - 1, 2, 2))

        # Draw cleaning goals as yellow points
        for cell in self.cleaner.cleaning_goals[self.cleaner.current_clean_goal_index:]:
            if self.cleaner.is_cell_cleaned(cell):
                continue
            sx, sy = self.world_to_screen(*self.cell_to_world(cell))
            if 0 <= sx < self.screen_size and 0 <= sy < self.screen_size:
                pygame.draw.circle(self.screen, (255, 255, 0), (sx, sy), 3)  # Yellow cleaning points

        # Draw cleaned cells
        for cell in self.cleaner.cleaned_cells:
            sx, sy = self.world_to_screen(*self.cell_to_world(cell))
            if 0 <= sx < self.screen_size and 0 <= sy < self.screen_size:
                pygame.draw.circle(self.screen, (0, 255, 0), (sx, sy), 2)  # Green for cleaned

        # Draw current path
        for cell in self.current_path:
            self.draw_map_point(cell, (255, 160, 0))

        # Draw current goal
        if self.current_goal_cell:
            sx, sy = self.world_to_screen(*self.cell_to_world(self.current_goal_cell))
            if 0 <= sx < self.screen_size and 0 <= sy < self.screen_size:
                pygame.draw.circle(self.screen, (255, 255, 255), (sx, sy), 5)  # White circle for current goal
                pygame.draw.circle(self.screen, (255, 120, 0), (sx, sy), 8, 2)  # Orange ring

        # Draw Robot
        if self.current_pose:
            rx, ry, ryaw = self.current_pose
            sx, sy = self.world_to_screen(rx, ry)
            pygame.draw.circle(self.screen, (255, 255, 255), (sx, sy), 10)
            # Heading line
            hx = sx + math.cos(ryaw) * 20
            hy = sy - math.sin(ryaw) * 20
            pygame.draw.line(self.screen, (255, 0, 0), (sx, sy), (hx, hy), 3)

        # Status Text
        self.draw_status_row(10, [("State:", self.cleaner.state)])
        self.draw_status_row(30, [("Goals:", len(self.cleaner.cleaning_goals))])
        self.draw_status_row(
            50,
            [
                ("Cleaned:", len(self.cleaner.cleaned_cells)),
                ("Remaining:", self.cleaner.remaining_cleaning_goals()),
            ],
        )

        pygame.display.flip()


if __name__ == '__main__':
    rclpy.init()
    try:
        controller = CleaningController()
        controller.get_logger().info("Cleaning Controller started")
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
