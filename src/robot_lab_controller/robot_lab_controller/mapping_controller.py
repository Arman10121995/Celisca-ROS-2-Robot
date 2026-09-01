#!/usr/bin/env python3
from collections import deque
import csv
from datetime import datetime
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from pathlib import Path
import sys
import os

# Pygame is optional (for visualization only)
try:
    import pygame
    HAS_PYGAME = True
except Exception:
    pygame = None
    HAS_PYGAME = False

# Import map saver module (from same directory)
import map_saver


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
        # From: WORKSPACE/install/robot_lab_controller/local/lib/python3.10/dist-packages/robot_lab_controller/
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
        # From: WORKSPACE/src/robot_lab_controller/robot_lab_controller/
        # To:   WORKSPACE/src/maps/generated_map/
        generated_map_path = os.path.abspath(os.path.join(module_dir, '..', '..', 'maps', 'generated_map'))
    
    return generated_map_path

CLEANING_GRID_SPACING_M = 0.40
ROBOT_OBSTACLE_CLEARANCE_M = 0.30
FRONT_OBSTACLE_STOP_DISTANCE_M = ROBOT_OBSTACLE_CLEARANCE_M + 0.35
SIDE_OBSTACLE_STOP_DISTANCE_M = ROBOT_OBSTACLE_CLEARANCE_M + 0.08
OBSTACLE_MAP_UPDATE_RANGE_M = ROBOT_OBSTACLE_CLEARANCE_M + 0.70
PERMANENT_POINT_PROXIMITY_M = 1.0


def euler_from_quaternion(quaternion):
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

class RoomVacuumController(Node):
    class State:
        MAPPING_SCAN = "MAPPING_SCAN"
        MAPPING_PLAN = "MAPPING_PLAN"
        MAPPING_GO_TO_FRONTIER = "MAPPING_GO_TO_FRONTIER"
        MAP_COMPLETE = "MAP_COMPLETE"
        CLEAN_PLAN = "CLEAN_PLAN"
        CLEAN_APPROACH_LANE = "CLEAN_APPROACH_LANE"
        CLEAN_SWEEP_LANE = "CLEAN_SWEEP_LANE"
        CLEAN_DONE = "CLEAN_DONE"
        FINAL_WALL_SCAN = "FINAL_WALL_SCAN"
        WALL_INSPECT = "WALL_INSPECT"  # After frontiers done: follow walls to promote all temp/purple to blue/locked

    def __init__(self):
        super().__init__(
            'room_vacuum_controller',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        self.declare_parameter_if_needed('use_sim_time', False)
        self.declare_parameter_if_needed('enable_viz', True)
        self.declare_parameter_if_needed('cmd_vel_topic', '/key_vel')
        self.declare_parameter_if_needed('scan_topic', '/scan')
        self.declare_parameter_if_needed('odom_topic', '/robot_lab_controller/odom')
        self.declare_parameter_if_needed('movement_log_enabled', True)
        self.declare_parameter_if_needed('movement_log_file', '')
        self.declare_parameter_if_needed('start_cleaning_after_mapping', True)
        self.declare_parameter_if_needed('cleaning_grid_spacing_m', CLEANING_GRID_SPACING_M)
        self.declare_parameter_if_needed('robot_obstacle_clearance_m', ROBOT_OBSTACLE_CLEARANCE_M)
        self.declare_parameter_if_needed('obstacle_map_update_range_m', OBSTACLE_MAP_UPDATE_RANGE_M)
        self.declare_parameter_if_needed('permanent_point_proximity_m', PERMANENT_POINT_PROXIMITY_M)
        self.declare_parameter_if_needed('save_map_enabled', True)
        # Use dynamic path for generated maps
        generated_map_dir = get_generated_map_path()
        self.declare_parameter_if_needed('map_output_dir', generated_map_dir)

        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        # Settings
        self.target_dist = 0.5
        self.forward_speed = 0.15
        self.rotate_speed = 0.7
        self.map_resolution = 0.15
        self.cleaning_grid_spacing = max(
            self.map_resolution,
            float(self.get_parameter('cleaning_grid_spacing_m').value),
        )
        self.robot_obstacle_clearance = max(
            self.map_resolution,
            float(self.get_parameter('robot_obstacle_clearance_m').value),
        )
        self.obstacle_map_update_range = max(
            self.robot_obstacle_clearance,
            float(self.get_parameter('obstacle_map_update_range_m').value),
        )
        self.permanent_point_proximity = float(self.get_parameter('permanent_point_proximity_m').value)
        self.lane_width = self.cleaning_grid_spacing
        self.map_lidar_range = 12.0
        self.map_scan_stride = 2
        self.initial_scan_turn_speed = 0.35
        self.frontier_min_unknown_neighbors = 3
        self.frontier_missing_limit = 1  # kept for compat; we no longer do rescans/spins after frontiers exhausted (go straight to wall inspect)

        self.frontier_reached_dist = 0.25
        self.path_waypoint_dist = 0.18
        self.path_replan_period = 8
        self.obstacle_stop_dist = max(FRONT_OBSTACLE_STOP_DISTANCE_M, self.robot_obstacle_clearance)
        self.side_obstacle_stop_dist = max(SIDE_OBSTACLE_STOP_DISTANCE_M, self.robot_obstacle_clearance)
        self.goal_blocked_dist = max(self.robot_obstacle_clearance + 0.05, 0.40)
        self.side_goal_blocked_dist = max(self.robot_obstacle_clearance * 0.75, 0.25)
        self.slow_down_dist = max(0.9, self.obstacle_stop_dist + 0.25)
        self.max_drive_angular = 0.45
        self.occupied_confirmations = 2
        self.occupied_clear_confirmations = 2
        self.max_occupancy_count = self.occupied_confirmations + self.occupied_clear_confirmations
        self.safety_radius_cells = max(1, int(math.ceil(self.robot_obstacle_clearance / self.map_resolution)))
        self.cleaning_goal_dist = 0.25
        self.cleaning_step_cells = max(1, int(round(self.cleaning_grid_spacing / self.map_resolution)))
        self.rejected_goal_radius_cells = max(1, int(round(0.20 / self.map_resolution)))
        self.start_cleaning_after_mapping = self.get_parameter('start_cleaning_after_mapping').value
        self.save_map_enabled = self.get_parameter('save_map_enabled').value
        self.map_output_dir = self.get_parameter('map_output_dir').value
        
        # Map saver instance
        self.map_saver = map_saver.MapSaver(self.map_resolution)
        
        # State variables
        self.state = self.State.MAPPING_SCAN
        self.current_pose = None
        self.total_dist_traveled = 0.0
        self.last_pose = None
        self.last_history_pose = None
        
        # Initial pose for map alignment (to handle robot rotation at startup)
        self.initial_pose = None  # (x, y, yaw) of first valid scan
        self.map_rotation_offset = 0.0  # Rotation to apply to align map with initial pose
        # Alignment to make map straight (un-tilted) on pygame regardless of robot yaw at launch
        self.alignment_complete = False
        
        # Wall tracking for final scan phase
        self.wall_points_visited = set()
        self.wall_precision_threshold = self.robot_obstacle_clearance
        self.final_scan_complete = False
        self.wall_follow_direction = 1
        self.current_wall_component = set()
        self.current_wall_approach_cell = None
        self.wall_inspection_skipped_cells = set()
        self.wall_inspection_skipped_map_version = -1
        self.wall_cluster_gap_cells = max(1, int(math.ceil(0.30 / self.map_resolution)))
        self.wall_approach_min_cells = self.safety_radius_cells + 1
        self.wall_approach_max_cells = max(
            self.wall_approach_min_cells + 2,
            int(math.ceil((self.robot_obstacle_clearance + 0.45) / self.map_resolution)),
        )
        self.wall_follow_start_distance = max(
            self.wall_precision_threshold * 2.5,
            self.wall_approach_min_cells * self.map_resolution,
        )

        self.target_heading = 0.0
        self.scan_start_yaw = None
        self.scan_last_yaw = None
        self.scan_accumulated_yaw = 0.0
        self.initial_scan_complete = False
        self.no_frontier_count = 0
        self.replan_counter = 0
        self.path_failed = False
        self.latest_lidar_range = self.map_lidar_range

        # Mapping and cleaning data
        self.free_counts = {}
        self.occupied_counts = {}
        self.temporary_obstacle_counts = {}
        self.locked_obstacle_cells = set()
        self.current_scan_obstacle_cells = set()
        self.frontier_cells = []
        self.current_goal_cell = None
        self.current_path = []
        self.current_path_index = 0
        self.current_path_direct = False
        self.cleaning_goals = []
        self.current_clean_goal_index = 0
        self.current_cleaning_lane_cells = []
        self.current_lane_start_cell = None
        self.current_lane_endpoint_cell = None
        self.cleaned_cells = set()
        self.cleaned_version = 0
        self.rejected_frontier_cells = set()
        self.rejected_cleaning_cells = set()
        self.temporarily_skipped_cleaning_cells = set()
        self.map_version = 0
        self.last_cleaning_goal_version = -1
        self.history = [] # For visualization

        # Movement logging
        self.movement_log_enabled = self.get_parameter('movement_log_enabled').value
        self.movement_log_path = None
        self.movement_log_file = None
        self.movement_log_writer = None
        self.last_cmd = Twist()
        self.last_logged_odom_pose = None
        self.setup_movement_log()

        self.get_logger().info(
            "Room Vacuum Controller with mapping-first exploration "
            f"(cmd_vel_topic={self.cmd_vel_topic}, scan_topic={self.scan_topic}, odom_topic={self.odom_topic})"
        )

        # Separate cleaning (room_cleaner.py). Instantiate here after map data structures exist.
        # Cleaning flow is now independent; main post-frontier behavior is wall inspect to blue all.
        self.room_cleaner = None
        try:
            import cleaning_controller as rc
            cfg = rc.CleanerConfig(
                cleaning_grid_spacing=self.cleaning_grid_spacing,
                map_resolution=self.map_resolution,
                robot_obstacle_clearance=self.robot_obstacle_clearance,
                lane_width=self.lane_width,
            )
            self.room_cleaner = rc.RoomCleaner(cfg)
            
            # Try to load existing map from generated_map directory
            generated_map_dir = get_generated_map_path()
            if not self.room_cleaner.load_map_from_pgm(map_dir=generated_map_dir):
                # If no map found or failed to load, use current mapping data
                self.room_cleaner.set_map_data(
                    getattr(self, 'free_counts', {}),
                    getattr(self, 'occupied_counts', {}),
                    getattr(self, 'temporary_obstacle_counts', {})
                )
            self.room_cleaner.set_callbacks(
                is_occupied=getattr(self, 'is_cell_occupied', None),
                is_free=getattr(self, 'is_cell_free', None),
                is_blocked=getattr(self, 'is_cell_blocked', None),
                plan_path=getattr(self, 'plan_path_to_cell', None),
                reached_cell=getattr(self, 'reached_cell', None),
            )
        except Exception as exc:
            self.get_logger().debug(f"room_cleaner not integrated (ok, cleaning code still inline if used): {exc}")

        self.enable_viz = bool(self.get_parameter('enable_viz').value) and HAS_PYGAME
        if bool(self.get_parameter('enable_viz').value) and not HAS_PYGAME:
            self.get_logger().warn("pygame not available, visualization disabled")

        self.log_movement("node_start", note="Room vacuum controller started.")

        # Pygame Visualization (optional for headless operation)
        if self.enable_viz:
            pygame.init()
            self.screen_size = 600
            self.screen = pygame.display.set_mode((self.screen_size, self.screen_size))
            pygame.display.set_caption("Bumperbot Mapping and Cleaning Monitor")
            self.status_font = self.load_blocky_font(18)
            self.ring_surface_cache = {}
            self.default_scale = 40.0
            self.scale = self.default_scale # Pixels per meter
            self.min_scale = 8.0
            self.max_scale = 220.0
            self.view_center_x = 0.0
            self.view_center_y = 0.0
            self.view_locked = False
            self.show_max_vision = False
            self.show_current_scan = True
            self.view_menu_open = False
            self.dragging_view = False
            self.drag_start_mouse = None
            self.drag_start_center = None
            self.view_menu_button_rect = pygame.Rect(self.screen_size - 118, 10, 108, 28)
            self.view_menu_width = 190
            self.view_menu_item_height = 28
            self.viz_timer = self.create_timer(0.1, self.pygame_loop)

    def declare_parameter_if_needed(self, name, default_value):
        if not self.has_parameter(name):
            self.declare_parameter(name, default_value)

    def odom_callback(self, msg):
        yaw = euler_from_quaternion(msg.pose.pose.orientation)
        self.current_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)
        
        if self.last_pose is not None:
            dist = math.sqrt((self.current_pose[0] - self.last_pose[0])**2 + 
                             (self.current_pose[1] - self.last_pose[1])**2)
            self.total_dist_traveled += dist
            
        self.record_visited_position()
        
        self.last_pose = self.current_pose
        self.log_odom_if_needed()

    def record_visited_position(self):
        if self.current_pose is None:
            return

        if self.last_history_pose is None:
            self.history.append((self.current_pose[0], self.current_pose[1]))
            self.last_history_pose = self.current_pose
            self.mark_cleaned_area(self.world_to_cell(self.current_pose[0], self.current_pose[1]))
            return

        dist = math.sqrt((self.current_pose[0] - self.last_history_pose[0])**2 +
                         (self.current_pose[1] - self.last_history_pose[1])**2)
        if dist <= 0.05:
            return

        self.history.append((self.current_pose[0], self.current_pose[1]))
        self.last_history_pose = self.current_pose
        self.mark_cleaned_area(self.world_to_cell(self.current_pose[0], self.current_pose[1]))

    def pygame_loop(self):
        for event in pygame.event.get():
            self.handle_pygame_event(event)

        self.screen.fill((30, 30, 30)) # Dark background

        # Draw occupancy map.
        for cell in self.known_free_cells():
            sx, sy = self.world_to_screen(*self.cell_to_world(cell))
            if 0 <= sx < self.screen_size and 0 <= sy < self.screen_size:
                pygame.draw.rect(self.screen, (45, 45, 45), (sx - 1, sy - 1, 2, 2))

        for cell in self.known_occupied_cells():
            sx, sy = self.world_to_screen(*self.cell_to_world(cell))
            if 0 <= sx < self.screen_size and 0 <= sy < self.screen_size:
                pygame.draw.rect(self.screen, (0, 100, 255), (sx - 2, sy - 2, 4, 4))  # Blue - permanent/locked obstacles

        for cell in self.known_temporary_obstacle_cells():
            sx, sy = self.world_to_screen(*self.cell_to_world(cell))
            if 0 <= sx < self.screen_size and 0 <= sy < self.screen_size:
                pygame.draw.rect(self.screen, (128, 0, 128), (sx - 2, sy - 2, 4, 4))  # Purple - unsure/temporary walls (revisable until close)

        # Draw current scan obstacle cells as white outline squares (toggleable)
        if self.show_current_scan:
            for cell in self.current_scan_obstacle_cells:
                sx, sy = self.world_to_screen(*self.cell_to_world(cell))
                if 0 <= sx < self.screen_size and 0 <= sy < self.screen_size:
                    # Draw white square outline on top of existing blue/purple squares
                    pygame.draw.rect(self.screen, (255, 255, 255), (sx - 3, sy - 3, 7, 7), 1)  # White outline square - current scan points

        # Draw Path History
        for cell in {self.world_to_cell(x, y) for x, y in self.history}:
            self.draw_map_point(cell, (235, 235, 235))

        # Draw active path and cleaning goals.
        for cell in self.current_path[self.current_path_index:]:
            self.draw_map_point(cell, (255, 160, 0))

        for cell in self.cleaning_goals[self.current_clean_goal_index:]:
            if self.is_cell_cleaned(cell):
                continue
            sx, sy = self.world_to_screen(*self.cell_to_world(cell))
            if 0 <= sx < self.screen_size and 0 <= sy < self.screen_size:
                pygame.draw.circle(self.screen, (255, 230, 80), (sx, sy), 2)

        for cell in self.frontier_cells:
            sx, sy = self.world_to_screen(*self.cell_to_world(cell))
            if 0 <= sx < self.screen_size and 0 <= sy < self.screen_size:
                self.draw_smooth_ring((sx, sy), 4, (0, 255, 0), 2)

        if self.current_goal_cell:
            sx, sy = self.world_to_screen(*self.cell_to_world(self.current_goal_cell))
            if 0 <= sx < self.screen_size and 0 <= sy < self.screen_size:
                self.draw_smooth_ring((sx, sy), 7, (255, 120, 0), 2)

        if self.show_max_vision and self.current_pose:
            self.draw_max_vision_ring()

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
        self.draw_status_row(10, [("State:", self.state)])
        self.draw_status_row(30, [("Frontiers:", len(self.frontier_cells)), ("Path:", len(self.current_path))])
        self.draw_status_row(
            50,
            [
                ("Free:", len(self.known_free_cells())),
                ("Walls:", len(self.known_occupied_cells())),
                ("Temp:", len(self.known_temporary_obstacle_cells())),
                ("Clean:", self.remaining_cleaning_goals()),
            ],
        )
        self.draw_view_menu(self.status_font)

        pygame.display.flip()

    def world_to_screen(self, x, y):
        view_center_x, view_center_y = self.get_view_center()
        # Flip y because pygame y increases downwards
        screen_x = int(self.screen_size / 2 + (x - view_center_x) * self.scale)
        screen_y = int(self.screen_size / 2 - (y - view_center_y) * self.scale)
        return screen_x, screen_y

    def screen_to_world(self, screen_x, screen_y):
        view_center_x, view_center_y = self.get_view_center()
        world_x = view_center_x + (screen_x - self.screen_size / 2) / self.scale
        world_y = view_center_y - (screen_y - self.screen_size / 2) / self.scale
        return world_x, world_y

    def load_blocky_font(self, size):
        for font_name in ("dejavusansmono", "liberationmono", "consolas", "couriernew", "monospace"):
            font_path = pygame.font.match_font(font_name, bold=True)
            if font_path:
                return pygame.font.Font(font_path, size)
        return pygame.font.SysFont("monospace", size, bold=True)

    def draw_map_point(self, cell, color):
        sx, sy = self.world_to_screen(*self.cell_to_world(cell))
        rect = pygame.Rect(sx - 2, sy - 2, 4, 4)
        if rect.right < 0 or rect.left >= self.screen_size or rect.bottom < 0 or rect.top >= self.screen_size:
            return

        pygame.draw.rect(self.screen, color, rect)

    def draw_smooth_ring(self, center, radius, color, width):
        ring = self.get_smooth_ring_surface(radius, color, width)
        rect = ring.get_rect(center=center)
        self.screen.blit(ring, rect)

    def draw_max_vision_ring(self):
        sx, sy = self.world_to_screen(self.current_pose[0], self.current_pose[1])
        radius_px = max(1, int(round(self.latest_lidar_range * self.scale)))
        pygame.draw.circle(self.screen, (255, 65, 65), (sx, sy), radius_px, 2)

    def get_smooth_ring_surface(self, radius, color, width):
        cache_key = (radius, color, width)
        if cache_key in self.ring_surface_cache:
            return self.ring_surface_cache[cache_key]

        scale = 4
        padding = 3
        size = (radius + padding) * 2
        scaled_size = size * scale
        scaled_radius = radius * scale
        scaled_width = max(1, width * scale)
        surface = pygame.Surface((scaled_size, scaled_size), pygame.SRCALPHA)
        pygame.draw.circle(
            surface,
            color,
            (scaled_size // 2, scaled_size // 2),
            scaled_radius,
            scaled_width,
        )
        ring = pygame.transform.smoothscale(surface, (size, size))
        self.ring_surface_cache[cache_key] = ring
        return ring

    def draw_status_row(self, y, fields):
        x = 10
        label_color = (255, 145, 45)
        value_color = (255, 255, 255)
        for label, value in fields:
            label_surface = self.status_font.render(f"{label} ", True, label_color)
            value_surface = self.status_font.render(str(value), True, value_color)
            self.screen.blit(label_surface, (x, y))
            x += label_surface.get_width()
            self.screen.blit(value_surface, (x, y))
            x += value_surface.get_width() + 16
            
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
            elif event.key == pygame.K_v:
                self.show_max_vision = not self.show_max_vision
            elif event.key == pygame.K_r:
                self.reset_view()
            return

        if event.type == pygame.MOUSEWHEEL:
            self.zoom_view(event.y, pygame.mouse.get_pos())
            return

        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:
                if self.view_menu_button_rect.collidepoint(event.pos):
                    self.view_menu_open = not self.view_menu_open
                    return
                if self.view_menu_open:
                    if self.handle_view_menu_click(event.pos):
                        return
                    self.view_menu_open = False
                    return
            if event.button == 4:
                self.zoom_view(1, event.pos)
                return
            if event.button == 5:
                self.zoom_view(-1, event.pos)
                return
            if event.button in (1, 2):
                self.start_view_drag(event.pos)
                return

        if event.type == pygame.MOUSEBUTTONUP:
            if event.button in (1, 2):
                self.dragging_view = False
                return

        if event.type == pygame.MOUSEMOTION and self.dragging_view:
            self.update_view_drag(event.pos)

    def draw_view_menu(self, font):
        button_color = (62, 62, 62) if not self.view_menu_open else (84, 84, 84)
        pygame.draw.rect(self.screen, button_color, self.view_menu_button_rect, border_radius=3)
        pygame.draw.rect(self.screen, (190, 190, 190), self.view_menu_button_rect, 1, border_radius=3)
        label_surface = font.render("VIEW", True, (255, 255, 255))
        label_rect = label_surface.get_rect(center=self.view_menu_button_rect.center)
        self.screen.blit(label_surface, label_rect)

        if not self.view_menu_open:
            return

        menu_rect = self.get_view_menu_rect()
        pygame.draw.rect(self.screen, (48, 48, 48), menu_rect)
        pygame.draw.rect(self.screen, (205, 205, 205), menu_rect, 1)
        self.draw_view_menu_item(0, "Follow View", self.view_locked, font)
        self.draw_view_menu_item(1, "Show Max Vision", self.show_max_vision, font)
        self.draw_view_menu_item(2, "Show Current Scan", self.show_current_scan, font)
        self.draw_view_menu_item(3, "Save Map", False, font)

    def draw_view_menu_item(self, index, label, checked, font):
        rect = self.get_view_menu_item_rect(index)
        pygame.draw.rect(self.screen, (58, 58, 58), rect)
        if index > 0:
            pygame.draw.line(self.screen, (75, 75, 75), rect.topleft, rect.topright)

        check_text = "[x]" if checked else "[ ]"
        check_surface = font.render(check_text, True, (255, 145, 45))
        label_surface = font.render(label, True, (255, 255, 255))
        self.screen.blit(check_surface, (rect.left + 8, rect.top + 5))
        self.screen.blit(label_surface, (rect.left + 48, rect.top + 5))

    def handle_view_menu_click(self, pos):
        if self.get_view_menu_item_rect(0).collidepoint(pos):
            self.toggle_view_lock()
            self.view_menu_open = False
            return True
        if self.get_view_menu_item_rect(1).collidepoint(pos):
            self.show_max_vision = not self.show_max_vision
            self.view_menu_open = False
            return True
        if self.get_view_menu_item_rect(2).collidepoint(pos):
            self.show_current_scan = not self.show_current_scan
            self.view_menu_open = False
            return True
        if self.get_view_menu_item_rect(3).collidepoint(pos):
            self.save_map_to_pgm()
            self.view_menu_open = False
            return True
        return False

    def get_view_menu_rect(self):
        height = self.view_menu_item_height * 4
        left = self.screen_size - self.view_menu_width - 10
        top = self.view_menu_button_rect.bottom + 3
        return pygame.Rect(left, top, self.view_menu_width, height)

    def get_view_menu_item_rect(self, index):
        menu_rect = self.get_view_menu_rect()
        return pygame.Rect(
            menu_rect.left,
            menu_rect.top + index * self.view_menu_item_height,
            menu_rect.width,
            self.view_menu_item_height,
        )

    def toggle_view_lock(self):
        if self.view_locked:
            self.view_center_x, self.view_center_y = self.get_view_center()
            self.view_locked = False
        else:
            self.view_locked = True
            self.dragging_view = False

    def reset_view(self):
        self.view_locked = False
        self.scale = self.default_scale
        if self.current_pose:
            self.view_center_x = self.current_pose[0]
            self.view_center_y = self.current_pose[1]
        else:
            self.view_center_x = 0.0
            self.view_center_y = 0.0

    def start_view_drag(self, mouse_pos):
        if self.view_locked:
            self.view_center_x, self.view_center_y = self.get_view_center()
            self.view_locked = False
        self.dragging_view = True
        self.drag_start_mouse = mouse_pos
        self.drag_start_center = (self.view_center_x, self.view_center_y)

    def update_view_drag(self, mouse_pos):
        if self.drag_start_mouse is None or self.drag_start_center is None:
            return
        dx = mouse_pos[0] - self.drag_start_mouse[0]
        dy = mouse_pos[1] - self.drag_start_mouse[1]
        self.view_center_x = self.drag_start_center[0] - dx / self.scale
        self.view_center_y = self.drag_start_center[1] + dy / self.scale

    def zoom_view(self, wheel_delta, mouse_pos):
        if wheel_delta == 0:
            return
        before = self.screen_to_world(mouse_pos[0], mouse_pos[1])
        zoom_factor = 1.15 ** wheel_delta
        self.scale = self.clamp(self.scale * zoom_factor, self.min_scale, self.max_scale)
        if not self.view_locked:
            after = self.screen_to_world(mouse_pos[0], mouse_pos[1])
            self.view_center_x += before[0] - after[0]
            self.view_center_y += before[1] - after[1]

    def scan_callback(self, msg):
        if self.current_pose is None:
            return

        # Compute distances early (needed for alignment publish and later logic)
        dist_front = self.get_min_dist(msg, -0.2, 0.2)
        dist_front_left = self.get_min_dist(msg, 0.2, 0.75)
        dist_front_right = self.get_min_dist(msg, -0.75, -0.2)
        dist_left = self.get_min_dist(msg, 1.3, 1.8)
        dist_right = self.get_min_dist(msg, -1.8, -1.3)

        # First, rotate the robot to yaw ~0 if not already aligned. This ensures that
        # when we capture the initial_pose the map coordinate system starts "straight"
        # (no tilt on the pygame window caused by the robot's yaw at script start).
        if not self.alignment_complete:
            yaw = self.current_pose[2]
            err = self.normalize_angle(0.0 - yaw)
            if abs(err) < 0.06:
                # Close enough to straight
                self.initial_pose = self.current_pose
                self.map_rotation_offset = 0.0
                self.alignment_complete = True
                self.reset_scan_tracking()
                self.get_logger().info("Initial heading aligned to 0 for straight (un-tilted) map on pygame.")
            else:
                # Rotate toward 0 yaw first (do not process mapping or set initial yet)
                cmd = Twist()
                cmd.angular.z = self.initial_scan_turn_speed if err > 0 else -self.initial_scan_turn_speed
                self.publish_and_log(cmd, dist_front, dist_left, dist_right, "initial_align_to_straight")
                return

        # Set initial pose on first scan to handle robot rotation at startup (now after alignment)
        if self.initial_pose is None:
            self.initial_pose = self.current_pose
            self.map_rotation_offset = self.initial_pose[2]
            self.get_logger().info(f"Initial pose set: x={self.initial_pose[0]:.2f}, y={self.initial_pose[1]:.2f}, yaw={self.initial_pose[2]:.2f}")

        cmd = Twist()
        self.update_latest_lidar_range(msg)
        map_version_before_scan = self.map_version
        self.update_occupancy_map(msg)
        map_changed = self.map_version != map_version_before_scan
        front_blocked = self.is_front_blocked(dist_front, dist_front_left, dist_front_right)
        self.path_failed = False
        
        # Track wall points when we're close to them
        self.track_visited_wall_points(msg)
        
        # Check if we should start final wall scan after cleaning is done
        if self.state == self.State.CLEAN_DONE and not self.final_scan_complete and self.all_walls_visited(msg) == False:
            self.get_logger().info("Starting final wall scan phase.")
            self.state = self.State.FINAL_WALL_SCAN

        if self.state == self.State.MAPPING_SCAN:
            note = self.mapping_scan(cmd)

        elif self.state == self.State.MAPPING_PLAN:
            note = self.plan_next_mapping_goal()

        elif self.state == self.State.MAPPING_GO_TO_FRONTIER:
            path_reached = self.follow_current_path(cmd)
            if path_reached:
                self.set_state(self.State.MAPPING_SCAN, "frontier reached")
                note = "mapping_frontier_reached"
            elif self.path_failed:
                self.current_path = []
                self.reject_current_goal(self.rejected_frontier_cells, "frontier path became invalid")
                self.set_state(self.State.MAPPING_PLAN, "frontier path failed")
                note = "mapping_path_failed_skip_goal"
            elif self.is_goal_blocked_forward_command(cmd, dist_front, dist_front_left, dist_front_right):
                self.clear_motion_command(cmd)
                self.current_path = []
                self.mark_current_front_obstacles(dist_front, dist_front_left, dist_front_right)
                self.reject_current_goal(self.rejected_frontier_cells, "frontier blocked by obstacle")
                self.set_state(self.State.MAPPING_PLAN, "front obstacle while driving to frontier")
                note = "mapping_blocked_skip_goal"
            elif self.is_soft_blocked_forward_command(cmd, front_blocked):
                self.limit_speed_near_obstacle(cmd, dist_front, dist_front_left, dist_front_right)
                note = "mapping_follow_path_near_obstacle"
            else:
                self.limit_speed_near_obstacle(cmd, dist_front, dist_front_left, dist_front_right)
                note = "mapping_follow_path"

        elif self.state == self.State.MAP_COMPLETE:
            note = "map_complete_waiting"
            # Cleaning is now separate (see room_cleaner.py). After frontiers + wall inspect
            # we reach here with all obstacles blue. Do not auto start inline cleaning.
            if self.start_cleaning_after_mapping:
                self.get_logger().info("Mapping + wall blueing complete. Use room_cleaner.py for separate cleaning phase.")

        elif self.state == self.State.WALL_INSPECT:
            # Follow walls/obstacles closely. This gets the robot near every found wall/temp
            # so that promote/resolve in update_occupancy promotes temps to locked (blue).
            # Stop when no more temporary (purple) points remain -- all are now blue.
            remaining_temps = set(self.known_temporary_obstacle_cells())
            if not remaining_temps:
                self.get_logger().info("Wall inspection complete! All found walls/obstacles are now blue (locked).")
                # Save map now that all are resolved to blue
                if self.save_map_enabled and self.initial_pose is not None:
                    self.save_current_map()
                self.clear_wall_inspection_target()
                self.wall_inspection_skipped_cells.clear()
                self.set_state(self.State.MAP_COMPLETE, "all obstacles promoted to blue via wall follow")
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                note = "wall_inspect_complete"
            else:
                note = self.inspect_temporary_walls(msg, cmd, remaining_temps)

        elif self.state == self.State.CLEAN_PLAN:
            note = self.plan_cleaning()

        elif self.state in (self.State.CLEAN_APPROACH_LANE, self.State.CLEAN_SWEEP_LANE):
            refresh_note = self.refresh_cleaning_goals_if_needed(map_changed, "lidar map changed while cleaning")
            if self.state not in (self.State.CLEAN_APPROACH_LANE, self.State.CLEAN_SWEEP_LANE):
                note = refresh_note
            elif self.state == self.State.CLEAN_APPROACH_LANE and self.current_cleaning_lane_start_reached():
                note = self.start_cleaning_lane_sweep()
            elif self.state == self.State.CLEAN_SWEEP_LANE and self.current_cleaning_lane_finished():
                note = self.finish_current_cleaning_lane()
            else:
                path_reached = self.follow_current_path(cmd)
                if path_reached and self.state == self.State.CLEAN_APPROACH_LANE:
                    note = self.start_cleaning_lane_sweep()
                elif path_reached:
                    note = self.finish_current_cleaning_lane()
                elif self.path_failed:
                    self.current_path = []
                    note = self.refresh_cleaning_goals("cleaning path became invalid")
                elif self.is_goal_blocked_forward_command(cmd, dist_front, dist_front_left, dist_front_right):
                    self.clear_motion_command(cmd)
                    self.current_path = []
                    self.mark_current_front_obstacles(dist_front, dist_front_left, dist_front_right)
                    note = self.refresh_cleaning_goals("cleaning goal blocked by new obstacle")
                elif self.is_soft_blocked_forward_command(cmd, front_blocked):
                    self.limit_speed_near_obstacle(cmd, dist_front, dist_front_left, dist_front_right)
                    note = "cleaning_follow_path_near_obstacle"
                else:
                    self.limit_speed_near_obstacle(cmd, dist_front, dist_front_left, dist_front_right)
                    note = "cleaning_follow_path"
            if refresh_note is not None and note != refresh_note:
                note = f"{refresh_note}; {note}"

        elif self.state == self.State.CLEAN_DONE:
            note = "cleaning_done_stop"
            refresh_note = self.refresh_cleaning_goals_if_needed(map_changed, "lidar map changed after cleaning")
            if refresh_note is not None:
                note = refresh_note
        
        elif self.state == self.State.FINAL_WALL_SCAN:
            if self.all_walls_visited(msg):
                self.get_logger().info("Final wall scan complete! All walls have been visited closely.")
                self.final_scan_complete = True
                self.set_state(self.State.CLEAN_DONE, "final wall scan complete")
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                note = "final_wall_scan_complete"
            else:
                cmd = self.follow_walls_closely(msg, cmd)
                note = "final_wall_scan_following"

        self.publish_and_log(cmd, dist_front, dist_left, dist_right, note)

    def is_soft_blocked_forward_command(self, cmd, front_blocked):
        return front_blocked and cmd.linear.x > 0.0

    def is_goal_blocked_forward_command(self, cmd, dist_front, dist_front_left, dist_front_right):
        if cmd.linear.x <= 0.0:
            return False
        if dist_front < self.goal_blocked_dist:
            return True
        if dist_front_left < self.side_goal_blocked_dist:
            return True
        if dist_front_right < self.side_goal_blocked_dist:
            return True
        return False

    def stop_linear_for_obstacle(self, cmd, dist_front_left=None, dist_front_right=None):
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        if abs(cmd.angular.z) >= 0.05:
            return
        if dist_front_left is not None and dist_front_right is not None and dist_front_left < dist_front_right:
            cmd.angular.z = -self.rotate_speed * 0.5
        else:
            cmd.angular.z = self.rotate_speed * 0.5


    def clear_motion_command(self, cmd):
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0

    def mapping_scan(self, cmd):
        yaw = self.current_pose[2]
        if self.scan_start_yaw is None:
            self.scan_start_yaw = yaw
            self.scan_last_yaw = yaw
            self.scan_accumulated_yaw = 0.0

        yaw_delta = abs(self.normalize_angle(yaw - self.scan_last_yaw))
        self.scan_accumulated_yaw += yaw_delta
        self.scan_last_yaw = yaw

        cmd.angular.z = self.initial_scan_turn_speed
        if self.scan_accumulated_yaw >= 2.0 * math.pi:
            self.initial_scan_complete = True
            self.reset_scan_tracking()
            self.set_state(self.State.MAPPING_PLAN, "finished 360 degree map scan")
            return "mapping_scan_complete"

        return "mapping_scan_rotate"

    def plan_next_mapping_goal(self):
        self.frontier_cells = self.find_frontiers()
        if not self.frontier_cells:
            # No more frontiers after visiting/rotating at previous ones.
            # Do NOT do extra MAPPING_SCAN rotations (that caused long spinning).
            # Immediately switch to wall inspection to follow walls/obstacles
            # and get close so all purple/temps are promoted to blue/locked.
            self.frontier_cells = []
            self.no_frontier_count = 0
            self.set_state(self.State.WALL_INSPECT, "no more frontiers - follow walls until all obstacles are blue")
            return "mapping_frontiers_exhausted_start_wall_inspect"

        # Frontiers (green points) exist. Reset missing counter.
        # The robot must visit them (go + rotate at each) so that rotations can
        # create/reveal more frontiers until the whole room boundary (all blue/purple walls)
        # has been approached and no free "exits" remain into unknown.
        self.no_frontier_count = 0

        # Build ordered candidates (nearest first is fine; old logic avoided too-close ones).
        start = self.world_to_cell(self.current_pose[0], self.current_pose[1]) if self.current_pose else (0, 0)
        min_dist_cells = max(2, int(0.6 / self.map_resolution))

        # Filter out already rejected; sort by distance.
        candidates = [
            c for c in self.frontier_cells
            if not self.is_rejected_goal(c, self.rejected_frontier_cells)
        ]
        candidates.sort(key=lambda cell: abs(cell[0] - start[0]) + abs(cell[1] - start[1]))

        chosen_goal = None
        for cell in candidates:
            # Skip very close ones if we have farther candidates (prefer some progress), but try them if needed.
            dist = abs(cell[0] - start[0]) + abs(cell[1] - start[1])
            if dist < min_dist_cells and chosen_goal is not None:
                continue

            if not self.is_cell_free(cell) or self.is_cell_blocked(cell):
                self.reject_goal_cell(cell, self.rejected_frontier_cells, "frontier no longer free/blocked")
                continue

            if self.plan_path_to_cell(cell):
                chosen_goal = cell
                break
            else:
                self.reject_goal_cell(cell, self.rejected_frontier_cells, "frontier was not pathable")

        if chosen_goal is None:
            # Frontiers existed in list but none pathable (after rejects).
            # Instead of spinning rescans, go to wall inspect phase to blue what we have.
            self.frontier_cells = []
            self.set_state(self.State.WALL_INSPECT, "no pathable frontiers left - wall inspect to promote to blue")
            return "mapping_no_pathable_frontiers_wall_inspect"

        # Found a pathable green point to go to.
        self.set_state(self.State.MAPPING_GO_TO_FRONTIER, f"frontier goal {chosen_goal}")
        return f"mapping_frontier_goal_{chosen_goal}"

    def complete_mapping(self):
        self.current_goal_cell = None
        self.current_path = []
        self.frontier_cells = []
        self.set_state(self.State.MAP_COMPLETE, "no reachable frontiers left")
        self.get_logger().info("Map complete: no reachable open exits/frontiers remain.")
        
        # Save the map if enabled
        if self.save_map_enabled and self.initial_pose is not None:
            self.save_current_map()

    def save_current_map(self):
        """Save the current occupancy map to PGM and YAML files."""
        try:
            # Get map origin from initial pose
            origin_x, origin_y, origin_yaw = self.initial_pose
            
            # Use fixed filename - always overwrite the same map
            map_name = "map"
            
            # Save the map
            map_path = map_saver.save_occupancy_map(
                self.free_counts,
                self.occupied_counts,
                origin_x,
                origin_y,
                self.map_resolution,
                output_dir=self.map_output_dir,
                map_name=map_name
            )
            
            self.get_logger().info(f"Map saved to: {map_path}")
            self.get_logger().info(f"Map YAML metadata saved to: {self.map_output_dir}/{map_name}.yaml")
            
        except Exception as e:
            self.get_logger().error(f"Failed to save map: {e}")

    def save_map_to_pgm(self):
        """Save the current map to PGM file when menu button is pressed."""
        if self.initial_pose is None:
            self.get_logger().warn("Cannot save map: initial pose not set yet")
            return
            
        try:
            # Get map origin from initial pose
            origin_x, origin_y, origin_yaw = self.initial_pose
            
            # Use fixed filename - always overwrite the same map
            map_name = "map"
            
            # Save the map
            map_path = map_saver.save_occupancy_map(
                self.free_counts,
                self.occupied_counts,
                origin_x,
                origin_y,
                self.map_resolution,
                output_dir=self.map_output_dir,
                map_name=map_name
            )
            
            self.get_logger().info(f"Map saved to: {map_path}")
            self.get_logger().info(f"Map YAML metadata saved to: {self.map_output_dir}/{map_name}.yaml")
            
        except Exception as e:
            self.get_logger().error(f"Failed to save map: {e}")

    def plan_cleaning(self):
        # Prefer separate room_cleaner for planning if available (cleaning decoupled)
        if self.room_cleaner is not None:
            try:
                self.room_cleaner.set_map_data(
                    self.free_counts, self.occupied_counts, self.temporary_obstacle_counts
                )
                self.temporarily_skipped_cleaning_cells.clear()
                self.mark_history_cleaned()
                note = self.room_cleaner.plan_cleaning()
                # sync back the state for controller execution loop
                self.cleaning_goals = list(self.room_cleaner.cleaning_goals)
                self.current_clean_goal_index = self.room_cleaner.current_clean_goal_index
                self.current_cleaning_lane_cells = list(self.room_cleaner.current_cleaning_lane_cells)
                self.current_lane_start_cell = self.room_cleaner.current_lane_start_cell
                self.current_lane_endpoint_cell = self.room_cleaner.current_lane_endpoint_cell
                self.current_goal_cell = self.room_cleaner.current_goal_cell
                return note or self.plan_next_cleaning_lane("cleaning started via room_cleaner")
            except Exception:
                pass  # fall back to inline

        self.temporarily_skipped_cleaning_cells.clear()
        self.mark_history_cleaned()
        self.cleaning_goals = self.build_cleaning_goals()
        return self.plan_next_cleaning_lane("cleaning started")

    def refresh_cleaning_goals_if_needed(self, map_changed, reason):
        if not map_changed and self.last_cleaning_goal_version == self.map_version:
            return None
        return self.refresh_cleaning_goals(reason)

    def refresh_cleaning_goals(self, reason):
        old_remaining = set(self.cleaning_goals[self.current_clean_goal_index:])
        self.temporarily_skipped_cleaning_cells.clear()
        self.mark_history_cleaned()
        new_goals = self.build_cleaning_goals()
        new_goal_set = set(new_goals)
        added_count = len(new_goal_set - old_remaining)
        removed_count = len(old_remaining - new_goal_set)
        self.cleaning_goals = new_goals
        note = self.plan_next_cleaning_lane(reason)

        self.log_movement(
            "cleaning_goals_refreshed",
            note=f"{reason}; added={added_count}; removed={removed_count}; goals={len(new_goals)}",
        )
        return f"cleaning_goals_refreshed_added_{added_count}_removed_{removed_count}; {note}"

    def plan_next_cleaning_lane(self, reason):
        self.clear_cleaning_lane_state()
        self.current_path = []
        self.current_path_direct = False
        self.current_clean_goal_index = 0
        self.last_cleaning_goal_version = self.map_version

        if not self.cleaning_goals:
            self.current_goal_cell = None
            self.set_state(self.State.CLEAN_DONE, "no reachable cells to clean")
            return "cleaning_no_goals"

        while self.current_clean_goal_index < len(self.cleaning_goals):
            start_cell = self.cleaning_goals[self.current_clean_goal_index]
            if (
                self.is_cell_cleaned(start_cell)
                or self.is_rejected_goal(start_cell, self.rejected_cleaning_cells)
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
                self.set_state(self.State.CLEAN_APPROACH_LANE, reason)
                return f"cleaning_approach_lane_{self.current_lane_start_cell}_to_{self.current_lane_endpoint_cell}"

            self.skip_cleaning_lane_cells()
            self.current_clean_goal_index += len(lane_cells)

        self.clear_cleaning_lane_state()
        self.current_goal_cell = None
        self.current_path = []
        self.set_state(self.State.CLEAN_DONE, "no cleaning lanes reachable")
        return "cleaning_lanes_unreachable"

    def clear_cleaning_lane_state(self):
        self.current_cleaning_lane_cells = []
        self.current_lane_start_cell = None
        self.current_lane_endpoint_cell = None

    def get_cleaning_lane_cells(self, start_index):
        if start_index >= len(self.cleaning_goals):
            return []

        start_cell = self.cleaning_goals[start_index]
        lane_cells = [start_cell]
        max_gap_cells = max(self.cleaning_step_cells + 1, int(math.ceil(self.cleaning_step_cells * 1.5)))

        for cell in self.cleaning_goals[start_index + 1:]:
            previous = lane_cells[-1]
            if cell[1] != start_cell[1]:
                break
            if abs(cell[0] - previous[0]) > max_gap_cells:
                break
            if (
                self.is_cell_cleaned(cell)
                or self.is_rejected_goal(cell, self.rejected_cleaning_cells)
                or cell in self.temporarily_skipped_cleaning_cells
            ):
                break
            lane_cells.append(cell)

        return lane_cells

    def current_cleaning_lane_start_reached(self):
        if self.current_lane_start_cell is None:
            return False
        return self.reached_cell(self.current_lane_start_cell, self.cleaning_goal_dist)

    def current_cleaning_lane_finished(self):
        if self.current_lane_endpoint_cell is None:
            return False
        if self.reached_cell(self.current_lane_endpoint_cell, self.cleaning_goal_dist):
            return True
        return all(self.is_cell_cleaned(cell) for cell in self.current_cleaning_lane_cells)

    def start_cleaning_lane_sweep(self):
        self.mark_cleaned_area(self.current_lane_start_cell)

        remaining_lane_cells = [
            cell for cell in self.current_cleaning_lane_cells
            if not self.is_cell_cleaned(cell) and not self.is_cell_blocked(cell)
        ]
        if not remaining_lane_cells:
            return self.finish_current_cleaning_lane()

        for endpoint in reversed(remaining_lane_cells):
            if self.plan_direct_path_to_cell(endpoint):
                self.current_lane_endpoint_cell = endpoint
                self.current_goal_cell = endpoint
                self.set_state(self.State.CLEAN_SWEEP_LANE, f"sweeping lane to {endpoint}")
                return f"cleaning_sweep_lane_to_{endpoint}"

        self.skip_cleaning_lane_cells()
        self.current_clean_goal_index += len(self.current_cleaning_lane_cells)
        return self.plan_next_cleaning_lane("skipped cleaning lane without clear straight sweep")

    def finish_current_cleaning_lane(self):
        if self.current_pose is not None:
            self.mark_cleaned_area(self.world_to_cell(self.current_pose[0], self.current_pose[1]))
        self.current_clean_goal_index += len(self.current_cleaning_lane_cells)
        self.cleaning_goals = self.build_cleaning_goals()
        return self.plan_next_cleaning_lane("finished cleaning lane")

    def skip_cleaning_lane_cells(self):
        for cell in self.current_cleaning_lane_cells:
            self.temporarily_skipped_cleaning_cells.add(cell)

    def setup_movement_log(self):
        if not self.movement_log_enabled:
            return

        configured_path = self.get_parameter('movement_log_file').value.strip()
        if configured_path:
            log_path = Path(configured_path).expanduser()
        else:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            log_path = Path.home() / ".ros" / "room_vacuum_logs" / f"room_vacuum_movement_{timestamp}.txt"

        try:
            log_path.parent.mkdir(parents=True, exist_ok=True)
            self.movement_log_file = log_path.open("w", newline="")
        except OSError as exc:
            self.movement_log_enabled = False
            self.get_logger().error(f"Could not open movement log file '{log_path}': {exc}")
            return

        self.movement_log_path = log_path
        self.movement_log_file.write("# Bumperbot room vacuum movement log\n")
        self.movement_log_file.write("# Share this whole file when debugging robot movement.\n")
        self.movement_log_writer = csv.writer(self.movement_log_file)
        self.movement_log_writer.writerow([
            "ros_time_sec",
            "event",
            "state",
            "pose_x_m",
            "pose_y_m",
            "yaw_rad",
            "cmd_linear_x_mps",
            "cmd_linear_y_mps",
            "cmd_linear_z_mps",
            "cmd_angular_x_radps",
            "cmd_angular_y_radps",
            "cmd_angular_z_radps",
            "dist_front_m",
            "dist_left_m",
            "dist_right_m",
            "target_heading_rad",
            "total_dist_traveled_m",
            "frontier_count",
            "cleaning_goal_index",
            "note",
        ])
        self.movement_log_file.flush()
        self.get_logger().info(f"Movement log file: {log_path}")

    def publish_and_log(self, cmd, dist_front=None, dist_left=None, dist_right=None, note=""):
        self.cmd_vel_pub.publish(cmd)
        self.last_cmd = cmd
        self.log_movement(
            "command",
            cmd=cmd,
            dist_front=dist_front,
            dist_left=dist_left,
            dist_right=dist_right,
            note=note,
        )

    def set_state(self, new_state, reason=""):
        if new_state == self.state:
            return

        old_state = self.state
        self.state = new_state
        if new_state == self.State.MAPPING_SCAN:
            self.reset_scan_tracking()
        self.log_movement("state_change", note=f"{old_state} -> {new_state}: {reason}")

    def log_odom_if_needed(self):
        if self.last_logged_odom_pose is None:
            self.last_logged_odom_pose = self.current_pose
            self.log_movement("odom", note="first_odom_pose")
            return

        dx = self.current_pose[0] - self.last_logged_odom_pose[0]
        dy = self.current_pose[1] - self.last_logged_odom_pose[1]
        dyaw = self.normalize_angle(self.current_pose[2] - self.last_logged_odom_pose[2])
        if math.sqrt(dx**2 + dy**2) < 0.02 and abs(dyaw) < 0.03:
            return

        self.last_logged_odom_pose = self.current_pose
        self.log_movement("odom", note="pose_changed")

    def log_movement(self, event, cmd=None, dist_front=None, dist_left=None, dist_right=None, note=""):
        if not self.movement_log_enabled or self.movement_log_writer is None:
            return

        pose = self.current_pose
        if cmd is None:
            cmd = self.last_cmd

        row = [
            self.format_log_value(self.get_clock().now().nanoseconds / 1e9),
            event,
            self.state,
            self.format_log_value(pose[0] if pose else None),
            self.format_log_value(pose[1] if pose else None),
            self.format_log_value(pose[2] if pose else None),
            self.format_log_value(cmd.linear.x),
            self.format_log_value(cmd.linear.y),
            self.format_log_value(cmd.linear.z),
            self.format_log_value(cmd.angular.x),
            self.format_log_value(cmd.angular.y),
            self.format_log_value(cmd.angular.z),
            self.format_log_value(dist_front),
            self.format_log_value(dist_left),
            self.format_log_value(dist_right),
            self.format_log_value(self.target_heading),
            self.format_log_value(self.total_dist_traveled),
            len(self.frontier_cells),
            self.current_clean_goal_index,
            note,
        ]

        try:
            self.movement_log_writer.writerow(row)
            self.movement_log_file.flush()
        except OSError as exc:
            self.movement_log_enabled = False
            self.get_logger().error(f"Movement logging disabled after write failure: {exc}")

    def format_log_value(self, value):
        if value is None:
            return ""
        if isinstance(value, float):
            if math.isfinite(value):
                return f"{value:.6f}"
            return "inf" if value > 0 else "-inf"
        return value

    def close_movement_log(self):
        if self.movement_log_file is None:
            return

        self.log_movement("node_stop", note="Room vacuum controller stopped.")
        self.movement_log_file.close()
        self.movement_log_file = None
        self.movement_log_writer = None

    def destroy_node(self):
        self.close_movement_log()
        if getattr(self, 'enable_viz', False):
            try:
                pygame.quit()
            except Exception:
                pass
        super().destroy_node()

    def get_min_dist(self, msg, start, end):
        min_d = float('inf')
        for i, d in enumerate(msg.ranges):
            a = self.normalize_angle(msg.angle_min + i*msg.angle_increment - math.pi)
            if start <= a <= end and self.is_valid_scan_distance(msg, d):
                min_d = min(min_d, d)
        return min_d

    def is_front_blocked(self, dist_front, dist_front_left, dist_front_right):
        if dist_front < self.obstacle_stop_dist:
            return True
        if dist_front_left < self.side_obstacle_stop_dist:
            return True
        if dist_front_right < self.side_obstacle_stop_dist:
            return True
        return False

    def limit_speed_near_obstacle(self, cmd, dist_front, dist_front_left, dist_front_right):
        closest_front = min(dist_front, dist_front_left, dist_front_right)
        if cmd.linear.x <= 0.0 or not math.isfinite(closest_front):
            return
        if closest_front < self.slow_down_dist:
            cmd.linear.x = min(cmd.linear.x, self.forward_speed * 0.45)

    def reset_scan_tracking(self):
        self.scan_start_yaw = None
        self.scan_last_yaw = None
        self.scan_accumulated_yaw = 0.0

    def update_occupancy_map(self, msg):
        if self.current_pose is None:
            return

        robot_x, robot_y, robot_yaw = self.current_pose
        robot_cell = self.world_to_cell(robot_x, robot_y)
        self.mark_free_cell(robot_cell)

        max_clear_range = self.get_mapping_clear_range(msg)
        obstacle_lock_range = self.permanent_point_proximity
        
        # Clear current scan obstacles at start of new scan
        self.current_scan_obstacle_cells = set()
        
        for i in range(0, len(msg.ranges), self.map_scan_stride):
            distance = msg.ranges[i]
            hit_wall = self.is_valid_scan_distance(msg, distance) and distance <= self.map_lidar_range
            ray_distance = distance if hit_wall else max_clear_range

            relative_angle = self.normalize_angle(msg.angle_min + i * msg.angle_increment - math.pi)
            world_angle = robot_yaw + relative_angle
            end_x = robot_x + ray_distance * math.cos(world_angle)
            end_y = robot_y + ray_distance * math.sin(world_angle)
            ray_cells = self.bresenham_cells(robot_cell, self.world_to_cell(end_x, end_y))
            if not ray_cells:
                continue

            # Full rays may clear tentative cells; close-confirmed obstacle cells stay locked.
            free_cells = ray_cells[:-1] if hit_wall else ray_cells
            for cell in free_cells:
                self.mark_free_cell(cell)
            if hit_wall:
                obstacle_cell = ray_cells[-1]
                # Track this cell as part of current scan for green point visualization
                self.current_scan_obstacle_cells.add(obstacle_cell)
                
                close_obstacle_evidence = self.is_cell_close_to_pose(
                    obstacle_cell,
                    robot_x,
                    robot_y,
                    obstacle_lock_range,
                )
                # Distant/unsure hits are marked temporary (purple). Purples are fixed now (persist
                # until close range data resolves them to blue or removes if proven absent).
                self.mark_occupied_cell(
                    obstacle_cell,
                    temporary=True,
                    lock=close_obstacle_evidence,
                )

        # After rays, resolve close purples (promote if hit close; close free handled in mark_free).
        self.promote_close_temporary_obstacles(robot_x, robot_y, obstacle_lock_range)

        # For any remaining close purple, use lidar direction sampling to see if we can
        # prove it (lock to blue) or disprove (remove). This is "blue point sees nothing/something".
        for cell in list(self.temporary_obstacle_counts.keys()):
            if self.is_cell_close_to_pose(cell, robot_x, robot_y, obstacle_lock_range * 1.2):
                if cell in self.current_scan_obstacle_cells:
                    continue  # already handled
                if self._current_lidar_sees_wall_at(cell, msg, robot_x, robot_y, robot_yaw):
                    self.lock_obstacle_cell(cell)
                elif self._current_lidar_sees_clear_at(cell, msg, robot_x, robot_y, robot_yaw):
                    self.temporary_obstacle_counts.pop(cell, None)
                    self.occupied_counts.pop(cell, None)
                    self.mark_map_changed()

    def get_mapping_clear_range(self, msg):
        if math.isfinite(msg.range_max):
            return min(self.map_lidar_range, max(msg.range_min, msg.range_max - 0.05))
        return self.map_lidar_range

    def update_latest_lidar_range(self, msg):
        if math.isfinite(msg.range_max):
            self.latest_lidar_range = msg.range_max

    def is_valid_scan_distance(self, msg, distance):
        # Trust all finite readings (including distant). Distant hits create
        # "unsure" temporary/purple wall points (fixed/persistent) that the robot
        # treats as walls. They are only resolved (blue or discarded) when close.
        if not math.isfinite(distance):
            return False
        return True
    
    def is_close_scan_distance(self, msg, distance):
        # Close readings are authoritative and never filtered
        if not math.isfinite(distance):
            return False
        if distance > self.wall_precision_threshold:
            return False
        return True

    def track_visited_wall_points(self, msg):
        if self.current_pose is None:
            return
        x, y, yaw = self.current_pose
        for i, r in enumerate(msg.ranges):
            if not self.is_close_scan_distance(msg, r):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            lidar_angle = self.normalize_angle(angle - math.pi)
            rel_x = r * math.cos(lidar_angle)
            rel_y = r * math.sin(lidar_angle)
            world_x = x + rel_x * math.cos(yaw) - rel_y * math.sin(yaw)
            world_y = y + rel_x * math.sin(yaw) + rel_y * math.cos(yaw)
            grid_x = round(world_x * 10) / 10
            grid_y = round(world_y * 10) / 10
            self.wall_points_visited.add((grid_x, grid_y))

    def all_walls_visited(self, msg):
        if self.current_pose is None:
            return False
        current_wall_points = set()
        x, y, yaw = self.current_pose
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r > msg.range_max - 0.1:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            lidar_angle = self.normalize_angle(angle - math.pi)
            rel_x = r * math.cos(lidar_angle)
            rel_y = r * math.sin(lidar_angle)
            world_x = x + rel_x * math.cos(yaw) - rel_y * math.sin(yaw)
            world_y = y + rel_x * math.sin(yaw) + rel_y * math.cos(yaw)
            grid_x = round(world_x * 10) / 10
            grid_y = round(world_y * 10) / 10
            current_wall_points.add((grid_x, grid_y))
        if len(current_wall_points) == 0:
            return True
        for point in current_wall_points:
            found = False
            for visited in self.wall_points_visited:
                dist = math.sqrt((point[0] - visited[0])**2 + (point[1] - visited[1])**2)
                if dist < self.wall_precision_threshold * 2:
                    found = True
                    break
            if not found:
                return False
        return True

    def inspect_temporary_walls(self, msg, cmd, remaining_temps):
        if self.wall_inspection_skipped_map_version != self.map_version:
            self.wall_inspection_skipped_cells.clear()
            self.wall_inspection_skipped_map_version = self.map_version
        self.wall_inspection_skipped_cells.intersection_update(remaining_temps)
        components = self.find_temporary_wall_components(remaining_temps)

        if self.current_wall_component:
            refreshed_component = self.find_matching_wall_component(components, self.current_wall_component)
            if refreshed_component:
                self.current_wall_component = refreshed_component
            else:
                completed_size = len(self.current_wall_component)
                self.get_logger().info(f"Wall cluster scanned; switching target ({completed_size} cells resolved).")
                self.log_movement("wall_component_complete", note=f"{completed_size} cells resolved")
                self.clear_wall_inspection_target()

        if not self.current_wall_component:
            selectable_components = [
                component for component in components
                if not component.intersection(self.wall_inspection_skipped_cells)
            ]

            if not self.select_next_wall_inspection_component(selectable_components):
                self.clear_wall_inspection_target()
                self.follow_walls_closely(msg, cmd)
                return f"wall_inspect_fallback_following (temps_left={len(remaining_temps)})"

        if not self.is_pose_near_wall_component(self.current_wall_component):
            if self.current_wall_approach_cell is None:
                if not self.plan_wall_approach_path(self.current_wall_component):
                    skipped_size = len(self.current_wall_component)
                    self.wall_inspection_skipped_cells.update(self.current_wall_component)
                    self.wall_inspection_skipped_map_version = self.map_version
                    self.get_logger().warn(f"Skipping unreachable wall cluster ({skipped_size} cells).")
                    self.clear_wall_inspection_target()
                    return f"wall_inspect_skip_unreachable_wall (temps_left={len(remaining_temps)})"

            path_reached = self.follow_current_path(cmd)
            if self.path_failed:
                skipped_size = len(self.current_wall_component)
                self.wall_inspection_skipped_cells.update(self.current_wall_component)
                self.wall_inspection_skipped_map_version = self.map_version
                self.clear_wall_inspection_target()
                return f"wall_inspect_path_failed_skip_wall ({skipped_size} cells)"

            if not path_reached and not self.is_pose_near_wall_component(self.current_wall_component):
                return (
                    f"wall_inspect_go_to_wall "
                    f"(active_wall_temps={len(self.current_wall_component)}, temps_left={len(remaining_temps)})"
                )

            self.current_path = []
            self.current_goal_cell = None
            self.current_wall_approach_cell = None

        self.follow_walls_closely(msg, cmd)
        return (
            f"wall_inspect_follow_selected_wall "
            f"(active_wall_temps={len(self.current_wall_component)}, temps_left={len(remaining_temps)})"
        )

    def clear_wall_inspection_target(self):
        self.current_wall_component = set()
        self.current_wall_approach_cell = None
        self.current_path = []
        self.current_goal_cell = None
        self.current_path_index = 0
        self.current_path_direct = False

    def find_temporary_wall_components(self, temp_cells):
        remaining = set(temp_cells)
        components = []
        while remaining:
            seed = remaining.pop()
            component = {seed}
            queue = deque([seed])
            while queue:
                cell = queue.popleft()
                for neighbor in self.nearby_wall_cells(cell, self.wall_cluster_gap_cells):
                    if neighbor not in remaining:
                        continue
                    remaining.remove(neighbor)
                    component.add(neighbor)
                    queue.append(neighbor)
            components.append(component)
        return components

    def nearby_wall_cells(self, cell, gap_cells):
        x, y = cell
        for dx in range(-gap_cells, gap_cells + 1):
            for dy in range(-gap_cells, gap_cells + 1):
                if dx == 0 and dy == 0:
                    continue
                if dx * dx + dy * dy > gap_cells * gap_cells:
                    continue
                yield (x + dx, y + dy)

    def find_matching_wall_component(self, components, previous_component):
        for component in components:
            if component.intersection(previous_component):
                return component
        for component in components:
            if self.wall_components_are_near(component, previous_component, self.wall_cluster_gap_cells):
                return component
        return None

    def wall_components_are_near(self, first_component, second_component, gap_cells):
        if len(first_component) > len(second_component):
            first_component, second_component = second_component, first_component
        second_lookup = set(second_component)
        for cell in first_component:
            for neighbor in self.nearby_wall_cells(cell, gap_cells):
                if neighbor in second_lookup:
                    return True
        return False

    def select_next_wall_inspection_component(self, components):
        if self.current_pose is None or not components:
            return False

        robot_cell = self.world_to_cell(self.current_pose[0], self.current_pose[1])
        ordered_components = sorted(
            components,
            key=lambda component: (self.min_cell_distance_to_component(robot_cell, component), -len(component)),
        )

        for component in ordered_components:
            self.current_wall_component = set(component)
            self.current_wall_approach_cell = None
            self.current_path = []
            self.current_goal_cell = None

            if self.is_pose_near_wall_component(component):
                self.get_logger().info(f"Inspecting nearby wall cluster ({len(component)} temporary cells).")
                return True

            if self.plan_wall_approach_path(component):
                self.get_logger().info(
                    f"Inspecting wall cluster ({len(component)} temporary cells) via {self.current_wall_approach_cell}."
                )
                return True

            self.wall_inspection_skipped_cells.update(component)
            self.wall_inspection_skipped_map_version = self.map_version

        self.clear_wall_inspection_target()
        return False

    def plan_wall_approach_path(self, component):
        goal = self.find_wall_approach_cell(component)
        if goal is None:
            return False

        self.current_path = []
        self.current_goal_cell = None
        if not (self.plan_direct_path_to_cell(goal) or self.plan_path_to_cell(goal)):
            return False

        self.current_wall_approach_cell = goal
        return True

    def find_wall_approach_cell(self, component):
        if self.current_pose is None:
            return None

        reachable = self.find_reachable_cells()
        if not reachable:
            return None

        robot_cell = self.world_to_cell(self.current_pose[0], self.current_pose[1])
        candidates = set()
        for wall_cell in component:
            for dx in range(-self.wall_approach_max_cells, self.wall_approach_max_cells + 1):
                for dy in range(-self.wall_approach_max_cells, self.wall_approach_max_cells + 1):
                    if dx == 0 and dy == 0:
                        continue
                    distance_cells = math.sqrt(dx * dx + dy * dy)
                    if distance_cells < self.wall_approach_min_cells:
                        continue
                    if distance_cells > self.wall_approach_max_cells:
                        continue
                    candidate = (wall_cell[0] + dx, wall_cell[1] + dy)
                    if candidate not in reachable:
                        continue
                    if not self.is_cell_free(candidate) or self.is_cell_blocked(candidate):
                        continue
                    candidates.add(candidate)

        if not candidates:
            return None

        return min(
            candidates,
            key=lambda candidate: (
                abs(candidate[0] - robot_cell[0]) + abs(candidate[1] - robot_cell[1]),
                self.min_cell_distance_to_component(candidate, component),
            ),
        )

    def is_pose_near_wall_component(self, component):
        if self.current_pose is None or not component:
            return False

        x, y, _ = self.current_pose
        for cell in component:
            cell_x, cell_y = self.cell_to_world(cell)
            if math.sqrt((cell_x - x)**2 + (cell_y - y)**2) <= self.wall_follow_start_distance:
                return True
        return False

    def min_cell_distance_to_component(self, cell, component):
        if not component:
            return float('inf')
        return min(abs(cell[0] - other[0]) + abs(cell[1] - other[1]) for other in component)

    def follow_walls_closely(self, msg, cmd):
        closest_dir = 0
        min_dist = float('inf')
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            if r < min_dist:
                min_dist = r
                closest_dir = i
        closest_angle = msg.angle_min + closest_dir * msg.angle_increment
        lidar_angle = self.normalize_angle(closest_angle - math.pi)
        if min_dist > self.wall_precision_threshold * 2:
            cmd.linear.x = 0.0
            err = self.normalize_angle(lidar_angle - 0)
            cmd.angular.z = self.rotate_speed if err > 0 else -self.rotate_speed
            return cmd
        right_dist = self.get_min_dist(msg, -math.pi/2, -0.1)
        left_dist = self.get_min_dist(msg, 0.1, math.pi/2)
        front_dist = self.get_min_dist(msg, -0.1, 0.1)
        if right_dist < left_dist:
            target_dist = self.wall_precision_threshold
            error = right_dist - target_dist
            cmd.linear.x = min(self.forward_speed * 0.5, max(0.05, self.forward_speed * 0.5))
            cmd.angular.z = -error * 2.0
        elif left_dist < right_dist:
            target_dist = self.wall_precision_threshold
            error = left_dist - target_dist
            cmd.linear.x = min(self.forward_speed * 0.5, max(0.05, self.forward_speed * 0.5))
            cmd.angular.z = error * 2.0
        else:
            if front_dist > self.wall_precision_threshold:
                cmd.linear.x = self.forward_speed * 0.3
                cmd.angular.z = 0.0
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = self.rotate_speed * self.wall_follow_direction
        return cmd

    def mark_map_changed(self):
        self.map_version += 1

    def promote_close_temporary_obstacles(self, robot_x, robot_y, max_distance):
        """When robot gets close to a purple point, resolve it using current scan evidence.
        If this scan saw (hit) the cell close -> promote to blue (permanent).
        Otherwise leave the purple fixed (it will only be removed if close free evidence already cleared it).
        This ensures purple only becomes blue when "seen" as wall when close, or removed when close scan sees nothing.
        """
        if not self.temporary_obstacle_counts:
            return
        
        promoted_count = 0
        cells_to_promote = []
        for cell in list(self.temporary_obstacle_counts.keys()):
            if self.is_cell_close_to_pose(cell, robot_x, robot_y, max_distance):
                # Only promote if current scan explicitly hit near this cell (we "see" the wall)
                if cell in self.current_scan_obstacle_cells:
                    cells_to_promote.append(cell)
                    promoted_count += 1
                # If not hit this scan but close, we leave it as fixed purple for now
                # (close free would have removed via mark_free if it proved absent).
        
        # Promote collected cells after iteration is complete
        for cell in cells_to_promote:
            self.lock_obstacle_cell(cell)
        
        if promoted_count > 0:
            self.get_logger().debug(f"Promoted {promoted_count} temporary obstacles to permanent")

    def is_cell_close_to_pose(self, cell, x, y, max_distance):
        cell_x, cell_y = self.cell_to_world(cell)
        return math.sqrt((cell_x - x)**2 + (cell_y - y)**2) <= max_distance

    def _current_lidar_sees_wall_at(self, cell, msg, rx, ry, ryaw, tol=0.25):
        """Return True if the current lidar scan sees an obstacle near the given cell location."""
        cx, cy = self.cell_to_world(cell)
        dx, dy = cx - rx, cy - ry
        dist = math.hypot(dx, dy)
        if dist < 0.05:
            return True
        bearing = math.atan2(dy, dx)
        rel_angle = self.normalize_angle(bearing - ryaw)
        # find closest ray
        if len(msg.ranges) == 0:
            return False
        inc = msg.angle_increment if msg.angle_increment != 0 else (2*math.pi / len(msg.ranges))
        amin = msg.angle_min
        idx = int(round( (rel_angle - amin) / inc ))
        idx = max(0, min(len(msg.ranges)-1, idx))
        r = msg.ranges[idx]
        if not math.isfinite(r):
            return False
        return abs(r - dist) <= tol

    def _current_lidar_sees_clear_at(self, cell, msg, rx, ry, ryaw, tol=0.20):
        """Return True if the current lidar in direction of cell is clear (measured farther than cell)."""
        cx, cy = self.cell_to_world(cell)
        dx, dy = cx - rx, cy - ry
        dist = math.hypot(dx, dy)
        if dist < 0.05:
            return False
        bearing = math.atan2(dy, dx)
        rel_angle = self.normalize_angle(bearing - ryaw)
        if len(msg.ranges) == 0:
            return False
        inc = msg.angle_increment if msg.angle_increment != 0 else (2*math.pi / len(msg.ranges))
        amin = msg.angle_min
        idx = int(round( (rel_angle - amin) / inc ))
        idx = max(0, min(len(msg.ranges)-1, idx))
        r = msg.ranges[idx]
        if not math.isfinite(r):
            return True  # no return -> treat as clear for our purpose?
        return r > dist + tol

    def mark_free_cell(self, cell, clear_occupied=True):
        was_free = self.is_cell_free(cell)
        was_occupied = self.is_cell_occupied(cell)
        self.free_counts[cell] = self.free_counts.get(cell, 0) + 1

        if clear_occupied:
            if cell in self.temporary_obstacle_counts and cell not in self.locked_obstacle_cells:
                # Purple points are "fixed" and do not disappear from distant free rays or movement.
                # Only when the robot is close do we trust the scan to possibly override them.
                # If close + free here, it proves the old purple was wrong ("sees there is nothing").
                if self.current_pose is not None:
                    close_threshold = self.obstacle_map_update_range * 1.5
                    if self.is_cell_close_to_pose(cell, self.current_pose[0], self.current_pose[1], close_threshold):
                        self.temporary_obstacle_counts.pop(cell, None)
                        self.occupied_counts.pop(cell, None)
                # otherwise keep the purple fixed
            # Locked/permanent (blue) never cleared by free rays.

        if was_occupied != self.is_cell_occupied(cell) or (not was_free and self.is_cell_free(cell)):
            self.mark_map_changed()

    def mark_occupied_cell(self, cell, confirmed=False, temporary=False, lock=False):
        was_occupied = self.is_cell_occupied(cell)

        # Temporary branch: purple unsure points. These are now "fixed" (persist like blue)
        # until the robot gets close. When close + current scan sees the obstacle, it becomes blue.
        # When close + current scan sees free, mark_free will remove it (blue evidence proves wrong).
        if temporary and cell not in self.locked_obstacle_cells:
            count = self.temporary_obstacle_counts.get(cell, 0) + 1
            if confirmed:
                count = max(count, self.occupied_confirmations)
            self.temporary_obstacle_counts[cell] = min(count, self.max_occupancy_count)
            # Also record in occupied_counts so map saver includes unsure walls
            self.occupied_counts[cell] = min(count, self.max_occupancy_count)
            if not was_occupied and self.is_cell_occupied(cell):
                self.mark_map_changed()
            if lock:
                self.lock_obstacle_cell(cell)
            return

        if cell in self.temporary_obstacle_counts:
            del self.temporary_obstacle_counts[cell]

        count = self.occupied_counts.get(cell, 0) + 1
        if confirmed:
            count = max(count, self.occupied_confirmations)
        self.occupied_counts[cell] = min(count, self.max_occupancy_count)
        if not was_occupied and self.is_cell_occupied(cell):
            self.mark_map_changed()
        if lock:
            self.lock_obstacle_cell(cell)

    def lock_obstacle_cell(self, cell):
        # Remove from temporary and add to locked/occupied (turns purple -> blue)
        if cell in self.temporary_obstacle_counts:
            self.temporary_obstacle_counts.pop(cell, None)
        
        # Ensure cell is in occupied_counts for map saving
        self.occupied_counts[cell] = max(
            self.occupied_counts.get(cell, 0),
            self.occupied_confirmations,
        )
        
        if cell not in self.locked_obstacle_cells:
            self.locked_obstacle_cells.add(cell)
            self.mark_map_changed()

    def world_to_map(self, x, y):
        """Convert world coordinates to map coordinates (aligned with initial pose)."""
        if self.initial_pose is None:
            return (x, y)
        ix, iy, iyaw = self.initial_pose
        # Rotate point by -initial_yaw to align with initial orientation
        cos_i = math.cos(-iyaw)
        sin_i = math.sin(-iyaw)
        map_x = (x - ix) * cos_i - (y - iy) * sin_i
        map_y = (x - ix) * sin_i + (y - iy) * cos_i
        return (map_x, map_y)

    def map_to_world(self, map_x, map_y):
        """Convert map coordinates back to world coordinates."""
        if self.initial_pose is None:
            return (map_x, map_y)
        ix, iy, iyaw = self.initial_pose
        # Rotate point by initial_yaw
        cos_i = math.cos(iyaw)
        sin_i = math.sin(iyaw)
        world_x = ix + map_x * cos_i - map_y * sin_i
        world_y = iy + map_x * sin_i + map_y * cos_i
        return (world_x, world_y)

    def world_to_cell(self, x, y):
        map_x, map_y = self.world_to_map(x, y)
        return (
            int(round(map_x / self.map_resolution)),
            int(round(map_y / self.map_resolution)),
        )

    def cell_to_world(self, cell):
        map_x = cell[0] * self.map_resolution
        map_y = cell[1] * self.map_resolution
        return self.map_to_world(map_x, map_y)

    def bresenham_cells(self, start, end):
        x0, y0 = start
        x1, y1 = end
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        cells = []

        while True:
            cells.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy

        return cells

    def is_permanent_obstacle(self, cell):
        # Only points locked after the robot got close to them are permanent/fixed walls.
        return cell in self.locked_obstacle_cells

    def is_temporary_obstacle(self, cell):
        """Unsure/tentative wall points (purple) that count as walls for navigation.
        They persist even when out of current lidar view; they only expire after not being
        re-observed (hit) when the robot gets close.
        """
        occupied = self.temporary_obstacle_counts.get(cell, 0)
        return occupied >= self.occupied_confirmations

    def is_cell_occupied(self, cell):
        """Any occupied cell (permanent or temporary/unsure purple) -- robot treats as wall."""
        return self.is_permanent_obstacle(cell) or self.is_temporary_obstacle(cell)

    def is_cell_free(self, cell):
        return self.free_counts.get(cell, 0) > 0 and not self.is_cell_occupied(cell)

    def is_cell_unknown(self, cell):
        # A cell is unknown if no free observations and no strong obstacle evidence.
        # Low-count temporary sightings (count<2, not yet purple) do not block "unknown" status.
        # This keeps exploration going while still letting confirmed unsure walls (>=2) act as barriers.
        if self.free_counts.get(cell, 0) > 0:
            return False
        if self.is_permanent_obstacle(cell) or self.is_temporary_obstacle(cell):
            return False
        return True

    def is_cell_blocked(self, cell):
        radius = self.safety_radius_cells
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if dx * dx + dy * dy > radius * radius:
                    continue
                if self.is_cell_occupied((cell[0] + dx, cell[1] + dy)):
                    return True
        return False

    def known_free_cells(self):
        return [cell for cell in self.free_counts if self.is_cell_free(cell)]

    def known_occupied_cells(self):
        """Blue = permanent walls (locked after close approach)."""
        return [cell for cell in self.occupied_counts if self.is_permanent_obstacle(cell)]

    def known_temporary_obstacle_cells(self):
        """Purple unsure points (temporary obstacles).
        Treated as walls by the robot. Survive robot movement (not cleared by free rays when a cell
        leaves the current scan). They are fixed until resolved by close-range lidar (promote to blue or removed if scan shows nothing there).
        or when promoted on close approach.
        """
        return [cell for cell in self.temporary_obstacle_counts if self.is_temporary_obstacle(cell)]

    def neighbors4(self, cell):
        x, y = cell
        return [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]

    def neighbors8(self, cell):
        x, y = cell
        return [
            (x + dx, y + dy)
            for dx in (-1, 0, 1)
            for dy in (-1, 0, 1)
            if dx != 0 or dy != 0
        ]

    def find_reachable_cells(self):
        if self.current_pose is None:
            return set()

        start = self.world_to_cell(self.current_pose[0], self.current_pose[1])
        self.mark_free_cell(start)
        queue = deque([start])
        reachable = {start}

        while queue:
            cell = queue.popleft()
            for neighbor in self.neighbors4(cell):
                if neighbor in reachable:
                    continue
                if not self.is_cell_free(neighbor):
                    continue
                if self.is_cell_blocked(neighbor) and neighbor != start:
                    continue
                reachable.add(neighbor)
                queue.append(neighbor)

        return reachable

    def find_frontiers(self):
        reachable = self.find_reachable_cells()
        frontiers = []
        for cell in reachable:
            if self.is_cell_blocked(cell):
                continue
            if self.is_rejected_goal(cell, self.rejected_frontier_cells):
                continue
            unknown_neighbors = 0
            for neighbor in self.neighbors8(cell):
                if self.is_cell_unknown(neighbor) and not self.is_cell_blocked(neighbor):
                    unknown_neighbors += 1
            if unknown_neighbors >= self.frontier_min_unknown_neighbors:
                frontiers.append(cell)
        return frontiers

    # choose_frontier_goal was inlined+extended inside plan_next_mapping_goal
    # (which now tries *multiple* candidates until a pathable green point is found,
    #  so we keep exploring/rotating at successive frontiers until none remain).

    def plan_path_to_cell(self, goal):
        self.current_path_direct = False
        if self.current_pose is None or goal is None:
            return False

        start = self.world_to_cell(self.current_pose[0], self.current_pose[1])
        self.mark_free_cell(start)
        if not self.is_cell_free(goal) or self.is_cell_blocked(goal):
            return False

        queue = deque([start])
        parents = {start: None}
        while queue:
            cell = queue.popleft()
            if cell == goal:
                break
            for neighbor in self.neighbors4(cell):
                if neighbor in parents:
                    continue
                if not self.is_cell_free(neighbor):
                    continue
                if self.is_cell_blocked(neighbor) and neighbor != goal:
                    continue
                parents[neighbor] = cell
                queue.append(neighbor)

        if goal not in parents:
            return False

        path = []
        cell = goal
        while cell is not None:
            path.append(cell)
            cell = parents[cell]
        path.reverse()

        self.current_goal_cell = goal
        self.current_path = path
        self.current_path_index = 0
        self.current_path_direct = False
        self.replan_counter = 0
        return True

    def plan_direct_path_to_cell(self, goal):
        if self.current_pose is None or goal is None:
            return False
        if not self.is_direct_path_clear(goal):
            return False

        self.current_goal_cell = goal
        self.current_path = [goal]
        self.current_path_index = 0
        self.current_path_direct = True
        self.replan_counter = 0
        return True

    def is_direct_path_clear(self, goal):
        if self.current_pose is None or goal is None:
            return False

        start = self.world_to_cell(self.current_pose[0], self.current_pose[1])
        if not self.is_cell_free(goal) or self.is_cell_blocked(goal):
            return False

        for cell in self.bresenham_cells(start, goal)[1:]:
            if not self.is_cell_free(cell):
                return False
            if self.is_cell_blocked(cell):
                return False
        return True

    def follow_current_path(self, cmd):
        if not self.current_path:
            if self.current_goal_cell is None:
                return True
            if not self.plan_path_to_cell(self.current_goal_cell):
                self.path_failed = True
                return False

        self.replan_counter += 1
        if self.current_path_direct and self.current_goal_cell is not None:
            if not self.is_direct_path_clear(self.current_goal_cell):
                self.path_failed = True
                return False
            self.replan_counter = 0
        elif self.replan_counter >= self.path_replan_period and self.current_goal_cell is not None:
            if not self.plan_path_to_cell(self.current_goal_cell):
                self.replan_counter = 0
                self.log_movement(
                    "path_replan_failed",
                    note=f"{self.current_goal_cell}: keeping current path",
                )

        while self.current_path_index < len(self.current_path):
            target_cell = self.current_path[self.current_path_index]
            if not self.reached_cell(target_cell, self.path_waypoint_dist):
                break
            self.current_path_index += 1

        if self.current_path_index >= len(self.current_path):
            return True

        target_x, target_y = self.cell_to_world(self.current_path[self.current_path_index])
        self.drive_to_world_point(cmd, target_x, target_y)
        return False

    def drive_to_world_point(self, cmd, target_x, target_y):
        dx = target_x - self.current_pose[0]
        dy = target_y - self.current_pose[1]
        self.target_heading = math.atan2(dy, dx)
        heading_error = self.normalize_angle(self.target_heading - self.current_pose[2])
        angular_gain = 1.2 if self.current_path_direct else 1.6
        heading_gate = 0.18 if self.current_path_direct else 0.55
        cmd.angular.z = self.clamp(heading_error * angular_gain, -self.max_drive_angular, self.max_drive_angular)
        if abs(heading_error) < heading_gate:
            cmd.linear.x = self.forward_speed

    def reached_cell(self, cell, tolerance):
        if cell is None or self.current_pose is None:
            return False
        x, y = self.cell_to_world(cell)
        return math.sqrt((x - self.current_pose[0])**2 + (y - self.current_pose[1])**2) <= tolerance

    def build_cleaning_goals(self):
        reachable = sorted(self.find_reachable_cells(), key=lambda cell: (cell[1], cell[0]))
        safe_cells = [
            cell for cell in reachable
            if not self.is_cell_blocked(cell)
            and not self.is_rejected_goal(cell, self.rejected_cleaning_cells)
            and cell not in self.cleaned_cells
        ]
        if not safe_cells:
            return []

        min_y = min(cell[1] for cell in safe_cells)
        max_y = max(cell[1] for cell in safe_cells)
        goals = []
        reverse = False
        safe_set = set(safe_cells)

        for y in range(min_y, max_y + 1, self.cleaning_step_cells):
            row = sorted([cell for cell in safe_set if cell[1] == y], key=lambda cell: cell[0], reverse=reverse)
            last_x = None
            for cell in row:
                if last_x is not None and abs(cell[0] - last_x) < self.cleaning_step_cells:
                    continue
                goals.append(cell)
                last_x = cell[0]
            reverse = not reverse

        return goals

    def mark_history_cleaned(self):
        for x, y in self.history:
            self.mark_cleaned_area(self.world_to_cell(x, y))

    def is_cell_cleaned(self, cell):
        return cell in self.cleaned_cells

    def mark_cleaned_area(self, center_cell):
        if center_cell is None:
            return

        added_any = False
        radius = max(0, int(round(0.5 * self.cleaning_grid_spacing / self.map_resolution)))
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if radius > 0 and dx * dx + dy * dy > radius * radius:
                    continue
                cell = (center_cell[0] + dx, center_cell[1] + dy)
                if self.is_cell_free(cell) and cell not in self.cleaned_cells:
                    self.cleaned_cells.add(cell)
                    added_any = True

        if added_any:
            self.cleaned_version += 1

    def reject_current_goal(self, rejected_cells, reason):
        if self.current_goal_cell is None:
            return

        goal = self.current_goal_cell
        self.reject_goal_cell(goal, rejected_cells, reason)
        self.current_goal_cell = None

    def reject_goal_cell(self, goal, rejected_cells, reason):
        for dx in range(-self.rejected_goal_radius_cells, self.rejected_goal_radius_cells + 1):
            for dy in range(-self.rejected_goal_radius_cells, self.rejected_goal_radius_cells + 1):
                if dx * dx + dy * dy > self.rejected_goal_radius_cells * self.rejected_goal_radius_cells:
                    continue
                rejected_cells.add((goal[0] + dx, goal[1] + dy))

        self.log_movement("goal_rejected", note=f"{goal}: {reason}")

    def is_rejected_goal(self, cell, rejected_cells):
        return cell in rejected_cells

    def mark_current_front_obstacles(self, dist_front, dist_front_left, dist_front_right):
        if self.current_pose is None:
            return

        obstacle_rays = [
            (dist_front, 0.0),
            (dist_front_left, 0.45),
            (dist_front_right, -0.45),
        ]
        marked_any = False
        for distance, relative_angle in obstacle_rays:
            if not math.isfinite(distance):
                continue
            if distance > self.slow_down_dist:
                continue
            self.mark_obstacle_at(distance, relative_angle)
            marked_any = True

        if not marked_any:
            self.mark_obstacle_at(self.obstacle_stop_dist, 0.0)

    def mark_obstacle_at(self, distance, relative_angle):
        distance = min(distance, self.obstacle_stop_dist)
        world_angle = self.current_pose[2] + relative_angle
        x = self.current_pose[0] + distance * math.cos(world_angle)
        y = self.current_pose[1] + distance * math.sin(world_angle)
        self.mark_occupied_cell(
            self.world_to_cell(x, y),
            confirmed=True,
            temporary=not self.initial_scan_complete,
            lock=True,
        )

    def remaining_cleaning_goals(self):
        return sum(1 for cell in self.cleaning_goals[self.current_clean_goal_index:] if not self.is_cell_cleaned(cell))

    def clamp(self, value, low, high):
        return max(low, min(high, value))

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

def main():
    rclpy.init()
    node = RoomVacuumController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_cmd = Twist()
        node.publish_and_log(stop_cmd, note="shutdown_stop")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
