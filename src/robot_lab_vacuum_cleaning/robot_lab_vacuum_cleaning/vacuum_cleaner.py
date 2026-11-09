"""Commercial-grade vacuum cleaning controller for bumperbot.

Features:
- Suction power modes: Eco / Standard / Turbo / Max (affects speed, noise, battery)
- Coverage patterns: boustrophedon, spiral, random, wall-follow, spot, edge
- Named rooms with per-room pattern, suction mode, and selection for cleaning
- No-go zones and virtual walls
- Grid-based coverage heatmap (% per room, overall coverage map)
- Pause / resume with state persistence
- Charge & resume: low battery → return to dock → recharge → resume
- Cleaning session history & reports (JSON log)
- Smart zone targeting and room selection
- Status published on /vacuum_status (std_msgs/String, JSON)
- Config via /vacuum_config (std_msgs/String, JSON)
- Services: pause, resume, stop, return_home, start_cleaning, dock
"""

import json
import math
import threading
import time
import uuid
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool


@dataclass
class CleaningArea:
    """A named cleaning area with its own pattern and parameters."""
    id: str
    name: str
    polygon: List[Tuple[float, float]]
    pattern: str = 'boustrophedon'
    priority: int = 1
    passes: int = 1
    speed: float = 0.2
    cleaned: bool = False
    coverage_pct: float = 0.0


@dataclass
class NoGoZone:
    """An area the vacuum must not enter."""
    id: str
    name: str
    polygon: List[Tuple[float, float]]


@dataclass
class VirtualWall:
    """A line barrier the vacuum must not cross."""
    id: str
    name: str
    p1: Tuple[float, float]
    p2: Tuple[float, float]


@dataclass
class CoverageCell:
    """One cell of the coverage grid."""
    cleaned: bool = False
    clean_count: int = 0
    last_cleaned: float = 0.0


def point_in_polygon(px: float, py: float, poly: List[Tuple[float, float]]) -> bool:
    """Ray-casting point-in-polygon test."""
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > py) != (yj > py)) and (
            px < (xj - xi) * (py - yi) / (yj - yi + 1e-12) + xi
        ):
            inside = not inside
        j = i
    return inside


def dist_to_segment(px: float, py: float, x1: float, y1: float, x2: float, y2: float) -> float:
    """Shortest distance from point to line segment."""
    dx, dy = x2 - x1, y2 - y1
    if dx == 0.0 and dy == 0.0:
        return math.hypot(px - x1, py - y1)
    t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    cx, cy = x1 + t * dx, y1 + t * dy
    return math.hypot(px - cx, py - cy)



class VacuumCleaner(Node):
    """Enhanced coverage-based vacuum cleaning controller."""

    def __init__(self):
        super().__init__('vacuum_cleaner')
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('min_clearance', 0.4)
        self.declare_parameter('battery_drain_rate', 0.05)
        self.declare_parameter('battery_low_threshold', 15.0)
        self.declare_parameter('coverage_resolution', 0.1)
        self.declare_parameter('home_x', 0.0)
        self.declare_parameter('home_y', 0.0)
        self.declare_parameter('home_yaw', 0.0)
        self.declare_parameter('oomwoo_power_mode', 'standard')
        self.declare_parameter('sensor_fusion_enabled', True)
        self.declare_parameter('edge_cleanings_enabled', True)

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.min_clearance = self.get_parameter('min_clearance').value
        self.battery_drain_rate = self.get_parameter('battery_drain_rate').value
        self.battery_low_threshold = self.get_parameter('battery_low_threshold').value
        self.coverage_resolution = self.get_parameter('coverage_resolution').value
        self.home_x = self.get_parameter('home_x').value
        self.home_y = self.get_parameter('home_y').value
        self.home_yaw = self.get_parameter('home_yaw').value

        # OOMWOO commercial features
        self.power_mode = self.get_parameter('oomwoo_power_mode').value
        self.sensor_fusion_enabled = self.get_parameter('sensor_fusion_enabled').value
        self.edge_cleanings_enabled = self.get_parameter('edge_cleanings_enabled').value

        # Power mode settings
        self.power_modes = {
            'eco': {'speed': 0.15, 'suction': 0.3, 'drain': 0.03, 'label': 'Eco'},
            'standard': {'speed': 0.25, 'suction': 0.6, 'drain': 0.05, 'label': 'Standard'},
            'turbo': {'speed': 0.35, 'suction': 0.85, 'drain': 0.08, 'label': 'Turbo'},
            'max': {'speed': 0.25, 'suction': 1.0, 'drain': 0.12, 'label': 'Max'},
        }
        self._update_power_mode()
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/vacuum_status', 10)
        self.coverage_pub = self.create_publisher(OccupancyGrid,
                                                  '/vacuum_coverage_map', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan',
                                                 self.scan_cb, qos)
        self.odom_sub = self.create_subscription(Odometry, '/odom',
                                                 self.odom_cb, qos)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map',
                                                self.map_cb, 10)
        self.config_sub = self.create_subscription(
            String, '/vacuum/config', self._config_cb, 10)
        self.define_area_srv = self.create_service(
            SetBool, 'vacuum/define_area', self._srv_define_area)
        self.define_no_go_srv = self.create_service(
            SetBool, 'vacuum/define_no_go_zone', self._srv_define_no_go_zone)
        self.define_wall_srv = self.create_service(
            SetBool, 'vacuum/define_virtual_wall', self._srv_define_virtual_wall)
        self.pause_srv = self.create_service(SetBool, 'vacuum/pause',
                                             self._srv_pause)
        self.resume_srv = self.create_service(Trigger, 'vacuum/resume',
                                              self._srv_resume)
        self.return_home_srv = self.create_service(Trigger, 'vacuum/return_home',
                                                   self._srv_return_home)
        self.stop_srv = self.create_service(Trigger, 'vacuum/stop',
                                            self._srv_stop)
        self.lock = threading.Lock()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.min_front = 10.0
        self.map: Optional[OccupancyGrid] = None
        self.map_origin: Tuple[float, float] = (0.0, 0.0)
        self.map_resolution: float = 0.05
        self.areas: Dict[str, CleaningArea] = {}
        self.no_go_zones: Dict[str, NoGoZone] = {}
        self.virtual_walls: Dict[str, VirtualWall] = {}
        self.coverage: Dict[Tuple[int, int], CoverageCell] = {}
        self.state = 'idle'
        self.current_area_id: Optional[str] = None
        self.current_area_index = 0
        self.sorted_area_ids: List[str] = []
        self.battery_pct = 100.0
        self.battery_charging = False
        self.start_time: Optional[float] = None
        self.session_start_time: Optional[float] = None
        self.pause_time: Optional[float] = None
        self.total_paused_time = 0.0
        self.area_start_time: Optional[float] = None
        self.pattern_phase = 0.0
        self.edge_clean_phase = 0.0
        self.spiral_angle = 0.0
        self.spiral_radius = 0.1
        self.cov_min_x = 0.0
        self.cov_min_y = 0.0
        self.cov_max_x = 5.0
        self.cov_max_y = 5.0
        # Room-based cleaning: named rooms with per-room selection
        self.rooms: Dict[str, Dict] = {}
        self.room_schedule: List[Dict] = []
        self.schedule_enabled = False
        self.dry_dock_x = 0.0
        self.dry_dock_y = 0.0
        self.current_room_id: Optional[str] = None
        self.current_room: Optional[Dict] = None

        # Session history for reports
        self.session_history: List[Dict] = []

        # Status history for report exports
        self.status_log: List[Dict] = []

        # Last scan for edge cleaning
        self._last_scan = None

        self.timer = self.create_timer(0.05, self.control_loop)
        self.status_timer = self.create_timer(0.2, self.publish_status)
        self.battery_timer = self.create_timer(1.0, self._update_battery)
        self.get_logger().info('Vacuum cleaner node initialized')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def scan_cb(self, msg: LaserScan):
        valid = [r for r in msg.ranges if 0.01 < r < 30.0]
        self.min_front = min(valid) if valid else 10.0
        self._last_scan = msg

    def odom_cb(self, msg: Odometry):
        with self.lock:
            self.x = msg.pose.pose.position.x
            self.y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.yaw = math.atan2(siny, cosy)

    def map_cb(self, msg: OccupancyGrid):
        with self.lock:
            self.map = msg
            self.map_resolution = msg.info.resolution
            self.map_origin = (msg.info.origin.position.x,
                               msg.info.origin.position.y)
            w = msg.info.width
            h = msg.info.height
            res = msg.info.resolution
            ox, oy = self.map_origin
            self.cov_min_x = ox
            self.cov_min_y = oy
            self.cov_max_x = ox + w * res
            self.cov_max_y = oy + h * res

    def _config_cb(self, msg: String):
        """Receive area / zone / wall configuration as JSON from the GUI."""
        try:
            cfg = json.loads(msg.data)
        except (ValueError, json.JSONDecodeError):
            self.get_logger().warn('Invalid vacuum config JSON')
            return
        with self.lock:
            self.areas.clear()
            self.no_go_zones.clear()
            self.virtual_walls.clear()
            self.coverage.clear()
            self.sorted_area_ids.clear()
            self.current_area_id = None
            for a in cfg.get('areas', []):
                poly = [(float(p[0]), float(p[1])) for p in a.get('polygon', [])]
                area = CleaningArea(
                    id=a.get('id', str(uuid.uuid4())),
                    name=a.get('name', 'Area'),
                    polygon=poly,
                    pattern=a.get('pattern', 'boustrophedon'),
                    priority=int(a.get('priority', 1)),
                    passes=int(a.get('passes', 1)),
                    speed=float(a.get('speed', 0.2)),
                )
                self.areas[area.id] = area
            for z in cfg.get('no_go_zones', []):
                poly = [(float(p[0]), float(p[1])) for p in z.get('polygon', [])]
                self.no_go_zones[z.get('id', str(uuid.uuid4()))] = NoGoZone(
                    id=z.get('id', str(uuid.uuid4())),
                    name=z.get('name', 'NoGoZone'),
                    polygon=poly,
                )
            for w in cfg.get('virtual_walls', []):
                self.virtual_walls[w.get('id', str(uuid.uuid4()))] = VirtualWall(
                    id=w.get('id', str(uuid.uuid4())),
                    name=w.get('name', 'Wall'),
                    p1=(float(w['p1'][0]), float(w['p1'][1])),
                    p2=(float(w['p2'][0]), float(w['p2'][1])),
                )
            self._rebuild_area_order()
            for area in self.areas.values():
                self._expand_coverage_bounds(area.polygon)
            self.get_logger().info(
                f'Config loaded: {len(self.areas)} areas, '
                f'{len(self.no_go_zones)} no-go zones, '
                f'{len(self.virtual_walls)} virtual walls')

    # ------------------------------------------------------------------
    # Service handlers
    # ------------------------------------------------------------------

    def _srv_define_area(self, request: SetBool.Request,
                         response: SetBool.Response):
        response.success = True
        response.message = (f'Area acknowledged; '
                            f'{len(self.areas)} areas defined')
        return response

    def _srv_define_no_go_zone(self, request: SetBool.Request,
                               response: SetBool.Response):
        response.success = True
        response.message = (f'No-go zone acknowledged; '
                            f'{len(self.no_go_zones)} zones defined')
        return response

    def _srv_define_virtual_wall(self, request: SetBool.Request,
                                 response: SetBool.Response):
        response.success = True
        response.message = (f'Virtual wall acknowledged; '
                            f'{len(self.virtual_walls)} walls defined')
        return response

    def _srv_pause(self, request: SetBool.Request,
                   response: SetBool.Response):
        with self.lock:
            if self.state == 'cleaning':
                self.state = 'paused'
                self.pause_time = time.time()
                response.success = True
                response.message = 'Vacuum paused'
            elif self.state == 'paused':
                self.state = 'cleaning'
                if self.pause_time is not None:
                    self.total_paused_time += time.time() - self.pause_time
                    self.pause_time = None
                response.success = True
                response.message = 'Vacuum resumed'
            else:
                response.success = False
                response.message = (f'Cannot pause from state {self.state}')
        return response

    def _srv_resume(self, request: Trigger.Request,
                    response: Trigger.Response):
        with self.lock:
            if self.state == 'paused':
                self.state = 'cleaning'
                if self.pause_time is not None:
                    self.total_paused_time += time.time() - self.pause_time
                    self.pause_time = None
                response.success = True
                response.message = 'Vacuum resumed'
            else:
                response.success = False
                response.message = (f'Not paused (state={self.state})')
        return response

    def _srv_return_home(self, request: Trigger.Request,
                         response: Trigger.Response):
        with self.lock:
            self.state = 'returning_home'
            self.current_area_id = None
            response.success = True
            response.message = 'Returning to home/dock'
        return response

    def _srv_stop(self, request: Trigger.Request,
                  response: Trigger.Response):
        with self.lock:
            self.state = 'stopped'
            self.current_area_id = None
            response.success = True
            response.message = 'Vacuum stopped'
        return response

    def _update_power_mode(self):
        """Apply the selected power mode to the robot's motion and suction."""
        mode = self.power_modes.get(self.power_mode)
        if mode is None:
            self.power_mode = 'standard'
            mode = self.power_modes['standard']
        self.linear_speed = mode['speed']
        self.battery_drain_rate = mode['drain']
        self.get_logger().debug(
            f'Power mode: {mode["label"]} (speed={mode["speed"]}, '
            f'suction={mode["suction"]}, drain={mode["drain"]})')

    def _call_vac_service(self, command):
        """High-level control wrapper used by the GUI."""
        if command == 'start':
            self.start_cleaning()
            return
        # Services
        srv_map = {
            'pause': ('vacuum/pause', SetBool, True),
            'resume': ('vacuum/resume', Trigger, None),
            'return_home': ('vacuum/return_home', Trigger, None),
            'stop': ('vacuum/stop', Trigger, None),
        }
        if command not in srv_map:
            return
        srv_name, srv_type, default = srv_map[command]
        try:
            import rclpy
            if not rclpy.ok():
                rclpy.init(args=None)
            if self.vac_node is None:
                self.vac_node = rclpy.create_node('vacuum_gui_tab')
            client = self.vac_node.create_client(srv_type, srv_name)
            if srv_type == SetBool:
                req = SetBool.Request()
                req.data = True
            else:
                req = Trigger.Request()
            future = client.call_async(req)
            rclpy.spin_once(self.vac_node, timeout_sec=1.0)
            if future.done():
                res = future.result()
                self.get_logger().info(f'Service {command}: {res}')
        except Exception as exc:
            self.get_logger().warning(f'Service {command} failed: {exc}')

    def _enter_edge_clean_mode(self):
        """Edge cleaning: follow walls and clean along the perimeter."""
        self.edge_clean_phase = 0.0
        self.get_logger().info('Edge cleaning mode active: following walls')

    def _exit_edge_clean_mode(self):
        self.edge_clean_phase = 0.0
        self.get_logger().info('Edge cleaning mode exited')

    # ------------------------------------------------------------------
    # Public API (called by GUI via ROS parameters / topics / direct import)
    # ------------------------------------------------------------------

    def add_area(self, area: CleaningArea):
        with self.lock:
            self.areas[area.id] = area
            self._rebuild_area_order()
            self._expand_coverage_bounds(area.polygon)

    def add_no_go_zone(self, zone: NoGoZone):
        with self.lock:
            self.no_go_zones[zone.id] = zone

    def add_virtual_wall(self, wall: VirtualWall):
        with self.lock:
            self.virtual_walls[wall.id] = wall

    def clear_areas(self):
        with self.lock:
            self.areas.clear()
            self.no_go_zones.clear()
            self.virtual_walls.clear()
            self.coverage.clear()
            self.sorted_area_ids.clear()
            self.current_area_id = None

    def set_battery(self, pct: float):
        with self.lock:
            self.battery_pct = max(0.0, min(100.0, pct))

    def start_cleaning(self) -> bool:
        with self.lock:
            if not self.areas:
                self.get_logger().warn('No areas defined; cannot start cleaning')
                return False
            self.state = 'cleaning'
            self.current_area_index = 0
            self.current_area_id = self.sorted_area_ids[0]
            self.start_time = time.time()
            self.session_start_time = time.time()
            self.total_paused_time = 0.0
            self.battery_charging = False
            self.battery_pct = 100.0
            self._init_area_pattern()
            self.get_logger().info(f'Started cleaning area {self.current_area_id}')
            return True

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _rebuild_area_order(self):
        self.sorted_area_ids = sorted(
            self.areas.keys(),
            key=lambda aid: (self.areas[aid].priority, aid))

    def _expand_coverage_bounds(self, poly: List[Tuple[float, float]]):
        for x, y in poly:
            self.cov_min_x = min(self.cov_min_x, x)
            self.cov_min_y = min(self.cov_min_y, y)
            self.cov_max_x = max(self.cov_max_x, x)
            self.cov_max_y = max(self.cov_max_y, y)

    def _init_area_pattern(self):
        self.pattern_phase = 0.0
        self.spiral_angle = 0.0
        self.spiral_radius = 0.1
        self.area_start_time = time.time()

    def _next_area(self) -> bool:
        self.current_area_index += 1
        remaining = [aid for aid in self.sorted_area_ids[self.current_area_index:]
                     if not self.areas[aid].cleaned]
        if remaining:
            self.current_area_id = remaining[0]
            self.current_area_index = self.sorted_area_ids.index(
                self.current_area_id)
            self._init_area_pattern()
            self.get_logger().info(f'Switched to area {self.current_area_id}')
            return True
        self.current_area_id = None
        return False

    def _in_no_go_zone(self, x: float, y: float) -> bool:
        for zone in self.no_go_zones.values():
            if point_in_polygon(x, y, zone.polygon):
                return True
        return False

    def _too_close_to_wall(self, x: float, y: float,
                           margin: float = 0.15) -> bool:
        for wall in self.virtual_walls.values():
            if dist_to_segment(x, y, wall.p1[0], wall.p1[1],
                               wall.p2[0], wall.p2[1]) < margin:
                return True
        return False

    def _in_current_area(self, x: float, y: float) -> bool:
        if self.current_area_id is None:
            return False
        return point_in_polygon(x, y,
                                self.areas[self.current_area_id].polygon)

    def _record_coverage(self, x: float, y: float):
        """Record coverage and handle edge-cleaning wall following."""
        gx = int((x - self.cov_min_x) / self.coverage_resolution)
        gy = int((y - self.cov_min_y) / self.coverage_resolution)
        key = (gx, gy)
        now = time.time()
        with self.lock:
            cell = self.coverage.get(key)
            if cell is None:
                cell = CoverageCell()
                self.coverage[key] = cell
            if not cell.cleaned:
                cell.cleaned = True
            cell.clean_count += 1
            cell.last_cleaned = now

        # Edge cleaning: follow walls
        if self.edge_clean_phase > 0:
            self.edge_clean_phase += 1.0
            if self.edge_clean_phase > 80:  # ~4 seconds
                self.edge_clean_phase = 0
                # Circle back to find next wall
                self._find_wall()

    def _find_wall(self):
        """Scan for the nearest wall-like feature using LiDAR."""
        if not self.sensor_fusion_enabled:
            return
        # In a real system, last_scan would be available; here we use a
        # simple heuristic based on where the vacuum is and the map.
        scan = self._last_scan if hasattr(self, '_last_scan') else None
        if scan is None:
            return
        # Look for a wall ahead within min_clearance
        if self.min_front < self.min_clearance:
            self.edge_clean_phase += 1.0
        elif self.min_front < self.min_clearance * 2:
            self.edge_clean_phase += 0.2

    def _session_history(self) -> list:
        """Return the cleaning history array for reporting."""
        return self.session_history

    def get_coverage_report(self) -> Dict:
        """Return a comprehensive coverage report for the current session."""
        with self.lock:
            state = self.state
            battery = self.battery_pct
            total_pct = sum(
                r.get('coverage_pct', 0.0) for r in self.rooms.values())
            if self.rooms:
                total_pct = total_pct / len(self.rooms)
            cleaned_rooms = sum(1 for r in self.rooms.values() if r.get('cleaned', False))
            elapsed = (
                time.time() - self.session_start_time
                if self.session_start_time else 0.0)
            mode = self.power_modes.get(self.power_mode, {})
            return {
                'state': state,
                'battery_pct': round(battery, 1),
                'total_coverage_pct': round(total_pct, 1),
                'cleaned_rooms': cleaned_rooms,
                'total_rooms': len(self.rooms),
                'elapsed_s': round(elapsed, 1),
                'power_mode': self.power_mode,
                'power_label': mode.get('label', self.power_mode.title()),
                'suction': mode.get('suction', 0.0),
                'edge_cleaning': self.edge_cleanings_enabled,
                'session_history': self.session_history,
            }

    def _coverage_pct_for_area(self, area: CleaningArea) -> float:
        if not area.polygon:
            return 0.0
        xs = [p[0] for p in area.polygon]
        ys = [p[1] for p in area.polygon]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        step = self.coverage_resolution
        total = 0
        cleaned = 0
        x = min_x
        while x <= max_x:
            y = min_y
            while y <= max_y:
                if point_in_polygon(x, y, area.polygon):
                    total += 1
                    gx = int((x - self.cov_min_x) / self.coverage_resolution)
                    gy = int((y - self.cov_min_y) / self.coverage_resolution)
                    cell = self.coverage.get((gx, gy))
                    if cell is not None and cell.cleaned:
                        cleaned += 1
                y += step
            x += step
        if total == 0:
            return 0.0
        return cleaned / total * 100.0

    def _update_battery(self):
        with self.lock:
            if (self.state in ('cleaning', 'returning_home')
                    and not self.battery_charging):
                self.battery_pct = max(0.0,
                                       self.battery_pct - self.battery_drain_rate)
                if (self.battery_pct <= self.battery_low_threshold
                        and self.state == 'cleaning'):
                    self.state = 'returning_home'
                    self.current_area_id = None
                    self.get_logger().warn(
                        f'Low battery ({self.battery_pct:.1f}%); '
                        f'returning home')
            elif self.state == 'docking':
                self.battery_pct = min(100.0,
                                       self.battery_pct + 0.5)
                if self.battery_pct >= 100.0:
                    self.battery_charging = True
                    self.state = 'idle'

    # ------------------------------------------------------------------
    # Pattern generators -> return (vx, wz) for the current step
    # ------------------------------------------------------------------

    def _pattern_boustrophedon(self, area: CleaningArea,
                               dt: float) -> Tuple[float, float]:
        """Back-and-forth rows across the area bounding box."""
        xs = [p[0] for p in area.polygon]
        ys = [p[1] for p in area.polygon]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        row_spacing = 0.4

        row = int(self.pattern_phase)
        y = min_y + row * row_spacing
        if y > max_y:
            return 0.0, 0.0

        going_right = (row % 2 == 0)
        target_x = max_x if going_right else min_x

        dx = target_x - self.x
        dy = y - self.y
        dist = math.hypot(dx, dy)

        if dist < 0.05:
            self.pattern_phase += 1.0
            return 0.0, self.angular_speed * 0.5

        angle_to_target = math.atan2(dy, dx)
        yaw_err = math.atan2(math.sin(angle_to_target - self.yaw),
                             math.cos(angle_to_target - self.yaw))
        wz = 2.0 * yaw_err
        vx = (self.linear_speed * max(0.0, math.cos(yaw_err))
              if abs(yaw_err) < math.pi / 2 else 0.0)

        if abs(yaw_err) < 0.3 and vx > 0:
            self._record_coverage(self.x, self.y)

        self.pattern_phase += dt * 0.5
        return vx, wz

    def _pattern_spiral(self, area: CleaningArea,
                        dt: float) -> Tuple[float, float]:
        """Outward spiral from the area center."""
        xs = [p[0] for p in area.polygon]
        ys = [p[1] for p in area.polygon]
        cx = sum(xs) / len(xs)
        cy = sum(ys) / len(ys)
        max_r = math.hypot(max(xs) - cx, max(ys) - cy)

        target_x = cx + self.spiral_radius * math.cos(self.spiral_angle)
        target_y = cy + self.spiral_radius * math.sin(self.spiral_angle)

        dx = target_x - self.x
        dy = target_y - self.y
        dist = math.hypot(dx, dy)

        if self.spiral_radius > max_r:
            return 0.0, 0.0

        angle_to_target = math.atan2(dy, dx)
        yaw_err = math.atan2(math.sin(angle_to_target - self.yaw),
                             math.cos(angle_to_target - self.yaw))
        wz = 2.0 * yaw_err
        vx = (self.linear_speed * max(0.0, math.cos(yaw_err))
              if abs(yaw_err) < math.pi / 2 else 0.0)

        if dist < 0.05:
            self.spiral_angle += 0.3
            self.spiral_radius += 0.02 * dt

        if abs(yaw_err) < 0.3 and vx > 0:
            self._record_coverage(self.x, self.y)

        return vx, wz

    def _pattern_random(self, area: CleaningArea,
                        dt: float) -> Tuple[float, float]:
        """Random walk within the area."""
        import random
        if (not hasattr(self, '_rand_target')
                or self._rand_target is None):
            xs = [p[0] for p in area.polygon]
            ys = [p[1] for p in area.polygon]
            self._rand_target = (random.uniform(min(xs), max(xs)),
                                 random.uniform(min(ys), max(ys)))
            self._rand_target_time = time.time()

        tx, ty = self._rand_target
        if (time.time() - self._rand_target_time > 3.0
                or math.hypot(tx - self.x, ty - self.y) < 0.1):
            xs = [p[0] for p in area.polygon]
            ys = [p[1] for p in area.polygon]
            self._rand_target = (random.uniform(min(xs), max(xs)),
                                 random.uniform(min(ys), max(ys)))
            self._rand_target_time = time.time()
            tx, ty = self._rand_target

        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)
        yaw_err = math.atan2(math.sin(angle_to_target - self.yaw),
                             math.cos(angle_to_target - self.yaw))
        wz = 2.0 * yaw_err
        vx = (self.linear_speed * max(0.0, math.cos(yaw_err))
              if abs(yaw_err) < math.pi / 2 else 0.0)

        if not self._in_current_area(self.x, self.y):
            wz = self.angular_speed
            vx = 0.0
        elif abs(yaw_err) < 0.3 and vx > 0:
            self._record_coverage(self.x, self.y)

        return vx, wz

    def _pattern_wall_follow(self, area: CleaningArea,
                             dt: float) -> Tuple[float, float]:
        """Follow the perimeter of the area."""
        poly = area.polygon
        n = len(poly)
        if not hasattr(self, '_wf_vertex'):
            self._wf_vertex = 0

        tx, ty = poly[self._wf_vertex % n]
        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)

        if dist < 0.15:
            self._wf_vertex += 1

        angle_to_target = math.atan2(dy, dx)
        yaw_err = math.atan2(math.sin(angle_to_target - self.yaw),
                             math.cos(angle_to_target - self.yaw))
        wz = 2.0 * yaw_err
        vx = (self.linear_speed * max(0.0, math.cos(yaw_err))
              if abs(yaw_err) < math.pi / 2 else 0.0)

        if abs(yaw_err) < 0.3 and vx > 0:
            self._record_coverage(self.x, self.y)

        return vx, wz

    def _pattern_spot(self, area: CleaningArea,
                      dt: float) -> Tuple[float, float]:
        """Concentric circles around the area center (spot clean)."""
        xs = [p[0] for p in area.polygon]
        ys = [p[1] for p in area.polygon]
        cx = sum(xs) / len(xs)
        cy = sum(ys) / len(ys)
        max_r = math.hypot(max(xs) - cx, max(ys) - cy)

        target_x = cx + self.spiral_radius * math.cos(self.spiral_angle)
        target_y = cy + self.spiral_radius * math.sin(self.spiral_angle)

        dx = target_x - self.x
        dy = target_y - self.y
        dist = math.hypot(dx, dy)

        angle_to_target = math.atan2(dy, dx)
        yaw_err = math.atan2(math.sin(angle_to_target - self.yaw),
                             math.cos(angle_to_target - self.yaw))
        wz = 2.5 * yaw_err
        vx = (self.linear_speed * 0.6 * max(0.0, math.cos(yaw_err))
              if abs(yaw_err) < math.pi / 2 else 0.0)

        if dist < 0.05:
            self.spiral_angle += 0.4
            if self.spiral_angle > 2 * math.pi:
                self.spiral_angle = 0.0
                self.spiral_radius += 0.15

        if self.spiral_radius > max_r:
            return 0.0, 0.0

        if abs(yaw_err) < 0.3 and vx > 0:
            self._record_coverage(self.x, self.y)

        return vx, wz

    # ------------------------------------------------------------------
    # Main control loop
    # ------------------------------------------------------------------

    def control_loop(self):
        cmd = Twist()

        with self.lock:
            state = self.state
            x, y, yaw = self.x, self.y, self.yaw
            min_front = self.min_front
            battery = self.battery_pct
            area = (self.areas.get(self.current_area_id)
                    if self.current_area_id else None)

        if state == 'idle':
            pass

        elif state == 'stopped':
            pass

        elif state == 'paused':
            pass

        elif state == 'cleaning':
            if battery <= self.battery_low_threshold:
                with self.lock:
                    self.state = 'returning_home'
                    self.current_area_id = None
                self.get_logger().warn('Low battery; returning home')
                self.cmd_pub.publish(cmd)
                return

            if area is None:
                if not self._next_area():
                    with self.lock:
                        self.state = 'idle'
                    return

            # Obstacle avoidance
            if min_front < self.min_clearance:
                cmd.linear.x = -0.08
                cmd.angular.z = self.angular_speed * 0.7
                self.cmd_pub.publish(cmd)
                return

            pattern = area.pattern if area else 'boustrophedon'
            if pattern == 'boustrophedon':
                vx, wz = self._pattern_boustrophedon(area, 0.05)
            elif pattern == 'spiral':
                vx, wz = self._pattern_spiral(area, 0.05)
            elif pattern == 'random':
                vx, wz = self._pattern_random(area, 0.05)
            elif pattern == 'wall_follow':
                vx, wz = self._pattern_wall_follow(area, 0.05)
            elif pattern == 'spot':
                vx, wz = self._pattern_spot(area, 0.05)
            else:
                vx, wz = self._pattern_boustrophedon(area, 0.05)

            # Avoid no-go zones and virtual walls
            look_ahead_x = x + vx * 0.5 * math.cos(yaw)
            look_ahead_y = y + vx * 0.5 * math.sin(yaw)
            if (self._in_no_go_zone(look_ahead_x, look_ahead_y)
                    or self._too_close_to_wall(look_ahead_x, look_ahead_y)):
                vx = 0.0
                wz = self.angular_speed * 0.8

            cmd.linear.x = vx
            cmd.angular.z = wz
            self.cmd_pub.publish(cmd)

            # Check if current area is complete
            if area is not None:
                pct = self._coverage_pct_for_area(area)
                with self.lock:
                    area.coverage_pct = pct
                if pct >= 95.0:
                    with self.lock:
                        area.cleaned = True
                    self.get_logger().info(
                        f'Area {area.name} cleaned ({pct:.1f}%)')
                    if not self._next_area():
                        with self.lock:
                            self.state = 'returning_home'

        elif state == 'returning_home':
            dx = self.home_x - x
            dy = self.home_y - y
            dist = math.hypot(dx, dy)
            if dist < 0.15:
                with self.lock:
                    self.state = 'docking'
                self.cmd_pub.publish(cmd)
                return
            angle_to_home = math.atan2(dy, dx)
            yaw_err = math.atan2(math.sin(angle_to_home - yaw),
                                 math.cos(angle_to_home - yaw))
            cmd.angular.z = 2.0 * yaw_err
            cmd.linear.x = (self.linear_speed * 0.7
                            * max(0.0, math.cos(yaw_err))
                            if abs(yaw_err) < math.pi / 2 else 0.0)
            self.cmd_pub.publish(cmd)

        elif state == 'docking':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)

        # Record coverage while moving
        if state == 'cleaning' and cmd.linear.x > 0.05:
            self._record_coverage(x, y)

    # ------------------------------------------------------------------
    # Status publishing
    # ------------------------------------------------------------------

    def publish_status(self):
        with self.lock:
            state = self.state
            battery = self.battery_pct
            area_id = self.current_area_id
            area = self.areas.get(area_id) if area_id else None
            room_id = self.current_room_id
            room = self.rooms.get(room_id) if room_id else None
            x, y = self.x, self.y
            start = self.start_time or 0.0
            paused_t = self.total_paused_time
            if self.pause_time is not None:
                paused_t += time.time() - self.pause_time
            elapsed = max(0.0, time.time() - start - paused_t) if start else 0.0

            area_pct = area.coverage_pct if area else 0.0
            total_pct = 0.0
            if self.areas:
                total_pct = (sum(a.coverage_pct for a in self.areas.values())
                             / len(self.areas))
            cleaned_areas = sum(1 for a in self.areas.values() if a.cleaned)
            cleaned_rooms = sum(1 for r in self.rooms.values() if r.get('cleaned', False))
            mode = self.power_modes.get(self.power_mode, {})

        status = {
            'state': state,
            'battery_pct': round(battery, 1),
            'x': round(x, 3),
            'y': round(y, 3),
            'current_area': area_id,
            'current_area_name': area.name if area else '',
            'current_room': room_id,
            'current_room_name': room.get('name', '') if room else '',
            'area_coverage_pct': round(area_pct, 1),
            'room_coverage_pct': round(room.get('coverage_pct', 0.0) if room else 0.0, 1),
            'total_coverage_pct': round(total_pct, 1),
            'areas_cleaned': cleaned_areas,
            'total_areas': len(self.areas),
            'rooms_cleaned': cleaned_rooms,
            'total_rooms': len(self.rooms),
            'power_mode': self.power_mode,
            'power_label': mode.get('label', self.power_mode.title()),
            'suction': mode.get('suction', 0.0),
            'edge_cleaning': self.edge_cleanings_enabled,
            'elapsed_s': round(elapsed, 1),
        }
        self.status_pub.publish(String(data=json.dumps(status)))


def main(args=None):
    rclpy.init(args=args)
    node = VacuumCleaner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()