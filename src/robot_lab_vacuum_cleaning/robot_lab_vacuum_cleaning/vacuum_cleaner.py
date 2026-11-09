"""Robot-vacuum cleaning node (experimental).

Runs the cleaning system of ``mission.py`` on a robot in any mode that
provides a 2D map and a map -> base transform: ``nav`` (static map +
localisation) or ``slam`` (live map; the mapping run explores it).  The node
drives the robot itself on ``cmd_vel`` (twist_mux navigation input), so
joystick/keyboard still override it.

Interfaces
  sub  /map                    nav_msgs/OccupancyGrid (transient local)
  sub  /scan                   sensor_msgs/LaserScan
  sub  /bumper                 ros_gz_interfaces/Contacts (optional)
  sub  /cliff/*                sensor_msgs/LaserScan, downward range (optional)
  sub  /vacuum/command         std_msgs/String JSON {"cmd": "start", ...}
  pub  /cmd_vel                geometry_msgs/Twist
  pub  /vacuum/status          std_msgs/String JSON, 2 Hz
  pub  /vacuum/layout          std_msgs/String JSON (latched): rooms, zones, settings
  pub  /vacuum/history         std_msgs/String JSON (latched): last runs
  pub  /vacuum/plan            std_msgs/String JSON: active and queued paths
  pub  /vacuum/events          std_msgs/String JSON, one per event
  pub  /vacuum/coverage        nav_msgs/OccupancyGrid, 100 = cleaned (latched)
  pub  /vacuum/rooms           nav_msgs/OccupancyGrid, value = room id (latched)
  pub  /vacuum/path            nav_msgs/Path, the path being followed
  srv  vacuum/{start,pause,resume,stop,dock,locate}  std_srvs/Trigger

Battery, dustbin and consumable wear are models (see mission.py).
"""

import json
import math
import os
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path as PathMsg
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .grid import Grid
from .mission import DEFAULT_POWER_MODES, Mission, RobotConfig, Sense
from .store import Store, schedule_due

try:
    from ros_gz_interfaces.msg import Contacts
except ImportError:  # bumper input is optional
    Contacts = None

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
ACTIVE_STATES = ('cleaning', 'mapping', 'returning', 'docking')


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def robot_config_from_profile(profile: dict) -> RobotConfig:
    """RobotConfig from a robots.yaml entry (``vacuum:`` block)."""
    drive = profile.get('drive') or {}
    vac = profile.get('vacuum') or {}
    cfg = RobotConfig()
    radius = vac.get('radius', drive.get('footprint_radius'))
    if radius:
        cfg.robot_radius = float(radius)
    for key in ('cleaning_width', 'side_brush_reach', 'charge_min', 'dustbin_m2', 'margin', 'max_turn'):
        if key in vac:
            setattr(cfg, key, float(vac[key]))
    cfg.max_speed = float(vac.get('max_speed', min(0.35, float(drive.get('max_speed', 0.3)))))
    modes = vac.get('power_modes') or profile.get('vacuum_power_modes')
    if modes:
        parsed = {}
        for m in modes:
            base = dict(DEFAULT_POWER_MODES.get(m['id'], DEFAULT_POWER_MODES['standard']))
            base.update({k: v for k, v in m.items() if k != 'id'})
            parsed[m['id']] = base
        cfg.power_modes = parsed
    return cfg


def load_profile(robot_model: str, robots_config: str) -> dict:
    path = robots_config
    if not path:
        try:
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(get_package_share_directory('robot_lab_robots'), 'config', 'robots.yaml')
        except Exception:
            return {}
    try:
        with open(path, encoding='utf-8') as handle:
            return (yaml.safe_load(handle).get('robots') or {}).get(robot_model) or {}
    except (OSError, AttributeError, yaml.YAMLError):
        return {}


class VacuumCleaner(Node):
    def __init__(self):
        super().__init__('vacuum_cleaner')
        p = self.declare_parameter
        p('robot_model', 'oomwoo')
        p('robots_config', '')
        p('map_key', '')
        p('global_frame', 'map')
        p('base_frame', 'base_footprint')
        p('map_topic', '/map')
        p('scan_topic', '/scan')
        p('cmd_vel_topic', '/cmd_vel')
        p('bumper_topic', '/bumper')
        p('cliff_topics', ['/cliff/front_left', '/cliff/front_right'])
        p('cliff_drop_m', 0.05)
        p('store_dir', '')
        p('control_rate', 20.0)
        p('battery_time_scale', -1.0)
        p('auto_start', '')          # headless: a mode, or a start command as JSON
        p('auto_start_delay', 8.0)
        p('report_path', '')         # headless: write the run record here
        p('exit_on_finish', False)
        gp = lambda name: self.get_parameter(name).value  # noqa: E731

        self.robot_model = gp('robot_model')
        profile = load_profile(self.robot_model, gp('robots_config'))
        config = robot_config_from_profile(profile)
        store = Store(Path(gp('store_dir'))) if gp('store_dir') else Store()
        self.mission = Mission(config, store, log=self._on_log,
                               map_key=gp('map_key') or 'default')
        if gp('battery_time_scale') > 0:
            self.mission.settings['battery_time_scale'] = float(gp('battery_time_scale'))
        self.global_frame = gp('global_frame')
        self.base_frame = gp('base_frame')
        self.cliff_drop = float(gp('cliff_drop_m'))

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_pub = self.create_publisher(Twist, gp('cmd_vel_topic'), 10)
        self.status_pub = self.create_publisher(String, '/vacuum/status', 10)
        self.layout_pub = self.create_publisher(String, '/vacuum/layout', LATCHED)
        self.history_pub = self.create_publisher(String, '/vacuum/history', LATCHED)
        self.plan_pub = self.create_publisher(String, '/vacuum/plan', LATCHED)
        self.events_pub = self.create_publisher(String, '/vacuum/events', 50)
        self.coverage_pub = self.create_publisher(OccupancyGrid, '/vacuum/coverage', LATCHED)
        self.rooms_pub = self.create_publisher(OccupancyGrid, '/vacuum/rooms', LATCHED)
        self.path_pub = self.create_publisher(PathMsg, '/vacuum/path', LATCHED)

        self.create_subscription(OccupancyGrid, gp('map_topic'), self._map_cb, LATCHED)
        self.create_subscription(LaserScan, gp('scan_topic'), self._scan_cb, qos_profile_sensor_data)
        self.create_subscription(String, '/vacuum/command', self._command_cb, 20)
        if Contacts is not None and gp('bumper_topic'):
            self.create_subscription(Contacts, gp('bumper_topic'), self._bumper_cb, qos_profile_sensor_data)
        self.cliff = {}
        for topic in gp('cliff_topics') or []:
            if topic:
                self.create_subscription(LaserScan, topic, lambda msg, t=topic: self._cliff_cb(t, msg),
                                         qos_profile_sensor_data)
        for name in ('start', 'pause', 'resume', 'stop', 'dock', 'locate'):
            self.create_service(Trigger, 'vacuum/' + name,
                                lambda req, res, n=name: self._trigger(n, res))

        self.scan = None
        self.scan_base = None      # (points Nx2 in base frame)
        self.laser_tf = {}
        self.bump_until = 0.0
        self.bumper_msgs = 0
        self.bumper_hits = 0
        self.scan_stamp = 0.0
        self.pose = None
        self.map_msg = None
        self.map_seen = 0.0
        self.last_map_update = 0.0
        self.pending_map = None
        self.event_count = 0
        self.was_active = False
        self.last_plan_key = None
        self.last_coverage_pub = 0.0
        self.last_layout_key = None
        self.fired = set()
        self.history_sent = -1
        self.started_at = time.monotonic()
        self.auto_started = False
        self.auto_job = None
        self.auto_cmd = None
        auto = str(gp('auto_start') or '').strip()
        if auto.startswith('{'):
            self.auto_cmd = dict(json.loads(auto), cmd='start')
        elif auto:
            self.auto_cmd = {'cmd': 'start', 'mode': auto}

        rate = max(5.0, float(gp('control_rate')))
        self.create_timer(1.0 / rate, self._control)
        self.create_timer(0.5, self._publish_status)
        self.create_timer(1.0, self._publish_slow)
        self.create_timer(20.0, self._check_schedule)
        self.get_logger().info('Vacuum cleaner ready for %s (radius %.2f m, width %.2f m)' % (
            self.robot_model, config.robot_radius, config.cleaning_width))

    # ------------------------------------------------------------------
    # Inputs
    # ------------------------------------------------------------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _map_cb(self, msg: OccupancyGrid):
        self.pending_map = msg

    def _apply_map(self):
        msg = self.pending_map
        if msg is None or time.monotonic() - self.last_map_update < 2.0:
            return
        self.pending_map = None
        self.last_map_update = time.monotonic()
        info = msg.info
        grid = Grid.from_occupancy(info.width, info.height, info.resolution,
                                   (info.origin.position.x, info.origin.position.y), msg.data)
        first = self.mission.grid is None
        self.mission.update_map(grid)
        self.map_msg = msg
        if first:
            self.get_logger().info('Map %dx%d at %.3f m' % (info.width, info.height, info.resolution))
        self._publish_rooms(force=True)
        self._publish_layout(force=True)

    def _scan_cb(self, msg: LaserScan):
        frame = msg.header.frame_id
        tf = self.laser_tf.get(frame)
        if tf is None:
            try:
                t = self.tf_buffer.lookup_transform(self.base_frame, frame, Time())
            except TransformException:
                return
            tf = (t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation))
            self.laser_tf[frame] = tf
        ranges = np.asarray(msg.ranges, dtype=float)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        ok = np.isfinite(ranges) & (ranges > max(msg.range_min, 0.02)) & (ranges < min(msg.range_max, 6.0))
        r, a = ranges[ok], angles[ok] + tf[2]
        self.scan_base = np.column_stack((tf[0] + r * np.cos(a), tf[1] + r * np.sin(a)))
        self.scan_stamp = time.monotonic()

    def _bumper_cb(self, msg):
        self.bumper_msgs += 1
        if len(msg.contacts):
            self.bumper_hits += 1
            self.bump_until = time.monotonic() + 0.3

    def _cliff_cb(self, topic, msg: LaserScan):
        vals = [r for r in msg.ranges if math.isfinite(r)]
        reading = min(vals) if vals else math.inf
        floor = self.cliff.get(topic, {}).get('floor')
        if floor is None and math.isfinite(reading):
            floor = reading  # first reading on the floor is the reference
        self.cliff[topic] = {'floor': floor, 'reading': reading if math.isfinite(reading) else None,
                             'stamp': time.monotonic(),
                             'drop': (not math.isfinite(reading)) or (floor is not None and reading > floor + self.cliff_drop)}

    def _lookup_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame, Time())
        except TransformException:
            return None
        tr = t.transform
        return (tr.translation.x, tr.translation.y, yaw_of(tr.rotation))

    def _sense(self, pose) -> Sense:
        sense = Sense()
        pts = self.scan_base
        radius = self.mission.config.robot_radius
        if pts is not None and len(pts):
            d = np.hypot(pts[:, 0], pts[:, 1])
            ang = np.arctan2(pts[:, 1], pts[:, 0])
            ahead = np.abs(ang) < math.radians(35)
            if ahead.any():
                sense.front_clear = float(d[ahead].min()) - radius
            if pose is not None:
                near = pts[d < 1.5]
                c, s = math.cos(pose[2]), math.sin(pose[2])
                sense.points = np.column_stack((pose[0] + c * near[:, 0] - s * near[:, 1],
                                                pose[1] + s * near[:, 0] + c * near[:, 1]))
        sense.bumper = time.monotonic() < self.bump_until
        sense.cliff = any(v.get('drop') for v in self.cliff.values())
        return sense

    # ------------------------------------------------------------------
    # Control
    # ------------------------------------------------------------------
    def _control(self):
        self._apply_map()
        self.pose = self._lookup_pose()
        if self.pose is None and self.mission.state in ACTIVE_STATES:
            # Lost localisation: hold still.
            self.cmd_pub.publish(Twist())
            return
        v, w = self.mission.step(self._now(), self.pose, self._sense(self.pose))
        active = self.mission.state in ACTIVE_STATES or bool(self.mission.maneuvers) or v != 0.0 or w != 0.0
        if active or self.was_active:
            cmd = Twist()
            cmd.linear.x = float(v)
            cmd.angular.z = float(w)
            self.cmd_pub.publish(cmd)
        self.was_active = active
        self._flush_events()
        self._auto()

    def _auto(self):
        if self.auto_cmd is None:
            return
        if not self.auto_started:
            ready = self.mission.grid is not None and self.pose is not None
            if ready and time.monotonic() - self.started_at > float(self.get_parameter('auto_start_delay').value):
                self.auto_started = True
                ok, message = self.mission.command(dict(self.auto_cmd, source='auto'))
                self.auto_job = self.mission.job['id'] if ok and self.mission.job else None
                if not ok:
                    self._write_report({'result': 'not started', 'message': message})
            return
        record = self.mission.last_record
        if self.auto_job and record and record.get('id') == self.auto_job and \
                self.mission.state not in ('returning', 'docking'):
            self.auto_job = None
            self._write_report(record)

    def _write_report(self, record):
        path = self.get_parameter('report_path').value
        if path:
            data = dict(record)
            data['final_state'] = self.mission.state
            data['final_pose'] = list(self.pose) if self.pose else None
            data['dock'] = self.mission.layout.get('dock')
            data['robot_model'] = self.robot_model
            data['sensors'] = (getattr(self, 'last_status', None) or {}).get('sensors')
            data['events_log'] = list(self.mission.events)
            data['rooms'] = {str(k): v for k, v in self.mission.layout['rooms'].items()}
            Path(path).parent.mkdir(parents=True, exist_ok=True)
            with open(path, 'w', encoding='utf-8') as handle:
                json.dump(data, handle, indent=1, sort_keys=True)
            self.get_logger().info('Wrote the run report to %s' % path)
        if self.get_parameter('exit_on_finish').value:
            self.cmd_pub.publish(Twist())
            raise SystemExit(0)

    def _check_schedule(self):
        now = time.localtime()
        for entry in self.mission.schedule:
            key = (entry.get('id'), time.strftime('%Y-%m-%d %H:%M', now))
            if key in self.fired or not schedule_due(entry, now):
                continue
            self.fired.add(key)
            if self.mission.state not in ('idle', 'docked', 'charging'):
                self.mission.log('warn', 'Scheduled cleaning skipped: robot is %s' % self.mission.state)
                continue
            cmd = dict(entry, cmd='start', source='schedule')
            self.mission.command(cmd)

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------
    def _command_cb(self, msg: String):
        try:
            cmd = json.loads(msg.data)
        except ValueError:
            self.mission.log('warn', 'Ignored a command that is not JSON')
            return
        if not isinstance(cmd, dict):
            return
        self.cmd_pub.publish(Twist())
        if cmd.get('cmd') == 'history':
            self.history_sent = -1
            return
        self.mission.command(cmd)
        self._publish_layout(force=True)
        self._publish_rooms(force=cmd.get('cmd') in ('segment', 'merge_rooms', 'split_room'))

    def _trigger(self, name, response):
        cmd = {'cmd': name}
        if name == 'start':
            cmd['mode'] = 'all'
        response.success, response.message = self.mission.command(cmd)
        return response

    # ------------------------------------------------------------------
    # Outputs
    # ------------------------------------------------------------------
    def _on_log(self, level, text):
        log = self.get_logger()
        {'warn': log.warn, 'error': log.error}.get(level, log.info)(text)

    def _flush_events(self):
        for event in self.mission.events:
            if event['seq'] > self.event_count:
                self.events_pub.publish(String(data=json.dumps(event)))
        self.event_count = self.mission.event_seq

    def _publish_status(self):
        status = self.mission.status()
        status['robot_model'] = self.robot_model
        status['localised'] = self.pose is not None
        status['map_ready'] = self.mission.grid is not None
        status['stamp'] = time.time()
        now = time.monotonic()
        status['sensors'] = {
            'scan_age_s': round(now - self.scan_stamp, 2) if self.scan_stamp else None,
            'bumper_messages': self.bumper_msgs,
            'bumper_contacts': self.bumper_hits,
            'cliff': {t.rsplit('/', 1)[-1]: {'floor_m': None if v['floor'] is None else round(v['floor'], 4),
                                             'reading_m': None if v['reading'] is None else round(v['reading'], 4),
                                             'drop': bool(v['drop']), 'age_s': round(now - v['stamp'], 2)}
                      for t, v in self.cliff.items()},
        }
        self.last_status = status
        self.status_pub.publish(String(data=json.dumps(status)))
        self._publish_plan()

    def _publish_slow(self):
        mission = self.mission
        if mission.grid is not None and self.map_msg is not None and mission.coverage is not None:
            if mission.state in ACTIVE_STATES or time.monotonic() - self.last_coverage_pub > 10.0:
                self.coverage_pub.publish(self._grid_msg(np.where(mission.coverage, 100, 0)))
                self.last_coverage_pub = time.monotonic()
        self._publish_layout()
        if mission.store is not None:
            history = mission.store.load_history(30)
            if len(history) != self.history_sent or (history and history[-1].get('id') != getattr(self, '_hist_last', None)):
                self.history_sent = len(history)
                self._hist_last = history[-1].get('id') if history else None
                self.history_pub.publish(String(data=json.dumps(history)))

    def _publish_layout(self, force=False):
        msg = self.mission.layout_message()
        key = json.dumps(msg, sort_keys=True)
        if force or key != self.last_layout_key:
            self.last_layout_key = key
            self.layout_pub.publish(String(data=key))

    def _publish_rooms(self, force=False):
        labels = self.mission.room_labels
        if self.map_msg is None or labels is None or labels.shape != self.mission.grid.data.shape:
            return
        if force:
            self.rooms_pub.publish(self._grid_msg(np.clip(labels, 0, 100)))

    def _publish_plan(self):
        mission = self.mission
        active = mission.active
        key = (id(active), len(mission.tasks), mission.state)
        if key == self.last_plan_key:
            return
        self.last_plan_key = key
        queued, budget = [], 6000
        for task in list(mission.tasks):
            pts = task.points if len(task.points) <= 2 else [task.points[0], task.points[-1]] \
                if task.kind == 'lane' else task.points[::2] + [task.points[-1]]
            budget -= len(pts)
            if budget < 0:
                break
            queued.append([[round(x, 2), round(y, 2)] for x, y in pts])
        plan = {'active': [[round(x, 2), round(y, 2)] for x, y in (active.points if active else [])],
                'active_kind': active.kind if active else '', 'queued': queued}
        self.plan_pub.publish(String(data=json.dumps(plan)))
        path = PathMsg()
        path.header.frame_id = self.global_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in (active.points if active else []):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x, ps.pose.position.y = float(x), float(y)
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.path_pub.publish(path)

    def _grid_msg(self, values: np.ndarray) -> OccupancyGrid:
        msg = OccupancyGrid()
        msg.header.frame_id = self.map_msg.header.frame_id or self.global_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info = self.map_msg.info
        msg.data = values.astype(np.int8).ravel().tolist()
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = VacuumCleaner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
