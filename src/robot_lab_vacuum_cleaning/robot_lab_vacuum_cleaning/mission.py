"""Cleaning mission: jobs, path following, obstacle handling, battery,
dock, dustbin, consumables and history (no ROS imports).

The ROS node feeds ``step()`` the robot pose in the map frame and what the
sensors report, and publishes the returned (v, w).  Time is passed in, so
the same code runs on simulation time and in unit tests.

The battery, dustbin and consumable wear are models (the simulators have no
battery); their rates come from the robot profile and ``battery_time_scale``.
"""

import math
import time
import uuid
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Deque, Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import planner, rooms
from .grid import Grid, Point
from .store import CONSUMABLES, DEFAULT_SETTINGS, Store, in_window

Pose = Tuple[float, float, float]

DEFAULT_POWER_MODES = {
    'eco': {'label': 'Eco', 'speed': 0.20, 'suction': 0.3, 'runtime_min': 180.0},
    'standard': {'label': 'Standard', 'speed': 0.25, 'suction': 0.6, 'runtime_min': 150.0},
    'turbo': {'label': 'Turbo', 'speed': 0.25, 'suction': 0.85, 'runtime_min': 110.0},
    'max': {'label': 'Max', 'speed': 0.20, 'suction': 1.0, 'runtime_min': 80.0},
}

MODES = ('all', 'rooms', 'zones', 'spot', 'edges', 'explore')
WORK_KINDS = ('lane', 'edge', 'spot')


@dataclass
class RobotConfig:
    robot_radius: float = 0.15
    cleaning_width: float = 0.26      # main brush + suction inlet sweep
    side_brush_reach: float = 0.03    # beyond the body, for edge passes
    lane_overlap: float = 0.08        # fraction of the width lanes overlap
    max_speed: float = 0.30
    max_turn: float = 1.6
    charge_min: float = 180.0         # 0-100 % on the dock
    dustbin_m2: float = 150.0         # floor area that fills the bin
    margin: float = 0.04              # path clearance beyond the radius
    power_modes: Dict[str, Dict] = field(default_factory=lambda: dict(DEFAULT_POWER_MODES))

    @property
    def lane_spacing(self) -> float:
        return self.cleaning_width * (1.0 - self.lane_overlap)


@dataclass
class Sense:
    front_clear: float = math.inf     # metres from the body edge, +-35 deg ahead
    points: Optional[np.ndarray] = None   # Nx2 obstacle points (map frame)
    bumper: bool = False
    cliff: bool = False


@dataclass
class Task:
    kind: str                 # lane, edge, spot, transit, explore, dock
    points: List[Point]
    room: int = 0
    suction: str = 'standard'
    retries: int = 0
    transits: int = 0

    @property
    def clean(self) -> bool:
        return self.kind in WORK_KINDS

    def length(self) -> float:
        return sum(math.dist(p, q) for p, q in zip(self.points, self.points[1:]))


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class PathFollower:
    """Pure pursuit with rotate-in-place, obstacle slow-down and stop."""

    def __init__(self, points: Sequence[Point], speed: float, max_turn: float,
                 lookahead: float = 0.28, goal_tolerance: float = 0.07,
                 stop_margin: float = 0.03):
        self.points = list(points)
        self.speed = speed
        self.max_turn = max_turn
        self.lookahead = lookahead
        self.goal_tolerance = goal_tolerance
        self.stop_margin = stop_margin
        self.index = 0

    def remaining(self) -> List[Point]:
        return self.points[self.index:]

    def remaining_length(self, pose: Pose) -> float:
        pts = self.points[self.index:]
        if not pts:
            return 0.0
        d = math.dist(pose[:2], pts[0])
        return d + sum(math.dist(p, q) for p, q in zip(pts, pts[1:]))

    def step(self, pose: Pose, front_clear: float) -> Tuple[float, float, str]:
        x, y, yaw = pose
        pts = self.points
        if not pts:
            return 0.0, 0.0, 'done'
        # Advance the progress index to the nearest point in a short window.
        end = min(len(pts), self.index + 12)
        best = min(range(self.index, end), key=lambda i: math.dist((x, y), pts[i]))
        self.index = best
        goal = pts[-1]
        dist_goal = math.dist((x, y), goal)
        if dist_goal < self.goal_tolerance and self.index >= len(pts) - 2:
            return 0.0, 0.0, 'done'
        target = goal
        for p in pts[self.index:]:
            if math.dist((x, y), p) >= self.lookahead:
                target = p
                break
        heading = math.atan2(target[1] - y, target[0] - x)
        err = wrap(heading - yaw)
        remaining = self.remaining_length(pose)
        # An obstacle between us and the end of the path blocks it; one just
        # past the end (a wall at the end of a lane) does not.
        if front_clear < self.stop_margin and remaining > front_clear + 0.06 and abs(err) <= 0.75:
            return 0.0, 0.0, 'blocked'
        if abs(err) > 0.75:
            w = max(-self.max_turn, min(self.max_turn, 2.2 * err))
            return 0.0, w, 'running'
        v = self.speed * max(0.25, math.cos(err))
        v = min(v, 0.6 * dist_goal + 0.04)
        if front_clear < 0.25 and remaining > front_clear + 0.06:
            v *= max(0.25, front_clear / 0.25)
        curvature = 2.0 * math.sin(err) / max(self.lookahead, math.dist((x, y), target))
        w = max(-self.max_turn, min(self.max_turn, v * curvature + 0.8 * err))
        return v, w, 'running'


class Mission:
    def __init__(self, config: RobotConfig, store: Optional[Store] = None,
                 log: Callable[[str, str], None] = None, map_key: str = 'default'):
        self.config = config
        self.store = store
        self._log = log or (lambda level, text: None)
        self.map_key = map_key
        self.events: Deque[Dict] = deque(maxlen=200)
        self.event_seq = 0
        self.settings = dict(DEFAULT_SETTINGS)
        self.schedule: List[Dict] = []
        self.consumables = {key: 0.0 for key in CONSUMABLES}
        if store is not None:
            self.settings = store.load_settings()
            self.schedule = store.load_schedule()
            self.consumables = store.load_consumables()
        # Map and planning state.
        self.grid: Optional[Grid] = None
        self.pmap: Optional[planner.PlanningMap] = None
        self.session_blocked: Optional[np.ndarray] = None
        self.coverage: Optional[np.ndarray] = None
        self.room_labels: Optional[np.ndarray] = None
        self.layout = self._empty_layout()
        self._layout_loaded = False
        # Robot and job state.
        self.state = 'idle'
        self.error = ''
        self.pose: Optional[Pose] = None
        self.battery = 100.0
        self.dustbin_m2 = 0.0
        self.job: Optional[Dict] = None
        self.tasks: Deque[Task] = deque()
        self.active: Optional[Task] = None
        self.follower: Optional[PathFollower] = None
        self.maneuvers: Deque[List[float]] = deque()   # [v, w, seconds left]
        self.paused_from = ''
        self.resume_after_dock = ''
        self.home_reason = ''
        self.dock_phase = ''
        self.target_mask: Optional[np.ndarray] = None
        self.last_now: Optional[float] = None
        self.last_paint: Optional[Point] = None
        self.watch: Optional[Tuple[float, Pose]] = None
        self.stuck_count = 0
        self.explore_blacklist: List[Point] = []
        self.distance_m = 0.0
        self.plan_points: List[Point] = []
        self.current_suction = self.settings.get('suction', 'standard')
        self.last_record: Optional[Dict] = None
        self._bin_warned = False
        self._empty_timer = 0.0

    # ------------------------------------------------------------------
    # Logging and layout
    # ------------------------------------------------------------------
    def log(self, level: str, text: str) -> None:
        self.event_seq += 1
        self.events.append({'seq': self.event_seq, 't': time.time(), 'level': level, 'text': text})
        self._log(level, text)

    @staticmethod
    def _empty_layout() -> Dict:
        return {'rooms': {}, 'room_info': {}, 'room_order': [], 'no_go': [],
                'walls': [], 'zones': [], 'dock': None, 'signature': ''}

    def keepout(self) -> planner.Keepout:
        return planner.Keepout(
            polygons=[z['polygon'] for z in self.layout.get('no_go', []) if len(z.get('polygon', [])) >= 3],
            walls=[(tuple(w['p1']), tuple(w['p2'])) for w in self.layout.get('walls', [])])

    def save_layout(self) -> None:
        if self.store is not None:
            self.store.save_layout(self.map_key, self.layout, self.room_labels)

    def set_map_key(self, key: str) -> None:
        if key != self.map_key:
            self.map_key = key
            self._layout_loaded = False
            self.layout = self._empty_layout()
            self.room_labels = None
            if self.grid is not None:
                self._load_layout_for(self.grid)

    def _load_layout_for(self, grid: Grid) -> None:
        self._layout_loaded = True
        if self.store is None:
            return
        layout, labels = self.store.load_layout(self.map_key)
        if layout is None:
            return
        self.layout = self._empty_layout()
        self.layout.update(layout)
        self.layout['rooms'] = {int(k): v for k, v in layout.get('rooms', {}).items()}
        self.layout['room_info'] = {int(k): v for k, v in layout.get('room_info', {}).items()}
        self.layout['room_order'] = [int(v) for v in layout.get('room_order', [])]
        if labels is not None and labels.shape == grid.data.shape and \
                layout.get('signature') == grid.signature():
            self.room_labels = labels.astype(np.int32)
        self.log('info', 'Loaded the saved layout for map %s' % self.map_key)

    # ------------------------------------------------------------------
    # Map
    # ------------------------------------------------------------------
    def update_map(self, grid: Grid) -> None:
        old = self.grid
        self.grid = grid
        if not self._layout_loaded:
            self._load_layout_for(grid)
        if old is not None and old.signature() != grid.signature():
            self.coverage = _regrid(old, self.coverage, grid)
            self.session_blocked = _regrid(old, self.session_blocked, grid)
            self.target_mask = _regrid(old, self.target_mask, grid)
            if self.room_labels is not None:
                self.room_labels = _regrid(old, self.room_labels, grid)
        if self.coverage is None or self.coverage.shape != grid.data.shape:
            self.coverage = np.zeros(grid.data.shape, dtype=bool)
        if self.session_blocked is None or self.session_blocked.shape != grid.data.shape:
            self.session_blocked = np.zeros(grid.data.shape, dtype=bool)
        self.rebuild_planning_map()

    def rebuild_planning_map(self) -> None:
        if self.grid is None:
            return
        self.pmap = planner.PlanningMap(self.grid, self.config.robot_radius, self.config.margin,
                                        keepout=self.keepout(), extra_blocked=self.session_blocked)

    def segment(self, door_width: float = 0.9) -> int:
        if self.grid is None:
            return 0
        old_info = dict(self.layout.get('room_info', {}))
        old_meta = dict(self.layout.get('rooms', {}))
        labels = rooms.segment_rooms(self.grid, door_width=door_width)
        info = rooms.room_info(self.grid, labels)
        carried = rooms.match_names(old_info, labels, self.grid, old_meta)
        self.room_labels = labels
        self._set_rooms(info, carried)
        self.log('info', 'Divided the map into %d rooms' % len(info))
        return len(info)

    def _set_rooms(self, info: Dict[int, Dict], meta: Dict[int, Dict]) -> None:
        names = {}
        for rid in sorted(info):
            m = dict(meta.get(rid, {}))
            m.setdefault('name', 'Room %d' % rid)
            names[rid] = m
        self.layout['rooms'] = names
        self.layout['room_info'] = info
        order = [r for r in self.layout.get('room_order', []) if r in info]
        order += [r for r in sorted(info) if r not in order]
        self.layout['room_order'] = order
        self.layout['signature'] = self.grid.signature() if self.grid else ''
        self.save_layout()

    def ensure_rooms(self) -> None:
        if self.room_labels is None or self.grid is None or \
                self.room_labels.shape != self.grid.data.shape:
            self.segment()

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------
    def command(self, cmd: Dict) -> Tuple[bool, str]:
        name = str(cmd.get('cmd', ''))
        handler = getattr(self, '_cmd_' + name, None)
        if handler is None:
            return False, 'Unknown command %r' % name
        try:
            ok, message = handler(cmd)
        except (KeyError, ValueError, TypeError, IndexError) as exc:
            ok, message = False, '%s failed: %s' % (name, exc)
        self.log('info' if ok else 'warn', message)
        return ok, message

    def _cmd_start(self, cmd):
        if self.state in ('cleaning', 'returning', 'docking', 'mapping') and not cmd.get('force'):
            return False, 'Busy (%s); stop or pause first' % self.state
        if self.grid is None or self.pose is None:
            return False, 'Waiting for the map and the robot position'
        mode = cmd.get('mode', 'all')
        if mode not in MODES:
            return False, 'Unknown cleaning mode %r' % mode
        return self.start_job(cmd)

    def _cmd_pause(self, _cmd):
        if self.state in ('cleaning', 'returning', 'docking', 'mapping'):
            self.paused_from = self.state
            self.state = 'paused'
            self._park_active()
            return True, 'Paused'
        return False, 'Nothing to pause (%s)' % self.state

    def _cmd_resume(self, _cmd):
        if self.state == 'paused':
            self.state = self.paused_from or 'cleaning'
            if self.state in ('returning', 'docking'):
                self.go_home(self.home_reason or 'resume')
            return True, 'Resumed'
        if self.state == 'error':
            self.error = ''
            self.stuck_count = 0
            if self.job is not None and (self.tasks or self.job['mode'] == 'explore'):
                self.state = 'mapping' if self.job['mode'] == 'explore' else 'cleaning'
                return True, 'Error cleared; continuing the job'
            self.state = 'idle'
            return True, 'Error cleared'
        if self.state in ('docked', 'charging') and self.job is not None and self.tasks:
            self.resume_after_dock = ''
            self.state = 'cleaning'
            return True, 'Continuing the interrupted job'
        return False, 'Nothing to resume (%s)' % self.state

    def _cmd_stop(self, _cmd):
        self._finish_job('stopped')
        self._clear_motion()
        if self.state not in ('docked', 'charging'):
            self.state = 'idle'
        return True, 'Stopped'

    def _cmd_dock(self, _cmd):
        if self.job is not None:
            self._finish_job('returned to dock')
        return self.go_home('requested')

    def _cmd_locate(self, _cmd):
        if self.state in ('idle', 'paused', 'error'):
            # A short wiggle; not on the dock, where it would undock.
            self.maneuvers.extend([[0.0, 0.8, 0.4], [0.0, -0.8, 0.8], [0.0, 0.8, 0.4]])
        return True, "I'm here"

    def _cmd_segment(self, cmd):
        if self.grid is None:
            return False, 'No map yet'
        n = self.segment(float(cmd.get('door_width', 0.9)))
        return True, 'Found %d rooms' % n

    def _cmd_rename_room(self, cmd):
        rid = int(cmd['room'])
        if rid not in self.layout['rooms']:
            return False, 'No room %d' % rid
        self.layout['rooms'][rid]['name'] = str(cmd['name'])[:40] or 'Room %d' % rid
        self.save_layout()
        return True, 'Renamed room %d to %s' % (rid, self.layout['rooms'][rid]['name'])

    def _cmd_room_settings(self, cmd):
        rid = int(cmd['room'])
        if rid not in self.layout['rooms']:
            return False, 'No room %d' % rid
        meta = self.layout['rooms'][rid]
        for key in ('suction', 'passes', 'edge'):
            if key in cmd:
                if cmd[key] is None:
                    meta.pop(key, None)
                elif key == 'suction' and cmd[key] not in self.config.power_modes:
                    return False, 'Unknown suction %r' % cmd[key]
                else:
                    meta[key] = int(cmd[key]) if key == 'passes' else cmd[key]
        self.save_layout()
        return True, 'Updated %s' % meta['name']

    def _cmd_room_order(self, cmd):
        order = [int(r) for r in cmd['order'] if int(r) in self.layout['rooms']]
        order += [r for r in self.layout['rooms'] if r not in order]
        self.layout['room_order'] = order
        self.save_layout()
        return True, 'Room order saved'

    def _cmd_merge_rooms(self, cmd):
        ids = [int(r) for r in cmd['rooms'] if int(r) in self.layout['rooms']]
        if len(ids) < 2 or self.room_labels is None:
            return False, 'Select two or more rooms to merge'
        self.room_labels, keep = rooms.merge_rooms(self.room_labels, ids)
        meta = {r: m for r, m in self.layout['rooms'].items() if r not in ids[1:]}
        self._set_rooms(rooms.room_info(self.grid, self.room_labels), meta)
        return True, 'Merged into %s' % self.layout['rooms'][keep]['name']

    def _cmd_split_room(self, cmd):
        rid = int(cmd['room'])
        if self.room_labels is None or rid not in self.layout['rooms']:
            return False, 'No room %d' % rid
        p1, p2 = tuple(cmd['line'][0]), tuple(cmd['line'][1])
        self.room_labels, ids = rooms.split_room(self.grid, self.room_labels, rid, p1, p2)
        if len(ids) < 2:
            return False, 'The line does not divide %s' % self.layout['rooms'][rid]['name']
        meta = dict(self.layout['rooms'])
        self._set_rooms(rooms.room_info(self.grid, self.room_labels), meta)
        return True, 'Split %s into %d rooms' % (meta[rid]['name'], len(ids))

    def _cmd_set_layout(self, cmd):
        """Replace the operator geometry: no_go, walls, zones, dock."""
        for key in ('no_go', 'walls', 'zones'):
            if key in cmd:
                self.layout[key] = list(cmd[key])
        if 'dock' in cmd:
            self.layout['dock'] = list(cmd['dock']) if cmd['dock'] else None
        self.save_layout()
        self.rebuild_planning_map()
        return True, 'Layout saved (%d no-go, %d walls, %d zones)' % (
            len(self.layout['no_go']), len(self.layout['walls']), len(self.layout['zones']))

    def _cmd_set_dock(self, cmd):
        pose = cmd.get('pose') or (list(self.pose) if self.pose else None)
        if not pose:
            return False, 'No pose for the dock'
        self.layout['dock'] = [float(v) for v in pose[:3]]
        self.save_layout()
        return True, 'Dock set at (%.2f, %.2f)' % (pose[0], pose[1])

    def _cmd_settings(self, cmd):
        for key, value in cmd.get('values', {}).items():
            if key in DEFAULT_SETTINGS:
                self.settings[key] = value
        if self.store is not None:
            self.store.save_settings(self.settings)
        return True, 'Settings saved'

    def _cmd_schedule(self, cmd):
        entries = []
        for e in cmd.get('entries', []):
            entry = {'id': str(e.get('id') or uuid.uuid4().hex[:8]),
                     'enabled': bool(e.get('enabled', True)),
                     'days': sorted({int(d) for d in e.get('days', range(7)) if 0 <= int(d) <= 6}),
                     'time': str(e.get('time', '09:00')),
                     'mode': e.get('mode', 'all') if e.get('mode', 'all') in ('all', 'rooms', 'edges') else 'all',
                     'rooms': [int(r) for r in e.get('rooms', [])],
                     'suction': e.get('suction', self.settings.get('suction', 'standard')),
                     'passes': int(e.get('passes', 1))}
            entries.append(entry)
        self.schedule = entries
        if self.store is not None:
            self.store.save_schedule(entries)
        return True, '%d scheduled cleanings saved' % len(entries)

    def _cmd_reset_consumable(self, cmd):
        key = cmd['name']
        if key not in CONSUMABLES:
            return False, 'Unknown part %r' % key
        self.consumables[key] = 0.0
        if self.store is not None:
            self.store.save_consumables(self.consumables)
        return True, '%s timer reset' % CONSUMABLES[key]['label']

    def _cmd_empty_bin(self, _cmd):
        self.dustbin_m2 = 0.0
        return True, 'Dustbin emptied'

    def _cmd_clear_map_edits(self, _cmd):
        self.session_blocked = np.zeros(self.grid.data.shape, dtype=bool) if self.grid else None
        self.rebuild_planning_map()
        return True, 'Obstacles remembered from earlier runs cleared'

    def _cmd_set_battery(self, cmd):
        self.battery = max(0.0, min(100.0, float(cmd['pct'])))
        return True, 'Battery model set to %.0f%%' % self.battery

    # ------------------------------------------------------------------
    # Jobs
    # ------------------------------------------------------------------
    def start_job(self, req: Dict) -> Tuple[bool, str]:
        mode = req.get('mode', 'all')
        if self.job is not None:
            self._finish_job('replaced by a new job')
        self._bin_warned = False
        suction = req.get('suction') or self.settings.get('suction', 'standard')
        if suction not in self.config.power_modes:
            suction = 'standard'
        passes = max(1, min(3, int(req.get('passes') or self.settings.get('passes', 1))))
        edge = bool(req.get('edge', self.settings.get('edge', True)))
        if self.layout.get('dock') is None and self.pose is not None and \
                self.state in ('idle', 'docked', 'charging'):
            self.layout['dock'] = list(self.pose)
            self.save_layout()
            self.log('info', 'Dock set at the start position')
        self.session_blocked[:] = False
        self.rebuild_planning_map()
        self._clear_motion()
        self.tasks.clear()
        self.target_mask = np.zeros(self.grid.data.shape, dtype=bool)
        if mode != 'explore':
            if mode in ('all', 'rooms', 'edges'):
                self.ensure_rooms()
            tasks, target = self.build_plan(req, mode, suction, passes, edge)
            if not tasks:
                return False, 'Nothing reachable to clean for this selection'
            self.tasks.extend(tasks)
            self.target_mask = target
        self.coverage[:] = False
        self.distance_m = 0.0
        self.stuck_count = 0
        self.explore_blacklist = []
        self.job = {
            'id': time.strftime('%Y%m%d-%H%M%S') + '-' + uuid.uuid4().hex[:4],
            'mode': mode, 'suction': suction, 'passes': passes, 'edge': edge,
            'rooms': [int(r) for r in req.get('rooms', [])],
            'source': req.get('source', 'app'),
            'started': time.time(), 'recharges': 0, 'bin_empties': 0, 'skipped': 0,
            'battery_start': round(self.battery, 1), 'events': 0,
            'estimate_s': round(self.estimate_seconds(), 1),
        }
        self.state = 'mapping' if mode == 'explore' else 'cleaning'
        self.current_suction = suction
        what = {'all': 'whole home', 'rooms': '%d room(s)' % len(self.job['rooms']),
                'zones': '%d zone(s)' % len(req.get('zones', [])), 'spot': 'spot',
                'edges': 'edges', 'explore': 'mapping run'}[mode]
        if mode == 'explore':
            return True, 'Started a mapping run'
        return True, 'Started %s cleaning: %d paths, about %d min' % (
            what, len(self.tasks), round(self.job['estimate_s'] / 60.0))

    def build_plan(self, req: Dict, mode: str, suction: str, passes: int, edge: bool):
        pm, grid, cfg = self.pmap, self.grid, self.config
        here = self.pose[:2]
        reach = pm.reachable_from(here)
        tasks: List[Task] = []
        target = np.zeros(grid.data.shape, dtype=bool)
        clean_r = cfg.cleaning_width / 2.0
        edge_r = cfg.robot_radius + cfg.side_brush_reach
        free = grid.free_mask()

        def add_region(region, room_id, room_suction, room_passes, room_edge, lanes=True):
            nonlocal here, target
            if region.sum() * grid.resolution ** 2 < 0.02:
                return
            if room_edge:
                for line in planner.order_polylines(planner.edge_contours(pm, region), here):
                    tasks.append(Task('edge', line, room_id, room_suction))
                    here = line[-1]
            if lanes:
                axis = planner.best_axis(grid, region, cfg.lane_spacing)
                for p in range(room_passes):
                    ax = axis if p % 2 == 0 else ('y' if axis == 'x' else 'x')
                    segs = planner.lane_segments(grid, region, cfg.lane_spacing, ax, clean_r)
                    for a, b in planner.order_segments(segs, here):
                        tasks.append(Task('lane', planner.segment_points(a, b), room_id, room_suction))
                        here = b
            reach_r = edge_r if room_edge else clean_r
            target |= planner.dilate(region, reach_r / grid.resolution) & free

        if mode in ('all', 'rooms', 'edges'):
            labels = self.room_labels
            ids = [int(r) for r in req.get('rooms', [])] if mode == 'rooms' else \
                list(self.layout.get('room_order') or sorted(self.layout['rooms']))
            if mode == 'rooms' and not ids:
                return [], target
            for rid in ids:
                meta = self.layout['rooms'].get(rid, {})
                region = pm.traversable & reach & (labels == rid)
                add_region(region, rid,
                           meta.get('suction') or suction,
                           int(meta.get('passes') or passes),
                           mode == 'edges' or bool(meta.get('edge', edge)),
                           lanes=mode != 'edges')
            if mode == 'all':
                # Reachable floor outside every room (corridor slivers).
                rest = pm.traversable & reach & (labels == 0)
                if rest.sum() * grid.resolution ** 2 > 0.3:
                    add_region(rest, 0, suction, passes, False)
        elif mode == 'zones':
            zones = req.get('zones') or self.layout.get('zones', [])
            for zone in zones:
                x0, y0, x1, y1 = zone['rect'] if isinstance(zone, dict) else zone
                poly = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
                region = pm.traversable & reach & grid.polygon_mask(poly)
                zp = int(zone.get('passes', passes)) if isinstance(zone, dict) else passes
                add_region(region, 0, suction, zp, False)
        elif mode == 'spot':
            centre = tuple(req.get('point') or here)
            side = float(req.get('side', 1.6))
            for p in range(max(passes, 2)):
                runs = planner.spot_spiral(pm, centre, side, cfg.lane_spacing)
                if p % 2 == 1:
                    runs = [list(reversed(r)) for r in reversed(runs)]
                for run in runs:
                    tasks.append(Task('spot', run, 0, suction))
            half = side / 2.0 + clean_r
            poly = [(centre[0] - half, centre[1] - half), (centre[0] + half, centre[1] - half),
                    (centre[0] + half, centre[1] + half), (centre[0] - half, centre[1] + half)]
            target = grid.polygon_mask(poly) & planner.dilate(pm.traversable & reach, clean_r / grid.resolution) & free
        return tasks, target

    def estimate_seconds(self, tasks: Optional[Sequence[Task]] = None) -> float:
        tasks = list(self.tasks if tasks is None else tasks)
        if not tasks:
            return 0.0
        speed = max(0.05, self._speed('standard') * 0.8)
        total, here = 0.0, self.pose[:2] if self.pose else tasks[0].points[0]
        for t in tasks:
            total += (math.dist(here, t.points[0]) * 1.3 + t.length()) / speed + 2.5
            here = t.points[-1]
        return total

    def _finish_job(self, result: str) -> None:
        job = self.job
        if job is None:
            return
        self.job = None
        self.tasks.clear()
        record = dict(job)
        record.update({
            'result': result,
            'ended': time.time(),
            'duration_s': round(time.time() - job['started'], 1),
            'area_m2': round(self.cleaned_area(), 2),
            'coverage_pct': round(self.coverage_pct(), 1),
            'distance_m': round(self.distance_m, 1),
            'battery_end': round(self.battery, 1),
            'map': self.map_key,
            'signature': self.grid.signature() if self.grid else '',
            'room_names': [self.layout['rooms'].get(r, {}).get('name', str(r)) for r in job['rooms']],
        })
        self.last_record = record
        if self.store is not None:
            self.store.append_history(record, self.coverage)
            self.store.save_consumables(self.consumables)
        self.log('info', 'Job %s: %s, %.1f m2 cleaned, %.0f%% of the target' % (
            record['id'], result, record['area_m2'], record['coverage_pct']))

    # ------------------------------------------------------------------
    # Home and dock
    # ------------------------------------------------------------------
    def dock_pose(self) -> Optional[Pose]:
        d = self.layout.get('dock')
        return tuple(d) if d else None

    def _home_or_idle(self, reason: str) -> None:
        ok, message = self.go_home(reason)
        self.log('info' if ok else 'warn', message)
        if not ok and self.state in ('cleaning', 'mapping'):
            self.state = 'idle'

    def go_home(self, reason: str) -> Tuple[bool, str]:
        dock = self.dock_pose()
        if dock is None or self.pose is None or self.pmap is None:
            return False, 'No dock position known'
        self._clear_motion()
        self.home_reason = reason
        pre = (dock[0] - 0.35 * math.cos(dock[2]), dock[1] - 0.35 * math.sin(dock[2]))
        if math.dist(self.pose[:2], dock[:2]) < 0.12:
            self.state = 'docking'
            self.dock_phase = 'align'
            return True, 'At the dock'
        path = self.pmap.astar(self.pose[:2], pre) or self.pmap.astar(self.pose[:2], dock[:2])
        if path is None:
            self.state = 'error'
            self.error = 'Cannot find a way back to the dock'
            return False, self.error
        self.state = 'returning'
        self._begin(Task('dock', path))
        return True, 'Returning to the dock (%s)' % reason

    # ------------------------------------------------------------------
    # Control step
    # ------------------------------------------------------------------
    def step(self, now: float, pose: Optional[Pose], sense: Sense) -> Tuple[float, float]:
        dt = 0.0 if self.last_now is None else max(0.0, min(0.5, now - self.last_now))
        self.last_now = now
        if pose is not None and self.pose is not None:
            self.distance_m += math.dist(pose[:2], self.pose[:2]) if self.state in (
                'cleaning', 'returning', 'mapping') else 0.0
        self.pose = pose
        self._update_battery(dt)
        if pose is None or self.grid is None:
            return 0.0, 0.0
        if self.maneuvers:
            m = self.maneuvers[0]
            m[2] -= dt
            if m[2] <= 0:
                self.maneuvers.popleft()
            return m[0], m[1]
        if self.state == 'cleaning':
            return self._step_cleaning(now, pose, sense, dt)
        if self.state == 'mapping':
            return self._step_mapping(now, pose, sense)
        if self.state == 'returning':
            return self._step_returning(now, pose, sense)
        if self.state == 'docking':
            return self._step_docking(pose, dt)
        if self.state == 'emptying':
            self._empty_timer -= dt
            if self._empty_timer <= 0:
                self.dustbin_m2 = 0.0
                if self.job is not None:
                    self.job['bin_empties'] += 1
                self.log('info', 'Dustbin emptied by the dock')
                self._after_dock()
        return 0.0, 0.0

    def _step_cleaning(self, now, pose, sense, dt):
        cfg = self.config
        if self.battery <= float(self.settings.get('low_battery_pct', 20.0)):
            self._park_active()
            if self.settings.get('charge_resume', True):
                self.resume_after_dock = 'charge'
                self.job['recharges'] += 1
                self._home_or_idle('low battery, will resume after charging')
            else:
                self._finish_job('low battery')
                self._home_or_idle('low battery')
            return 0.0, 0.0
        if self.dustbin_m2 >= cfg.dustbin_m2:
            if self.settings.get('auto_empty', True) and self.resume_after_dock != 'empty':
                self._park_active()
                self.resume_after_dock = 'empty'
                self._home_or_idle('dustbin full, emptying at the dock')
                return 0.0, 0.0
            if not getattr(self, '_bin_warned', False):
                self._bin_warned = True
                self.log('warn', 'Dustbin full - empty it (no auto-empty dock)')
        if self.active is None and not self._next_task(pose):
            self._finish_job('completed')
            self._home_or_idle('job complete')
            return 0.0, 0.0
        if self.active is None:
            return 0.0, 0.0
        self.current_suction = self.active.suction
        return self._follow(now, pose, sense, dt)

    def _next_task(self, pose) -> bool:
        """Make the next task (or a transit to it) active.  False when done."""
        while self.tasks:
            task = self.tasks[0]
            start = task.points[0]
            if math.dist(pose[:2], start) <= 0.2 or task.transits >= 2:
                self.tasks.popleft()
                self._begin(task)
                return True
            path = self.pmap.astar(pose[:2], start)
            if path is None:
                # Start unreachable: try a later point of the same path.
                later = [i for i, p in enumerate(task.points[1:], 1)
                         if self.pmap.is_traversable(self.grid.world_to_cell(*p))]
                path = None
                for i in later[:: max(1, len(later) // 6)] if later else []:
                    path = self.pmap.astar(pose[:2], task.points[i])
                    if path is not None:
                        task.points = task.points[i:]
                        break
                if path is None:
                    self.tasks.popleft()
                    self.job['skipped'] += 1
                    continue
            task.transits += 1
            self._begin(Task('transit', path, task.room, task.suction))
            return True
        return False

    def _begin(self, task: Task) -> None:
        self.active = task
        speed = self._speed(task.suction) * float(self.settings.get('speed_scale', 1.0))
        if task.kind == 'edge':
            speed *= 0.7
        elif task.kind in ('transit', 'dock', 'explore'):
            speed = min(self.config.max_speed, speed * 1.2)
        speed = min(speed, self.config.max_speed)
        self.follower = PathFollower(task.points, speed, self.config.max_turn,
                                     goal_tolerance=0.1 if task.kind in ('transit', 'explore') else 0.07)
        self.plan_points = list(task.points)
        self.watch = None

    def _follow(self, now, pose, sense, dt):
        task = self.active
        if sense.bumper or sense.cliff:
            self._on_blocked(pose, sense, contact=True)
            return 0.0, 0.0
        v, w, status = self.follower.step(pose, sense.front_clear)
        if status == 'done':
            self._task_done()
            return 0.0, 0.0
        if status == 'blocked':
            self._on_blocked(pose, sense, contact=False)
            return 0.0, 0.0
        if self._stuck(now, pose, v, w):
            return 0.0, 0.0
        if task.clean or self.state == 'cleaning':
            self._paint(pose, task)
            self._wear(dt, abs(v) > 0.02 or abs(w) > 0.1)
        return v, w

    def _task_done(self) -> None:
        task = self.active
        self.active = None
        self.follower = None
        if task is None:
            return
        if task.kind == 'dock':
            self.state = 'docking'
            self.dock_phase = 'align'

    def _park_active(self) -> None:
        """Put the unfinished rest of a work task back at the queue front."""
        task, follower = self.active, self.follower
        self.active = None
        self.follower = None
        self.maneuvers.clear()
        if task is not None and task.clean and follower is not None:
            rest = follower.remaining()
            if len(rest) >= 2:
                self.tasks.appendleft(Task(task.kind, rest, task.room, task.suction, task.retries))

    def _clear_motion(self) -> None:
        self.active = None
        self.follower = None
        self.maneuvers.clear()
        self.plan_points = []

    def _on_blocked(self, pose, sense, contact: bool) -> None:
        cfg = self.config
        x, y, yaw = pose
        grid = self.grid
        marks = []
        if sense.points is not None and len(sense.points):
            d = np.hypot(sense.points[:, 0] - x, sense.points[:, 1] - y)
            near = sense.points[d < cfg.robot_radius + 0.45]
            marks.extend(map(tuple, near))
        if contact or not marks:
            ahead = cfg.robot_radius + (0.06 if sense.cliff else 0.03)
            for a in np.linspace(-0.6, 0.6, 7):
                marks.append((x + ahead * math.cos(yaw + a), y + ahead * math.sin(yaw + a)))
        for mx, my in marks:
            ix, iy = grid.world_to_cell(mx, my)
            if grid.inside(ix, iy):
                self.session_blocked[iy, ix] = True
        self.rebuild_planning_map()
        if self.job is not None:
            self.job['events'] += 1
        what = 'cliff' if sense.cliff else 'bump' if sense.bumper else 'obstacle'
        self.log('info', '%s at (%.2f, %.2f); going around' % (what.capitalize(), x, y))
        # Back away first: the detour starts from where it can turn freely.
        self.maneuvers.append([-0.08, 0.0, 1.2 if contact else 0.8])
        task, follower = self.active, self.follower
        self.active = None
        self.follower = None
        if task is None:
            return
        if task.kind in ('transit', 'explore'):
            # Re-plan the transit on the next step; the work task is still queued.
            if self.tasks:
                self.tasks[0].transits = max(0, self.tasks[0].transits - 1)
                self.tasks[0].retries += 1
                if self.tasks[0].retries > 5:
                    self.tasks.popleft()
                    self.job['skipped'] += 1
            if task.kind == 'explore':
                self.explore_blacklist.append(task.points[-1])
            return
        if task.kind == 'dock':
            self.go_home(self.home_reason)
            return
        rest = follower.remaining() if follower else []
        keep = [i for i, p in enumerate(rest) if math.dist(p, (x, y)) > cfg.robot_radius + 0.25
                and self.pmap.is_traversable(grid.world_to_cell(*p))]
        task.retries += 1
        if keep and task.retries <= 5:
            self.tasks.appendleft(Task(task.kind, rest[keep[0]:], task.room, task.suction, task.retries))
        elif self.job is not None:
            self.job['skipped'] += 1

    def _stuck(self, now, pose, v, w) -> bool:
        if abs(v) < 0.02 and abs(w) < 0.1:
            self.watch = None
            return False
        if self.watch is None:
            self.watch = (now, pose)
            return False
        t0, p0 = self.watch
        if math.dist(pose[:2], p0[:2]) > 0.05 or abs(wrap(pose[2] - p0[2])) > 0.25:
            self.watch = (now, pose)
            return False
        if now - t0 < 6.0:
            return False
        self.watch = None
        self.stuck_count += 1
        if self.stuck_count > 3:
            self._park_active()
            self.paused_from = self.state
            self.state = 'error'
            self.error = 'Stuck - move the robot to open floor and resume'
            self.log('error', self.error)
            return True
        self.log('warn', 'Not moving; trying to free itself (%d/3)' % self.stuck_count)
        self.maneuvers.extend([[-0.08, 0.0, 1.5], [0.0, 1.2, 1.3]])
        if self.active is not None:
            self._on_blocked(pose, Sense(), contact=False)
        return True

    def _step_returning(self, now, pose, sense):
        if self.active is None:
            ok, _ = self.go_home(self.home_reason)
            if not ok:
                return 0.0, 0.0
            if self.state != 'returning':
                return 0.0, 0.0
        return self._follow(now, pose, sense, 0.0)

    def _step_docking(self, pose, dt):
        dock = self.dock_pose()
        x, y, yaw = pose
        if dock is None:
            self.state = 'idle'
            return 0.0, 0.0
        if self.dock_phase == 'align':
            bearing = math.atan2(dock[1] - y, dock[0] - x)
            dist = math.dist((x, y), dock[:2])
            goal = bearing if dist > 0.04 else dock[2]
            err = wrap(goal - yaw)
            if abs(err) > 0.06:
                return 0.0, max(-1.0, min(1.0, 2.0 * err))
            self.dock_phase = 'enter' if dist > 0.04 else 'final'
            return 0.0, 0.0
        if self.dock_phase == 'enter':
            dist = math.dist((x, y), dock[:2])
            bearing = math.atan2(dock[1] - y, dock[0] - x)
            err = wrap(bearing - yaw)
            if dist < 0.03 or abs(err) > 1.2:
                self.dock_phase = 'final'
                return 0.0, 0.0
            return min(0.08, 0.5 * dist + 0.02), max(-0.5, min(0.5, 1.5 * err))
        if self.dock_phase == 'final':
            err = wrap(dock[2] - yaw)
            if abs(err) > 0.06:
                return 0.0, max(-0.8, min(0.8, 2.0 * err))
            self.dock_phase = ''
            self.log('info', 'Docked')
            self._after_dock()
        return 0.0, 0.0

    def _after_dock(self) -> None:
        reason = self.resume_after_dock
        if reason == 'empty' and self.dustbin_m2 > 0.0 and self.state != 'emptying':
            self.state = 'emptying'
            self._empty_timer = 10.0
            return
        self.state = 'charging' if self.battery < 99.9 else 'docked'
        if reason == 'empty':
            self.resume_after_dock = ''
            if self.job is not None and self.tasks:
                self.state = 'cleaning'
                self.log('info', 'Resuming the job after emptying the bin')

    def _step_mapping(self, now, pose, sense):
        if self.active is None:
            target = self._choose_frontier(pose)
            if target is None:
                self._finish_job('map complete')
                self.segment()
                self._home_or_idle('mapping complete')
                return 0.0, 0.0
            self._begin(Task('explore', target))
        return self._follow(now, pose, sense, 0.0)

    def _choose_frontier(self, pose) -> Optional[List[Point]]:
        fronts = planner.frontiers(self.grid, self.pmap)
        fronts = [(p, n) for p, n in fronts
                  if all(math.dist(p, b) > 0.5 for b in self.explore_blacklist)]
        fronts.sort(key=lambda f: math.dist(pose[:2], f[0]) - 0.02 * f[1] * self.grid.resolution)
        for point, _ in fronts[:6]:
            path = self.pmap.astar(pose[:2], point)
            if path is not None and len(path) >= 2:
                return path
            self.explore_blacklist.append(point)
        return None

    # ------------------------------------------------------------------
    # Models
    # ------------------------------------------------------------------
    def _speed(self, suction: str) -> float:
        mode = self.config.power_modes.get(suction) or self.config.power_modes.get('standard', {})
        return float(mode.get('speed', 0.25))

    def _update_battery(self, dt: float) -> None:
        scale = float(self.settings.get('battery_time_scale', 1.0))
        if self.state in ('cleaning', 'mapping', 'returning', 'docking'):
            runtime = float(self.config.power_modes.get(self.current_suction, {}).get('runtime_min', 150.0))
            if self.state != 'cleaning':
                runtime *= 1.6  # suction off / low while travelling home or mapping
            self.battery = max(0.0, self.battery - dt * scale * 100.0 / (runtime * 60.0))
        elif self.state in ('charging', 'docked', 'emptying'):
            self.battery = min(100.0, self.battery + dt * scale * 100.0 / (self.config.charge_min * 60.0))
            if self.state == 'charging':
                resume = float(self.settings.get('resume_battery_pct', 80.0))
                if self.resume_after_dock == 'charge' and self.battery >= resume and \
                        self.job is not None and self.tasks:
                    if self._dnd_active():
                        return
                    self.resume_after_dock = ''
                    self.state = 'cleaning'
                    self.log('info', 'Charged to %.0f%%; resuming the job' % self.battery)
                elif self.battery >= 99.9:
                    self.state = 'docked'

    def _dnd_active(self) -> bool:
        dnd = self.settings.get('dnd') or {}
        return bool(dnd.get('enabled')) and in_window(dnd.get('start', '22:00'), dnd.get('end', '08:00'),
                                                       time.localtime())

    def _wear(self, dt: float, moving: bool) -> None:
        if not moving:
            return
        for key in self.consumables:
            self.consumables[key] += dt

    def _paint(self, pose, task: Task) -> None:
        grid = self.grid
        x, y, _ = pose
        if self.last_paint is not None and math.dist(self.last_paint, (x, y)) < grid.resolution:
            return
        self.last_paint = (x, y)
        if not task.clean:
            return
        radius = (self.config.robot_radius + self.config.side_brush_reach) if task.kind == 'edge' \
            else self.config.cleaning_width / 2.0
        r = radius / grid.resolution
        ix, iy = grid.world_to_cell(x, y)
        ri = int(math.ceil(r))
        x0, x1 = max(ix - ri, 0), min(ix + ri + 1, grid.width)
        y0, y1 = max(iy - ri, 0), min(iy + ri + 1, grid.height)
        if x0 >= x1 or y0 >= y1:
            return
        yy, xx = np.ogrid[y0:y1, x0:x1]
        disc = (xx - ix) ** 2 + (yy - iy) ** 2 <= r * r
        before = self.coverage[y0:y1, x0:x1].copy()
        self.coverage[y0:y1, x0:x1] |= disc
        fresh = int((self.coverage[y0:y1, x0:x1] & ~before).sum())
        mode = self.config.power_modes.get(task.suction, {})
        self.dustbin_m2 += fresh * grid.resolution ** 2 * (0.5 + float(mode.get('suction', 0.6)))

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------
    def cleaned_area(self) -> float:
        if self.coverage is None or self.grid is None:
            return 0.0
        return float((self.coverage & self.grid.free_mask()).sum()) * self.grid.resolution ** 2

    def coverage_pct(self) -> float:
        if self.target_mask is None or self.coverage is None or not self.target_mask.any():
            return 0.0
        return 100.0 * float((self.coverage & self.target_mask).sum()) / float(self.target_mask.sum())

    def current_room(self) -> int:
        if self.pose is None or self.room_labels is None or self.grid is None:
            return 0
        ix, iy = self.grid.world_to_cell(*self.pose[:2])
        return int(self.room_labels[iy, ix]) if self.grid.inside(ix, iy) else 0

    def status(self) -> Dict:
        cfg = self.config
        job = self.job or {}
        room = self.current_room()
        runtime = float(cfg.power_modes.get(self.current_suction, {}).get('runtime_min', 150.0))
        scale = float(self.settings.get('battery_time_scale', 1.0))
        remaining_s = self.estimate_seconds() if self.tasks else 0.0
        if self.active is not None and self.follower is not None and self.pose is not None:
            remaining_s += self.follower.remaining_length(self.pose) / max(0.05, self.follower.speed)
        return {
            'state': self.state,
            'error': self.error,
            'pose': [round(v, 3) for v in self.pose] if self.pose else None,
            'battery_pct': round(self.battery, 1),
            'battery_runtime_min': round(self.battery / 100.0 * runtime / max(scale, 1e-6), 1),
            'suction': self.current_suction,
            'suction_label': cfg.power_modes.get(self.current_suction, {}).get('label', self.current_suction),
            'job': {k: job.get(k) for k in ('id', 'mode', 'suction', 'passes', 'edge', 'rooms',
                                            'started', 'recharges', 'skipped', 'events',
                                            'estimate_s', 'source')} if job else None,
            'task': self.active.kind if self.active else '',
            'tasks_left': len(self.tasks) + (1 if self.active is not None and self.active.clean else 0),
            'remaining_s': round(remaining_s, 1),
            'elapsed_s': round(time.time() - job['started'], 1) if job else 0.0,
            'area_m2': round(self.cleaned_area(), 2),
            'coverage_pct': round(self.coverage_pct(), 1),
            'distance_m': round(self.distance_m, 1),
            'room': room,
            'room_name': self.layout['rooms'].get(room, {}).get('name', '') if room else '',
            'dustbin_pct': round(min(100.0, 100.0 * self.dustbin_m2 / cfg.dustbin_m2), 1),
            'dock': self.layout.get('dock'),
            'map': self.map_key,
            'consumables': {key: {'label': c['label'], 'life_h': c['life_h'],
                                  'used_h': round(self.consumables.get(key, 0.0) / 3600.0, 3),
                                  'left_pct': round(max(0.0, 100.0 * (1.0 - self.consumables.get(key, 0.0)
                                                                     / 3600.0 / c['life_h'])), 1)}
                            for key, c in CONSUMABLES.items()},
            'resume_after_dock': self.resume_after_dock,
            'dnd_active': self._dnd_active(),
        }

    def layout_message(self) -> Dict:
        modes = {k: {'label': v.get('label', k), 'speed': v.get('speed'), 'runtime_min': v.get('runtime_min')}
                 for k, v in self.config.power_modes.items()}
        return {
            'map': self.map_key,
            'rooms': {str(k): dict(v, **self.layout['room_info'].get(k, {}))
                      for k, v in self.layout['rooms'].items()},
            'room_order': self.layout.get('room_order', []),
            'no_go': self.layout.get('no_go', []),
            'walls': self.layout.get('walls', []),
            'zones': self.layout.get('zones', []),
            'dock': self.layout.get('dock'),
            'settings': self.settings,
            'schedule': self.schedule,
            'power_modes': modes,
            'robot': {'radius': self.config.robot_radius, 'cleaning_width': self.config.cleaning_width},
        }


def _regrid(old: Grid, array: Optional[np.ndarray], new: Grid) -> Optional[np.ndarray]:
    """Move a per-cell array from ``old``'s geometry to ``new``'s (same
    resolution; a SLAM map that grew or shifted its origin)."""
    if array is None:
        return None
    out = np.zeros(new.data.shape, dtype=array.dtype)
    if abs(old.resolution - new.resolution) > 1e-6:
        return out
    dx = int(round((old.origin[0] - new.origin[0]) / new.resolution))
    dy = int(round((old.origin[1] - new.origin[1]) / new.resolution))
    h, w = array.shape
    ys0, xs0 = max(0, dy), max(0, dx)
    ys1, xs1 = min(new.height, dy + h), min(new.width, dx + w)
    if ys1 > ys0 and xs1 > xs0:
        out[ys0:ys1, xs0:xs1] = array[ys0 - dy:ys1 - dy, xs0 - dx:xs1 - dx]
    return out
