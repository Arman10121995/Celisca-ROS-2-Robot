#!/usr/bin/env python3
"""Pick and place for a URDF arm on any backend (ROADMAP R5.7, R5.9).

The plan comes from robot_lab_mujoco.arm_pick_place: a private MuJoCo
kinematics and collision model of the robot URDF and the world's static
boxes. It executes through the shared articulation contract
(``/molar/articulation/{state,command,heartbeat,stop}``). Object poses come
from simulator truth on ``/simulation/free_bodies`` (MuJoCo, PyBullet);
there is no perception here.

**Interface**

- ``/pick_place/request`` (String JSON ``{object, support, lift, approach}``)
  starts one task. A task is only ever started by an explicit request.
- ``/pick_place/stop`` (Trigger) stops the task and the articulation.
- ``/pick_place/state`` (String JSON, 2 Hz) reports the phase, stage,
  message, the objects and supports, and the last result.

**Execution**

- Arm and finger targets go out every ``PERIOD`` s, densified so that no
  joint is asked for more than ``RATE`` rad/s (below the contract's caps).
  Each stage then waits until the measured joints settle.

**Judged by truth**

- **Grasp:** after closing, the fingers stop on the object.
- **Lift:** the object rises with the tool and keeps its offset to it.
- **Release:** after the retreat, the object rests on the selected support:
  at its top, inside its footprint, not moving.

**Mobile manipulators** (profile ``pick_place.mobile``; R5.9)

- One request fetches the object: drive to a standoff at the object's
  table, pick, back off, carry to the support, place, back off.
- **Transit legs** go through Nav2 (``navigate_to_pose``, in the ``map``
  frame) when its action server is up, i.e. in Nav mode with localization.
  The run-up point is only a hand-off: once localization (``/amcl_pose``)
  puts the base within ``NAV2_HANDOFF`` of it, the goal is cancelled and
  docking starts (DWB circled the 7 cm goal tolerance for minutes).
  Otherwise they are straight segments on ``/cmd_vel``, steered by the
  ground-truth base pose (``/odom/ground_truth``), and must keep their
  clearance to every table, or the request is refused. A request can force
  the straight legs with ``"transit": "straight"``.
- **Docking legs** are always short straight segments steered by truth: the
  final ``approach_run`` into each table, and the back-off away from it.
  The arm is planned from the same truth pose, as the objects are.
- While Nav2 carries the object, the costmap footprint is the convex hull
  of the base circle and a strip out to the held object, so the controller
  checks the arm and object too; afterwards it returns to the base circle. The receipt records, at each Nav2 arrival, the pose error
  to the goal and the localization error (``/amcl_pose`` against truth).
- Each arm phase is planned on arrival from the measured base pose.
- The base and the arm never move together. The node sends no base command
  while the articulation is active, and no arm target while the base moves.
  The articulation contract also enforces this (``base_inhibited``, and a
  hold when the base moves). The receipt records both directions.
- While carrying, the object must keep its pose relative to the base.

**Stopping and receipts**

- Stop, a stale articulation state, a lost plant or a failed check ends the
  task with Stop.
- Each task writes a JSON receipt under ``$ROBOT_LAB_RUNTIME_ROOT/pick-place/``.
"""
import json
import math
import os
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point32, Polygon, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger

PERIOD = 0.1            # s between targets while moving
RATE = 0.3              # rad/s, below the arm channels' 0.35 cap
SETTLE_TOLERANCE = 0.01  # rad
STAGE_TIMEOUT = 30.0    # s per stage
FINGER_STILL_WINDOW = 0.5   # s: fingers count as stopped when they move
FINGER_STILL_SPAN = 0.0005  # m (or rad) less than this within the window
STATE_STALE = 1.0       # s
# Base driving (mobile profiles): below TIAGo's 0.2 m/s, 0.4 rad/s limits.
DRIVE_SPEED = 0.15      # m/s
TURN_SPEED = 0.3        # rad/s
POSITION_TOLERANCE = 0.03   # m
YAW_TOLERANCE = 0.03        # rad
DRIVE_TIMEOUT = 240.0   # s per drive phase (Nav2 transit legs took 65-75 s under load)
TRACE_PERIOD = 0.5      # s between stability samples in the receipt
STILL_SPEED = 0.01      # m/s and rad/s: the base counts as stopped
CARRY_DRIFT = 0.03      # m of object motion relative to the base while carrying
NAV2_MIN_LEG = 0.5      # m: shorter transit legs stay straight (DWB circles on tiny paths)
NAV2_HANDOFF = 0.15      # m: localization this close to the run-up point hands over to docking
NAV_BASE_RADIUS = 0.3   # m, the Nav2 overlay's robot_radius (asset_pal_tiago.yaml)
MOBILE_DEFAULTS = dict(reach=0.62, max_depth=0.3, back_off=0.6, approach_run=0.6, base_radius=0.35,
                       carry_radius=0.85, place_inset=0.15)


def _now():
    return time.monotonic()


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _plain(value):
    """JSON fallback for numpy scalars and arrays in check results."""
    return value.tolist() if hasattr(value, 'tolist') else str(value)


class PickPlace(Node):
    def __init__(self):
        super().__init__('urdf_pick_place')
        for name, default in (('model_path', ''), ('articulation_config', ''), ('pick_place_config', ''),
                              ('world_path', ''), ('base_x', 0.0), ('base_y', 0.0), ('base_z', 0.0),
                              ('base_yaw', 0.0)):
            self.declare_parameter(name, default)
        self.config = json.loads(self.get_parameter('pick_place_config').value)
        articulation = json.loads(self.get_parameter('articulation_config').value)
        from robot_lab_mujoco.arm_pick_place import ArmSpec
        self.spec = ArmSpec.from_config(self.config)
        self.home = {name: float(articulation['home'][name]) for name in self.spec.arm_joints}
        # Coupled followers (the second Panda finger) are measured, not commanded.
        self.finger_masters = [n for n in self.spec.finger_joints if n in articulation['joints']]
        self.base = tuple(float(self.get_parameter(k).value) for k in ('base_x', 'base_y', 'base_z', 'base_yaw'))
        self.supports, self.object_sizes = self._world_boxes()
        self.robot_mjcf = None
        self.truth, self.truth_at = {}, -math.inf
        self.state, self.state_at = {}, -math.inf
        self.task = None
        self.last_result = None
        self.message = 'Select an object and a support, then request a pick and place.'
        self.mobile = dict(MOBILE_DEFAULTS, **self.config['mobile']) if self.config.get('mobile') else None
        self.base_pose, self.base_twist, self.base_at = None, (0.0, 0.0), -math.inf
        if self.mobile:
            self.create_subscription(Odometry, '/odom/ground_truth', self._on_odom, 10)
            self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 1)
            from nav2_msgs.action import NavigateToPose
            self.nav_action = NavigateToPose
            self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            self.footprint_pubs = [self.create_publisher(Polygon, topic, 1) for topic in
                                   ('/local_costmap/footprint', '/global_costmap/footprint')]
            self.amcl_pose, self.amcl_at = None, -math.inf
            self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, 10)
        self.create_subscription(String, '/simulation/free_bodies', self._on_truth, 10)
        self.create_subscription(String, '/molar/articulation/state', self._on_state, 10)
        self.create_subscription(String, '/pick_place/request', self._on_request, 1)
        self.command_pub = self.create_publisher(String, '/molar/articulation/command', 1)
        self.heartbeat_pub = self.create_publisher(Empty, '/molar/articulation/heartbeat', 1)
        self.state_pub = self.create_publisher(String, '/pick_place/state', 1)
        self.stop_client = self.create_client(Trigger, '/molar/articulation/stop')
        self.create_service(Trigger, '/pick_place/stop', self._on_stop)
        self.create_timer(PERIOD, self._tick)
        self.create_timer(0.5, self._publish_state)
        root = Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT', '/workspace/molar/robot_lab_runtime')) / 'pick-place'
        root.mkdir(parents=True, exist_ok=True)
        self.receipts = root

    # --- inputs ---------------------------------------------------------------
    def _world_boxes(self):
        """Supports (static boxes) and loose box sizes from the world file."""
        from robot_lab_mujoco.arm_pick_place import Box
        from robot_lab_utils.sdf_world import extract_dynamic_bodies, extract_static_shapes
        path = self.get_parameter('world_path').value
        if not path or not os.path.isfile(path):
            return {}, {}
        sizes = {body['name']: list(body['shape']['size']) for body in extract_dynamic_bodies(path)
                 if body['shape']['type'] == 'box'}
        shapes, _ = extract_static_shapes(path, lambda uri, base: '', exclude_dynamic=True)
        boxes = {}
        for shape in shapes:
            # A support is a box a robot could set things on: under 1.2 m tall,
            # not the floor (wider than 2 m).
            if shape['type'] == 'box' and max(shape['size'][:2]) < 2.0 and shape['size'][2] < 1.2:
                boxes[shape['model']] = Box(shape['model'], shape['position'], shape['orientation'], shape['size'])
        return boxes, sizes

    def _on_truth(self, message):
        self.truth = json.loads(message.data)
        self.truth_at = _now()

    def _on_odom(self, message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.base_pose = (p.x, p.y, p.z, yaw)
        t = message.twist.twist
        self.base_twist = (math.hypot(t.linear.x, t.linear.y), abs(t.angular.z))
        self.base_at = _now()

    def _on_amcl(self, message):
        p, q = message.pose.pose.position, message.pose.pose.orientation
        self.amcl_pose = (p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        self.amcl_at = _now()

    def _on_state(self, message):
        self.state = json.loads(message.data)
        self.state_at = _now()

    def measured(self):
        return {c['name']: c['measured'] for c in self.state.get('channels', []) if c.get('measured') is not None}

    def _publish_state(self):
        task = self.task or {}
        self.state_pub.publish(String(data=json.dumps(dict(
            schema='robot_lab.pick_place_state.v1', phase=task.get('phase', 'idle'), stage=task.get('stage'),
            message=self.message, objects=sorted(self.truth), supports=sorted(self.supports),
            mobile=bool(self.mobile), drive_phase=(task.get('phases') or [{}])[task.get('phase_index', 0)].get('name')
            if task else None,
            last_result=self.last_result, truth_age_s=round(_now() - self.truth_at, 2) if self.truth else None),
            default=_plain)))

    # --- requests ---------------------------------------------------------------
    def _on_stop(self, request, response):
        self._finish('stopped', 'Stopped by request')
        response.success, response.message = True, 'stopped'
        return response

    def _on_request(self, message):
        if self.task is not None:
            self.message = 'A task is already running; Stop it first.'
            return
        try:
            request = json.loads(message.data)
            self.task = self._plan(request)
        except Exception as exc:  # noqa: BLE001 - reported to the operator
            self.task = None
            self.message = 'Not started: %s' % exc
            self.last_result = dict(passed=False, error=str(exc), request=message.data)

    def _plan(self, request):
        """Validate a request and build its phases; arm phases on a mobile base are planned on arrival."""
        name, support_name = request.get('object'), request.get('support')
        if name not in self.truth or _now() - self.truth_at > STATE_STALE:
            raise ValueError('no fresh truth for object %r' % name)
        if support_name not in self.supports:
            raise ValueError('unknown support %r (have %s)' % (support_name, sorted(self.supports)))
        if not self.state.get('initialized') or self.state.get('fault') or _now() - self.state_at > STATE_STALE:
            raise ValueError('articulation not ready: %s' % self.state.get('status'))
        measured = self.measured()
        missing = [n for n in self.spec.arm_joints + self.spec.finger_joints if n not in measured]
        if missing:
            raise ValueError('no measurement for %s' % missing)
        if not self.object_sizes.get(name):
            raise ValueError('object %r is not a loose box of the world file' % name)
        lift = float(request.get('lift', 0.10))
        approach = float(request.get('approach', 0.10))
        if not 0.03 <= lift <= 0.25 or not 0.03 <= approach <= 0.25:
            raise ValueError('lift and approach must be 0.03-0.25 m')
        if self.robot_mjcf is None:
            from robot_lab_adapter.legged_training import _package_map
            from robot_lab_mujoco.mujoco_spawner import _build_mjcf_from_urdf
            urdf = Path(self.get_parameter('model_path').value).read_text()
            self.robot_mjcf = _build_mjcf_from_urdf(urdf, _package_map(urdf), robot_name='pick_place_robot')
        task = dict(phase='executing', request=request, object=name, support=support_name, lift=lift,
                    approach=approach, started=_now(), log=[], plan={}, phase_index=0,
                    object_start=list(self.truth[name][:3]),
                    interlock=dict(max_base_speed_in_arm_phases=0.0, max_base_turn_in_arm_phases=0.0,
                                   drive_ticks_with_articulation_active=0, drive_ticks=0))
        if not self.mobile:
            task['phases'] = [dict(kind='arm', name='pick_place')]
            self._plan_arm(task, task['phases'][0])
            return task
        if self.base_pose is None or _now() - self.base_at > STATE_STALE:
            raise ValueError('no fresh base pose on /odom/ground_truth')
        table = self._table_under(name)
        if table is None:
            raise ValueError('object %r is not resting on a known table' % name)
        task['table'] = table.name
        transit = request.get('transit', 'auto')
        if transit not in ('auto', 'nav2', 'straight'):
            raise ValueError('transit must be auto, nav2 or straight')
        nav2 = transit == 'nav2' or (transit == 'auto' and self.nav_client.server_is_ready())
        if transit == 'nav2' and not self.nav_client.server_is_ready():
            raise ValueError('no Nav2 navigate_to_pose server (start the robot in Nav mode)')
        task['transit'] = 'nav2' if nav2 else 'straight'
        task['plan']['transit'] = task['transit']
        # Check both stops now so that a bad request is refused before moving.
        pick_stop = self._pick_standoff(task)
        place_stop = self._place_standoff(task, pick_stop[:2])
        task['plan'].update(pick_base=[round(v, 3) for v in pick_stop], place_base=[round(v, 3) for v in place_stop[0]],
                            place_xy=[round(v, 3) for v in place_stop[1]])
        task['phases'] = [dict(kind='drive', name='to_object'), dict(kind='arm', name='pick'),
                          dict(kind='drive', name='carry', carrying=True), dict(kind='arm', name='place'),
                          dict(kind='drive', name='clear')]
        self._start_phase(task)
        return task

    def _table_under(self, name):
        from robot_lab_mujoco.arm_pick_place import box_distance
        pose = self.truth[name]
        half = float(self.object_sizes[name][2]) / 2.0
        for box in self.supports.values():
            if box_distance(box, pose) < 1e-6 and abs(pose[2] - half - box.top) < 0.02:
                return box
        return None

    def _pick_standoff(self, task):
        from robot_lab_mujoco.arm_pick_place import standoff
        table = self.supports[task['table']]
        x, y, yaw, depth = standoff(table, self.truth[task['object']][:2], self.mobile['reach'], self.base_pose[:2])
        if depth > self.mobile['max_depth']:
            raise ValueError('object %.2f m inside the table edge; out of reach (%.2f m)' % (depth, self.mobile['max_depth']))
        return x, y, yaw

    def _place_standoff(self, task, robot_xy):
        from robot_lab_mujoco.arm_pick_place import place_standoff
        return place_standoff(self.supports[task['support']], robot_xy, self.mobile['place_inset'], self.mobile['reach'])

    def _route(self, task, goal, start, back_off, clearance):
        """Waypoints start -> (back off) -> run-up point -> goal; checked against every table.

        With Nav2 transit the leg to the run-up point is Nav2's: it plans
        around the tables itself, so only the docking legs and the turn at
        the run-up point are checked here.
        """
        from robot_lab_mujoco.arm_pick_place import segment_clearance
        x, y, yaw = goal
        run = self.mobile['approach_run']
        pre = (x - run * math.cos(yaw), y - run * math.sin(yaw))
        points, waypoints = [tuple(start[:2])], []
        if back_off:
            out = (start[0] - back_off * math.cos(start[3]), start[1] - back_off * math.sin(start[3]))
            waypoints.append(dict(x=out[0], y=out[1], reverse=True))
            points.append(out)
        transit_from = points[-1]
        nav2 = task.get('transit') == 'nav2' and math.hypot(pre[0] - transit_from[0], pre[1] - transit_from[1]) >= NAV2_MIN_LEG
        waypoints += [dict(x=pre[0], y=pre[1], yaw=yaw, nav2=True) if nav2 else dict(x=pre[0], y=pre[1]),
                      dict(x=x, y=y, yaw=yaw)]
        points.append(pre)
        points.append((x, y))
        boxes = list(self.supports.values())
        # Legs between turns keep ``clearance`` (the carried arm's sweep). The
        # straight back-off and the last leg in run along the arm, which stays
        # over the table, so they keep the base's own clearance.
        legs = list(zip(points[:-1], points[1:]))
        for index, (a, b) in enumerate(legs):
            straight = index == len(legs) - 1 or (back_off and index == 0)
            if nav2 and not straight:
                continue
            needed = self.mobile['base_radius'] if straight else clearance
            distance, nearest = segment_clearance(boxes, a, b)
            if distance < needed:
                raise ValueError('no straight route clear of %s (%.2f m < %.2f m); move the robot or the tables'
                                 % (nearest, distance, needed))
        for point in points[1:-1]:     # turning in place: the carried arm sweeps a circle
            distance, nearest = segment_clearance(boxes, point, point)
            if distance < clearance:
                raise ValueError('cannot turn at (%.2f, %.2f): %s %.2f m away' % (point[0], point[1], nearest, distance))
        return waypoints

    def _start_phase(self, task):
        phase = task['phases'][task['phase_index']]
        phase['started'] = _now()
        task['stage'] = phase['name']
        self.message = 'Phase %s' % phase['name']
        if phase['kind'] == 'arm':
            self._plan_arm(task, phase)
            return
        pose = self.base_pose
        if phase['name'] == 'to_object':
            phase['waypoints'] = self._route(task, self._pick_standoff(task), pose, 0.0, self.mobile['base_radius'])
        elif phase['name'] == 'carry':
            goal, xy = self._place_standoff(task, pose[:2])
            task['plan']['place_xy'] = [round(v, 3) for v in xy]
            phase['waypoints'] = self._route(task, goal, pose, self.mobile['back_off'], self.mobile['carry_radius'])
            phase['reference'] = self._in_base(self.truth[task['object']][:3]).tolist()
        else:
            out = (pose[0] - self.mobile['back_off'] * math.cos(pose[3]),
                   pose[1] - self.mobile['back_off'] * math.sin(pose[3]))
            phase['waypoints'] = [dict(x=out[0], y=out[1], reverse=True)]
        phase['waypoint'] = 0

    def _in_base(self, point):
        x, y, z, yaw = self.base_pose
        dx, dy = point[0] - x, point[1] - y
        return np.array([math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy, point[2] - z])

    def _plan_arm(self, task, phase):
        """Plan an arm phase from the measured arm, base and object."""
        from robot_lab_mujoco.arm_pick_place import ArmScene, Box, plan_pick, plan_pick_place, plan_place, synchronised
        measured = self.measured()
        name = task['object']
        if _now() - self.truth_at > STATE_STALE:
            raise ValueError('object truth stale')
        pose = self.truth[name]
        obj = Box(name, pose[:3], pose[3:], self.object_sizes[name])
        base = self.base if not self.mobile else self.base_pose
        planned_joints = set(self.spec.arm_joints + self.spec.finger_joints)
        held = {c['name']: c['measured'] for c in self.state.get('channels', [])
                if c.get('mode') == 'position' and c.get('measured') is not None and c['name'] not in planned_joints}
        scene = ArmScene(self.robot_mjcf, self.spec, list(self.supports.values()), obj, base=base, held=held)
        start = {n: float(measured[n]) for n in self.spec.arm_joints}
        options = dict(approach=task['approach'], lift=task['lift'], seeds=[self.home])
        support = self.supports[task['support']]
        if phase['name'] == 'pick_place':
            planned = plan_pick_place(scene, start, obj, support, **options)
            task['plan'].update(grasp_width_m=planned['grasp_width_m'], place_centre=planned['place_centre'])
        elif phase['name'] == 'pick':
            planned = plan_pick(scene, start, obj, **options)
            task['plan'].update(grasp_width_m=planned['grasp_width_m'], pick_base_measured=[round(v, 4) for v in base])
            task['object_start'] = list(pose[:3])
        else:
            scene.set_joints(dict(start, **{n: float(measured[n]) for n in self.spec.finger_joints}))
            scene.attach()
            planned = plan_place(scene, start, obj, support, place_xy=task['plan']['place_xy'], **options)
            task['plan'].update(place_centre=planned['place_centre'], place_base_measured=[round(v, 4) for v in base])
        stages, previous = [], dict(start)
        for stage in planned['stages']:
            targets = synchronised(stage['waypoints'], previous, RATE, PERIOD)
            previous = dict(stage['waypoints'][-1])
            stages.append(dict(name=stage['stage'], targets=targets,
                               fingers={n: v for n, v in stage['fingers'].items() if n in self.finger_masters},
                               finger_goal=stage['fingers']))
        task.update(stages=stages, index=0, step=0, scene=scene, obj=obj, stage=stages[0]['name'],
                    stage_started=_now())
        task.pop('wait_until', None)
        self.message = 'Planned %d %s stages; executing.' % (len(stages), phase['name'])

    # --- execution ----------------------------------------------------------------
    def _send(self, targets):
        self.command_pub.publish(String(data=json.dumps(dict(plant_id=self.state.get('plant_id'), targets=targets))))

    def _tcp(self, task):
        scene = task['scene']
        measured = self.measured()
        scene.detach()
        scene.set_joints({n: measured[n] for n in self.spec.arm_joints + self.spec.finger_joints})
        return scene.tcp()

    def _tick(self):
        try:
            self._advance()
        except Exception as exc:  # noqa: BLE001 - a task error must stop the arm, not the node
            self.get_logger().error('pick and place error: %r' % exc)
            self._finish('failed', 'internal error: %s' % exc)

    def _advance(self):
        task = self.task
        if task is None:
            return
        self.heartbeat_pub.publish(Empty())
        if _now() - self.state_at > STATE_STALE or self.state.get('fault'):
            return self._finish('failed', 'articulation state stale or faulted: %s' % self.state.get('status'))
        phase = task['phases'][task['phase_index']]
        if _now() - task.get('traced', -math.inf) >= TRACE_PERIOD:
            self._trace(task)
        if phase['kind'] == 'drive':
            return self._drive(task, phase)
        if self.mobile:
            if _now() - self.base_at > STATE_STALE:
                return self._finish('failed', 'base pose stale')
            interlock = task['interlock']
            interlock['max_base_speed_in_arm_phases'] = max(interlock['max_base_speed_in_arm_phases'], self.base_twist[0])
            interlock['max_base_turn_in_arm_phases'] = max(interlock['max_base_turn_in_arm_phases'], self.base_twist[1])
        stage = task['stages'][task['index']]
        if _now() - task['stage_started'] > STAGE_TIMEOUT:
            return self._finish('failed', 'stage %s timed out' % stage['name'])
        if task['step'] < len(stage['targets']):
            self._send(dict(stage['targets'][task['step']], **stage['fingers']))
            task['step'] += 1
            return
        if not self._settled(stage):
            self._send(dict(stage['targets'][-1], **stage['fingers']))
            return
        check = self._check(task, stage['name'])
        if check is None:
            return                              # still waiting on a timed observation
        tcp = self._tcp(task)[0]
        task['log'].append(dict(stage=stage['name'], seconds=round(_now() - task['stage_started'], 2),
                                object_truth=[round(v, 4) for v in self.truth[task['object']][:3]],
                                tcp_estimate=[round(float(v), 4) for v in tcp], **check))
        if not check.get('passed', True):
            return self._finish('failed', '%s: %s' % (stage['name'], check.get('reason')))
        task['index'] += 1
        task['step'] = 0
        task['stage_started'] = _now()
        task.pop('wait_until', None)
        task.pop('finger_history', None)
        if task['index'] >= len(task['stages']):
            return self._next_phase(task)
        task['stage'] = task['stages'][task['index']]['name']
        self.message = 'Stage %s' % task['stage']

    def _next_phase(self, task):
        phase = task['phases'][task['phase_index']]
        task['log'].append(dict(phase=phase['name'], seconds=round(_now() - phase.get('started', task['started']), 2)))
        if self.mobile and phase['kind'] == 'arm' and self.stop_client.service_is_ready():
            # Hold the settled arm so that its residual target error does not
            # keep the base inhibited; the grip keeps its bounded squeeze.
            self.stop_client.call_async(Trigger.Request())
        task['phase_index'] += 1
        if task['phase_index'] >= len(task['phases']):
            return self._finish('succeeded', 'Placed %s on %s' % (task['object'], task['support']))
        try:
            self._start_phase(task)
        except Exception as exc:  # noqa: BLE001 - reported in the receipt
            return self._finish('failed', '%s not planned: %s' % (task['phases'][task['phase_index']]['name'], exc))

    def _drive(self, task, phase):
        """Follow the phase's waypoints on /cmd_vel; never while the articulation is active."""
        if self.base_pose is None or _now() - self.base_at > STATE_STALE:
            return self._finish('failed', 'base pose stale')
        if _now() - phase['started'] > DRIVE_TIMEOUT:
            return self._finish('failed', 'drive %s timed out' % phase['name'])
        interlock = task['interlock']
        interlock['drive_ticks'] += 1
        if phase.get('carrying'):
            drift = float(np.linalg.norm(self._in_base(self.truth[task['object']][:3]) - np.asarray(phase['reference'])))
            phase['max_drift'] = max(phase.get('max_drift', 0.0), drift)
            if _now() - self.truth_at > STATE_STALE or drift > CARRY_DRIFT:
                offset = self._in_base(self.truth[task['object']][:3]) - np.asarray(phase['reference'])
                return self._finish('failed', 'object moved %.3f m relative to the base while carrying (%s)'
                                    % (drift, ', '.join('%+.3f' % v for v in offset)))
        if self.state.get('active'):
            # Never drive a moving arm: wait for it to settle.
            interlock['drive_ticks_with_articulation_active'] += 1
            if phase.get('nav_goal') is not None:
                return self._finish('failed', 'the arm moved during a Nav2 leg')
            return self._twist(0.0, 0.0)
        x, y, _z, yaw = self.base_pose
        if phase['waypoint'] < len(phase['waypoints']) and phase['waypoints'][phase['waypoint']].get('nav2'):
            return self._nav2_leg(task, phase, phase['waypoints'][phase['waypoint']])
        if phase['waypoint'] >= len(phase['waypoints']):
            self._twist(0.0, 0.0)
            if max(self.base_twist) < STILL_SPEED:
                phase['still'] = phase.get('still') or _now()
                if _now() - phase['still'] > 0.5:
                    entry = dict(final_pose=[round(v, 4) for v in self.base_pose])
                    if phase.get('carrying'):
                        entry['max_carry_drift_m'] = round(phase.get('max_drift', 0.0), 4)
                    task['log'].append(dict(phase=phase['name'] + '_arrived', **entry))
                    return self._next_phase(task)
            else:
                phase['still'] = None
            return
        target = phase['waypoints'][phase['waypoint']]
        dx, dy = target['x'] - x, target['y'] - y
        distance = math.hypot(dx, dy)
        reverse = target.get('reverse', False)
        if distance > POSITION_TOLERANCE and not phase.get('at_point'):
            heading = math.atan2(dy, dx) + (math.pi if reverse else 0.0)
            error = _wrap(heading - yaw)
            # Passed it (overshoot along the travel direction): count as reached.
            along = (math.cos(yaw) * dx + math.sin(yaw) * dy) * (-1 if reverse else 1)
            if phase.get('rolling') and along < 0:
                phase['at_point'] = True
                return self._twist(0.0, 0.0)
            if abs(error) > (0.5 if phase.get('rolling') else 0.1):
                phase['rolling'] = False
                return self._twist(0.0, max(-TURN_SPEED, min(TURN_SPEED, 1.5 * error)))
            phase['rolling'] = True
            speed = min(DRIVE_SPEED, 0.6 * distance + 0.02)
            return self._twist(-speed if reverse else speed, max(-TURN_SPEED, min(TURN_SPEED, 1.5 * error)))
        phase['at_point'] = True
        if 'yaw' in target:
            error = _wrap(target['yaw'] - yaw)
            if abs(error) > YAW_TOLERANCE:
                return self._twist(0.0, max(-TURN_SPEED, min(TURN_SPEED, max(0.08, 1.5 * abs(error)) * math.copysign(1, error))))
        phase['waypoint'] += 1
        phase['rolling'] = phase['at_point'] = False
        self._twist(0.0, 0.0)

    def _nav2_leg(self, task, phase, target):
        """Send one NavigateToPose goal and wait for its result; Nav2 owns /cmd_vel meanwhile."""
        if phase.get('nav_goal') is None:
            if not self.nav_client.server_is_ready():
                return self._finish('failed', 'Nav2 navigate_to_pose server lost')
            self._set_footprint(self._carry_outline(task) if phase.get('carrying') else None)
            goal = self.nav_action.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x, goal.pose.pose.position.y = float(target['x']), float(target['y'])
            goal.pose.pose.orientation.z = math.sin(target['yaw'] / 2.0)
            goal.pose.pose.orientation.w = math.cos(target['yaw'] / 2.0)
            phase['nav_goal'] = dict(sent=_now(), future=self.nav_client.send_goal_async(goal), result=None)
            return
        nav = phase['nav_goal']
        if nav['result'] is None:
            if not nav['future'].done():
                return
            handle = nav['future'].result()
            if not handle.accepted:
                return self._finish('failed', 'Nav2 rejected the %s goal' % phase['name'])
            nav['handle'], nav['result'] = handle, handle.get_result_async()
            return
        from action_msgs.msg import GoalStatus
        located = self.amcl_pose is not None and _now() - self.amcl_at < 2.0
        if located and math.hypot(self.amcl_pose[0] - target['x'], self.amcl_pose[1] - target['y']) < NAV2_HANDOFF:
            if not nav['result'].done():
                nav['handle'].cancel_goal_async()
            status = GoalStatus.STATUS_SUCCEEDED
            handed_over = True
        elif not nav['result'].done():
            return
        else:
            status, handed_over = nav['result'].result().status, False
        x, y, _z, yaw = self.base_pose
        entry = dict(phase=phase['name'] + '_nav2', status=int(status), handed_over=handed_over,
                     seconds=round(_now() - nav['sent'], 2),
                     goal=[round(target['x'], 3), round(target['y'], 3), round(target['yaw'], 3)],
                     truth=[round(x, 4), round(y, 4), round(yaw, 4)],
                     goal_error_m=round(math.hypot(target['x'] - x, target['y'] - y), 4),
                     goal_yaw_error_rad=round(_wrap(target['yaw'] - yaw), 4))
        if self.amcl_pose is not None:
            entry['localization_error_m'] = round(math.hypot(self.amcl_pose[0] - x, self.amcl_pose[1] - y), 4)
            entry['localization_yaw_error_rad'] = round(_wrap(self.amcl_pose[2] - yaw), 4)
            entry['localization_age_s'] = round(_now() - self.amcl_at, 2)
        task['log'].append(entry)
        if status != GoalStatus.STATUS_SUCCEEDED:
            return self._finish('failed', 'Nav2 %s leg ended with status %d' % (phase['name'], status))
        self._set_footprint(None)
        phase['nav_goal'] = None
        phase['waypoint'] += 1
        phase['rolling'] = phase['at_point'] = False

    def _carry_outline(self, task):
        """Base circle plus a strip to 5 cm past the held object, as base-frame points."""
        reach = float(self._in_base(self.truth[task['object']][:3])[0]) + max(self.object_sizes[task['object']][:2]) / 2.0 + 0.05
        return [(x, -0.15) for x in (0.0, reach)] + [(reach, 0.15), (0.0, 0.15)]

    def _set_footprint(self, extra):
        """Costmap footprint (both costmaps): convex hull of the base circle and ``extra`` points."""
        points = [(NAV_BASE_RADIUS * math.cos(a), NAV_BASE_RADIUS * math.sin(a))
                  for a in np.linspace(0.0, 2 * math.pi, 16, endpoint=False)] + list(extra or [])
        points = sorted(set(points))

        def turn(o, a, b):
            return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
        hull = []
        for sequence in (points, points[::-1]):     # monotone chain: lower, then upper hull
            chain = []
            for point in sequence:
                while len(chain) >= 2 and turn(chain[-2], chain[-1], point) <= 0:
                    chain.pop()
                chain.append(point)
            hull += chain[:-1]
        polygon = Polygon(points=[Point32(x=float(x), y=float(y), z=0.0) for x, y in hull])
        for publisher in self.footprint_pubs:
            publisher.publish(polygon)

    def _trace(self, task):
        """One stability sample: base and object truth, fingers and the object in the base frame."""
        task['traced'] = _now()
        measured = self.measured()
        sample = dict(t=round(_now() - task['started'], 2), stage=task.get('stage'),
                      object=[round(v, 4) for v in self.truth.get(task['object'], [])[:7]],
                      fingers=[round(measured[n], 4) for n in self.spec.finger_joints if n in measured],
                      finger_references=[round(c['reference'], 4) for c in self.state.get('channels', [])
                                         if c['name'] in self.spec.finger_joints and c.get('reference') is not None])
        if self.mobile and self.base_pose is not None:
            sample['base'] = [round(v, 4) for v in self.base_pose]
            if task['object'] in self.truth:
                sample['object_in_base'] = [round(float(v), 4) for v in self._in_base(self.truth[task['object']][:3])]
        task.setdefault('trace', []).append(sample)

    def _twist(self, linear, angular):
        message = Twist()
        message.linear.x, message.angular.z = float(linear), float(angular)
        self.cmd_pub.publish(message)

    def _settled(self, stage):
        measured = self.measured()
        final = stage['targets'][-1]
        if any(abs(measured[n] - final[n]) > SETTLE_TOLERANCE for n in final):
            return False
        # Fingers settle on target, or once stopped (held by the object) after
        # they had time to start moving. Stopped is judged from positions:
        # Bullet's reported finger velocities jitter while squeezing.
        on_target = all(abs(measured[n] - stage['finger_goal'][n]) < 0.002 for n in self.spec.finger_joints)
        history = self.task.setdefault('finger_history', [])
        history.append((_now(), [measured[n] for n in self.spec.finger_joints]))
        while history and history[0][0] < _now() - FINGER_STILL_WINDOW:
            history.pop(0)
        spans = [max(v[k] for _t, v in history) - min(v[k] for _t, v in history) for k in range(len(self.spec.finger_joints))]
        stopped = len(history) >= 3 and max(spans) < FINGER_STILL_SPAN
        return on_target or (stopped and _now() - self.task['stage_started'] > 1.0)

    def _check(self, task, stage):
        """Truth checks after a stage; None while a timed observation is still running."""
        name = task['object']
        if _now() - self.truth_at > STATE_STALE:
            return dict(passed=False, reason='object truth stale')
        pose = np.asarray(self.truth[name][:3])
        if stage == 'close':
            measured = self.measured()
            gaps = [measured[n] for n in self.spec.finger_joints]
            width = task['plan']['grasp_width_m']
            # Each finger stopped against the object, not at its closed limit.
            gripping = all(g > 0.25 * width / 2 for g in gaps)
            return dict(passed=gripping, finger_positions=[round(g, 4) for g in gaps],
                        reason='fingers closed past the object' if not gripping else '')
        if stage == 'lift':
            if 'wait_until' not in task:
                task['wait_until'] = _now() + 1.0       # hold one second unsupported
                position, rotation = self._tcp(task)
                task['held_offset'] = (rotation.T @ (pose - position)).tolist()
                return None
            if _now() < task['wait_until']:
                return None
            position, rotation = self._tcp(task)
            offset = rotation.T @ (pose - position)
            drift = float(np.linalg.norm(offset - np.asarray(task['held_offset'])))
            rise = float(pose[2] - task['object_start'][2])
            lift = float(task['request'].get('lift', 0.10))
            passed = rise > 0.8 * lift and drift < 0.01
            return dict(passed=passed, rise_m=round(rise, 4), hold_drift_m=round(drift, 4),
                        reason='object did not rise with the tool' if not passed else '')
        if stage == 'retreat':
            if 'wait_until' not in task:
                task['wait_until'] = _now() + 1.0
                task['rest_sample'] = pose.tolist()
                return None
            if _now() < task['wait_until']:
                return None
            support = self.supports[task['support']]
            half = float(task['obj'].size[2]) / 2.0
            local = self._in_support(support, pose)
            speed = float(np.linalg.norm(pose - np.asarray(task['rest_sample'])))
            on_top = abs(pose[2] - (support.top + half)) < 0.01
            inside = abs(local[0]) <= support.size[0] / 2.0 and abs(local[1]) <= support.size[1] / 2.0
            passed = on_top and inside and speed < 0.005
            return dict(passed=passed, final_pose=[round(v, 4) for v in pose], height_error_m=round(pose[2] - support.top - half, 4),
                        inside_support=inside, moved_last_s_m=round(speed, 4),
                        reason='' if passed else 'object not resting on %s' % support.name)
        return dict(passed=True)

    @staticmethod
    def _in_support(support, point):
        from robot_lab_mujoco.arm_pick_place import yaw_of
        yaw = yaw_of(support.quaternion)
        dx, dy = point[0] - support.position[0], point[1] - support.position[1]
        return (math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy)

    def _finish(self, outcome, message):
        task, self.task = self.task, None
        if self.mobile:
            phase = task['phases'][min(task['phase_index'], len(task['phases']) - 1)] if task else {}
            nav = phase.get('nav_goal') or {}
            if nav.get('handle') is not None:
                nav['handle'].cancel_goal_async()
            if task and task.get('transit') == 'nav2':
                self._set_footprint(None)
            self._twist(0.0, 0.0)
        if outcome != 'succeeded' and self.stop_client.service_is_ready():
            self.stop_client.call_async(Trigger.Request())
        self.message = message
        if task is None:
            return
        result = dict(schema='robot_lab.pick_place_result.v1', outcome=outcome, passed=outcome == 'succeeded',
                      message=message, request=task['request'], object=task['object'], support=task['support'],
                      object_start=task['object_start'], plan=task['plan'], stages=task['log'],
                      mobile=bool(self.mobile), interlock=task.get('interlock') if self.mobile else None,
                      transit=task.get('transit'),
                      trace=task.get('trace', []),
                      phase=task['phases'][min(task['phase_index'], len(task['phases']) - 1)]['name'],
                      seconds=round(_now() - task['started'], 2), plant_id=self.state.get('plant_id'),
                      model_sha256=self.state.get('model_sha256'), finished=time.strftime('%Y-%m-%dT%H:%M:%S'))
        path = self.receipts / ('%s_%s.json' % (time.strftime('%Y%m%dT%H%M%S'), task['object']))
        path.write_text(json.dumps(result, indent=1, default=_plain) + '\n')
        result['receipt'] = str(path)
        self.last_result = result
        self.get_logger().info('pick and place %s: %s (%s)' % (outcome, message, path))


def main():
    rclpy.init()
    node = PickPlace()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
