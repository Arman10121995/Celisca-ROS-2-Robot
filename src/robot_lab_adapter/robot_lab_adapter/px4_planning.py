"""Asynchronous selected-world collision planning without blocking FCU ticks."""
import hashlib
import json
import math
from pathlib import Path
import queue
import threading
import time
import uuid

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMessage
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

# Sensor loss in flight (R5.11): hold at once, land if fresh sensing has not returned by then.
SENSOR_LOSS_LAND_S = 10.
# Replanning around predicted moving obstacles: at most once per this interval.
REPLAN_INTERVAL_S = 1.5
# A single late depth frame is not sensor loss: loss is declared after this much continuous staleness.
SENSOR_STALE_GRACE_S = 1.
# Evasion: holding still is not safe from an object that does not react (a walker walked into a
# hovering drone, 2026-11-13). When a moving track is predicted to reach the drone where it hovers,
# climb to this far above the track's top, within the altitude limit.
EVADE_ABOVE_M = 1.
# Walkways: a forward camera cannot see a person walking in from the side (contact at 0.26 m,
# 2026-11-13). Where moving tracks have swept before (SweptMemory), the drone holds at the edge,
# looks both ways and crosses only when no track is predicted on the route for the whole crossing.
SCAN_LOOKAHEAD_M = 2.5
SCAN_YAW_DEG = 75.
SCAN_DWELL_S = .8              # at each heading: enough depth frames for a moving track
SCAN_TURN_TIMEOUT_S = 4.
SCAN_VALID_S = 20.             # one scan per walkway approach
CROSS_WAIT_S = 15.             # then look again
SCANS_PER_WALKWAY = 3          # then fall back to replanning
# A non-reactive walker passing every 7.5 s leaves no window for an 11 s crossing at walker height
# (0.51 m, 2026-11-13): walkways are crossed above their swept columns, this far over their top
# (beyond the 0.7 m vehicle radius) ...
OVERFLY_ABOVE_M = .3
# ... but no higher than the level depth camera still sees ground within this share of its range:
# at 4.2-4.6 m it saw too little valid depth, which is sensor loss, and the drone landed (2026-11-13).
DEPTH_RANGE_M, VISIBLE_SHARE = 8., .8          # depth_points' range limit


class PX4Planning(Node):
    def __init__(self, controller, world, vehicle_envelope=None, observed_obstacles=False):
        super().__init__('px4_static_aerial_planner')
        self.controller, self.world = controller, world
        self.vehicle_envelope = vehicle_envelope
        self.obstacles = None
        if observed_obstacles:
            from .aerial_obstacles import AerialObstacles
            self.obstacles = AerialObstacles(self)
            self.obstacles.ground_z = controller.ground_z
            try:
                from robot_lab_utils.aerial_planning import StaticScene
                self.obstacles.static_scene = StaticScene(world)    # depth points on it are not moving objects
            except Exception as exc:
                self.get_logger().warning('Static scene for the tracking filter unavailable: %s' % exc)
            controller.route_guard = self.obstacles.route_guard
        self.takeoff_plan = None
        self.takeoff_state = dict(state='unchecked')
        controller.takeoff_guard = self
        self.plant_id = uuid.uuid4().hex
        self.scene, self.route, self.future_events = None, None, queue.Queue()
        self.canceled = threading.Event()
        self.generation, self.worker = 0, None
        self.goal_enu, self.replan_epoch, self.last_replan = None, None, -math.inf
        self.last_evasion = -math.inf
        self.scanning, self.crossing_until, self.last_scan, self.scans = None, -math.inf, -math.inf, 0
        self.sensor_loss_since, self.sensor_loss, self.stale_since = None, dict(state='ok'), None
        self.state = dict(plant_id=self.plant_id, state='idle', qualification=
            'experimental static + observed aerial planning; flight validation pending' if self.obstacles else
            'implemented static aerial planning; validation pending')
        self.publisher = self.create_publisher(String, '/px4/planning/status', 10)
        self.path_pub = self.create_publisher(PathMessage, '/px4/planned_path', 10)
        self.create_subscription(PoseStamped, '/px4/plan_goal', self.goal, 1)
        self.create_service(Trigger, '/px4/plan_execute', self.execute)
        self.create_service(Trigger, '/px4/plan_cancel', self.cancel)
        self.create_service(Trigger, '/px4/takeoff_check', self.check_takeoff)
        self.create_timer(.2, self.poll)

    def goal(self, message):
        # Every goal request is numbered in the status, so a client can tell its own result from an earlier one.
        self.state['goal_request'] = self.state.get('goal_request', 0)+1
        if (self.worker and self.worker.is_alive() or message.header.frame_id != 'map'
                or self.controller.phase != 'flying' or not self.controller.ready()
                or self.controller.route or any(self.controller.manual)):
            self.state.update(state='rejected', error='Plan requires an airborne owned vehicle, frame map, and idle planner')
            return
        p = message.pose.position
        goal = [p.x, p.y, p.z]
        if not all(map(math.isfinite, goal)):
            self.state.update(state='rejected', error='Finite 3D goal required')
            return
        try:
            observations, observation_metadata = self.obstacles.snapshot() if self.obstacles else ((), None)
        except ValueError as exc:
            self.state.update(state='rejected', error=str(exc))
            return
        self.route = None
        self.goal_enu = goal
        self.generation += 1
        self.canceled = threading.Event()
        generation, canceled = self.generation, self.canceled
        start = list(self.controller.position())
        command_generation = self.controller.command_generation
        minimum, maximum = self.controller.ground_z+.5, self.controller.ground_z+10
        self.state.update(state='planning', goal_enu=goal, error='')
        def work():
            try:
                from robot_lab_utils.aerial_planning import StaticScene, plan
                scene = StaticScene(self.world)
                from robot_lab_utils.observed_aerial_scene import ObservedScene
                collision_scene = ObservedScene(scene, *observations) if observations else scene
                route, details = plan(collision_scene, start, goal, radius=.7, canceled=canceled.is_set,
                    min_altitude=minimum, max_altitude=maximum)
                stamps = {resource: (Path(resource).stat().st_mtime_ns, Path(resource).stat().st_size)
                          for resource in scene.resources}
                if any(stamps[path] != stamp for path, stamp in scene.resource_stamps.items()):
                    raise ValueError('Collision source changed while planning; request a new plan')
                result = dict(generation=generation, scene=scene, route=route, details=details,
                    command_generation=command_generation, observations=observation_metadata,
                    resources=stamps, start=start, world_sha256=hashlib.sha256(Path(self.world).read_bytes()).hexdigest())
            except Exception as exc:
                result = dict(generation=generation, error=str(exc))
            self.future_events.put(result)
        self.worker = threading.Thread(target=work, daemon=True)
        self.worker.start()

    def check_takeoff(self, _request, response):
        c = self.controller
        if (self.worker and self.worker.is_alive() or not self.vehicle_envelope or not c.takeoff_ready()):
            response.success, response.message = False, 'Check requires the exact native X500 envelope, fresh idle/landed FCU state and an idle planner.'
            return response
        self.takeoff_plan = None
        self.generation += 1
        self.canceled = threading.Event()
        generation, canceled = self.generation, self.canceled
        start = list(c.position())
        desired_rise = float(c.get_parameter('takeoff_alt').value)
        self.takeoff_state = dict(state='checking', start_enu=start)
        def work():
            try:
                from robot_lab_utils.aerial_planning import StaticScene
                from robot_lab_utils.aerial_takeoff import check_takeoff
                scene = StaticScene(self.world)
                result = check_takeoff(scene, start, self.vehicle_envelope,
                    desired_rise=desired_rise, canceled=canceled.is_set)
                resources = dict(scene.resource_stamps)
                resources.update(self.vehicle_envelope['resources'])
                for resource in scene.resources:
                    if resource not in resources:
                        resources[resource] = (Path(resource).stat().st_mtime_ns, Path(resource).stat().st_size)
                if any((Path(path).stat().st_mtime_ns, Path(path).stat().st_size) != stamp
                       for path, stamp in resources.items()):
                    raise ValueError('Takeoff collision resources changed during the check.')
                event = dict(generation=generation, kind='takeoff', details=result, resources=resources)
            except Exception as exc:
                event = dict(generation=generation, kind='takeoff', error=str(exc))
            self.future_events.put(event)
        self.worker = threading.Thread(target=work, daemon=True)
        self.worker.start()
        response.success, response.message = True, 'Checking the selected world asynchronously; press Takeoff after a fresh clear result.'
        return response

    def consume_takeoff(self):
        plan = self.takeoff_plan
        if (not plan or time.monotonic()-plan['checked_at'] > 30 or not self.controller.takeoff_ready()
                or math.dist(self.controller.position(), plan['details']['start_enu']) > .05):
            raise ValueError('Check takeoff clearance first; a fresh result from the current landed FCU pose is required.')
        if any((Path(path).stat().st_mtime_ns, Path(path).stat().st_size) != stamp
               for path, stamp in plan['resources'].items()):
            self.takeoff_plan = None
            self.takeoff_state.update(state='invalidated', error='World/vehicle resources changed; check again.')
            raise ValueError(self.takeoff_state['error'])
        self.takeoff_plan = None
        self.takeoff_state.update(state='consumed')
        return plan['details']['target_enu'][2]

    def execute(self, _request, response):
        try:
            if (self.route is None or time.monotonic()-self.planned_at > 15 or not self.controller.ready()
                    or self.controller.phase != 'flying' or math.dist(self.controller.position(), self.start) > .3):   # hover drift
                raise ValueError('Fresh plan from the current measured FCU pose required')
            if not self.resources_current():
                raise ValueError('Selected collision resources changed; plan again')
            if self.controller.command_generation != self.plan_command_generation or any(self.controller.manual):
                raise ValueError('Operator/flight command changed since Plan; plan again')
            if self.obstacles:
                observations, metadata = self.obstacles.snapshot()
                if self.worker and self.worker.is_alive():
                    raise ValueError('Wait for the current route check')
                route, start, scene = self.route, list(self.controller.position()), self.scene
                epoch = self.controller.command_generation
                self.generation += 1
                generation = self.generation
                self.canceled = threading.Event()
                canceled = self.canceled
                self.state.update(state='checking_execution', error='')
                def work():
                    try:
                        from robot_lab_utils.observed_aerial_scene import ObservedScene
                        collision_scene = ObservedScene(scene, *observations)
                        deadline = time.monotonic()+3.
                        for a, b in zip(route[:-1], route[1:]):
                            if not collision_scene.segment(a, b, .7, deadline, canceled.is_set):
                                raise ValueError('Current observed occupancy blocks the route; Plan again')
                        event = dict(kind='execute_route', generation=generation, command_generation=epoch,
                            route=route, start=start, observations=metadata)
                    except Exception as exc:
                        event = dict(kind='execute_route', generation=generation, error=str(exc))
                    self.future_events.put(event)
                self.worker = threading.Thread(target=work, daemon=True)
                self.worker.start()
                response.success, response.message = True, 'Rechecking against fresh observations asynchronously; execution follows a clear result.'
                return response
            self.controller.set_path(self.route)
            self.route = None
            self.state.update(state='executing', error='')
            response.success, response.message = True, 'Executing checked static waypoints; no dynamic-obstacle prediction'
        except Exception as exc:
            response.success, response.message = False, str(exc)
        return response

    def resources_current(self):
        try:
            return all((Path(path).stat().st_mtime_ns, Path(path).stat().st_size) == stamp
                       for path, stamp in self.resources.items())
        except OSError:
            return False

    def replan(self, epoch):
        """Plan again to the same goal around the swept volumes of moving tracks (asynchronous)."""
        if self.worker and self.worker.is_alive() or self.goal_enu is None:
            return False
        try:
            observations, metadata = self.obstacles.snapshot()
        except ValueError:
            return False
        from robot_lab_utils.aerial_tracking import PredictedObstacles
        predicted = PredictedObstacles(self.obstacles.moving_tracks())
        self.last_replan = time.monotonic()
        self.generation += 1
        self.canceled = threading.Event()
        generation, canceled = self.generation, self.canceled
        start, goal = list(self.controller.position()), list(self.goal_enu)
        minimum, maximum = self.controller.ground_z+.5, self.controller.ground_z+10
        self.state.update(state='replanning', error='', moving_obstacles=len(predicted.capsules))
        def work():
            try:
                from robot_lab_utils.aerial_planning import StaticScene, plan
                from robot_lab_utils.observed_aerial_scene import ObservedScene
                scene = ObservedScene(StaticScene(self.world), *observations, predicted)
                route, details = plan(scene, start, goal, radius=.7, canceled=canceled.is_set, timeout=8.,
                                      min_altitude=minimum, max_altitude=maximum)
                event = dict(kind='replan', generation=generation, epoch=epoch, route=route, start=start,
                             details=details, observations=metadata, predicted=len(predicted.capsules))
            except Exception as exc:
                event = dict(kind='replan', generation=generation, epoch=epoch, error=str(exc))
            self.future_events.put(event)
        self.worker = threading.Thread(target=work, daemon=True)
        self.worker.start()
        return True

    def evade(self):
        """Hovering drone in a moving object's predicted path: climb above it (then the replan resumes)."""
        c = self.controller
        if (not self.obstacles or c.phase != 'flying' or not c.ready() or c.route or any(c.manual)
                or self.sensor_loss_since is not None):
            return False
        from robot_lab_utils.aerial_tracking import time_conflict
        position = c.position()
        conflict = time_conflict(position, [], 0., self.obstacles.moving_tracks(), radius=.7)
        if not conflict:
            return False
        when, track = conflict
        ceiling = c.ground_z + 10.
        height = min(ceiling, max(position[2], track.z_high + EVADE_ABOVE_M))
        if height - position[2] < .2 and c.target and c.target[2] >= height - .05:
            return False                       # already above it (or climbing there)
        if height <= track.z_high + .3:
            self.state.update(state='evasion_limited', error='altitude limit below the obstacle top')
            return False
        c.clear_path('Observed route held: evading moving obstacle %d (%.1f s); climbing to %.1f m' % (
            track.id, when, height))
        c.route_status = ('Observed route held: evading moving obstacle %d (%.1f s); climbing to %.1f m'
                          % (track.id, when, height))   # clear_path keeps the status of an empty route
        c.target = [position[0], position[1], height]
        self.last_evasion = time.monotonic()
        self.canceled.set()                    # a replan from the old position is obsolete
        self.state.update(state='evading', error='', evasion=dict(track=track.state(), climb_to_m=round(height, 2),
                                                                 predicted_in_s=round(when, 2)))
        return True

    def walkway(self, route, lookahead=SCAN_LOOKAHEAD_M):
        """Where ``route`` (from the current position) meets a swept walkway: dict(column, entry, length,
        inside) with distances along the route, or None if it does not within ``lookahead``."""
        swept = getattr(self.obstacles, 'swept', None)
        columns = swept.snapshot()[1] if swept else []
        if not columns or not route:
            return None
        def inside(q):
            return next((column for column in columns if math.hypot(q[0]-column[0], q[1]-column[1]) <= column[2]+.7
                         and column[3]-.7 <= q[2] <= column[4]+.7), None)
        def samples():
            points, travelled = [self.controller.position()]+[list(p) for p in route], 0.
            for a, b in zip(points[:-1], points[1:]):
                length = math.dist(a, b)
                for k in range(max(1, int(length/.1))):
                    yield travelled+k*.1, [a[i]+(b[i]-a[i])*k*.1/max(length, 1e-9) for i in range(3)]
                travelled += length
        first = last = None
        top = -math.inf
        for d, q in samples():
            column = inside(q)
            if column and first is None:
                if d > lookahead:
                    return None
                first = (column, d)
            if column:
                last, top = d, max(top, column[4])
            elif first is not None and d-last > 1.:
                break
        if first is None:
            return None
        return dict(column=first[0], entry=first[1], length=last-first[1], inside=first[1] == 0., top=top)

    def overfly(self, route, found):
        """(route, distance to the far side): ``route`` with the walkway crossed above its swept columns,
        straight up here, across, then on."""
        c = self.controller
        height = found['top']+.7+OVERFLY_ABOVE_M
        exit_d = found['entry']+found['length']+.6
        info = getattr(self.obstacles, 'intrinsic', None)
        half_fov = (math.atan(info[0].height/2/info[0].k[4]) if info and info[0].k[4] > 0
                    else math.radians(25.))
        visible = c.ground_z+VISIBLE_SHARE*DEPTH_RANGE_M*math.tan(half_fov)-.3
        height = min(height, visible)
        if height < found['top']+.7 or height > c.ground_z+10.-.1:
            return route, exit_d               # cannot clear the walkway and still see: cross at route height
        position = c.position()
        points = [position]+[list(p) for p in route]
        travelled, across, rest = 0., None, []
        for a, b in zip(points[:-1], points[1:]):
            length = math.dist(a, b)
            if across is None and travelled+length >= exit_d:
                f = (exit_d-travelled)/max(length, 1e-9)
                across = [a[i]+(b[i]-a[i])*f for i in range(3)]
            if across is not None and travelled+length > exit_d:
                rest.append(list(b))
            travelled += length
        if across is None:
            across = list(points[-1])
        climb = [position[0], position[1], max(position[2], height)]
        across = [across[0], across[1], max(across[2], height)]
        return [climb, across]+rest, math.dist(position, climb)+math.dist(climb, across)

    def scan(self):
        """Look both ways at a walkway edge, then cross when no moving track is predicted on the route."""
        from .aerial_obstacles import ROUTE_SPEED
        from .px4_frames import yaw_enu
        from robot_lab_utils.aerial_tracking import time_conflict
        c, now = self.controller, time.monotonic()
        healthy = (self.obstacles and c.phase == 'flying' and c.ready() and not any(c.manual)
                   and self.sensor_loss_since is None)
        if self.scanning:
            scan = self.scanning
            if not healthy or self.state['state'] != 'scanning':
                c.target_yaw, self.scanning = scan['base'], None
                return
            if scan['headings']:
                target = scan['headings'][0]
                yaw = yaw_enu(c.flight.attitude.yaw) if c.flight.attitude else target
                error = abs(math.atan2(math.sin(yaw-target), math.cos(yaw-target)))
                if scan['reached'] is None and (error < math.radians(10) or now-scan['since'] > SCAN_TURN_TIMEOUT_S):
                    scan['reached'] = now
                if scan['reached'] is not None and now-scan['reached'] >= SCAN_DWELL_S:
                    scan['headings'].pop(0)
                    if scan['headings']:
                        c.target_yaw, scan['since'], scan['reached'] = scan['headings'][0], now, None
                    else:
                        scan['looked_at'] = self.last_scan = now
                return
            # Looked both ways: cross when no track (seen, or coasting out of view) is predicted on the
            # route during the whole crossing; otherwise wait, then look again, then replan.
            tracks = self.obstacles.tracker.moving()
            horizon = (scan['exit_d']+1.)/ROUTE_SPEED + 1.   # until the drone is past the walkway
            conflict = time_conflict(c.position(), scan['route'], ROUTE_SPEED, tracks, horizon=horizon)
            guard = self.obstacles.route_guard(c.position(), scan['route'])
            if not conflict and not guard:
                self.scanning, self.scans = None, 0
                self.crossing_until = now+scan['entry']/ROUTE_SPEED+2.   # until the drone is in the walkway
                try:
                    c.set_path([c.position()]+scan['route'])   # set_path's first point is the start
                    self.state.update(state='executing', error='', walkway=dict(scan['report'], crossed_after_s=round(now-scan['started'], 2)))
                except ValueError as exc:
                    self.state.update(state='rejected', error=str(exc))
                return
            scan['report']['waiting_for'] = (('moving obstacle %d in %.1f s' % (conflict[1].id, conflict[0])) if conflict else guard)
            if now-scan['looked_at'] > CROSS_WAIT_S:
                self.scanning, self.last_scan = None, now
                if self.scans >= SCANS_PER_WALKWAY:
                    self.state.update(state='route_stopped', error='', walkway=dict(scan['report'], outcome='replan'))
                else:
                    self.last_scan = -math.inf       # look again
                    self.state.update(state='executing')
                    c.set_path([c.position()]+scan['route'])   # re-enter the scan from the same edge
            return
        if not healthy or self.state['state'] != 'executing' or not c.route:
            return
        found = self.walkway(c.route)
        if not found:
            return
        if found['inside']:
            # Crossing (or caught) in a walkway: if a track is predicted on the rest of the crossing, climb
            # above it and stay there; the replan from up there meets the walkway as a new crossing.
            position = c.position()
            conflict = time_conflict(position, c.route, ROUTE_SPEED, self.obstacles.tracker.moving(),
                                     horizon=(found['length']+2.)/ROUTE_SPEED+1.)
            if conflict and now-self.last_evasion > 2.:
                when, track = conflict
                height = min(c.ground_z+10., max(position[2], track.z_high+EVADE_ABOVE_M))
                if height-position[2] >= .2 and height > track.z_high+.3:
                    reason = ('Observed route held: moving obstacle %d predicted in the walkway in %.1f s; climbing to %.1f m'
                              % (track.id, when, height))
                    c.clear_path(reason)
                    c.route_status = reason
                    c.target = [position[0], position[1], height]
                    self.last_evasion = now
                    self.canceled.set()
                    self.state.update(state='evading', error='', evasion=dict(track=track.state(), walkway=True,
                                      climb_to_m=round(height, 2), predicted_in_s=round(when, 2)))
            return
        if now < self.crossing_until:
            return                             # this crossing was cleared; the drone is entering
        column, length = found['column'], found['length']
        route, exit_d = self.overfly([list(p) for p in c.route], found)
        base = c.target_yaw
        look = now-self.last_scan >= SCAN_VALID_S  # a recent look both ways is reused; the timing check is not
        c.clear_path('Observed route held: %s the walkway at (%.1f, %.1f) before crossing'
                     % ('looking along' if look else 'timing', column[0], column[1]))
        c.target = list(c.position())
        headings = [base+math.radians(SCAN_YAW_DEG), base-math.radians(SCAN_YAW_DEG), base] if look else []
        if look:
            self.scans += 1
            c.target_yaw = headings[0]
        self.scanning = dict(base=base, headings=headings, since=now, reached=None, route=route, exit_d=exit_d,
                             entry=found['entry'], started=now, looked_at=None if look else now,
                             report=dict(walkway=[round(v, 2) for v in column[:2]], crossing_m=round(length, 2),
                                         scan=self.scans, looked=look, crossing_z=round(route[0][2], 2)))
        self.state.update(state='scanning', error='', walkway=self.scanning['report'])

    def sensor_guard(self):
        """Stale depth/map in flight: hold now, land after SENSOR_LOSS_LAND_S unless sensing returns."""
        c = self.controller
        if not self.obstacles or c.phase != 'flying' or not c.ready():
            self.sensor_loss_since = self.stale_since = None
            if self.sensor_loss.get('state') != 'landing':
                self.sensor_loss = dict(state='ok')
            return
        now = time.monotonic()
        if self.obstacles.sensing_fresh():
            if self.sensor_loss_since is not None:
                self.sensor_loss = dict(state='recovered', lost_s=round(now-self.sensor_loss_since, 2))
            self.sensor_loss_since = self.stale_since = None
            return
        if getattr(self, 'stale_since', None) is None:
            self.stale_since = now
        if now-self.stale_since < SENSOR_STALE_GRACE_S and self.sensor_loss_since is None:
            return
        if self.sensor_loss_since is None:
            self.sensor_loss_since = now
            self.canceled.set()
            c.clear_path('sensor loss: holding position; landing in %.0f s unless sensing returns' % SENSOR_LOSS_LAND_S)
            c.target = c.position()
            c.manual = (0., 0., 0., 0.)
        lost = now-self.sensor_loss_since
        self.sensor_loss = dict(state='holding', lost_s=round(lost, 2), land_in_s=round(max(0., SENSOR_LOSS_LAND_S-lost), 2))
        if lost >= SENSOR_LOSS_LAND_S:
            response = c.land(None, Trigger.Response())
            self.sensor_loss = dict(state='landing' if response.success else 'land_refused', lost_s=round(lost, 2),
                                    message=response.message)
            self.sensor_loss_since = None

    def cancel(self, _request, response):
        self.canceled.set()
        self.generation += 1
        self.route = None
        self.goal_enu = None
        self.takeoff_plan = None
        self.takeoff_state = dict(state='canceled')
        self.controller.clear_path('planning/route canceled')
        if self.controller.phase == 'flying' and self.controller.ready():
            self.controller.target = self.controller.position()
        self.state.update(state='canceled', error='')
        response.success, response.message = True, 'Planner canceled; airborne route holds current FCU position'
        return response

    def poll(self):
        if self.obstacles:
            self.obstacles.poll()
        try:
            while True:
                result = self.future_events.get_nowait()
                if result['generation'] != self.generation:
                    continue
                if result.get('kind') == 'replan':
                    c = self.controller
                    if 'error' in result:
                        self.state.update(state='replan_failed', error=result['error'])
                    elif (c.ready() and c.phase == 'flying' and c.command_generation == result['epoch']
                            and not any(c.manual) and not c.route and self.sensor_loss_since is None
                            and math.dist(c.position(), result['start']) <= .3):
                        try:
                            c.set_path(result['route'])
                            self.state.update(state='executing', error='', replanned=dict(
                                waypoints=len(result['route']), predicted_obstacles=result['predicted'],
                                **result['details']))
                        except ValueError as exc:
                            self.state.update(state='rejected', error=str(exc))
                    else:
                        self.state.update(state='invalidated', error='FCU/operator state changed during the replan')
                    continue
                if result.get('kind') == 'execute_route':
                    if 'error' in result:
                        self.route = None
                        self.state.update(state='rejected', error=result['error'])
                    elif (self.obstacles.ready() and self.controller.ready() and self.controller.phase == 'flying'
                            and self.controller.command_generation == result['command_generation']
                            and not any(self.controller.manual) and math.dist(self.controller.position(), result['start']) <= .3
                            and self.resources_current()):
                        try:
                            self.controller.set_path(result['route'])
                            self.route = None
                            self.state.update(state='executing', error='', executed_observations=result['observations'])
                        except ValueError as exc:
                            self.state.update(state='rejected', error=str(exc))
                    else:
                        self.route = None
                        self.state.update(state='invalidated', error='FCU/operator/resources/observation state changed during route recheck')
                    continue
                if result.get('kind') == 'takeoff':
                    if 'error' in result:
                        self.takeoff_state = dict(state='failed', error=result['error'])
                    elif self.controller.takeoff_ready() and math.dist(self.controller.position(), result['details']['start_enu']) <= .05:
                        self.takeoff_plan = dict(result, checked_at=time.monotonic())
                        self.takeoff_state = dict(state='clear', **result['details'], error='')
                    else:
                        self.takeoff_state = dict(state='invalidated', error='FCU state/pose changed while checking; check again.')
                    continue
                if 'error' in result:
                    self.state.update(state='failed', error=result['error'])
                    continue
                if self.controller.command_generation != result['command_generation']:
                    self.route = None
                    self.state.update(state='invalidated', error='Operator/flight command interrupted planning')
                    continue
                self.scene, self.route, self.start = result['scene'], result['route'], result['start']
                self.plan_command_generation = result['command_generation']
                self.resources, self.planned_at = result['resources'], time.monotonic()
                self.state.update(state='planned', waypoints=len(self.route), **result['details'],
                    world_sha256=result['world_sha256'], error='')
                if result.get('observations'):
                    self.state['planned_observations'] = result['observations']
                path = PathMessage()
                path.header.frame_id, path.header.stamp = 'map', self.get_clock().now().to_msg()
                for point in self.route:
                    pose = PoseStamped(header=path.header)
                    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = point
                    pose.pose.orientation.w = 1.
                    path.poses.append(pose)
                self.path_pub.publish(path)
        except queue.Empty:
            pass
        if self.route is not None and time.monotonic()-self.planned_at > 15:
            self.route = None
            self.state.update(state='expired', error='Plan expired; plan from the current position')
        if self.obstacles:
            self.sensor_guard()
            self.scan()
            self.evade()                       # a walker heading for the hovering drone overrides a scan
        if self.state['state'] == 'executing' and not self.controller.route:
            self.state.update(state='route_stopped', execution=self.controller.route_status)
        c = self.controller
        if (self.obstacles and self.goal_enu is not None and self.state['state'] in ('route_stopped', 'replan_failed', 'evading')
                and (str(c.route_status).startswith('Observed route held:')      # moving, occupied or stale
                     or str(c.route_status).startswith('sensor loss: holding'))
                and self.obstacles.ready()
                and c.phase == 'flying' and not any(c.manual) and self.sensor_loss_since is None
                and time.monotonic()-self.last_replan > REPLAN_INTERVAL_S
                and time.monotonic()-self.last_evasion > 2.):              # let an evasive climb finish
            # The guard held (a predicted moving obstacle, current occupancy in the lookahead, or a
            # recovered sensor): replan to the goal around current observations and predicted sweeps.
            self.replan(c.command_generation)
        if self.takeoff_plan and (time.monotonic()-self.takeoff_plan['checked_at'] > 30
                or not self.controller.takeoff_ready()
                or math.dist(self.controller.position(), self.takeoff_plan['details']['start_enu']) > .05):
            self.takeoff_plan = None
            self.takeoff_state.update(state='expired', error='Landed FCU pose/state changed or the 30 s check expired.')
        if self.route is not None and self.controller.command_generation != self.plan_command_generation:
            self.route = None
            self.state.update(state='invalidated', error='Operator/flight command invalidated the pending route')
        self.publisher.publish(String(data=json.dumps(dict(self.state, takeoff=self.takeoff_state, sensor_loss=self.sensor_loss,
            obstacles=self.obstacles.state() if self.obstacles else dict(enabled=False)))))

    def close(self):
        self.canceled.set()
        if self.obstacles:
            self.controller.route_guard = None
            self.obstacles.close()
