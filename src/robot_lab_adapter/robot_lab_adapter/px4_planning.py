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


class PX4Planning(Node):
    def __init__(self, controller, world, vehicle_envelope=None):
        super().__init__('px4_static_aerial_planner')
        self.controller, self.world = controller, world
        self.vehicle_envelope = vehicle_envelope
        self.takeoff_plan = None
        self.takeoff_state = dict(state='unchecked')
        controller.takeoff_guard = self
        self.plant_id = uuid.uuid4().hex
        self.scene, self.route, self.future_events = None, None, queue.Queue()
        self.canceled = threading.Event()
        self.generation, self.worker = 0, None
        self.state = dict(plant_id=self.plant_id, state='idle', qualification='implemented static aerial planning; validation pending')
        self.publisher = self.create_publisher(String, '/px4/planning/status', 10)
        self.path_pub = self.create_publisher(PathMessage, '/px4/planned_path', 10)
        self.create_subscription(PoseStamped, '/px4/plan_goal', self.goal, 1)
        self.create_service(Trigger, '/px4/plan_execute', self.execute)
        self.create_service(Trigger, '/px4/plan_cancel', self.cancel)
        self.create_service(Trigger, '/px4/takeoff_check', self.check_takeoff)
        self.create_timer(.2, self.poll)

    def goal(self, message):
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
        self.route = None
        self.generation += 1
        self.canceled = threading.Event()
        generation, canceled = self.generation, self.canceled
        start = list(self.controller.position())
        minimum, maximum = self.controller.ground_z+.5, self.controller.ground_z+10
        self.state.update(state='planning', goal_enu=goal, error='')
        def work():
            try:
                from robot_lab_utils.aerial_planning import StaticScene, plan
                scene = StaticScene(self.world)
                route, details = plan(scene, start, goal, radius=.7, canceled=canceled.is_set,
                    min_altitude=minimum, max_altitude=maximum)
                stamps = {resource: (Path(resource).stat().st_mtime_ns, Path(resource).stat().st_size)
                          for resource in scene.resources}
                if any(stamps[path] != stamp for path, stamp in scene.resource_stamps.items()):
                    raise ValueError('Collision source changed while planning; request a new plan')
                result = dict(generation=generation, scene=scene, route=route, details=details,
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
                    or self.controller.phase != 'flying' or math.dist(self.controller.position(), self.start) > .05):
                raise ValueError('Fresh plan from the current measured FCU pose required')
            if any((Path(path).stat().st_mtime_ns, Path(path).stat().st_size) != stamp for path, stamp in self.resources.items()):
                raise ValueError('Selected collision resources changed; plan again')
            self.controller.set_path(self.route)
            self.route = None
            self.state.update(state='executing', error='')
            response.success, response.message = True, 'Executing checked static waypoints; no dynamic-obstacle prediction'
        except Exception as exc:
            response.success, response.message = False, str(exc)
        return response

    def cancel(self, _request, response):
        self.canceled.set()
        self.generation += 1
        self.route = None
        self.takeoff_plan = None
        self.takeoff_state = dict(state='canceled')
        self.controller.clear_path('planning/route canceled')
        if self.controller.phase == 'flying' and self.controller.ready():
            self.controller.target = self.controller.position()
        self.state.update(state='canceled', error='')
        response.success, response.message = True, 'Planner canceled; airborne route holds current FCU position'
        return response

    def poll(self):
        try:
            while True:
                result = self.future_events.get_nowait()
                if result['generation'] != self.generation:
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
                self.scene, self.route, self.start = result['scene'], result['route'], result['start']
                self.resources, self.planned_at = result['resources'], time.monotonic()
                self.state.update(state='planned', waypoints=len(self.route), **result['details'],
                    world_sha256=result['world_sha256'], error='')
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
        if self.state['state'] == 'executing' and not self.controller.route:
            self.state.update(state='route_stopped', execution=self.controller.route_status)
        if self.takeoff_plan and (time.monotonic()-self.takeoff_plan['checked_at'] > 30
                or not self.controller.takeoff_ready()
                or math.dist(self.controller.position(), self.takeoff_plan['details']['start_enu']) > .05):
            self.takeoff_plan = None
            self.takeoff_state.update(state='expired', error='Landed FCU pose/state changed or the 30 s check expired.')
        self.publisher.publish(String(data=json.dumps(dict(self.state, takeoff=self.takeoff_state))))

    def close(self):
        self.canceled.set()
