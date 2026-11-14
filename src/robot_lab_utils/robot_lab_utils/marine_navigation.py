#!/usr/bin/env python3
"""Estimated-pose marine Plan/Execute over known collision worlds + observations.

Native marine controllers still own actual bounded thrusters/fins. No vehicle
pose writes, underwater GPS, empty-space assumptions or invented task success.
"""
from concurrent.futures import ThreadPoolExecutor
import json
import math
import os
from pathlib import Path
import sqlite3
import threading
import time
import uuid

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from scipy.spatial.transform import Rotation

from robot_lab_utils.aerial_planning import StaticScene, plan
from robot_lab_utils.marine_controller import rpy
from robot_lab_utils.marine_navigation_model import waypoint_command
from robot_lab_utils.observed_aerial_scene import ObservedScene, VoxelObstacles


class MarineNavigation(Node):
    def __init__(self):
        super().__init__('marine_mission_controller')
        for key, value in dict(world_file='', robot_id='lab_rov', surface_z=0., bottom_z=-6.,
                               radius=.55, database_path='', control_allowed=True, localization_mode=False,
                               saved_map_source='').items():
            self.declare_parameter(key, value)
        self.robot = self.get_parameter('robot_id').value
        self.auv = self.robot == 'tethys_auv'
        self.surface_vessel = self.robot in ('lab_usv','lab_amphibious')
        self.surface = float(self.get_parameter('surface_z').value)
        self.bottom = float(self.get_parameter('bottom_z').value)
        self.radius = float(self.get_parameter('radius').value)
        self.database = str(self.get_parameter('database_path').value)
        self.localization = bool(self.get_parameter('localization_mode').value)
        self.localized_at = -math.inf
        self.localized_stamp = -math.inf
        self.scene = StaticScene(str(self.get_parameter('world_file').value))
        self.shore=None
        if self.robot=='lab_amphibious':
            from robot_lab_utils.amphibious_planning import ShoreScene
            try:self.shore=ShoreScene(str(self.get_parameter('world_file').value),self.surface)
            except ValueError:pass # Other worlds retain explicit surface-only routes.
        self.observed = None
        self.cloud_at = -math.inf
        self.native = self.position = self.orientation = self.velocity = None
        self.angular_velocity = math.nan
        self.pose_at = self.native_at = -math.inf
        self.pose_std = math.inf
        self.status = 'idle'
        self.reason = 'Plan from sensor odometry; explicit Execute required'
        self.route = None
        self.auv_follower = None
        self.auv_plan = None
        self.index = 0
        self.arrival_at = 0.
        self.deadline = 0.
        self.claim_until = 0.
        self.generation = 0
        self.stop_token = threading.Event()
        self.future = None
        self.future_generation = None
        self.future_kind = None
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='marine-plan')
        self.tf = Buffer(node=self)
        self.listener = TransformListener(self.tf, self)
        self.drive_pub = self.create_publisher(Twist, '/marine/mission_vel', 10)
        self.control_pub = self.create_publisher(String, '/marine/command', 10)
        self.state_pub = self.create_publisher(String, '/marine/mission/state', 10)
        self.create_subscription(String, '/marine/state', self.observe_native, 10)
        self.replan_at = -math.inf
        # This is an operator-authored world current, not a water-current
        # sensor. Native force control consumes it; AUV route costs use it too.
        self.current_vel = None
        self.current_at = -math.inf
        self.create_subscription(Vector3, '/marine/current', self.observe_current, 10)
        self.create_subscription(Odometry, '/marine/estimated_odometry', self.observe_pose, 10)
        self.create_subscription(PointCloud2, '/marine/slam/cloud_map', self.observe_cloud, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/marine/slam/localization_pose', self.observe_localization, 10)
        self.create_subscription(String, '/marine/mission/command', self.command, 10)
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.1, self.tick, clock=self.steady)
        runtime = Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT', '/workspace/molar/robot_lab_runtime'))
        self.directory = runtime / 'marine_missions' / uuid.uuid4().hex
        self.directory.mkdir(parents=True)
        self.events = (self.directory / 'events.jsonl').open('a', buffering=1)
        self.event_bytes = 0
        from robot_lab_utils.map_archive import RtabmapArchive
        self.archive = RtabmapArchive(self, self.database, '/marine/slam', '/marine/mapping',
            self.directory, self.get_parameter('world_file').value, self.robot, self.map_result,
            estimator_contract='amphibious_downward_flow_range_v1' if self.robot=='lab_amphibious' else 'marine_spawn_imu_v1')
        saved = str(self.get_parameter('saved_map_source').value)
        if self.localization and saved:
            cloud = Path(saved).parent/'occupied.npz'
            if cloud.is_file():
                with np.load(cloud, allow_pickle=False) as stored:
                    points=stored['points']
                    if self.shore is not None:points=self.shore.above_support(points)
                    self.observed = VoxelObstacles(points, voxel=.2)

    def observe_localization(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        age=self.get_clock().now().nanoseconds*1e-9-stamp
        covariance=np.asarray(msg.pose.covariance)
        if (msg.header.frame_id == 'map' and np.isfinite([p.x,p.y,p.z,q.x,q.y,q.z,q.w]).all()
                and abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)<.01
                and np.isfinite(covariance).all() and min(covariance[[0,7,14]])>=0.
                and max(covariance[[0,7,14]])<=.25 and -.2<=age<=3.
                and stamp>self.localized_stamp):
            self.localized_at,self.localized_stamp=time.monotonic(),stamp

    def map_result(self, phase, reason, result):
        self.reason = reason
        self.event('map_saved' if phase == 'saved' else 'map_save_failed', result=result, reason=reason)

    def event(self, name, **data):
        row = json.dumps(dict(event=name, wall=time.time(), position=self.position.tolist() if self.position is not None else None,
                              **data), allow_nan=False) + '\n'
        if self.event_bytes + len(row) < 8 * 1024 * 1024:
            self.events.write(row)
            self.event_bytes += len(row)

    def observe_native(self, msg):
        try:
            data = json.loads(msg.data)
            if data['robot_id'] != self.robot:
                return
            if self.native and (data['session_id'] != self.native['session_id'] or data.get('operator_id') != self.native.get('operator_id')):
                self.cancel('Native marine session/operator changed')
            self.native, self.native_at = data, time.monotonic()
        except (ValueError, KeyError, TypeError):
            return

    def observe_current(self, msg):
        """Record finite authored current commands without inventing a sensor."""
        values = (msg.x, msg.y, msg.z)
        if all(math.isfinite(value) for value in values):
            self.current_vel = values
            self.current_at = time.monotonic()

    def observe_pose(self, msg):
        try:
            transform = self.tf.lookup_transform('map', msg.header.frame_id,
                rclpy.time.Time.from_msg(msg.header.stamp), timeout=rclpy.duration.Duration(seconds=0.))
            t, q = transform.transform.translation, transform.transform.rotation
            matrix = Rotation.from_quat([q.x, q.y, q.z, q.w])
            p, orientation = msg.pose.pose.position, msg.pose.pose.orientation
            values = np.array([p.x, p.y, p.z, orientation.x, orientation.y, orientation.z, orientation.w])
            if not np.isfinite(values).all():
                return
            self.position = matrix.apply(values[:3]) + [t.x, t.y, t.z]
            combined = matrix * Rotation.from_quat(values[3:])
            self.orientation = combined.as_euler('xyz')
            body = [getattr(msg.twist.twist.linear, k) for k in 'xyz']
            self.velocity = combined.apply(body)
            self.angular_velocity = float(msg.twist.twist.angular.z)
            self.pose_std = math.sqrt(max(0., msg.pose.covariance[0], msg.pose.covariance[7], msg.pose.covariance[14]))
            self.pose_at = time.monotonic()
        except (TransformException, ValueError):
            return

    def observe_cloud(self, msg):
        if msg.header.frame_id != 'map' or msg.width * msg.height > 300000 or time.monotonic() - self.cloud_at < 1.:
            return
        # RTAB-Map uses padded XYZ/RGB records; structured numpy.void values
        # cannot be cast directly to float (and crashed the first live run).
        try:
            points = point_cloud2.read_points_numpy(msg, field_names=['x','y','z'], skip_nans=True).astype(float)
        except (ValueError,TypeError,AssertionError) as exc:
            self.reason='Observed cloud conversion refused: '+str(exc)
            return
        if len(points):
            try:
                if self.shore is not None:points=self.shore.above_support(points)
                self.observed = VoxelObstacles(points, voxel=.2)
                self.cloud_at = time.monotonic()
            except ValueError as exc:
                self.reason = 'Observed cloud refused: ' + str(exc)

    def fresh(self):
        now = time.monotonic()
        return (now - self.pose_at < .6 and now - self.native_at < .6 and self.native is not None
                and self.native.get('ready') and self.pose_std < .25
                and (not self.localization or now-self.localized_at < 3.))

    def control(self, op, **extra):
        if self.native:
            self.control_pub.publish(String(data=json.dumps(dict(session_id=self.native['session_id'],
                operator_id=self.native['operator_id'], op=op, **extra), allow_nan=False)))

    def cancel(self, reason):
        self.stop_token.set()
        self.generation += 1
        self.route = None
        self.auv_follower = None
        self.auv_plan = None
        self.status, self.reason = ('canceled' if self.status in ('executing','planning','replanning','braking','coasting') else 'idle'), reason
        self.drive_pub.publish(Twist())
        if self.native and self.native.get('input_owner') == 'mission':
            self.control('stop')
        self.event('interrupted', reason=reason)

    def command(self, msg):
        try:
            if len(msg.data) > 4096:
                raise ValueError('Marine mission command too large')
            data = json.loads(msg.data)
            if not self.native or data.get('session_id') != self.native['session_id']:
                return
            if data['op'] == 'cancel':
                self.cancel('Operator canceled mission')
                return
            if (not self.get_parameter('control_allowed').value or data.get('operator_id') != self.native['operator_id']
                    or not self.native.get('operator_fresh')):
                raise ValueError('Live marine operator and Marine mode required')
            if data['op'] == 'save_map':
                self.save_map()
                return
            if not self.fresh():
                raise ValueError('Fresh pressure/DVL/IMU odometry and a valid map transform required')
            if data['op'] == 'plan':
                if self.robot=='lab_amphibious' and self.shore is None and (not self.native.get('native_plant') or self.native['native_plant']['wet_fraction']<.1):
                    raise ValueError('Amphibious autonomous routes currently require immersion; use manual Drive for the shore transition')
                if self.future is not None:
                    raise ValueError('Wait for the current planner to stop')
                self.cancel('New route requested')
                target = [float(data[k]) for k in ('x', 'y', 'z')]
                if not np.isfinite(target).all():
                    raise ValueError('Finite target XYZ required')
                self.stop_token = threading.Event()
                self.status = 'planning'
                start, orientation = self.position.copy(), self.orientation.copy()
                if self.surface_vessel and self.shore is None and abs(target[2]-start[2])>.1:
                    raise ValueError('Surface vessel goals must keep the estimated draft; use Waypoint ahead or the estimated Z')
                scene = self.shore.with_observation(self.observed) if self.shore else ObservedScene(self.scene, self.observed) if self.observed else self.scene
                radius = self.radius + 3 * self.pose_std
                token, generation = self.stop_token, self.generation
                self.future_generation, self.future_kind = generation, 'plan'
                observed=self.observed
                goal_heading=float(data['goal_heading']) if data.get('goal_heading') is not None else None
                turn_radius=float(data.get('turn_radius',12.))
                # Operator-authored current (the same vector Gazebo applies); routes are ranked by
                # time through it. Stale or absent currents plan as still water.
                current=(list(self.current_vel) if self.current_vel is not None
                         and time.monotonic()-self.current_at<2. else None)
                def compute():
                    if self.auv:
                        from robot_lab_utils.auv_routes import plan_auv
                        route,info=plan_auv(scene,start,orientation[2],target,radius,self.bottom,self.surface,
                            token.is_set,goal_heading=goal_heading,turn_radius=turn_radius,current=current)
                    elif self.shore is not None:
                        route,info=self.shore.route(start,target,radius,token.is_set,observation=observed)
                    else:route, info = plan(scene, start, target, radius=radius, resolution=.4, timeout=15.,
                        canceled=token.is_set, min_altitude=self.bottom + radius + .1,
                        max_altitude=self.surface - radius - .1) if not self.surface_vessel else plan(
                            scene,start,target,radius=radius,resolution=.4,timeout=15.,canceled=token.is_set,
                            min_altitude=float(start[2])-.1,max_altitude=float(start[2])+.1)
                    return generation, route, info, start.tolist()
                self.future = self.worker.submit(compute)
            elif data['op'] == 'execute':
                if self.status != 'planned' or not self.route or not self.native.get('enabled'):
                    raise ValueError('Plan a route and Arm native marine control before Execute')
                if math.dist(self.position, self.route[0]) > .25:
                    raise ValueError('Vehicle moved since planning; plan again')
                if self.auv and self.auv_follower is None:
                    raise ValueError('AUV needs a curvature-checked forward route before Execute')
                self.control('mission_claim')
                self.status, self.reason, self.index = 'executing', 'Estimated-pose route active', 1
                budget=max(180.,min(600.,self.auv_plan['length_m']/.2*3.)) if self.auv else 180.
                self.deadline, self.arrival_at = time.monotonic()+budget, 0.
                self.claim_until = time.monotonic()+.6
                self.event('execute', route=self.route, observer='sensor-only odometry / optional RTAB-Map transform')
            else:
                raise ValueError('Unknown marine mission operation')
        except (ValueError, KeyError, TypeError, OSError, sqlite3.Error) as exc:
            self.reason = str(exc)
            self.event('rejected', reason=str(exc))

    def save_map(self):
        if self.status in ('planning', 'replanning', 'executing', 'braking', 'coasting'):
            raise ValueError('Cancel/finish the active marine route before flushing its SLAM map')
        self.archive.request()
        self.reason = self.archive.message

    def _try_replan(self, reason):
        """Attempt to replan the active route from the current estimated position.

        Only replans if the last replan attempt was more than 5 s ago (to avoid
        thrashing on persistent obstacles) and the route has a goal to reach.
        Obstacle-aware replanning does not claim dynamic obstacle qualification;
        it is a best-effort re-plan over the known static scene plus the latest
        observed point cloud.
        """
        # Never leave the previous positive velocity latched at a newly
        # blocked segment, including while waiting out the retry cooldown.
        self.drive_pub.publish(Twist())
        if self.auv:
            self.cancel('AUV curve blocked; propulsion stopped. Explicit curvature-constrained replan required')
            return
        now = time.monotonic()
        self.reason = reason
        if now - self.replan_at < 5. or self.future is not None:
            return
        if not self.route or len(self.route) < 2:
            return
        goal = np.asarray(self.route[-1]).copy()
        start = self.position.copy()
        scene = self.shore.with_observation(self.observed) if self.shore else ObservedScene(self.scene, self.observed) if self.observed else self.scene
        radius = self.radius + 3 * self.pose_std
        self.stop_token = token = threading.Event()
        self.generation += 1
        generation = self.generation
        self.future_generation, self.future_kind = generation, 'replan'
        observed = self.observed
        self.replan_at = now
        self.status, self.reason = 'replanning', reason
        self.event('replan_started', goal=goal, reason=reason)

        def compute():
            if self.shore is not None:
                route, info = self.shore.route(start, goal, radius, token.is_set, observation=observed)
            else:
                route, info = plan(scene, start, goal, radius=radius, resolution=.4, timeout=15.,
                canceled=token.is_set,
                min_altitude=self.bottom + radius + .1,
                max_altitude=self.surface - radius - .1) if not self.surface_vessel else plan(
                    scene, start, np.asarray(goal), radius=radius, resolution=.4, timeout=15.,
                    canceled=token.is_set,
                    min_altitude=float(start[2]) - .1, max_altitude=float(start[2]) + .1)
            return generation, route, info, start.tolist()

        self.future = self.worker.submit(compute)

    def _poll_plan(self, now):
        if self.future is not None and self.future.done():
            future, generation, kind = self.future, self.future_generation, self.future_kind
            self.future = None
            if generation != self.generation:
                self.event('planner_result_discarded', reason='Canceled/replaced generation')
                return
            try:
                result_generation, route, info, start = future.result()
                if result_generation != self.generation or self.stop_token.is_set():
                    return
                if kind == 'replan':
                    if (self.status != 'replanning' or not self.fresh() or not self.native.get('operator_fresh')
                            or not self.native.get('enabled') or self.native.get('input_owner') != 'mission'
                            or now > self.deadline or math.dist(self.position, start) > .25):
                        self.cancel('Replan ownership, observation, start position or deadline changed; explicit Plan/Execute required')
                        return
                    self.route, self.status = route, 'executing'
                    self.index = 1
                    self.reason = 'Route replanned around observed obstacle; execution resumed'
                    self.event('replan_completed', route=route, info=info)
                elif self.status == 'planning':
                    if getattr(self,'auv',False):
                        if info.get('kind')!='auv_dubins':raise ValueError('Unvalidated AUV path kind')
                        from robot_lab_utils.auv_routes import AuvPathFollower
                        self.auv_follower=AuvPathFollower(route,info['headings_rad'])
                        self.auv_plan=info
                    self.route, self.status = route, 'planned'
                    self.reason = 'Source collision route planned; explicit Execute required'
                    self.event('planned', route=route, info=info)
            except Exception as exc:
                reason = ('Replan failed: ' if kind == 'replan' else 'Planning failed: ') + str(exc)
                self.cancel(reason)
                self.event('replan_failed' if kind == 'replan' else 'plan_failed', reason=reason)

    def tick(self):
        now = time.monotonic()
        if self.status in ('planning', 'planned', 'replanning'):
            if not self.fresh() or not self.native.get('operator_fresh'):
                self.cancel('Marine planner observation/operator lost; explicit Plan/Execute required')
            elif self.status == 'replanning':
                # Keep the native mission input watchdog alive with neutral
                # input while retaining every other ownership/time barrier.
                self.drive_pub.publish(Twist())
                if not self.native.get('enabled') or self.native.get('input_owner') != 'mission' or now > self.deadline:
                    self.cancel('Marine replanning control ownership/time budget lost')
        self._poll_plan(now)
        if self.status in ('executing', 'braking', 'coasting'):
            if not self.fresh() or not self.native.get('operator_fresh') or now > self.deadline:
                self.cancel('Marine mission sensor/operator/time budget lost')
            elif self.status == 'braking':
                if not self.native.get('enabled') or self.native.get('input_owner')!='mission':
                    self.cancel('AUV braking ownership/control lost; propulsion stopped')
                else:
                    # Zero desired surge while still armed uses the existing
                    # signed propeller velocity feedback to brake. A user Stop
                    # remains zero thrust and passive coast; it is distinct
                    # from a controlled autonomous arrival.
                    self.drive_pub.publish(Twist())
                    if np.linalg.norm(self.velocity)<.05:
                        if math.dist(self.position,self.route[-1])>.6:
                            self.cancel('AUV braking settled outside the estimated goal envelope')
                        else:
                            self.control('stop')
                            self.status,self.reason='completed','AUV signed-thrust braking settled; propulsion stopped'
                            self.event('estimated_arrival',route=self.route,arrival='signed-thrust braking',physical_qualification=False)
            elif self.status == 'coasting':
                if np.linalg.norm(self.velocity) < .05:
                    if math.dist(self.position, self.route[-1]) > .6:
                        self.cancel('AUV coast ended outside the estimated goal envelope')
                    else:
                        self.status, self.reason = 'completed', 'AUV transect ended and estimated coasting settled'
                        self.event('estimated_arrival', route=self.route, physical_qualification=False)
            elif not self.native.get('enabled'):
                self.cancel('Native marine controller stopped; explicit rearm/replan required')
            elif self.native.get('input_owner') != 'mission' and now > self.claim_until:
                self.cancel('Marine mission ownership was not acknowledged or was replaced')
            else:
                if self.native.get('input_owner') == 'mission':
                    target = self.route[self.index]
                    auv_brake=False
                    if self.auv:
                        try:
                            command,target,self.index,auv_brake=self.auv_follower.command(self.position,self.orientation[2],self.velocity)
                        except ValueError as exc:
                            self.cancel(str(exc));return
                    scene = self.shore.with_observation(self.observed) if self.shore else ObservedScene(self.scene, self.observed) if self.observed else self.scene
                    checked_target=np.asarray(target)
                    if self.shore:
                        # The full static route was checked during planning.
                        # Recheck the next 1.5 m against current observations;
                        # tracing a 12 m route every tick exhausted the 50 ms
                        # control budget under concurrent plant load.
                        delta=checked_target-self.position;length=float(np.linalg.norm(delta))
                        checked_target=self.position+delta*min(1.,1.5/max(length,1e-9))
                    try:
                        clear = scene.segment(self.position, checked_target, self.radius+3*self.pose_std, now+.05, lambda: False)
                    except TimeoutError:
                        self.cancel('Collision recheck deadline exceeded; explicit replan required');return
                    if not clear:
                        self._try_replan('Observed segment blocked; replanning from current position')
                    else:
                        if not self.auv:command, distance = waypoint_command(self.position, self.orientation, self.velocity, target)
                        else:distance=math.dist(self.position,self.route[-1])
                        if self.surface_vessel:
                            dx,dy=target[0]-self.position[0],target[1]-self.position[1]
                            distance=math.hypot(dx,dy)
                            error=math.atan2(math.sin(math.atan2(dy,dx)-self.orientation[2]),math.cos(math.atan2(dy,dx)-self.orientation[2]))
                            command=(min(.25,.5*distance)*max(0.,math.cos(error)) if abs(error)<1. else 0.,
                                     0.,0.,max(-.3,min(.3,.9*error)))
                        arrival=(max(.35,min(1.2,2.5*float(np.linalg.norm(self.velocity))+.1)) if self.auv else .08 if self.shore else .15)
                        if self.auv and auv_brake:
                            self.status,self.reason='braking','Curved AUV route terminal corridor reached; signed-thrust braking'
                            self.event('auv_braking',speed_m_s=float(np.linalg.norm(self.velocity)),remaining_m=distance)
                            command=(0.,0.,0.,0.)
                        elif not self.auv and distance < arrival:
                            if self.index < len(self.route)-1:
                                self.index += 1
                                self.arrival_at = 0.
                            elif self.auv:
                                self.status, self.reason = 'braking', 'Zero desired surge; signed-thrust braking before propulsion Stop'
                                self.event('auv_braking',speed_m_s=float(np.linalg.norm(self.velocity)),remaining_m=distance)
                            elif np.linalg.norm(self.velocity) < .04 and (not self.surface_vessel or abs(self.angular_velocity)<.04):
                                self.arrival_at = self.arrival_at or now
                                if now - self.arrival_at > .5:
                                    self.control('mission_release')
                                    self.status, self.reason = 'completed', 'Estimated goal and velocity settled; native depth hold retained'
                                    self.event('estimated_arrival', route=self.route, physical_qualification=False)
                            else:
                                self.arrival_at = 0.
                            command = (0., 0., 0., 0.)
                        output = Twist()
                        output.linear.x, output.linear.y, output.linear.z, output.angular.z = command
                        self.drive_pub.publish(output)
        data = dict(status=self.status, reason=self.reason, robot_id=self.robot, session_id=self.native['session_id'] if self.native else None,
            ready=self.fresh(), position=self.position.tolist() if self.position is not None else None,
            estimated_yaw=float(self.orientation[2]) if self.orientation is not None else None,
            route=self.route, waypoint=self.index, position_std_m=self.pose_std if math.isfinite(self.pose_std) else None,
            observed_cells=self.observed.cell_count if self.observed else 0, directory=str(self.directory),
            authored_current_m_s=list(self.current_vel) if self.current_vel is not None else None,
            authored_current_age_s=now-self.current_at if math.isfinite(self.current_at) else None,
            current_cost_planning=False,
            route_kind='auv_dubins' if self.auv_plan else 'source_collision_route',
            auv_turn_radius_m=self.auv_plan['turn_radius_m'] if self.auv_plan else None,
            auv_cross_track_m=self.auv_follower.cross_track if self.auv_follower else None,
            localization_mode=self.localization, localized=self.localization and now-self.localized_at < 3.,
            mapping_status=self.archive.phase, saved_map=self.archive.result,
            estimator=('Authored downward motion/range simulation + native IMU/source support map; no image matching' if self.robot=='lab_amphibious' else 'Pressure/DVL simulation + native IMU; optional real RGB-D SLAM map transform'),
            scope='Experimental estimated-pose mission; no independent physical navigation qualification')
        self.state_pub.publish(String(data=json.dumps(data, allow_nan=False)))

    def close(self):
        if rclpy.ok():
            self.cancel('Marine navigation shutdown')
        else:
            self.stop_token.set()
        self.worker.shutdown(wait=True, cancel_futures=True)
        self.events.close()


def main(args=None):
    from robot_lab_utils.process_lifetime import exit_with_parent
    exit_with_parent(); rclpy.init(args=args)
    node = MarineNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
