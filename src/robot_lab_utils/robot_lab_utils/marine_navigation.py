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
from geometry_msgs.msg import Twist
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
                               radius=.55, database_path='', control_allowed=True).items():
            self.declare_parameter(key, value)
        self.robot = self.get_parameter('robot_id').value
        self.auv = self.robot == 'tethys_auv'
        self.surface = float(self.get_parameter('surface_z').value)
        self.bottom = float(self.get_parameter('bottom_z').value)
        self.radius = float(self.get_parameter('radius').value)
        self.database = str(self.get_parameter('database_path').value)
        self.scene = StaticScene(str(self.get_parameter('world_file').value))
        self.observed = None
        self.cloud_at = -math.inf
        self.native = self.position = self.orientation = self.velocity = None
        self.pose_at = self.native_at = -math.inf
        self.pose_std = math.inf
        self.status = 'idle'
        self.reason = 'Plan from sensor odometry; explicit Execute required'
        self.route = None
        self.index = 0
        self.arrival_at = 0.
        self.deadline = 0.
        self.claim_until = 0.
        self.generation = 0
        self.stop_token = threading.Event()
        self.future = None
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='marine-plan')
        self.tf = Buffer(node=self)
        self.listener = TransformListener(self.tf, self)
        self.drive_pub = self.create_publisher(Twist, '/marine/mission_vel', 10)
        self.control_pub = self.create_publisher(String, '/marine/command', 10)
        self.state_pub = self.create_publisher(String, '/marine/mission/state', 10)
        self.create_subscription(String, '/marine/state', self.observe_native, 10)
        self.create_subscription(Odometry, '/marine/estimated_odometry', self.observe_pose, 10)
        self.create_subscription(PointCloud2, '/marine/slam/cloud_map', self.observe_cloud, 10)
        self.create_subscription(String, '/marine/mission/command', self.command, 10)
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.1, self.tick, clock=self.steady)
        runtime = Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT', '/workspace/molar/robot_lab_runtime'))
        self.directory = runtime / 'marine_missions' / uuid.uuid4().hex
        self.directory.mkdir(parents=True)
        self.events = (self.directory / 'events.jsonl').open('a', buffering=1)
        self.event_bytes = 0

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
            if self.native and data['session_id'] != self.native['session_id']:
                self.cancel('Native marine session changed')
            self.native, self.native_at = data, time.monotonic()
        except (ValueError, KeyError, TypeError):
            return

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
                self.observed = VoxelObstacles(points, voxel=.2)
                self.cloud_at = time.monotonic()
            except ValueError as exc:
                self.reason = 'Observed cloud refused: ' + str(exc)

    def fresh(self):
        now = time.monotonic()
        return (now - self.pose_at < .6 and now - self.native_at < .6 and self.native is not None
                and self.native.get('ready') and self.pose_std < .25)

    def control(self, op, **extra):
        if self.native:
            self.control_pub.publish(String(data=json.dumps(dict(session_id=self.native['session_id'],
                operator_id=self.native['operator_id'], op=op, **extra), allow_nan=False)))

    def cancel(self, reason):
        self.stop_token.set()
        self.generation += 1
        self.route = None
        self.status, self.reason = 'idle', reason
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
                if self.future is not None:
                    raise ValueError('Wait for the current planner to stop')
                self.cancel('New route requested')
                target = [float(data[k]) for k in ('x', 'y', 'z')]
                if not np.isfinite(target).all():
                    raise ValueError('Finite target XYZ required')
                self.stop_token = threading.Event()
                self.status = 'planning'
                start, orientation = self.position.copy(), self.orientation.copy()
                scene = ObservedScene(self.scene, self.observed) if self.observed else self.scene
                radius = self.radius + 3 * self.pose_std
                token, generation = self.stop_token, self.generation
                def compute():
                    route, info = plan(scene, start, target, radius=radius, resolution=.4, timeout=15.,
                        canceled=token.is_set, min_altitude=self.bottom + radius + .1,
                        max_altitude=self.surface - radius - .1)
                    if self.auv:
                        # This controller has no stop-and-pivot capability. Only
                        # source-clear, forward-aligned cruise transects qualify
                        # for initial execution; arbitrary A* corners are refused.
                        bearing = math.atan2(target[1]-start[1], target[0]-start[0])
                        error = math.atan2(math.sin(bearing-orientation[2]-math.pi), math.cos(bearing-orientation[2]-math.pi))
                        if len(route) != 2 or abs(error) > .25 or np.linalg.norm(np.array(target[:2])-start[:2]) < 2.:
                            raise ValueError('AUV currently requires a direct forward cruise transect ≥2 m and within 0.25 rad; no pivot/corner execution')
                        direction = (np.asarray(target)-start) / np.linalg.norm(np.asarray(target)-start)
                        if not scene.segment(target, np.asarray(target)+3*direction, radius, time.monotonic()+2., token.is_set):
                            raise ValueError('AUV endpoint needs a 3 m clear coasting corridor')
                    return generation, route, info, start.tolist()
                self.future = self.worker.submit(compute)
            elif data['op'] == 'execute':
                if self.status != 'planned' or not self.route or not self.native.get('enabled'):
                    raise ValueError('Plan a route and Arm native marine control before Execute')
                if math.dist(self.position, self.route[0]) > .25:
                    raise ValueError('Vehicle moved since planning; plan again')
                self.control('mission_claim')
                self.status, self.reason, self.index = 'executing', 'Estimated-pose route active', 1
                self.deadline, self.arrival_at = time.monotonic()+180., 0.
                self.claim_until = time.monotonic()+.6
                self.event('execute', route=self.route, observer='sensor-only odometry / optional RTAB-Map transform')
            else:
                raise ValueError('Unknown marine mission operation')
        except (ValueError, KeyError, TypeError, OSError, sqlite3.Error) as exc:
            self.reason = str(exc)
            self.event('rejected', reason=str(exc))

    def save_map(self):
        source = Path(self.database)
        if not self.database or not source.is_file() or not self.observed:
            raise ValueError('Start RGB-D SLAM and acquire a real 3D cloud before saving')
        output = self.directory / ('map-' + uuid.uuid4().hex[:8])
        output.mkdir()
        with sqlite3.connect('file:' + str(source) + '?mode=ro', uri=True, timeout=2.) as src:
            with sqlite3.connect(output / 'rtabmap.db') as dst:
                src.backup(dst, pages=256)
        np.savez_compressed(output / 'occupied.npz', points=self.observed.centres, voxel=.2)
        (output / 'manifest.json').write_text(json.dumps(dict(robot=self.robot,
            source_world=self.get_parameter('world_file').value, source_database=str(source),
            points=len(self.observed.centres), observer='DVL/pressure emulation + native IMU + actual RGB-D RTAB-Map',
            scope='Saved actual SLAM database/cloud; reload/navigation qualification pending'), indent=2)+'\n')
        self.reason = 'Saved actual database and occupied cloud: ' + str(output)
        self.event('map_saved', directory=str(output), points=len(self.observed.centres))

    def tick(self):
        now = time.monotonic()
        if self.future is not None and self.future.done():
            try:
                generation, route, info, start = self.future.result()
                if generation == self.generation:
                    self.route, self.status = route, 'planned'
                    self.reason = 'Source collision route planned; explicit Execute required'
                    self.event('planned', route=route, info=info)
            except Exception as exc:
                self.status, self.reason = 'idle', 'Planning failed: ' + str(exc)
                self.event('plan_failed', reason=str(exc))
            self.future = None
        if self.status in ('executing', 'coasting'):
            if not self.fresh() or not self.native.get('operator_fresh') or now > self.deadline:
                self.cancel('Marine mission sensor/operator/time budget lost')
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
                    scene = ObservedScene(self.scene, self.observed) if self.observed else self.scene
                    try:
                        clear = scene.segment(self.position, target, self.radius+3*self.pose_std, now+.05, lambda: False)
                    except TimeoutError:
                        clear = False
                    if not clear:
                        self.cancel('Current source/observed segment blocked; replan required')
                    else:
                        command, distance = waypoint_command(self.position, self.orientation, self.velocity, target, self.auv)
                        if distance < (.35 if self.auv else .15):
                            if self.index < len(self.route)-1:
                                self.index += 1
                                self.arrival_at = 0.
                            elif self.auv:
                                self.control('stop')
                                self.status, self.reason = 'coasting', 'Propulsion stopped; waiting for measured estimated coast'
                            elif np.linalg.norm(self.velocity) < .04:
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
            route=self.route, waypoint=self.index, position_std_m=self.pose_std if math.isfinite(self.pose_std) else None,
            observed_cells=self.observed.cell_count if self.observed else 0, directory=str(self.directory),
            estimator='Pressure/DVL simulation + native IMU; optional real RGB-D SLAM map transform',
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
