"""Coordinated Nav2 and measured native articulation; no synthetic object success.

The executor lives with the native plant so base/arm ownership changes are
atomic with physics. Nav2 is an asynchronous ROS action, never a pose write.
"""
import hashlib
import json
import math
import os
from pathlib import Path
import time
import uuid

import numpy as np
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time as TimeMessage
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from std_msgs.msg import String
from std_srvs.srv import Trigger

from robot_lab_utils.mobile_task import validate_task


class NativeMobileTasks:
    def __init__(self, node):
        self.node, self.mobile, self.arm = node, node.mobile, node.articulation
        self.steps, self.handle, self.pending = None, None, False
        self.nav_guard_until, self.last_heartbeat = 0., -math.inf
        self.index, self.generation, self.task_id = 0, 0, ''
        self.events, self.event_path = [], None
        self.started, self.phase_started, self.settled_since = 0., 0., None
        digest = hashlib.sha256()
        world = node.get_parameter('world_xml').value
        if world:
            digest.update(Path(world).read_bytes())
        for key in ('geom_type', 'geom_size', 'geom_pos', 'geom_quat', 'geom_bodyid',
                    'geom_contype', 'geom_conaffinity', 'mesh_vert', 'mesh_face',
                    'mesh_pos', 'mesh_quat', 'flex_vert', 'flex_elem', 'flex_vertbodyid',
                    'hfield_data', 'body_pos', 'body_quat'):
            digest.update(np.ascontiguousarray(getattr(node.model, key)).tobytes())
        self.scene_id = digest.hexdigest()
        self.state = dict(state='idle', reason='', task_id='', step=0, total=0)
        self.client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        self.publisher = node.create_publisher(String, '/mobile_task/state', 10)
        node.create_subscription(String, '/mobile_task/command', self.receive, 1)
        node.create_subscription(String, '/mobile_task/heartbeat', self.heartbeat, 1)
        # During a task only Nav2's original output can command the base.
        # The mux output remains available for ordinary operator workflows.
        node.create_subscription(Twist, '/cmd_vel', self.navigation_velocity, 1)
        node.create_subscription(Twist, '/key_vel', self.intervention, 1)
        node.create_subscription(Twist, '/input_joy/cmd_vel', self.intervention, 1)
        node.create_subscription(PoseStamped, '/goal_pose', self.external_goal, 1)
        node.create_service(Trigger, '/mobile_task/stop', self.stop)
        node.create_timer(.1, self.tick, clock=Clock(clock_type=ClockType.SYSTEM_TIME))

    def active(self):
        return self.steps is not None

    def blocks_mux(self):
        return self.active() or self.pending or self.handle is not None or time.monotonic() < self.nav_guard_until

    def pose(self):
        p = self.node.data.xpos[self.mobile.base]
        rotation = self.node.data.xmat[self.mobile.base].reshape(3, 3)
        return [float(p[0]), float(p[1]), math.atan2(rotation[1, 0], rotation[0, 0])]

    def heartbeat(self, message):
        if message.data == self.task_id and self.active():
            self.last_heartbeat = time.monotonic()

    def intervention(self, _message):
        if self.active():
            self.cancel('Operator Drive/Space/joystick intervention')

    def external_goal(self, _message):
        if self.active():
            self.cancel('External RViz navigation goal intervention')

    def navigation_velocity(self, message):
        if (self.active() and self.state['state'] == 'navigating' and self.handle is not None
                and not self.pending and time.monotonic()-self.last_heartbeat < .8):
            self.mobile.receive(message, task_command=True)

    def receive(self, message):
        try:
            if len(message.data) > 2*1024*1024:
                raise ValueError('Task exceeds the 2 MB command bound.')
            request = json.loads(message.data)
            if not isinstance(request, dict) or request.get('plant_id') != self.arm.plant_id:
                return
            if request.get('operation') == 'cancel':
                self.cancel('Operator canceled task')
                return
            if request.get('operation') != 'run' or self.blocks_mux():
                raise ValueError('Stop the current task/navigation before starting another task.')
            if self.arm.fault or self.mobile.fault or self.mobile.moving() or self.mobile.manipulator_busy():
                raise ValueError('Wait for measured base/articulation to stop and clear plant faults.')
            task_id = request.get('task_id')
            if not isinstance(task_id, str) or not 1 <= len(task_id) <= 80:
                raise ValueError('A unique operator task ID is required.')
            steps = validate_task(request.get('task'), self.arm.source_sha256, self.scene_id,
                self.arm.channels, self.node.get_parameter('native_task').value == 'nav')
            if any(step['kind'] == 'navigate' for step in steps) and not self.client.server_is_ready():
                raise ValueError('The selected-world Nav2 action server is not ready.')
            directory = Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT', '/workspace/molar/robot_lab_runtime'))/'mobile-tasks'
            directory.mkdir(parents=True, exist_ok=True)
            if directory.stat().st_dev == Path('/').stat().st_dev:
                raise ValueError('Mobile task journals must live on the mounted SSD.')
            self.event_path = directory/(time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex+'.jsonl')
            self.event_path.touch(exist_ok=False)
            self.task_id, self.steps, self.events = task_id, steps, []
            self.generation += 1
            self.index, self.started, self.last_heartbeat = 0, time.monotonic(), time.monotonic()
            self.arm.hold('Mobile task owns articulation')
            self.enter('waiting_for_settle', 'Task accepted; waiting for measured stopped plant')
            self.record(task_document=request['task'])
        except (ValueError, TypeError, KeyError, OSError) as exc:
            if self.active():
                self.cancel(str(exc), failed=True)
            else:
                self.state.update(state='rejected', reason=str(exc))

    def enter(self, state, reason):
        self.state.update(state=state, reason=reason, task_id=self.task_id,
            step=self.index+1 if self.steps else self.index, total=len(self.steps) if self.steps else self.state.get('total', 0))
        self.phase_started, self.settled_since = time.monotonic(), None
        self.record()

    def record(self, **measurement):
        event = dict(self.state, sim_time=float(self.node.data.time), wall_time=time.time(),
            physical_pose=self.pose(), plant_id=self.arm.plant_id,
            source_sha256=self.arm.source_sha256, scene_id=self.scene_id, **measurement)
        self.events.append(event)
        if self.event_path:
            try:
                with self.event_path.open('a') as stream:
                    stream.write(json.dumps(event, allow_nan=False)+'\n')
            except (OSError, ValueError) as exc:
                self.event_path = None
                self.cancel('Task journal failed: '+str(exc), failed=True)

    def halt_base(self):
        self.mobile.command[:] = 0
        self.mobile.received = time.monotonic()
        # The original acceleration-limited controller stops the actual wheels.

    def cancel(self, reason, failed=False):
        was_active = self.active()
        self.steps = None
        self.generation += 1
        self.halt_base()
        self.arm.hold(reason)
        if self.handle:
            try:
                self.handle.cancel_goal_async()
            except Exception as exc:
                reason += '; Nav2 cancel failed: '+str(exc)
        if was_active:
            self.enter('failed' if failed else 'interrupted', reason)

    def stop(self, _request, response):
        self.cancel('Stop mobile task requested')
        response.success, response.message = True, 'Base stop and arm hold requested; Nav2 cancellation is asynchronous.'
        return response

    def begin_navigation(self, step):
        if not self.mobile.stowed():
            self.cancel('Retract the arm before the next navigation step.', failed=True)
            return
        goal = NavigateToPose.Goal()
        nanoseconds = round(float(self.node.data.time)*1e9)
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = TimeMessage(sec=nanoseconds//10**9, nanosec=nanoseconds%10**9)
        x, y, yaw = step['pose']
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        generation = self.generation
        self.pending = True
        self.enter('navigating', 'Nav2 goal requested; arm held and retracted')
        try:
            future = self.client.send_goal_async(goal)
            future.add_done_callback(lambda future: self.accepted(future, generation))
        except Exception as exc:
            self.pending = False
            self.cancel('Nav2 goal request failed: '+str(exc), failed=True)

    def accepted(self, future, generation):
        self.pending = False
        try:
            handle = future.result()
            if not handle.accepted:
                if self.active() and generation == self.generation:
                    self.cancel('Nav2 rejected goal', failed=True)
                return
            self.handle = handle
            handle.get_result_async().add_done_callback(lambda future: self.navigation_result(future, handle, generation))
            if not self.active() or generation != self.generation:
                handle.cancel_goal_async()
        except Exception as exc:
            self.cancel('Nav2 response failed: '+str(exc), failed=True)

    def navigation_result(self, future, handle, generation):
        if self.handle is handle:
            self.handle = None
        self.nav_guard_until = time.monotonic()+.5
        self.halt_base()
        if not self.active() or generation != self.generation:
            return
        try:
            status = future.result().status
            self.record(nav2_result=int(status))
            if status != GoalStatus.STATUS_SUCCEEDED:
                self.cancel('Nav2 task ended with status '+str(status), failed=True)
                return
            self.enter('navigation_settle', 'Nav2 succeeded; checking physical pose and stopped base')
        except Exception as exc:
            self.cancel('Nav2 result failed: '+str(exc), failed=True)

    def settled(self):
        stopped = (not self.mobile.moving() and not self.mobile.manipulator_busy()
            and np.all(np.abs(self.arm.measured_rates()) <= .05*np.asarray([c['max_rate'] for c in self.arm.channels])))
        if not stopped:
            self.settled_since = None
        elif self.settled_since is None:
            self.settled_since = self.node.data.time
        return stopped and self.node.data.time-self.settled_since >= 1.

    def advance(self):
        self.record(step_result='measured_step_reached')
        self.index += 1
        if self.index == len(self.steps):
            self.steps = None
            self.halt_base()
            self.enter('measured_steps_reached', 'Navigation/actuator measurements reached; object/grasp outcome unassessed')
        else:
            self.enter('waiting_for_settle', 'Waiting for measured stopped plant before next step')

    def tick(self):
        if self.active():
            now = time.monotonic()
            if now-self.last_heartbeat > .8:
                self.cancel('Task operator heartbeat lost')
            elif self.arm.fault or self.mobile.fault or not np.all(np.isfinite(self.node.data.qpos)):
                self.cancel('Native plant fault/non-finite state', failed=True)
            elif now-self.started > 1800 or now-self.phase_started > (600 if self.state['state'] in ('navigating', 'articulating') else 60):
                self.cancel('Task/step wall-clock deadline exceeded', failed=True)
            else:
                step = self.steps[self.index]
                phase = self.state['state']
                if phase == 'waiting_for_settle':
                    self.halt_base()
                    if self.settled() and now >= self.nav_guard_until:
                        if step['kind'] == 'navigate':
                            self.begin_navigation(step)
                        else:
                            self.sequence_id = self.task_id+'.'+str(self.index)
                            try:
                                self.arm.start_sequence(dict(source_sha256=self.arm.source_sha256,
                                    sequence_id=self.sequence_id, waypoints=step['waypoints']))
                                self.enter('articulating', 'Replaying source-matched taught configurations with stopped base')
                            except (ValueError, TypeError, KeyError) as exc:
                                self.cancel(str(exc), failed=True)
                elif phase == 'navigation_settle':
                    pose, goal = self.pose(), step['pose']
                    error = math.dist(pose[:2], goal[:2])
                    yaw_error = abs(math.atan2(math.sin(pose[2]-goal[2]), math.cos(pose[2]-goal[2])))
                    if self.settled():
                        self.record(position_error_m=error, heading_error_rad=yaw_error)
                        if error <= .15 and yaw_error <= math.radians(5):
                            self.advance()
                        else:
                            self.cancel('Nav2 result differs from physical map pose by >0.15 m / 5 degrees', failed=True)
                elif phase == 'articulating':
                    feedback = self.arm.sequence_state
                    if self.mobile.moving() or feedback.get('sequence_id') != self.sequence_id:
                        self.cancel('Articulation ownership/base movement changed', failed=True)
                    elif feedback['state'] == 'measured_waypoints_reached':
                        self.advance()
                    elif feedback['state'] != 'executing':
                        self.cancel('Articulation interrupted: '+feedback.get('reason', ''), failed=True)
        self.publisher.publish(String(data=json.dumps(dict(self.state,
            plant_id=self.arm.plant_id, source_sha256=self.arm.source_sha256, scene_id=self.scene_id,
            physical_pose=self.pose(), active=self.active(), nav_cancel_pending=bool(self.pending or self.handle),
            navigation_available=self.node.get_parameter('native_task').value == 'nav' and self.client.server_is_ready(),
            journal=str(self.event_path) if self.event_path else '', object_outcome='unassessed',
            qualification='implemented candidate; physical task validation pending'))))
