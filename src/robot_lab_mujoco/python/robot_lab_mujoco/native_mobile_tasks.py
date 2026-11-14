"""Coordinated Nav2 and measured native articulation; no synthetic object success.

The executor lives with the native plant so base/arm ownership changes are
atomic with physics. Nav2 is an asynchronous ROS action, never a pose write.
"""
import hashlib
import copy
import json
import math
import os
from pathlib import Path
import time
import uuid
import threading
from types import SimpleNamespace

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
from robot_lab_utils.object_custody import ObjectCustody


class NativeMobileTasks:
    def __init__(self, node):
        self.node, self.mobile, self.arm = node, node.mobile, node.articulation
        self.steps, self.handle, self.pending = None, None, False
        self.object_candidates, self.finger_candidates = [], []
        for body in range(2, node.model.nbody):
            name = node.mujoco.mj_id2name(node.model, node.mujoco.mjtObj.mjOBJ_BODY, body)
            geoms = np.flatnonzero(node.model.geom_bodyid == body)
            physical = any(node.model.geom_contype[g] or node.model.geom_conaffinity[g] for g in geoms)
            if not name or not physical:
                continue
            if body < node.robot_bodies:
                self.finger_candidates.append(name)
            else:
                joint = node.model.body_jntadr[body]
                if joint >= 0 and node.model.jnt_type[joint] == node.mujoco.mjtJoint.mjJNT_FREE:
                    self.object_candidates.append(name)
        self.reach_worker, self.reach_result = None, None
        self.reach_orientation = None
        self.gripped_step, self.grip_lost_since = None, None
        self.payload_body, self.gripped_body, self.payload_confirmed = None, None, False
        self.custody, self.object_outcome, self.object_measurement = None, 'unassessed', {}
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
        from robot_lab_utils.object_estimates import ObjectEstimates
        self.object_estimates=ObjectEstimates()
        self.perception_estimates={'aruco_pnp':self.object_estimates,'rgbd_box_top':ObjectEstimates('rgbd_box_top')}
        self.perception_error='No camera object observations yet'
        node.create_subscription(String,'/robot_lab/object_estimates',self.receive_estimates,10)
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

    def estimate_identity(self):
        return dict(plant_id=self.arm.plant_id,source_sha256=self.arm.source_sha256,scene_id=self.scene_id)

    def receive_estimates(self,message):
        try:
            if len(message.data)>131072:raise ValueError('Camera estimates exceed message bound')
            record=json.loads(message.data)
            source=record.get('pose_source') if isinstance(record,dict) else None
            if source not in self.perception_estimates:raise ValueError('Unknown camera object source')
            self.perception_estimates[source].receive(record,self.estimate_identity(),float(self.node.data.time))
            self.perception_error=''
        except (ValueError,TypeError,KeyError) as exc:self.perception_error=str(exc)

    def object_bodies(self, step):
        model, mj = self.node.model, self.node.mujoco
        ids = [mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, step[key])
               for key in ('object', 'left_body', 'right_body')]
        obj, left, right = ids
        if obj < self.node.robot_bodies or not all(1 < b < self.node.robot_bodies for b in (left, right)):
            raise ValueError('Object must be a world body; fingers must be robot bodies')
        adr = model.body_jntadr[obj]
        if adr < 0 or model.jnt_type[adr] != mj.mjtJoint.mjJNT_FREE:
            raise ValueError('Object checkpoint requires an independently free physical object')
        # Require separate opposing subtrees; an ancestor cannot be its own opposing finger.
        def descendants(body):
            result = {body}
            for child in range(body+1, model.nbody):
                if model.body_parentid[child] in result:
                    result.add(child)
            return result
        sides = [descendants(left), descendants(right)]
        if sides[0] & sides[1]:
            raise ValueError('Opposing finger subtrees overlap')
        return obj, sides

    def measure_object(self, step):
        obj, sides = self.object_bodies(step)
        model, data, mj = self.node.model, self.node.data, self.node.mujoco
        forces, support_force = [0., 0.], 0.
        for index, contact in enumerate(data.contact[:data.ncon]):
            bodies = [int(model.geom_bodyid[g]) for g in contact.geom]
            if obj not in bodies:
                continue
            wrench = np.zeros(6)
            mj.mj_contactForce(model, data, index, wrench)
            force = float(abs(wrench[0]))
            other = bodies[1] if bodies[0] == obj else bodies[0]
            for side, members in enumerate(sides):
                if other in members:
                    forces[side] += force
            if other not in sides[0] | sides[1]:
                # Release needs support from the environment, not the robot palm/chassis.
                if step.get('condition') != 'released' or other == 0 or other >= self.node.robot_bodies:
                    support_force += force
        velocity = np.zeros(6)
        mj.mj_objectVelocity(model, data, mj.mjtObj.mjOBJ_BODY, obj, velocity, 0)
        # Relative to the first finger's frame, so base/arm motion is not slip.
        reference = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, step['left_body'])
        relative = data.xmat[reference].reshape(3, 3).T @ (data.xpos[obj]-data.xpos[reference])
        return dict(stamp=float(data.time), relative=relative.tolist(), left_force=forces[0],
                    right_force=forces[1], supported=support_force>=.1, speed=float(np.linalg.norm(velocity[3:])))

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
            self.custody, self.object_outcome, self.object_measurement = None, 'unassessed', {}
            steps = validate_task(request.get('task'), self.arm.source_sha256, self.scene_id,
                self.arm.channels, self.node.get_parameter('native_task').value == 'nav')
            for step in steps:
                if step['kind'] in ('object_check','reach_object','grip','stow_arm'):
                    self.object_bodies(step)
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
        self.reach_orientation = None
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

    def grasp_settling_mask(self):
        mask = np.zeros(len(self.arm.channels),dtype=bool)
        if not self.active() or self.gripped_step is None or self.gripped_body is None:
            return mask
        observation = self.measure_object(self.gripped_step)
        if min(observation['left_force'],observation['right_force']) < .1:
            return mask
        measured = self.arm.measured_controls()
        for index,channel in enumerate(self.arm.channels):
            if channel['name'] in ('grip','gripper') and self.arm.target[index]<=measured[index]+.001:
                mask[index] = True  # blocked by measured opposing object contacts
        return mask

    def begin_reach(self, step):
        source=step.get('pose_source','native_truth')
        if source in ('aruco_pnp','rgbd_box_top'):
            observed=self.perception_estimates[source].get(step['object'],self.estimate_identity(),float(self.node.data.time))
            obj,_=self.object_bodies(step);model=self.node.model;mj=self.node.mujoco
            if obj in (self.gripped_body,self.payload_body):
                raise ValueError('Camera object-relative targets require an unheld object; use an explicit carry/lift target')
            geoms=[g for g in range(model.ngeom) if model.geom_bodyid[g]==obj
                   and (model.geom_contype[g] or model.geom_conaffinity[g])]
            if (len(geoms)!=1 or int(model.geom_type[geoms[0]])!=int(mj.mjtGeom.mjGEOM_BOX)
                    or observed.get('type')!='box' or not np.allclose(observed.get('size'),2*model.geom_size[geoms[0]],rtol=0,atol=1e-6)):
                raise ValueError('Camera binding must match the selected source box geometry')
            from scipy.spatial.transform import Rotation
            rotation=Rotation.from_quat(np.asarray(observed['orientation_wxyz'])[[1,2,3,0]])
            step=copy.deepcopy(step)
            offset=rotation.apply(model.geom_pos[geoms[0]]) if source=='aruco_pnp' else np.zeros(3)
            step['target_xyz']=(np.asarray(observed['position'])+offset
                                +np.asarray(step.get('offset',[0.,0.,0.]))).tolist()
            self.record(perception_target=observed,pose_source=source)
        self.reach_orientation = None
        if self.reach_worker is not None and self.reach_worker.is_alive():
            raise ValueError('Previous reach planner is still stopping; retry after its bounded deadline')
        from robot_lab_mujoco.native_mobile_reach import plan_reach
        snapshot = self.node.mujoco.MjData(self.node.model)
        self.node.mujoco.mj_copyData(snapshot,self.node.model,self.node.data)
        initial = self.arm.measured_controls().copy()
        measured = initial.copy()
        if self.payload_body is not None:
            for i,c in enumerate(self.arm.channels):
                if c['name'] in ('grip','gripper'):
                    initial[i] = self.arm.target[i]  # preserve actual motor compression target
        generation = self.generation
        self.reach_result = None
        node = SimpleNamespace(model=self.node.model,mujoco=self.node.mujoco)
        channels = [dict(c) for c in self.arm.channels]
        robot_bodies, robot_joints = self.node.robot_bodies, self.node.robot_joints
        excluded, payload = list(self.mobile.joints), self.gripped_body
        def work():
            try:
                result = plan_reach(node,snapshot,channels,initial,dict(step),robot_bodies,
                                    robot_joints,excluded,payload)
                result['start_measured'] = measured.tolist()
                self.reach_result = (generation,result,'')
            except Exception as exc:
                self.reach_result = (generation,None,type(exc).__name__+': '+str(exc))
        self.enter('planning_reach','Planning a bounded source-control reach on a private scene snapshot')
        self.reach_worker = threading.Thread(target=work,daemon=True)
        self.reach_worker.start()

    def begin_grip(self, step):
        indices = [i for i,c in enumerate(self.arm.channels) if c['name'] in ('grip','gripper')]
        if len(indices)!=1:
            raise ValueError('No unique reviewed source gripper control')
        if step['operation']=='open':
            self.gripped_body, self.custody, self.payload_confirmed, self.gripped_step = None, None, False, None
        self.grip_index = indices[0]
        channel = self.arm.channels[self.grip_index]
        self.arm.target = self.arm.measured_controls().copy()
        self.arm.target[self.grip_index] = channel['limits'][1 if step['operation']=='open' else 0]
        self.arm.status = 'jogging'
        self.grip_since = None
        self.enter('gripping','Waiting for measured source gripper opening/contact')

    def begin_navigation(self, step):
        if self.payload_body is not None and not self.payload_confirmed:
            self.cancel('A measured grasp checkpoint is required before transport.',failed=True)
            return
        if not self.mobile.payload_footprint_ready():
            self.cancel('Wait for fresh measured payload footprint before navigation.', failed=True)
            return
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
        self.reach_orientation = None
        self.record(step_result='measured_step_reached')
        self.index += 1
        if self.index == len(self.steps):
            self.steps = None
            self.halt_base()
            self.enter('measured_steps_reached', 'Measured steps reached; object outcome: '+self.object_outcome)
        else:
            self.enter('waiting_for_settle', 'Waiting for measured stopped plant before next step')

    def tick(self):
        if self.active():
            now = time.monotonic()
            if now-self.last_heartbeat > .8:
                self.cancel('Task operator heartbeat lost')
            elif (self.state['state'] in ('navigating', 'navigation_settle')
                    and not self.mobile.payload_footprint_ready()):
                self.cancel('Measured payload footprint unavailable/stale', failed=True)
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
                            if self.mobile.payload_footprint_ready():
                                self.begin_navigation(step)
                            else:
                                self.state['reason'] = 'Waiting for measured grasp and fresh footprint receipts from both costmaps'
                        elif step['kind'] in ('reach_object','grip','stow_arm'):
                            try:
                                self.begin_reach(step) if step['kind'] in ('reach_object','stow_arm') else self.begin_grip(step)
                            except (ValueError,TypeError,KeyError) as exc:
                                self.cancel(str(exc),failed=True)
                        elif step['kind'] == 'object_check':
                            self.object_gate = ObjectCustody()
                            if step['condition'] == 'released':
                                self.custody = None
                            self.enter('checking_object', 'Waiting for sustained measured contact/support and low slip')
                        else:
                            self.sequence_id = self.task_id+'.'+str(self.index)
                            try:
                                self.arm.start_sequence(dict(source_sha256=self.arm.source_sha256,
                                    sequence_id=self.sequence_id, waypoints=step['waypoints']))
                                self.enter('articulating', 'Replaying source-matched taught configurations with stopped base')
                            except (ValueError, TypeError, KeyError) as exc:
                                self.cancel(str(exc), failed=True)
                elif phase == 'planning_reach':
                    self.halt_base()
                    if self.reach_result is not None:
                        generation, result, error = self.reach_result
                        self.reach_result = None
                        if generation != self.generation or error:
                            self.cancel(error or 'Reach planning generation changed',failed=True)
                        else:
                            obj,_ = self.object_bodies(step)
                            span = np.array([c['limits'][1]-c['limits'][0] for c in self.arm.channels])
                            drift = np.abs(self.arm.measured_controls()-result['start_measured'])
                            if (self.mobile.moving() or np.any(drift>np.maximum(.002,.01*span))
                                    or np.linalg.norm(self.node.data.xpos[obj]-result['object_position'])>.01
                                    or abs(float(np.dot(self.node.data.xquat[obj],result['object_quaternion']))) < .999):
                                self.cancel('Robot/object moved during reach planning; request a fresh task',failed=True)
                            else:
                                self.sequence_id = self.task_id+'.'+str(self.index)
                                try:
                                    self.record(reach_plan=result)
                                    self.arm.start_sequence(dict(source_sha256=self.arm.source_sha256,
                                        sequence_id=self.sequence_id,waypoints=result['waypoints']))
                                    if result.get('target_orientation_wxyz') is not None:
                                        self.reach_orientation=(self.node.mujoco.mj_name2id(self.node.model,
                                            self.node.mujoco.mjtObj.mjOBJ_BODY,step['left_body']),
                                            np.asarray(result['target_orientation_wxyz']))
                                    self.enter('articulating','Executing collision-inspected source-control reach ('+
                                               result['planner']['algorithm']+')'+
                                               ('; IK attempt '+str(result['planner']['selected_ik_attempt']+1)
                                                if len(result['planner'].get('ik_attempts',[]))>1 else ''))
                                except (ValueError,TypeError,KeyError) as exc:
                                    self.cancel(str(exc),failed=True)
                elif phase == 'gripping':
                    self.halt_base()
                    observation = self.measure_object(step)
                    self.object_measurement = observation
                    if step['operation']=='close':
                        ready = min(observation['left_force'],observation['right_force'])>=.1
                    else:
                        ready = (abs(self.arm.measured_controls()[self.grip_index]-self.arm.target[self.grip_index])<=.002
                                 and abs(self.arm.measured_rates()[self.grip_index])<=.005
                                 and max(observation['left_force'],observation['right_force'])<.1)
                    if self.mobile.moving():
                        self.cancel('Base moved during gripper operation',failed=True)
                    elif not ready:
                        self.grip_since = None
                    elif self.grip_since is None:
                        self.grip_since = self.node.data.time
                    elif self.node.data.time-self.grip_since>=.5:
                        if step['operation']=='close':
                            self.payload_body = self.gripped_body = self.object_bodies(step)[0]
                            self.gripped_step, self.grip_lost_since = dict(step), None
                            self.payload_confirmed = False
                        self.record(gripper_operation=step['operation'],object_observation=observation)
                        self.advance()
                elif phase == 'checking_object':
                    self.halt_base()
                    measurement = self.measure_object(step)
                    self.object_measurement = measurement
                    if self.object_gate.observe(**measurement, released=step['condition']=='released'):
                        self.payload_body = (self.object_bodies(step)[0] if step['condition']=='grasped' else None)
                        self.gripped_body = self.payload_body
                        self.gripped_step = dict(step) if self.payload_body is not None else None
                        self.payload_confirmed = step['condition']=='grasped'
                        self.object_outcome = step['condition']
                        self.record(object_observation=measurement, object_condition=self.object_outcome)
                        self.custody = (dict(step=step, relative=measurement['relative'], lost_since=None)
                                        if step['condition']=='grasped' else None)
                        self.advance()
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
                    orientation_guard=getattr(self,'reach_orientation',None)
                    orientation_error=(2*np.arccos(np.clip(abs(np.dot(self.node.data.xquat[orientation_guard[0]],
                        orientation_guard[1])),0.,1.)) if orientation_guard is not None else 0.)
                    if orientation_error>.08 and (step.get('keep_orientation')
                            or feedback.get('state')=='measured_waypoints_reached'):
                        self.cancel('Measured pad orientation left the held reach envelope',failed=True)
                    elif self.mobile.moving() or feedback.get('sequence_id') != self.sequence_id:
                        self.cancel('Articulation ownership/base movement changed', failed=True)
                    elif feedback['state'] == 'measured_waypoints_reached':
                        self.advance()
                    elif feedback['state'] != 'executing':
                        self.cancel('Articulation interrupted: '+feedback.get('reason', ''), failed=True)
        if (self.active() and self.gripped_step is not None and self.gripped_body is not None
                and self.steps[self.index]['kind'] in ('reach_object','stow_arm')):
            observation = self.measure_object(self.gripped_step)
            if min(observation['left_force'],observation['right_force'])>=.1:
                self.grip_lost_since = None
            elif self.grip_lost_since is None:
                self.grip_lost_since = observation['stamp']
            elif observation['stamp']-self.grip_lost_since>=.25:
                self.gripped_body, self.payload_confirmed = None, False
                self.cancel('Opposing object contacts lost during generated reach',failed=True)
        if (self.active() and self.custody is not None
                and self.steps[self.index]['kind'] == 'navigate'):
            observed = self.measure_object(self.custody['step'])
            self.object_measurement = observed
            lost = (min(observed['left_force'], observed['right_force']) < .1 or observed['supported']
                    or math.dist(observed['relative'], self.custody['relative']) > .01)
            if not lost:
                self.custody['lost_since'] = None
            elif self.custody['lost_since'] is None:
                self.custody['lost_since'] = observed['stamp']
            elif observed['stamp']-self.custody['lost_since'] >= .25:
                self.gripped_body, self.payload_confirmed = None, False
                self.object_outcome = 'custody_lost'
                self.record(object_observation=observed, object_condition=self.object_outcome)
                self.cancel('Measured object support/contact/slip changed during carry', failed=True)
        self.publisher.publish(String(data=json.dumps(dict(self.state,
            plant_id=self.arm.plant_id, source_sha256=self.arm.source_sha256, scene_id=self.scene_id,
            physical_pose=self.pose(), active=self.active(), nav_cancel_pending=bool(self.pending or self.handle),
            navigation_available=self.node.get_parameter('native_task').value == 'nav' and self.client.server_is_ready(),
            object_candidates=self.object_candidates, finger_candidates=self.finger_candidates,
            perception_objects=(list(self.object_estimates.targets) if time.monotonic()-self.object_estimates.received<1.
                and -.05<=float(self.node.data.time)-getattr(self.object_estimates,'stamp',-math.inf)<=.5 else []),
            perception_error=self.perception_error,
            perception_errors={source:store.reason for source,store in self.perception_estimates.items()},
            perception_sources={source:list(store.targets) for source,store in self.perception_estimates.items()
                if store.identity==self.estimate_identity() and time.monotonic()-store.received<1.
                and -.05<=float(self.node.data.time)-getattr(store,'stamp',-math.inf)<=.5},
            payload_name=(self.node.mujoco.mj_id2name(self.node.model, self.node.mujoco.mjtObj.mjOBJ_BODY, self.payload_body)
                          if self.payload_body is not None else ''),
            costmap_footprint_consumers=sorted(self.node.sensors.footprint_receipts.received),
            payload_body=self.payload_body, payload_confirmed=self.payload_confirmed, payload_footprint_ready=self.mobile.payload_footprint_ready(),
            journal=str(self.event_path) if self.event_path else '', object_outcome=self.object_outcome, object_measurement=self.object_measurement,
            qualification='implemented candidate; physical task validation pending'))))
