"""Generic fixed-base native position actuator control and measured feedback.

Supports authored direct-joint and coupled-tendon position servos. It never
invents a locomotion controller or changes source actuator gains/transmissions.
"""
import json
import hashlib
from pathlib import Path
import time
import uuid

import numpy as np
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger


from robot_lab_utils.native_actuation import position_channels


class NativeArticulationControl:
    def __init__(self, node):
        self.node, self.model, self.data = node, node.model, node.data
        self.channels = position_channels(self.model, node.robot_joints, allow_mobile_base=node.mobile is not None)
        excluded = set(map(int, self.model.jnt_bodyid[node.mobile.joints])) if node.mobile else set()
        self.contact_bodies = set(range(2, node.robot_bodies))-excluded
        if not self.channels:
            raise ValueError('No compatible bounded fixed-base position actuators in this native model')
        self.indices = np.array([channel['index'] for channel in self.channels])
        self.plant_id = uuid.uuid4().hex
        self.source_sha256 = hashlib.sha256(Path(node.get_parameter('native_mjcf').value).read_bytes()).hexdigest()
        self.sequence = None
        self.sequence_state = dict(state='idle', index=0, total=0)
        self.last_heartbeat = -float('inf')
        self.status = 'holding'
        self.fault = ''
        self.target = self.measured_controls()
        self.commanded = self.target.copy()
        self.home = self.target.copy()
        if self.model.nkey:
            self.home = np.clip(self.data.ctrl[self.indices].copy(),
                *np.asarray([channel['limits'] for channel in self.channels]).T)
        self.servo_channels = [index for index, channel in enumerate(self.channels)
                               if channel['joint'] and channel['units'] in ('rad', 'm')]
        self.servo = None
        if self.servo_channels:
            from robot_lab_mujoco.native_cartesian_servo import NativeCartesianServo
            names = [self.channels[index]['joint'] for index in self.servo_channels]
            if len(set(names)) == len(names):
                joints = [node.mujoco.mj_name2id(self.model, node.mujoco.mjtObj.mjOBJ_JOINT, name) for name in names]
                self.servo = NativeCartesianServo(node, joints,
                    [self.channels[index]['limits'] for index in self.servo_channels],
                    [self.channels[index]['max_rate'] for index in self.servo_channels])
                node.create_subscription(String, '/articulation/servo_command', self.servo_command, 1)
        self.pub = node.create_publisher(String, '/articulation/state', 10)
        node.create_subscription(String, '/articulation/command', self.receive, 1)
        node.create_subscription(Empty, '/articulation/heartbeat', self.heartbeat, 1)
        node.create_service(Trigger, '/articulation/stop', self.stop)

    def heartbeat(self, _message):
        self.last_heartbeat = time.monotonic()

    def servo_command(self, message):
        if self.servo is None or self.fault or time.monotonic()-self.last_heartbeat > .8:
            return
        if self.node.mobile and self.node.mobile.moving():
            self.servo.clear()
            return
        try:
            request = json.loads(message.data)
            if not isinstance(request, dict) or request.get('plant_id') != self.plant_id:
                return
            if self.sequence is not None:
                self.hold('Cartesian jog interrupted replay')
            self.servo.receive(message)
            if not self.servo.active():
                self.hold('Servo released; holding measured position')
        except (ValueError, TypeError):
            self.servo.clear()

    def measured_controls(self):
        gain = self.model.actuator_gainprm[self.indices, 0]
        bias = self.model.actuator_biasprm[self.indices]
        values = (-bias[:, 0] - bias[:, 1] * self.data.actuator_length[self.indices]) / gain
        return np.clip(values, *self.model.actuator_ctrlrange[self.indices].T)

    def receive(self, message):
        try:
            request = json.loads(message.data)
            if not isinstance(request, dict):
                raise ValueError('Native command must be a JSON object')
            if request.get('plant_id') != self.plant_id:
                raise ValueError('Command belongs to another simulator instance')
            if self.node.mobile and self.node.mobile.moving():
                raise ValueError('Stop the mobile base before articulation control')
            if self.fault or time.monotonic() - self.last_heartbeat > .8:
                raise ValueError('Controller fault or operator heartbeat missing')
            if request.get('operation') == 'sequence':
                self.start_sequence(request)
                return
            updates = request['targets']
            if not isinstance(updates, dict) or not updates:
                raise ValueError('Supply named actuator targets')
            known = {channel['name']: index for index, channel in enumerate(self.channels)}
            if set(updates) - known.keys():
                raise ValueError('Unknown native actuator')
            target = self.target.copy()
            for name, value in updates.items():
                value = float(value)
                index = known[name]
                lower, upper = self.channels[index]['limits']
                if not np.isfinite(value) or not lower <= value <= upper:
                    raise ValueError('Target exceeds source actuator bounds: '+name)
                target[index] = value
            self.target = target
            self.cancel_sequence('Manual joint target interrupted replay')
            if self.servo:
                self.servo.clear()
            self.status = 'jogging'
        except (ValueError, TypeError, KeyError, IndexError) as exc:
            self.hold('Rejected command: '+str(exc))

    def hold(self, reason='stopped; holding measured position'):
        self.cancel_sequence(reason)
        if self.servo:
            self.servo.clear()
        self.target = self.measured_controls()
        self.commanded = self.target.copy()
        self.status = reason

    def cancel_sequence(self, reason):
        if self.sequence is not None:
            self.sequence_state.update(state='interrupted', reason=reason)
            self.sequence = None

    def start_sequence(self, request):
        if self.sequence is not None or self.servo and self.servo.active():
            raise ValueError('Stop the current sequence/Servo before starting replay')
        if request.get('source_sha256') != self.source_sha256:
            raise ValueError('Replay source differs from the executed native model')
        waypoints = request.get('waypoints')
        if not isinstance(waypoints, list) or not 1 <= len(waypoints) <= 64:
            raise ValueError('Teach between 1 and 64 measured waypoints')
        names = [channel['name'] for channel in self.channels]
        limits = np.asarray([channel['limits'] for channel in self.channels])
        sequence = []
        for waypoint in waypoints:
            if not isinstance(waypoint, dict):
                raise ValueError('Waypoints must be JSON objects')
            targets = waypoint['targets']
            if not isinstance(targets, dict) or set(targets) != set(names):
                raise ValueError('Each waypoint must contain every source actuator channel')
            target = np.asarray([float(targets[name]) for name in names])
            dwell = float(waypoint.get('settle_s', .5))
            if (not np.all(np.isfinite(target)) or np.any(target < limits[:, 0])
                    or np.any(target > limits[:, 1]) or not np.isfinite(dwell) or not .2 <= dwell <= 5.):
                raise ValueError('Replay target/dwell exceeds source bounds')
            sequence.append((target, dwell))
        self.sequence, self.sequence_index = sequence, 0
        self.sequence_started_wall = time.monotonic()
        self.sequence_state = dict(state='executing', index=1, total=len(sequence),
            completion_scope='Measured actuator-coordinate settling; grasp/object mission unassessed')
        self.begin_waypoint()

    def begin_waypoint(self):
        self.target = self.sequence[self.sequence_index][0].copy()
        self.settled_since = None
        self.waypoint_started_wall = time.monotonic()
        self.status = 'taught sequence; waiting for measured settling'
        self.sequence_state['index'] = self.sequence_index+1

    def measured_rates(self):
        gain = self.model.actuator_gainprm[self.indices, 0]
        factor = -self.model.actuator_biasprm[self.indices, 1]/gain
        return self.data.actuator_velocity[self.indices]*factor

    def advance_sequence(self):
        if self.sequence is None:
            return
        now = time.monotonic()
        if now-self.sequence_started_wall > 600 or now-self.waypoint_started_wall > 120:
            self.hold('Replay settling deadline exceeded; holding measured articulation')
            self.sequence_state['state'] = 'failed'
            return
        span = np.asarray([channel['limits'][1]-channel['limits'][0] for channel in self.channels])
        tolerances = np.asarray([min(.01 if channel['units'] == 'rad' else .002, .005*span[index])
            if channel['units'] in ('rad', 'm') else .005*span[index]
            for index, channel in enumerate(self.channels)])
        rates = np.asarray([channel['max_rate'] for channel in self.channels])
        settled = (np.all(np.abs(self.measured_controls()-self.target) <= tolerances)
            and np.all(np.abs(self.measured_rates()) <= .05*rates)
            and np.all(np.abs(self.commanded-self.target) <= tolerances))
        if not settled:
            self.settled_since = None
            return
        if self.settled_since is None:
            self.settled_since = self.data.time
        if self.data.time-self.settled_since < self.sequence[self.sequence_index][1]:
            return
        self.sequence_index += 1
        if self.sequence_index < len(self.sequence):
            self.begin_waypoint()
        else:
            self.sequence = None
            self.sequence_state.update(state='measured_waypoints_reached', reason='Object mission remains unassessed')
            self.status = 'Replay measured actuator waypoints reached; inspect object outcome'

    def stop(self, _request, response):
        self.hold()
        response.success, response.message = True, self.status
        return response

    def reset(self):
        self.fault = ''
        self.last_heartbeat = -float('inf')
        self.hold('reset; holding measured position')

    def before_step(self):
        if self.fault:
            self.cancel_sequence(self.fault)
            self.data.ctrl[self.indices] = 0
            return
        if not np.all(np.isfinite(self.data.qpos)) or not np.all(np.isfinite(self.data.qvel)):
            self.fault = 'Non-finite native state; reset required'
            self.cancel_sequence(self.fault)
            self.data.ctrl[self.indices] = 0
            return
        if self.status == 'jogging' and any(contact.dist < -.002 and any(
                int(self.model.geom_bodyid[geom]) == 0 or int(self.model.geom_bodyid[geom]) >= self.node.robot_bodies
                for geom in contact.geom)
                and any(int(self.model.geom_bodyid[geom]) in self.contact_bodies for geom in contact.geom)
                for contact in self.data.contact[:self.data.ncon]):
            self.hold('environment penetration; holding measured position')
        if self.sequence is not None and any(contact.dist < -.002 and any(
                int(self.model.geom_bodyid[geom]) == 0 or int(self.model.geom_bodyid[geom]) >= self.node.robot_bodies
                for geom in contact.geom) and any(int(self.model.geom_bodyid[geom]) in self.contact_bodies
                for geom in contact.geom) for contact in self.data.contact[:self.data.ncon]):
            self.hold('Environment penetration interrupted replay')
        if time.monotonic() - self.last_heartbeat > .8:
            self.hold('heartbeat lost; holding measured position')
        elif self.servo:
            target = self.servo.target(.02)
            if target is not None:
                self.target[self.servo_channels] = target
                self.status = 'experimental Cartesian servo'
            elif self.status == 'experimental Cartesian servo':
                self.hold(self.servo.error or 'Servo command expired; holding measured position')
        rates = np.array([channel['max_rate'] for channel in self.channels])
        self.commanded += np.clip(self.target - self.commanded,
                                 -rates * self.model.opt.timestep, rates * self.model.opt.timestep)
        self.data.ctrl[self.indices] = self.commanded

    def after_step(self):
        self.advance_sequence()
        if np.max(np.abs(self.target - self.commanded)) < 1e-6:
            if self.status == 'jogging':
                self.status = 'holding target; inspect measured joint feedback'

    def publish(self):
        measured = self.measured_controls()
        velocities = self.measured_rates()
        channels = [dict(channel, target=float(self.target[index]),
                         command=float(self.commanded[index]), home=float(self.home[index]),
                         measured=float(measured[index]),
                         measured_rate=float(velocities[index]),
                         actuator_force=float(self.data.actuator_force[channel['index']]))
                    for index, channel in enumerate(self.channels)]
        self.pub.publish(String(data=json.dumps(dict(plant_id=self.plant_id, channels=channels,
            source_sha256=self.source_sha256, sequence=self.sequence_state,
            native_model=self.node.get_parameter('native_mjcf').value,
            status=self.status, fault=self.fault, time=float(self.data.time),
            servo=self.servo.state() if self.servo else None,
            mobile_state=dict(moving=self.node.mobile.moving(), arm_stowed=bool(self.node.mobile.stowed()),
                status=self.node.mobile.status) if self.node.mobile else None,
            qualification='implemented experimental position controls; validation pending'))))
