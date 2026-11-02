#!/usr/bin/env python3
"""Physical facility feedback, RMF door/lift protocol and measured docking.

Battery energy is an authored estimate from actual reported motor power.
Missing motor feedback makes it unknown; no nominal library SOC is published.
"""
import argparse
import json
import math
from pathlib import Path
import time
import uuid
import xml.etree.ElementTree as ET

from robot_lab_utils.fleet_energy import EnergyEstimate, qualified_source
from robot_lab_utils.fleet_transport import LiftTransport
from robot_lab_utils.fleet_transport_queue import TransportQueue
from robot_lab_utils.fleet_lift_recovery import recovery_passengers
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState, LaserScan
from nav_msgs.msg import Odometry,Path as NavPath
from std_msgs.msg import String,Float64
from geometry_msgs.msg import TransformStamped
from rosgraph_msgs.msg import Clock as ClockMessage
import yaml
import tf2_ros
from nav2_msgs.srv import LoadMap, ClearEntireCostmap


class Infrastructure(Node):
    def __init__(self,session):
        super().__init__('fleet_infrastructure');self.session=session;self.identity=Path(session['directory']).name
        self.config=yaml.safe_load(Path(session['infrastructure_file']).read_text());self.token=uuid.uuid4().hex
        self.positions={};self.joint_at=-math.inf;self.clock=None;self.clock_at=-math.inf
        self.joint_stamp=None;self.rates={}
        self.runtime=None;self.runtime_at=-math.inf;self.bodies={};self.paths={};self.docking={};self.events=[]
        self.doors={name:False for name in self.config['doors']};self.outputs={};self.floor='L0';self.lift_request=None;self.lift_owner='';self.lift_at=-math.inf
        self.hold=False;self.lift_gui=False;self.door_owners={name:set() for name in self.doors}
        self.energy = {m['id']: dict(floor='L0') for m in session['members']}
        self.energy_models = {}
        sources = self.config['energy'].get('effort_sources', {})
        for member in session['members']:
            description = ET.parse(member['description'])
            joints = [joint.get('name') for joint in description.findall('.//ros2_control/joint')
                      if joint.find("command_interface[@name='velocity']") is not None]
            certificate = sources.get(member['model'], {})
            verified = qualified_source(member, certificate)
            self.energy_models[member['id']] = EnergyEstimate(self.config['energy'], joints, verified)
        self.transports = {}
        self.transport_queue = TransportQueue()
        self.recovery = None
        self.last_rejection = ''
        self.floor_offsets={}
        self.floor_maps=session.get('floor_maps',self.config.get('floor_maps',{}));self.current_floor_map=''
        self.tf_broadcaster=tf2_ros.TransformBroadcaster(self)
        self.floor_z={name:float(self.config['lift']['floors']['L0']) for m in session['members'] for name in [m['id']]}
        joints=[v['joint'] for v in self.config['doors'].values()]+[self.config['lift']['joint']]
        self.targets={name:self.create_publisher(Float64,'/facility/'+name+'/target',10) for name in joints}
        self.publisher=self.create_publisher(String,'/fleet/infrastructure/state',10);self.fleet_command=self.create_publisher(String,'/fleet/command',10)
        self.create_subscription(JointState,'/facility/joint_states',self.joints,10)
        self.create_subscription(ClockMessage,'/clock',self.observe_clock,qos_profile_sensor_data)
        self.create_subscription(String,'/fleet/state',self.fleet_state,10)
        self.create_subscription(String,'/fleet/infrastructure/command',self.command,10)
        self.create_subscription(String,'/fleet/command',self.stop_command,10)
        for m in session['members']:
            name=m['id'];self.create_subscription(Odometry,'/'+name+'/odom/ground_truth',lambda msg,n=name:self.body(n,msg),qos_profile_sensor_data)
            self.create_subscription(JointState,'/'+name+'/joint_states',lambda msg,n=name:self.motor(n,msg),10)
            self.create_subscription(NavPath,'/'+name+'/plan',lambda msg,n=name:self.path(n,msg),10)
        self.localization_scans = {}
        self.localization_scan_after = {}
        for member in session['members']:
            if member.get('localization_scan'):
                name = member['id']
                self.localization_scans[name] = self.create_publisher(LaserScan, member['localization_scan'], 10)
                self.create_subscription(LaserScan, '/' + name + '/scan', lambda msg,n=name:self._relay_scan(n,msg), qos_profile_sensor_data)
        self.robot_floor_maps={m['id']:m.get('floor_maps',self.floor_maps) for m in session['members']}
        self.active_robot_maps={m['id']:m.get('map_yaml','') for m in session['members']}
        self.map_clients={m['id']:self.create_client(LoadMap,'/'+m['id']+'/map_server/load_map')
                          for m in session['members'] if session.get('navigation') and m.get('floor_maps')}
        self.map_clear_clients = {name: {kind: self.create_client(ClearEntireCostmap, '/' + name + '/' + kind + '_costmap/clear_entirely_' + kind + '_costmap') for kind in ('global', 'local')} for name in self.map_clients}
        self.map_clear_wait = {}
        self.map_clear_futures = {}
        self.map_futures={};self.map_transitions=set();self.map_errors={};self.map_status={};self.map_attempt_at={}
        self.rmf_control = {}
        self.rmf_control_at = -math.inf
        self.rmf_stopped_at = -math.inf
        self.rmf_active = False
        self.create_subscription(String, '/fleet/rmf/state', self.observe_rmf_control, 10)
        self.rmf=False
        try:
            from rmf_door_msgs.msg import DoorRequest,DoorState
            from rmf_lift_msgs.msg import LiftRequest,LiftState
            self.door_state=self.create_publisher(DoorState,'/door_states',10);self.lift_state=self.create_publisher(LiftState,'/lift_states',10)
            self.create_subscription(DoorRequest,'/door_requests',self.rmf_door_request,10)
            self.create_subscription(LiftRequest,'/lift_requests',self.rmf_lift_request,10);self.rmf=True
        except ImportError:pass
        self.create_timer(.1,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def observe_clock(self,msg):
        value=msg.clock.sec+msg.clock.nanosec*1e-9
        if self.clock is not None and value<self.clock:
            self.rmf_stopped_at = time.monotonic()
            self.rmf_active = False
            self.doors={n:False for n in self.doors};self.outputs={n:self.positions[n] for n in self.targets if n in self.positions};self.lift_request=None;self.lift_owner='';self.bodies.clear();self.paths.clear();self.docking.clear()
            self._cancel_transports('Simulation clock reset')
            self._stop_recovery('Simulation clock reset')
            self.hold=True;self.outputs={};self.positions={};self.joint_stamp=None;self.rates={};self.joint_at=-math.inf
            self.door_owners={name:set() for name in self.doors}
            for item in self.energy_models.values():item.invalidate('Simulation clock reset; energy coverage lost')
        self.clock=value;self.clock_at=time.monotonic()

    def joints(self,msg):
        values={n.split('::')[-1]:v for n,v in zip(msg.name,msg.position)}
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if all(n in values and math.isfinite(values[n]) for n in self.targets):
            if self.joint_stamp is not None and 0<stamp-self.joint_stamp<=.5:
                self.rates={n:(values[n]-self.positions[n])/(stamp-self.joint_stamp) for n in self.targets}
            self.positions=values;self.joint_at=time.monotonic();self.joint_stamp=stamp

    def fleet_state(self,msg):
        try:
            row=json.loads(msg.data)
            if not isinstance(row,dict):return
            if row['session_id']==self.identity:self.runtime=row;self.runtime_at=time.monotonic()
        except (ValueError,KeyError,TypeError):pass

    def body(self,name,msg):
        p,q=msg.pose.pose.position,msg.pose.pose.orientation;stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if not all(map(math.isfinite,[p.x,p.y,p.z,q.x,q.y,q.z,q.w])) or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>.01:return
        previous=self.bodies.get(name);speed=angular=None;heading=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        if previous and 0<stamp-previous['stamp']<=.5:
            speed=math.dist([p.x,p.y,p.z],previous['xyz'])/(stamp-previous['stamp'])
            angular=math.atan2(math.sin(heading-previous['yaw']),math.cos(heading-previous['yaw']))/(stamp-previous['stamp'])
        self.bodies[name]=dict(xyz=[p.x,p.y,p.z],yaw=heading,speed=speed,angular=angular,stamp=stamp,at=time.monotonic())
        self._observe_robot_floor(name, [p.x, p.y, p.z])

    def _observe_robot_floor(self,name,xyz):
        floors=self.config['lift']['floors'];tolerance=float(self.config.get('robot_floor_tolerance_m',.75))
        if name not in self.floor_offsets:
            initial=min(floors,key=lambda floor:abs(xyz[2]-floors[floor]))
            if abs(xyz[2]-floors[initial])>tolerance:return
            self.floor_offsets[name]=xyz[2]-floors[initial]
            self.energy[name]['floor']=initial
        observed_height=xyz[2]-self.floor_offsets[name]
        floor=min(floors,key=lambda item:abs(observed_height-floors[item]))
        if abs(observed_height-floors[floor])>tolerance:return
        if 2.8<=xyz[0]<=5.2 and -1.2<=xyz[1]<=1.2:
            lift=self.config['lift'];position=self.positions.get(lift['joint'])
            rate=self.rates.get(lift['joint'],math.inf)
            if position is None or abs(rate)>=.01 or abs(position-floors[floor])>=self.config['lift'].get('level_tolerance_m',.03):return
        self.energy[name]['floor']=floor

    def localization_scan_enabled(self, name):
        now = time.monotonic()
        body = self.bodies.get(name)
        if not body or now - body['at'] >= .6 or now - self.clock_at >= .6:
            return False
        floor = self.energy[name]['floor']
        if name in self.map_transitions or self.active_robot_maps.get(name) != self.robot_floor_maps[name].get(floor):
            return False
        # A planar map observation model is invalid while the physical sensor
        # traverses floors. Keep the existing AMCL/wheel XY prior; never replace
        # it with body truth. Raw obstacle/Drive scans remain on the original topic.
        x, y, _ = body['xyz']
        if 2.8 <= x <= 5.2 and -1.2 <= y <= 1.2:
            lift = self.config['lift']
            position = self.positions.get(lift['joint'])
            if (position is None or abs(self.rates.get(lift['joint'], math.inf)) >= .01
                    or abs(position - lift['floors'][floor]) >= lift.get('level_tolerance_m', .03)
                    or self.lift_request and self.lift_request[0] != floor):
                return False
        return True

    def _relay_scan(self, name, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if (msg.header.frame_id == name + '/laser_link' and self.clock is not None
                and -1.5 <= stamp - self.clock <= .2
                and stamp >= self.localization_scan_after.get(name, -math.inf) and self.localization_scan_enabled(name)):
            self.localization_scans[name].publish(msg)

    def _request_robot_map(self,name,floor,now):
        target=self.robot_floor_maps.get(name,{}).get(floor,'')
        if not target or name not in self.map_clients:return
        if name in self.map_errors:return
        if name in self.map_clear_wait:
            self._clear_costmaps(name)
            return
        if self.active_robot_maps.get(name)==target:
            if name in self.map_transitions:
                self.localization_scan_after[name] = self.clock if self.clock is not None else 0.
            self.map_transitions.discard(name);self.map_errors.pop(name,None);self.map_status[name]='ready'
            return
        self.map_transitions.add(name)
        if name in self.map_errors or name in self.map_futures or now-self.map_attempt_at.get(name,-math.inf)<1.:
            return
        self.map_attempt_at[name]=now
        client=self.map_clients[name]
        if not client.service_is_ready():
            self.map_status[name]='waiting for map server'
            return
        request=LoadMap.Request();request.map_url=target
        self.map_status[name]='loading '+floor
        future=client.call_async(request);self.map_futures[name]=future
        future.add_done_callback(lambda completed,n=name,f=floor,p=target:self._map_loaded(n,f,p,completed))

    def _map_loaded(self,name,floor,path,future):
        self.map_futures.pop(name,None)
        try:
            response=future.result()
            if response.result!=LoadMap.Response.RESULT_SUCCESS:
                self.map_errors[name]='map server rejected '+floor+' map (result '+str(response.result)+')'
                self.map_status[name]='failed'
                return
        except Exception as error:
            self.map_errors[name]='map load failed: '+str(error)
            self.map_status[name]='failed'
            return
        self.active_robot_maps[name]=path
        self.map_status[name]='clearing old-floor costmaps'
        self.map_errors.pop(name,None)
        self.map_clear_wait[name] = {'global', 'local'}

    def _clear_costmaps(self, name):
        for kind in tuple(self.map_clear_wait[name]):
            key = (name, kind)
            client = self.map_clear_clients[name][kind]
            if key in self.map_clear_futures or not client.service_is_ready():
                continue
            future = client.call_async(ClearEntireCostmap.Request())
            self.map_clear_futures[key] = future
            future.add_done_callback(lambda done,n=name,k=kind:self._costmap_cleared(n,k,done))

    def _costmap_cleared(self, name, kind, future):
        self.map_clear_futures.pop((name, kind), None)
        try:
            future.result()
        except Exception as error:
            self.map_errors[name] = 'Floor costmap reset failed: ' + str(error)
            self.map_status[name] = 'failed'
            return
        self.map_clear_wait[name].discard(kind)
        if not self.map_clear_wait[name]:
            self.map_clear_wait.pop(name)
            # The next tick releases only when the loaded map still matches
            # this robot's observed floor. An older callback cannot release it.
            self.map_status[name] = 'costmaps reset; checking observed floor'

    def path(self,name,msg):self.paths[name]=([(p.pose.position.x,p.pose.position.y) for p in msg.poses],time.monotonic())

    def motor(self,name,msg):
        now = time.monotonic()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        docked = self._docked_now(name, now)
        clock = self.clock if now - self.clock_at < .6 else None
        self.energy_models[name].observe(msg.name, msg.velocity, msg.effort,
                                         stamp, clock, now, docked)

    def _docked_now(self, name, now):
        item = self.docking.get(name)
        if not item or not item.get('docked') or now - self.joint_at >= .6:
            return False
        body = self.bodies.get(name)
        dock = self.config['docks'].get(item['dock'])
        return bool(body and dock and now - body['at'] < .6
                    and body['speed'] is not None and body['speed'] < .03
                    and body['angular'] is not None and abs(body['angular']) < .05
                    and math.hypot(body['xyz'][0] - dock['x'], body['xyz'][1] - dock['y']) < dock['position_tolerance']
                    and abs(body['xyz'][2] - dock.get('z', 0.)) < dock.get('height_tolerance', .12)
                    and abs(math.atan2(math.sin(body['yaw'] - dock['yaw']), math.cos(body['yaw'] - dock['yaw']))) < dock['yaw_tolerance'])

    def _cancel_transports(self, reason, targets=None):
        if hasattr(self, 'transport_queue'):
            self.transport_queue.cancel(reason, targets)
        for name, task in self.transports.items():
            if targets is None or name in targets:
                self._transport_actions(task, task.cancel(reason))

    def _transport_actions(self, task, actions):
        from types import SimpleNamespace
        for action in actions:
            if action['kind'] == 'route':
                self.fleet_command.publish(String(data=json.dumps(dict(session_id=self.identity,
                    operator_id=task.operator, op='route', targets=[task.robot],
                    request_id=action['request_id'], goals=[action['goal']]))))
            elif action['kind'] == 'lift':
                self.request_lift(SimpleNamespace(lift_name=self.config['lift']['name'],
                    session_id=task.prefix, request_type=1, destination_floor=action['floor'], door_state=2))
            elif action['kind'] == 'stop':
                self.fleet_command.publish(String(data=json.dumps(dict(session_id=self.identity,
                    operator_id=task.operator, op='stop', targets=[task.robot]))))
            elif action['kind'] == 'release' and self.lift_owner == task.prefix:
                self.lift_request = None
                self.lift_owner = ''
                self.lift_gui = False
                if action['hold']:
                    self.hold = True
                    self.outputs = {joint: self.positions[joint] for joint in self.targets if joint in self.positions}

    def _transport_tick(self, now, fresh, current_floor, door_rows):
        for name, task in self.transports.items():
            if not task.active:
                continue
            body = self.bodies.get(name)
            member = next((row for row in (self.runtime or {}).get('members', []) if row['id'] == name), {})
            operator = (self.runtime and now - self.runtime_at < 1. and
                        self.runtime.get('operator_fresh') and self.runtime.get('operator_id') == task.operator)
            valid = fresh and body and now - body['at'] < .6 and bool(member)
            if task.active and operator and valid and self.lift_owner == task.prefix:
                self.lift_at = now
            context = dict(fresh=bool(valid), operator_fresh=bool(operator), body=body, member=member,
                floor=self.energy[name]['floor'], clock=self.clock,
                history=(self.runtime or {}).get('task_history', []), car_floor=current_floor,
                door_open=all(door_rows.get(name, {}).get('mode')==2 for name,door in self.config['doors'].items() if name==self.config['lift']['door'] or door.get('landing_floor')==current_floor),
                map_ready=name not in self.map_transitions and self.map_status.get(name) == 'ready')
            # Publish the new reservation before its first route command.
            if now - task.started >= .35:
                self._transport_actions(task, task.tick(context, now))
        queue = self.transport_queue
        if queue.active:
            task = self.transports.get(queue.active['robot'])
            if task and not task.active:
                queue.finish(task.phase, task.reason)
                if task.phase != 'succeeded':
                    queue.cancel('Previous transport interrupted; explicit new itinerary required')
        if queue.active or not queue.pending:
            return
        head = queue.pending[0]
        operator = (self.runtime and now - self.runtime_at < 1. and self.runtime.get('operator_fresh')
                    and self.runtime.get('operator_id') == head['operator_id'])
        if not operator or not fresh:
            queue.cancel('Facility/operator feedback lost; explicit new itinerary required')
            return
        if now - head['enqueued_at'] > 600.:
            queue.cancel('Queue waiting deadline exceeded; explicit new itinerary required')
            return
        if self.hold or self.recovery and self.recovery['phase'] == 'recovering':
            head['reason'] = 'Lift held; explicitly recover/cancel before dispatch'
            return
        if any(task.active for task in self.transports.values()) or self.lift_owner and self.lift_owner != head['operator_id']:
            head['reason'] = 'Waiting for current lift session to release'
            return
        try:
            task = self._prepare_transport(head['robot'], head['floor'], head['operator_id'], now)
        except ValueError as error:
            head['reason'] = str(error)
            return
        queue.take()
        self.transports[task.robot] = task
        self.lift_owner, self.lift_at = task.prefix, now
        self.lift_request = None
        self.lift_gui = False

    def _prepare_transport(self, name, destination, operator, now):
        row = next((item for item in self.runtime['members'] if item['id'] == name), {})
        if not row.get('navigation_ready') or not row.get('included') or row.get('mode') != 'idle' or row.get('queued'):
            raise ValueError('Waiting for fresh idle member Nav2')
        task = LiftTransport(name, operator, self.energy[name]['floor'], destination, self.config['transport'], now)
        if set(self.bodies) != set(self.energy) or any(now - body['at'] >= .6 for body in self.bodies.values()):
            raise ValueError('Fresh independent observations of all fleet bodies required')
        body = self.bodies[name]
        lift = self.config['lift']
        if task.in_cabin(body['xyz']):
            if abs(self.positions.get(lift['joint'], -1000.) - lift['floors'][task.origin]) >= lift.get('level_tolerance_m', .03) or abs(self.rates.get(lift['joint'], math.inf)) >= .01:
                raise ValueError('Explicitly recover the occupied lift to a settled floor first')
        if any(peer != name and task.in_cabin(body['xyz']) for peer, body in self.bodies.items()):
            raise ValueError('Waiting for another physical robot to leave the car')
        radii = {member['id']: float(member.get('radius', .35)) for member in self.session['members']}
        for peer, body in self.bodies.items():
            if peer == name:
                continue
            for floor in (task.origin, task.destination):
                landing = task.config['landings'][floor]
                if (abs(body['xyz'][2] - lift['floors'][floor]) < .2 and
                        math.hypot(body['xyz'][0] - landing['x'], body['xyz'][1] - landing['y']) < radii[name] + radii[peer] + .15):
                    raise ValueError('Waiting for clear ' + floor + ' landing; move the parked peer away')
        return task

    def _stop_recovery(self, reason):
        if not self.recovery or self.recovery['phase'] != 'recovering':
            return
        self.recovery.update(phase='failed', reason=reason)
        self.hold = True
        self.outputs = {joint: self.positions[joint] for joint in self.targets if joint in self.positions}
        self.lift_request = None
        self.lift_owner = ''
        self.lift_gui = False

    def _recovery_tick(self, now, fresh, door_rows):
        task = self.recovery
        if not task or task['phase'] != 'recovering':
            return
        operator = (self.runtime and now - self.runtime_at < 1. and self.runtime.get('operator_fresh')
                    and self.runtime.get('operator_id') == task['operator_id'])
        if not fresh or not operator or now - task['started'] > 100.:
            self._stop_recovery('Recovery feedback/operator lost or deadline exceeded; explicit request required')
            return
        # During travel vertical body speeds are expected. Check identity and
        # containment, rather than applying the stationary admission test.
        height = self.positions[self.config['lift']['joint']]
        x0, x1, y0, y1 = self.config['transport']['cabin']
        radii = {member['id']: float(member.get('radius', .35)) for member in self.session['members']}
        for name in self.energy:
            body = self.bodies.get(name)
            if not body or now - body['at'] >= .6:
                self._stop_recovery('Recovery body observation lost')
                return
            x, y, z = body['xyz']
            radius = radii[name]
            if name in task['passengers']:
                if not (x0 <= x - radius and x + radius <= x1 and y0 <= y - radius and y + radius <= y1 and abs(z - height) < .2):
                    self._stop_recovery('Passenger left the observed safe cabin envelope')
                    return
            elif x0 - radius < x < x1 + radius and y0 - radius < y < y1 + radius and abs(z - height) < .2:
                self._stop_recovery('Unexpected member overlaps the recovery cabin')
                return
        self.lift_at = now
        lift = self.config['lift']
        level = abs(height - lift['floors'][task['floor']]) < lift.get('level_tolerance_m', .03)
        opened = all(door_rows.get(name, {}).get('mode') == 2 for name, door in self.config['doors'].items()
                     if name == lift['door'] or door.get('landing_floor') == task['floor'])
        if level and abs(self.rates.get(lift['joint'], math.inf)) < .01 and opened:
            task.update(phase='succeeded', reason='Observed level car and open landing; use manual Drive to exit or request a new floor transport')
            self.lift_owner = ''

    def _admit_transports(self, requests, operator, now):
        if 'transport' not in self.config or not self.session.get('navigation'):
            raise ValueError('Automatic transport requires a declared facility and member Nav2')
        if self.rmf_active or self.recovery and self.recovery['phase'] == 'recovering':
            raise ValueError('Release SDK/recovery ownership before a native itinerary')
        if self.hold:
            raise ValueError('Lift is held; explicitly recover it before requesting transport')
        members = {row['id']: row for row in self.runtime.get('members', [])}
        queued_names = {row['robot'] for row in self.transport_queue.pending}
        queued_names.update(name for name, task in self.transports.items() if task.active)
        if not isinstance(requests, list):
            raise ValueError('Transport itinerary must be a list')
        for request in requests:
            name = request.get('robot') if isinstance(request, dict) else None
            row = members.get(name, {}) if isinstance(name, str) else {}
            # A queue can wait for lifecycle readiness. Check actual active
            # Nav2 again at dispatch, rather than racing two status subscribers
            # during admission. No route can start through this admission gate.
            if name not in queued_names and (not row.get('included') or row.get('mode') != 'idle' or row.get('queued')):
                raise ValueError('Select included idle members, or append to an owned itinerary')
        self.transport_queue.append(requests, operator, {name: row['floor'] for name, row in self.energy.items()}, self.config['lift']['floors'], now)

    def safe_sweep(self,door):
        now=time.monotonic();x0,x1,y0,y1=door['sweep']
        if set(self.bodies)!=set(self.energy) or any(now-b['at']>.6 for b in self.bodies.values()):return False
        base=self.positions.get(door.get('z_joint'),door.get('base_z',0.))
        return not any(x0<=b['xyz'][0]<=x1 and y0<=b['xyz'][1]<=y1 and base-.2<=b['xyz'][2]<base+2.5 for b in self.bodies.values())

    def observe_rmf_control(self, msg):
        try:
            data=json.loads(msg.data)
            if isinstance(data,dict) and data.get('session_id')==self.identity:
                self.rmf_control=data
                self.rmf_control_at=time.monotonic()
        except (ValueError,KeyError,TypeError):pass

    def rmf_authorized(self):
        now=time.monotonic()
        state=getattr(self,'rmf_control',{})
        enabled_at=state.get('enabled_at')
        epoch=state.get('authorization_epoch')
        return bool(self.runtime and state.get('enabled') is True
            and isinstance(enabled_at,(int,float)) and math.isfinite(enabled_at)
            and getattr(self,'rmf_stopped_at',-math.inf)<enabled_at<=now
            and isinstance(epoch,str) and 1<=len(epoch)<=80
            and now-getattr(self,'rmf_control_at',-math.inf)<1.
            and now-self.runtime_at<1. and self.runtime.get('operator_fresh') is True
            and state.get('operator_id')==self.runtime.get('operator_id')
            and now-self.clock_at<.6 and now-self.joint_at<.6)

    def _begin_rmf_control(self):
        if (not self.rmf_authorized() or any(task.active for task in self.transports.values())
                or self.transport_queue.pending or self.recovery and self.recovery['phase'] == 'recovering'):
            return False
        if not self.rmf_active:
            self.doors={name:False for name in self.doors}
            self.door_owners={name:set() for name in self.doors}
            self.lift_request=None;self.lift_owner='';self.lift_gui=False
            self.docking.clear()
        self.rmf_active=True
        return True

    def rmf_door_request(self,msg):
        if self._begin_rmf_control():
            self.door_request(msg)

    def rmf_lift_request(self,msg):
        if self._begin_rmf_control():
            self.request_lift(msg)

    def door_request(self,msg):
        if msg.door_name in self.doors and msg.requested_mode.value in (0,2):
            if msg.requested_mode.value==2:self.door_owners[msg.door_name].add(msg.requester_id)
            else:self.door_owners[msg.door_name].discard(msg.requester_id)
            self.hold=False

    def request_lift(self,msg):
        if msg.lift_name!=self.config['lift']['name']:return
        now=time.monotonic()
        if any(task.active and msg.session_id != task.prefix for task in self.transports.values()):return
        # RMF may stop repeating requests once an event succeeds. A live
        # session retains ownership until SessionEnd, Stop or lease loss.
        if self.lift_owner and msg.session_id!=self.lift_owner:return False
        if msg.request_type==0:
            if msg.session_id==self.lift_owner:self.lift_owner='';self.lift_request=None
            return True
        if msg.request_type!=1 or not msg.session_id or len(msg.session_id)>128 or msg.destination_floor not in self.config['lift']['floors'] or msg.door_state not in (0,2):return
        self.lift_owner=msg.session_id;self.lift_at=now;self.lift_request=(msg.destination_floor,msg.door_state==2)
        self.lift_gui=False;self.hold=False
        return True

    def lift_target(self, position, door_rows, now):
        lift = self.config['lift']
        # A completed session still leaves a force-driven car at its last
        # commanded height. Following measured position here ratchets contact
        # disturbances into a new setpoint and lets the car drift during exit.
        target = self.outputs.get(lift['joint'], position)
        owned = now - self.lift_at < 5 or self.rmf_active and self.rmf_authorized()
        if self.lift_request and owned:
            floor, _ = self.lift_request
            closed = all(row['mode'] == 0 for name, row in door_rows.items()
                         if name == lift['door'] or self.config['doors'][name].get('landing_floor'))
            if closed or abs(position - lift['floors'][floor]) < lift.get('level_tolerance_m', .03):
                target = lift['floors'][floor]
        return target

    def stop_command(self,msg):
        try:
            if len(msg.data)>4096:return
            data=json.loads(msg.data)
            if not isinstance(data,dict):return
            if data.get('session_id')==self.identity and data.get('op')=='all_stop':
                self.rmf_stopped_at=time.monotonic();self.rmf_active=False
                self.doors={name:False for name in self.doors}
                self.door_owners={name:set() for name in self.doors}
                self.lift_owner='';self.lift_gui=False
                self._cancel_transports('Operator all-stop')
                self._stop_recovery('Operator all-stop')
                self.hold=True;self.outputs={n:self.positions[n] for n in self.targets if n in self.positions};self.lift_request=None;self.docking.clear()
            elif data.get('session_id') == self.identity and data.get('op') in ('stop', 'pause', 'reset', 'leave'):
                if self.runtime and data.get('operator_id') == self.runtime.get('operator_id') and self.runtime.get('operator_fresh'):
                    self._cancel_transports('Operator interrupted lift transport', data.get('targets', []))
                    if self.recovery and set(data.get('targets', [])) & set(self.recovery['passengers']):
                        self._stop_recovery('Operator interrupted lift recovery')
        except (ValueError,KeyError,TypeError):pass

    def command(self,msg):
        try:
            if len(msg.data)>4096:return
            data=json.loads(msg.data)
            if not isinstance(data,dict):return
            if not self.runtime or time.monotonic()-self.runtime_at>1 or data.get('session_id')!=self.identity:return
            if data.get('operator_id')!=self.runtime['operator_id'] or not self.runtime.get('operator_fresh'):return
            op=data['op']
            self.last_rejection = ''
            if op=='door' and data['door'] in self.doors:
                if type(data['open']) is not bool:return
                if self.hold:
                    raise ValueError('Explicitly recover the held lift before manual facility commands')
                self.doors[data['door']]=data['open'] is True;self.hold=False
            elif op in ('transport', 'transport_queue'):
                requests = [dict(robot=data['robot'], floor=data['floor'])] if op == 'transport' else data['requests']
                self._admit_transports(requests, data['operator_id'], time.monotonic())
            elif op == 'cancel_transports':
                self._cancel_transports('Operator canceled facility itinerary')
            elif op == 'recover_lift':
                if not self.hold or self.rmf_active or self.lift_owner or any(task.active for task in self.transports.values()):
                    raise ValueError('Recovery requires a held lift with no active requester')
                if data['floor'] not in self.config['lift']['floors']:
                    raise ValueError('Select a configured recovery floor')
                now = time.monotonic()
                if now - self.joint_at >= .6 or now - self.clock_at >= .6:
                    raise ValueError('Recovery requires fresh joint and clock observations')
                passengers = recovery_passengers(self.config, self.session['members'], self.bodies, self.positions, self.rates, now)
                rows = {row['id']: row for row in self.runtime['members']}
                if any(rows.get(name, {}).get('mode') != 'idle' or rows.get(name, {}).get('queued') for name in passengers):
                    raise ValueError('Stop each passenger’s navigation/manual control before recovery')
                self._cancel_transports('Explicit recovery replaces pending itinerary')
                self.recovery = dict(phase='recovering', floor=data['floor'], operator_id=data['operator_id'],
                    passengers=passengers, started=now, reason='Closing interlocked gates, then recovering to the selected floor')
                self.lift_owner = 'recovery-' + uuid.uuid4().hex
                self.lift_at = now
                self.lift_request = (data['floor'], True)
                self.lift_gui = False
                self.hold = False
            elif op=='lift':
                if (self.transport_queue.pending or any(task.active for task in self.transports.values())
                        or self.recovery and self.recovery['phase'] == 'recovering'):
                    raise ValueError('Cancel active transport/recovery before manual lift movement')
                if self.hold:
                    raise ValueError('Use explicit Recover lift for a held car')
                lift = self.config['lift']
                if not any(abs(self.positions.get(lift['joint'], -1000.) - z) < lift.get('level_tolerance_m', .03) for z in lift['floors'].values()):
                    raise ValueError('Use explicit Recover lift for a car between floors')
                from types import SimpleNamespace
                if not self.request_lift(SimpleNamespace(lift_name=self.config['lift']['name'],session_id=data['operator_id'],request_type=1,destination_floor=data['floor'],door_state=2)):
                    raise ValueError('Another requester retains lift-session ownership')
                self.lift_gui=True
            elif op=='dock':
                name,dock=data['robot'],data['dock'];target=self.config['docks'][dock]
                if name not in self.energy:return
                self.docking[name]=dict(dock=dock,request_id=data.get('request_id',uuid.uuid4().hex),settled=None,docked=False)
                command=dict(session_id=self.identity,operator_id=data['operator_id'],op='dispatch',targets=[name],x=target['x'],y=target['y'],yaw=target['yaw'],timeout=180.)
                self.fleet_command.publish(String(data=json.dumps(command)))
        except (ValueError,KeyError,TypeError) as error:
            self.last_rejection = str(error)
            self.get_logger().warning('Facility command rejected: '+str(error))

    def tick(self):
        now=time.monotonic();fresh=now-self.joint_at<.6 and now-self.clock_at<.6
        if self.lift_gui and (not self.runtime or now-self.runtime_at>=1. or not self.runtime.get('operator_fresh') or self.runtime.get('operator_id')!=self.lift_owner):
            self.lift_gui=False;self.lift_owner='';self.lift_request=None;self.hold=True
            self.outputs={joint:self.positions[joint] for joint in self.targets if joint in self.positions}
        if self.rmf_active and not self.rmf_authorized():
            self.rmf_stopped_at=now
            self.rmf_active=False
            self.door_owners={name:set() for name in self.doors}
            self.lift_request=None;self.lift_owner='';self.lift_gui=False
            self.hold=True
            self.outputs={joint:self.positions[joint] for joint in self.targets if joint in self.positions}
        if self.lift_gui and self.runtime and self.runtime.get('operator_id')==self.lift_owner and self.runtime.get('operator_fresh') and now-self.runtime_at<1:self.lift_at=now
        lift=self.config['lift'];lift_position=self.positions.get(lift['joint']);door_rows={}
        if fresh:
            for name,door in self.config['doors'].items():
                needed=self.doors[name] or bool(self.door_owners[name])
                # Actual Nav2 paths request the physical corridor door open.
                if name=='main':
                    needed |= any(now-at<1. and any(abs(x)<1.5 and abs(y)<1. for x,y in points) for points,at in self.paths.values())
                if name==lift['door'] and self.lift_request:
                    floor,opened=self.lift_request;needed=opened and abs(lift_position-lift['floors'][floor])<lift.get('level_tolerance_m',.03) and abs(self.rates.get(lift['joint'],math.inf))<.01
                if name==lift['door'] and not any(abs(lift_position-z)<lift.get('level_tolerance_m',.03) for z in lift['floors'].values()):needed=False
                if door.get('landing_floor'):
                    level=door['landing_floor']
                    needed=bool(self.lift_request and self.lift_request[0]==level and self.lift_request[1]
                        and abs(lift_position-lift['floors'][level])<lift.get('level_tolerance_m',.03)
                        and abs(self.rates.get(lift['joint'],math.inf))<.01)
                position=self.positions[door['joint']];target=door['opened'] if needed else door['closed']
                blocked=not needed and not self.safe_sweep(door)
                if blocked:target=position
                previous=self.outputs.get(door['joint'],position)
                if not self.hold:self.outputs[door['joint']]=position if blocked else max(previous-.025,min(previous+.025,target))
                mode=2 if abs(position-door['opened'])<.04 else 0 if abs(position)<.04 else 1
                door_rows[name]=dict(position_m=position,command_m=self.outputs.get(door['joint'],position),
                                      target_m=target,mode=mode,sweep_blocked=blocked)
                if self.rmf:
                    from rmf_door_msgs.msg import DoorState
                    msg=DoorState();msg.door_name=name;msg.door_time=self.get_clock().now().to_msg();msg.current_mode.value=mode;self.door_state.publish(msg)
            target=self.lift_target(lift_position,door_rows,now)
            previous=self.outputs.get(lift['joint'],lift_position)
            if not self.hold:self.outputs[lift['joint']]=max(previous-.02,min(previous+.02,target))
            for joint,value in self.outputs.items():self.targets[joint].publish(Float64(data=float(value)))
            for name,item in self.docking.items():
                b=self.bodies.get(name);d=self.config['docks'][item['dock']]
                good=(b and now-b['at']<.6 and b['speed'] is not None and b['speed']<.03 and b['angular'] is not None and abs(b['angular'])<.05 and math.hypot(b['xyz'][0]-d['x'],b['xyz'][1]-d['y'])<d['position_tolerance'] and abs(b['xyz'][2]-d.get('z',0.))<d.get('height_tolerance',.12) and abs(math.atan2(math.sin(b['yaw']-d['yaw']),math.cos(b['yaw']-d['yaw'])))<d['yaw_tolerance'])
                item['settled']=self.clock if good and item['settled'] is None else item['settled'] if good else None
                item['docked']=bool(good and self.clock-item['settled']>=1.)
                item['observation']=dict(b) if good else None
        energy = {name: model.state(now) for name, model in self.energy_models.items()}
        current_floor=next((f for f,z in lift['floors'].items() if lift_position is not None and abs(z-lift_position)<.03),'')
        if current_floor and abs(self.rates.get(lift['joint'],math.inf))<.01:self.current_floor_map=self.floor_maps.get(current_floor,'')
        if self.rmf:
            if not fresh:
                from rmf_door_msgs.msg import DoorState
                for name in self.doors:
                    door=DoorState();door.door_name=name;door.door_time=self.get_clock().now().to_msg();door.current_mode.value=3;self.door_state.publish(door)
            from rmf_lift_msgs.msg import LiftState
            msg=LiftState();msg.lift_name=lift['name'];msg.lift_time=self.get_clock().now().to_msg();msg.available_floors=list(lift['floors']);msg.current_floor=current_floor;msg.destination_floor=self.lift_request[0] if self.lift_request else current_floor
            rate=self.rates.get(lift['joint']);relevant=[row['mode'] for name,row in door_rows.items() if name==lift['door'] or self.config['doors'][name].get('landing_floor')==current_floor]
            msg.door_state=2 if relevant and all(mode==2 for mode in relevant) else 0 if relevant and all(mode==0 for mode in relevant) else 1
            msg.motion_state=3 if rate is None or not fresh else 0 if abs(rate)<.01 else 1 if rate>0 else 2
            msg.current_mode=2 if fresh else 4;msg.available_modes=[2];msg.session_id=self.lift_owner;self.lift_state.publish(msg)
        moving=(lift_position is not None and (not current_floor or self.lift_request and abs(lift_position-lift['floors'][self.lift_request[0]])>lift.get('level_tolerance_m',.03)))
        held=[name for name,b in self.bodies.items() if moving and 2.8<=b['xyz'][0]<=5.2 and -1.2<=b['xyz'][1]<=1.2]
        floors={name:self.energy[name]['floor'] for name in self.energy}
        for name,floor in floors.items():
            body=self.bodies.get(name)
            if body and now-body['at']<.6:self._request_robot_map(name,floor,now)
        robot_floor_maps={name:self.floor_maps.get(floor,'') for name,floor in floors.items()}
        held=sorted(set(held)|self.map_transitions)
        # Floor placement owns a distinct frame; AMCL alone owns map_frame ->
        # odom. Body truth classifies height only, never localization XY/yaw.
        for member in self.session['members']:
            name = member['id']
            frame = member.get('map_frame', 'map')
            if frame == 'map':
                continue
            target_z = float(self.config['lift']['floors'].get(floors[name], 0.0))
            self.floor_z[name] = target_z
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = frame
            t.transform.translation.z = target_z
            t.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(t)
        self._transport_tick(now, fresh, current_floor if abs(self.rates.get(lift['joint'], math.inf)) < .01 else '', door_rows)
        self._recovery_tick(now, fresh, door_rows)
        if self.recovery and self.recovery['phase'] == 'recovering':
            held = sorted(set(held) | set(self.recovery['passengers']))
        reservations = {row['robot']: 'queue-' + row['id'] for row in reversed(self.transport_queue.pending)}
        reservations.update({name: task.prefix for name, task in self.transports.items() if task.active})
        if self.recovery and self.recovery['phase'] == 'recovering':
            reservations.update({name: self.lift_owner for name in self.recovery['passengers']})
        floor_z={name:self.floor_z.get(name,0.0) for name in self.energy}
        state=dict(session_id=self.identity,ready=fresh,held_members=held,doors=door_rows,
                   lift=dict(position_m=lift_position,command_m=self.outputs.get(lift['joint'],lift_position),
                             current_floor=current_floor,destination=self.lift_request,floors=list(lift['floors'])),
               docks=self.config['docks'],docking=self.docking,energy=energy,robot_floors=floors,
               transports={name: task.state() for name, task in self.transports.items()},
               reservations=reservations, transport_queue=self.transport_queue.state(), lift_recovery=self.recovery,
               last_rejection=self.last_rejection,
               operator_id=(self.runtime or {}).get('operator_id',''),
               operator_fresh=bool(self.runtime and now-self.runtime_at<1. and self.runtime.get('operator_fresh')),
               localization_scan_enabled={name:self.localization_scan_enabled(name) for name in self.localization_scans},
               floor_z=floor_z,floor_maps=self.floor_maps,robot_floor_maps=robot_floor_maps,
               active_robot_maps=self.active_robot_maps,map_loading=sorted(self.map_transitions),
                   map_status=self.map_status,map_errors=self.map_errors,current_floor_map=self.current_floor_map,
                   operator_hold=self.hold,
               rmf_protocol=self.rmf, rmf_control_ready=self.rmf_authorized(),
               rmf_stopped_at=self.rmf_stopped_at if math.isfinite(self.rmf_stopped_at) else None,
               scope='Fresh observed body poses determine robot floors; member Nav2 maps load per observed floor and commands remain held until loading succeeds; authored energy estimate')
        self.publisher.publish(String(data=json.dumps(state,allow_nan=False)))


def main(args=None):
    from robot_lab_utils.process_lifetime import exit_with_parent
    exit_with_parent();rclpy.init(args=args);parser=argparse.ArgumentParser();parser.add_argument('--session',required=True);opts=parser.parse_args(rclpy.utilities.remove_ros_args()[1:]);node=Infrastructure(json.loads(Path(opts.session).read_text()))
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
