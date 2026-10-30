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
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry,Path as NavPath
from std_msgs.msg import String,Float64
from rosgraph_msgs.msg import Clock as ClockMessage
import yaml


class Infrastructure(Node):
    def __init__(self,session):
        super().__init__('fleet_infrastructure');self.session=session;self.identity=Path(session['directory']).name
        self.config=yaml.safe_load(Path(session['infrastructure_file']).read_text());self.token=uuid.uuid4().hex
        self.positions={};self.joint_at=-math.inf;self.clock=None;self.clock_at=-math.inf
        self.joint_stamp=None;self.rates={}
        self.runtime=None;self.runtime_at=-math.inf;self.bodies={};self.paths={};self.docking={};self.events=[]
        self.doors={name:False for name in self.config['doors']};self.outputs={};self.floor='L0';self.lift_request=None;self.lift_owner='';self.lift_at=-math.inf
        self.hold=False;self.lift_gui=False;self.door_owners={name:set() for name in self.doors}
        self.energy={m['id']:dict(joules=self.config['capacity_J'] if False else self.config['energy']['capacity_J']*self.config['energy']['initial_fraction'],last=None,power=None,at=-math.inf,valid=bool(self.config['energy'].get('effort_observation_verified')),floor='L0') for m in session['members']}
        self._effort_verified=bool(self.config['energy'].get('effort_observation_verified'))
        self._lift_prev=None;self._lift_prev_at=-math.inf
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
        self.rmf=False
        try:
            from rmf_door_msgs.msg import DoorRequest,DoorState
            from rmf_lift_msgs.msg import LiftRequest,LiftState
            self.door_state=self.create_publisher(DoorState,'/door_states',10);self.lift_state=self.create_publisher(LiftState,'/lift_states',10)
            self.create_subscription(DoorRequest,'/door_requests',self.door_request,10)
            self.create_subscription(LiftRequest,'/lift_requests',self.request_lift,10);self.rmf=True
        except ImportError:pass
        self.create_timer(.1,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def observe_clock(self,msg):
        value=msg.clock.sec+msg.clock.nanosec*1e-9
        if self.clock is not None and value<self.clock:
            self.doors={n:False for n in self.doors};self.outputs={n:self.positions[n] for n in self.targets if n in self.positions};self.lift_request=None;self.lift_owner='';self.bodies.clear();self.paths.clear();self.docking.clear()
            self.hold=True;self.outputs={};self.positions={};self.joint_stamp=None;self.rates={};self.joint_at=-math.inf
            self.door_owners={name:set() for name in self.doors}
            for item in self.energy.values():item['valid']=False;item['last']=None
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

    def path(self,name,msg):self.paths[name]=([(p.pose.position.x,p.pose.position.y) for p in msg.poses],time.monotonic())

    def motor(self,name,msg):
        entry=self.energy[name];stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        values=[(v,e) for n,v,e in zip(msg.name,msg.velocity,msg.effort) if 'wheel' in n]
        if not values or not all(math.isfinite(x) for pair in values for x in pair):return
        power=sum(max(0.,v*e) for v,e in values)/self.config['energy']['drive_efficiency']+self.config['energy']['hotel_W']
        if entry['last'] is not None:
            dt=stamp-entry['last']
            if not 0<dt<=.5:entry['valid']=False
            else:
                entry['valid']=self._effort_verified
                charge=self.config['energy']['charge_W'] if self.docking.get(name,{}).get('docked') else 0.
                entry['joules']=max(0.,min(self.config['energy']['capacity_J'],entry['joules']+(charge-power)*dt))
        entry.update(last=stamp,power=power,at=time.monotonic())

    def safe_sweep(self,door):
        now=time.monotonic();x0,x1,y0,y1=door['sweep']
        if set(self.bodies)!=set(self.energy) or any(now-b['at']>.6 for b in self.bodies.values()):return False
        base=self.positions.get(door.get('z_joint'),0.)
        return not any(x0<=b['xyz'][0]<=x1 and y0<=b['xyz'][1]<=y1 and base-.2<=b['xyz'][2]<base+2.5 for b in self.bodies.values())

    def door_request(self,msg):
        if msg.door_name in self.doors and msg.requested_mode.value in (0,2):
            if msg.requested_mode.value==2:self.door_owners[msg.door_name].add(msg.requester_id)
            else:self.door_owners[msg.door_name].discard(msg.requester_id)
            self.hold=False

    def request_lift(self,msg):
        if msg.lift_name!=self.config['lift']['name']:return
        now=time.monotonic()
        if self.lift_owner and msg.session_id!=self.lift_owner and now-self.lift_at<5:return
        if msg.request_type==0:
            if msg.session_id==self.lift_owner:self.lift_owner='';self.lift_request=None
            return
        if msg.request_type!=1 or not msg.session_id or len(msg.session_id)>128 or msg.destination_floor not in self.config['lift']['floors'] or msg.door_state not in (0,2):return
        self.lift_owner=msg.session_id;self.lift_at=now;self.lift_request=(msg.destination_floor,msg.door_state==2)
        self.lift_gui=False;self.hold=False

    def stop_command(self,msg):
        try:
            if len(msg.data)>4096:return
            data=json.loads(msg.data)
            if not isinstance(data,dict):return
            if data.get('session_id')==self.identity and data.get('op')=='all_stop':
                self.hold=True;self.outputs={n:self.positions[n] for n in self.targets if n in self.positions};self.lift_request=None;self.docking.clear()
        except (ValueError,KeyError,TypeError):pass

    def command(self,msg):
        try:
            if len(msg.data)>4096:return
            data=json.loads(msg.data)
            if not isinstance(data,dict):return
            if not self.runtime or time.monotonic()-self.runtime_at>1 or data.get('session_id')!=self.identity:return
            if data.get('operator_id')!=self.runtime['operator_id'] or not self.runtime.get('operator_fresh'):return
            op=data['op']
            if op=='door' and data['door'] in self.doors:
                if type(data['open']) is not bool:return
                self.doors[data['door']]=data['open'] is True;self.hold=False
            elif op=='lift':
                from types import SimpleNamespace
                self.request_lift(SimpleNamespace(lift_name=self.config['lift']['name'],session_id=data['operator_id'],request_type=1,destination_floor=data['floor'],door_state=2))
                self.lift_gui=True
            elif op=='dock':
                name,dock=data['robot'],data['dock'];target=self.config['docks'][dock]
                if name not in self.energy:return
                self.docking[name]=dict(dock=dock,request_id=data.get('request_id',uuid.uuid4().hex),settled=None,docked=False)
                command=dict(session_id=self.identity,operator_id=data['operator_id'],op='dispatch',targets=[name],x=target['x'],y=target['y'],yaw=target['yaw'],timeout=180.)
                self.fleet_command.publish(String(data=json.dumps(command)))
        except (ValueError,KeyError,TypeError):pass

    def tick(self):
        now=time.monotonic();fresh=now-self.joint_at<.6 and now-self.clock_at<.6
        if self.lift_gui and self.runtime and self.runtime.get('operator_id')==self.lift_owner and self.runtime.get('operator_fresh') and now-self.runtime_at<1:self.lift_at=now
        lift=self.config['lift'];lift_position=self.positions.get(lift['joint']);door_rows={}
        if fresh:
            for name,door in self.config['doors'].items():
                needed=self.doors[name] or bool(self.door_owners[name])
                # Actual Nav2 paths request the physical corridor door open.
                if name=='main':
                    needed |= any(now-at<1. and any(abs(x)<1.5 and abs(y)<1. for x,y in points) for points,at in self.paths.values())
                if name==lift['door'] and self.lift_request:
                    floor,opened=self.lift_request;needed=opened and abs(lift_position-lift['floors'][floor])<.03 and abs(self.rates.get(lift['joint'],math.inf))<.01
                if name==lift['door'] and not any(abs(lift_position-z)<.03 for z in lift['floors'].values()):needed=False
                position=self.positions[door['joint']];target=door['opened'] if needed else door['closed']
                blocked=not needed and not self.safe_sweep(door)
                if blocked:target=position
                previous=self.outputs.get(door['joint'],position)
                if not self.hold:self.outputs[door['joint']]=position if blocked else max(previous-.025,min(previous+.025,target))
                mode=2 if abs(position-door['opened'])<.04 else 0 if abs(position)<.04 else 1
                door_rows[name]=dict(position_m=position,target_m=target,mode=mode,sweep_blocked=blocked)
                if self.rmf:
                    from rmf_door_msgs.msg import DoorState
                    msg=DoorState();msg.door_name=name;msg.door_time=self.get_clock().now().to_msg();msg.current_mode.value=mode;self.door_state.publish(msg)
            target=lift_position
            if self.lift_request and now-self.lift_at<5:
                floor,_=self.lift_request
                if door_rows[lift['door']]['mode']==0 or abs(lift_position-lift['floors'][floor])<.03:target=lift['floors'][floor]
            previous=self.outputs.get(lift['joint'],lift_position)
            if not self.hold:self.outputs[lift['joint']]=max(previous-.02,min(previous+.02,target))
            for joint,value in self.outputs.items():self.targets[joint].publish(Float64(data=float(value)))
            for name,item in self.docking.items():
                b=self.bodies.get(name);d=self.config['docks'][item['dock']]
                good=(b and now-b['at']<.6 and b['speed'] is not None and b['speed']<.03 and b['angular'] is not None and abs(b['angular'])<.05 and math.hypot(b['xyz'][0]-d['x'],b['xyz'][1]-d['y'])<d['position_tolerance'] and abs(b['xyz'][2]-d.get('z',0.))<d.get('height_tolerance',.12) and abs(math.atan2(math.sin(b['yaw']-d['yaw']),math.cos(b['yaw']-d['yaw'])))<d['yaw_tolerance'])
                item['settled']=self.clock if good and item['settled'] is None else item['settled'] if good else None
                item['docked']=bool(good and self.clock-item['settled']>=1.)
                item['observation']=dict(b) if good else None
        energy={name:dict(fraction=v['joules']/self.config['energy']['capacity_J'] if v['valid'] and now-v['at']<.6 else None,power_W=v['power'] if now-v['at']<.6 else None,source='Authored battery model + reported wheel effort/rate; not hardware battery telemetry') for name,v in self.energy.items()}
        current_floor=next((f for f,z in lift['floors'].items() if lift_position is not None and abs(z-lift_position)<.03),'')
        if self.rmf:
            if not fresh:
                from rmf_door_msgs.msg import DoorState
                for name in self.doors:
                    door=DoorState();door.door_name=name;door.door_time=self.get_clock().now().to_msg();door.current_mode.value=3;self.door_state.publish(door)
            from rmf_lift_msgs.msg import LiftState
            msg=LiftState();msg.lift_name=lift['name'];msg.lift_time=self.get_clock().now().to_msg();msg.available_floors=list(lift['floors']);msg.current_floor=current_floor;msg.destination_floor=self.lift_request[0] if self.lift_request else current_floor
            rate=self.rates.get(lift['joint']);msg.door_state=door_rows.get(lift['door'],{}).get('mode',1)
            msg.motion_state=3 if rate is None or not fresh else 0 if abs(rate)<.01 else 1 if rate>0 else 2
            msg.current_mode=2 if fresh else 4;msg.available_modes=[2];msg.session_id=self.lift_owner;self.lift_state.publish(msg)
        # Multi-floor lift transport: when the lift car moves, robots aboard are
        # carried with it. Track the lift delta and apply it to any member whose
        # body is on the car footprint, then record floor transitions.
        lift_delta=0.
        if self._lift_prev is not None and lift_position is not None:
            lift_delta=lift_position-self._lift_prev
        self._lift_prev=lift_position;self._lift_prev_at=time.monotonic()
        on_lift=[name for name,b in self.bodies.items() if 2.8<=b['xyz'][0]<=5.2 and -1.2<=b['xyz'][1]<=1.2]
        if abs(lift_delta)>1e-4:
            for name in on_lift:
                b=self.bodies.get(name)
                if b:
                    b['xyz'][2]+=lift_delta
                    b['at']=time.monotonic()
                    new_floor=next((f for f,z in lift['floors'].items() if abs(b['xyz'][2]-z)<.03),self.energy[name]['floor'])
                    if new_floor!=self.energy[name]['floor']:
                        self.energy[name]['floor']=new_floor
        moving=(lift_position is not None and (not current_floor or self.lift_request and abs(lift_position-lift['floors'][self.lift_request[0]])>.03))
        held=[name for name,b in self.bodies.items() if moving and 2.8<=b['xyz'][0]<=5.2 and -1.2<=b['xyz'][1]<=1.2]
        floors={name:self.energy[name]['floor'] for name in self.energy}
        self.publisher.publish(String(data=json.dumps(dict(session_id=self.identity,ready=fresh,held_members=held,doors=door_rows,lift=dict(position_m=lift_position,current_floor=current_floor,destination=self.lift_request,floors=list(lift['floors'])),docks=self.config['docks'],docking=self.docking,energy=energy,robot_floors=floors,rmf_protocol=self.rmf,scope='Physical joint feedback and independently observed docking; lift transport tracks robots carried between L0/L1 floors; authored energy estimate'),allow_nan=False)))


def main(args=None):
    from robot_lab_utils.process_lifetime import exit_with_parent
    exit_with_parent();rclpy.init(args=args);parser=argparse.ArgumentParser();parser.add_argument('--session',required=True);opts=parser.parse_args(rclpy.utilities.remove_ros_args()[1:]);node=Infrastructure(json.loads(Path(opts.session).read_text()))
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
