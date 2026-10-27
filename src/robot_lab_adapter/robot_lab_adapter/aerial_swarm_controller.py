"""Actual per-FCU group commands with operator/member/geometry guards."""
import json
import math
from pathlib import Path
import time
import uuid
import numpy as np
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped,Twist
from nav_msgs.msg import Odometry
from robot_lab_utils.aerial_planning import StaticScene
from .aerial_swarm_model import velocities


class AerialSwarmController(Node):
    def __init__(self,session):
        super().__init__('aerial_swarm_coordinator')
        self.session=session;self.scene=StaticScene(session['source_world'])
        self.operator='';self.hb_at=-math.inf;self.mode='idle';self.targets=[];self.command=(0.,0.,0.,0.)
        self.command_at=-math.inf;self.anchor=None;self.spacing=3.;self.reason='Connected neutral; explicit checks and Takeoff required'
        self.members={}
        self.last_status=0.;self.last_output={};self.pending=[]
        self.generation=0
        self.takeoff_targets=[]
        self.pub=self.create_publisher(String,'/aerial_swarm/state',10)
        self.lease=self.create_publisher(String,'/aerial_swarm/vehicle_lease',10)
        self.create_subscription(String,'/aerial_swarm/command',self.operation,10)
        self.create_subscription(String,'/aerial_swarm/operator_heartbeat',self.heartbeat,10)
        self.create_subscription(Twist,'/aerial_swarm/key_vel',self.drive,10)
        for item in session['members']:
            row=dict(item,status=None,status_at=-math.inf,pose=None,pose_at=-math.inf,truth=None,truth_at=-math.inf,result=None)
            prefix=item['namespace'];self.members[item['id']]=row
            row['velocity_pub']=self.create_publisher(Twist,prefix+'/key_vel',10)
            row['goal_pub']=self.create_publisher(PoseStamped,prefix+'/px4/goal',10)
            row['request_pub']=self.create_publisher(String,prefix+'/command_request',10)
            self.create_subscription(String,prefix+'/command_reply',lambda msg,m=row:self.observe_reply(m,msg),10)
            self.create_subscription(String,prefix+'/px4/status',lambda msg,m=row:self.observe_status(m,msg),10)
            self.create_subscription(Odometry,prefix+'/px4/odometry',lambda msg,m=row:self.observe_pose(m,msg),10)
            self.create_subscription(Odometry,prefix+'/px4/odometry_truth',lambda msg,m=row:self.observe_truth(m,msg),10)
        self.events=(Path(session['directory'])/'events.jsonl').open('a',buffering=1);self.event_bytes=0
        self.create_timer(.1,self.tick)

    def event(self,kind,**data):
        row=json.dumps(dict(event=kind,wall=time.time(),**data),allow_nan=False)+'\n'
        if self.event_bytes+len(row)<8*1024**2:self.events.write(row);self.event_bytes+=len(row)

    def observe_status(self,member,msg):
        try:
            data=json.loads(msg.data)
            if 'phase' in data:member['status'],member['status_at']=data,time.monotonic()
        except (ValueError,TypeError):pass

    def observe_pose(self,member,msg):
        if msg.header.frame_id!='map' or msg.child_frame_id!=member['namespace'].strip('/')+'/base_link':return
        p=msg.pose.pose.position;q=msg.pose.pose.orientation
        values=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
        if not np.isfinite(values).all():return
        member['pose']=[p.x,p.y,p.z];member['yaw']=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        member['pose_at']=time.monotonic()

    def observe_truth(self,member,msg):
        p=msg.pose.pose.position
        if msg.header.frame_id=='map' and np.isfinite([p.x,p.y,p.z]).all():
            member['truth'],member['truth_at']=[p.x,p.y,p.z],time.monotonic()

    def fresh(self,row):
        return (time.monotonic()-row['status_at']<1.2 and time.monotonic()-row['pose_at']<.6
                and row['status'] and row['status'].get('position_ready'))

    def heartbeat(self,msg):
        try:
            data=json.loads(msg.data);operator=data['operator_id']
            if data['session_id']!=self.session['session_id'] or not isinstance(operator,str) or not 1<=len(operator)<=80:return
            if self.operator and operator!=self.operator and time.monotonic()-self.hb_at<.8:return
            if operator!=self.operator:self.hold('Operator changed; neutral ownership')
            self.operator,self.hb_at=operator,time.monotonic()
        except (ValueError,KeyError,TypeError):pass

    def service(self,member,op):
        if len(self.pending)>=16:raise ValueError('Wait for outstanding aerial service replies')
        request=uuid.uuid4().hex
        member['request_pub'].publish(String(data=json.dumps(dict(session_id=self.session['session_id'],
            operator_id=self.operator,generation=self.generation,request_id=request,op=op))))
        self.pending.append(dict(member=member['id'],op=op,request_id=request,generation=self.generation,started=time.monotonic()))
        self.event('service_requested',member=member['id'],operation=op)

    def observe_reply(self,member,msg):
        try:
            data=json.loads(msg.data)
            request=next((p for p in self.pending if p['request_id']==data['request_id'] and p['member']==member['id']),None)
            if request is None:return
            self.pending.remove(request)
            if data['generation']!=self.generation:return
            member['result']=dict(operation=request['op'],accepted=bool(data['accepted']),message=data['message'])
            if request['op']=='check' and data['accepted']:
                member['result'].update(check_position=data.get('check_position'),checked_at=time.monotonic())
            self.event('service_result',member=member['id'],**member['result'])
            if request['op']=='takeoff' and not data['accepted']:
                targets=list(self.takeoff_targets)
                self.hold('Group takeoff rejected; accepted peers aborted/landed: '+data['message'])
                for name in targets:self.service(self.members[name],'land')
        except (ValueError,KeyError,TypeError):pass

    def hold(self,reason,invalidate=True):
        if invalidate:
            self.generation+=1
            self.pending.clear()
        self.mode='idle';self.command=(0.,0.,0.,0.);self.reason=reason
        for member in self.members.values():
            member['velocity_pub'].publish(Twist())
            if member['status'] and member['status']['phase'] in ('flying','ascending'):
                try:self.service(member,'hold')
                except ValueError:pass

    def operation(self,msg):
        try:
            if len(msg.data)>4096:raise ValueError('Aerial group command too large')
            data=json.loads(msg.data)
            if data.get('session_id')!=self.session['session_id']:return
            op=data['op']
            if op=='stop':self.hold('Operator group Hold');return
            if data.get('operator_id')!=self.operator or time.monotonic()-self.hb_at>.8:
                raise ValueError('Connect a live aerial operator')
            names=data.get('targets',[])
            if not names or len(set(names))!=len(names) or any(n not in self.members for n in names):raise ValueError('Choose distinct actual FCU member ids')
            members=[self.members[name] for name in names]
            if op in ('check','takeoff','land','hold'):
                if op=='takeoff':
                    for member in members:
                        checked=member.get('result') or {}
                        if (not self.fresh(member) or not member['status'].get('takeoff_ready')
                                or checked.get('operation')!='check' or not checked.get('accepted')
                                or not checked.get('check_position') or time.monotonic()-checked.get('checked_at',-math.inf)>30.
                                or math.dist(member['pose'],checked['check_position'])>.04):
                            raise ValueError('Every selected FCU needs a fresh clear result at its current stable landed pose; check again')
                    self.takeoff_targets=names
                if op!='check':self.hold('Explicit '+op+' requested',invalidate=op!='takeoff')
                if op=='takeoff' and any(not self.fresh(m) for m in members):raise ValueError('Fresh member FCU position required')
                for member in members:self.service(member,op)
            elif op in ('manual','formation','rendezvous','dispersion','flocking'):
                if any(not self.fresh(m) or m['status']['phase']!='flying' for m in members):raise ValueError('All selected members must be airborne/fresh before group movement')
                if op!='manual' and len(members)<2:raise ValueError('Swarm mode requires at least two airborne vehicles')
                self.hold('New group mode starts neutral')
                self.mode=op;self.targets=names;self.command_at=-math.inf
                self.spacing=float(data.get('spacing',3.))
                self.anchor=[float(data[k]) for k in ('x','y','z')] if all(k in data for k in ('x','y','z')) else None
                if op=='formation' and self.anchor is None:
                    self.anchor=np.mean([m['pose'] for m in members],axis=0).tolist()
                if op!='manual':velocities(op,{m['id']:m['pose'] for m in members},self.spacing,self.anchor)
                self.reason=op+' active; physical separation/mission qualification pending'
            else:raise ValueError('Unsupported aerial group operation')
        except (ValueError,KeyError,TypeError) as exc:
            self.reason=str(exc);self.event('rejected',reason=str(exc))

    def drive(self,msg):
        if self.mode!='manual':return
        values=[msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.z]
        if not np.isfinite(values).all():self.hold('Invalid group velocity');return
        self.command=tuple(max(-cap,min(cap,v)) for v,cap in zip(values,[.4,.4,.25,.2]));self.command_at=time.monotonic()

    def tick(self):
        now=time.monotonic();operator_fresh=now-self.hb_at<.8
        if operator_fresh:
            self.lease.publish(String(data=json.dumps(dict(session_id=self.session['session_id'],operator_id=self.operator,generation=self.generation))))
        elif self.mode!='idle':self.hold('Aerial operator heartbeat lost; workers request Land')
        for request in list(self.pending):
            if now-request['started']>(25. if request['op']=='check' else 10.):
                row=dict(operation=request['op'],accepted=False,message='Actual worker reply timed out')
                self.members[request['member']]['result']=row;self.pending.remove(request)
                self.event('service_result',member=request['member'],**row)
        if self.mode!='idle':
            members=[self.members[n] for n in self.targets]
            if any(not self.fresh(m) or m['status']['phase']!='flying' for m in members):self.hold('Member feedback/airborne state lost')
            else:
                commands=None
                if self.mode!='manual':
                    try:commands=velocities(self.mode,{m['id']:m['pose'] for m in members},self.spacing,self.anchor)
                    except ValueError as exc:self.hold(str(exc))
                for member in members if self.mode!='idle' else []:
                    if self.mode=='manual':
                        vx,vy,vz,wz=self.command if now-self.command_at<.35 else (0.,0.,0.,0.)
                        yaw=member['yaw'];world=np.array([vx*math.cos(yaw)-vy*math.sin(yaw),vx*math.sin(yaw)+vy*math.cos(yaw),vz])
                    else:world=np.asarray(commands[member['id']]);wz=0.
                    delta=np.clip(world*.5,-.25,.25);target=np.asarray(member['pose'])+delta
                    target[2]=max(member['origin'][2]+.5,min(member['origin'][2]+10.,target[2]))
                    try:clear=self.scene.segment(member['pose'],target,.8,now+.04,lambda:False)
                    except TimeoutError:clear=False
                    for other in self.members.values():
                        if other is member:continue
                        if not self.fresh(other) or math.dist(target,other['pose'])<2.:
                            clear=False
                    if not clear:
                        member['velocity_pub'].publish(Twist());self.reason='Holding: source collision corridor or peer separation'
                        continue
                    if self.mode=='manual':
                        output=Twist();output.linear.x=vx;output.linear.y=vy;output.linear.z=vz;output.angular.z=wz
                        member['velocity_pub'].publish(output)
                    else:
                        # Preserve PX4's integrated position hold. Replacing
                        # its target with a measured-pose lookahead each tick
                        # discarded accumulated error and caused altitude droop.
                        yaw=member['yaw'];output=Twist()
                        output.linear.x=float(world[0]*math.cos(yaw)+world[1]*math.sin(yaw))
                        output.linear.y=float(-world[0]*math.sin(yaw)+world[1]*math.cos(yaw))
                        output.linear.z=float(world[2])
                        member['velocity_pub'].publish(output)
        if now-self.last_status>.2:
            self.last_status=now
            rows=[dict(id=m['id'],instance=m['instance'],system_id=m['system_id'],port=m['port'],namespace=m['namespace'],
                ready=self.fresh(m),status=m['status'],position=m['pose'],truth=m['truth'],last_service=m['result']) for m in self.members.values()]
            self.pub.publish(String(data=json.dumps(dict(schema_version=1,session_id=self.session['session_id'],
                map=self.session['map'],backend='Gazebo Harmonic / native PX4',directory=self.session['directory'],
                operator_id=self.operator,operator_fresh=operator_fresh,mode=self.mode,reason=self.reason,members=rows,
                position_reference='Known landed spawn prior until first accepted Takeoff; committed FCU odometry thereafter; simulator truth is separate',
                scope='Experimental actual shared-world multi-FCU controls; flight/swarm/clearance/loss qualification pending'),allow_nan=False)))

    def close(self):
        if self.context.ok():self.hold('Aerial shared session closing')
        self.events.close()
