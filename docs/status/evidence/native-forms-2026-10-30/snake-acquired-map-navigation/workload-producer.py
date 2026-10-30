"""Run real native extension tasks and record independent physical outcomes.

Control uses existing operator leases, estimators, planners and actuator owners.
Simulator body measurements are used only by this workload's observer.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import time
import uuid

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from .extension_contract import SCHEMA, RESULT_SCHEMA, CONTRACTS, LOSS_CONTRACT, endpoint_ok
from .workload_contract import validate_workload, verify_producer

PRODUCER_SHA256 = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class ExtensionWorkload(Node):
    def __init__(self, specification, directory):
        self.producer_sha256=verify_producer(specification,PRODUCER_SHA256)
        self.workload_sha256=digest(Path(directory)/'workload.json')
        super().__init__('robot_lab_measured_extension_workload',parameter_overrides=[Parameter('use_sim_time',value=True)])
        self.spec, self.directory = specification, Path(directory)
        self.workflow, self.task = specification['workflow'], specification['task']
        self.operator = 'extension-workload-'+uuid.uuid4().hex
        self.trace = (self.directory/'extension-trace.jsonl').open('w', buffering=1)
        self.trace_bytes = 0
        self.started = self.phase_at = time.monotonic()
        self.state_at = self.aux_at = self.hb_at = -math.inf
        self.runtime = self.aux = self.mapping = None
        self.session = self.sim_time = None
        self.phase, self.reason = 'waiting', 'Waiting for native session and operator lease'
        self.complete = self.interrupted = self.owned = self.succeeded = False
        self.truth, self.counts, self.body_subscriptions = {}, {}, []
        self.rows, self.settled, self.initial = [], {}, {}
        self.airborne={};self.landing_observed=set();self.loss_started=None
        self.forward_yaw=0.
        self.orientation_joints=[];self.orientation_samples=[]
        self.is_auv=False;self.goals=specification.get('goals',[]);self.mapping_endpoint=None
        self.goal_index = 0
        self.goal_handle = self.goal_future = self.result_future = None
        self.nav_status = None
        self.map_future = None
        self.contract = LOSS_CONTRACT if self.task=='operator_loss' else CONTRACTS[self.workflow]
        prefix = '/'+self.workflow
        self.command = self.create_publisher(String, prefix+'/command', 10)
        self.heartbeat = self.create_publisher(String, prefix+'/operator_heartbeat', 10)
        self.drive = self.create_publisher(Twist, '/key_vel' if self.workflow == 'gait' else prefix+'/key_vel', 10)
        self.create_subscription(String, prefix+'/state', self.observe_state, 10)
        self.create_subscription(Clock, '/clock', self.observe_clock, qos_profile_sensor_data)
        if self.workflow == 'marine':
            self.mission = self.create_publisher(String, '/marine/mission/command', 10)
            self.create_subscription(String, '/marine/mission/state', self.observe_aux, 10)
        if self.workflow == 'gait':
            self.create_subscription(JointState,'/joint_states',self.observe_orientation_joints,qos_profile_sensor_data)
            self.navigation = ActionClient(self, NavigateToPose, '/gait/navigate_to_pose')
            self.lifecycle=self.create_client(GetState,'/gait/bt_navigator/get_state')
            self.lifecycle_future=None;self.lifecycle_at=-math.inf;self.navigation_active=False
            self.map_service = self.create_client(Trigger, '/gait/mapping/save')
            self.create_subscription(String, '/gait/mapping/state', self.observe_mapping, 10)
        self.record(kind='start', specification=specification)

    def record(self, **values):
        row = json.dumps(dict(wall_monotonic=time.monotonic(), sim_time=self.sim_time, **values), allow_nan=False)+'\n'
        self.trace_bytes += len(row)
        if self.trace_bytes > 32*1024**2:
            self.complete = True; self.reason = 'Trace exceeded its 32 MiB budget'; return
        self.trace.write(row)

    def fail(self, reason):
        if not self.complete:
            self.complete = True; self.reason = str(reason); self.record(kind='failure', reason=self.reason)

    def transition(self, phase):
        self.phase, self.phase_at = phase, time.monotonic()
        self.record(kind='phase', phase=phase)

    def observe_clock(self, msg):
        value = msg.clock.sec+msg.clock.nanosec*1e-9
        if self.sim_time is not None and value < self.sim_time-1e-6:
            self.fail('Native clock regressed; reset invalidates this workload')
        self.sim_time = value

    def observe_aux(self, msg):
        self.aux, self.aux_at = json.loads(msg.data), time.monotonic()

    def observe_mapping(self, msg):
        self.mapping = json.loads(msg.data)

    def observe_state(self, msg):
        try:
            data = json.loads(msg.data)
            if self.session is not None and data['session_id'] != self.session:
                raise ValueError('Native session identity changed')
            if self.session is None:
                self.session = data['session_id']
                if self.workflow=='gait':
                    self.forward_yaw=float(data['drive_forward_yaw_rad'])
                    self.orientation_joints=list(data.get('drive_orientation_joints',[]))
                    if not math.isfinite(self.forward_yaw):raise ValueError('Source gait drive-frame yaw is invalid')
                if self.workflow == 'aerial_swarm':
                    topics = {m['id']:m['namespace']+'/px4/odometry_truth' for m in data['members']}
                    if not 2 <= len(topics) <= 4: raise ValueError('Actual 2–4 member aerial session required')
                else:
                    topics = {'body':'/marine/odometry' if self.workflow == 'marine' else '/odom/ground_truth'}
                    if self.workflow == 'marine' and 'auv' in data['robot_id']:
                        self.contract = CONTRACTS['auv'];self.is_auv=True
                        if self.task=='mapping' and self.spec['distance_m']<2.:
                            raise ValueError('AUV acquired-map transects require at least 2 m of physical forward travel')
                    if self.workflow=='marine' and data['robot_id']=='lab_usv':self.contract=CONTRACTS['surface']
                for name, topic in topics.items():
                    self.counts[name] = 0
                    self.body_subscriptions.append(self.create_subscription(Odometry, topic,
                        lambda message, name=name:self.observe_truth(name, message), qos_profile_sensor_data))
                self.record(kind='session', session_id=self.session, truth_topics=topics, contract=self.contract)
            self.runtime, self.state_at = data, time.monotonic()
            self.record(kind='native_state', state=data)
        except (ValueError, KeyError, TypeError) as exc:
            self.fail(exc)

    def observe_orientation_joints(self,msg):
        if not self.orientation_joints:return
        positions=dict(zip(msg.name,msg.position))
        if not all(name in positions and math.isfinite(positions[name]) for name in self.orientation_joints):return
        angles=[0.];heading=0.
        for name in self.orientation_joints:heading+=positions[name];angles.append(heading)
        offset=math.atan2(sum(map(math.sin,angles)),sum(map(math.cos,angles)))
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        self.orientation_samples.append((stamp,offset));self.orientation_samples=self.orientation_samples[-8:]

    def observe_truth(self, name, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        values = [p.x,p.y,p.z,q.x,q.y,q.z,q.w]
        if msg.header.frame_id not in ('map','odom'):
            self.fail('Independent body pose is not in selected-world coordinates'); return
        if not all(map(math.isfinite,values)) or abs(sum(v*v for v in values[3:])-1)>.01:
            self.fail('Invalid independent physical body pose'); return
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        if self.orientation_joints:
            if not self.orientation_samples:return
            nearest=min(self.orientation_samples,key=lambda sample:abs(sample[0]-stamp))
            if abs(nearest[0]-stamp)>.04:return
            yaw+=nearest[1]
        if self.workflow=='gait':yaw=math.atan2(math.sin(yaw+self.forward_yaw),math.cos(yaw+self.forward_yaw))
        pose = [p.x,p.y,yaw] if self.workflow == 'gait' else [p.x,p.y,p.z]
        previous = self.truth.get(name)
        if previous and stamp < previous['stamp']:
            self.fail('Independent body timestamp regressed'); return
        if previous and stamp == previous['stamp']: return
        dt = stamp-previous['stamp'] if previous else None
        speed = math.dist(pose[:2] if self.workflow=='gait' else pose,
                          previous['pose'][:2] if self.workflow=='gait' else previous['pose'])/dt if dt and dt<=.5 else None
        rate = math.atan2(math.sin(yaw-previous['yaw']),math.cos(yaw-previous['yaw']))/dt if dt and dt<=.5 else None
        self.truth[name] = dict(pose=pose,yaw=yaw,stamp=stamp,speed=speed,yaw_rate=rate,at=time.monotonic(),frame=msg.header.frame_id)
        self.counts[name] += 1
        # Aerial Gazebo truth carries simulation stamps without publishing ROS /clock.
        if self.workflow == 'aerial_swarm': self.sim_time = min(v['stamp'] for v in self.truth.values())
        if self.counts[name]%5 == 0: self.record(kind='body_truth',member_id=name,**self.truth[name])

    def send(self, op, mission=False, **values):
        data = dict(session_id=self.session, operator_id=self.operator, op=op, **values)
        if self.workflow == 'aerial_swarm': data['targets'] = [m['id'] for m in self.runtime['members']]
        (self.mission if mission else self.command).publish(String(data=json.dumps(data,allow_nan=False)))
        self.record(kind='command', mission=mission, command=data)

    def stable_endpoint(self, name, target):
        body = self.truth.get(name)
        if (body is None or time.monotonic()-body['at']>.5
                or not endpoint_ok(body['pose'],target,body['speed'],body['yaw_rate'],self.contract)):
            self.settled.pop(name,None); return None
        start = self.settled.setdefault(name,body['stamp'])
        duration = body['stamp']-start
        if duration < self.contract['settle_sim_s']: return None
        return dict(requested=target, physical_pose=body['pose'], linear_m_s=body['speed'],
                    angular_rad_s=body['yaw_rate'], settled_sim_s=duration, member_id=name)

    def start_goal(self):
        target = self.goals[self.goal_index]
        self.settled.clear()
        if self.workflow == 'marine':
            self.send('plan',mission=True,**dict(zip('xyz',target)))
        else:
            now=time.monotonic()
            if self.lifecycle_future is not None and self.lifecycle_future.done():
                self.navigation_active=self.lifecycle_future.result().current_state.id==3
                self.lifecycle_future=None
            if self.lifecycle_future is None and now-self.lifecycle_at>1. and self.lifecycle.service_is_ready():
                self.lifecycle_at=now;self.lifecycle_future=self.lifecycle.call_async(GetState.Request())
            if not self.navigation_active or not self.navigation.server_is_ready(): return False
            registration=self.runtime.get('goal_world_to_map')
            if not isinstance(registration,list) or len(registration)!=3 or not all(map(math.isfinite,registration)):
                self.fail('Native gait session lacks its configured world-to-map registration');return False
            x,y,heading=registration
            c,s=math.cos(heading),math.sin(heading)
            converted=[x+c*target[0]-s*target[1],y+s*target[0]+c*target[1],target[2]+heading]
            goal=NavigateToPose.Goal();goal.pose.header.frame_id='map'
            goal.pose.header.stamp=self.get_clock().now().to_msg()
            goal.pose.pose.position.x,goal.pose.pose.position.y=converted[:2]
            goal.pose.pose.orientation.z=math.sin(converted[2]/2);goal.pose.pose.orientation.w=math.cos(converted[2]/2)
            self.record(kind='estimated_frame_goal', world_goal=target, map_goal=converted)
            self.goal_future=self.navigation.send_goal_async(goal);self.result_future=None;self.nav_status=None
        self.transition('planning');return True

    def navigation_tick(self):
        if self.phase == 'armed': self.start_goal(); return
        if self.workflow == 'marine':
            if not self.aux or time.monotonic()-self.aux_at>1: return
            status=self.aux['status']
            if self.phase=='planning' and status=='planned': self.send('execute',mission=True); self.transition('executing')
            if self.phase=='planning' and time.monotonic()-self.phase_at>25:
                self.fail('Native planner did not produce a route: '+self.aux['reason']);return
            terminal=self.phase=='executing' and status=='completed'
            if self.phase=='executing' and status in ('failed','canceled','error'):
                self.fail('Native mission failed: '+self.aux['reason']);return
        else:
            if self.goal_future and self.goal_future.done():
                self.goal_handle=self.goal_future.result();self.goal_future=None
                if not self.goal_handle.accepted: self.fail('Native Nav2 rejected goal');return
                self.result_future=self.goal_handle.get_result_async();self.transition('executing')
            if self.result_future and self.result_future.done():
                self.nav_status=self.result_future.result().status;self.result_future=None
                if self.nav_status != 4: self.fail('Native Nav2 terminal status '+str(self.nav_status));return
            terminal=self.nav_status==4
        if terminal:
            row=self.stable_endpoint('body',self.goals[self.goal_index])
            if row:
                row.update(index=self.goal_index,native_terminal=True,nav2_status=self.nav_status)
                self.record(kind='endpoint',measurement=row);self.goal_index+=1
                if self.task=='mapping':
                    self.mapping_endpoint=row
                    self.send('stop');self.send('save_map',mission=True);self.transition('saving')
                else:
                    self.rows.append(row)
                    if self.goal_index==len(self.goals): self.succeeded=self.complete=True;self.reason='Every native goal physically settled'
                    else: self.transition('armed')

    def mapping_tick(self):
        if self.is_auv:
            if self.phase=='armed':
                if not self.aux or not self.aux['ready'] or time.monotonic()-self.aux_at>1.:return
                # Route generation uses sensor-estimated map pose, never the
                # independent body observer. Include the fixed arrival envelope
                # in the requested forward length; physical travel is verified.
                start=list(self.aux['position']);heading=self.aux['estimated_yaw']+math.pi
                distance=self.spec['distance_m']+self.contract['position_m']+.2
                self.goals=[[start[0]+distance*math.cos(heading),start[1]+distance*math.sin(heading),start[2]]]
                self.initial=self.truth['body']['pose'][:]
                self.start_goal();return
            if self.phase in ('planning','executing'):
                self.navigation_tick();return
            if self.phase!='saving':return
        if self.phase=='armed':
            self.initial=self.truth['body']['pose'][:];self.transition('acquiring')
        body=self.truth['body']
        if self.phase=='acquiring':
            traveled=math.dist(body['pose'][:2],self.initial[:2])
            if traveled>=self.spec['distance_m']: self.drive.publish(Twist());self.transition('mapping_brake')
            else:
                msg=Twist();msg.linear.x=.02 if self.workflow=='gait' else .1;self.drive.publish(msg)
        if self.phase=='mapping_brake':
            self.drive.publish(Twist())
            if body['speed'] is not None and body['speed']<=self.contract['linear_m_s']:
                self.settled.setdefault('mapping',body['stamp'])
                if body['stamp']-self.settled['mapping']>=1.:
                    self.send('stop')
                    if self.workflow=='marine': self.send('save_map',mission=True)
                    elif self.map_service.service_is_ready(): self.map_future=self.map_service.call_async(Trigger.Request())
                    else: self.fail('Actual map save service unavailable');return
                    self.transition('saving')
            else:self.settled.pop('mapping',None)
        if self.phase=='saving':
            if self.map_future and self.map_future.done() and not self.map_future.result().success:
                self.fail('Native map save rejected: '+self.map_future.result().message);return
            status=self.aux.get('mapping_status') if self.workflow=='marine' else (self.mapping or {}).get('status')
            artifact=self.aux.get('saved_map') if self.workflow=='marine' else (self.mapping or {}).get('result')
            if status=='failed': self.fail('Native map archival failed');return
            if status=='saved' and artifact:
                from .extension_result import inspect_map
                observation=inspect_map(artifact)
                traveled=math.dist(body['pose'][:2],self.initial[:2])
                if traveled<self.spec['distance_m']:
                    self.fail('Actual acquisition did not reach the requested physical travel');return
                self.rows.append(dict(map_artifact=artifact,observation=observation,
                    physical_start=self.initial,physical_pose=body['pose'],distance_m=traveled,route_endpoint=self.mapping_endpoint))
                self.succeeded=self.complete=True;self.reason='Actual travel and nonempty acquired map archived'

    def aerial_tick(self):
        members=self.runtime['members']
        if self.phase=='waiting' and all(m['ready'] for m in members): self.send('check');self.transition('checking')
        elif self.phase=='checking':
            replies=[m.get('last_service') or {} for m in members]
            if all(r.get('operation')=='check' for r in replies):
                if not all(r['accepted'] for r in replies): self.fail('Native takeoff clearance rejected: '+str(replies));return
                self.send('takeoff');self.transition('ascending')
        elif self.phase=='ascending':
            if time.monotonic()-self.phase_at>12. and any((m.get('status') or {}).get('phase') in ('idle','telemetry_lost') for m in members):
                self.fail('Native FCU did not arm or lost readiness; inspect owned PX4 health logs');return
            if any((m.get('last_service') or {}).get('operation')=='takeoff' and not m['last_service']['accepted'] for m in members):
                self.fail('Native takeoff rejected');return
            if all((m.get('status') or {}).get('phase')=='flying' for m in members):
                if self.task=='operator_loss':
                    if not all(m['id'] in self.initial and self.truth[m['id']]['pose'][2]-self.initial[m['id']][2]>self.contract['airborne_gain_m'] for m in members):return
                    self.airborne={m['id']:self.truth[m['id']]['pose'][:] for m in members}
                    self.loss_started=time.monotonic();self.settled.clear();self.transition('operator_loss')
                else:self.send('formation');self.transition('formation')
        elif self.phase=='formation':
            anchor=self.runtime.get('anchor')
            if not anchor:return
            rows=[]
            for i,name in enumerate(sorted(self.counts)):
                target=[anchor[0],anchor[1]+(i-(len(members)-1)/2)*self.runtime['spacing'],anchor[2]]
                row=self.stable_endpoint(name,target)
                if row is not None: rows.append(row)
            if len(rows)==len(members):
                self.rows=rows;self.send('land');self.transition('landing')
        elif self.phase=='operator_loss':
            rows=[]
            for m in members:
                name=m['id'];status=m.get('status') or {};body=self.truth[name]
                if status.get('phase')=='landing':self.landing_observed.add(name)
                settled=(not status.get('armed',True) and status.get('landed_state')==1
                    and abs(body['pose'][2]-self.initial[name][2])<=self.contract['landed_height_m']
                    and body['speed'] is not None and body['speed']<=self.contract['linear_m_s'])
                if not settled:self.settled.pop(name,None);continue
                start=self.settled.setdefault(name,body['stamp'])
                if body['stamp']-start>=self.contract['settle_sim_s'] and name in self.landing_observed:
                    rows.append(dict(member_id=name,physical_initial=self.initial[name],physical_airborne=self.airborne[name],
                        physical_pose=body['pose'],linear_m_s=body['speed'],settled_sim_s=body['stamp']-start,
                        observed_landing=name in self.landing_observed,landed_state=status['landed_state'],disarmed=not status['armed'],
                        heartbeat_silence_wall_s=time.monotonic()-self.loss_started))
            if len(rows)==len(members):
                self.rows=rows;self.succeeded=self.complete=True;self.reason='Operator lease loss caused actual native landing/disarm for every airborne member'
        elif self.phase=='landing' and all(not (m.get('status') or {}).get('armed',True) for m in members):
            self.succeeded=self.complete=True;self.reason='Actual formation settled and every FCU landed/disarmed'

    def tick(self):
        now=time.monotonic()
        if self.interrupted:self.fail('Operator interrupted workload')
        if self.complete:return
        if now-self.started>self.spec['timeout_s']:self.fail('Measured extension workload timed out');return
        if not self.runtime:
            if now-self.started>60:self.fail('No native session within startup deadline')
            return
        if now-self.state_at>1.:self.fail('Native session feedback lost');return
        if self.phase!='operator_loss' and now-self.hb_at>.15:
            self.hb_at=now;self.heartbeat.publish(String(data=json.dumps(dict(session_id=self.session,operator_id=self.operator))))
        if self.runtime['operator_id']!=self.operator:
            if self.owned:self.fail('Native operator lease lost')
            return
        self.owned=True
        if self.workflow=='aerial_swarm' and self.phase=='waiting':
            for name,body in self.truth.items():self.initial.setdefault(name,body['pose'][:])
        if not self.counts or any(n<3 for n in self.counts.values()):return
        if any(now-v['at']>1. for v in self.truth.values()):
            if self.phase!='waiting':self.fail('Independent physical observer lost')
            return
        if self.workflow=='aerial_swarm':self.aerial_tick();return
        if self.phase=='waiting' and self.runtime['ready']:
            if self.workflow=='marine' and (not self.aux or not self.aux['ready']):return
            self.send('arm');self.transition('arming')
        elif self.phase=='arming' and self.runtime['enabled']:self.transition('armed')
        elif self.phase not in ('waiting','arming'):
            if self.task=='mapping':self.mapping_tick()
            else:self.navigation_tick()

    def finish(self, specification_path):
        deadline=time.monotonic()+(30. if self.workflow=='aerial_swarm' else 8.)
        cancel=None
        if self.goal_handle and self.nav_status is None:cancel=self.goal_handle.cancel_goal_async()
        if self.owned:
            self.drive.publish(Twist())
            if self.workflow=='marine':self.send('cancel',mission=True)
            self.send('land' if self.workflow=='aerial_swarm' else 'stop')
        released=False
        while rclpy.ok() and self.owned and time.monotonic()<deadline:
            rclpy.spin_once(self,timeout_sec=.05)
            if self.workflow=='aerial_swarm':
                released=all(not (m.get('status') or {}).get('armed',True) for m in self.runtime['members'])
            else:released=not self.runtime.get('enabled',True) and (cancel is None or cancel.done())
            if released:break
        self.record(kind='finish',reason=self.reason,control_released=released);self.trace.close()
        result=dict(schema=RESULT_SCHEMA,workflow=self.workflow,task=self.task,session_id=self.session,
            run_id=os.environ['ROBOT_LAB_EXPERIMENT_ID'],ros_domain_id=int(os.environ['ROS_DOMAIN_ID']),
            success=bool(self.succeeded and released),reason=self.reason,measurements=self.rows,
            goal_contract=self.contract,truth_samples=self.counts,control_released=released,
            wall_duration_s=time.monotonic()-self.started,contacts=None,minimum_swept_clearance_m=None,
            producer_sha256=self.producer_sha256,workload_sha256=self.workload_sha256,
            trace_sha256=digest(self.directory/'extension-trace.jsonl'),
            scope='Actual native task and independent physical measurements; parent runner owns process cleanup')
        path=self.directory/'extension-mission-result.json';temporary=path.with_suffix('.new')
        temporary.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');temporary.replace(path)
        return result


def main(args=None):
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--spec',required=True);options=parser.parse_args(args)
    path=Path(options.spec).resolve()
    if path.stat().st_size>64*1024:parser.error('Workload exceeds 64 KiB')
    specification=validate_workload(json.loads(path.read_text()))
    if specification['schema']!=SCHEMA:parser.error('Select native extension workload')
    output=os.environ.get('ROBOT_LAB_EXPERIMENT_OUTPUT')
    if not output or not os.environ.get('ROBOT_LAB_EXPERIMENT_ID') or not os.environ.get('ROS_DOMAIN_ID'):
        parser.error('Runner output, identity and isolated domain required')
    directory=Path(output).resolve();directory.mkdir(parents=True,exist_ok=True)
    if directory.stat().st_dev==Path('/').stat().st_dev:parser.error('Artifacts must stay on mounted SSD')
    rclpy.init(args=[]);node=ExtensionWorkload(specification,directory)
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_:setattr(node,'interrupted',True))
    try:
        while rclpy.ok() and not node.complete:rclpy.spin_once(node,timeout_sec=.05);node.tick()
    except Exception as exc:node.fail('Workload exception: '+str(exc))
    finally:
        result=node.finish(path);node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    print(json.dumps(result),flush=True)
    return 0 if result['success'] else 1


if __name__=='__main__':raise SystemExit(main())
