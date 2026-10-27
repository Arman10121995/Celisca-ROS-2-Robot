"""Real fleet route producer with independent selected-world body measurements.

Attach to an isolated, already launched Robot Lab fleet. Dispatch never counts
as completion: each owned task needs a terminal result and measured settling.
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
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from .workload_contract import FLEET_WORKLOAD_SCHEMA, FLEET_RESULT_SCHEMA, FLEET_GOAL_CONTRACT, validate_workload


def endpoint_ok(goal, pose, speed, yaw_rate):
    if pose is None or speed is None or yaw_rate is None:
        return False
    error=math.atan2(math.sin(pose[2]-goal['yaw']),math.cos(pose[2]-goal['yaw']))
    return (all(map(math.isfinite,[*pose,speed,yaw_rate]))
        and math.hypot(pose[0]-goal['x'],pose[1]-goal['y'])<=FLEET_GOAL_CONTRACT['position_m']
        and abs(error)<=FLEET_GOAL_CONTRACT['heading_rad']
        and 0<=speed<=FLEET_GOAL_CONTRACT['linear_m_s'] and abs(yaw_rate)<=FLEET_GOAL_CONTRACT['angular_rad_s'])


class FleetWorkload(Node):
    def __init__(self, specification, directory):
        super().__init__('robot_lab_measured_fleet_workload')
        self.specification,self.directory=specification,Path(directory)
        self.operator='fleet-workload-'+uuid.uuid4().hex
        self.tag=uuid.uuid4().hex
        self.trace=(self.directory/'fleet-trace.jsonl').open('w',buffering=1)
        self.trace_bytes=0
        self.started=time.monotonic();self.state_at=self.hb_at=self.clock_at=-math.inf
        self.sim_time=self.runtime=self.session_id=None
        self.owned=self.complete=self.interrupted=False
        self.reason='Waiting for an actual fleet'
        self.requests={};self.truth={};self.truth_count={};self.last_trace={}
        self.measurements=[];self.truth_subscriptions=[]
        self.command=self.create_publisher(String,'/fleet/command',10)
        self.heartbeat=self.create_publisher(String,'/fleet/operator_heartbeat',10)
        self.create_subscription(String,'/fleet/state',self.observe_state,10)
        self.create_subscription(Clock,'/clock',self.observe_clock,qos_profile_sensor_data)
        self.record(kind='workload_start',specification=specification)

    def record(self, **values):
        row=json.dumps(dict(wall_monotonic=time.monotonic(),sim_time=self.sim_time,**values),allow_nan=False)+'\n'
        self.trace_bytes+=len(row)
        if self.trace_bytes>32*1024*1024:
            self.complete=True;self.reason='Actual fleet trace exceeded its 32 MiB budget';return
        self.trace.write(row)

    def fail(self, reason):
        if not self.complete:
            self.reason=str(reason);self.complete=True;self.record(kind='failure',reason=self.reason)

    def observe_clock(self, msg):
        stamp=msg.clock.sec+msg.clock.nanosec*1e-9
        if self.sim_time is not None and stamp<self.sim_time-1e-6:self.fail('World clock regressed')
        self.sim_time,self.clock_at=stamp,time.monotonic()

    def observe_state(self, msg):
        try:
            data=json.loads(msg.data)
            if self.session_id is not None and data['session_id']!=self.session_id:raise ValueError('Live fleet session changed')
            if self.session_id is None:
                selected={g['id'] for g in self.specification['fleet_goals']}
                if not selected<={m['id'] for m in data['members']}:raise ValueError('Workload IDs are absent from actual fleet')
                if self.specification.get('fleet_id',data['session_id'])!=data['session_id']:raise ValueError('Unexpected live fleet identity')
                self.session_id=data['session_id']
                for name in sorted(selected):
                    self.truth_count[name]=0
                    self.truth_subscriptions.append(self.create_subscription(Odometry,'/'+name+'/odom/ground_truth',
                        lambda m,n=name:self.observe_truth(n,m),qos_profile_sensor_data))
                self.record(kind='session_bound',session_id=self.session_id,map=data['map'],backend=data['backend'])
            self.runtime,self.state_at=data,time.monotonic()
            self.record(kind='fleet_state',state=data)
        except (ValueError,KeyError,TypeError) as exc:self.fail(exc)

    def observe_truth(self, name, msg):
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if (msg.header.frame_id!='map' or not all(map(math.isfinite,[p.x,p.y,p.z,q.x,q.y,q.z,q.w]))
                or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)>.01):
            self.fail('Invalid selected-world independent body pose for '+name);return
        heading=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        previous=self.truth.get(name);pose=[p.x,p.y,heading]
        if previous and stamp<previous['stamp']:self.fail('Independent truth clock regressed');return
        if previous and stamp==previous['stamp']:return
        dt=stamp-previous['stamp'] if previous else None
        speed=math.hypot(p.x-previous['pose'][0],p.y-previous['pose'][1])/dt if dt and dt<=.5 else None
        rate=math.atan2(math.sin(heading-previous['pose'][2]),math.cos(heading-previous['pose'][2]))/dt if dt and dt<=.5 else None
        self.truth[name]=dict(pose=pose,stamp=stamp,at=time.monotonic(),speed=speed,yaw_rate=rate)
        self.truth_count[name]+=1
        if stamp-self.last_trace.get(name,-math.inf)>=.1:
            self.last_trace[name]=stamp;self.record(kind='body_truth',member_id=name,**self.truth[name])

    def send(self, op, **values):
        data=dict(session_id=self.session_id,operator_id=self.operator,op=op,**values)
        self.command.publish(String(data=json.dumps(data,allow_nan=False)));self.record(kind='command',command=data)

    def dispatch(self, name, index):
        goal=self.specification['fleet_goals'][index];tag=self.tag+'-'+str(index)
        self.requests[name]=dict(index=index,source='route:'+tag,settled=None,started=time.monotonic())
        self.send('route',targets=[name],request_id=tag,goals=[{k:goal[k] for k in ('x','y','yaw')}])

    def tick(self):
        now=time.monotonic()
        if self.interrupted:self.fail('Operator interrupted the workload')
        if self.complete:return
        if now-self.started>self.specification.get('timeout_s',360.):self.fail('Measured fleet workload timed out');return
        if self.runtime is None:
            if now-self.started>45.:self.fail('No actual fleet feedback within startup deadline')
            return
        if now-self.state_at>1. or now-self.clock_at>1.:self.fail('Actual fleet feedback or clock lost');return
        if now-self.hb_at>.2:
            self.hb_at=now;self.heartbeat.publish(String(data=json.dumps(dict(session_id=self.session_id,operator_id=self.operator))))
        if self.runtime['operator_id']!=self.operator:
            if self.owned or now-self.started>45.:self.fail('Live fleet operator lease unavailable/lost')
            return
        self.owned=True;members={m['id']:m for m in self.runtime['members']}
        selected={g['id'] for g in self.specification['fleet_goals']}
        if not self.requests:
            if any(not members[n]['navigation_ready'] or self.truth_count[n]<3 for n in selected):return
            if any(members[n]['mode']!='idle' or members[n]['queued'] or members[n]['nav2_pending'] for n in selected):
                self.fail('Measured workload requires idle members with empty queues');return
            for n in sorted(selected):self.dispatch(n,next(i for i,g in enumerate(self.specification['fleet_goals']) if g['id']==n))
        for name,request in list(self.requests.items()):
            if request.get('done'):continue
            observed=[r for r in self.runtime['task_history'] if r['source']==request['source']]
            if not observed:
                if now-request['started']>120.:self.fail('No owned terminal task for '+name)
                continue
            terminal=observed[-1]
            if terminal['state']!='succeeded':self.fail('Owned task '+terminal['state']+': '+terminal['reason']);return
            body=self.truth.get(name);goal=self.specification['fleet_goals'][request['index']]
            fresh=body and now-body['at']<.5 and self.sim_time is not None and -.2<=self.sim_time-body['stamp']<=.5
            clear=not members[name]['nav2_pending'] and not members[name]['cancel_pending']
            if fresh and clear and endpoint_ok(goal,body['pose'],body['speed'],body['yaw_rate']):
                if request['settled'] is None:request['settled']=self.sim_time
                duration=self.sim_time-request['settled']
                if duration>=FLEET_GOAL_CONTRACT['settle_sim_s']:
                    self.measurements.append(dict(member_id=name,index=request['index'],requested=goal,
                        physical_pose=body['pose'],linear_m_s=body['speed'],angular_rad_s=body['yaw_rate'],
                        settled_sim_s=duration,task=terminal['task'],source=request['source'],nav2_terminal=True,success=True))
                    self.record(kind='measured_endpoint',measurement=self.measurements[-1])
                    following=next((i for i,g in enumerate(self.specification['fleet_goals']) if i>request['index'] and g['id']==name),None)
                    if following is None:request['done']=True
                    else:self.dispatch(name,following)
            else:request['settled']=None
            if now-request['started']>135.:self.fail('Physical endpoint did not settle for '+name);return
        if len(self.measurements)==len(self.specification['fleet_goals']):
            self.complete=True;self.reason='All owned actual routes reached independently measured settled endpoints'

    def finish(self, specification_path):
        if self.runtime and self.owned:self.send('stop',targets=list(self.requests))
        deadline=time.monotonic()+5.;cleanup=False
        while rclpy.ok() and time.monotonic()<deadline:
            rclpy.spin_once(self,timeout_sec=.05)
            if self.runtime and time.monotonic()-self.state_at<1. and all(not m['nav2_pending'] and not m['cancel_pending'] for m in self.runtime['members'] if m['id'] in self.requests):
                cleanup=True;break
        self.record(kind='workload_finish',reason=self.reason,owned_cleanup=cleanup);self.trace.close()
        result=dict(schema=FLEET_RESULT_SCHEMA,run_id=os.environ['ROBOT_LAB_EXPERIMENT_ID'],
            ros_domain_id=int(os.environ['ROS_DOMAIN_ID']),fleet_id=self.session_id,
            success=len(self.measurements)==len(self.specification['fleet_goals']) and cleanup,
            reason=self.reason,member_results=sorted(self.measurements,key=lambda m:m['index']),
            goal_contract=FLEET_GOAL_CONTRACT,truth_samples=self.truth_count,owned_cleanup=cleanup,
            contacts=None,minimum_swept_clearance_m=None,wall_duration_s=time.monotonic()-self.started,
            producer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            workload_sha256=hashlib.sha256(Path(specification_path).read_bytes()).hexdigest(),
            trace_sha256=hashlib.sha256((self.directory/'fleet-trace.jsonl').read_bytes()).hexdigest(),
            scope='Actual task terminals and independent body endpoints/settling; full mission/contact qualification separate')
        temporary=self.directory/'fleet-mission-result.new'
        temporary.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');temporary.replace(self.directory/'fleet-mission-result.json')
        return result


def main(args=None):
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--spec',required=True);options=parser.parse_args(args)
    path=Path(options.spec).resolve()
    if path.stat().st_size>64*1024:parser.error('Workload input exceeds 64 KiB')
    specification=validate_workload(json.loads(path.read_text()))
    if specification['schema']!=FLEET_WORKLOAD_SCHEMA:parser.error('Select a real fleet workload')
    output=os.environ.get('ROBOT_LAB_EXPERIMENT_OUTPUT')
    if not output or not os.environ.get('ROBOT_LAB_EXPERIMENT_ID') or not os.environ.get('ROS_DOMAIN_ID'):
        parser.error('Provide isolated runner output, run identity and ROS domain environment')
    directory=Path(output).resolve();directory.mkdir(parents=True,exist_ok=True)
    if directory.stat().st_dev==Path('/').stat().st_dev:parser.error('Fleet output must stay on the mounted SSD')
    rclpy.init(args=[]);node=FleetWorkload(specification,directory)
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,lambda *_a:setattr(node,'interrupted',True))
    try:
        while rclpy.ok() and not node.complete:rclpy.spin_once(node,timeout_sec=.05);node.tick()
    except Exception as exc:node.fail('Workload exception: '+str(exc))
    finally:
        result=node.finish(path);node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    print(json.dumps(result),flush=True)
    return 0 if result['success'] else 1


if __name__=='__main__':raise SystemExit(main())
