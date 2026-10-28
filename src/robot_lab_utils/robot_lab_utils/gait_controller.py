#!/usr/bin/env python3
"""Bounded source-geometry gait targets; joint PD stays inside the plant."""
import json
import math
from pathlib import Path
import time
import uuid
import numpy as np
import rclpy
from rclpy.clock import Clock,ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped,Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import JointState
from action_msgs.srv import CancelGoal
from std_msgs.msg import Float64MultiArray,String
from tf2_ros import TransformBroadcaster
from robot_lab_utils.gait_model import load_model


class GaitController(Node):
    def __init__(self):
        super().__init__('source_gait_controller')
        for key,value in dict(description_file='',gait_config='',control_allowed=True,navigation_enabled=False,
                              session_directory='',task='gait').items():self.declare_parameter(key,value)
        self.model=load_model(Path(self.get_parameter('description_file').value).read_text(),self.get_parameter('gait_config').value)
        self.cfg=self.model.config;self.robot_id=self.model.robot_id;self.session_id=uuid.uuid4().hex
        self.enabled=False;self.stand_requested=False;self.reason='Source stance held; walking disarmed'
        self.input_owner='manual'
        self.spine_phase,self.spine_freq=0.,1.
        self.operator='';self.hb_at=self.joints_at=self.pose_at=self.contacts_at=self.clock_at=self.cmd_at=-math.inf
        self.sim_time=self.last_sim=None;self.joints=None;self.pose=None;self.contacts=None
        self.joint_stamp=self.body_stamp=self.contact_stamp=self.engine_stamp=None
        self.reset_barrier=-math.inf
        self.targets=self.model.initial.copy();self.phase=0.;self.command=np.zeros(3);self.filtered=np.zeros(3)
        self.gait=self.cfg['default_gait'];self.max_ik_error=0.;self.planned={};self.samples=0;self.last_state=0.
        self.target_pub=self.create_publisher(Float64MultiArray,'/gait/joint_targets',10)
        self.state_pub=self.create_publisher(String,'/gait/state',10);self.tf=TransformBroadcaster(self)
        self.create_subscription(JointState,'/joint_states',self.observe_joints,10)
        self.create_subscription(Odometry,'/odom/ground_truth',self.observe_body,10)
        self.create_subscription(String,'/gait/contacts',self.observe_contacts,10)
        self.create_subscription(ClockMessage,'/clock',self.observe_clock,qos_profile_sensor_data)
        self.create_subscription(Twist,'/key_vel',self.manual_drive,10)
        self.create_subscription(Twist,'/gait/cmd_vel',self.nav_drive,10)
        self.nav_cancel=self.create_client(CancelGoal,'/gait/navigate_to_pose/_action/cancel_goal')
        self.create_subscription(String,'/gait/command',self.operation,10)
        self.create_subscription(String,'/gait/operator_heartbeat',self.heartbeat,10)
        self.steady=Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.02,self.tick,clock=self.steady)

    def observe_clock(self,msg):
        value=msg.clock.sec+msg.clock.nanosec*1e-9
        if self.sim_time is not None and value<self.sim_time:
            self.stop('Clock reset; re-arm from fresh measured stance');self.joints_at=self.pose_at=self.contacts_at=-math.inf
            self.joint_stamp=self.body_stamp=self.contact_stamp=self.engine_stamp=None
            self.reset_barrier=value
        if self.sim_time is None or value>self.sim_time:self.clock_at=time.monotonic()
        self.sim_time=value

    def observe_joints(self,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if not self.matching_stamp(stamp,self.joint_stamp):return
        if len(msg.name)!=len(msg.position):return
        positions=dict(zip(msg.name,msg.position))
        if not all(name in positions and math.isfinite(positions[name]) for name in self.model.names):return
        self.joints=np.array([positions[name] for name in self.model.names]);self.joints_at=time.monotonic();self.samples+=1;self.joint_stamp=stamp

    def observe_body(self,msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if msg.header.frame_id!='odom' or not self.matching_stamp(stamp,self.body_stamp):return
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        if not all(math.isfinite(getattr(p,k)) for k in 'xyz') or not all(math.isfinite(getattr(q,k)) for k in 'xyzw'):return
        if abs(sum(getattr(q,k)**2 for k in 'xyzw')-1.)>.05:return
        self.pose=msg;self.pose_at=time.monotonic();self.body_stamp=stamp
        # This explicitly truth-based visualization does not qualify an
        # independent legged odometry/SLAM source.
        transform=TransformStamped();transform.header=msg.header
        transform.child_frame_id='base_link'
        transform.transform.translation.x=p.x;transform.transform.translation.y=p.y;transform.transform.translation.z=p.z
        transform.transform.rotation=q;self.tf.sendTransform(transform)

    def observe_contacts(self,msg):
        try:
            if len(msg.data)>16000:return
            data=json.loads(msg.data);forces=data['normal_forces_N']
            stamp=float(data['clock_time']);engine_stamp=float(data['sim_time'])
            if not self.matching_stamp(stamp,self.contact_stamp) or not math.isfinite(engine_stamp):return
            if set(forces)!={leg['id'] for leg in self.model.legs}|{'body'}:return
            if not all(math.isfinite(v) and v>=0 for v in forces.values()):return
            if self.engine_stamp is not None and engine_stamp<self.engine_stamp:
                self.stop('Physical reset observed; source stance held until explicit re-arm')
                self.targets=self.model.initial.copy();self.reset_barrier=stamp
                self.joints_at=self.pose_at=-math.inf
            self.contacts=data;self.contacts_at=time.monotonic();self.contact_stamp=stamp;self.engine_stamp=engine_stamp
        except (ValueError,KeyError,TypeError):return

    def matching_stamp(self,stamp,previous):
        return (self.sim_time is not None and math.isfinite(stamp)
                and -1.<=stamp-self.sim_time<=.2 and stamp>=self.reset_barrier
                and (previous is None or stamp>previous))

    def fresh(self):
        now=time.monotonic()
        return max(now-self.joints_at,now-self.pose_at,now-self.contacts_at,now-self.clock_at)<.6

    def heartbeat(self,msg):
        try:
            if len(msg.data)>4096:return
            data=json.loads(msg.data);operator=data['operator_id']
            if data['session_id']!=self.session_id or not isinstance(operator,str) or not 1<=len(operator)<=80:return
            if self.operator and operator!=self.operator and time.monotonic()-self.hb_at<.8:return
            if operator!=self.operator:self.stop('Operator changed; input neutral')
            self.operator,self.hb_at=operator,time.monotonic()
        except (ValueError,KeyError,TypeError):return

    def stop(self,reason):
        was_active=self.enabled or self.input_owner=='navigation'
        self.enabled=False;self.command[:]=0.;self.filtered[:]=0.;self.cmd_at=-math.inf
        self.stand_requested=False
        self.reason=reason
        self.input_owner='manual'
        if (was_active or reason.startswith('Operator')) and self.get_parameter('navigation_enabled').value and self.nav_cancel.service_is_ready():
            self.nav_cancel.call_async(CancelGoal.Request())
        # Hold the last valid motor targets: never teleport feet to home or
        # advance another gait phase on Stop/loss. Physics remains free.

    def drive(self,msg):
        if not self.enabled:return
        values=np.array([msg.linear.x,msg.linear.y,msg.angular.z])
        if not np.isfinite(values).all() or abs(msg.linear.z)+abs(msg.angular.x)+abs(msg.angular.y)>1e-9:
            self.stop('Invalid gait velocity');return
        self.command=np.clip(values,[-self.cfg['max_speed'],-self.cfg['max_speed'],-self.cfg['max_yaw']],
                             [self.cfg['max_speed'],self.cfg['max_speed'],self.cfg['max_yaw']])
        self.cmd_at=time.monotonic()

    def manual_drive(self,msg):
        if self.input_owner=='navigation':
            if all(math.isfinite(v) and abs(v)<1e-9 for v in (msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z)):
                return  # Neutral GUI input never steals an autonomous lease; explicit Stop does.
            self.stop('Manual input canceled gait navigation; re-arm before walking')
        self.drive(msg)

    def nav_drive(self,msg):
        if self.input_owner=='navigation':self.drive(msg)

    def operation(self,msg):
        try:
            if len(msg.data)>4096:raise ValueError('Command too long')
            data=json.loads(msg.data)
            if data.get('session_id')!=self.session_id:return
            op=data['op']
            if op=='stop':
                reason='Operator Stop; current joint targets held' if self.enabled or self.stand_requested else self.reason
                self.stop(reason);return
            if not self.get_parameter('control_allowed').value:raise ValueError('Display is stance-only; choose Gait mode')
            if data.get('operator_id')!=self.operator or time.monotonic()-self.hb_at>.8:raise ValueError('Live operator lease required')
            if not self.fresh():raise ValueError('Fresh actual joints, body, contacts and clock required')
            if op=='arm':
                if self.contacts['normal_forces_N']['body']>2.:raise ValueError('Body is supported by an obstacle/floor; recover the stance first')
                supports=sum(self.contacts['normal_forces_N'][leg['id']]>.2 for leg in self.model.legs)
                if supports<len(self.model.legs)//2:raise ValueError('Insufficient measured foot support')
                self.enabled=True;self.command[:]=0.;self.filtered[:]=0.;self.cmd_at=-math.inf
                self.input_owner='navigation' if self.get_parameter('navigation_enabled').value else 'manual'
                self.reason='Walking armed; no travel until Drive input'
            elif op=='gait':
                allowed={'serpenoid'} if self.cfg.get('kind')=='serpentine' else {'ripple','wave',self.cfg['default_gait']}
                if data['gait'] not in allowed:raise ValueError('This source has no matching schedule')
                phase,spine_phase,spine_freq=[float(data.get(key,default)) for key,default in
                    [('phase',self.phase),('spine_phase',self.spine_phase),('spine_freq',self.spine_freq)]]
                if not all(math.isfinite(v) for v in (phase,spine_phase,spine_freq)) or not 0<=phase<=1 or not 0<=spine_phase<=1 or not .1<=spine_freq<=3:
                    raise ValueError('Cycle/spine phase must be 0–1; spine frequency multiplier 0.1–3')
                self.stop('Gait changed; re-arm explicitly');self.gait=data['gait']
                self.phase,self.spine_phase,self.spine_freq=phase,spine_phase,spine_freq
            elif op=='stand':
                self.stop('Source stance requested; bounded joint motion')
                self.stand_requested=True
            else:raise ValueError('Unknown gait operation')
        except (ValueError,KeyError,TypeError) as exc:self.reason='Rejected: '+str(exc)

    def tick(self):
        now=time.monotonic();ready=self.fresh()
        dt=0. if self.last_sim is None or self.sim_time is None else max(0.,min(.04,self.sim_time-self.last_sim))
        self.last_sim=self.sim_time
        if self.enabled and (not ready or now-self.hb_at>.8):self.stop('Observation/clock/operator loss; gait halted')
        tilt=None
        if self.pose:
            q=self.pose.pose.pose.orientation
            roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y))
            pitch=math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x))));tilt=max(abs(roll),abs(pitch))
            if self.enabled and (tilt>.65 or self.contacts['normal_forces_N']['body']>3.):self.stop('Measured tilt/body contact interrupted gait')
        if self.stand_requested and ready:
            step=self.model.velocity*dt
            self.targets+=np.clip(self.model.initial-self.targets,-step,step)
            if np.max(np.abs(self.model.initial-self.targets))<1e-6:self.stand_requested=False
        if self.enabled:
            wanted=self.command if now-self.cmd_at<.35 else np.zeros(3)
            self.filtered+=np.clip(wanted-self.filtered,-np.array([.06,.06,.25])*dt,np.array([.06,.06,.25])*dt)
            if np.linalg.norm(self.filtered)>.001 and dt>0:
                self.phase=(self.phase+dt/self.cfg['cycle_s']*(self.spine_freq if self.cfg.get('kind')=='serpentine' else 1.))%1.
                candidate,error,self.planned=self.model.targets(self.phase,self.filtered,self.targets,self.gait,
                    spine_phase=self.spine_phase,spine_freq=self.spine_freq)
                self.max_ik_error=error
                if error is not None and error>.003:self.stop('Foot IK error exceeds 3 mm; gait halted')
                else:
                    step=self.model.velocity*dt
                    self.targets=np.clip(self.targets+np.clip(candidate-self.targets,-step,step),self.model.lower,self.model.upper)
        message=Float64MultiArray();message.data=self.targets.tolist();self.target_pub.publish(message)
        if now-self.last_state>.1:
            self.last_state=now;p=self.pose.pose.pose.position if self.pose else None
            state=dict(schema_version=1,session_id=self.session_id,robot_id=self.robot_id,enabled=self.enabled,ready=ready,
                       control_allowed=self.get_parameter('control_allowed').value,reason=self.reason,gait=self.gait,
                       input_owner=self.input_owner,
                       task=self.get_parameter('task').value,directory=self.get_parameter('session_directory').value,
                       operator_id=self.operator,operator_fresh=now-self.hb_at<.8,physical_legs=self.cfg.get('physical_leg_count',len(self.model.legs)),
                       motion_kind=self.cfg.get('kind','multi_leg'),available_schedules=['serpenoid'] if self.cfg.get('kind')=='serpentine' else list(dict.fromkeys([self.cfg['default_gait'],'ripple','wave'])),
                       actuated_joints=len(self.model.names),joint_samples=self.samples,sim_time=self.sim_time,
                       drive_forward_yaw_rad=float(self.cfg['forward_yaw']),
                       position=dict(x=p.x,y=p.y,z=p.z) if p else None,tilt_rad=tilt,
                       actual_contacts=self.contacts,planned_stance=self.planned,max_ik_error_m=self.max_ik_error,
                       commanded_body_velocity=self.filtered.tolist(),joint_targets=self.targets.tolist(),
                       phase=self.phase,spine_phase=self.spine_phase,spine_freq=self.spine_freq,
                       observer='Simulator body pose for visualization/tilt only; legged odometry uses independent IMU, measured contacts and joints',
                       scope='Experimental authored gait and bounded in-step PD on exact configured geometry; physical missions pending')
            self.state_pub.publish(String(data=json.dumps(state,allow_nan=False)))


def main(args=None):
    from robot_lab_utils.process_lifetime import exit_with_parent
    exit_with_parent();rclpy.init(args=args);node=GaitController()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
