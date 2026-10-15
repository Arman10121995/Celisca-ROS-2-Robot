#!/usr/bin/env python3
"""Bounded physical ROV thrusters/depth hold; explicit truth-observer prototype.

The controller publishes native thruster forces. Buoyancy, drag, added mass
and currents act in Gazebo; this module never moves body/joint poses. The
current observer uses actual simulator odometry and must not qualify independent
underwater localization, DVL, sonar or SLAM.
"""
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
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float64,String
from tf2_ros import TransformBroadcaster
from robot_lab_utils.marine_model import MarineModel,rotation


def angle(value):return math.atan2(math.sin(value),math.cos(value))


def rpy(q):
    return (math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y)),
            math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x)))),
            math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))


class MarineController(Node):
    def __init__(self):
        super().__init__('marine_controller')
        for key,value in dict(description_file='',surface_z=0.,maximum_depth=5.4,density=1025.,robot_id='lab_rov',control_allowed=True).items():
            self.declare_parameter(key,value)
        self.model=MarineModel(Path(self.get_parameter('description_file').value).read_text(),self.get_parameter('density').value)
        self.surface=float(self.get_parameter('surface_z').value)
        self.maximum_depth=float(self.get_parameter('maximum_depth').value)
        self.robot_id=str(self.get_parameter('robot_id').value)
        self.session_id=uuid.uuid4().hex
        self.enabled=False
        self.reason='Disarmed; native water forces remain physical'
        self.operator=''
        self.hb_at=self.pose_at=self.cmd_at=self.clock_at=-math.inf
        self.clock_value=None
        self.pose=None
        self.velocity=None
        self.rates=None
        self.orientation=None
        self.target_depth=None
        self.target_heading=None
        self.command=(0.,0.,0.,0.)
        self.forces=np.zeros(8)
        self.last_sim=None
        self.last_state=0.
        self.count=0
        self.pub=[self.create_publisher(Float64,'/marine/thrusters/'+joint+'/force',10) for joint in self.model.joints]
        self.state_pub=self.create_publisher(String,'/marine/state',10)
        self.pressure_pub=self.create_publisher(FluidPressure,'/marine/pressure_ideal',10)
        self.tf=TransformBroadcaster(self)
        self.create_subscription(Odometry,'/marine/odometry',self._pose,10)
        self.create_subscription(Twist,'/marine/key_vel',self._drive,10)
        self.create_subscription(String,'/marine/command',self._command,10)
        self.create_subscription(String,'/marine/operator_heartbeat',self._heartbeat,10)
        self.create_subscription(ClockMessage,'/clock',self._observe_clock,qos_profile_sensor_data)
        self.steady=Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.02,self._tick,clock=self.steady)

    def _observe_clock(self,msg):
        value=msg.clock.sec+msg.clock.nanosec*1e-9
        if self.clock_value is not None and value<self.clock_value:
            self.stop('World clock rewound; reinitialize the marine session')
            self.pose_at=-math.inf
        self.clock_value,self.clock_at=value,time.monotonic()

    def _pose(self,msg):
        values=[getattr(msg.pose.pose.position,k) for k in 'xyz']+[getattr(msg.pose.pose.orientation,k) for k in 'xyzw']
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if (msg.header.frame_id!='map' or not all(math.isfinite(v) for v in values)
                or abs(sum(getattr(msg.pose.pose.orientation,k)**2 for k in 'xyzw')-1)>.05):
            self.pose_at=-math.inf
            self.stop('Invalid measured body pose')
            return
        if self.clock_value is None or not -1.<=stamp-self.clock_value<=.2:return
        orientation=rpy(msg.pose.pose.orientation)
        if self.pose is not None:
            previous=self.pose
            dt=stamp-(previous.header.stamp.sec+previous.header.stamp.nanosec*1e-9)
            if dt<=0:return
            self.velocity=np.array([(getattr(msg.pose.pose.position,k)-getattr(previous.pose.pose.position,k))/dt for k in 'xyz'])
            self.rates=np.array([angle(a-b)/dt for a,b in zip(orientation,self.orientation)])
        self.pose,self.orientation,self.pose_at=msg,orientation,time.monotonic()
        self.count+=1
        transform=TransformStamped()
        transform.header=msg.header
        transform.child_frame_id='base_link'
        for key in 'xyz':setattr(transform.transform.translation,key,getattr(msg.pose.pose.position,key))
        transform.transform.rotation=msg.pose.pose.orientation
        self.tf.sendTransform(transform)
        pressure=FluidPressure()
        pressure.header.stamp=msg.header.stamp
        pressure.header.frame_id='base_link'
        pressure.fluid_pressure=101325.+self.model.density*9.81*max(0.,self.surface-msg.pose.pose.position.z)
        pressure.variance=0.
        self.pressure_pub.publish(pressure)

    def _fresh(self):
        now=time.monotonic()
        return now-self.pose_at<.5 and now-self.clock_at<.5 and self.velocity is not None

    def _heartbeat(self,msg):
        try:
            data=json.loads(msg.data)
            if data['session_id']!=self.session_id:return
            operator=data['operator_id']
            if not isinstance(operator,str) or not 1<=len(operator)<=80:return
            if self.operator and operator!=self.operator and time.monotonic()-self.hb_at<.8:return
            if operator!=self.operator:self.stop('Operator changed; neutral ownership')
            self.operator,self.hb_at=operator,time.monotonic()
        except (TypeError,ValueError,KeyError):return

    def _drive(self,msg):
        if not self.enabled:return
        values=(msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.z,msg.angular.x,msg.angular.y)
        if not all(math.isfinite(v) for v in values) or abs(msg.angular.x)+abs(msg.angular.y)>1e-9:
            self.stop('Invalid marine velocity command');return
        self.command=tuple(max(-limit,min(limit,value)) for value,limit in zip(values[:4],(.35,.25,.2,.35)))
        self.cmd_at=time.monotonic()

    def _command(self,msg):
        try:
            if len(msg.data)>4096:raise ValueError('Marine command too long')
            data=json.loads(msg.data)
            if data.get('session_id')!=self.session_id:return
            op=data['op']
            if op=='stop':self.stop('Operator Stop; thruster forces zero');return
            if not self.get_parameter('control_allowed').value:raise ValueError('Display is observer-only; select Marine mode to actuate')
            if data.get('operator_id')!=self.operator or time.monotonic()-self.hb_at>.8:
                raise ValueError('Connect a live marine operator first')
            if not self._fresh():raise ValueError('Fresh actual body feedback and clock required')
            depth=self.surface-self.pose.pose.pose.position.z
            if op=='arm':
                self.enabled=True;self.command=(0.,0.,0.,0.);self.cmd_at=-math.inf
                self.target_depth=max(.35,min(self.maximum_depth,depth));self.target_heading=self.orientation[2]
                self.reason='Depth/heading hold armed; planar commands neutral'
            elif op in ('hold','depth_up','depth_down','set_depth','surface'):
                if not self.enabled:raise ValueError('Arm marine control first')
                if op=='hold':
                    self.command=(0.,0.,0.,0.);self.cmd_at=-math.inf
                    self.target_depth=depth;self.target_heading=self.orientation[2]
                elif op=='set_depth':
                    new_depth=float(data['depth'])
                    if not math.isfinite(new_depth):raise ValueError('Depth must be finite')
                    self.target_depth=new_depth
                elif op=='surface':self.target_depth=.35
                else:self.target_depth+=(-.2 if op=='depth_up' else .2)
                if not math.isfinite(self.target_depth):raise ValueError('Depth must be finite')
                self.target_depth=max(.35,min(self.maximum_depth,self.target_depth))
                self.reason='Physical depth/heading controller active'
            else:raise ValueError('Unknown marine operation: '+str(op))
        except (ValueError,TypeError,KeyError) as exc:
            self.reason='Rejected: '+str(exc)
            self.get_logger().warning(self.reason)

    def stop(self,reason):
        self.enabled=False;self.command=(0.,0.,0.,0.);self.cmd_at=-math.inf
        self.target_depth=self.target_heading=None
        self.reason=reason
        self.forces=np.zeros(8)
        for pub in self.pub:pub.publish(Float64(data=0.))

    def _tick(self):
        now=time.monotonic()
        fresh=self._fresh()
        if self.enabled and (not fresh or now-self.hb_at>.8):
            self.stop('Observer/clock/operator loss; thruster forces zero')
        dt=0. if self.last_sim is None or self.clock_value is None else max(0.,min(.05,self.clock_value-self.last_sim))
        self.last_sim=self.clock_value
        scale=1.
        if self.enabled:
            roll,pitch,heading=self.orientation
            depth=self.surface-self.pose.pose.pose.position.z
            if max(abs(roll),abs(pitch))>.8 or not -.1<depth<self.maximum_depth+.35:
                self.stop('Attitude/depth envelope exceeded')
            else:
                vx,vy,vz,wz=self.command if now-self.cmd_at<.35 else (0.,0.,0.,0.)
                self.target_depth=max(.35,min(self.maximum_depth,self.target_depth-vz*dt))
                self.target_heading=angle(self.target_heading+wz*dt)
                desired_z=max(-.2,min(.2,.8*(self.surface-self.target_depth-self.pose.pose.pose.position.z)))
                desired_world=rotation((0.,0.,heading))@np.array([vx,vy,0.])
                desired_world[2]=desired_z
                force=np.array([14.,18.,28.])*(desired_world-self.velocity)
                force[2]+=self.model.gravity_trim
                desired_yaw=max(-.35,min(.35,wz+.8*angle(self.target_heading-heading)))
                torque=np.array([-3.*roll-.8*self.rates[0],-3.*pitch-.8*self.rates[1],1.2*(desired_yaw-self.rates[2])])
                self.forces,scale=self.model.allocate(np.r_[rotation(self.orientation).T@force,torque])
        for pub,value in zip(self.pub,self.forces):pub.publish(Float64(data=float(value)))
        if now-self.last_state>.1:
            self.last_state=now
            position=self.pose.pose.pose.position if self.pose else None
            state=dict(schema_version=1,session_id=self.session_id,robot_id=self.robot_id,enabled=self.enabled,ready=fresh,
                       control_allowed=self.get_parameter('control_allowed').value,
                       reason=self.reason,operator_id=self.operator,operator_fresh=now-self.hb_at<.8,
                       depth=self.surface-position.z if position else None,target_depth=self.target_depth,
                       position=dict(x=position.x,y=position.y,z=position.z) if position else None,
                       rpy=list(self.orientation) if self.orientation else None,velocity_world=self.velocity.tolist() if self.velocity is not None else None,
                       commanded_thruster_forces_N=self.forces.tolist(),force_allocation_scale=scale,
                       mass_kg=self.model.mass,displaced_volume_m3=self.model.displacement,gravity_trim_N=self.model.gravity_trim,
                       body_samples=self.count,sim_time=self.clock_value,
                       observer='Actual simulator pose finite difference; ideal pressure is emulated from that pose. Independent estimation/SLAM unimplemented.',
                       scope='Authored laboratory ROV + native Gazebo marine forces; hardware calibration and mission qualification pending')
            self.state_pub.publish(String(data=json.dumps(state,allow_nan=False)))


def main(args=None):
    from robot_lab_utils.process_lifetime import exit_with_parent
    exit_with_parent();rclpy.init(args=args)
    node=MarineController()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        if rclpy.ok():node.stop('Marine controller shutdown')
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
