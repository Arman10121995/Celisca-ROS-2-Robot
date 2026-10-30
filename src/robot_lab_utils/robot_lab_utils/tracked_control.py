#!/usr/bin/env python3
"""Bounded manual/Nav2 owner for native Gazebo contact-surface tracks."""
import json
import copy
import math
import time
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import Imu, LaserScan, Image, CameraInfo, JointState
from std_msgs.msg import String, Float64
import uuid

FLIPPER_JOINTS=('front_left_flipper_j','rear_left_flipper_j','front_right_flipper_j','rear_right_flipper_j')


class TrackedControl(Node):
    def __init__(self):
        super().__init__('tracked_command_owner')
        self.declare_parameter('navigation',False)
        self.declare_parameter('robot_id','lab_tracked')
        self.declare_parameter('articulated_flippers',False)
        self.robot_id=self.get_parameter('robot_id').value;self.session=uuid.uuid4().hex
        self.articulated=self.get_parameter('articulated_flippers').value
        self.flipper_positions=None;self.flipper_at=-math.inf;self.flipper_clock=None
        self.flipper_requested=[0.]*4;self.flipper_output=[0.]*4
        if self.articulated:
            self.flipper_publishers=[self.create_publisher(Float64,'/tracked/flippers/'+name+'/target',10) for name in FLIPPER_JOINTS]
            self.create_subscription(JointState,'/joint_states',self.flipper_feedback,10)
            self.create_subscription(String,'/tracked/flipper_command',self.flipper_command,10)
        self.command=[0.,0.];self.actual=[0.,0.]
        self.input_at=self.odom_at=self.clock_at=self.imu_at=-math.inf
        self.manual_at=-math.inf;self.clock=None;self.tilt=None;self.owner='idle'
        self.publisher=self.create_publisher(Twist,'/tracked/cmd_vel',10)
        self.state=self.create_publisher(String,'/tracked/state',10)
        self.create_subscription(Twist,'/key_vel',lambda m:self.input(m,False),10)
        self.create_subscription(Twist,'/cmd_vel',lambda m:self.input(m,True),10)
        self.create_subscription(Odometry,'/robot_lab_controller/odom',self.odometry,10)
        self.create_subscription(ClockMessage,'/clock',self.observe_clock,qos_profile_sensor_data)
        self.create_subscription(Imu,'/imu/out',self.imu,qos_profile_sensor_data)
        self.sensor_handles=[]
        for kind,topic,frame in ((LaserScan,'/scan','laser_link'),(Imu,'/imu/out','imu_link'),
                (Image,'/camera/color/image_raw','camera_optical_frame'),(Image,'/camera/depth/image_raw','camera_optical_frame'),
                (CameraInfo,'/camera/color/camera_info','camera_optical_frame')):
            publisher=self.create_publisher(kind,topic,10)
            def relay(msg,publisher=publisher,frame=frame):
                output=copy.copy(msg);output.header=copy.copy(msg.header);output.header.frame_id=frame;publisher.publish(output)
            self.sensor_handles.append(self.create_subscription(kind,'/tracked/raw'+topic,relay,qos_profile_sensor_data))
        self.last=time.monotonic();self.steady=Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.02,self.tick,clock=self.steady)

    def input(self,msg,navigation):
        now=time.monotonic()
        if navigation and (not self.get_parameter('navigation').value or now-self.manual_at<1.):return
        values=[msg.linear.x,msg.angular.z]
        if not all(map(math.isfinite,values)):self.command=[0.,0.];return
        if abs(msg.linear.y)>.001 or abs(msg.linear.z)>.001:self.command=[0.,0.];return
        if not navigation:self.manual_at=now
        self.command=[max(-cap,min(cap,value)) for cap,value in zip((.3,.5),values)]
        self.input_at=now;self.owner='navigation' if navigation else 'manual'

    def odometry(self,msg):
        if msg.header.frame_id=='odom' and msg.child_frame_id=='base_footprint':self.odom_at=time.monotonic()

    def flipper_feedback(self,msg):
        positions={name.split('::')[-1]:value for name,value in zip(msg.name,msg.position)}
        if all(name in positions and math.isfinite(positions[name]) for name in FLIPPER_JOINTS):
            self.flipper_positions=[float(positions[name]) for name in FLIPPER_JOINTS];self.flipper_at=time.monotonic()

    def flipper_command(self,msg):
        try:
            if len(msg.data)>2048:return
            data=json.loads(msg.data)
            if data.get('session_id')!=self.session or data.get('robot_id')!=self.robot_id:return
            if data.get('op')=='hold':self.flipper_requested=list(self.flipper_output);return
            if data.get('op')!='target':return
            if time.monotonic()-self.flipper_at>.6 or time.monotonic()-self.clock_at>.6:return
            values=data['positions']
            if (not isinstance(values,list) or len(values)!=4
                    or any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or abs(v)>.8 for v in values)):return
            self.flipper_requested=list(map(float,values))
        except (ValueError,KeyError,TypeError,AttributeError):return

    def observe_clock(self,msg):
        value=msg.clock.sec+msg.clock.nanosec*1e-9
        if self.clock is not None and value<self.clock:self.command=[0.,0.];self.input_at=-math.inf
        if self.clock is not None and value<self.clock and self.articulated:
            self.flipper_requested=[0.]*4;self.flipper_output=[0.]*4;self.flipper_at=-math.inf;self.flipper_clock=None
        self.clock=value;self.clock_at=time.monotonic()

    def imu(self,msg):
        q=msg.orientation
        if all(map(math.isfinite,[q.x,q.y,q.z,q.w])):
            self.imu_at=time.monotonic()
            self.tilt=math.acos(max(-1.,min(1.,1-2*(q.x*q.x+q.y*q.y))))

    def tick(self):
        now=time.monotonic();dt=min(.05,now-self.last);self.last=now
        ready=now-self.odom_at<.6 and now-self.clock_at<.6 and now-self.imu_at<.6 and self.tilt is not None and self.tilt<.6
        fresh=ready and now-self.input_at<.35
        target=self.command if fresh else [0.,0.]
        for i in range(2):self.actual[i]+=max(-.4*dt,min(.4*dt,target[i]-self.actual[i]))
        if not ready:self.actual=[0.,0.]
        msg=Twist();msg.linear.x,msg.angular.z=self.actual;self.publisher.publish(msg)
        if self.articulated:
            step=0. if self.clock is None or self.flipper_clock is None else max(0.,min(.05,self.clock-self.flipper_clock))*.2
            self.flipper_clock=self.clock
            if now-self.flipper_at<.6 and now-self.clock_at<.6:
                for i in range(4):self.flipper_output[i]+=max(-step,min(step,self.flipper_requested[i]-self.flipper_output[i]))
            for publisher,value in zip(self.flipper_publishers,self.flipper_output):publisher.publish(Float64(data=float(value)))
        self.state.publish(String(data=json.dumps(dict(ready=ready,owner=self.owner if fresh else 'idle',
            robot_id=self.robot_id,session_id=self.session,
            flippers=dict(enabled=self.articulated,joints=FLIPPER_JOINTS,positions=self.flipper_positions,
                targets=self.flipper_output,requested=self.flipper_requested,feedback_fresh=now-self.flipper_at<.6),
            command=self.actual,sim_time=self.clock,tilt_rad=self.tilt,
            odometry='Native track dead reckoning; surface slip is not measured',
            scope='Experimental native tracked plant; physical missions require independent body measurements'))))


def main(args=None):
    from robot_lab_utils.process_lifetime import exit_with_parent
    exit_with_parent();rclpy.init(args=args);node=TrackedControl()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        try:
            if rclpy.ok():node.publisher.publish(Twist())
        except RCLError:pass  # Context may close between ok() and publish.
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
