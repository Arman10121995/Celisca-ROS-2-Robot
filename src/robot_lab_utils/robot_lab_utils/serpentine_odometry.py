#!/usr/bin/env python3
"""Rolling-contact snake odometry from encoders, source FK and native IMU.

Independent body truth is deliberately absent. Contact inconsistency, large
tilt, stale inputs and resets suspend output rather than inventing movement.
"""
import json
import math
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu,JointState
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from robot_lab_utils.serpentine_model import SerpentineModel,encoder_velocity


class SerpentineOdometry(Node):
    def __init__(self):
        super().__init__('serpentine_encoder_odometry')
        for key,value in dict(description_file='',gait_config='',initial_drive_yaw=0.).items():self.declare_parameter(key,value)
        self.model=SerpentineModel(Path(self.get_parameter('description_file').value).read_text(),self.get_parameter('gait_config').value)
        self.reference=float(self.get_parameter('initial_drive_yaw').value)
        self.imu=self.contacts=self.previous=None;self.imu_at=self.contacts_at=-math.inf
        self.position=np.zeros(2);self.last_stamp=self.engine_stamp=None;self.invalidated=False;self.samples=0
        self.covariance=.0001
        self.pub=self.create_publisher(Odometry,'/gait/odometry',10)
        self.state=self.create_publisher(String,'/gait/odometry_state',10);self.tf=TransformBroadcaster(self)
        self.create_subscription(Imu,'/imu/out',self.observe_imu,qos_profile_sensor_data)
        self.create_subscription(String,'/gait/contacts',self.observe_contacts,10)
        self.create_subscription(JointState,'/joint_states',self.observe_joints,10)

    def observe_imu(self,msg):
        q=msg.orientation;v=[q.x,q.y,q.z,q.w,msg.angular_velocity.z]
        if np.isfinite(v).all() and abs(sum(x*x for x in v[:4])-1)<.02:
            self.imu=msg;self.imu_at=time.monotonic()

    def observe_contacts(self,msg):
        try:
            data=json.loads(msg.data);stamp=float(data['sim_time'])
            if self.engine_stamp is not None and stamp<self.engine_stamp:self.invalidated=True
            self.engine_stamp=stamp
            if all(math.isfinite(v) and v>=0 for v in data['normal_forces_N'].values()):
                self.contacts=data;self.contacts_at=time.monotonic()
        except (ValueError,TypeError,KeyError):return

    def observe_joints(self,msg):
        if self.invalidated or self.imu is None or self.contacts is None:return
        now=time.monotonic();stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if max(now-self.imu_at,now-self.contacts_at)>.5:return
        imu_stamp=self.imu.header.stamp.sec+self.imu.header.stamp.nanosec*1e-9
        if abs(imu_stamp-stamp)>.15 or abs(self.contacts['clock_time']-stamp)>.15:return
        if self.last_stamp is not None and stamp<=self.last_stamp:
            if stamp<self.last_stamp:self.invalidated=True
            return
        if len(msg.name)!=len(msg.position) or len(msg.name)!=len(msg.velocity):return
        positions=dict(zip(msg.name,msg.position));rates=dict(zip(msg.name,msg.velocity))
        required=self.model.names+['wheel_'+str(i)+'_'+side+'_joint' for i in range(self.model.count) for side in ('left','right')]
        if any(name not in positions or name not in rates or not math.isfinite(positions[name]) or not math.isfinite(rates[name]) for name in required):return
        q=self.imu.orientation
        tilt=math.acos(max(-1.,min(1.,1-2*(q.x*q.x+q.y*q.y))))
        if tilt>.3 or self.contacts['normal_forces_N'].get('body',0.)>1.:return
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))-self.reference
        yaw=math.atan2(math.sin(yaw),math.cos(yaw));angular=self.imu.angular_velocity.z
        current=self.model.wheels(positions)
        supports={name for name in current if self.contacts['normal_forces_N'].get(name,0.)>.02}
        previous,self.previous=self.previous,(current,supports)
        old,self.last_stamp=self.last_stamp,stamp
        if previous is None:return
        dt=stamp-old
        try:
            velocity,scatter=encoder_velocity(previous[0],current,{name:rates[name+'_joint'] for name in current},
                supports&previous[1],dt,angular,float(self.model.config['wheel_radius']))
        except ValueError:return
        c,s=math.cos(yaw),math.sin(yaw)
        self.position+=np.array([[c,-s],[s,c]])@velocity*dt
        self.covariance+=(.00002+scatter*scatter*.01)*dt
        output=Odometry();output.header.stamp=msg.header.stamp;output.header.frame_id='gait/odom';output.child_frame_id='gait/base_footprint'
        output.pose.pose.position.x,output.pose.pose.position.y=map(float,self.position)
        output.pose.pose.orientation.z=math.sin(yaw/2);output.pose.pose.orientation.w=math.cos(yaw/2)
        output.twist.twist.linear.x,output.twist.twist.linear.y=map(float,velocity);output.twist.twist.angular.z=float(angular)
        output.pose.covariance[0]=output.pose.covariance[7]=self.covariance;output.pose.covariance[35]=.001
        self.pub.publish(output)
        transform=TransformStamped();transform.header=output.header;transform.child_frame_id='gait/base_footprint'
        transform.transform.translation.x,transform.transform.translation.y=map(float,self.position)
        transform.transform.rotation=output.pose.pose.orientation
        trunk=TransformStamped();trunk.header.stamp=msg.header.stamp;trunk.header.frame_id='gait/base_footprint';trunk.child_frame_id='gait/base_link'
        trunk.transform.translation.z=float(self.model.config['standing_height']);trunk.transform.rotation.w=1.
        self.tf.sendTransform([transform,trunk]);self.samples+=1
        if self.samples%10==0:self.state.publish(String(data=json.dumps(dict(samples=self.samples,supports=sorted(supports),
            position=self.position.tolist(),velocity_scatter_m_s=scatter,
            source='Actual unpowered wheel encoders + source spine FK + measured rolling contacts + native IMU; no body truth'),allow_nan=False)))


def main(args=None):
    from robot_lab_utils.process_lifetime import exit_with_parent
    exit_with_parent();rclpy.init(args=args);node=SerpentineOdometry()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
