#!/usr/bin/env python3
"""Contact-kinematic multi-leg odometry; no simulator-pose subscription.

Actual source-chain foot FK and native IMU orientation estimate body motion
from measured supporting feet. Slip and insufficient support suspend output;
this is experimental dead reckoning, not a terrain or get-up controller.
"""
import json
import math
from pathlib import Path
import time

import numpy as np
import PyKDL as kdl
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation
from robot_lab_utils.gait_model import GaitModel


def stance_velocity(previous, current, dt, angular, supports):
    if not 0 < dt <= .15 or len(supports) < 2:
        raise ValueError('Fresh multi-foot support required')
    samples = [-(current[name]-previous[name])/dt - np.cross(angular,current[name]) for name in supports]
    if not np.isfinite(samples).all():
        raise ValueError('Non-finite contact kinematics')
    velocity = np.median(samples,axis=0)
    scatter = float(np.max(np.linalg.norm(np.asarray(samples)-velocity,axis=1)))
    if scatter > .15 or np.linalg.norm(velocity) > 1.:
        raise ValueError('Foot-slip/inconsistent support velocity; odometry suspended')
    return velocity,scatter


class GaitOdometry(Node):
    def __init__(self):
        super().__init__('gait_contact_odometry')
        for key,value in dict(description_file='',gait_config='',initial_drive_yaw=0.).items():
            self.declare_parameter(key,value)
        self.model=GaitModel(Path(self.get_parameter('description_file').value).read_text(),self.get_parameter('gait_config').value)
        self.chains={}
        for leg in self.model.legs:
            chain=self.model.tree.getChain('base_link',leg['foot'])
            names=[chain.getSegment(i).getJoint().getName() for i in range(chain.getNrOfSegments())
                   if chain.getSegment(i).getJoint().getType()!=kdl.Joint.Fixed]
            self.chains[leg['id']]=(names,kdl.ChainFkSolverPos_recursive(chain))
        self.reference=Rotation.from_euler('z',float(self.get_parameter('initial_drive_yaw').value))
        self.forward=Rotation.from_euler('z',float(self.model.config['forward_yaw']))
        self.imu=self.contacts=None
        self.imu_at=self.contact_at=-math.inf
        self.previous=None
        self.position=np.zeros(2)
        self.covariance=np.eye(2)*.0001
        self.last_stamp=None
        self.reset_seen=False
        self.engine_stamp=None
        self.samples=0
        self.pub=self.create_publisher(Odometry,'/gait/odometry',10)
        self.state=self.create_publisher(String,'/gait/odometry_state',10)
        self.tf=TransformBroadcaster(self)
        self.create_subscription(Imu,'/imu/out',self.observe_imu,qos_profile_sensor_data)
        self.create_subscription(String,'/gait/contacts',self.observe_contacts,10)
        self.create_subscription(JointState,'/joint_states',self.observe_joints,10)

    def observe_imu(self,msg):
        q=msg.orientation
        if np.isfinite([q.x,q.y,q.z,q.w]).all() and abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)<.05:
            self.imu=msg;self.imu_at=time.monotonic()

    def observe_contacts(self,msg):
        try:
            data=json.loads(msg.data)
            forces=data['normal_forces_N']
            engine=float(data['sim_time'])
            if self.engine_stamp is not None and engine<self.engine_stamp:
                self.reset_seen=True
                self.state.publish(String(data=json.dumps(dict(reason='Physical reset requires a new odometry session',reset_required=True))))
            self.engine_stamp=engine
            if all(math.isfinite(v) and v>=0 for v in forces.values()):
                self.contacts=data;self.contact_at=time.monotonic()
        except (ValueError,KeyError,TypeError):pass

    def observe_joints(self,msg):
        if not self.imu or not self.contacts or self.reset_seen:return
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        imu_stamp=self.imu.header.stamp.sec+self.imu.header.stamp.nanosec*1e-9
        if max(time.monotonic()-self.imu_at,time.monotonic()-self.contact_at)>.5:return
        if self.contacts['normal_forces_N'].get('body',0.)>2.:return
        if abs(imu_stamp-stamp)>.15 or abs(self.contacts['clock_time']-stamp)>.15:return
        if self.last_stamp is not None and stamp<=self.last_stamp:
            if stamp<self.last_stamp:self.reset_seen=True
            return
        joints=dict(zip(msg.name,msg.position))
        current={}
        for name,(names,solver) in self.chains.items():
            if any(key not in joints or not math.isfinite(joints[key]) for key in names):return
            frame=kdl.Frame();solver.JntToCart(self.model.array([joints[key] for key in names]),frame)
            current[name]=np.array([frame.p.x(),frame.p.y(),frame.p.z()])
        q=self.imu.orientation
        body=self.reference.inv()*Rotation.from_quat([q.x,q.y,q.z,q.w])
        supports=[name for name in current if self.contacts['normal_forces_N'].get(name,0.)>.2]
        previous,self.previous=self.previous,(current,body)
        old_stamp,self.last_stamp=self.last_stamp,stamp
        if previous is None or len(supports)<max(2,len(current)//3):return
        dt=stamp-old_stamp
        if dt>.15:
            self.covariance+=np.eye(2)*(.1*dt)**2
            if dt>1.:self.reset_seen=True
            return
        angular=(previous[1].inv()*body).as_rotvec()/dt
        try:velocity,scatter=stance_velocity(previous[0],current,dt,angular,supports)
        except ValueError:return
        local=body.apply(velocity)
        self.position+=local[:2]*dt
        self.covariance+=np.eye(2)*((.005+scatter)**2*dt**2+.00002*dt)
        drive=body*self.forward
        yaw=drive.as_euler('xyz')[2]
        heading=Rotation.from_euler('z',yaw)
        height=-float(np.median([body.apply(current[n])[2] for n in supports]))
        output=Odometry();output.header.stamp=msg.header.stamp
        output.header.frame_id='gait/odom';output.child_frame_id='gait/base_footprint'
        output.pose.pose.position.x,output.pose.pose.position.y=map(float,self.position)
        quat=heading.as_quat()
        for i,key in enumerate('xyzw'):setattr(output.pose.pose.orientation,key,float(quat[i]))
        output.twist.twist.linear.x,output.twist.twist.linear.y=map(float,heading.inv().apply(local)[:2])
        output.twist.twist.angular.z=float(angular[2])
        output.pose.covariance[0]=float(self.covariance[0,0]);output.pose.covariance[7]=float(self.covariance[1,1]);output.pose.covariance[35]=.001
        self.pub.publish(output)
        base=TransformStamped();base.header=output.header;base.child_frame_id='gait/base_footprint'
        base.transform.translation.x,base.transform.translation.y=map(float,self.position)
        base.transform.rotation=output.pose.pose.orientation
        trunk=TransformStamped();trunk.header.stamp=msg.header.stamp;trunk.header.frame_id='gait/base_footprint';trunk.child_frame_id='gait/base_link'
        trunk.transform.translation.z=max(.02,height)
        quat=(heading.inv()*body).as_quat()
        for i,key in enumerate('xyzw'):setattr(trunk.transform.rotation,key,float(quat[i]))
        self.tf.sendTransform([base,trunk]);self.samples+=1
        if self.samples%10==0:self.state.publish(String(data=json.dumps(dict(samples=self.samples,supports=supports,
            velocity_scatter_m_s=scatter,position=self.position.tolist(),source='Measured foot contacts + complete source-chain FK + native IMU; no simulator pose input',
            scope='Experimental contact odometry; slip/terrain/reset and saved-map missions unqualified'))))


def main(args=None):
    from robot_lab_utils.process_lifetime import exit_with_parent
    exit_with_parent();rclpy.init(args=args);node=GaitOdometry()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
