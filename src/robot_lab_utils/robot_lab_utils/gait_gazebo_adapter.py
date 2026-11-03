#!/usr/bin/env python3
"""Restore source encoders around force PIDs and model static terrain ranges."""
import argparse
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET
import numpy as np
import PyKDL as kdl
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock,ClockType
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock as ClockMessage
from std_msgs.msg import Float64,Float64MultiArray,String
from robot_lab_utils.gait_model import GaitModel
from robot_lab_utils.gait_terrain import sample_probes, StaticTerrainRay


class Adapter(Node):
    def __init__(self,config):
        super().__init__('source_gait_native_adapter');self.config=config
        source=Path(config['description']).read_text();self.model=GaitModel(source,config['gait_model'])
        self.static_ray=StaticTerrainRay(config['source_world']);self.clock=None;self.clock_at=-math.inf
        self.targets=dict(config['offsets']);self.received=-math.inf
        self.joints={};self.pose=None;self.pose_at=-math.inf;self.joints_at=-math.inf
        self.state_stamp=None
        self.pub={name:self.create_publisher(Float64,topic,10) for name,topic in config['topics'].items()}
        self.states=self.create_publisher(JointState,'/joint_states',10)
        self.contacts=self.create_publisher(String,'/gait/contacts',10);self.terrain=self.create_publisher(String,'/gait/terrain',10)
        self.create_subscription(JointState,'/gait/native_joint_states',self.observe_joints,10)
        self.create_subscription(String,'/gait/native_contact_frame',self.observe_contacts,10)
        self.create_subscription(Odometry,'/odom/ground_truth',self.observe_pose,10)
        self.create_subscription(ClockMessage,'/clock',self.observe_clock,10)
        self.create_subscription(Float64MultiArray,'/gait/joint_targets',self.command,10)
        self.chains={}
        for name in {probe['root'] for probe in config['probes'].values()}|{probe['foot'] for probe in config['probes'].values()}:
            chain=self.model.tree.getChain('base_link',name)
            names=[chain.getSegment(i).getJoint().getName() for i in range(chain.getNrOfSegments()) if chain.getSegment(i).getJoint().getType()!=kdl.Joint.Fixed]
            self.chains[name]=(chain,names,kdl.ChainFkSolverPos_recursive(chain))
        root=ET.fromstring(source);self.radii={}
        for probe in config['probes'].values():
            shape=root.find("link[@name='%s']/collision/geometry/sphere"%probe['foot'])
            if shape is None:raise ValueError('Native terrain probe needs its declared source spherical sole')
            self.radii[probe['foot']]=float(shape.get('radius'))
        self.create_timer(.02,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.create_timer(.1,self.sense,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def observe_clock(self,msg):
        value=msg.clock.sec+msg.clock.nanosec*1e-9
        if self.clock is not None and value<self.clock:self.targets=dict(self.config['offsets']);self.received=-math.inf
        self.clock=value;self.clock_at=time.monotonic()

    def observe_pose(self,msg):
        self.pose=msg.pose.pose;self.pose_at=time.monotonic()

    def observe_joints(self,msg):
        output=JointState();output.header=msg.header
        for i,name in enumerate(msg.name):
            name=name.split('::')[-1]
            if name not in self.config['offsets']:continue
            value=float(msg.position[i])+self.config['offsets'][name]
            if not math.isfinite(value):return
            output.name.append(name);output.position.append(value)
            output.velocity.append(float(msg.velocity[i]) if len(msg.velocity)>i else 0.)
            output.effort.append(float(msg.effort[i]) if len(msg.effort)>i else 0.)
            self.joints[name]=value
        self.joints_at=time.monotonic()
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        # Match actual encoder intervals to the 50 Hz contact frames. The
        # stock publisher emits every 1 ms physics step; differentiating
        # these against an older 20 ms support set creates false slip.
        if self.state_stamp is None or stamp<self.state_stamp or stamp-self.state_stamp>=.02:
            self.state_stamp=stamp;self.states.publish(output)

    def observe_contacts(self,msg):
        try:
            value=json.loads(msg.data)
            if value.get('valid') is True:self.contacts.publish(msg)
        except (ValueError,TypeError):pass

    def command(self,msg):
        if len(msg.data)!=len(self.model.names) or not all(map(math.isfinite,msg.data)):return
        if any(not lo<=value<=hi for value,lo,hi in zip(msg.data,self.model.lower,self.model.upper)):return
        self.targets=dict(zip(self.model.names,map(float,msg.data)));self.received=time.monotonic()

    def tick(self):
        targets=self.targets if time.monotonic()-self.received<.5 else self.config['offsets']
        for name,pub in self.pub.items():pub.publish(Float64(data=float(targets[name]-self.config['offsets'][name])))

    def link_pose(self,name):
        q=self.pose.orientation;p=self.pose.position
        body=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();position=np.array([p.x,p.y,p.z])
        chain,names,solver=self.chains[name]
        if any(name not in self.joints for name in names):raise ValueError('Complete source joint feedback required for terrain sensor')
        frame=kdl.Frame();solver.JntToCart(self.model.array([self.joints[name] for name in names]),frame)
        matrix=np.array([[frame.M[i,j] for j in range(3)] for i in range(3)])
        return position+body@np.array([frame.p.x(),frame.p.y(),frame.p.z()]),body@matrix

    def ray(self,origin,length):
        if time.monotonic()>self.ray_deadline:raise ValueError('Static terrain sensor exceeded 80 ms budget')
        return self.static_ray(origin,length)

    def sense(self):
        now=time.monotonic()
        if self.clock is None:return
        ages=dict(pose=now-self.pose_at,joints=now-self.joints_at,clock=now-self.clock_at)
        if max(ages.values())>.5:
            self.terrain.publish(String(data=json.dumps(dict(clock_time=self.clock,error='Native terrain inputs stale: '+', '.join(f'{k}={v:.3f}s' for k,v in ages.items()),feet={}))))
            return
        self.ray_deadline=now+.08
        try:
            value=sample_probes(self.config['probes'],self.link_pose,lambda name:-self.radii[name],self.static_ray,self.clock)
            if time.monotonic()>self.ray_deadline:raise ValueError('Static terrain sensor exceeded 80 ms budget')
            value['source']='Authored selected static collision range model, placed by native pose/source FK; no dynamic-actor coverage'
            value['native_input_age_s']=ages
        except (ValueError,KeyError) as exc:value=dict(clock_time=self.clock,error=str(exc),feet={})
        self.terrain.publish(String(data=json.dumps(value,allow_nan=False)))


def main(args=None):
    from robot_lab_utils.process_lifetime import exit_with_parent
    exit_with_parent();rclpy.init(args=args);parser=argparse.ArgumentParser();parser.add_argument('--config',required=True)
    opts=parser.parse_args(rclpy.utilities.remove_ros_args()[1:]);node=Adapter(json.loads(Path(opts.config).read_text()))
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
