#!/usr/bin/env python3
"""Record source-base motion through the GUI input path, with real sensors.

Used with sim_drive_check.sh's CHECKER option. One observer/controller process
records the source model bytes, actual body trajectory, wheels, camera and scan.
This finite bring-up screen does not qualify SLAM or navigation missions.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image, LaserScan, JointState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import yaml

from sim_drive_check import DriveCheck


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot',required=True)
    parser.add_argument('--phase-scale',type=float,default=1.)
    parser.add_argument('--odom',default='/odom/ground_truth')
    parser.add_argument('--timeout',type=float,default=90.)
    parser.add_argument('--warmup',type=float,default=3.)
    args=parser.parse_args()
    runtime=Path(os.environ.get('ROBOT_LAB_RUNTIME_ROOT','/workspace/molar/robot_lab_runtime'))
    profile=yaml.safe_load((runtime/'external_assets/installed/robots.yaml').read_text())['robots'][args.robot]
    description=Path(profile['xacro'])
    result=dict(robot=args.robot,source_id=profile['source_id'],source_revision=profile.get('source_revision'),
        description=str(description),description_sha256=hashlib.sha256(description.read_bytes()).hexdigest(),
        odom=args.odom,input='/key_vel',scope='Initial source-plant control and sensing; no mission qualification',
        phases=[],images=[],scans=[],joints=[],body_poses=[],completed=False,error='')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=DriveCheck(args.odom,'/key_vel','sim')
    def image(message):
        if len(result['images'])<8:
            result['images'].append(dict(width=message.width,height=message.height,encoding=message.encoding,
                bytes=len(message.data),frame=message.header.frame_id,
                sha256=hashlib.sha256(bytes(message.data)).hexdigest()))
    def scan(message):
        if len(result['scans'])<8:result['scans'].append(dict(frame=message.header.frame_id,
            samples=len(message.ranges),range_min=message.range_min,range_max=message.range_max))
    def joints(message):
        stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        if len(result['joints'])<800 and (not result['joints'] or stamp-result['joints'][-1]['stamp']>=.05):
            result['joints'].append(dict(stamp=stamp,
            name=list(message.name),position=list(message.position),velocity=list(message.velocity),effort=list(message.effort)))
    node.create_subscription(Image,'/oakd/rgb/image_raw',image,qos_profile_sensor_data)
    node.create_subscription(LaserScan,'/scan',scan,qos_profile_sensor_data)
    node.create_subscription(JointState,'/joint_states',joints,qos_profile_sensor_data)
    def body(message):
        stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9
        p,q=message.pose.pose.position,message.pose.pose.orientation
        values=[p.x,p.y,p.z,q.w,q.x,q.y,q.z]
        if not all(math.isfinite(v) for v in values):raise RuntimeError('Non-finite body state: '+repr(values))
        if len(result['body_poses'])<1500 and (not result['body_poses'] or stamp-result['body_poses'][-1]['stamp']>=.02):
            result['body_poses'].append(dict(stamp=stamp,position=values[:3],quaternion_wxyz=values[3:]))
    node.create_subscription(Odometry,args.odom,body,qos_profile_sensor_data)
    started=time.monotonic()
    try:
        if not node.wait_for_odom(args.timeout):raise RuntimeError('No actual body feedback')
        node.run_phase(0.,0.,args.warmup)
        for name,vx,vy,wz,duration in [('forward',.25,0.,0.,3.),('reverse',-.25,0.,0.,3.),
                ('strafe',0.,.25,0.,4.),('turn',0.,0.,.25,4.),('neutral',0.,0.,0.,3.)]:
            value=node.run_phase(vx,wz,duration*args.phase_scale,vy=vy)
            result['phases'].append(dict(name=name,requested=[vx,vy,wz],measured=value))
            if value is None:raise RuntimeError('Source phase did not advance: '+name)
        node.run_phase(.25,0.,3.)
        result['publisher_loss']=node.run_silence(3.)
        result['completed']=True
    except (Exception,KeyboardInterrupt) as exc:result['error']=str(exc) or type(exc).__name__
    finally:
        if rclpy.ok():node.pub.publish(Twist())
        result['truth_samples']=[list(sample) for sample in node.samples]
        result['wall_s']=time.monotonic()-started
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    print(json.dumps(result,indent=2,allow_nan=False))
    return 0 if result['completed'] else 1


if __name__=='__main__':raise SystemExit(main())
