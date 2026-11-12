#!/usr/bin/env python3
"""Finite source-plant bring-up probe through the normal GUI-equivalent API.

Records actual simulator odometry, source joints, images and controller state.
It does not certify navigation, collision avoidance, waves or hardware parity.
"""
import argparse
import json
import hashlib
import math
import os
import re
from pathlib import Path
import signal
import subprocess
import time
import uuid
import shutil
import xml.etree.ElementTree as ET

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot',required=True)
    parser.add_argument('--map',default='surface_harbor')
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--timeout',type=float,default=120.)
    args=parser.parse_args()
    args.out=args.out.resolve()
    if not str(args.out).startswith('/workspace/') or args.out.exists():
        parser.error('Use a fresh SSD output file')
    args.out.parent.mkdir(parents=True,exist_ok=True)
    root=Path(__file__).resolve().parents[1]
    snapshot=args.out.with_suffix('.source');snapshot.mkdir()
    hashes={}
    for relative in ['scripts/screen_marine_counterpart.py',
            'scripts/provision_robot_counterparts.py',
            'src/robot_lab_bringup/launch/marine_simulator.launch.py',
            *['src/robot_lab_utils/robot_lab_utils/'+name+'.py' for name in ('marine_model','marine_controller','surface_controller','marine_pipeline','bluerov_source')]]:
        source=root/relative;destination=snapshot/relative
        destination.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,destination)
        hashes[relative]=hashlib.sha256(source.read_bytes()).hexdigest()
    env=dict(os.environ,IGN_PARTITION='marine_probe_'+uuid.uuid4().hex)
    env['GZ_PARTITION']=env['IGN_PARTITION']
    trace=[];observed={};joint_samples=[];images=[];state={}
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=rclpy.create_node('source_marine_probe')
    command=node.create_publisher(String,'/marine/command',10)
    heartbeat=node.create_publisher(String,'/marine/operator_heartbeat',10)
    drive=node.create_publisher(Twist,'/marine/key_vel',10)
    def status(message):
        try:state.update(json.loads(message.data))
        except (TypeError,ValueError):pass
    def pose(message):
        p=message.pose.pose.position;q=message.pose.pose.orientation
        observed.update(t=message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
                        pose=[p.x,p.y,p.z],quaternion=[q.x,q.y,q.z,q.w],received=time.monotonic())
    def joints(message):
        observed['joints']=dict(names=list(message.name),position=list(message.position),velocity=list(message.velocity))
        if len(joint_samples)<10:joint_samples.append(dict(names=list(message.name),position=list(message.position)))
    def image(message):
        if len(images)<5:images.append(dict(width=message.width,height=message.height,bytes=len(message.data),frame=message.header.frame_id))
    node.create_subscription(String,'/marine/state',status,10)
    node.create_subscription(Odometry,'/marine/odometry',pose,10)
    node.create_subscription(JointState,'/joint_states',joints,qos_profile_sensor_data)
    node.create_subscription(Image,'/oakd/rgb/image_raw',image,qos_profile_sensor_data)
    owner='source-probe-'+uuid.uuid4().hex[:8]
    phases=[('neutral',2.,0.,0.),('forward',8.,.25,0.),('stop',4.,0.,0.),('turn',8.,0.,.2),('stop_final',4.,0.,0.)]
    stage=-1;at=None;start=time.monotonic();last_record=0.;error='';complete=False
    logfile=args.out.with_suffix('.launch.log')
    with logfile.open('w') as stream:
        process=subprocess.Popen(['ros2','launch','robot_lab_bringup','simulated_robot.launch.py',
            'robot_model:='+args.robot,'simulator:=gazebo','mode:=marine','map_name:='+args.map,
            'allow_experimental_assets:=true','gui:=false','start_rviz:=false'],
            env=env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            while time.monotonic()-start<args.timeout:
                rclpy.spin_once(node,timeout_sec=.03)
                if process.poll() is not None:raise RuntimeError('Launch exited: '+str(process.returncode))
                if logfile.stat().st_size>64*1024*1024:raise RuntimeError('Log exceeded 64 MB bound')
                session=state.get('session_id')
                if not session:continue
                envelope=dict(session_id=session,operator_id=owner)
                heartbeat.publish(String(data=json.dumps(envelope)))
                if stage<0:
                    if not state.get('ready') or not observed or time.monotonic()-observed['received']>.5:continue
                    # Let the source hull settle before its explicit neutral arm.
                    if observed['t']<6.:continue
                    command.publish(String(data=json.dumps(dict(envelope,op='arm'))))
                    if not state.get('enabled'):continue
                    stage=0;at=observed['t']
                if not state.get('enabled'):raise RuntimeError('Controller disarmed: '+state.get('reason',''))
                name,duration,surge,yaw=phases[stage]
                msg=Twist();msg.linear.x=surge;msg.angular.z=yaw;drive.publish(msg)
                if time.monotonic()-last_record>.1:
                    trace.append(dict(phase=name,sim=observed.get('t'),pose=observed.get('pose'),
                        quaternion=observed.get('quaternion'),joints=observed.get('joints'),state=dict(state)))
                    last_record=time.monotonic()
                if observed['t']-at>=duration:
                    stage+=1
                    if stage==len(phases):complete=True;break
                    at=observed['t']
            if not complete:raise RuntimeError('Finite wall deadline reached')
        except (Exception,KeyboardInterrupt) as exc:error=str(exc) or type(exc).__name__
        finally:
            if rclpy.ok() and state.get('session_id'):
                command.publish(String(data=json.dumps(dict(session_id=state['session_id'],op='stop'))))
                drive.publish(Twist())
            os.killpg(process.pid,signal.SIGINT) if process.poll() is None else None
            try:process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM)
                try:process.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
            node.destroy_node();rclpy.shutdown()
    result=dict(robot=args.robot,map=args.map,backend='Gazebo Fortress',completed_phases=complete,
                error=error,wall_s=time.monotonic()-start,trace=trace,joint_samples=joint_samples,images=images,
                source_sha256=hashes,source_snapshot=str(snapshot),
                scope='Initial physical source bring-up; no navigation/contact/wave/hardware qualification')
    match=re.search(r'\[robot_lab marine\] (.*session.json)',logfile.read_text())
    if match:
        manifest=json.loads(Path(match.group(1)).read_text())
        result['session']=manifest
        description=Path(manifest['description'])
        resources=[description,Path(manifest.get('active_world',manifest['source_world']))]
        resources += [Path(p.get('filename')) for p in ET.parse(description).findall('gazebo/plugin')
                      if Path(p.get('filename')).is_file()]
        result['physical_sha256']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in resources}
        shutil.copy2(description,snapshot/'actual-description.urdf')
        shutil.copy2(resources[1],snapshot/'actual-world.sdf')
    args.out.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('trace','joint_samples')},indent=2))
    return 0 if complete else 1


if __name__=='__main__':raise SystemExit(main())
