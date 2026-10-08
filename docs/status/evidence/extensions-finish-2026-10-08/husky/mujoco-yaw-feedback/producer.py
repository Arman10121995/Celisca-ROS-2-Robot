#!/usr/bin/env python3
"""Measure a normal GUI wheeled Drive workflow against independent engine poses.

Run serially with sourced ROS/SSD environments, Xvfb and an isolated domain.
This is a physical motion/sensor screen on one named map, not universal robot
or SLAM/navigation qualification. Original and negative traces are retained.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import time
from types import SimpleNamespace

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState, LaserScan
from tf2_ros import Buffer, TransformListener

from robot_lab_gui.launcher import SimulationLauncherGui
from robot_lab_utils.sim_frames import offset_from_root


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot', required=True)
    parser.add_argument('--backend', required=True, choices=['gazebo','mujoco','pybullet','isaac'])
    parser.add_argument('--map', default='nav_empty')
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--startup-budget', default=200., type=float)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.output.parent.stat().st_dev == Path('/').stat().st_dev:
        parser.error('Physical trial output must be on the workspace SSD')
    args.output.mkdir(exist_ok=False)
    out, root = args.output, Path(__file__).resolve().parents[1]
    (out/'producer.py').write_bytes(Path(__file__).read_bytes())
    app = SimulationLauncherGui(); app.update()
    node = None
    body, joints, commands, scans, depths, infos = [], [], [], [], [], []
    report = dict(robot=args.robot,backend=args.backend,map=args.map,mode='display',passed=False,
        scope='Normal GUI neutral/WASD/Stop/publisher loss/owned relaunch and measured wheel/TF/sensor feedback on this map only; no SLAM or navigation.')
    last_log = 0.

    def pump():
        nonlocal last_log
        app.update()
        rclpy.spin_once(node, timeout_sec=.005)
        if time.monotonic()-last_log > 5:
            (out/'live-gui-output.log').write_text(app.output.get('1.0','end'))
            last_log = time.monotonic()
        assert app.process.poll() is None, 'Owned plant exited during Drive'

    def state(message):
        p,q = message.pose.pose.position, message.pose.pose.orientation
        body.append(dict(wall=time.monotonic(),sim=message.header.stamp.sec+message.header.stamp.nanosec/1e9,
            x=p.x,y=p.y,z=p.z,yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),
            tilt=math.acos(max(-1.,min(1.,1-2*(q.x*q.x+q.y*q.y))))))

    def depth(message):
        if message.encoding not in ('32FC1','16UC1'):
            return
        values = np.frombuffer(message.data,dtype=np.float32 if message.encoding=='32FC1' else np.uint16)[::19]
        if message.encoding == '16UC1': values = values.astype(float)/1000.
        values = values[np.isfinite(values)&(values>0)]
        depths.append(dict(frame=message.header.frame_id,width=message.width,height=message.height,
            finite_positive=len(values),minimum=float(values.min()) if len(values) else None,
            maximum=float(values.max()) if len(values) else None))

    def interval(seconds):
        start = body[-1]['sim']; deadline = time.monotonic()+max(60.,seconds*50)
        while body[-1]['sim']-start < seconds and time.monotonic() < deadline: pump()
        assert body[-1]['sim']-start >= seconds, 'Simulation clock stalled'

    def measure(points):
        assert len(points)>10, 'Insufficient independent body samples'
        end = points[-1]; tail = [s for s in points if s['sim']>end['sim']-.5]
        first = tail[0]; dt = end['sim']-first['sim']
        turn = sum(math.atan2(math.sin(b['yaw']-a['yaw']),math.cos(b['yaw']-a['yaw'])) for a,b in zip(tail,tail[1:]))
        heading = first['yaw']+turn/2
        return dict(dx=end['x']-points[0]['x'],dy=end['y']-points[0]['y'],
            yaw_change=sum(math.atan2(math.sin(b['yaw']-a['yaw']),math.cos(b['yaw']-a['yaw'])) for a,b in zip(points,points[1:])),
            tail_vx=((end['x']-first['x'])*math.cos(heading)+(end['y']-first['y'])*math.sin(heading))/dt,
            tail_wz=turn/dt,max_tilt=max(s['tilt'] for s in points),sim_s=end['sim']-points[0]['sim'])

    def key(value, down=True):
        (app._drive_key_press if down else app._drive_key_release)(SimpleNamespace(keysym=value,widget=app))

    def stop():
        start = len(body); key('space'); interval(2.5)
        result = measure(body[start:])
        assert abs(result['tail_vx'])<.025 and abs(result['tail_wz'])<.05, result
        return result

    def wait_ready():
        first, first_joints = len(body),len(joints)
        deadline = time.monotonic()+args.startup_budget
        while time.monotonic()<deadline and (len(body)-first<50 or len(joints)-first_joints<30): pump()
        assert len(body)-first>=50 and len(joints)-first_joints>=30, (len(body)-first,len(joints)-first_joints)
        interval(1.5)
        settled=body[-1]
        floor=float(app.map_profiles[args.map].get('spawn',{}).get('z',0))
        assert floor-.25<=settled['z']<=floor+.75,('Robot fell or failed to settle',settled)

    def stop_launch():
        app._stop_drive(); app._stop_launch(); deadline = time.monotonic()+60
        while app._launch_running and time.monotonic()<deadline:
            app.update();rclpy.spin_once(node,timeout_sec=.01)
        code = app.process.poll()
        assert code == 0, ('Owned launch cleanup',code)
        return code

    try:
        app.robot_var.set(args.robot);app.simulator_var.set(args.backend);app.map_var.set(args.map)
        app.mode_var.set('display');app.gui_var.set('false');app._update_from_selection();app.update()
        assert app.start_button.instate(['!disabled']),app.validation_var.get()
        assert app.drive_input_checkbox.instate(['!disabled']),app.drive_status_var.get()
        profile = app.robot_profiles[args.robot]; model = Path(profile['xacro'])
        sensor = profile.get('sensor_config',{}); drive = profile['drive']
        wheels = drive.get('left_wheel_joints',[drive['left_wheel_joint']])+drive.get('right_wheel_joints',[drive['right_wheel_joint']])
        report['command'] = app.command_var.get()
        (out/'executed.urdf').write_bytes(model.read_bytes())
        files = [p for name in ('robot_lab_utils','robot_lab_gui','robot_lab_bringup','robot_lab_description',
            'robot_lab_mujoco','robot_lab_pybullet','robot_lab_isaac') for p in (root/'src'/name).rglob('*.py')]
        files += [root/'scripts/extension_husky_control.py',Path(profile['xacro']).parent/'drive-controllers.yaml']
        (out/'source-manifest.json').write_text(json.dumps(dict(revision=subprocess.check_output(
            ['git','rev-parse','HEAD'],cwd=root,text=True).strip(),profile=profile,domain=os.environ.get('ROS_DOMAIN_ID'),
            source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files if p.is_file()},
            executed_urdf_sha256=hashlib.sha256(model.read_bytes()).hexdigest()),indent=2)+'\n')
        rclpy.init();node=rclpy.create_node('robot_lab_gui_drive_observer')
        node.create_subscription(Odometry,'/odom/ground_truth',state,qos_profile_sensor_data)
        node.create_subscription(JointState,'/joint_states',lambda m:joints.append(dict(names=m.name,positions=list(m.position),velocities=list(m.velocity))),50)
        node.create_subscription(Twist,'/key_vel',lambda m:commands.append(dict(wall=time.monotonic(),vx=m.linear.x,wz=m.angular.z)),50)
        node.create_subscription(LaserScan,'/scan',lambda m:scans.append(dict(frame=m.header.frame_id,samples=len(m.ranges),
            minimum=m.range_min,maximum=m.range_max,finite=sum(math.isfinite(v) for v in m.ranges))),qos_profile_sensor_data)
        node.create_subscription(Image,'/oakd/depth/image_raw',depth,qos_profile_sensor_data)
        node.create_subscription(CameraInfo,'/oakd/rgb/camera_info',lambda m:infos.append(dict(frame=m.header.frame_id,
            width=m.width,height=m.height,k=list(m.k))),qos_profile_sensor_data)
        tf = Buffer();listener = TransformListener(tf,node)
        app.start_button.invoke();report['owned_pid']=app.process.pid
        wait_ready();origin = [body[-1]['x'],body[-1]['y']]
        report['phases']={};start=len(body);count=len(commands)
        app.drive_input_checkbox.invoke();interval(1.)
        report['phases']['armed_neutral']=measure(body[start:]);report['neutral_commands']=len(commands)-count
        assert report['neutral_commands']==0, 'Arming published a movement command'
        for label,value in [('forward','w'),('reverse','s'),('left_turn','a'),('right_turn','d')]:
            start=len(body);key(value);interval(3.5);key(value,False)
            result=measure(body[start:]);report['phases'][label]=result
            if value in ('w','s'):
                signed = result['tail_vx']*(1 if value=='w' else -1)
                assert .5*drive['max_speed']<signed<1.4*drive['max_speed'],result
            else:
                signed = result['tail_wz']*(1 if value=='a' else -1)
                assert .5*drive['max_angular_speed']<signed<1.4*drive['max_angular_speed'],result
            report['phases'][label+'_stop']=stop()
        start=len(body);key('w');interval(2.);report['phases']['pre_loss']=measure(body[start:])
        if app.drive_repeat_job is not None:app.after_cancel(app.drive_repeat_job);app.drive_repeat_job=None
        start=len(body);count=len(commands);interval(3.)
        result=measure(body[start:]);report['phases']['publisher_loss']=result
        report['silence_commands']=len(commands)-count
        assert report['silence_commands']<=1 and abs(result['tail_vx'])<.025 and abs(result['tail_wz'])<.05,result
        assert all(math.isfinite(v) for s in body for v in (s['x'],s['y'],s['z'],s['tilt']))
        assert max(s['tilt'] for s in body)<.3,'Body became unstable'
        report['body_height_m']=dict(minimum=min(s['z'] for s in body),maximum=max(s['z'] for s in body))
        assert report['body_height_m']['maximum']-report['body_height_m']['minimum']<.25,'Body left the floor'
        report['wheel_feedback']={}
        for wheel in wheels:
            values=[j['positions'][j['names'].index(wheel)] for j in joints if wheel in j['names'] and j['positions']]
            assert values and max(values)-min(values)>.1,('Missing physical wheel feedback',wheel)
            report['wheel_feedback'][wheel]=dict(minimum=min(values),maximum=max(values),samples=len(values))
        if sensor:
            assert len(scans)>5 and all(s['frame']==sensor['laser_link_name'] and s['samples']==sensor['scan_samples'] and
                abs(s['minimum']-sensor['scan_range_min'])<1e-5 and abs(s['maximum']-sensor['scan_range_max'])<1e-5 for s in scans)
            assert max(s['finite'] for s in scans)>30, 'Lidar saw no geometry'
            if sensor.get('camera_rate',0)>0:
                assert len(depths)>5 and len(infos)>5 and max(d['finite_positive'] for d in depths)>100
                assert all(d['frame']==sensor['camera_optical_frame'] and d['width']==sensor['camera_width'] for d in depths)
            frames={}
            for frame in [sensor['laser_link_name']]+([sensor['camera_optical_frame']] if sensor.get('camera_rate',0)>0 else []):
                expected=offset_from_root(model.read_text(),frame)
                # Robot descriptions used here are plain installed URDFs.
                import xml.etree.ElementTree as ET
                description=ET.parse(model).getroot()
                children={j.find('child').get('link') for j in description.findall('joint')}
                base=next(l.get('name') for l in description.findall('link') if l.get('name') not in children)
                pose=tf.lookup_transform(base,frame,rclpy.time.Time()).transform
                actual=[pose.translation.x,pose.translation.y,pose.translation.z]
                assert max(abs(a-b) for a,b in zip(actual,expected[0]))<1e-5,(frame,actual,expected)
                frames[frame]=dict(position_m=actual,expected_position_m=list(expected[0]))
            report['sensors']=dict(scan_messages=len(scans),depth_messages=len(depths),camera_info_messages=len(infos),
                                  last_scan=scans[-1],last_depth=depths[-1] if depths else None,frames=frames)
        report['real_time_factor']=(body[-1]['sim']-body[0]['sim'])/(body[-1]['wall']-body[0]['wall'])
        report['first_launch_returncode']=stop_launch()
        # A real owned stop/relaunch tests the full initial plant state on every
        # engine, including Gazebo where the robot-only Reset button is disabled.
        app.start_button.invoke();wait_ready()
        restored=[body[-1]['x'],body[-1]['y']]
        report['relaunch_reset']=dict(original_xy=origin,restored_xy=restored,error_m=math.dist(origin,restored))
        assert report['relaunch_reset']['error_m']<.04,report['relaunch_reset']
        report['launch_returncode']=stop_launch();report['body_samples']=len(body);report['joint_samples']=len(joints)
        report['passed']=True
    except BaseException as exc:
        report['exception']=repr(exc)
        raise
    finally:
        if app.process and app.process.poll() is None:
            app._stop_drive();app._stop_launch();deadline=time.monotonic()+60
            while app._launch_running and time.monotonic()<deadline:app.update();time.sleep(.02)
        report['launch_returncode']=app.process.poll() if app.process else None
        (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        (out/'trace.json').write_text(json.dumps(dict(body=body,joints=joints,commands=commands,scans=scans,depths=depths,camera_info=infos)))
        (out/'gui-output.log').write_text(app.output.get('1.0','end'))
        if node:node.destroy_node()
        app._on_close()
        if rclpy.ok():rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
