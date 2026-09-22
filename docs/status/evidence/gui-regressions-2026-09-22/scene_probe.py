#!/usr/bin/env python3
"""Live launch/state smoke probe. Args: backend robot map mode duration output.

Use a sourced ROS overlay, simulator dependencies, and an isolated ROS domain.
This checks startup and measured state; it does not qualify mission completion.
"""
import json, math, os, signal, subprocess, sys, time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, Image
from nav_msgs.msg import Odometry, OccupancyGrid
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
sim, robot, arena, mode, duration, output = sys.argv[1:]
out = Path(output); out.mkdir(parents=True, exist_ok=True)
cmd = ['ros2', 'launch', 'robot_lab_bringup', 'simulated_robot.launch.py',
       'simulator:='+sim, 'robot_model:='+robot, 'map_name:='+arena,
       'mode:='+mode, 'gui:=false', 'start_rviz:=false']
if arena != 'none':
    cmd += ['world_path:=maps/'+arena+'/worlds/'+arena+'.world',
            'map_yaml:=maps/'+arena+'/maps/map.yaml']
rclpy.init(); node = rclpy.create_node('gui_scene_probe')
counts = {}; latest = {}; states = []; times = []
def receive(key, msg):
    counts[key] = counts.get(key, 0) + 1; latest[key] = msg
    if key == 'joints':
        states.append({'t': msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9,
                       'names': list(msg.name), 'position': list(msg.position),
                       'velocity': list(msg.velocity)})
    if key == 'clock': times.append(msg.clock.sec + msg.clock.nanosec*1e-9)
for key, typ, topic in [('joints',JointState,'/joint_states'), ('clock',Clock,'/clock'),
                        ('odom',Odometry,'/odom/ground_truth'), ('tf',TFMessage,'/tf'),
                        ('ready',Bool,'/robot_lab/ready'), ('rgb',Image,'/oakd/rgb/image_raw'),
                        ('depth',Image,'/oakd/depth/image_raw'), ('map',OccupancyGrid,'/map')]:
    node.create_subscription(typ, topic, lambda m,k=key:receive(k,m), qos_profile_sensor_data)
result = {'command':cmd, 'passed':False}
with (out/'launch.log').open('w') as log:
    child = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
try:
    until = time.monotonic()+float(duration)
    while child.poll() is None and time.monotonic()<until:
        rclpy.spin_once(node, timeout_sec=.05)
    log_before_stop = (out/'launch.log').read_text()
    settled = [v for s in states if s['t']>2 for v in s['velocity']]
    checks = {'clock_advances':len(times)>10 and times[-1]>times[0],
              'no_startup_crash':'process has died' not in log_before_stop and 'Traceback' not in log_before_stop,
              'measured_state': (counts.get('joints',0)>10 and counts.get('tf',0)>10) if robot!='none' else counts.get('joints',0)==0,
              'finite_joints':all(math.isfinite(v) for s in states for v in s['position']+s['velocity'])}
    if mode == 'display' and robot not in ('none','bumperbot','labbot'):
        checks['settled_display_joints'] = bool(settled) and max(abs(v) for v in settled)<.1
    if mode == '3d_slam': checks['rgbd_received'] = counts.get('rgb',0)>5 and counts.get('depth',0)>5
    result.update(checks=checks, passed=all(checks.values()), counts=counts,
                  final_sim_time=times[-1] if times else None,
                  settled_max_joint_speed=max((abs(v) for v in settled),default=None))
finally:
    if child.poll() is None:
        # Let launch forward its one shutdown signal to its children.
        child.send_signal(signal.SIGINT)
        try: child.wait(10)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid,signal.SIGTERM); child.wait(5)
    result['launch_exit_code'] = child.returncode
    node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()
    (out/'report.json').write_text(json.dumps(result,indent=2)+'\n')
    (out/'joints.json').write_text(json.dumps(states)+'\n')
    print(json.dumps(result,indent=2))
sys.exit(0 if result['passed'] else 1)
