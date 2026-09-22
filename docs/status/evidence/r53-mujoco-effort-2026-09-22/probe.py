#!/usr/bin/env python3
"""Live common-launch stance comparison; run: probe.py OUTPUT active|passive.

Requires a sourced ROS overlay and the workspace simulator dependencies.
Set an isolated ROS_DOMAIN_ID in the caller. Only child process groups created
by this probe are stopped. Exit 1 means stance/cleanup qualification failed;
the passive condition is expected to fail the stance criterion.
"""
import json,math,os,signal,subprocess,sys,time
from pathlib import Path
import rclpy
from sensor_msgs.msg import Imu,JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory as share
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True)
active=len(sys.argv)>2 and sys.argv[2]=='active'
rclpy.init();n=rclpy.create_node('bhl_live_effort_probe');latest={};records=[];counts={};children=[]
def put(k,m):
 latest[k]=m;counts[k]=counts.get(k,0)+1
for key,typ,topic in [('imu',Imu,'/imu/out'),('js',JointState,'/joint_states'),('odom',Odometry,'/odom/ground_truth'),('ready',Bool,'/robot_lab/ready')]:
 n.create_subscription(typ,topic,lambda m,k=key:put(k,m),10)
reset=n.create_client(Trigger,'/robot_lab/reset')
def start(name,cmd):
 with (out/(name+'.log')).open('w') as log:p=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 children.append(p);return p
def stop(p):
 if p.poll() is None:
  os.killpg(p.pid,signal.SIGINT)
  try:p.wait(5)
  except subprocess.TimeoutExpired:
   os.killpg(p.pid,signal.SIGTERM)
   try:p.wait(3)
   except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(3)
def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
result={'active':active,'passed':False}
try:
 controller=None
 if active:
  controller=start('balance',[sys.executable,'-m','robot_lab_adapter.humanoid_standing_controller','--ros-args','-p','use_sim_time:=true'])
  until=time.monotonic()+1
  while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.01)
 # Use the installed common launch entrypoint, including automatic BHL effort config.
 launch=start('launch',['ros2','launch','robot_lab_bringup','simulated_robot.launch.py','mode:=display','map_name:=nav_empty','robot_model:=berkeley_humanoid_lite_sim','simulator:=mujoco','gui:=false','start_rviz:=false'])
 deadline=time.monotonic()+90;last=-1
 while time.monotonic()<deadline:
  assert launch.poll() is None,'launch exited'
  if controller:assert controller.poll() is None,'balance exited'
  rclpy.spin_once(n,timeout_sec=.01)
  if not {'imu','js','odom','ready'}<=latest.keys() or not latest['ready'].data:continue
  js=latest['js'];t=stamp(js)
  assert len(js.position)==len(js.velocity)==len(js.effort)==22, 'incomplete articulated state'
  assert all(math.isfinite(v) for v in list(js.position)+list(js.velocity)+list(js.effort)), 'non-finite state'
  if t==last:continue
  last=t;q=latest['imu'].orientation
  tilt=math.acos(max(-1,min(1,1-2*(q.x*q.x+q.y*q.y))))
  records.append({'t':t,'tilt':tilt,'effort_max':max(abs(v) for v in js.effort),'joint_speed_max':max(abs(v) for v in js.velocity)})
  if len(records)%250==0:print(records[-1],flush=True)
  if t>=12:break
 assert records and records[-1]['t']>=12,'missing state/sim budget'
 result.update(max_tilt=max(r['tilt'] for r in records),max_effort=max(r['effort_max'] for r in records),counts=counts,sim_seconds=records[-1]['t'])
 if controller:stop(controller)
 until=time.monotonic()+1
 while time.monotonic()<until:rclpy.spin_once(n,timeout_sec=.01)
 result['watchdog_effort']=max(abs(v) for v in latest['js'].effort)
 assert reset.wait_for_service(timeout_sec=3)
 oldtime=stamp(latest['js']);future=reset.call_async(Trigger.Request());deadline=time.monotonic()+5
 while not future.done() and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.01)
 assert future.done() and future.result().success,'reset failed'
 deadline=time.monotonic()+5
 while stamp(latest['js'])>=oldtime and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.01)
 result['reset_time']=stamp(latest['js']);result['reset_effort']=max(abs(v) for v in latest['js'].effort)
 result['passed']=result['max_tilt']<.2 and result['max_effort']<=20.0001 and result['watchdog_effort']<1e-8 and result['reset_effort']<1e-8 and result['reset_time']<oldtime
except Exception as e:result['error']=repr(e)
finally:
 for child in reversed(children):stop(child)
 result['child_exit_codes']=[child.returncode for child in children]
 result['clean_shutdown']=all(child.returncode==0 for child in children) and all('Traceback' not in path.read_text() and 'process has died' not in path.read_text() for path in out.glob('*.log'))
 result['passed']=result['passed'] and result['clean_shutdown']
 n.destroy_node();rclpy.shutdown()
 (out/'report.json').write_text(json.dumps(result,indent=2)+'\n')
 (out/'trace.json').write_text(json.dumps(records,indent=2)+'\n')
 print(json.dumps(result,indent=2),flush=True)

sys.exit(0 if result['passed'] else 1)
