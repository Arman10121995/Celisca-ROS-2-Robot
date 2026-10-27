import hashlib,json,os,signal,subprocess,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
base=Path(os.environ['LAB_EXTENSION_RUN']);folder=base/('gait-nav-'+time.strftime('%H%M%S'));folder.mkdir()
root=Path('/workspace/molar/ros_ws/bumperbot_ws');files=list((root/'src/robot_lab_utils/robot_lab_utils').glob('gait_*.py'))
(folder/'sources.json').write_text(json.dumps({str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},indent=2))
map_yaml=Path(os.environ['LAB_GAIT_MAP']);source_hash=hashlib.sha256(map_yaml.read_bytes()).hexdigest()
command=['ros2','launch','robot_lab_bringup','simulated_robot.launch.py','robot_model:=hexapod_ros2','mode:=gait','simulator:=mujoco','map_name:=nav_empty','gait_task:=nav','gait_map_yaml:='+str(map_yaml),'gui:=false','start_rviz:=false','enable_joystick:=false']
(folder/'command.json').write_text(json.dumps(command));log=(folder/'launch.log').open('w');proc=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init();node=Node('actual_gait_navigation_diagnostic');native=None;states=[];truth=[];estimated=[];amcl=[]
def observe(msg):
 global native
 native=json.loads(msg.data);states.append(native)
def odom(msg,rows):
 p=msg.pose.pose.position;rows.append(dict(stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,position=[p.x,p.y,p.z]))
node.create_subscription(String,'/gait/state',observe,10);node.create_subscription(Odometry,'/odom/ground_truth',lambda m:odom(m,truth),10);node.create_subscription(Odometry,'/gait/odometry',lambda m:odom(m,estimated),10)
node.create_subscription(PoseWithCovarianceStamped,'/gait/amcl_pose',lambda m:amcl.append(dict(x=m.pose.pose.position.x,y=m.pose.pose.position.y)),10)
hb=node.create_publisher(String,'/gait/operator_heartbeat',10);ctrl=node.create_publisher(String,'/gait/command',10);client=ActionClient(node,NavigateToPose,'/gait/navigate_to_pose')
started=time.monotonic();phase='waiting';last=0.;common=None;future=None;handle=None;result=None;retry_at=0.;target=[.4,0.,0.]
try:
 while time.monotonic()-started<95 and proc.poll() is None:
  rclpy.spin_once(node,timeout_sec=.02);now=time.monotonic()
  if native:
   common=dict(session_id=native['session_id'],operator_id='gait-nav-diagnostic');hb.publish(String(data=json.dumps(common)))
   if phase=='waiting' and native['ready'] and native.get('operator_id')==common['operator_id']:
    ctrl.publish(String(data=json.dumps(dict(common,op='arm'))));phase='arming'
   elif phase=='arming' and native['enabled'] and amcl and client.server_is_ready() and now>=retry_at:
    goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.pose.position.x=target[0];goal.pose.pose.orientation.w=1.
    future=client.send_goal_async(goal);phase='submitting'
   elif phase=='submitting' and future.done():
    handle=future.result()
    if handle.accepted:future=handle.get_result_async();phase='navigating'
    else:phase='arming';retry_at=now+2.
   elif phase=='navigating' and future.done():
    result=dict(status=future.result().status);phase='terminal';break
  if now-last>12.:
   last=now;print(json.dumps(dict(seconds=round(now-started,1),phase=phase,native=native.get('reason') if native else None,truth=truth[-1:] ,estimate=estimated[-1:],amcl=amcl[-1:])),flush=True)
finally:
 if handle and handle.accepted and phase!='terminal':
  cancel=handle.cancel_goal_async();until=time.monotonic()+3
  while not cancel.done() and time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
 if common:ctrl.publish(String(data=json.dumps(dict(common,op='stop'))))
 for _ in range(4):rclpy.spin_once(node,timeout_sec=.05)
 node.destroy_node();rclpy.shutdown()
 from robot_lab_gui.process_control import stop_owned_launch
 stop_owned_launch(proc,grace_timeout=25.)
 log.close();report=dict(folder=str(folder),phase=phase,result=result,native=native,requested=target,map_yaml=str(map_yaml),map_source_unchanged=source_hash==hashlib.sha256(map_yaml.read_bytes()).hexdigest(),last_amcl=amcl[-1:] ,returncode=proc.returncode)
 (folder/'report.json').write_text(json.dumps(report,indent=2));(folder/'states.json').write_text(json.dumps(states));(folder/'motion.json').write_text(json.dumps(dict(truth=truth,estimated=estimated,amcl=amcl)));print(json.dumps(dict(folder=str(folder),phase=phase,result=result)),flush=True)
