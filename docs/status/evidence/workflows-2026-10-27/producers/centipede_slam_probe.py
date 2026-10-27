import hashlib,json,os,signal,subprocess,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry,OccupancyGrid
from std_msgs.msg import String
from std_srvs.srv import Trigger
base=Path(os.environ['LAB_EXTENSION_RUN']);folder=base/('gait-centipede-'+time.strftime('%H%M%S'));folder.mkdir()
root=Path('/workspace/molar/ros_ws/bumperbot_ws')
files=list((root/'src/robot_lab_utils/robot_lab_utils').glob('gait_*.py'))+[root/'src/robot_lab_utils/robot_lab_utils/map_archive.py']
(folder/'sources.json').write_text(json.dumps({str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},indent=2))
command=['ros2','launch','robot_lab_bringup','simulated_robot.launch.py','robot_model:=lab_centipede','mode:=gait','simulator:=mujoco','map_name:=nav_empty','gait_task:=slam','gui:=false','start_rviz:=false','enable_joystick:=false']
(folder/'command.json').write_text(json.dumps(command));log=(folder/'launch.log').open('w')
process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init();node=Node('gait_slam_diagnostic');native=None;mapping=None;truth=[];estimated=[];states=[];maps=[]
def observe(msg):
 global native
 native=json.loads(msg.data);states.append(native)
def mapping_state(msg):
 global mapping
 mapping=json.loads(msg.data)
def odom(msg,rows):
 p=msg.pose.pose.position;rows.append(dict(stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,position=[p.x,p.y,p.z]))
node.create_subscription(String,'/gait/state',observe,10);node.create_subscription(String,'/gait/mapping/state',mapping_state,10)
node.create_subscription(Odometry,'/odom/ground_truth',lambda m:odom(m,truth),10)
node.create_subscription(Odometry,'/gait/odometry',lambda m:odom(m,estimated),10)
node.create_subscription(OccupancyGrid,'/gait/map',lambda m:maps.append(dict(stamp=m.header.stamp.sec,width=m.info.width,height=m.info.height,known=sum(v>=0 for v in m.data))),10)
hb=node.create_publisher(String,'/gait/operator_heartbeat',10);ctrl=node.create_publisher(String,'/gait/command',10);drive=node.create_publisher(Twist,'/key_vel',10)
client=node.create_client(Trigger,'/gait/mapping/save')
started=time.monotonic();phase='waiting';sent=started;future=None;saved=None;last=0.;common=None
try:
 while time.monotonic()-started<85 and process.poll() is None:
  rclpy.spin_once(node,timeout_sec=.02);now=time.monotonic()
  if native:
   common=dict(session_id=native['session_id'],operator_id='gait-slam-diagnostic');hb.publish(String(data=json.dumps(common)))
   if phase=='waiting' and native['ready'] and native.get('operator_id')==common['operator_id']:
    ctrl.publish(String(data=json.dumps(dict(common,op='arm'))));phase='arming';sent=now
   elif phase=='arming' and native['enabled']:
    phase='walking';sent=float(native['sim_time'])
   elif phase=='walking':
    elapsed=float(native['sim_time'])-sent;msg=Twist();msg.linear.x=.025 if elapsed<8. else 0.;drive.publish(msg)
    if elapsed>10.:
     ctrl.publish(String(data=json.dumps(dict(common,op='stop'))));phase='saving'
   elif phase=='saving' and maps and client.service_is_ready():
    future=client.call_async(Trigger.Request());phase='save_requested'
   elif phase=='save_requested' and future.done():
    saved=dict(success=future.result().success,message=future.result().message);phase='saved' if saved['success'] else 'save_failed';break
  if now-last>12.:
   last=now;print(json.dumps(dict(seconds=round(now-started,1),phase=phase,ready=native.get('ready') if native else None,reason=native.get('reason') if native else None,maps=len(maps),truth=len(truth),estimated=len(estimated))),flush=True)
finally:
 if common:ctrl.publish(String(data=json.dumps(dict(common,op='stop'))))
 for _ in range(4):rclpy.spin_once(node,timeout_sec=.05)
 node.destroy_node();rclpy.shutdown()
 from robot_lab_gui.process_control import stop_owned_launch
 stop_owned_launch(process,grace_timeout=25.)
 log.close()
 report=dict(folder=str(folder),phase=phase,saved=saved,mapping=mapping,native=native,truth_samples=len(truth),estimated_samples=len(estimated),maps=maps,returncode=process.returncode)
 (folder/'report.json').write_text(json.dumps(report,indent=2));(folder/'states.json').write_text(json.dumps(states));(folder/'motion.json').write_text(json.dumps(dict(truth=truth,estimated=estimated)));print(json.dumps(dict(folder=str(folder),phase=phase,saved=saved)),flush=True)
