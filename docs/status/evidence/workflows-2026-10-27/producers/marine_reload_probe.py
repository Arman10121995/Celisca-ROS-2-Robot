import hashlib,json,os,signal,subprocess,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
base=Path(os.environ['LAB_EXTENSION_RUN']);folder=base/('marine-reload-'+time.strftime('%H%M%S'));folder.mkdir()
database=Path(os.environ['LAB_SAVED_MARINE_DATABASE']);source_hash=hashlib.sha256(database.read_bytes()).hexdigest()
root=Path('/workspace/molar/ros_ws/bumperbot_ws');files=list((root/'src/robot_lab_utils/robot_lab_utils').glob('marine_*.py'))+[root/'src/robot_lab_utils/robot_lab_utils/map_archive.py']
(folder/'sources.json').write_text(json.dumps({str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},indent=2))
command=['ros2','launch','robot_lab_bringup','simulated_robot.launch.py','robot_model:=lab_rov','mode:=marine','simulator:=gazebo','map_name:=marine_tank','marine_slam:=true','marine_localization:=true','marine_database:='+str(database),'gui:=false','start_rviz:=false','enable_joystick:=false']
(folder/'command.json').write_text(json.dumps(command));log=(folder/'launch.log').open('w');proc=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init();node=Node('marine_reload_diagnostic');native=None;mission=None;states=[];localizations=[];truth=[];estimated=[]
def observe(msg):
 global native
 native=json.loads(msg.data)
def observe_mission(msg):
 global mission
 mission=json.loads(msg.data);states.append(dict(wall=time.time(),**mission))
def odom(msg,rows):
 p=msg.pose.pose.position;rows.append(dict(stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,wall=time.time(),position=[p.x,p.y,p.z]))
node.create_subscription(String,'/marine/state',observe,10);node.create_subscription(String,'/marine/mission/state',observe_mission,10)
node.create_subscription(Odometry,'/marine/odometry',lambda m:odom(m,truth),10);node.create_subscription(Odometry,'/marine/estimated_odometry',lambda m:odom(m,estimated),10)
node.create_subscription(PoseWithCovarianceStamped,'/marine/slam/localization_pose',lambda m:localizations.append(dict(stamp=m.header.stamp.sec,covariance=list(m.pose.covariance))),10)
hb=node.create_publisher(String,'/marine/operator_heartbeat',10);ctrl=node.create_publisher(String,'/marine/command',10);task=node.create_publisher(String,'/marine/mission/command',10)
started=time.monotonic();phase='waiting';common=None;target=None;last=0.
try:
 while time.monotonic()-started<90 and proc.poll() is None:
  rclpy.spin_once(node,timeout_sec=.02);now=time.monotonic()
  if native:
   common=dict(session_id=native['session_id'],operator_id='marine-reload-diagnostic');hb.publish(String(data=json.dumps(common)))
   if phase=='waiting' and native['ready'] and native.get('operator_id')==common['operator_id'] :
    ctrl.publish(String(data=json.dumps(dict(common,op='arm'))));phase='localizing'
   elif phase=='localizing' and native['enabled'] and mission and mission['ready']:
    target=[mission['position'][0]+.7,mission['position'][1],mission['position'][2]]
    task.publish(String(data=json.dumps(dict(common,op='plan',**dict(zip('xyz',target))))));phase='planning'
   elif phase=='planning' and mission['status']=='planned':
    task.publish(String(data=json.dumps(dict(common,op='execute'))));phase='executing'
   elif phase=='executing' and mission['status'] in ('completed','failed','interrupted'):
    phase=mission['status'];break
  if now-last>12.:
   last=now;print(json.dumps(dict(seconds=round(now-started,1),phase=phase,native_ready=native.get('ready') if native else None,mission=mission,localizations=len(localizations))),flush=True)
finally:
 if common:ctrl.publish(String(data=json.dumps(dict(common,op='stop'))))
 for _ in range(4):rclpy.spin_once(node,timeout_sec=.05)
 node.destroy_node();rclpy.shutdown()
 from robot_lab_gui.process_control import stop_owned_launch
 stop_owned_launch(proc,grace_timeout=25.)
 log.close();report=dict(folder=str(folder),phase=phase,target=target,mission=mission,native=native,source_database=str(database),source_hash_before=source_hash,source_hash_after=hashlib.sha256(database.read_bytes()).hexdigest(),localization_samples=len(localizations),returncode=proc.returncode)
 (folder/'report.json').write_text(json.dumps(report,indent=2));(folder/'states.json').write_text(json.dumps(states));(folder/'motion.json').write_text(json.dumps(dict(truth=truth,estimated=estimated,localizations=localizations)));print(json.dumps(dict(folder=str(folder),phase=phase,localizations=len(localizations))),flush=True)
