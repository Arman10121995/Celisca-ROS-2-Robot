import hashlib,json,os,signal,subprocess,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image,CameraInfo,PointCloud2
from geometry_msgs.msg import Twist

base=Path(os.environ['LAB_EXTENSION_RUN'])
folder=base/('marine-slam-'+time.strftime('%H%M%S'));folder.mkdir()
root=Path('/workspace/molar/ros_ws/bumperbot_ws')
files=list((root/'src/robot_lab_utils/robot_lab_utils').glob('marine_*.py'))+[root/'src/robot_lab_utils/robot_lab_utils/map_archive.py']
(folder/'sources.json').write_text(json.dumps({str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},indent=2))
command=['ros2','launch','robot_lab_bringup','simulated_robot.launch.py','robot_model:=lab_rov',
 'mode:=marine','simulator:=gazebo','map_name:=marine_tank','marine_slam:=true','gui:=false','start_rviz:=false','enable_joystick:=false']
(folder/'command.json').write_text(json.dumps(command));log=(folder/'launch.log').open('w')
process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init();node=Node('marine_slam_diagnostic');native=None;mission=None;frames={};counts={};states=[]
def observe(msg):
 global native
 native=json.loads(msg.data)
def observe_mission(msg):
 global mission
 mission=json.loads(msg.data);states.append(mission)
def image(msg,topic):frames[topic]=msg.header.frame_id;counts[topic]=counts.get(topic,0)+1
node.create_subscription(String,'/marine/state',observe,10);node.create_subscription(String,'/marine/mission/state',observe_mission,10)
for kind,topic in [(Image,'/oakd/rgb/image_raw'),(Image,'/marine/camera/rgb'),(CameraInfo,'/marine/camera/info'),(PointCloud2,'/marine/slam/cloud_map')]:
 node.create_subscription(kind,topic,lambda msg,t=topic:image(msg,t),10)
hb=node.create_publisher(String,'/marine/operator_heartbeat',10);ctrl=node.create_publisher(String,'/marine/command',10)
drive=node.create_publisher(Twist,'/marine/key_vel',10);task=node.create_publisher(String,'/marine/mission/command',10)
started=time.monotonic();phase='waiting';sent=0.;saved=False
try:
 while time.monotonic()-started<55 and process.poll() is None:
  rclpy.spin_once(node,timeout_sec=.03);now=time.monotonic()
  if native:
   common=dict(session_id=native['session_id'],operator_id='marine-slam-diagnostic')
   hb.publish(String(data=json.dumps(common)))
   if phase=='waiting' and native['ready'] and native.get('operator_id')==common['operator_id']:
    ctrl.publish(String(data=json.dumps(dict(common,op='arm'))));phase='driving';sent=now
   elif phase=='driving' and native['enabled']:
    msg=Twist();msg.linear.x=.15 if now-sent<20 else 0.;drive.publish(msg)
    if now-sent>24:
     task.publish(String(data=json.dumps(dict(common,op='save_map'))));phase='saving';sent=now
   elif phase=='saving' and mission and 'Saved actual database' in mission['reason']:
    saved=True;break
finally:
 if native:ctrl.publish(String(data=json.dumps(dict(common,op='stop'))))
 for _ in range(4):rclpy.spin_once(node,timeout_sec=.05)
 node.destroy_node();rclpy.shutdown()
 from robot_lab_gui.process_control import stop_owned_launch
 stop_owned_launch(process,grace_timeout=25.)
 log.close()
 report=dict(folder=str(folder),counts=counts,frames=frames,mission=mission,native=native,saved=saved,returncode=process.returncode)
 (folder/'report.json').write_text(json.dumps(report,indent=2));(folder/'states.json').write_text(json.dumps(states));print(json.dumps(report),flush=True)
