import json,os,subprocess,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from robot_lab_gui.process_control import stop_owned_launch
folder=Path(os.environ['LAB_EXTENSION_RUN'])/('tracked-'+str(time.time_ns()));folder.mkdir()
cmd=['ros2','launch','robot_lab_bringup','simulated_robot.launch.py','robot_model:=lab_tracked','mode:=display','map_name:=nav_empty','simulator:=gazebo','gui:=false','start_rviz:=false','enable_joystick:=false']
(folder/'command.json').write_text(json.dumps(cmd));log=(folder/'launch.log').open('w')
process=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init();node=Node('tracked_actual_probe');body=[];states=[]
def pose(msg):
 p=msg.pose.pose.position;body.append([msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,p.x,p.y,p.z])
node.create_subscription(Odometry,'/odom/ground_truth',pose,qos_profile_sensor_data)
node.create_subscription(String,'/tracked/state',lambda m:states.append(json.loads(m.data)),10)
pub=node.create_publisher(Twist,'/key_vel',10);start=time.monotonic();ready=None
try:
 while time.monotonic()-start<35 and process.poll() is None:
  rclpy.spin_once(node,timeout_sec=.02)
  if states and states[-1]['ready']:
   ready=ready or time.monotonic();msg=Twist();msg.linear.x=.1 if time.monotonic()-ready<10 else 0.;pub.publish(msg)
  if ready and time.monotonic()-ready>14:break
finally:
 pub.publish(Twist());node.destroy_node();rclpy.shutdown();stop_owned_launch(process,grace_timeout=15);log.close()
 (folder/'measurements.json').write_text(json.dumps(dict(body=body,states=states,returncode=process.returncode)))
 print(json.dumps(dict(folder=str(folder),samples=len(body),first=body[0] if body else None,last=body[-1] if body else None)),flush=True)
