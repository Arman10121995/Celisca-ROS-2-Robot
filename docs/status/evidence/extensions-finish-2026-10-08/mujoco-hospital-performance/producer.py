import json,time,os
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan,Image
rclpy.init();n=Node('read_only_matched_clock_window');samples=[];counts={'scan':0,'depth':0}
def clock(m):samples.append([time.monotonic(),m.clock.sec+m.clock.nanosec*1e-9])
def count(k):counts[k]+=1
subs=[n.create_subscription(Clock,'/clock',clock,qos_profile_sensor_data),n.create_subscription(LaserScan,'/scan',lambda m:count('scan'),qos_profile_sensor_data),n.create_subscription(Image,'/oakd/depth/image_raw',lambda m:count('depth'),qos_profile_sensor_data)]
deadline=time.monotonic()+60
while not samples and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.1)
start=time.monotonic()
while time.monotonic()-start<15:rclpy.spin_once(n,timeout_sec=.1)
r=dict(scope='Read-only matched delivery window on normal Labbot/MuJoCo/hospital v4 navigation; no control changes, speed fix or general rate qualification.',domain=os.environ['ROS_DOMAIN_ID'],samples=samples,message_counts=counts)
if len(samples)>2:
    r['wall_s']=samples[-1][0]-samples[0][0];r['simulation_s']=samples[-1][1]-samples[0][1];r['real_time_factor']=r['simulation_s']/r['wall_s']
Path(__file__).with_name('report.json').write_text(json.dumps(r,indent=2)+'\n');print({k:v for k,v in r.items() if k!='samples'})
n.destroy_node();rclpy.shutdown()
