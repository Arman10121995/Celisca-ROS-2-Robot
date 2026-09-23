#!/usr/bin/env python3
"""Send the same NavigateToPose action as RViz; save success and measured motion."""
import json, math, os, signal, subprocess, sys, time
from pathlib import Path
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from robot_lab_gui.process_control import stop_group
sim, robot, arena, output = sys.argv[1:5]
out=Path(output);out.mkdir(parents=True,exist_ok=True)
cmd=['ros2','launch','robot_lab_bringup','simulated_robot.launch.py',f'simulator:={sim}',f'robot_model:={robot}',f'map_name:={arena}','mode:=nav','gui:=false','start_rviz:=false']
rclpy.init();node=rclpy.create_node('navigation_regression_probe',parameter_overrides=[Parameter('use_sim_time',value=True)])
poses=[];commands=[];feedback=[]
def odom(m):
 p=m.pose.pose.position;poses.append([m.header.stamp.sec+m.header.stamp.nanosec*1e-9,p.x,p.y,p.z])
node.create_subscription(Odometry,'/odom/ground_truth',odom,qos_profile_sensor_data)
node.create_subscription(Twist,'/cmd_vel',lambda m:commands.append([m.linear.x,m.angular.z]),10)
client=ActionClient(node,NavigateToPose,'/navigate_to_pose'); lifecycle=node.create_client(GetState,'/bt_navigator/get_state')
report={'command':cmd,'passed':False}
with (out/'launch.log').open('w') as log: process=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
def wait(future,seconds):
 deadline=time.monotonic()+seconds
 while not future.done() and process.poll() is None and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.05)
 if not future.done():raise TimeoutError('ROS operation timed out')
 return future.result()
try:
 deadline=time.monotonic()+180
 active=False
 while process.poll() is None and time.monotonic()<deadline:
  rclpy.spin_once(node,timeout_sec=.05)
  if lifecycle.service_is_ready():
   result=wait(lifecycle.call_async(GetState.Request()),5)
   if result.current_state.id==3:active=True;break
 if not active:raise TimeoutError('Navigator did not activate')
 report['navigator_active']=True
 goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=node.get_clock().now().to_msg()
 goal.pose.pose.position.x=float(sys.argv[5]) if len(sys.argv)>5 else 1.0
 goal.pose.pose.position.y=float(sys.argv[6]) if len(sys.argv)>6 else 0.0
 goal.pose.pose.orientation.w=1.0
 report['goal']=[goal.pose.pose.position.x,goal.pose.pose.position.y]
 handle=wait(client.send_goal_async(goal,feedback_callback=lambda m:feedback.append(m.feedback.distance_remaining)),10)
 report['accepted']=handle.accepted
 if not handle.accepted:raise RuntimeError('Goal rejected')
 result=wait(handle.get_result_async(),180)
 report['status']=result.status;report['passed']=result.status==4
except Exception as e:report['error']=str(e)
finally:
 stop_group(process.pid);process.wait(timeout=5)
 report.update(exit_code=process.returncode,pose_first=poses[0] if poses else None,pose_last=poses[-1] if poses else None,cmd_count=len(commands),nonzero_commands=sum(abs(a)+abs(b)>1e-6 for a,b in commands),feedback=feedback[-10:])
 (out/'report.json').write_text(json.dumps(report,indent=2)+'\n');(out/'poses.json').write_text(json.dumps(poses)+'\n')
 node.destroy_node();rclpy.shutdown();print(json.dumps(report,indent=2))
sys.exit(0 if report['passed'] else 1)
