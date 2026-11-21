"""Operator loss on the ported track plants (the October 27 tracked_loss_probe.py with a simulator argument).

Usage: LAB_EXTENSION_RUN=<dir> tracked_loss_probe.py <simulator>
Drives at 0.1 m/s through /key_vel, freezes the command owner (SIGSTOP) after
7 s of readiness and records truth for another 7 s. Passes when the body
travelled forward before the fault and moved < 5 mm in the final 2
simulator seconds (the plant's own 0.4 s watchdog must stop it).
"""
import json, math, os, signal, subprocess, sys, time
from pathlib import Path
import psutil
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from robot_lab_gui.process_control import stop_owned_launch
simulator = sys.argv[1]
folder = Path(os.environ['LAB_EXTENSION_RUN']) / ('tracked-loss-%s-%d' % (simulator, time.time_ns())); folder.mkdir(parents=True)
cmd = ['ros2', 'launch', 'robot_lab_bringup', 'simulated_robot.launch.py', 'robot_model:=lab_tracked', 'mode:=display', 'map_name:=nav_empty',
       'simulator:=' + simulator, 'gui:=false', 'start_rviz:=false', 'enable_joystick:=false']
(folder / 'command.json').write_text(json.dumps(cmd)); log = (folder / 'launch.log').open('w')
process = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
rclpy.init(); node = Node('tracked_loss_probe'); body = []; states = []
def pose(msg):
    p = msg.pose.pose.position; body.append([msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, p.x, p.y, p.z])
node.create_subscription(Odometry, '/odom/ground_truth', pose, qos_profile_sensor_data)
node.create_subscription(String, '/tracked/state', lambda m: states.append(json.loads(m.data)), 10)
pub = node.create_publisher(Twist, '/key_vel', 10); start = time.monotonic(); ready = None; paused = None; fault_at = None; fault_sim = None
try:
    while time.monotonic() - start < 120 and process.poll() is None:
        rclpy.spin_once(node, timeout_sec=.02)
        if states and states[-1]['ready']:
            ready = ready or time.monotonic(); msg = Twist(); msg.linear.x = .1; pub.publish(msg)
            if time.monotonic() - ready > 7 and paused is None:
                for child in psutil.Process(process.pid).children(recursive=True):
                    if any('tracked_control.py' in arg for arg in child.cmdline()):
                        paused = child.pid; os.kill(paused, signal.SIGSTOP); fault_at = time.monotonic(); fault_sim = body[-1][0]; break
        if ready and time.monotonic() - ready > 14: break
finally:
    if paused is not None:
        try: os.kill(paused, signal.SIGCONT)
        except ProcessLookupError: pass
    pub.publish(Twist()); node.destroy_node(); rclpy.shutdown(); stop_owned_launch(process, grace_timeout=15); log.close()
    def at(t):
        return min(body, key=lambda row: abs(row[0] - t))
    result = dict(simulator=simulator, samples=len(body), paused_controller_pid=paused)
    if body and fault_sim is not None:
        first, fault, last = body[0], at(fault_sim), body[-1]
        tail = at(last[0] - 2.)
        result.update(travel_before_fault_m=round(math.dist(first[1:3], fault[1:3]), 4),
                      travel_after_fault_m=round(math.dist(fault[1:3], last[1:3]), 4),
                      final_2s_displacement_m=round(math.dist(tail[1:3], last[1:3]), 4),
                      sim_after_fault_s=round(last[0] - fault[0], 3))
        result['passed'] = result['travel_before_fault_m'] > .1 and result['final_2s_displacement_m'] < .005 and result['sim_after_fault_s'] > 2.
    (folder / 'measurements.json').write_text(json.dumps(dict(result=result, body=body, states=states[-5:], fault_at=fault_at)))
    print(json.dumps(result), flush=True)
