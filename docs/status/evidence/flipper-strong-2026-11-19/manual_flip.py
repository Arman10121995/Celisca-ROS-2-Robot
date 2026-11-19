"""Manual flipper experiment: FRONT/REAR joint targets, drive forward, print truth."""
import json, math, os, sys, time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
rclpy.init(); node = rclpy.create_node('manual_flip')
state, body = {}, {}
def truth(m):
    p, q = m.pose.pose.position, m.pose.pose.orientation
    body.update(y=p.y, z=p.z, pitch=math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x)))))
node.create_subscription(String, '/tracked/state', lambda m: state.update(json.loads(m.data)), 10)
node.create_subscription(Odometry, '/odom/ground_truth', truth, qos_profile_sensor_data)
drive = node.create_publisher(Twist, '/key_vel', 10); flip = node.create_publisher(String, '/tracked/flipper_command', 10)
plan = [tuple(map(float, s.split(','))) for s in sys.argv[1:]]   # front,rear,seconds
while not (state and body): rclpy.spin_once(node, timeout_sec=.1)
for front, rear, seconds in plan:
    end = time.monotonic() + seconds; last = 0
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.05)
        flip.publish(String(data=json.dumps(dict(session_id=state['session_id'], robot_id=state['robot_id'], op='target', positions=[front, rear, front, rear]))))
        t = Twist(); t.linear.x = .15; drive.publish(t)
        if time.monotonic() - last > 1.5:
            last = time.monotonic(); print('f=%+.2f r=%+.2f y=%.3f z=%.3f p=%+.3f pos=%s' % (front, rear, body['y'], body['z'], body['pitch'], [round(v, 2) for v in state['flippers'].get('positions') or []]), flush=True)
drive.publish(Twist()); rclpy.spin_once(node, timeout_sec=.2)
