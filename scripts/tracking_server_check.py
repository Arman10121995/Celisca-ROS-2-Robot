"""ROS-level check of the FollowPath tracking server against a kinematic fake robot.

Runs with wall time: a fake robot integrates /cmd_vel (ActuatorPlant) and
publishes map->base_footprint; the server under test is started separately.
Sends one L-shaped path, then a second path during which the fake robot's TF
stops (watchdog).  Prints one JSON line.

  export ROS_DOMAIN_ID=152   # keep it off any running simulation
  ros2 run robot_lab_algorithms nav2_local_planner --ros-args -p planner:=lqr -p use_sim_time:=false &
  python3 scripts/tracking_server_check.py
"""
import json
import math
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from robot_lab_algorithms.tracking_control import ActuatorPlant, Limits


class FakeRobot(Node):
    def __init__(self):
        super().__init__('fake_robot')
        self.plant = ActuatorPlant(limits=Limits(), pose=(0.0, 0.0, 0.0))
        self.command = (0.0, 0.0)
        self.publish_tf = True
        self.commands = []
        self.lock = threading.Lock()
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.tf = TransformBroadcaster(self)
        self.create_timer(0.02, self.step)

    def on_cmd(self, msg):
        with self.lock:
            self.command = (msg.linear.x, msg.angular.z)
            self.commands.append((time.time(), msg.linear.x, msg.angular.z))

    def step(self):
        with self.lock:
            pose = self.plant.step(self.command, 0.02)
        if not self.publish_tf:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = 'map', 'base_footprint'
        t.transform.translation.x, t.transform.translation.y = float(pose[0]), float(pose[1])
        t.transform.rotation.z, t.transform.rotation.w = math.sin(pose[2] / 2), math.cos(pose[2] / 2)
        self.tf.sendTransform(t)


def make_path(node, points):
    path = Path()
    path.header.frame_id = 'map'
    path.header.stamp = node.get_clock().now().to_msg()
    for i, (x, y) in enumerate(points):
        p = PoseStamped()
        p.header = path.header
        p.pose.position.x, p.pose.position.y = x, y
        if i + 1 < len(points):
            yaw = math.atan2(points[i + 1][1] - y, points[i + 1][0] - x)
        p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        path.poses.append(p)
    return path


def main():
    rclpy.init()
    robot = FakeRobot()
    client_node = Node('follow_path_client')
    client = ActionClient(client_node, FollowPath, '/robot_lab/follow_path')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(robot)
    executor.add_node(client_node)
    threading.Thread(target=executor.spin, daemon=True).start()
    report = {}
    if not client.wait_for_server(timeout_sec=30):
        print(json.dumps({'error': 'no server'}))
        return 1
    # 1) L-shaped path: 2 m east, then 1.5 m north (staircase corner like a grid planner).
    points = [(0.05 * i, 0.0) for i in range(41)] + [(2.0, 0.05 * i) for i in range(1, 31)]
    goal = FollowPath.Goal()
    goal.path = make_path(client_node, points)
    start = time.time()
    future = client.send_goal_async(goal)
    while not future.done():
        time.sleep(0.05)
    handle = future.result()
    result = handle.get_result_async()
    while not result.done() and time.time() - start < 60:
        time.sleep(0.05)
    pose = robot.plant.pose
    report['path'] = {'status': result.result().status if result.done() else 'timeout',
                      'seconds': round(time.time() - start, 1),
                      'final_xy': [round(float(pose[0]), 3), round(float(pose[1]), 3)],
                      'final_error_m': round(math.hypot(pose[0] - 2.0, pose[1] - 1.5), 3)}
    # 2) Watchdog: TF stops 3 s into a new path; commands must drop to zero and the goal abort.
    points = [(2.0 + 0.05 * i, 1.5) for i in range(61)]
    goal.path = make_path(client_node, points)
    start = time.time()
    future = client.send_goal_async(goal)
    while not future.done():
        time.sleep(0.05)
    result = future.result().get_result_async()
    time.sleep(3.0)
    robot.publish_tf = False
    stopped_at = time.time()
    while not result.done() and time.time() - start < 30:
        time.sleep(0.05)
    after = [c for c in robot.commands if c[0] > stopped_at + 0.8]
    report['watchdog'] = {'status': result.result().status if result.done() else 'timeout',
                          'abort_after_tf_loss_s': round(time.time() - stopped_at, 1),
                          'commands_after_0.8s': len(after),
                          'max_abs_command_after': max((abs(c[1]) + abs(c[2]) for c in after), default=0.0)}
    print(json.dumps(report))
    executor.shutdown()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
