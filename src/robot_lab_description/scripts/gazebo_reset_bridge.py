#!/usr/bin/env python3
"""Expose the shared Robot Lab reset service over Gazebo Sim world control."""

from __future__ import annotations

from threading import Event
import math

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ros_gz_interfaces.srv import ControlWorld, SetEntityPose
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from robot_lab_utils.reset_notifications import ResetNotifications


class GazeboResetBridge(Node):
    """Reset the selected robot without rewinding the Gazebo world clock."""

    def __init__(self) -> None:
        super().__init__("gazebo_reset_bridge")
        self.declare_parameter("world_name", "empty")
        for name, value in [('robot_name', ''), ('spawn_x', 0.0),
                            ('spawn_y', 0.0), ('spawn_yaw', 0.0)]:
            self.declare_parameter(name, value)
        world_name = str(self.get_parameter("world_name").value)
        self._group = ReentrantCallbackGroup()
        self._control = self.create_client(
            ControlWorld, f"/world/{world_name}/control",
            callback_group=self._group)
        self._set_pose = self.create_client(SetEntityPose, f'/world/{world_name}/set_pose',
                                          callback_group=self._group)
        self._control_reset = self.create_client(Trigger, '/robot_lab/control_reset', callback_group=self._group)
        self._stop_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._initial_pose = None
        self._stable_since = None
        self.create_subscription(Odometry, '/odom/ground_truth', self._observe_spawn, 10,
                                 callback_group=self._group)
        self._notifications = ResetNotifications(self)
        self.create_service(
            Trigger, "/robot_lab/reset", self._reset,
            callback_group=self._group)
        self.get_logger().info(
            f"Gazebo reset bridge ready for world '{world_name}'")

    def _observe_spawn(self, msg):
        if self._initial_pose is not None:
            return
        p, velocity = msg.pose.pose.position, msg.twist.twist
        near = math.hypot(p.x-self.get_parameter('spawn_x').value,
                          p.y-self.get_parameter('spawn_y').value) < 0.03
        speed = math.sqrt(sum(getattr(velocity.linear,a)**2 for a in 'xyz'))
        angular = math.sqrt(sum(getattr(velocity.angular,a)**2 for a in 'xyz'))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        if near and speed < 0.02 and angular < 0.02:
            if self._stable_since is None:
                self._stable_since = stamp
            elif stamp-self._stable_since >= 0.2:
                self._initial_pose = msg.pose.pose
        else:
            self._stable_since = None

    @staticmethod
    def _wait(future, timeout):
        completed = Event()
        future.add_done_callback(lambda _future: completed.set())
        if not completed.wait(timeout):
            return None
        try:
            return future.result()
        except Exception:
            return None

    def _reset(self, _request, response):
        if self.get_parameter('robot_name').value:
            # Fortress does not implement model_only world reset. A full
            # world rewind invalidates ROS timestamps and controller state.
            # Restore the actual settled model pose without rewinding time.
            if self._initial_pose is None or not self._set_pose.wait_for_service(timeout_sec=5):
                response.success = False
                response.message = 'No settled spawn pose or Gazebo set_pose service available'
                return response
            self._stop_pub.publish(Twist())
            if self._control_reset.service_is_ready():
                stopped = self._control_reset.call_async(Trigger.Request())
                result = self._wait(stopped, 5)
                if result is None or not result.success:
                    response.success = False
                    response.message = 'Drive controller reset did not complete'
                    return response
            request = SetEntityPose.Request()
            request.entity.name = self.get_parameter('robot_name').value
            request.entity.type = request.entity.MODEL
            request.pose = self._initial_pose
            future = self._set_pose.call_async(request)
            result = self._wait(future, 10)
            response.success = bool(result and result.success)
            response.message = ('Gazebo robot pose/control reset; world clock preserved' if response.success
                                else 'Gazebo robot pose reset failed')
            if response.success:
                self._notifications.notify()
            return response
        if not self._control.wait_for_service(timeout_sec=5.0):
            response.success = False
            response.message = "Gazebo world control service is unavailable"
            return response

        request = ControlWorld.Request()
        request.world_control.reset.all = True
        future = self._control.call_async(request)
        completed = Event()
        future.add_done_callback(lambda _future: completed.set())
        if not completed.wait(timeout=10.0):
            response.success = False
            response.message = "Gazebo world reset timed out"
            return response

        try:
            result = future.result()
        except Exception as exc:
            response.success = False
            response.message = f"Gazebo world reset failed: {exc}"
            return response
        response.success = bool(result and result.success)
        response.message = ("Gazebo world reset" if response.success
                            else "Gazebo rejected the world reset")
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboResetBridge()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
