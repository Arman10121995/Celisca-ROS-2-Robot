#!/usr/bin/env python3
"""Expose the shared Robot Lab reset service over Gazebo Sim world control."""

from __future__ import annotations

from threading import Event

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from ros_gz_interfaces.srv import ControlWorld
from std_srvs.srv import Trigger


class GazeboResetBridge(Node):
    """Bridge ``/robot_lab/reset`` to the selected Gazebo world's reset API."""

    def __init__(self) -> None:
        super().__init__("gazebo_reset_bridge")
        self.declare_parameter("world_name", "empty")
        world_name = str(self.get_parameter("world_name").value)
        self._group = ReentrantCallbackGroup()
        self._control = self.create_client(
            ControlWorld, f"/world/{world_name}/control",
            callback_group=self._group)
        self.create_service(
            Trigger, "/robot_lab/reset", self._reset,
            callback_group=self._group)
        self.get_logger().info(
            f"Gazebo reset bridge ready for world '{world_name}'")

    def _reset(self, _request, response):
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