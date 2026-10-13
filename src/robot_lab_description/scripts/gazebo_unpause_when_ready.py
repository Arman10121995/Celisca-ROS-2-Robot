#!/usr/bin/env python3
"""Unpause a paused Gazebo world once a ros2_control controller is configured.

A legged robot spawned into a running world falls limp until its PD
controller is active: the spawner needs a few seconds after the model (and
its controller manager) appear.  The Go2 landed on its side in Gazebo
nav_obstacle and never stood up (2026-10-13).  Started paused, the world
does not step while the controller loads; this node waits until the
controller reports ``inactive`` (configured, its activation waiting for the
first update) or ``active``, then unpauses through the world control
service, so activation happens on the first physics step.  After
``timeout_s`` it unpauses anyway.
"""
import subprocess
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.node import Node


def main():
    rclpy.init()
    node = Node('gazebo_unpause_when_ready')
    node.declare_parameter('world_name', 'default')
    node.declare_parameter('controller', '')
    node.declare_parameter('controller_manager', '/controller_manager')
    node.declare_parameter('timeout_s', 60.0)
    world = node.get_parameter('world_name').value
    controller = node.get_parameter('controller').value
    client = node.create_client(ListControllers,
                                node.get_parameter('controller_manager').value + '/list_controllers')
    deadline = time.monotonic() + float(node.get_parameter('timeout_s').value)
    reason = 'timeout'
    while rclpy.ok() and time.monotonic() < deadline:
        if client.wait_for_service(timeout_sec=0.5):
            future = client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
            result = future.result()
            states = {c.name: c.state for c in (result.controller if result else [])}
            if states.get(controller) in ('inactive', 'active'):
                reason = '%s is %s' % (controller, states[controller])
                break
        time.sleep(0.2)
    command = ['ign', 'service', '-s', '/world/%s/control' % world,
               '--reqtype', 'ignition.msgs.WorldControl', '--reptype', 'ignition.msgs.Boolean',
               '--timeout', '5000', '--req', 'pause: false']
    done = subprocess.run(command, capture_output=True, text=True)
    node.get_logger().info('unpaused world %s (%s): %s' % (world, reason, (done.stdout or done.stderr).strip()))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
