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

A controller switch (the joint_state_broadcaster's activation, for one)
completes only in the controller manager's update loop, which a paused
world never runs, and the manager's other services wait behind it.  Fully
paused, the spawners timed out and the Go2 never got its controller
(2026-10-13).  So while it waits, the node steps the paused world by
``step_iterations`` at a time (a few milliseconds of simulated time per
poll): the switches complete and the robot hardly moves.
"""
import subprocess
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from rclpy.node import Node


def _world_control(world, request):
    command = ['ign', 'service', '-s', '/world/%s/control' % world,
               '--reqtype', 'ignition.msgs.WorldControl', '--reptype', 'ignition.msgs.Boolean',
               '--timeout', '5000', '--req', request]
    done = subprocess.run(command, capture_output=True, text=True)
    return (done.stdout or done.stderr).strip()


def main():
    rclpy.init()
    node = Node('gazebo_unpause_when_ready')
    node.declare_parameter('world_name', 'default')
    node.declare_parameter('controller', '')
    node.declare_parameter('controller_manager', '/controller_manager')
    node.declare_parameter('timeout_s', 60.0)
    node.declare_parameter('step_iterations', 10)
    world = node.get_parameter('world_name').value
    controller = node.get_parameter('controller').value
    steps = int(node.get_parameter('step_iterations').value)
    client = node.create_client(ListControllers,
                                node.get_parameter('controller_manager').value + '/list_controllers')
    deadline = time.monotonic() + float(node.get_parameter('timeout_s').value)
    reason = 'timeout'
    stepped = 0
    while rclpy.ok() and time.monotonic() < deadline:
        if client.wait_for_service(timeout_sec=0.5):
            if steps > 0:
                _world_control(world, 'pause: true, multi_step: %d' % steps)
                stepped += steps
            future = client.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
            result = future.result() if future.done() else None
            states = {c.name: c.state for c in (result.controller if result else [])}
            if states.get(controller) in ('inactive', 'active'):
                reason = '%s is %s' % (controller, states[controller])
                break
        time.sleep(0.2)
    reply = _world_control(world, 'pause: false')
    node.get_logger().info('unpaused world %s (%s, %d iterations stepped while waiting): %s' % (
        world, reason, stepped, reply))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
