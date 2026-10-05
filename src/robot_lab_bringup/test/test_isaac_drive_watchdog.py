"""ROS input loss must produce a zero SDK command without another Twist.

This tests the real ROS node/timer and serialized worker boundary. A recording
pipe and explicit clock fixture replace Kit; body braking is a native trial.
"""
import io
import json
import time
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip('rclpy')
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from robot_lab_isaac.isaac_spawner import IsaacSpawner


@pytest.mark.integration
@pytest.mark.parametrize('topic', ['/cmd_vel', '/robot_lab_controller/cmd_vel_unstamped'])
def test_diff_input_loss_stops_worker_without_another_message(topic, monkeypatch):
    owned = not rclpy.ok()
    if owned:
        monkeypatch.setenv('ROS_DOMAIN_ID', '225')
        rclpy.init(args=[])
    node = IsaacSpawner()
    node._timer.cancel()  # No Kit/physics process in this interface test.
    node._pub_timer.cancel()  # Never publish the clock fixture as body truth.
    node._state = {'t': 0.0}
    pipe = io.StringIO()
    node._proc = SimpleNamespace(stdin=pipe)
    client = rclpy.create_node('isaac_watchdog_input')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(client)
    pub = client.create_publisher(Twist, topic, 10)

    reference = time.monotonic()

    def wait(condition, timeout=2.0, advance=True):
        end = time.monotonic() + timeout
        while not condition() and time.monotonic() < end:
            if advance:
                node._state = {'t': time.monotonic() - reference}
            executor.spin_once(timeout_sec=0.01)
        assert condition(), pipe.getvalue()

    def packets():
        return [json.loads(line) for line in pipe.getvalue().splitlines()]

    try:
        wait(lambda: pub.get_subscription_count() > 0, advance=False)
        msg = Twist()
        msg.linear.x, msg.angular.z = 0.2, 0.4
        pub.publish(msg)
        wait(lambda: node._last_cmd_time > 0.0, advance=False)
        received = time.monotonic()
        wait(lambda: time.monotonic() - received > 0.15, advance=False)
        assert all(value == 0.0 for packet in packets() for value in
                   packet.get('joint_targets', {}).get('velocity', {}).values())
        wait(lambda: any(any(abs(v) > 0.1 for v in
                            packet.get('joint_targets', {}).get('velocity', {}).values())
                         for packet in packets()))
        sent = node._last_cmd_time
        # Spin the actual wall timer; no Stop message or simulated clock.
        wait(lambda: time.monotonic() - sent > 0.65)
        target = packets()[-1]['joint_targets']['velocity']
        assert set(target) == {'wheel_left_joint', 'wheel_right_joint'}
        assert all(value == 0.0 for value in target.values())
        assert node._last_cmd_time == sent
    finally:
        node._proc = None
        executor.shutdown()
        client.destroy_node()
        node.destroy_node()
        if owned:
            rclpy.shutdown()
