"""Verify the ROS odometry bridge against physical pose displacements.

Kit is not started here. The explicit SDK state fixture reproduces the
measured contact-velocity bias; native estimator and braking trials remain
separate from this interface check.
"""
import math
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from robot_lab_isaac.isaac_spawner import IsaacSpawner


@pytest.mark.integration
def test_ground_truth_twist_matches_pose_in_the_child_frame(monkeypatch):
    owned = not rclpy.ok()
    if owned:
        monkeypatch.setenv('ROS_DOMAIN_ID', '225')
        rclpy.init(args=[])
    bridge = IsaacSpawner()
    bridge._timer.cancel()
    bridge._pub_timer.cancel()
    observer = rclpy.create_node('isaac_odometry_observer')
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(observer)
    received = []
    subscription = observer.create_subscription(
        Odometry, '/odom/ground_truth', received.append, 10)

    def pump_until(condition):
        end = time.monotonic() + 3.0
        while not condition() and time.monotonic() < end:
            executor.spin_once(timeout_sec=.01)
        assert condition()

    def publish(stamp, y):
        # The SDK's nonzero contact velocity is inconsistent with this
        # stationary pose. Only the native pose displacement is truth.
        bridge._state = dict(t=stamp, pos=[1.0, y, .033],
                             orn=[0, 0, math.sin(math.pi/4), math.cos(math.pi/4)],
                             lin=[.02, .03, 0], ang=[0, 0, .1])
        bridge._publish()
        pump_until(lambda: any(m.header.stamp.sec == stamp for m in received))
        return next(m for m in received if m.header.stamp.sec == stamp)

    try:
        pump_until(lambda: bridge._odom_pub.get_subscription_count() > 0)
        publish(1, 2.0)
        still = publish(2, 2.0)
        assert still.pose.pose.position.y == 2.0
        assert still.twist.twist.linear.x == 0.0
        assert still.twist.twist.linear.y == 0.0
        assert still.twist.twist.angular.z == 0.0
        moving = publish(3, 2.2)
        assert moving.child_frame_id == bridge._base_frame
        assert moving.twist.twist.linear.x == pytest.approx(.2)
        assert moving.twist.twist.linear.y == pytest.approx(0, abs=1e-12)
        assert moving.twist.twist.angular.z == 0.0
    finally:
        executor.shutdown()
        observer.destroy_node()
        bridge.destroy_node()
        if owned:
            rclpy.shutdown()
