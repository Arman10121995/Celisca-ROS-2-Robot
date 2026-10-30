"""R10.7 validation: tracked control input, safety, and state logic.

These unit tests exercise the TrackedControl logic (input validation, command
capping, navigation/manual ownership, clock rewind, IMU tilt readiness)
without requiring a Gazebo runtime.
"""
import math
import time
from types import SimpleNamespace
import pytest

from robot_lab_utils.tracked_control import TrackedControl


def _make_controller(navigation=False):
    node = TrackedControl.__new__(TrackedControl)
    node.navigation = navigation
    node.articulated = False
    node.robot_id = 'lab_tracked'
    node.session = 'test-session'
    node.flipper_positions = None
    node.flipper_output = [0.] * 4
    node.flipper_requested = [0.] * 4
    node.flipper_at = -math.inf
    node.command = [0., 0.]
    node.actual = [0., 0.]
    node.input_at = -math.inf
    node.odom_at = -math.inf
    node.clock_at = -math.inf
    node.imu_at = -math.inf
    node.manual_at = -math.inf
    node.clock = None
    node.tilt = None
    node.owner = 'idle'
    node.last = time.monotonic()
    node.get_parameter = lambda key: SimpleNamespace(value=navigation if key == 'navigation' else None)
    node.publisher = SimpleNamespace(publish=lambda msg: None)
    node.state = SimpleNamespace(publish=lambda msg: None)
    return node


def test_input_caps_linear_and_angular():
    node = _make_controller()
    from geometry_msgs.msg import Twist
    msg = Twist()
    msg.linear.x = 10.0
    msg.angular.z = 10.0
    node.input(msg, False)
    assert node.command == [0.3, 0.5]


def test_input_rejects_nonfinite():
    node = _make_controller()
    from geometry_msgs.msg import Twist
    msg = Twist()
    msg.linear.x = float('nan')
    msg.angular.z = 0.5
    node.input(msg, False)
    assert node.command == [0., 0.]


def test_input_rejects_lateral_motion():
    node = _make_controller()
    from geometry_msgs.msg import Twist
    msg = Twist()
    msg.linear.x = 0.2
    msg.linear.y = 0.1
    msg.angular.z = 0.3
    node.input(msg, False)
    assert node.command == [0., 0.]


def test_input_manual_sets_owner_and_timestamp():
    node = _make_controller()
    from geometry_msgs.msg import Twist
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = 0.3
    node.input(msg, False)
    assert node.owner == 'manual'
    assert node.input_at > -math.inf
    assert node.manual_at > -math.inf


def test_input_navigation_blocked_without_navigation_param():
    node = _make_controller(navigation=False)
    from geometry_msgs.msg import Twist
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = 0.3
    node.input(msg, True)
    assert node.command == [0., 0.]


def test_input_navigation_blocked_during_manual_override():
    node = _make_controller(navigation=True)
    from geometry_msgs.msg import Twist
    manual = Twist()
    manual.linear.x = 0.1
    manual.angular.z = 0.2
    node.input(manual, False)
    nav = Twist()
    nav.linear.x = 0.3
    nav.angular.z = 0.4
    node.input(nav, True)
    assert node.command == [0.1, 0.2]


def test_observe_clock_rewind_resets_input():
    node = _make_controller()
    node.clock = 10.0
    node.command = [0.2, 0.3]
    node.input_at = 5.0
    from rosgraph_msgs.msg import Clock as ClockMessage
    from rclpy.time import Time
    msg = ClockMessage()
    msg.clock = Time(seconds=5.0).to_msg()
    node.observe_clock(msg)
    assert node.command == [0., 0.]
    assert node.input_at == -math.inf


def test_imu_sets_tilt():
    node = _make_controller()
    from sensor_msgs.msg import Imu
    msg = Imu()
    msg.orientation.x = 0.
    msg.orientation.y = 0.
    msg.orientation.z = 0.
    msg.orientation.w = 1.
    node.imu(msg)
    assert node.tilt is not None
    assert abs(node.tilt) < 1e-12


def test_imu_rejects_nonfinite():
    node = _make_controller()
    from sensor_msgs.msg import Imu
    msg = Imu()
    msg.orientation.x = float('nan')
    msg.orientation.y = 0.
    msg.orientation.z = 0.
    msg.orientation.w = 1.
    node.imu(msg)
    assert node.tilt is None


def test_odometry_sets_timestamp():
    node = _make_controller()
    from nav_msgs.msg import Odometry
    msg = Odometry()
    msg.header.frame_id = 'odom'
    msg.child_frame_id = 'base_footprint'
    node.odometry(msg)
    assert node.odom_at > -math.inf


def test_tick_zeros_when_not_ready():
    node = _make_controller()
    node.last = time.monotonic() - 0.02
    node.tick()
    assert node.actual == [0., 0.]
