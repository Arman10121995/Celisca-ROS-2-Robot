"""R10.4 validation: current, loss, reset qualification for the marine controller.

These unit tests exercise the MarineController logic (current compensation,
operator heartbeat/loss, world-clock rewind/reset) without requiring a Gazebo
runtime. They are separate from the sensor contracts in test_marine_contract.py
and test_marine_sensors.py.
"""
import math
import uuid
from pathlib import Path
import sys
import json
import rclpy
import pytest
from geometry_msgs.msg import Vector3
from rclpy.time import Time
from rosgraph_msgs.msg import Clock as ClockMessage
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/robot_lab_utils'))
from robot_lab_utils.marine_controller import MarineController


@pytest.fixture(scope='module', autouse=True)
def _ros_init():
    rclpy.init()
    yield
    rclpy.shutdown()


def _make_controller():
    node = MarineController.__new__(MarineController)
    node.session_id = uuid.uuid4().hex
    node.enabled = True
    node.operator = 'op_a'
    node.hb_at = -math.inf
    node.clock_value = None
    node.clock_at = -math.inf
    node.pose_at = -math.inf
    node.stop = lambda reason: None
    node.surface = 0.0
    node.maximum_depth = 5.4
    node.current = None
    node.current_stamp = -math.inf
    node.count = 0
    node.pose = None
    node.orientation = None
    node.velocity = None
    node.rates = None
    node.target_depth = None
    node.target_heading = None
    node.command = (0.0, 0.0, 0.0, 0.0)
    node.cmd_at = -math.inf
    return node


def test_current_compensation_stores_world_vector():
    """Current qualification: the controller stores a finite world-frame current vector."""
    node = _make_controller()
    msg = Vector3(); msg.x = 0.25; msg.y = -0.1; msg.z = 0.05
    node._current(msg)
    assert node.current is not None
    assert abs(node.current[0] - 0.25) < 1e-12
    assert abs(node.current[1] - (-0.1)) < 1e-12
    assert abs(node.current[2] - 0.05) < 1e-12


def test_current_rejects_nonfinite_values():
    """Current qualification: non-finite current messages are ignored."""
    node = _make_controller()
    node.current = None
    msg = Vector3(); msg.x = float('nan'); msg.y = 0.0; msg.z = 0.0
    node._current(msg)
    assert node.current is None


def test_heartbeat_operator_change_triggers_stop():
    """Loss qualification: an operator change triggers a neutral stop."""
    node = _make_controller()
    calls = []
    node.stop = lambda reason: calls.append(reason)
    msg = String()
    msg.data = json.dumps({'session_id': node.session_id, 'operator_id': 'op_b'})
    node._heartbeat(msg)
    assert len(calls) == 1
    assert 'Operator changed' in calls[0]
    assert node.operator == 'op_b'


def test_heartbeat_same_operator_is_accepted():
    """Loss qualification: a heartbeat from the current operator is accepted without stop."""
    node = _make_controller()
    calls = []
    node.stop = lambda reason: calls.append(reason)
    msg = String()
    msg.data = json.dumps({'session_id': node.session_id, 'operator_id': 'op_a'})
    node._heartbeat(msg)
    assert len(calls) == 0
    assert node.operator == 'op_a'


def test_clock_rewind_triggers_stop():
    """Reset qualification: a rewound world clock stops the controller session."""
    node = _make_controller()
    calls = []
    node.stop = lambda reason: calls.append(reason)
    node.clock_value = 100.0
    msg = ClockMessage()
    msg.clock = Time(seconds=50.0).to_msg()
    node._observe_clock(msg)
    assert len(calls) == 1
    assert 'clock rewound' in calls[0]


def test_clock_forward_is_accepted():
    """Reset qualification: a forward-moving world clock is accepted without stop."""
    node = _make_controller()
    calls = []
    node.stop = lambda reason: calls.append(reason)
    node.clock_value = 10.0
    msg = ClockMessage()
    msg.clock = Time(seconds=20.0).to_msg()
    node._observe_clock(msg)
    assert len(calls) == 0
    assert node.clock_value == 20.0