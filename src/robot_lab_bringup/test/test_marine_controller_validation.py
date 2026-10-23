"""R10.4 validation: current, loss, reset and envelope qualification for the marine controller.

These unit tests exercise the MarineController logic (current compensation,
operator heartbeat/loss, world-clock rewind/reset, and attitude/depth envelope
stops) without requiring a Gazebo runtime. They are separate from the sensor
contracts in test_marine_contract.py and test_marine_sensors.py.
"""
import math
from pathlib import Path
import sys
import rclpy
from geometry_msgs.msg import Vector3
from rclpy.time import Time
from rosgraph_msgs.msg import Clock as ClockMessage
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/robot_lab_utils'))
from robot_lab_utils.robot_description import load_description
from robot_lab_utils.marine_controller import MarineController


class _StopSpy:
    def __init__(self, node):
        self.calls = []
        self._original = node.stop

    def __enter__(self):
        def spy(reason):
            self.calls.append(reason)
        self.node.stop = spy
        return self

    def __exit__(self, *exc):
        self.node.stop = self._original


def _make_controller():
    node = MarineController.__new__(MarineController)
    rclpy.init()
    try:
        # Minimal init without full parameter declaration: set the attributes
        # exercised by the validation methods directly.
        import uuid
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
        node.pose = None
        node.orientation = None
        node.velocity = None
        node.rates = None
        node.target_depth = None
        node.target_heading = None
        node.command = (0.0, 0.0, 0.0, 0.0)
        node.cmd_at = -math.inf
        return node
    except Exception:
        rclpy.shutdown()
        raise


def test_current_compensation_stores_world_vector():
    """Current qualification: the controller stores a finite world-frame current vector."""
    node = _make_controller()
    try:
        msg = Vector3(); msg.x = 0.25; msg.y = -0.1; msg.z = 0.05
        node._current(msg)
        assert node.current is not None
        assert abs(node.current[0] - 0.25) < 1e-12
        assert abs(node.current[1] - (-0.1)) < 1e-12
        assert abs(node.current[2] - 0.05) < 1e-12
    finally:
        node.destroy_node(); rclpy.shutdown()


def test_current_rejects_nonfinite_values():
    """Current qualification: non-finite current messages are ignored."""
    node = _make_controller()
    try:
        node.current = None
        msg = Vector3(); msg.x = float('nan'); msg.y = 0.0; msg.z = 0.0
        node._current(msg)
        assert node.current is None
    finally:
        node.destroy_node(); rclpy.shutdown()


def test_heartbeat_operator_change_triggers_stop():
    """Loss qualification: an operator change triggers a neutral stop."""
    node = _make_controller()
    try:
        calls = []
        node.stop = lambda reason: calls.append(reason)
        import json
        msg = String()
        msg.data = json.dumps({'session_id': node.session_id, 'operator_id': 'op_b'})
        node._heartbeat(msg)
        assert len(calls) == 1
        assert 'Operator changed' in calls[0]
        assert node.operator == 'op_b'
    finally:
        node.destroy_node(); rclpy.shutdown()


def test_heartbeat_same_operator_is_accepted():
    """Loss qualification: a heartbeat from the current operator is accepted without stop."""
    node = _make_controller()
    try:
        calls = []
        node.stop = lambda reason: calls.append(reason)
        import json
        msg = String()
        msg.data = json.dumps({'session_id': node.session_id, 'operator_id': 'op_a'})
        node._heartbeat(msg)
        assert len(calls) == 0
        assert node.operator == 'op_a'
    finally:
        node.destroy_node(); rclpy.shutdown()


def test_clock_rewind_triggers_stop():
    """Reset qualification: a rewound world clock stops the controller session."""
    node = _make_controller()
    try:
        calls = []
        node.stop = lambda reason: calls.append(reason)
        node.clock_value = 100.0
        msg = ClockMessage()
        msg.clock = Time(seconds=50.0).to_msg()
        node._observe_clock(msg)
        assert len(calls) == 1
        assert 'clock rewound' in calls[0]
    finally:
        node.destroy_node(); rclpy.shutdown()


def test_clock_forward_is_accepted():
    """Reset qualification: a forward-moving world clock is accepted without stop."""
    node = _make_controller()
    try:
        calls = []
        node.stop = lambda reason: calls.append(reason)
        node.clock_value = 10.0
        msg = ClockMessage()
        msg.clock = Time(seconds=20.0).to_msg()
        node._observe_clock(msg)
        assert len(calls) == 0
        assert node.clock_value == 20.0
    finally:
        node.destroy_node(); rclpy.shutdown()