"""The built-in Nav2 workload node must construct (2026-10-15 it did not:
its goal handle shadowed rclpy.Node.handle, so every queued Nav2 workload
died at start-up with "handle cannot be modified after node creation")."""
import os
import tempfile
from pathlib import Path

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('nav2_msgs')


def test_workload_node_constructs_and_records_its_start(monkeypatch):
    from robot_lab_benchmark.nav2_workload import Nav2Workload
    from robot_lab_benchmark.workload_contract import WORKLOAD_SCHEMA, validate_workload
    monkeypatch.setenv('ROS_DOMAIN_ID', os.environ.get('ROS_DOMAIN_ID', '87'))
    monkeypatch.setenv('ROBOT_LAB_EXPERIMENT_ID', 'unit-test')
    specification = validate_workload({'schema': WORKLOAD_SCHEMA, 'truth_coordinates': 'selected_world',
                                       'goals': [[1.0, 2.0, 0.5]]})
    rclpy.init()
    try:
        with tempfile.TemporaryDirectory() as folder:
            node = Nav2Workload(specification, Path(folder))
            assert node.goal_handle is None
            node.trace.close()
            node.destroy_node()
            first = (Path(folder)/'mission-trace.jsonl').read_text().splitlines()[0]
            assert '"workload_start"' in first and '"unit-test"' in first
    finally:
        rclpy.shutdown()


def test_an_accepted_goal_is_recorded(monkeypatch):
    """The accepted goal's uuid (a NumPy uint8 array) must reach the JSON trace."""
    from types import SimpleNamespace
    import numpy as np
    from robot_lab_benchmark.nav2_workload import Nav2Workload
    from robot_lab_benchmark.workload_contract import WORKLOAD_SCHEMA, validate_workload
    monkeypatch.setenv('ROS_DOMAIN_ID', os.environ.get('ROS_DOMAIN_ID', '87'))
    monkeypatch.setenv('ROBOT_LAB_EXPERIMENT_ID', 'unit-test')
    specification = validate_workload({'schema': WORKLOAD_SCHEMA, 'truth_coordinates': 'selected_world',
                                       'goals': [[1.0, 2.0, 0.5]]})
    rclpy.init()
    try:
        with tempfile.TemporaryDirectory() as folder:
            node = Nav2Workload(specification, Path(folder))
            result_future = SimpleNamespace(add_done_callback=lambda callback: None)
            handle = SimpleNamespace(accepted=True, goal_id=SimpleNamespace(uuid=np.arange(16, dtype=np.uint8)),
                                     get_result_async=lambda: result_future)
            node.accepted(SimpleNamespace(result=lambda: handle))
            assert node.state != 'failed', node.reason
            assert node.goal_handle is handle
            node.trace.close()
            node.destroy_node()
            assert '"goal_accepted"' in (Path(folder)/'mission-trace.jsonl').read_text()
    finally:
        rclpy.shutdown()
