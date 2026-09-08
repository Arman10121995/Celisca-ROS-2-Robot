# Copyright 2026 Bumperbot contributors
# Licensed under the Apache License, Version 2.0

"""R2.3 - Readiness, health and reset contract tests.

Verifies that the simulator adapters expose:
  * A /robot_lab/ready topic (std_msgs/Bool) published when ready.
  * A /robot_lab/health topic (diagnostic_msgs/DiagnosticArray) with status.
  * A /robot_lab/reset service (std_srvs/Trigger) for simulation reset.
  * Offline fallback is a diagnostic mode (Isaac), never successful physics.
"""

import threading
import time

import pytest
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus


def _importable(module_name):
    import importlib.util
    return importlib.util.find_spec(module_name) is not None


def test_pybullet_declares_readiness_contract():
    """PyBullet spawner must declare /robot_lab/ready, /robot_lab/health, /robot_lab/reset."""
    if not _importable("pybullet"):
        pytest.skip("pybullet not installed")
    if not _importable("rclpy"):
        pytest.skip("rclpy not importable")

    import rclpy
    from robot_lab_pybullet.pybullet_spawner import PyBulletSpawner

    rclpy.init()
    try:
        node = PyBulletSpawner()
        topics = [p.topic_name for p in node.publishers]
        assert "/robot_lab/ready" in topics, (
            "PyBullet missing /robot_lab/ready. Has: %s" % topics)
        assert "/robot_lab/health" in topics, (
            "PyBullet missing /robot_lab/health. Has: %s" % topics)
        services = [s.srv_name for s in node.services]
        assert "/robot_lab/reset" in services, (
            "PyBullet missing /robot_lab/reset. Has: %s" % services)
        node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_mujoco_declares_readiness_contract():
    """MuJoCo spawner must declare /robot_lab/ready, /robot_lab/health, /robot_lab/reset."""
    if not _importable("mujoco"):
        pytest.skip("mujoco not installed")
    if not _importable("rclpy"):
        pytest.skip("rclpy not importable")

    import rclpy
    from robot_lab_mujoco.mujoco_spawner import MuJoCoSpawner

    rclpy.init()
    try:
        node = MuJoCoSpawner()
        topics = [p.topic_name for p in node.publishers]
        assert "/robot_lab/ready" in topics, (
            "MuJoCo missing /robot_lab/ready. Has: %s" % topics)
        assert "/robot_lab/health" in topics, (
            "MuJoCo missing /robot_lab/health. Has: %s" % topics)
        services = [s.srv_name for s in node.services]
        assert "/robot_lab/reset" in services, (
            "MuJoCo missing /robot_lab/reset. Has: %s" % services)
        node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_isaac_declares_readiness_contract():
    """Isaac spawner must declare /robot_lab/ready, /robot_lab/health, /robot_lab/reset."""
    if not _importable("rclpy"):
        pytest.skip("rclpy not importable")

    import rclpy
    from robot_lab_isaac.isaac_spawner import IsaacSpawner

    rclpy.init()
    try:
        node = IsaacSpawner()
        topics = [p.topic_name for p in node.publishers]
        assert "/robot_lab/ready" in topics, (
            "Isaac missing /robot_lab/ready. Has: %s" % topics)
        assert "/robot_lab/health" in topics, (
            "Isaac missing /robot_lab/health. Has: %s" % topics)
        services = [s.srv_name for s in node.services]
        assert "/robot_lab/reset" in services, (
            "Isaac missing /robot_lab/reset. Has: %s" % services)
        node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_isaac_health_reports_offline_mode():
    """Isaac spawner in offline mode must report WARN health status."""
    if not _importable("rclpy"):
        pytest.skip("rclpy not importable")

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from robot_lab_isaac.isaac_spawner import IsaacSpawner

    rclpy.init()
    try:
        node = IsaacSpawner()
        health_msgs = []
        sub_node = rclpy.create_node("health_listener")
        sub_node.create_subscription(
            DiagnosticArray, "/robot_lab/health",
            lambda m: health_msgs.append(m), 10)

        exe = SingleThreadedExecutor()
        exe.add_node(node)
        exe.add_node(sub_node)
        t = threading.Thread(target=exe.spin, daemon=True)
        t.start()

        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and len(health_msgs) < 1:
            time.sleep(0.05)

        exe.shutdown()
        node.destroy_node()
        sub_node.destroy_node()

        assert len(health_msgs) >= 1, "No health messages received"
        status = health_msgs[0].status[0]
        assert status.level == DiagnosticStatus.WARN, (
            "Expected WARN for offline mode, got %s: %s" % (
                status.level, status.message))
    finally:
        if rclpy.ok():
            rclpy.shutdown()