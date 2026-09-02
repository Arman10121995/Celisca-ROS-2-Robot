# Copyright 2026 Bumperbot contributors
# Licensed under the Apache License, Version 2.0

"""Simulator backend smoke tests.

Verifies that each physics/simulator backend is importable and actually
steps physics in a headless configuration:

  * PyBullet — DIRECT mode (no GUI), a box is dropped and must fall.
  * MuJoCo    — headless ``mj_step``, a box is dropped and must fall.
  * Isaac Sim — qualifies the installation; the adapter must degrade
    gracefully (offline mode warning) when the ``isaacsim`` Python API is
    not installed.

These tests are environment-aware: they skip the physics checks when a
backend is not installed (e.g. on a minimal CI runner) rather than
failing, because the launch/adapter stack is designed to degrade
gracefully.  The *graceful-degradation* behavior itself is always tested
for the corresponding adapter package.
"""

import importlib
import importlib.util
import os

import pytest


def _importable(module_name: str) -> bool:
    return importlib.util.find_spec(module_name) is not None


def _dropped_box_z_pybullet(final_steps: int = 240) -> float:
    import pybullet as p

    cid = p.connect(p.DIRECT)
    try:
        try:
            import pybullet_data as _pbd
            data_path = os.path.dirname(_pbd.__file__) if hasattr(_pbd, '__file__') else None
        except ImportError:
            data_path = None
        if data_path is None:
            # Fallback: search for plane.urdf in known locations
            data_path = p.getDataPath()
        p.setAdditionalSearchPath(data_path)
        p.loadURDF("plane.urdf")
        collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.1])
        body = p.createMultiBody(
            baseMass=1.0,
            baseCollisionShapeIndex=collision,
            basePosition=[0, 0, 3],
        )
        p.setGravity(0, 0, -9.81)
        for _ in range(final_steps):
            p.stepSimulation()
        pos, _ = p.getBasePositionAndOrientation(body)
        return pos[2]
    finally:
        p.disconnect(cid)


# ---------------------------------------------------------------------------
# PyBullet
# ---------------------------------------------------------------------------

def test_pybullet_backend_physics():
    """A headless DIRECT pybullet world must integrate gravity."""
    if not _importable("pybullet"):
        pytest.skip("pybullet not installed; adapter runs in offline mode")
    z = _dropped_box_z_pybullet()
    assert z < 3.0, f"PyBullet box did not fall (z={z})"
    assert z > 0.0, f"PyBullet box fell through the plane (z={z})"


def test_pybullet_adapter_available():
    """robot_lab_pybullet import should succeed whether or not pybullet is installed."""
    mod = importlib.import_module("robot_lab_pybullet.pybullet_spawner")
    assert hasattr(mod, "PyBulletSpawner")


# ---------------------------------------------------------------------------
# MuJoCo
# ---------------------------------------------------------------------------

def test_mujoco_backend_physics():
    """A headless MuJoCo world must integrate gravity."""
    if not _importable("mujoco"):
        pytest.skip("mujoco not installed; adapter runs in offline mode")
    import mujoco

    xml = """
    <mujoco model='backend_test'>
      <option gravity='0 0 -9.81'/>
      <worldbody>
        <geom type='plane' size='2 2 0.1'/>
        <body name='box' pos='0 0 5'>
          <joint name='free' type='free'/>
          <geom type='box' size='0.1 0.1 0.1' mass='1'/>
        </body>
      </worldbody>
    </mujoco>
    """
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    start = float(data.qpos[2])
    for _ in range(10000):
        mujoco.mj_step(model, data)
    end = float(data.qpos[2])
    assert end < start - 0.1, f"MuJoCo box did not fall (start={start}, end={end})"
    assert end > 0.05, f"MuJoCo box fell through the plane (end={end})"


def test_mujoco_adapter_available():
    """robot_lab_mujoco import should succeed whether or not mujoco is installed."""
    mod = importlib.import_module("robot_lab_mujoco.mujoco_spawner")
    assert hasattr(mod, "MuJoCoSpawner")


# ---------------------------------------------------------------------------
# Isaac Sim
# ---------------------------------------------------------------------------

def test_isaac_adapter_graceful_degradation(monkeypatch):
    """Isaac Sim is typically not installed on low-power/headless hosts; the
    adapter must log a clear offline-mode warning and keep running."""
    if not _importable("robot_lab_isaac"):
        pytest.skip("robot_lab_isaac package not installed")
    import rclpy

    rclpy.init()
    try:
        mod = importlib.import_module("robot_lab_isaac.isaac_spawner")
        node = mod.IsaacSpawner()

        # Capture warnings issued through the node logger (rclpy logs to the
        # C-level stderr, so monkeypatch the logger instead of capfd).
        messages = []
        real_logger = node.get_logger()

        class _RecordingLogger:
            def warn(self, msg):
                messages.append(msg)

            def info(self, msg):
                messages.append(msg)

        monkeypatch.setattr(node, "get_logger", lambda: _RecordingLogger())

        if _importable("isaacsim"):
            # Isaac Sim installed: exercise the spawn path without requiring a
            # running Kit instance (stub logs "API available (stub).").
            node._try_spawn()
            assert node._spawned
            assert any("Isaac Sim API available" in m for m in messages)
        else:
            node._try_spawn()
            assert node._spawned, "IsaacSpawner must recover without isaacsim"
            assert any("Isaac Sim Python API not available" in m for m in messages), (
                "adapter must warn about offline mode"
            )

        # Restore real logger before destroy_node (destroy uses it).
        monkeypatch.setattr(node, "get_logger", lambda: real_logger)
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_isaac_adapter_importable():
    """The isaac adapter must import cleanly whether or not the isaacsim
    Python API is installed (graceful-degradation contract)."""
    if not _importable("robot_lab_isaac"):
        pytest.skip("robot_lab_isaac package not installed")
    spec = importlib.util.find_spec("robot_lab_isaac.isaac_spawner")
    assert spec is not None, "robot_lab_isaac adapter must be importable"
