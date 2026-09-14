"""Spawn and reset the real MuJoCo robot state without stepping or rendering."""

import math
import itertools
from pathlib import Path
import sys
from unittest.mock import Mock, patch

import numpy as np
import pytest

pytest.importorskip("mujoco")
rclpy = pytest.importorskip("rclpy")
from ament_index_python.packages import get_package_share_directory as share
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

SRC = Path(__file__).resolve().parents[2]
for package in (SRC / "robot_lab_mujoco" / "python", SRC / "robot_lab_utils"):
    sys.path.insert(0, str(package))

from robot_lab_mujoco.mujoco_spawner import MuJoCoSpawner  # noqa: E402

pytestmark = pytest.mark.integration


@pytest.fixture
def node():
    rclpy.init(args=[
        "--ros-args", "-p", "gui:=false", "-p", "camera_rate:=0.0",
        "-p", "spawn_x:=1.0", "-p", "spawn_y:=-0.5",
        "-p", "spawn_z:=0.02", "-p", "spawn_yaw:=0.7",
        "-p", "model:=" + share("robot_lab_robots") + "/bumperbot/urdf/bumperbot.urdf.xacro",
        "-p", "world_xml:=" + share("robot_lab_maps") + "/mjcf/nav_empty.xml",
    ])
    spawner = MuJoCoSpawner()
    try:
        # Keep the imported model at the exact initial state for assertions.
        with patch.object(spawner, "_loop"):
            spawner._spawn()
            spawner._thread.join(timeout=2.0)
        assert spawner._model_source == "urdf"
        yield spawner
    finally:
        spawner.destroy_node()
        rclpy.shutdown()


def test_spawn_yaw_rotates_about_vertical_axis(node):
    adr = node._free_joint_qpos_adr
    np.testing.assert_allclose(node._data.qpos[adr:adr + 3], [1.0, -0.5, 0.02])
    np.testing.assert_allclose(node._data.qpos[adr + 3:adr + 7],
                               [math.cos(0.35), 0, 0, math.sin(0.35)])


def test_reset_restores_physics_clock_joints_and_clears_forces(node):
    initial = node._data.qpos.copy()
    node._data.time = 12.0
    node._data.qpos[:] += 0.1
    node._data.qvel[:] = 1.0
    node._data.ctrl[:] = 4.0
    node._data.qfrc_applied[:] = 2.0
    node._data.xfrc_applied[:] = 3.0
    command = Twist()
    command.linear.x = 0.3
    node._on_cmd(command)

    result = node._on_reset(Trigger.Request(), Trigger.Response())

    assert result.success, result.message
    assert node._data.time == 0.0
    assert node._sim_t == 0.0
    np.testing.assert_allclose(node._data.qpos, initial)
    for values in (node._data.qvel, node._data.ctrl, node._data.qfrc_applied,
                   node._data.xfrc_applied):
        assert np.all(values == 0)
    assert node._twist.linear.x == 0.0
    assert node._twist.angular.z == 0.0


def test_reset_updates_odometry_before_next_physics_tick(node):
    node._bpos = [9.0, 8.0, 7.0]
    node._born = [1.0, 0.0, 0.0, 0.0]
    node._blin = [1.0, 2.0, 3.0]
    node._bang = [0.0, 0.0, 2.0]
    assert node._on_reset(Trigger.Request(), Trigger.Response()).success
    with patch.object(node, "_odom_pub", Mock()) as publisher:
        node._pub_odom()
        odom = publisher.publish.call_args.args[0]
    position = odom.pose.pose.position
    np.testing.assert_allclose([position.x, position.y, position.z], [1.0, -0.5, 0.02])
    orientation = odom.pose.pose.orientation
    np.testing.assert_allclose([orientation.x, orientation.y, orientation.z, orientation.w],
                               [0, 0, math.sin(0.35), math.cos(0.35)])
    assert odom.twist.twist == Twist()
    assert odom.header.stamp.sec == odom.header.stamp.nanosec == 0


def test_slow_camera_does_not_trigger_on_every_physics_step(node):
    node._camera = {"rate": 5.0}
    frames = []

    def render():
        frames.append(node._sim_t)
        if node._sim_t >= 0.4:
            node._running = False

    # Simulate expensive rendering without requiring OpenGL or waiting in real
    # time. Each loop exceeds the camera's wall-time period; the sensor must
    # still wait for 0.2 simulated seconds between frames.
    wall_time = itertools.count(start=100.0, step=0.25)
    with patch("robot_lab_mujoco.mujoco_spawner.time.monotonic",
               side_effect=lambda: next(wall_time)), \
            patch.object(node, "_pub_camera", side_effect=render):
        node._loop()
    assert len(frames) == 3
    np.testing.assert_allclose(np.diff(frames), [0.2, 0.2], atol=1e-8)
