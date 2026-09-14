"""Opt-in ROS smoke: imported Bumperbot drive, RGB-D, clock and live reset.

Run in a sourced workspace with ROBOT_LAB_RUN_MUJOCO_SMOKE=1 and an OpenGL
context (DISPLAY/Xvfb or MUJOCO_GL=egl). Only the test's child process is stopped.
"""

import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
import pytest

pytestmark = [
    pytest.mark.integration,
    pytest.mark.skipif(os.environ.get("ROBOT_LAB_RUN_MUJOCO_SMOKE") != "1",
                       reason="Opt in to the live MuJoCo ROS smoke"),
]


def test_mujoco_drive_camera_and_reset(tmp_path, monkeypatch):
    mujoco = pytest.importorskip("mujoco")
    rclpy = pytest.importorskip("rclpy")
    from ament_index_python.packages import get_package_share_directory as share
    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import CameraInfo, Image, LaserScan
    from std_msgs.msg import Bool
    from std_srvs.srv import Trigger

    # This test starts a simulated robot and communicates only on its own domain.
    domain = 173
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    context = Context()
    context.init(args=[], domain_id=domain)
    listener = rclpy.create_node("mujoco_runtime_probe", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(listener)
    latest, counts, clocks = {}, {}, []
    sensor_stamps = {"rgb": [], "odom": []}

    def stamp(msg):
        value = msg.clock if isinstance(msg, Clock) else msg.header.stamp
        return value.sec + value.nanosec * 1e-9

    def record(key, msg):
        latest[key] = msg
        counts[key] = counts.get(key, 0) + 1
        if key == "clock":
            clocks.append(stamp(msg))
        if key in sensor_stamps:
            sensor_stamps[key].append(stamp(msg))

    for key, msg_type, topic in (
            ("odom", Odometry, "/odom/ground_truth"),
            ("clock", Clock, "/clock"),
            ("scan", LaserScan, "/scan"),
            ("rgb", Image, "/oakd/rgb/image_raw"),
            ("depth", Image, "/oakd/depth/image_raw"),
            ("info", CameraInfo, "/oakd/rgb/camera_info"),
            ("ready", Bool, "/robot_lab/ready"),
            ("health", DiagnosticArray, "/robot_lab/health")):
        listener.create_subscription(msg_type, topic,
                                     lambda msg, key=key: record(key, msg), 10)
    publisher = listener.create_publisher(Twist, "/cmd_vel", 10)
    reset = listener.create_client(Trigger, "/robot_lab/reset")
    source = Path(__file__).resolve().parents[2]
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain))
    env["PYTHONPATH"] = os.pathsep.join([
        str(source / "robot_lab_mujoco" / "python"),
        str(source / "robot_lab_utils"), env.get("PYTHONPATH", "")])
    command = [sys.executable, "-c",
               "from robot_lab_mujoco.mujoco_spawner import main; main()", "--ros-args",
               "-p", "gui:=false", "-p", "spawn_yaw:=0.7",
               "-p", "model:=" + share("robot_lab_robots") + "/bumperbot/urdf/bumperbot.urdf.xacro",
               "-p", "world_xml:=" + share("robot_lab_maps") + "/mjcf/nav_empty.xml"]
    artifacts = Path(os.environ.get("ROBOT_LAB_MUJOCO_ARTIFACT_DIR", str(tmp_path)))
    artifacts.mkdir(parents=True, exist_ok=True)
    log_path = artifacts / "spawner.log"
    process = None
    try:
        with log_path.open("w") as log:
            process = subprocess.Popen(command, env=env, stdout=log,
                                       stderr=subprocess.STDOUT, start_new_session=True)

        def spin_until(condition, timeout=20.0, drive=None):
            deadline = time.monotonic() + timeout
            last_command = 0.0
            while not condition():
                assert process.poll() is None, log_path.read_text()
                assert time.monotonic() < deadline, (
                    f"Timed out; received {counts}\n{log_path.read_text()}")
                if drive is not None and time.monotonic() - last_command >= 0.05:
                    publisher.publish(drive)
                    last_command = time.monotonic()
                executor.spin_once(timeout_sec=0.02)

        spin_until(lambda: len(latest) == 8 and latest["ready"].data
                   and stamp(latest["odom"]) >= 0.5)
        status = latest["health"].status[0]
        assert {item.key: item.value for item in status.values}["model_source"] == "urdf"
        assert status.level == 0

        # Camera output must be real, metric and registered in the optical frame.
        spin_until(lambda: stamp(latest["rgb"]) == stamp(latest["depth"])
                   == stamp(latest["info"]))
        rgb, depth, info = (latest[key] for key in ("rgb", "depth", "info"))
        assert (rgb.width, rgb.height, rgb.encoding) == (320, 240, "rgb8")
        assert (depth.width, depth.height, depth.encoding) == (320, 240, "32FC1")
        assert rgb.header.frame_id == depth.header.frame_id == info.header.frame_id
        assert rgb.header.frame_id == "oakd_rgb_camera_optical_frame"
        assert len(rgb.data) == 320 * 240 * 3
        pixels = np.frombuffer(bytes(rgb.data), dtype=np.uint8)
        assert pixels.std() > 1.0
        ranges = np.frombuffer(bytes(depth.data), dtype=np.float32)
        finite = ranges[np.isfinite(ranges)]
        assert finite.size > ranges.size / 2
        assert finite.min() > 0.0
        assert info.k[0] == pytest.approx(160.0 / math.tan(1.25 / 2.0))
        assert sum(math.isfinite(r) for r in latest["scan"].ranges) > 0

        start = latest["odom"].pose.pose.position
        initial = (start.x, start.y)
        start_time = stamp(latest["odom"])
        drive = Twist()
        drive.linear.x = 0.2
        spin_until(lambda: stamp(latest["odom"]) >= start_time + 1.5,
                   timeout=30.0, drive=drive)
        pose = latest["odom"].pose.pose
        drive_duration = stamp(latest["odom"]) - start_time
        dx, dy = pose.position.x - initial[0], pose.position.y - initial[1]
        forward = dx * math.cos(0.7) + dy * math.sin(0.7)
        lateral = -dx * math.sin(0.7) + dy * math.cos(0.7)
        assert forward > 0.18, f"Only moved {forward:.3f} m forward"
        assert abs(lateral) < 0.06
        assert abs(pose.orientation.x) < 0.05 and abs(pose.orientation.y) < 0.05
        for key, interval in (("rgb", 0.2), ("odom", 0.02)):
            stamps = sensor_stamps[key]
            assert len(stamps) >= 2
            assert min(b - a for a, b in zip(stamps, stamps[1:])) >= interval - 1e-8

        # Reset while a nonzero command is still fresh and sensors are rendering.
        assert reset.wait_for_service(timeout_sec=3.0)
        before_reset = len(clocks)
        before_rgb = counts["rgb"]
        future = reset.call_async(Trigger.Request())
        spin_until(future.done, timeout=5.0)
        assert future.result().success, future.result().message
        spin_until(lambda: any(value < 0.1 for value in clocks[before_reset:]), timeout=5.0)
        spin_until(lambda: 0.6 <= stamp(latest["odom"]) < 1.5
                   and counts["rgb"] > before_rgb, timeout=20.0)
        after = latest["odom"]
        distance_after_reset = math.hypot(after.pose.pose.position.x,
                                         after.pose.pose.position.y)
        assert distance_after_reset < 0.03
        assert abs(after.twist.twist.linear.x) < 0.02
        assert all(b >= a for a, b in zip(clocks[:before_reset], clocks[1:before_reset]))
        metrics = {"mujoco_version": mujoco.__version__, "robot": "bumperbot",
                   "world": "nav_empty", "spawn_yaw_rad": 0.7,
                   "forward_distance_m": forward, "lateral_distance_m": lateral,
                   "drive_sim_seconds": drive_duration,
                   "reset_position_error_m": distance_after_reset,
                   "reset_speed_mps": after.twist.twist.linear.x,
                   "depth_finite_fraction": finite.size / ranges.size,
                   "depth_min_m": float(finite.min()), "received": counts}
        (artifacts / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n")
        print(json.dumps(metrics, sort_keys=True))
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3.0)
        executor.shutdown()
        listener.destroy_node()
        context.shutdown()
    assert process.returncode == 0, log_path.read_text()
