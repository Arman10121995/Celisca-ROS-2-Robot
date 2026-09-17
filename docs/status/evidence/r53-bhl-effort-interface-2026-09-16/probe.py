"""R5.3 effort-interface live probe: measure the wiring in a running sim.

Subscribes to ``/joint_states``, the IMU topic and the command topic, collects
for a fixed window and reports:

- measured publish rates for each topic (the command topic must be live);
- the maximum body tilt from the IMU quaternion (the 0.70 rad fall threshold
  is the failure bar);
- the published command magnitudes. On the effort interface the pre-command
  hold is **zero drive** (the node's target equals the measured pose, so
  ``tau = Kp*(q* - q) ~= 0``); on the position interface the same topic would
  carry ~0.4 rad knee values instead. This is what distinguishes the two
  interfaces on the wire, live.

Writes ``live_report.json`` next to the log and exits non-zero on a failed bar.
"""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray

COMMAND_TOPIC = "/bhl_standing_controller/commands"
JOINT_STATES_TOPIC = "/joint_states"
TILT_FALL_RAD = 0.70
WINDOW_S = 12.0


def _tilt(x: float, y: float, z: float, w: float) -> float:
    """Body tilt (rad) from the ZYX quaternion, as bhl_balance computes it."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        return 0.0
    sinp = 2.0 * (w * y - z * x) / (norm * norm)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    return max(abs(roll), abs(pitch))


class Probe(Node):
    def __init__(self, imu_topic: str) -> None:
        super().__init__("r53_effort_probe")
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.joint_stamps: list[float] = []
        self.command_stamps: list[float] = []
        self.imu_stamps: list[float] = []
        self.tilts: list[float] = []
        self.commands: list[list[float]] = []
        self.max_abs_joint_position = 0.0
        self.create_subscription(JointState, JOINT_STATES_TOPIC, self._on_joints, qos)
        self.create_subscription(Imu, imu_topic, self._on_imu, qos)
        self.create_subscription(
            Float64MultiArray, COMMAND_TOPIC, self._on_command, qos)

    def _on_joints(self, msg: JointState) -> None:
        self.joint_stamps.append(time.monotonic())
        for value in msg.position:
            self.max_abs_joint_position = max(
                self.max_abs_joint_position, abs(float(value)))

    def _on_imu(self, msg: Imu) -> None:
        self.imu_stamps.append(time.monotonic())
        q = msg.orientation
        self.tilts.append(_tilt(q.x, q.y, q.z, q.w))

    def _on_command(self, msg: Float64MultiArray) -> None:
        self.command_stamps.append(time.monotonic())
        self.commands.append([float(v) for v in msg.data])


def _rate(stamps: list[float]) -> float:
    if len(stamps) < 2:
        return 0.0
    span = stamps[-1] - stamps[0]
    return (len(stamps) - 1) / span if span > 0 else 0.0


def main(argv: list[str]) -> int:
    out_dir = Path(argv[1]) if len(argv) > 1 else Path(__file__).resolve().parent
    imu_topic = argv[2] if len(argv) > 2 else "/imu/out"

    rclpy.init()
    node = Probe(imu_topic)
    deadline = time.monotonic() + WINDOW_S
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    report = {
        "imu_topic": imu_topic,
        "window_s": WINDOW_S,
        "joint_states_hz": round(_rate(node.joint_stamps), 3),
        "imu_hz": round(_rate(node.imu_stamps), 3),
        "command_hz": round(_rate(node.command_stamps), 3),
        "command_messages": len(node.commands),
        "samples": {
            "joint_states": len(node.joint_stamps),
            "imu": len(node.imu_stamps),
        },
        "tilt_max_rad": round(max(node.tilts), 4) if node.tilts else None,
        "max_abs_command": (
            round(max(abs(v) for msg in node.commands for v in msg), 6)
            if node.commands else None
        ),
        "max_abs_joint_position_rad": round(node.max_abs_joint_position, 4),
    }
    report["checks"] = {
        "command_topic_live": report["command_hz"] > 1.0,
        "joint_states_live": report["joint_states_hz"] > 1.0,
        "imu_live": report["imu_hz"] > 1.0,
        # Effort interface: the pre-command hold is zero drive, so the command
        # topic must NOT carry the ~0.4 rad knee target the position path would.
        "command_is_effort_not_position": (
            report["max_abs_command"] is not None
            and report["max_abs_command"] < 0.05
        ),
        "no_fall": (
            report["tilt_max_rad"] is not None
            and report["tilt_max_rad"] < TILT_FALL_RAD
        ),
    }
    report["pass"] = all(report["checks"].values())

    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "live_report.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))

    node.destroy_node()
    rclpy.shutdown()
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
