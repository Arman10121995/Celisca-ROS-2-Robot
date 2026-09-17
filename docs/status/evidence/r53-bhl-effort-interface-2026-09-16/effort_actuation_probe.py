"""R5.3 effort-interface live actuation probe.

Decisive check for the effort control path: publish a *known torque* to two
knee joints on ``/bhl_standing_controller/commands`` and verify the joints
actually move, and move in opposite directions when the torque sign flips.

This is the effort-path counterpart of the earlier position-path actuation
probe: it proves the effort command interface is honoured by the
``effort_controllers/JointGroupEffortController`` in gz-sim, independently of
whether the balance law can hold the biped up.

Writes ``actuation_report.json`` and exits non-zero on a failed bar.
"""

from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

JOINT_STATES_TOPIC = "/joint_states"
COMMAND_TOPIC = "/bhl_standing_controller/commands"
KNEE_JOINTS = ("leg_left_knee_pitch_joint", "leg_right_knee_pitch_joint")
#: The 22-joint canonical order the controller maps positionally.
JOINT_ORDER = (
    "leg_left_hip_roll_joint", "leg_left_hip_yaw_joint",
    "leg_left_hip_pitch_joint", "leg_left_knee_pitch_joint",
    "leg_left_ankle_pitch_joint", "leg_left_ankle_roll_joint",
    "leg_right_hip_roll_joint", "leg_right_hip_yaw_joint",
    "leg_right_hip_pitch_joint", "leg_right_knee_pitch_joint",
    "leg_right_ankle_pitch_joint", "leg_right_ankle_roll_joint",
    "arm_left_shoulder_pitch_joint", "arm_left_shoulder_roll_joint",
    "arm_left_shoulder_yaw_joint", "arm_left_elbow_roll_joint",
    "arm_left_elbow_pitch_joint", "arm_right_shoulder_pitch_joint",
    "arm_right_shoulder_roll_joint", "arm_right_shoulder_yaw_joint",
    "arm_right_elbow_roll_joint", "arm_right_elbow_pitch_joint",
)
TORQUE_NM = 8.0
PHASE_S = 2.5


class ActuationProbe(Node):
    def __init__(self) -> None:
        super().__init__("r53_effort_actuation_probe")
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.positions: dict[str, float] = {}
        self.velocities: dict[str, float] = {}
        self.create_subscription(JointState, JOINT_STATES_TOPIC, self._on_joints, qos)
        self.pub = self.create_publisher(
            Float64MultiArray, COMMAND_TOPIC,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

    def _on_joints(self, msg: JointState) -> None:
        self.positions = dict(zip(msg.name, msg.position))
        if msg.velocity:
            self.velocities = dict(zip(msg.name, msg.velocity))

    def command(self, torque: float) -> None:
        msg = Float64MultiArray()
        msg.data = [
            torque if joint in KNEE_JOINTS else 0.0 for joint in JOINT_ORDER
        ]
        self.pub.publish(msg)

    def hold(self, torque: float, seconds: float) -> dict:
        """Publish *torque* at 50 Hz for *seconds*; return the peak |dq|."""
        deadline = time.monotonic() + seconds
        peak = {j: 0.0 for j in KNEE_JOINTS}
        while time.monotonic() < deadline:
            self.command(torque)
            rclpy.spin_once(self, timeout_sec=0.02)
            for joint in KNEE_JOINTS:
                peak[joint] = max(peak[joint], abs(self.velocities.get(joint, 0.0)))
        return peak


def main(argv: list[str]) -> int:
    out_dir = Path(argv[1]) if len(argv) > 1 else Path(__file__).resolve().parent
    rclpy.init()
    node = ActuationProbe()

    # Wait for joint states (up to 15 s).
    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline and not node.positions:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not node.positions:
        print("FAIL: no /joint_states received")
        return 1

    node.hold(0.0, 0.5)
    before = {j: node.positions.get(j, float("nan")) for j in KNEE_JOINTS}

    peak_pos = node.hold(+TORQUE_NM, PHASE_S)
    mid = {j: node.positions.get(j, float("nan")) for j in KNEE_JOINTS}

    peak_neg = node.hold(-TORQUE_NM, PHASE_S)
    after = {j: node.positions.get(j, float("nan")) for j in KNEE_JOINTS}

    node.command(0.0)  # leave the joints unpowered

    delta_pos = {j: mid[j] - before[j] for j in KNEE_JOINTS}
    delta_neg = {j: after[j] - mid[j] for j in KNEE_JOINTS}

    report = {
        "commanded_torque_nm": TORQUE_NM,
        "phase_s": PHASE_S,
        "knee_positions_rad": {"before": before, "after_plus": mid, "after_minus": after},
        "delta_plus_nm_rad": delta_pos,
        "delta_minus_nm_rad": delta_neg,
        "peak_abs_velocity_rad_s": {
            "plus": peak_pos, "minus": peak_neg,
        },
    }
    moved = any(abs(v) > 0.01 for v in list(delta_pos.values()) + list(delta_neg.values()))
    sign_flips = all(
        delta_pos[j] * delta_neg[j] < 0
        for j in KNEE_JOINTS
        if abs(delta_pos[j]) > 0.01 and abs(delta_neg[j]) > 0.01
    )
    report["checks"] = {
        "effort_moves_the_knees": moved,
        "opposite_torque_reverses_motion": bool(sign_flips),
    }
    report["pass"] = all(report["checks"].values())

    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "actuation_report.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))

    node.destroy_node()
    rclpy.shutdown()
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))