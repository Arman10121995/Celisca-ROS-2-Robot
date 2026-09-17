#!/usr/bin/env python3
"""R5.3 live validation of the parallel-ankle balance fix (commit 80925bf).

Bring up the dispatch path (bhl_standing_controller effort forwarder active),
start the humanoid-standing-controller node (same-sign ankle strategy, K=0.7,
PD effort loop closed in the node), and track tilt + joints + the commanded
effort stream.

Success (stance-hold run): no SAFE_STOP, tilt stays well below the 0.70 rad
fall threshold with no divergence trend.
Success (drop run): the landing perturbation excites tilt, the ankle strategy
arrests it, and tilt decays back below 0.15 rad without SAFE_STOP.
"""
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray

TILT_FALL = 0.70

ANKLE_ROLL = ["leg_left_ankle_roll_joint", "leg_right_ankle_roll_joint"]
ANKLE_PITCH = ["leg_left_ankle_pitch_joint", "leg_right_ankle_pitch_joint"]


class Probe(Node):
    def __init__(self):
        super().__init__("r53_ankle_balance_probe")
        self.tilt = 0.0
        self.joints = {}
        self.efforts = {}
        self.joint_names = []
        self.safe_stop = False
        self.create_subscription(Imu, "/imu/out", self._imu, 10)
        self.create_subscription(JointState, "/joint_states", self._js, 50)
        self.create_subscription(
            Float64MultiArray, "/bhl_standing_controller/commands",
            self._cmds, 50)

    def _imu(self, msg):
        x, y, z, w = (msg.orientation.x, msg.orientation.y,
                      msg.orientation.z, msg.orientation.w)
        self.tilt = 2.0 * math.acos(min(1.0, max(-1.0, abs(w))))

    def _js(self, msg):
        self.joints = dict(zip(msg.name, msg.position))
        if not self.joint_names:
            self.joint_names = list(msg.name)

    def _cmds(self, msg):
        # The effort controller commands the 22 canonical joints in order;
        # record the stream so the same-sign ankle response is checkable.
        if self.joint_names and len(msg.data) == len(self.joint_names):
            self.efforts = dict(zip(self.joint_names, msg.data))


def main():
    hold_s = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    report = {"phases": [], "max_tilt": 0.0, "safe_stop": False,
              "tilt_series": [], "ankle_effort_series": [],
              "knee_series": []}
    rclpy.init()
    probe = Probe()
    t0 = time.monotonic()
    last_sample = -1.0
    start = t0
    while time.monotonic() - start < hold_s:
        rclpy.spin_once(probe, timeout_sec=0.05)
        now = time.monotonic() - t0
        if probe.tilt > report["max_tilt"]:
            report["max_tilt"] = round(probe.tilt, 3)
        if probe.tilt >= TILT_FALL:
            report["safe_stop"] = True
        if now - last_sample >= 0.2:  # 5 Hz decimation
            last_sample = now
            report["tilt_series"].append([round(now, 2),
                                          round(probe.tilt, 4)])
            if probe.efforts:
                report["ankle_effort_series"].append({
                    "t": round(now, 2),
                    "roll": [round(probe.efforts.get(j, 0.0), 3)
                             for j in ANKLE_ROLL],
                    "pitch": [round(probe.efforts.get(j, 0.0), 3)
                              for j in ANKLE_PITCH],
                })
            kl = probe.joints.get("leg_left_knee_pitch_joint")
            kr = probe.joints.get("leg_right_knee_pitch_joint")
            report["knee_series"].append(
                [round(now, 2),
                 round(kl, 4) if kl is not None else None,
                 round(kr, 4) if kr is not None else None])

    def phase(name, lo, hi):
        vals = [v for t, v in report["tilt_series"] if lo <= t < hi]
        if vals:
            report["phases"].append({
                "phase": name, "t": [lo, hi],
                "tilt_max": round(max(vals), 4),
                "tilt_end": round(vals[-1], 4),
            })

    third = hold_s / 3.0
    phase("early", 0.0, third)
    phase("mid", third, 2 * third)
    phase("late", 2 * third, hold_s)

    with open("/tmp/r53ankle/balance_report.json", "w") as f:
        json.dump(report, f, indent=1)
    print(json.dumps(report["phases"], indent=1))
    print("max_tilt:", report["max_tilt"], "safe_stop:", report["safe_stop"])
    rclpy.shutdown()


if __name__ == "__main__":
    main()
