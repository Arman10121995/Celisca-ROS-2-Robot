#!/usr/bin/env python3
"""R5.3 live validation of the StartupSettle startup transient.

Bring up the dispatch path (both controllers active), start the policy node
(rebuilt with the measured-pose hold + bounded ramp), send zero cmd_vel to
trigger the ramp, and track tilt + joint motion through the whole sequence.

Success: no safe_stop, tilt stays below 0.70 rad, knees ramp toward +0.4 rad
without destabilizing the robot.
"""
import json
import math
import subprocess
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Twist

TILT_FALL = 0.70


class Probe(Node):
    def __init__(self):
        super().__init__("r53_settle_probe")
        self.tilt = 0.0
        self.joints = {}
        self.js_rate_n = 0
        self.create_subscription(Imu, "/imu/out", self._imu, 10)
        self.create_subscription(JointState, "/joint_states", self._js, 50)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timeline = []          # (t, phase, tilt, knee_l, knee_r)

    def _imu(self, msg):
        x, y, z, w = (msg.orientation.x, msg.orientation.y,
                      msg.orientation.z, msg.orientation.w)
        self.tilt = 2.0 * math.acos(min(1.0, abs(w)))

    def _js(self, msg):
        self.js_rate_n += 1
        self.joints = dict(zip(msg.name, msg.position))

    def sample(self, phase):
        kl = self.joints.get("leg_left_knee_pitch_joint")
        kr = self.joints.get("leg_right_knee_pitch_joint")
        self.timeline.append({
            "t": round(time.monotonic() - self.t0, 2),
            "phase": phase,
            "tilt_rad": round(self.tilt, 3),
            "knee_left": round(kl, 3) if kl is not None else None,
            "knee_right": round(kr, 3) if kr is not None else None,
        })

    def send_zero_cmd(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)


def main():
    report = {"phases": [], "timeline": [], "safe_stop": False,
              "max_tilt": 0.0}
    rclpy.init()
    probe = Probe()
    probe.t0 = time.monotonic()

    def record(phase, seconds, rate_hz=2.0):
        start = time.monotonic()
        while time.monotonic() - start < seconds:
            rclpy.spin_once(probe, timeout_sec=0.05)
            if time.monotonic() - probe.t0 and (
                    len(probe.timeline) == 0 or
                    probe.timeline[-1]["phase"] != phase or
                    time.monotonic() - start >=
                    len([e for e in probe.timeline if e["phase"] == phase])
                    / rate_hz):
                probe.sample(phase)
            if probe.tilt > report["max_tilt"]:
                report["max_tilt"] = round(probe.tilt, 3)
            if probe.tilt >= TILT_FALL:
                report["safe_stop"] = True
        report["phases"].append({
            "phase": phase, "tilt_end": round(probe.tilt, 3),
            "knee_left": probe.joints.get("leg_left_knee_pitch_joint"),
            "knee_right": probe.joints.get("leg_right_knee_pitch_joint"),
        })

    # Phase A: pre-policy window - hold publishes the measured pose
    record("pre_policy_hold", 5.0)
    # Start the policy node (rebuilt: measured-pose hold + ramp on cmd_vel)
    policy = subprocess.Popen(
        ["/home/molar1/bumperbot_ws/install/robot_lab_adapter/lib/robot_lab_adapter/humanoid-policy-controller",
         "--imu_topic", "/imu/out",
         "--joint_states_topic", "/joint_states",
         "--command_topic", "/bhl_standing_controller/commands",
         "--cmd_vel_topic", "/cmd_vel",
         "--policy_name", "policy_humanoid",
         "--command_rate_hz", "25.0",
         "--ros-args", "-p", "settle_duration_s:=12.0"],
        stdout=open("/tmp/r53settle/policy.log", "w"),
        stderr=subprocess.STDOUT)
    # Phase B: policy attached, still idle - hold stays measured pose
    record("policy_idle_hold", 5.0)
    # Phase C: zero cmd_vel triggers the slow ramp (12 s to default pose)
    probe.send_zero_cmd()
    record("ramp_after_cmd_vel", 20.0, rate_hz=4.0)
    # Phase D: settled - watch stability
    record("post_ramp_stability", 10.0)

    report["timeline"] = probe.timeline
    policy.terminate()
    try:
        policy.wait(timeout=5)
    except subprocess.TimeoutExpired:
        policy.kill()
    with open("/tmp/r53settle/settle_report.json", "w") as f:
        json.dump(report, f, indent=1)
    print(json.dumps(report["phases"], indent=1))
    print("max_tilt:", report["max_tilt"], "safe_stop:", report["safe_stop"])
    rclpy.shutdown()


if __name__ == "__main__":
    main()
