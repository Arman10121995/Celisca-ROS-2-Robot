"""R5.3 actuation probe: do position commands actually move the BHL joints?

Publishes to /bhl_standing_controller/commands (Float64MultiArray, the
controller's 22-joint order) in two phases, while recording tilt + tracking:

  Phase 1 (0-8 s):   all-zero commands. If the position command interface is
                     honored, the straight-legged robot is rigid and stands.
  Phase 2 (8-20 s):  distinctive targets: both knees 0.4 rad, both arm
                     shoulder pitches 0.8 rad. If honored, those joints move.

Writes a JSON report with per-phase max tilt, joint-tracking samples, and the
measured extremes of the four commanded joints.
"""
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.duration import Duration
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray

PHASE1_END = 8.0
PHASE2_END = 20.0
CMD_HZ = 50.0

# bhl_controllers.yaml joint order: 12 legs then 10 arms.
LEG_LEFT = ["leg_left_hip_roll_joint", "leg_left_hip_yaw_joint",
            "leg_left_hip_pitch_joint", "leg_left_knee_pitch_joint",
            "leg_left_ankle_pitch_joint", "leg_left_ankle_roll_joint"]
LEG_RIGHT = ["leg_right_hip_roll_joint", "leg_right_hip_yaw_joint",
             "leg_right_hip_pitch_joint", "leg_right_knee_pitch_joint",
             "leg_right_ankle_pitch_joint", "leg_right_ankle_roll_joint"]
ARMS_LEFT = ["arm_left_shoulder_pitch_joint", "arm_left_shoulder_roll_joint",
             "arm_left_shoulder_yaw_joint", "arm_left_elbow_roll_joint",
             "arm_left_elbow_pitch_joint"]
ARMS_RIGHT = ["arm_right_shoulder_pitch_joint", "arm_right_shoulder_roll_joint",
              "arm_right_shoulder_yaw_joint", "arm_right_elbow_roll_joint",
              "arm_right_elbow_pitch_joint"]
JOINTS = LEG_LEFT + LEG_RIGHT + ARMS_LEFT + ARMS_RIGHT

KNEE_L = JOINTS.index("leg_left_knee_pitch_joint")     # 3
KNEE_R = JOINTS.index("leg_right_knee_pitch_joint")    # 9
SHOULDER_L = JOINTS.index("arm_left_shoulder_pitch_joint")   # 12
SHOULDER_R = JOINTS.index("arm_right_shoulder_pitch_joint")  # 17

WATCHED = [SHOULDER_L, SHOULDER_R, KNEE_L, KNEE_R]
WATCHED_NAMES = [JOINTS[i] for i in WATCHED]


def tilt_from_quat(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return 0.0
    x, y, z, w = x / n, y / n, z / n, w / n
    gz_z = 1 - 2 * (x * x + y * y)
    return math.acos(max(-1.0, min(1.0, gz_z)))


class Probe(Node):
    def __init__(self):
        super().__init__("r53_actuation_probe")
        self.pub = self.create_publisher(Float64MultiArray,
                                         "/bhl_standing_controller/commands", 10)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Imu, "/imu/out", self._on_imu, qos)
        self.create_subscription(JointState, "/joint_states", self._on_js, qos)
        self.create_subscription(Time, "/clock", self._on_clock, qos)
        self.t0 = time.time()
        self.sim_time = None
        self.sim_time_first = None
        self.sim_time_last = None
        self.joints = {}
        self.timeline = []  # every 0.5 s: phase, tilt, watched joint positions
        self._last_sample = -1.0
        self._phase_tilt_max = [0.0, 0.0]
        self.phase = 0
        self.create_timer(1.0 / CMD_HZ, self._tick)

    def _command_for(self, t):
        if t < PHASE1_END:
            return [0.0] * len(JOINTS)
        cmd = [0.0] * len(JOINTS)
        cmd[KNEE_L] = 0.4
        cmd[KNEE_R] = 0.4
        cmd[SHOULDER_L] = 0.8
        cmd[SHOULDER_R] = 0.8
        return cmd

    def _tick(self):
        t = time.time() - self.t0
        phase = 0 if t < PHASE1_END else 1
        self.phase = phase
        msg = Float64MultiArray()
        msg.data = self._command_for(t)
        self.pub.publish(msg)
        if t - self._last_sample >= 0.5:
            self._last_sample = t
            self.timeline.append({
                "t": round(t, 1), "phase": phase + 1,
                "sim_t": round(self.sim_time, 2) if self.sim_time is not None else None,
                "tilt": round(self._tilt, 4) if self._tilt is not None else None,
                **{name: round(self.joints.get(name, float("nan")), 4)
                   for name in WATCHED_NAMES},
            })
        if t >= PHASE2_END:
            self._finish()
            raise SystemExit

    _tilt = None

    def _on_clock(self, msg):
        sim = msg.sec + msg.nanosec * 1e-9
        if self.sim_time_first is None:
            self.sim_time_first = sim
        self.sim_time = sim
        self.sim_time_last = sim

    def _on_imu(self, msg):
        self._tilt = tilt_from_quat(msg.orientation.x, msg.orientation.y,
                                    msg.orientation.z, msg.orientation.w)
        self._phase_tilt_max[self.phase] = max(
            self._phase_tilt_max[self.phase], self._tilt)

    def _on_js(self, msg):
        self.joints = dict(zip(msg.name, msg.position))

    def _finish(self):
        watched = {n: self.joints.get(n) for n in WATCHED_NAMES}
        sim_advanced = None
        if self.sim_time_first is not None and self.sim_time_last is not None:
            sim_advanced = round(self.sim_time_last - self.sim_time_first, 2)
        report = {
            "sim_time_advanced_s": sim_advanced,
            "phase1_zero_hold_max_tilt_rad": round(self._phase_tilt_max[0], 4),
            "phase2_commanded_max_tilt_rad": round(self._phase_tilt_max[1], 4),
            "watched_joints_final": {k: round(v, 4)
                                     for k, v in watched.items() if v is not None},
            "watched_joints_abs_max": {
                name: round(max((abs(row.get(name, 0.0) or 0.0)
                                 for row in self.timeline), default=0.0), 4)
                for name in WATCHED_NAMES},
            "timeline": self.timeline,
        }
        with open(OUT_PATH, "w") as f:
            json.dump(report, f, indent=2)
        print(json.dumps({k: report[k] for k in list(report)[:4]}, indent=2))


OUT_PATH = sys.argv[1] if len(sys.argv) > 1 else "/tmp/r53actuation/actuation_report.json"


def main():
    rclpy.init()
    node = Probe()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
