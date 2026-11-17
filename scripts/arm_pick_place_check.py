#!/usr/bin/env python3
"""Request one pick and place from a running urdf_pick_place node and report the receipt (R5.7, R5.9).

The checker waits for the node to see the object, publishes one
/pick_place/request, waits for the task's result and prints it as JSON. It
exits 0 when the node judged the task passed by simulator truth.

With ``--probe-interlock`` (mobile manipulators) it also tries to make the
base and the arm interfere, from outside the node:

- while the arm moves to the object it sends /cmd_vel 0.1 m/s for one
  second; the base must not move;
- while the base carries the object it sends one articulation target
  (head_1_joint +0.1 rad); the contract must reject it and the head must
  not move.

  python3 scripts/run_with_launch.py --log run.log --launch \\
      "robot_model:=asset_pal_tiago simulator:=mujoco mode:=display map_name:=mobile_fetch_lab" \\
      -- python3 scripts/arm_pick_place_check.py --robot asset_pal_tiago --object fetch_block \\
         --support drop_table --probe-interlock

  python3 scripts/run_with_launch.py --log run.log --launch \\
      "robot_model:=asset_franka_panda_panda simulator:=pybullet mode:=display map_name:=arm_pick_place_lab" \\
      -- python3 scripts/arm_pick_place_check.py --object pickup_cube --support place_table
"""
import argparse
import json
import sys
import time

import rclpy
from std_msgs.msg import Empty, String


class Probe:
    """Outside attempts to move the base during an arm stage and the arm during a carry."""

    def __init__(self, node, arm, command, heartbeat):
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        self.node, self.arm, self.command, self.heartbeat, self.Twist = node, arm, command, heartbeat, Twist
        self.cmd = node.create_publisher(Twist, '/cmd_vel', 1)
        self.pose = None
        node.create_subscription(Odometry, '/odom/ground_truth', self._odom, 10)
        self.report = {}
        self.drive_probe = self.arm_probe = None

    def _odom(self, message):
        p = message.pose.pose.position
        self.pose = (p.x, p.y)

    def _head(self):
        return next((c['measured'] for c in self.arm.get('channels', []) if c['name'] == 'head_1_joint'), None)

    def tick(self, state):
        now = time.monotonic()
        if self.drive_probe is None and state.get('stage') == 'above_object' and self.pose and self.arm.get('active'):
            self.drive_probe = dict(start=self.pose, until=now + 1.0, watch=now + 2.0,
                                    base_inhibited=bool(self.arm.get('base_inhibited')), arm_active=bool(self.arm.get('active')))
        if self.drive_probe and 'moved_m' not in self.drive_probe:
            if now < self.drive_probe['until']:
                message = self.Twist()
                message.linear.x = 0.1
                self.cmd.publish(message)
            elif now >= self.drive_probe['watch']:
                start = self.drive_probe.pop('start')
                self.drive_probe['moved_m'] = round(((self.pose[0] - start[0]) ** 2 + (self.pose[1] - start[1]) ** 2) ** 0.5, 4)
                self.drive_probe.pop('until')
                self.drive_probe.pop('watch')
                self.report['cmd_vel_during_arm_motion'] = self.drive_probe
        if (self.arm_probe is None and state.get('stage') == 'carry' and state.get('message', '').startswith('Phase carry')
                and self.arm.get('plant_id') and self._head() is not None):
            # Wait until the base is really moving (the node waits for the arm to settle first).
            if self.pose and getattr(self, '_carry_start', None) is None:
                self._carry_start = self.pose
            moved = self.pose and ((self.pose[0] - self._carry_start[0]) ** 2 + (self.pose[1] - self._carry_start[1]) ** 2) ** 0.5
            if moved and moved > 0.1:
                head = self._head()
                self.heartbeat.publish(Empty())
                self.command.publish(String(data=json.dumps(dict(plant_id=self.arm['plant_id'],
                                                                 targets={'head_1_joint': head + 0.1}))))
                self.arm_probe = dict(head_start=head, watch=now + 2.0)
        if self.arm_probe and 'head_moved_rad' not in self.arm_probe and now >= self.arm_probe['watch']:
            self.arm_probe['head_moved_rad'] = round(abs(self._head() - self.arm_probe.pop('head_start')), 4)
            self.arm_probe.pop('watch')
            self.arm_probe['status_after'] = self.arm.get('status')
            self.report['arm_target_during_carry'] = self.arm_probe

    def passed(self):
        drive = self.report.get('cmd_vel_during_arm_motion')
        arm = self.report.get('arm_target_during_carry')
        return bool(drive and drive['moved_m'] < 0.01 and arm and arm['head_moved_rad'] < 0.01)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--robot', default='asset_franka_panda_panda', help='profile whose articulation Home is used')
    ap.add_argument('--object', default='pickup_cube')
    ap.add_argument('--support', default='place_table')
    ap.add_argument('--lift', type=float, default=0.10)
    ap.add_argument('--ready-timeout', type=float, default=180.0)
    ap.add_argument('--task-timeout', type=float, default=600.0)
    ap.add_argument('--probe-interlock', action='store_true', help='try base/arm interference (mobile)')
    args = ap.parse_args()
    rclpy.init()
    node = rclpy.create_node('arm_pick_place_check')
    state = {}
    node.create_subscription(String, '/pick_place/state', lambda m: state.update(json.loads(m.data)), 10)
    request = node.create_publisher(String, '/pick_place/request', 1)
    report = dict(object=args.object, support=args.support)
    end = time.monotonic() + args.ready_timeout
    while time.monotonic() < end and args.object not in state.get('objects', []):
        rclpy.spin_once(node, timeout_sec=0.2)
    if args.object not in state.get('objects', []):
        report.update(passed=False, error='object never seen', state=state)
        print(json.dumps(report, indent=1))
        return 1
    # Home first through the articulation contract, as an operator would.
    from robot_lab_utils.installed_assets import installed_profiles
    home_pose = installed_profiles('robots')[args.robot]['articulation']['home']
    arm = {}
    node.create_subscription(String, '/molar/articulation/state', lambda m: arm.update(json.loads(m.data)), 10)
    command = node.create_publisher(String, '/molar/articulation/command', 1)
    heartbeat = node.create_publisher(Empty, '/molar/articulation/heartbeat', 1)
    end = time.monotonic() + 60.0
    sent = False
    while time.monotonic() < end:
        heartbeat.publish(Empty())
        rclpy.spin_once(node, timeout_sec=0.1)
        channels = {c['name']: c for c in arm.get('channels', [])}
        home = home_pose
        if not sent and arm.get('plant_id') and channels and home:
            command.publish(String(data=json.dumps(dict(plant_id=arm['plant_id'], targets=home))))
            sent = True
        elif sent and all(abs((channels[n]['measured'] or 0.) - v) < 0.02 for n, v in home.items() if n in channels) \
                and not arm.get('active'):
            break
    report['home_reached'] = bool(sent and not arm.get('active'))
    time.sleep(1.0)
    previous = json.dumps(state.get('last_result'))
    request.publish(String(data=json.dumps(dict(object=args.object, support=args.support, lift=args.lift))))
    probe = Probe(node, arm, command, heartbeat) if args.probe_interlock else None
    end = time.monotonic() + args.task_timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if probe:
            probe.tick(state)
        if json.dumps(state.get('last_result')) != previous and state.get('phase') == 'idle':
            break
    report.update(result=state.get('last_result'), final_message=state.get('message'))
    report['passed'] = bool((state.get('last_result') or {}).get('passed'))
    if probe:
        report['interlock_probe'] = probe.report
        report['passed'] = report['passed'] and probe.passed()
    print(json.dumps(report, indent=1))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
