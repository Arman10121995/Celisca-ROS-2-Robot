#!/usr/bin/env python3
"""Request one pick and place from a running urdf_pick_place node and report the receipt (R5.7).

The checker waits for the node to see the object, publishes one
/pick_place/request, waits for the task's result and prints it as JSON. It
exits 0 when the node judged the task passed by simulator truth.

  python3 scripts/run_with_launch.py --log run.log --launch \\
      "robot_model:=asset_franka_panda_panda simulator:=pybullet mode:=display map_name:=arm_pick_place_lab" \\
      -- python3 scripts/arm_pick_place_check.py --object pickup_cube --support place_table
"""
import argparse
import json
import sys
import time

import rclpy
from std_msgs.msg import String


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--robot', default='asset_franka_panda_panda', help='profile whose articulation Home is used')
    ap.add_argument('--object', default='pickup_cube')
    ap.add_argument('--support', default='place_table')
    ap.add_argument('--lift', type=float, default=0.10)
    ap.add_argument('--ready-timeout', type=float, default=180.0)
    ap.add_argument('--task-timeout', type=float, default=600.0)
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
    from std_msgs.msg import Empty
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
    end = time.monotonic() + args.task_timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if json.dumps(state.get('last_result')) != previous and state.get('phase') == 'idle':
            break
    report.update(result=state.get('last_result'), final_message=state.get('message'))
    report['passed'] = bool((state.get('last_result') or {}).get('passed'))
    print(json.dumps(report, indent=1))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
