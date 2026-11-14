#!/usr/bin/env python3
"""Berkeley Humanoid Lite drive screen in MuJoCo (R5.3): walk and held turns.

For each case, launches ``berkeley_humanoid_lite_sim`` in MuJoCo ``loc`` on
``nav_empty`` (the GUI Drive path: /key_vel -> twist_mux -> policy), runs the
retained ``probe_key_vel.py`` (docs/status/evidence/r53-bhl-gui-drive-2026-09-24)
and stops the launch by process group.  The command turns on at 3 simulated
seconds and is held until 13 s (10 s), published at 25 Hz.

  python3 scripts/bhl_policy_screen.py --out <dir> [--policy-dir <export>/bhl] [--cases walk,turn_pos,turn_neg]

Without ``--policy-dir`` the vendored upstream checkpoint runs.  Each case
writes <case>.json (the probe record), <case>.launch.log and a summary in
summary.json: forward distance, yaw change and yaw rate over the held window,
peak tilt and whether the robot ended upright.
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PROBE = ROOT / 'docs/status/evidence/r53-bhl-gui-drive-2026-09-24/probe_key_vel.py'
CASES = {'walk': (0.25, 0.0), 'turn_pos': (0.0, 0.3), 'turn_neg': (0.0, -0.3)}
START_T, STOP_T = 3.0, 13.0


def summarize(record):
    """Distance/yaw over the held window from the probe's records.

    probe_key_vel.py records rows ``[t, x, y, tilt, effort count, yaw, mux vx, mux wz]``
    (every tenth sample).
    """
    rows = record.get('records') or []
    if not rows:
        return dict(error='no records', keys=sorted(record))
    def at(t):
        return min(rows, key=lambda r: abs(r[0] - t))
    def turned(window):
        return sum(math.atan2(math.sin(q[5] - p[5]), math.cos(q[5] - p[5])) for p, q in zip(window, window[1:]))
    a, b = at(START_T), at(STOP_T)
    held = [r for r in rows if START_T <= r[0] <= STOP_T]
    late = [r for r in held if r[0] >= STOP_T - 5.0]
    return dict(forward_m=round(math.hypot(b[1] - a[1], b[2] - a[2]), 3),
                yaw_change_rad=round(turned(held), 3),
                yaw_rate_rad_s=round(turned(held) / (STOP_T - START_T), 3),
                late_yaw_rate_rad_s=round(turned(late) / 5.0, 3) if len(late) > 1 else None,
                peak_tilt_rad=round(float(record.get('max_tilt', 0.0)), 3),
                end_tilt_rad=round(float(record.get('end_tilt', 0.0)), 3),
                upright=bool(float(record.get('end_tilt', 9.0)) < 0.7),
                mux_messages=record.get('messages', {}).get('mux'))


def run(case, out, policy_dir):
    speed, yaw = CASES[case]
    log = open(out / ('%s.launch.log' % case), 'w')
    args = ['ros2', 'launch', 'robot_lab_bringup', 'simulated_robot.launch.py', 'mode:=loc', 'simulator:=mujoco',
            'map_name:=nav_empty', 'robot_model:=berkeley_humanoid_lite_sim', 'gui:=false', 'start_rviz:=false']
    if policy_dir:
        args.append('bhl_policy_dir:=%s' % policy_dir)
    launch = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        time.sleep(12.0)
        env = dict(os.environ, BHL_PROBE_TOPIC='/key_vel', BHL_PROBE_SPEED=str(speed), BHL_PROBE_YAW=str(yaw),
                   BHL_PROBE_START_T=str(START_T), BHL_PROBE_STOP_T=str(STOP_T), BHL_PROBE_RATE_HZ='25',
                   BHL_PROBE_TIMEOUT_S='150')
        probe = subprocess.run([sys.executable, str(PROBE)], env=env, capture_output=True, text=True, timeout=240)
        (out / ('%s.json' % case)).write_text(probe.stdout)
        try:
            record = json.loads(probe.stdout)
        except ValueError:
            return dict(error='probe output not JSON', stderr=probe.stderr[-500:])
        return summarize(record)
    finally:
        os.killpg(launch.pid, signal.SIGINT)
        try:
            launch.wait(timeout=40)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGKILL)
            launch.wait()
        log.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--out', required=True)
    parser.add_argument('--policy-dir', default='')
    parser.add_argument('--cases', default=','.join(CASES))
    options = parser.parse_args()
    out = Path(options.out)
    out.mkdir(parents=True, exist_ok=True)
    summary = dict(policy_dir=options.policy_dir or 'vendored upstream policy_humanoid', start_t=START_T,
                   stop_t=STOP_T, cases={})
    for case in options.cases.split(','):
        summary['cases'][case] = dict(command=CASES[case], **run(case, out, options.policy_dir))
        print(case, json.dumps(summary['cases'][case]), flush=True)
    (out / 'summary.json').write_text(json.dumps(summary, indent=1) + '\n')


if __name__ == '__main__':
    main()
