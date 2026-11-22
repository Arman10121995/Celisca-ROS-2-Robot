#!/usr/bin/env python3
"""V7: one fleet_ward case as run 3 of 2026-11-18 (r102-fleet-ward): Bumperbot and Labbot cross the
one door in opposite directions, then dock at the far chargers with modeled energy rising, measured
by ward_probe.py; the shared-world launch runs in its own process group and is stopped after.

usage: run_case.py <case-dir>
"""
import os, signal, subprocess, sys, time
from pathlib import Path

HERE = Path('/workspace/molar/ros_ws/bumperbot_ws'); DIR = Path(__file__).resolve().parent
PROBE = HERE/'docs/status/evidence/r102-fleet-ward-2026-11-18/ward_probe.py'


def main():
    case = Path(sys.argv[1]); case.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, IGN_PARTITION='v7_ward_'+str(os.getpid()), GZ_PARTITION='v7_ward_'+str(os.getpid()))
    with (case/'launch.log').open('w') as log:
        launch = subprocess.Popen(['ros2', 'launch', 'robot_lab_bringup', 'shared_world.launch.py',
                                   'session:='+str(DIR/'ward-session.yaml'), 'navigation:=true', 'gui:=false', 'rviz:=false'],
                                  stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
    try:
        with (case/'probe.log').open('w') as log:
            code = subprocess.run(['/usr/bin/python3', '-s', str(PROBE)], stdout=log, stderr=subprocess.STDOUT, timeout=600,
                                  env=dict(env, CASE_OUTPUT=str(case/'run'))).returncode
    except subprocess.TimeoutExpired:
        code = 124
    finally:
        for sig, wait in ((signal.SIGINT, 40), (signal.SIGTERM, 10), (signal.SIGKILL, 5)):
            try:
                os.killpg(launch.pid, sig)
            except ProcessLookupError:
                break
            deadline = time.monotonic()+wait
            while time.monotonic() < deadline and launch.poll() is None:
                time.sleep(.5)
            if launch.poll() is not None:
                break
    leftover = subprocess.run(['pgrep', '-g', str(launch.pid)], capture_output=True, text=True).stdout.split()
    (case/'leftover-pids.txt').write_text(' '.join(leftover)+'\n')
    return code


if __name__ == '__main__':
    sys.exit(main())
