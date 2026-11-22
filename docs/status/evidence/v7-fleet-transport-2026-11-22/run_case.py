#!/usr/bin/env python3
"""V7: one fleet lift transport case (r11-molar-transport-2026-11-20 protocol): start the
shared-world session in its own process group, run transport_trial.py for one member into
CASE_OUTPUT, then stop the launch's process group.

usage: run_case.py <session.yaml> <member-id> <case-dir>
"""
import os, signal, subprocess, sys, time
from pathlib import Path

HERE = Path('/workspace/molar/ros_ws/bumperbot_ws')
TRIAL = HERE/'docs/status/evidence/r11-molar-transport-2026-11-20/transport_trial.py'


def main():
    session, member, case = Path(sys.argv[1]).resolve(), sys.argv[2], Path(sys.argv[3])
    case.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, IGN_PARTITION='v7_lift_'+str(os.getpid()), GZ_PARTITION='v7_lift_'+str(os.getpid()))
    with (case/'launch.log').open('w') as log:
        launch = subprocess.Popen(['ros2', 'launch', 'robot_lab_bringup', 'shared_world.launch.py', 'session:='+str(session),
                                   'navigation:=true', 'gui:=false', 'rviz:=false'],
                                  stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
    try:
        with (case/'trial.log').open('w') as log:
            code = subprocess.run([sys.executable, str(TRIAL)], stdout=log, stderr=subprocess.STDOUT, timeout=1200,
                                  env=dict(env, CASE_OUTPUT=str(case/'run'), CASE_ROBOT=member)).returncode
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
