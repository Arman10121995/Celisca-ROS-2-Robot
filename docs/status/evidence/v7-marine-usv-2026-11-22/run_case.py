#!/usr/bin/env python3
"""V7: one lab_usv backend case as on 2026-11-20 (r107-usv-backends): launch the marine
simulator, run rov_probe.py into CASE_OUTPUT, then stop the launch's own process group.

usage: [MARINE_MODEL=lab_rov] run_case.py <simulator> <case-dir>
"""
import os, signal, subprocess, sys, time
from pathlib import Path

HERE = Path('/workspace/molar/ros_ws/bumperbot_ws')
PROBE = HERE/'docs/status/evidence/marine-mujoco-currents-2026-11-14/rov_probe.py'


def main():
    simulator, case = sys.argv[1], Path(sys.argv[2])
    case.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, IGN_PARTITION='v7_usv_'+str(os.getpid()), GZ_PARTITION='v7_usv_'+str(os.getpid()))
    with (case/'launch.log').open('w') as log:
        launch = subprocess.Popen(['ros2', 'launch', 'robot_lab_bringup', 'marine_simulator.launch.py', 'robot_model:='+os.environ.get('MARINE_MODEL', 'lab_usv'),
                                   'map_name:=marine_tank', 'simulator:='+simulator, 'gui:=false', 'rviz:=false'],
                                  stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
    try:
        with (case/'probe.log').open('w') as log:
            code = subprocess.run([sys.executable, str(PROBE)], env=dict(env, CASE_OUTPUT=str(case)),
                                  stdout=log, stderr=subprocess.STDOUT, timeout=480).returncode
    except subprocess.TimeoutExpired:
        code = 124
    finally:
        for sig, wait in ((signal.SIGINT, 30), (signal.SIGTERM, 10), (signal.SIGKILL, 5)):
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
