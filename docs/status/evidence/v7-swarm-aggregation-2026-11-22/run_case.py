#!/usr/bin/env python3
"""V7: one swarm aggregation case as run 6 of 2026-11-18 (r103-swarm-aggregation): three
Bumperbots spread 3.6-4.0 m apart in swarm_arena, local_aggregation for 180 s, measured by
swarm_probe.py; the shared-world launch runs in its own process group and is stopped after.

usage: [SWARM_SESSION=<session.yaml>] [SWARM_PHASES=behaviour:seconds,...] run_case.py <case-dir>
"""
import os, signal, subprocess, sys, time
from pathlib import Path

HERE = Path('/workspace/molar/ros_ws/bumperbot_ws'); DIR = Path(__file__).resolve().parent
PROBE = HERE/'docs/status/evidence/swarm-distributed-2026-11-14/swarm_probe.py'


def main():
    case = Path(sys.argv[1]); case.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, IGN_PARTITION='v7_swarm_'+str(os.getpid()), GZ_PARTITION='v7_swarm_'+str(os.getpid()))
    with (case/'launch.log').open('w') as log:
        launch = subprocess.Popen(['ros2', 'launch', 'robot_lab_bringup', 'shared_world.launch.py',
                                   'session:='+os.environ.get('SWARM_SESSION', str(DIR/'session-arena-spread.yaml')), 'navigation:=true', 'gui:=false', 'rviz:=false'],
                                  stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
    try:
        with (case/'probe.log').open('w') as log:
            code = subprocess.run(['/usr/bin/python3', '-s', str(PROBE)], stdout=log, stderr=subprocess.STDOUT, timeout=600,
                                  env=dict(env, CASE_OUTPUT=str(case/'agg'), CASE_SKIP_COVERAGE='1',
                                           CASE_PHASES=os.environ.get('SWARM_PHASES', 'local_aggregation:180'))).returncode
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
