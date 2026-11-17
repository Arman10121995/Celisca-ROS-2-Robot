#!/usr/bin/env python3
"""Run a checker against a launch it owns, then stop the launch by process group.

  python3 scripts/run_with_launch.py --log out.launch.log --warmup 20 \\
      --launch "robot_model:=X simulator:=pybullet mode:=display map_name:=nav_empty" \\
      -- python3 scripts/screen_urdf_arm.py

The checker's stdout is passed through and its exit code returned. Each run
gets its own Gazebo transport partition, and the launch is started in its own
process group, so no stray processes remain.
"""
import argparse
import os
import shlex
import signal
import subprocess
import sys
import time


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('--launch', required=True, help='simulated_robot.launch.py arguments')
    ap.add_argument('--log', required=True)
    ap.add_argument('--warmup', type=float, default=15.0)
    ap.add_argument('--timeout', type=float, default=1800.0)
    ap.add_argument('checker', nargs=argparse.REMAINDER)
    a = ap.parse_args()
    checker = a.checker[1:] if a.checker[:1] == ['--'] else a.checker
    env = dict(os.environ, IGN_PARTITION='rwl_%d' % os.getpid(), GZ_PARTITION='rwl_%d' % os.getpid())
    log = open(a.log, 'w')
    launch = subprocess.Popen(['ros2', 'launch', 'robot_lab_bringup', 'simulated_robot.launch.py', 'gui:=false',
                               'start_rviz:=false'] + shlex.split(a.launch),
                              env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    code = 1
    try:
        time.sleep(a.warmup)
        if launch.poll() is not None:
            print('{"error": "launch exited before the checker started"}')
            return 1
        code = subprocess.run(checker, env=env, timeout=a.timeout).returncode
    except subprocess.TimeoutExpired:
        print('{"error": "checker timed out"}')
    finally:
        try:
            os.killpg(launch.pid, signal.SIGINT)
            launch.wait(timeout=40)
        except subprocess.TimeoutExpired:
            os.killpg(launch.pid, signal.SIGKILL)
            launch.wait()
        except ProcessLookupError:
            pass
        log.close()
    return code


if __name__ == '__main__':
    sys.exit(main())
