"""Stop only the process group created for a GUI launch.

Gazebo's wrapper can exit before its server. The original leader PID remains
the group ID even after that leader exits; looking it up again loses ownership.
"""
import os
import signal
import subprocess
import time


def signal_group(group, sig):
    try:
        os.killpg(group, sig)
        return True
    except ProcessLookupError:
        return False


def stop_group(group, interrupt_timeout=5.0, terminate_timeout=2.0):
    for sig, timeout in ((signal.SIGINT, interrupt_timeout),
                         (signal.SIGTERM, terminate_timeout)):
        if not signal_group(group, sig):
            return
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if not signal_group(group, 0):
                return
            time.sleep(0.05)
    signal_group(group, signal.SIGKILL)


def stop_owned_launch(process, grace_timeout=12.0):
    """Let ros2 launch forward one interrupt and coordinate child cleanup.

    Interrupting the whole group first made launch forward a second SIGINT
    to its children, interrupting their finally blocks and killing the leader
    during escalation. Keep group cleanup for a stuck launch or orphan.
    """
    if process.poll() is None:
        try:
            process.send_signal(signal.SIGINT)
            process.wait(timeout=grace_timeout)
        except ProcessLookupError:
            pass
        except subprocess.TimeoutExpired:
            stop_group(process.pid)
            return
    stop_group(process.pid, interrupt_timeout=0.0)
