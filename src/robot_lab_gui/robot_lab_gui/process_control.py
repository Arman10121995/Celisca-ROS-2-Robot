"""Stop only the process group created for a GUI launch.

Gazebo's wrapper can exit before its server. The original leader PID remains
the group ID even after that leader exits; looking it up again loses ownership.
"""
import os
import signal
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
