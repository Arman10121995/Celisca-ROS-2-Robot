"""Tie a simulator node's lifetime to the ``ros2 launch`` that started it.

When the launch process dies without stopping its children (the GUI is
closed, the launch is killed, a terminal disappears), a PyBullet or MuJoCo
spawner kept simulating on its own: six such orphans from earlier
navigation runs were found still publishing /clock, /odom and /scan into
the same ROS domain hours later, so every new run saw several clocks and
several robots.  Linux can signal a process when its parent exits; this
asks for SIGTERM.
"""
import ctypes
import os
import signal
import sys

_PR_SET_PDEATHSIG = 1


def exit_with_parent(sig=signal.SIGTERM):
    """Deliver *sig* to this process when its parent exits (Linux only).

    Returns False when unsupported.  If the parent is already gone the
    signal is raised immediately, since the kernel would never send it.
    """
    parent = os.getppid()
    try:
        libc = ctypes.CDLL("libc.so.6", use_errno=True)
        if libc.prctl(_PR_SET_PDEATHSIG, int(sig), 0, 0, 0) != 0:
            return False
    except (OSError, AttributeError):
        return False
    if os.getppid() != parent:
        os.kill(os.getpid(), sig)
    return True


def parent_bound_command(*command):
    """Run a launch-owned native executable without an extra supervising child."""
    if not command:
        raise ValueError('Parent-bound command needs an executable')
    return [sys.executable, '-m', __name__, str(os.getpid()), *command]


def _exec_for_parent(arguments):
    if len(arguments) < 2:
        raise SystemExit('Expected launcher PID and command')
    parent = int(arguments[0])
    if os.getppid() != parent:
        raise SystemExit('Launcher disappeared before child setup')
    if not exit_with_parent() or os.getppid() != parent:
        raise SystemExit('Could not bind simulator lifetime to its launcher')
    os.execvp(arguments[1], arguments[1:])


if __name__ == '__main__':
    _exec_for_parent(sys.argv[1:])
