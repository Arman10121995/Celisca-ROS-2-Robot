"""Stopping the Isaac backend must not leave the simulator running.

Isaac's python.sh runs the interpreter as a child instead of exec-ing it, so
killing the launched process only killed the shell wrapper and orphaned Kit.
These tests reproduce that process shape with stand-ins, without Isaac.
"""
import os
import subprocess
import sys
import tempfile
import textwrap
import unittest

_SRC = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(_SRC, "robot_lab_isaac", "python"))

try:
    from robot_lab_isaac.isaac_spawner import _group_alive, _terminate_runtime
except Exception as exc:  # pragma: no cover - needs a sourced ROS environment
    _IMPORT_ERROR = exc
else:
    _IMPORT_ERROR = None


def _wrapper(child_code):
    """A python.sh-shaped wrapper: runs a child without exec."""
    directory = tempfile.mkdtemp()
    child = os.path.join(directory, "child.py")
    with open(child, "w") as handle:
        handle.write(textwrap.dedent(child_code))
    script = os.path.join(directory, "python.sh")
    with open(script, "w") as handle:
        handle.write('#!/bin/bash\n"%s" "%s"\n' % (sys.executable, child))
    os.chmod(script, 0o755)
    return subprocess.Popen([script], stdin=subprocess.PIPE,
                            stdout=subprocess.DEVNULL, text=True,
                            start_new_session=True)


@unittest.skipIf(_IMPORT_ERROR is not None, "isaac_spawner not importable")
class IsaacRuntimeShutdownTests(unittest.TestCase):

    def test_runtime_that_honours_stdin_eof_stops_gracefully(self):
        proc = _wrapper("""
            import sys
            sys.stdin.read()   # the runtime stops on stdin EOF
        """)
        self.assertEqual("graceful",
                         _terminate_runtime(proc, graceful_timeout=5.0))
        self.assertFalse(_group_alive(proc.pid))

    def test_hung_runtime_behind_a_non_exec_wrapper_is_fully_killed(self):
        proc = _wrapper("""
            import signal, time
            signal.signal(signal.SIGTERM, signal.SIG_IGN)  # like a hung Kit
            time.sleep(300)
        """)
        outcome = _terminate_runtime(proc, graceful_timeout=0.3,
                                     term_timeout=0.3, kill_timeout=5.0)
        self.assertEqual("sigkill", outcome)
        self.assertFalse(_group_alive(proc.pid),
                         "the child behind the wrapper survived")

    def test_runtime_that_exits_on_sigterm_is_stopped_with_sigterm(self):
        proc = _wrapper("""
            import time
            time.sleep(300)   # ignores stdin, default SIGTERM action
        """)
        self.assertEqual("sigterm",
                         _terminate_runtime(proc, graceful_timeout=0.3,
                                            term_timeout=5.0))
        self.assertFalse(_group_alive(proc.pid))

    def test_already_stopped_runtime_is_reported(self):
        proc = _wrapper("pass\n")
        proc.wait(timeout=10)
        self.assertEqual("not-running", _terminate_runtime(proc))

    def test_the_total_stop_budget_fits_inside_launch_escalation(self):
        """graceful + SIGTERM stages must finish before launch sends SIGTERM."""
        import inspect
        defaults = inspect.signature(_terminate_runtime).parameters
        budget = (defaults["graceful_timeout"].default
                  + defaults["term_timeout"].default)
        self.assertLess(budget, 5.0)


if __name__ == "__main__":
    unittest.main()
