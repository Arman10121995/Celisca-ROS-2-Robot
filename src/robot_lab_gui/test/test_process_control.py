"""A launch leader exiting must not leave a simulator to poison the next run."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_lab_gui.process_control import stop_group


def test_cleanup_reaches_orphan_without_touching_another_session(tmp_path):
    child_pid_file = tmp_path / 'child.pid'
    child_code = """
import os, signal, sys, time
signal.signal(signal.SIGINT, signal.SIG_IGN)
signal.signal(signal.SIGTERM, signal.SIG_IGN)
open(sys.argv[1], 'w').write(str(os.getpid()))
time.sleep(60)
"""
    leader_code = """
import subprocess, sys, time
from pathlib import Path
subprocess.Popen([sys.executable, '-c', sys.argv[2], sys.argv[1]])
while not Path(sys.argv[1]).exists(): time.sleep(.01)
"""
    unrelated = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'],
                                 start_new_session=True)
    leader = subprocess.Popen([sys.executable, '-c', leader_code, str(child_pid_file), child_code],
                              start_new_session=True)
    try:
        leader.wait(timeout=5)
        child = int(child_pid_file.read_text())
        assert os.getpgid(child) == leader.pid
        stop_group(leader.pid, interrupt_timeout=.1, terminate_timeout=.1)
        deadline = time.monotonic() + 3
        state = Path('/proc') / str(child) / 'stat'
        while state.exists() and state.read_text().split()[2] != 'Z' and time.monotonic() < deadline:
            time.sleep(.01)
        assert not state.exists() or state.read_text().split()[2] == 'Z'
        assert unrelated.poll() is None
    finally:
        stop_group(leader.pid, interrupt_timeout=0, terminate_timeout=0)
        unrelated.terminate()
        unrelated.wait(timeout=3)
