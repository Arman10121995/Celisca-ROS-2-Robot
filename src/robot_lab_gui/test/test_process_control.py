"""A launch leader exiting must not leave a simulator to poison the next run."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_lab_gui.process_control import stop_group, stop_owned_launch


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


def test_owned_launch_forwards_only_one_interrupt_and_finishes_cleanup(tmp_path):
    child_code = '''
import signal, sys, time
def interrupt(*args):
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    time.sleep(.3)
    sys.exit(0)
signal.signal(signal.SIGINT, interrupt)
print('ready', flush=True)
while True: time.sleep(.1)
'''
    leader_code = '''
import signal, subprocess, sys
child = subprocess.Popen([sys.executable, '-c', sys.argv[1]], stdout=subprocess.PIPE, text=True)
child.stdout.readline()
signal.signal(signal.SIGINT, lambda *args: child.send_signal(signal.SIGINT))
print('ready', flush=True)
sys.exit(child.wait())
'''
    process = subprocess.Popen([sys.executable, '-c', leader_code, child_code],
                               stdout=subprocess.PIPE, text=True, start_new_session=True)
    try:
        assert process.stdout.readline().strip() == 'ready'
        stop_owned_launch(process, grace_timeout=2)
        assert process.returncode == 0
    finally:
        stop_group(process.pid, interrupt_timeout=0, terminate_timeout=0)
