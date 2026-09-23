"""Processes of a stopped or killed run must not outlive it.

Six simulator spawners from earlier GUI navigation runs were found still
running hours later, all publishing /clock, /odom and /scan into the same
ROS domain, so every new run saw several clocks and several robots.
"""
import os
import subprocess
import sys
import time
import unittest
import uuid

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))), "robot_lab_utils"))

from robot_lab_utils import partition_reaper  # noqa: E402

SLEEPER = [sys.executable, "-c", "import time; time.sleep(60)"]


class ReaperTests(unittest.TestCase):

    def _start(self, **env):
        process = subprocess.Popen(SLEEPER, env=dict(os.environ, **env))
        self.addCleanup(process.kill)
        return process

    def test_only_processes_carrying_the_run_id_are_stopped(self):
        run = "robot_lab_run_" + uuid.uuid4().hex
        ours = self._start(ROBOT_LAB_RUN_ID=run)
        other = self._start(ROBOT_LAB_RUN_ID="robot_lab_run_other")
        time.sleep(0.3)
        self.assertEqual([ours.pid], partition_reaper.processes_in_partition(
            "ROBOT_LAB_RUN_ID=" + run))
        self.assertEqual([], partition_reaper.reap("ROBOT_LAB_RUN_ID=" + run, grace=0.2))
        self.assertIsNotNone(ours.wait(timeout=5))
        self.assertIsNone(other.poll())

    def test_a_bare_name_means_a_gazebo_partition(self):
        partition = "robot_lab_test_" + uuid.uuid4().hex
        server = self._start(IGN_PARTITION=partition)
        time.sleep(0.3)
        self.assertEqual([server.pid], partition_reaper.processes_in_partition(partition))


class ParentDeathTests(unittest.TestCase):

    def test_child_exits_when_its_parent_is_killed(self):
        code = ("import os, subprocess, sys, time\n"
                "child = subprocess.Popen([sys.executable, '-c', "
                "'import sys; sys.path.insert(0, sys.argv[1]);"
                "from robot_lab_utils.process_lifetime import exit_with_parent;"
                "exit_with_parent(); import time; time.sleep(60)', sys.argv[1]])\n"
                "print(child.pid, flush=True)\n"
                "time.sleep(60)\n")
        utils = os.path.dirname(os.path.dirname(os.path.abspath(
            partition_reaper.__file__)))
        parent = subprocess.Popen([sys.executable, "-c", code, utils],
                                  stdout=subprocess.PIPE, text=True)
        child = int(parent.stdout.readline())
        time.sleep(0.5)
        parent.kill()
        parent.wait()
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and os.path.exists("/proc/%d" % child):
            with open("/proc/%d/stat" % child) as stat:
                if stat.read().split()[2] == "Z":
                    break
            time.sleep(0.1)
        alive = os.path.exists("/proc/%d" % child)
        if alive:
            with open("/proc/%d/stat" % child) as stat:
                alive = stat.read().split()[2] != "Z"
        self.assertFalse(alive, "child outlived its parent")


if __name__ == "__main__":
    unittest.main()
