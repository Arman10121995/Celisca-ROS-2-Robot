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


class FlightLifetimeTests(unittest.TestCase):
    """A PX4 flight's SITL and Harmonic server must not outlive the run.

    On 12 October 2026 a flight's PX4 and ``gz sim`` kept running for hours
    after their launch: the PX4 branch of the launch returned before the
    run id and reaper were set up, and the runtime died on SIGTERM without
    its cleanup.  Their Harmonic discovery traffic also flooded every
    Fortress run with "Unknown message type" (~1M log lines per mission).
    """

    def _runtime(self):
        try:
            from robot_lab_adapter import px4_sitl_runtime
        except ImportError as exc:  # pragma: no cover - ROS environment missing
            self.skipTest(str(exc))
        return px4_sitl_runtime

    def test_flight_launch_gets_a_run_id_and_time_to_clean_up(self):
        import importlib.util
        from unittest import mock
        try:
            from launch import LaunchContext
            from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
        except ImportError as exc:  # pragma: no cover
            self.skipTest(str(exc))
        path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                            "launch", "simulated_robot.launch.py")
        spec = importlib.util.spec_from_file_location("robot_lab_flight_lifetime_launch", path)
        module = importlib.util.module_from_spec(spec)
        try:
            spec.loader.exec_module(module)
            description = module.generate_launch_description()
        except Exception as exc:  # pragma: no cover - packages not installed
            self.skipTest("launch file needs the installed workspace: %s" % exc)
        context = LaunchContext()
        for entity in description.entities:
            if isinstance(entity, DeclareLaunchArgument) and entity.default_value is not None:
                context.launch_configurations[entity.name] = "".join(
                    part.perform(context) for part in entity.default_value)
        context.launch_configurations.update(
            robot_model="px4_x500", simulator="gazebo", mode="flight", map_name="nav_obstacle",
            gui="false", start_rviz="false")
        with mock.patch.dict(os.environ, PX4_PYTHON=sys.executable), \
                mock.patch("subprocess.Popen") as popen:
            actions = module._build_simulation_actions(context)
        reaper = [call for call in popen.call_args_list
                  if "robot_lab_utils.partition_reaper" in call.args[0]]
        self.assertEqual(1, len(reaper))
        run_ids = [a for a in actions if isinstance(a, SetEnvironmentVariable)
                   and "".join(p.perform(context) for p in a.name) == "ROBOT_LAB_RUN_ID"]
        self.assertEqual(1, len(run_ids))
        marker = reaper[0].args[0][-1]
        self.assertEqual("ROBOT_LAB_RUN_ID=" + "".join(p.perform(context) for p in run_ids[0].value), marker)
        flights = [a for a in actions if isinstance(a, ExecuteProcess)]
        self.assertEqual(1, len(flights))
        self.assertLess(actions.index(run_ids[0]), actions.index(flights[0]))
        # Humble keeps the escalation timeout private (ExecuteLocal).
        timeout = flights[0]._ExecuteLocal__sigterm_timeout
        self.assertGreaterEqual(float("".join(p.perform(context) for p in timeout)), 20.0)

    def test_sigterm_takes_the_cleanup_path(self):
        runtime = self._runtime()
        import signal
        previous = signal.signal(signal.SIGTERM, runtime._interrupt)
        try:
            with self.assertRaises(KeyboardInterrupt):
                os.kill(os.getpid(), signal.SIGTERM)
                time.sleep(1.0)
        finally:
            signal.signal(signal.SIGTERM, previous)

    def test_harmonic_discovery_stays_off_the_fortress_ports(self):
        ports = self._runtime().HARMONIC_DISCOVERY_PORTS
        # gz-transport13 reads only GZ_DISCOVERY_*; ign-transport11 (Fortress)
        # only IGN_DISCOVERY_* and defaults to 10317/10318.
        self.assertEqual({"GZ_DISCOVERY_MSG_PORT", "GZ_DISCOVERY_SRV_PORT"}, set(ports))
        self.assertTrue({"10317", "10318"}.isdisjoint(ports.values()))
        self.assertEqual(2, len(set(ports.values())))


if __name__ == "__main__":
    unittest.main()
