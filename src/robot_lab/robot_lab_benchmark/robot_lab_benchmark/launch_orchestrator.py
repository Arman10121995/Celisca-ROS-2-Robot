"""Seeded launch/reset/run/stop orchestration with rosbag capture (P6.3)."""
from __future__ import annotations

import json
import os
import signal
import subprocess
import time
from pathlib import Path
from typing import Any, Dict, List, Optional


class LaunchOrchestrator:
    """Manage the lifecycle of a seeded simulation benchmark run."""

    def __init__(
        self,
        launch_package: str = 'robot_lab_adapter',
        launch_file: str = 'select_robot.launch.py',
        output_dir: Optional[str] = None,
        ros2_cmd: str = 'ros2',
        bash_cmd: str = '/opt/ros/humble/setup.bash',
    ):
        self.launch_package = launch_package
        self.launch_file = launch_file
        self.output_dir = Path(output_dir) if output_dir else Path.cwd() / 'benchmark_runs'
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.ros2_cmd = ros2_cmd
        self.bash_cmd = bash_cmd
        self._launch_proc: Optional[subprocess.Popen] = None
        self._bag_proc: Optional[subprocess.Popen] = None

    def launch(
        self,
        robot_id: str,
        environment_id: str,
        scenario_id: str,
        seed: int,
        extra_args: Optional[Dict[str, str]] = None,
    ) -> bool:
        """Start the simulation launch file as a background subprocess."""
        run_dir = self.output_dir / f"{robot_id}_{environment_id}_{seed}"
        run_dir.mkdir(parents=True, exist_ok=True)
        cmd = (
            f"source {self.bash_cmd} && {self.ros2_cmd} launch "
            f"{self.launch_package} {self.launch_file} "
            f"robot_id:={robot_id} environment_id:={environment_id} "
            f"scenario_id:={scenario_id} seed:={seed}"
        )
        if extra_args:
            for k, v in extra_args.items():
                cmd += f" {k}:={v}"
        try:
            log_path = run_dir / 'launch.log'
            with open(log_path, 'w', encoding='utf-8') as log_file:
                self._launch_proc = subprocess.Popen(
                    ['bash', '-c', cmd],
                    stdout=log_file, stderr=subprocess.STDOUT,
                    preexec_fn=os.setsid,
                )
            time.sleep(0.5)
            return self._launch_proc.poll() is None
        except Exception as e:
            print(f"Failed to launch simulation: {e}")
            return False

    def reset(self, reset_service: str = '/gazebo/reset_world',
             timeout_sec: float = 5.0) -> bool:
        """Call the simulator reset service via ros2 service call."""
        try:
            # First check the service exists — ros2 service call hangs if not
            check_cmd = (f"source {self.bash_cmd} && {self.ros2_cmd} service "
                         f"list | grep {reset_service}")
            check = subprocess.run(
                ['bash', '-c', check_cmd], capture_output=True,
                text=True, timeout=timeout_sec,
            )
            if reset_service not in check.stdout:
                return False
            cmd = (f"source {self.bash_cmd} && {self.ros2_cmd} service call "
                   f"{reset_service} std_srvs/srv/Empty")
            result = subprocess.run(
                ['bash', '-c', cmd], capture_output=True,
                text=True, timeout=timeout_sec,
            )
            return result.returncode == 0
        except Exception as e:
            print(f"Failed to reset world: {e}")
            return False

    def run(
        self,
        seed: int,
        duration_sec: float = 60.0,
        bag_capture: bool = False,
        topics: Optional[List[str]] = None,
    ) -> Dict[str, Any]:
        """Execute the measurement window with optional rosbag capture."""
        bag_proc = None
        bag_path: Optional[Path] = None
        start = time.time()
        try:
            if bag_capture:
                bag_proc, bag_path = self._start_bag(seed, topics)
            remaining = duration_sec
            while remaining > 0:
                time.sleep(min(0.5, remaining))
                remaining -= 0.5
            elapsed = time.time() - start
            if bag_proc:
                self._stop_bag(bag_proc)
            return {
                'success': True,
                'elapsed_seconds': elapsed,
                'bag_path': str(bag_path) if bag_path else None,
                'bag_capture': bag_capture,
            }
        except Exception as e:
            if bag_proc:
                self._stop_bag(bag_proc)
            return {
                'success': False,
                'elapsed_seconds': time.time() - start,
                'error': str(e),
                'bag_capture': bag_capture,
            }

    def stop(self) -> bool:
        """Terminate any running rosbag and the launch subprocess."""
        ok = True
        if self._bag_proc:
            ok = self._stop_bag(self._bag_proc) and ok
            self._bag_proc = None
        if self._launch_proc:
            try:
                os.killpg(os.getpgid(self._launch_proc.pid), signal.SIGTERM)
                self._launch_proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self._launch_proc.pid), signal.SIGKILL)
            except Exception as e:
                print(f"Failed to stop launch: {e}")
                ok = False
            self._launch_proc = None
        return ok

    def _start_bag(
        self, seed: int, topics: Optional[List[str]] = None,
    ) -> tuple:
        """Start rosbag recording; returns (proc, bag_path)."""
        run_dir = self.output_dir / f"bag_{seed}"
        run_dir.mkdir(parents=True, exist_ok=True)
        bag_path = run_dir / f"run_{seed}"
        # Pre-create the directory so the path exists even if ros2 bag fails
        bag_path.mkdir(parents=True, exist_ok=True)
        topic_str = ' '.join(topics) if topics else '/scan /odom /imu'
        cmd = (f"source {self.bash_cmd} && {self.ros2_cmd} bag record "
               f"-o {bag_path} {topic_str}")
        proc = subprocess.Popen(
            ['bash', '-c', cmd],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
        time.sleep(0.5)
        return proc, bag_path

    @staticmethod
    def _stop_bag(proc: subprocess.Popen) -> bool:
        """Gracefully stop a rosbag record process."""
        try:
            proc.terminate()
            proc.wait(timeout=5.0)
            return True
        except subprocess.TimeoutExpired:
            proc.kill()
            return False

    def execute_full_run(
        self,
        robot_id: str,
        environment_id: str,
        scenario_id: str,
        seed: int,
        duration_sec: float = 60.0,
        bag_capture: bool = False,
        reset_service: str = '/gazebo/reset_world',
        topics: Optional[List[str]] = None,
        launch_args: Optional[Dict[str, str]] = None,
    ) -> Dict[str, Any]:
        """Complete launch -> reset -> run -> stop cycle."""
        run_dir = self.output_dir / f"{robot_id}_{environment_id}_{seed}"
        run_dir.mkdir(parents=True, exist_ok=True)
        summary: Dict[str, Any] = {
            'schema_version': '1.0',
            'robot_id': robot_id,
            'environment_id': environment_id,
            'scenario_id': scenario_id,
            'seed': seed,
            'launch_args': launch_args or {},
        }
        try:
            summary['launch_ok'] = self.launch(
                robot_id, environment_id, scenario_id, seed, launch_args,
            )
            summary['reset_ok'] = self.reset(reset_service)
            run_result = self.run(seed, duration_sec, bag_capture, topics)
            summary.update(run_result)
        finally:
            summary['stop_ok'] = self.stop()
        summary['manifest_path'] = str(run_dir / 'manifest.json')
        Path(summary['manifest_path']).write_text(
            json.dumps(summary, indent=2, default=str), encoding='utf-8',
        )
        return summary
