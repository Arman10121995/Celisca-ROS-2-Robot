from __future__ import annotations

import json
import subprocess
import time
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import rclpy
from rclpy.node import Node


class BenchmarkExecutor(Node):
    """Execute benchmark runs with ROS 2 service integration and rosbag capture."""

    def __init__(self, name: str = 'benchmark_executor'):
        try:
            rclpy.init()
        except RuntimeError:
            pass

        super().__init__(name)

    def reset_world(self, reset_service: str = '/gazebo/reset_world') -> bool:
        """Call reset_world service on simulator."""
        try:
            from std_srvs.srv import Empty
            
            client = self.create_client(Empty, reset_service)
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warning(f"Service {reset_service} not available")
                return False

            request = Empty.Request()
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            return future.done() and future.result() is not None
        except Exception as e:
            self.get_logger().error(f"Failed to reset world: {e}")
            return False

    def record_rosbag(
        self,
        output_path: str,
        topics: Optional[list[str]] = None,
        duration_sec: Optional[float] = None,
    ) -> subprocess.Popen:
        """Start a rosbag record subprocess."""
        if topics is None:
            topics = ['/scan', '/odom', '/imu', '/camera/image_raw']

        cmd = ['ros2', 'bag', 'record', '-o', output_path] + topics

        if duration_sec:
            cmd.extend(['--max-bag-size', str(int(duration_sec * 100))])

        try:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            time.sleep(0.5)
            return proc
        except Exception as e:
            self.get_logger().error(f"Failed to start rosbag recording: {e}")
            return None

    def stop_rosbag(self, proc: subprocess.Popen) -> bool:
        """Stop a rosbag record subprocess."""
        if proc is None:
            return False

        try:
            proc.terminate()
            proc.wait(timeout=5.0)
            return proc.returncode == 0
        except subprocess.TimeoutExpired:
            proc.kill()
            return False
        except Exception as e:
            self.get_logger().error(f"Failed to stop rosbag: {e}")
            return False

    def run_experiment(
        self,
        experiment_id: str,
        robot_id: str,
        environment_id: str,
        scenario_id: str,
        seed: int,
        reset_service: str = '/gazebo/reset_world',
        duration_sec: float = 60.0,
        bag_capture: bool = False,
        bag_path: Optional[str] = None,
    ) -> Dict[str, Any]:
        """Execute a single benchmark run with measurement."""
        start_time = time.time()
        bag_proc = None

        try:
            # Reset simulator
            if not self.reset_world(reset_service):
                return {
                    'success': False,
                    'elapsed_seconds': 0.0,
                    'path_length_m': 0.0,
                    'collision_count': 0,
                    'min_clearance_m': 0.0,
                    'error': 'reset_failed',
                }

            # Start rosbag capture if requested
            if bag_capture and bag_path:
                bag_proc = self.record_rosbag(bag_path, duration_sec=duration_sec)

            # Wait for experiment duration
            time.sleep(duration_sec)

            # Stop rosbag
            if bag_proc:
                self.stop_rosbag(bag_proc)

            elapsed = time.time() - start_time

            # Collect results from simulator (placeholder)
            return {
                'success': True,
                'elapsed_seconds': elapsed,
                'path_length_m': 25.0,  # Placeholder
                'collision_count': 0,  # Placeholder
                'min_clearance_m': 0.5,  # Placeholder
            }
        except Exception as e:
            self.get_logger().error(f"Experiment failed: {e}")
            if bag_proc:
                self.stop_rosbag(bag_proc)

            return {
                'success': False,
                'elapsed_seconds': time.time() - start_time,
                'path_length_m': 0.0,
                'collision_count': 0,
                'min_clearance_m': 0.0,
                'error': str(e),
            }

    def shutdown(self) -> None:
        """Clean up ROS 2 resources."""
        try:
            rclpy.shutdown()
        except Exception:
            pass
