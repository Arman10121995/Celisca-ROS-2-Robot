from __future__ import annotations

import json
import math
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
        if not math.isfinite(duration_sec) or not 0 < duration_sec <= 3600:
            raise ValueError('Use a bounded finite capture duration within one hour')
        start_time = time.monotonic()
        bag_proc = None
        subscription = None
        positions, distance = [], 0.
        invalid_truth = False

        try:
            # Reset simulator
            if not self.reset_world(reset_service):
                return {
                    'success': False,
                    'elapsed_seconds': 0.0,
                    'path_length_m': None,
                    'collision_count': None,
                    'min_clearance_m': None,
                    'error': 'reset_failed',
                }

            # Start rosbag capture if requested
            if bag_capture and bag_path:
                bag_proc = self.record_rosbag(bag_path, duration_sec=duration_sec)

            from nav_msgs.msg import Odometry
            from rclpy.qos import qos_profile_sensor_data
            def receive(message):
                nonlocal distance, invalid_truth
                point = message.pose.pose.position
                value = [point.x, point.y, point.z]
                if not all(map(math.isfinite, value)):
                    invalid_truth = True
                    return
                if positions:
                    distance += math.dist(positions[-1], value)
                positions.append(value)
                positions[:] = positions[-2:]
            subscription = self.create_subscription(Odometry, '/odom/ground_truth', receive, qos_profile_sensor_data)
            deadline = time.monotonic()+duration_sec
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=min(.1, max(0., deadline-time.monotonic())))

            # Stop rosbag
            if bag_proc:
                self.stop_rosbag(bag_proc)

            elapsed = time.monotonic() - start_time

            return {
                'success': False,
                'capture_completed': True,
                'mission_outcome': 'unassessed',
                'elapsed_seconds': elapsed,
                'path_length_m': distance if len(positions) >= 2 and not invalid_truth else None,
                'collision_count': None,
                'min_clearance_m': None,
                'measurement_scope': 'Actual ground-truth odometry capture; no scenario evaluator, contact or footprint-clearance source',
            }
        except Exception as e:
            self.get_logger().error(f"Experiment failed: {e}")
            if bag_proc:
                self.stop_rosbag(bag_proc)

            return {
                'success': False,
                'elapsed_seconds': time.monotonic() - start_time,
                'path_length_m': None,
                'collision_count': None,
                'min_clearance_m': None,
                'error': str(e),
            }
        finally:
            if subscription is not None:
                self.destroy_subscription(subscription)

    def shutdown(self) -> None:
        """Clean up ROS 2 resources."""
        try:
            rclpy.shutdown()
        except Exception:
            pass
