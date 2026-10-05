#!/usr/bin/env python3
"""Own the upstream Humble SLAM process and acknowledge a real graph restart."""
import os
from pathlib import Path
import signal
import subprocess
import sys
from threading import Event

# A fresh child process installs the Linux parent-death signal before exec.
# Keep this before ROS imports; no fork-time callbacks touch ROS worker locks.
if len(sys.argv)>2 and sys.argv[1] == '--child':
    import ctypes
    expected_parent = int(sys.argv[2])
    if ctypes.CDLL(None).prctl(1,signal.SIGTERM,0,0,0) != 0:
        raise SystemExit('Cannot set SLAM child parent-death signal')
    if os.getppid() != expected_parent:
        raise SystemExit(1)
    os.execv(sys.argv[3],sys.argv[3:])

import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger


class SlamSupervisor(Node):
    def __init__(self):
        super().__init__('slam_supervisor')
        binary = Path(get_package_prefix('slam_toolbox')) / 'lib/slam_toolbox/sync_slam_toolbox_node'
        self.command = [sys.executable,str(Path(__file__).resolve()),'--child',str(os.getpid()),
                       str(binary), *[arg.replace('__node:=slam_supervisor','__node:=slam_toolbox')
                                       for arg in sys.argv[1:]]]
        self.process = None
        self.ready = Event()
        self.generation_stamp = 0
        group = ReentrantCallbackGroup()
        self.create_subscription(PoseWithCovarianceStamped,'/pose',self.on_pose,10,callback_group=group)
        self.create_service(Trigger,'/robot_lab/mapping_reset',self.reset,callback_group=group)
        self.start()

    def on_pose(self, msg):
        stamp = msg.header.stamp.sec*1_000_000_000 + msg.header.stamp.nanosec
        if stamp >= self.generation_stamp:
            self.ready.set()

    def start(self):
        self.generation_stamp = self.get_clock().now().nanoseconds
        self.ready.clear()
        self.process = subprocess.Popen(self.command,start_new_session=True)

    def stop(self):
        if self.process is None:
            return
        for sig,seconds in ((signal.SIGINT,10),(signal.SIGTERM,3),(signal.SIGKILL,2)):
            if self.process.poll() is not None:
                break
            try:
                os.killpg(self.process.pid,sig)
                self.process.wait(timeout=seconds)
            except (ProcessLookupError,subprocess.TimeoutExpired):
                continue
        if self.process.poll() is None:
            raise RuntimeError('SLAM process did not stop')

    def reset(self, _request, response):
        try:
            self.stop()
            self.start()
            response.success = self.ready.wait(15) and self.process.poll() is None
            response.message = ('New SLAM graph accepted its first live scan' if response.success
                                else 'SLAM restarted but did not accept a fresh scan')
        except (OSError,RuntimeError) as exc:
            response.success = False
            response.message = str(exc)
        return response


def main():
    def terminate(_sig,_frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,terminate)
    rclpy.init()
    node = SlamSupervisor()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
