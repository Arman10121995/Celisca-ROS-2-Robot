#!/usr/bin/env python3
"""Fly the RGB-D X500 through check/takeoff/yaw sweep/land while RTAB-Map maps.

Implementation diagnostic for R5.10 aerial mapping: it uses the same
services and /key_vel input as the GUI Drone page and reports what the
mapper produced (RTAB-Map /rtabmap/info node ids, cloud size).  A launch
with px4_vehicle:=x500_depth aerial_mapping:=rtabmap must be running.
"""
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    from rtabmap_msgs.msg import Info
except ImportError:  # pragma: no cover
    Info = None


class Smoke(Node):
    def __init__(self):
        super().__init__('aerial_mapping_smoke')
        self.status, self.planning, self.info_ids, self.cloud_points = {}, {}, set(), 0
        self.loop_closures, self.localizations = 0, 0
        self.create_subscription(String, '/px4/status', lambda m: setattr(self, 'status', json.loads(m.data)), 10)
        self.create_subscription(String, '/px4/planning/status', lambda m: setattr(self, 'planning', json.loads(m.data)), 10)
        if Info is not None:
            self.create_subscription(Info, '/rtabmap/info', self.info, 10)
        from geometry_msgs.msg import PoseWithCovarianceStamped
        self.create_subscription(PoseWithCovarianceStamped, '/rtabmap/localization_pose',
                                 lambda _m: setattr(self, 'localizations', self.localizations+1), 10)
        self.create_subscription(PointCloud2, '/rtabmap/cloud_map', self.cloud, 1)
        self.key = self.create_publisher(Twist, '/key_vel', 10)
        self.log = []

    def info(self, message):
        self.info_ids.add(message.ref_id)
        if message.loop_closure_id > 0 or message.proximity_detection_id > 0:
            self.loop_closures += 1

    def cloud(self, message):
        self.cloud_points = message.width*message.height

    def spin_for(self, seconds, twist=None):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            if twist is not None:
                self.key.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait(self, predicate, timeout, label):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
            if predicate():
                self.log.append((label, 'ok', round(time.monotonic(), 1)))
                return True
        self.log.append((label, 'timeout', self.status.get('phase'), self.planning.get('takeoff')))
        return False

    def call(self, name):
        client = self.create_client(Trigger, name)
        if not client.wait_for_service(timeout_sec=20):
            self.log.append((name, 'unavailable'))
            return False
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=20)
        result = future.result()
        self.log.append((name, bool(result and result.success), result.message if result else None))
        return bool(result and result.success)


def main():
    rclpy.init()
    node = Smoke()
    ok = node.wait(lambda: node.status.get('takeoff_ready'), 240, 'fcu ready')
    ok = ok and node.call('/px4/takeoff_check')
    ok = ok and node.wait(lambda: (node.planning.get('takeoff') or {}).get('state') in ('clear', 'failed'), 120, 'takeoff check')
    takeoff = (node.planning.get('takeoff') or {})
    if ok and takeoff.get('state') == 'clear':
        ok = node.call('/px4/takeoff') and node.wait(lambda: node.status.get('phase') == 'flying', 90, 'airborne')
        if ok:
            node.spin_for(5)
            sweep = Twist()
            sweep.angular.z = 1.0  # full manual yaw request (the profile caps the rate)
            node.spin_for(30, sweep)
            node.spin_for(3, Twist())
            forward = Twist()
            forward.linear.x = 0.5
            node.spin_for(6, forward)
            node.spin_for(4, Twist())
        node.call('/px4/land')
        node.wait(lambda: not node.status.get('armed'), 120, 'landed')
    node.spin_for(3)
    report = dict(steps=node.log, takeoff=takeoff, final_status=node.status,
                  rtabmap_nodes=len(node.info_ids), cloud_points=node.cloud_points,
                  loop_or_proximity_closures=node.loop_closures, localization_poses=node.localizations)
    print(json.dumps(report, indent=1, default=str))
    node.destroy_node()
    rclpy.shutdown()
    mapped = report['rtabmap_nodes'] > 1 or report['localization_poses'] > 0
    return 0 if mapped else 1


if __name__ == '__main__':
    sys.exit(main())
