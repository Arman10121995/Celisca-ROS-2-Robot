#!/usr/bin/env python3
"""Measure an actual fleet-facility lift ride and per-member map transition."""
import hashlib
import json
import math
import os
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


OPERATOR = 'floor-map-trial-20261102'
MEMBER = 'alpha'


class FloorMapTrial(Node):
    def __init__(self, output):
        super().__init__('fleet_floor_map_trial')
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=True)
        self.trace = (self.output / 'trace.jsonl').open('w', buffering=1)
        self.fleet = None
        self.infrastructure = None
        self.pose = None
        self.phase = 'wait_ready'
        self.phase_at = time.monotonic()
        self.settle_at = None
        self.l0_z = None
        self.last_heartbeat = 0.0
        self.last_drive = 0.0
        self.last_trace = 0.0
        self.reason = 'Timed out before completion'
        self.success = False
        self.create_subscription(String, '/fleet/state', self.on_fleet, 10)
        self.create_subscription(String, '/fleet/infrastructure/state', self.on_infrastructure, 10)
        self.create_subscription(Odometry, '/alpha/odom/ground_truth', self.on_pose, 10)
        self.heartbeat_pub = self.create_publisher(String, '/fleet/operator_heartbeat', 10)
        self.fleet_pub = self.create_publisher(String, '/fleet/command', 10)
        self.infrastructure_pub = self.create_publisher(String, '/fleet/infrastructure/command', 10)
        self.drive_pub = self.create_publisher(Twist, '/alpha/key_vel', 10)

    def on_fleet(self, msg):
        try:
            self.fleet = json.loads(msg.data)
        except (ValueError, TypeError):
            return

    def on_infrastructure(self, msg):
        try:
            self.infrastructure = json.loads(msg.data)
        except (ValueError, TypeError):
            return

    def on_pose(self, msg):
        p = msg.pose.pose.position
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        values = (p.x, p.y, p.z, stamp)
        if not all(math.isfinite(value) for value in values):
            return
        speed = None
        if self.pose and stamp > self.pose['stamp']:
            dt = stamp - self.pose['stamp']
            speed = math.dist((p.x, p.y, p.z), (self.pose['x'], self.pose['y'], self.pose['z'])) / dt
        self.pose = dict(x=p.x, y=p.y, z=p.z, stamp=stamp, speed=speed)

    def record(self, kind, **values):
        row = dict(kind=kind, wall_time=time.time(), phase=self.phase, **values)
        self.trace.write(json.dumps(row, allow_nan=False) + '\n')

    def advance(self, phase, **values):
        self.phase = phase
        self.phase_at = time.monotonic()
        self.settle_at = None
        self.record('phase', **values)

    def send_fleet(self, operation, **values):
        data = dict(session_id=self.fleet['session_id'], operator_id=OPERATOR, op=operation, **values)
        self.fleet_pub.publish(String(data=json.dumps(data)))
        self.record('fleet_command', command=data)

    def send_infrastructure(self, operation, **values):
        data = dict(session_id=self.infrastructure['session_id'], operator_id=OPERATOR,
                    op=operation, **values)
        self.infrastructure_pub.publish(String(data=json.dumps(data)))
        self.record('infrastructure_command', command=data)

    def member(self):
        return next((row for row in self.fleet.get('members', []) if row['id'] == MEMBER), {})

    def map_name(self):
        path = self.infrastructure.get('active_robot_maps', {}).get(MEMBER, '')
        return Path(path).name if path else ''

    def heartbeat(self, now):
        if not self.fleet or now - self.last_heartbeat < .15:
            return
        data = dict(session_id=self.fleet['session_id'], operator_id=OPERATOR)
        self.heartbeat_pub.publish(String(data=json.dumps(data)))
        self.last_heartbeat = now

    def step(self, now):
        if self.phase == 'wait_ready':
            if (self.fleet and self.infrastructure and self.infrastructure.get('ready')
                    and self.member().get('navigation_ready')
                    and self.fleet.get('operator_id') == OPERATOR and self.pose
                    and self.member().get('mode') == 'idle' and self.pose['speed'] is not None
                    and math.hypot(self.pose['x'] - 2.5, self.pose['y']) < .2 and self.pose['speed'] < .05):
                self.settle_at = self.settle_at or self.fleet['sim_time']
                if self.fleet['sim_time'] - self.settle_at >= 1.0:
                    self.send_infrastructure('lift', floor='L0')
                    self.advance('open_l0_door', measured_spawn=self.pose)
            else:
                self.settle_at = None
        elif self.phase == 'approach_lift':
            row = self.member()
            if row.get('mode') == 'idle' and row.get('reason', '').startswith('Nav2 ended status='):
                raise RuntimeError('Nav2 approach failed before the lift: ' + row['reason'])
            if (row.get('mode') == 'idle' and self.pose and self.pose['speed'] is not None
                    and math.hypot(self.pose['x'] - 2.5, self.pose['y']) < .2 and self.pose['speed'] < .05):
                self.settle_at = self.settle_at or self.fleet['sim_time']
                if self.fleet['sim_time'] - self.settle_at >= 1.0:
                    self.send_infrastructure('lift', floor='L0')
                    self.advance('open_l0_door')
            else:
                self.settle_at = None
        elif self.phase == 'open_l0_door':
            if self.infrastructure['doors']['lift']['mode'] == 2:
                self.send_fleet('manual', targets=[MEMBER])
                self.advance('arm_boarding_drive')
        elif self.phase == 'arm_boarding_drive':
            if self.member().get('mode') == 'manual':
                if self.pose and self.pose['x'] >= 3.65 and abs(self.pose['y']) < .45:
                    self.drive_pub.publish(Twist())
                    self.send_fleet('stop', targets=[MEMBER])
                    self.advance('settle_on_car')
                elif now - self.last_drive >= .1:
                    command = Twist()
                    command.linear.x = .12
                    self.drive_pub.publish(command)
                    self.last_drive = now
        elif self.phase == 'settle_on_car':
            if (self.member().get('mode') == 'idle' and self.pose and self.pose['speed'] is not None
                    and 3.6 <= self.pose['x'] <= 4.8 and abs(self.pose['y']) < .7 and self.pose['speed'] < .05):
                self.settle_at = self.settle_at or self.fleet['sim_time']
                if self.fleet['sim_time'] - self.settle_at >= 1.0:
                    self.l0_z = self.pose['z']
                    self.send_infrastructure('lift', floor='L1')
                    self.advance('ride_to_l1', body_z_start=self.l0_z)
            else:
                self.settle_at = None
        elif self.phase == 'ride_to_l1':
            floors = self.infrastructure.get('robot_floors', {})
            errors = self.infrastructure.get('map_errors', {})
            if (self.pose and self.pose['z'] >= self.l0_z + 2.7 and floors.get(MEMBER) == 'L1'
                    and self.map_name() == 'map_L1.yaml' and MEMBER not in errors):
                self.record('floor_arrival', floor='L1', body=self.pose, active_map=self.map_name(),
                            map_status=self.infrastructure.get('map_status', {}).get(MEMBER))
                self.send_infrastructure('lift', floor='L0')
                self.advance('ride_to_l0', body_z_start=self.pose['z'])
        elif self.phase == 'ride_to_l0':
            floors = self.infrastructure.get('robot_floors', {})
            errors = self.infrastructure.get('map_errors', {})
            if (self.pose and self.pose['z'] <= self.l0_z + .3 and floors.get(MEMBER) == 'L0'
                    and self.map_name() == 'map.yaml' and MEMBER not in errors
                    and self.infrastructure['doors']['lift']['mode'] == 2):
                self.record('floor_arrival', floor='L0', body=self.pose, active_map=self.map_name(),
                            map_status=self.infrastructure.get('map_status', {}).get(MEMBER))
                self.success = True
                self.reason = 'Actual body rode L0-L1-L0; observed robot floor and namespaced Nav2 map changed at both landings'
                self.advance('complete')
        if self.phase not in ('complete', 'wait_ready'):
            timeouts = dict(approach_lift=110, open_l0_door=30, arm_boarding_drive=30,
                            settle_on_car=20, ride_to_l1=60, ride_to_l0=60)
            if now - self.phase_at > timeouts.get(self.phase, 240):
                raise TimeoutError('Phase timed out: ' + self.phase)
        if self.fleet and now - self.last_trace >= .5:
            self.record('sample', body=self.pose, member=self.member(),
                        facility=self.infrastructure, sim_time=self.fleet.get('sim_time'))
            self.last_trace = now


def main():
    output = os.environ.get('ROBOT_LAB_EXPERIMENT_OUTPUT')
    if not output:
        raise RuntimeError('Set ROBOT_LAB_EXPERIMENT_OUTPUT to an SSD run directory')
    rclpy.init()
    node = FloorMapTrial(output)
    started = time.monotonic()
    error = None
    try:
        while time.monotonic() - started < 260 and node.phase != 'complete':
            rclpy.spin_once(node, timeout_sec=.05)
            now = time.monotonic()
            node.heartbeat(now)
            if node.fleet and node.infrastructure and node.fleet.get('session_id') == node.infrastructure.get('session_id'):
                node.step(now)
        if node.phase != 'complete' and not node.reason.startswith('Actual body'):
            raise TimeoutError('Trial wall-time budget expired in phase ' + node.phase)
    except Exception as exc:
        error = str(exc)
        node.reason = error
    finally:
        if node.fleet:
            data = dict(session_id=node.fleet['session_id'], operator_id=OPERATOR,
                        op='all_stop', targets=[])
            node.fleet_pub.publish(String(data=json.dumps(data)))
        node.record('finish', success=node.success, reason=node.reason, error=error,
                body=node.pose, result_phase=node.phase)
        node.trace.close()
        result = dict(success=node.success, reason=node.reason, error=error, phase=node.phase,
                      wall_seconds=time.monotonic() - started,
                      session_id=node.fleet.get('session_id') if node.fleet else None,
                      body_final=node.pose,
                      source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
        (node.output / 'mission-result.json').write_text(json.dumps(result, indent=2) + '\n')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(json.dumps(result), flush=True)
    return 0 if node.success else 1


if __name__ == '__main__':
    raise SystemExit(main())