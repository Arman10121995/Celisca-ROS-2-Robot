#!/usr/bin/env python3
"""Seeded sensor-fault relay for live sensor topics (ROADMAP R6.2).

The relay subscribes a clean sensor topic and republishes it with faults
applied: noise, bias, drift, delay, dropout, occlusion and outliers, each
from its own seed and each active in a time window (scenario seconds on the
simulation clock). Truth topics are refused. For the same input messages and
seeds the output is identical, and every output message's digest goes to
a JSONL trace, so a fault trace can be checked to repeat.

Scenario YAML::

    seed: 7
    relays:
    - input: /scan_clean
      output: /scan
      type: LaserScan          # LaserScan, Imu, Odometry or Image
      faults:
      - {kind: noise, std: 0.02, start_s: 10, end_s: 40}
      - {kind: dropout, probability: 0.1}
      - {kind: occlusion, fraction: 0.25}         # a contiguous sector / image band
      - {kind: delay, seconds: 0.2}
      - {kind: outlier, probability: 0.01, scale: 3.0}
      - {kind: bias, value: 0.05}
      - {kind: drift, rate: 0.001}                 # units per second since the window opened

Gazebo launches take ``sensor_faults:=<scenario.yaml>``: each relayed topic's
bridge output is renamed ``<topic>_clean`` and this relay publishes the
original name, so every consumer (AMCL, SLAM, Nav2) sees the faulted data.
"""
import argparse
import collections
import hashlib
import json
import math
from pathlib import Path

import numpy as np

KINDS = ('noise', 'bias', 'drift', 'delay', 'dropout', 'occlusion', 'outlier')
TYPES = ('LaserScan', 'Imu', 'Odometry', 'Image')


def validate(scenario):
    relays = scenario.get('relays')
    if not isinstance(relays, list) or not relays:
        raise ValueError('A sensor-fault scenario needs at least one relay')
    for relay in relays:
        if relay.get('type') not in TYPES:
            raise ValueError('Relay type must be one of %s' % (TYPES,))
        for key in ('input', 'output'):
            if 'truth' in str(relay.get(key, '')).lower():
                raise ValueError('Truth topics are never faulted: ' + str(relay.get(key)))
        if relay['input'] == relay['output']:
            raise ValueError('Relay input and output must differ')
        for fault in relay.get('faults', []):
            if fault.get('kind') not in KINDS:
                raise ValueError('Unknown fault kind %r' % fault.get('kind'))
            for key, value in fault.items():
                if isinstance(value, (int, float)) and not math.isfinite(value):
                    raise ValueError('Fault parameters must be finite')
    return scenario


class FaultModel:
    """Faults for one relay; ``apply(values, t)`` changes a float array in place.

    Each fault draws from its own generator (scenario seed, relay index,
    fault index), advanced once per message, so traces repeat exactly.
    """
    def __init__(self, faults, seed, relay_index):
        self.faults = []
        for index, fault in enumerate(faults):
            self.faults.append(dict(fault, rng=np.random.default_rng([int(seed), relay_index, index])))

    def active(self, fault, t):
        return fault.get('start_s', 0.) <= t < fault.get('end_s', math.inf)

    def drop(self, t):
        dropped = False
        for fault in self.faults:
            if fault['kind'] == 'dropout':
                draw = fault['rng'].random()
                dropped |= self.active(fault, t) and draw < fault.get('probability', 0.)
        return dropped

    def delay(self, t):
        return max([f.get('seconds', 0.) for f in self.faults if f['kind'] == 'delay' and self.active(f, t)] or [0.])

    def apply(self, values, t, sector=False):
        """Faults on a float vector; returns a dict summary for the trace."""
        summary = {}
        n = len(values)
        for fault in self.faults:
            kind, rng = fault['kind'], fault['rng']
            if kind in ('dropout', 'delay'):
                continue
            draws = None
            if kind == 'noise':
                draws = rng.normal(0., fault.get('std', 0.), n)
            elif kind == 'outlier':
                draws = rng.random(n)
            elif kind == 'occlusion':
                draws = rng.random()
            if not self.active(fault, t) or n == 0:
                continue
            if kind == 'noise':
                values += draws
            elif kind == 'bias':
                values += fault.get('value', 0.)
            elif kind == 'drift':
                values += fault.get('rate', 0.) * (t - fault.get('start_s', 0.))
            elif kind == 'outlier':
                hit = draws < fault.get('probability', 0.)
                values[hit] *= fault.get('scale', 3.)
                summary['outliers'] = int(hit.sum())
            elif kind == 'occlusion':
                count = int(round(n * fault.get('fraction', 0.)))
                if count:
                    start = int(draws * n)
                    # A sector (scan rays, image rows) or, for small vectors, the first components.
                    index = (start + np.arange(count)) % n if sector else np.arange(count)
                    values[index] = np.nan
                    summary['occluded'] = count
            summary[kind] = True
        return summary


def fault_scan(msg, model, t):
    ranges = np.asarray(msg.ranges, dtype=float)
    finite = np.isfinite(ranges)
    work = ranges.copy()
    summary = model.apply(work, t, sector=True)
    occluded = np.isnan(work) & finite
    work = np.where(finite | occluded, work, ranges)            # inf/-inf stay as they were
    work[occluded] = math.inf                                   # an occluded ray returns nothing
    work[np.isfinite(work)] = np.clip(work[np.isfinite(work)], msg.range_min, msg.range_max)
    msg.ranges = work.astype(np.float32).tolist()
    return summary


def fault_vector3(vector, model, t):
    values = np.array([vector.x, vector.y, vector.z], dtype=float)
    summary = model.apply(values, t)
    if not np.isfinite(values).all():          # occlusion means the reading is missing
        values = np.where(np.isfinite(values), values, 0.)
    vector.x, vector.y, vector.z = map(float, values)
    return summary


def fault_imu(msg, model, t):
    summary = fault_vector3(msg.angular_velocity, model, t)
    fault_vector3(msg.linear_acceleration, model, t)
    return summary


def fault_odometry(msg, model, t):
    summary = fault_vector3(msg.twist.twist.linear, model, t)
    p = msg.pose.pose.position
    values = np.array([p.x, p.y], dtype=float)
    model.apply(values, t)
    p.x, p.y = (float(v) if math.isfinite(v) else 0. for v in values)
    return summary


def fault_image(msg, model, t):
    """Depth (32FC1) or 8-bit colour; occlusion is a contiguous run of a row-major image, i.e. a band of rows."""
    if msg.encoding == '32FC1':
        data = np.frombuffer(bytes(msg.data), dtype=np.float32).astype(float)
        summary = model.apply(data, t, sector=True)
        msg.data = data.astype(np.float32).tobytes()     # occluded pixels become NaN (no depth)
        return summary
    if msg.encoding in ('rgb8', 'bgr8', 'mono8', 'rgba8', 'bgra8'):
        data = np.frombuffer(bytes(msg.data), dtype=np.uint8).astype(float)
        summary = model.apply(data, t, sector=True)
        msg.data = np.clip(np.nan_to_num(data, nan=0.), 0, 255).astype(np.uint8).tobytes()
        return summary
    return dict(unsupported_encoding=msg.encoding)


APPLY = dict(LaserScan=fault_scan, Imu=fault_imu, Odometry=fault_odometry, Image=fault_image)


def digest(msg, kind):
    if kind == 'LaserScan':
        payload = np.asarray(msg.ranges, dtype=np.float32).tobytes()
    elif kind == 'Imu':
        payload = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]).tobytes()
    elif kind == 'Odometry':
        payload = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.twist.twist.linear.x,
                            msg.twist.twist.linear.y, msg.twist.twist.linear.z]).tobytes()
    else:
        payload = bytes(msg.data)
    return hashlib.sha256(payload).hexdigest()[:16]


class Relay:
    """One input→output relay: dropout/delay buffering, faults and trace rows (no ROS)."""
    def __init__(self, spec, seed, index):
        self.spec, self.kind = spec, spec['type']
        self.model = FaultModel(spec.get('faults', []), seed, index)
        self.pending = collections.deque()
        self.count = 0

    def receive(self, msg, t):
        """Queue a message at scenario time t; returns its trace row."""
        self.count += 1
        if self.model.drop(t):
            return dict(index=self.count, t=round(t, 4), dropped=True)
        summary = APPLY[self.kind](msg, self.model, t)
        release = t + self.model.delay(t)
        self.pending.append((release, msg))
        return dict(index=self.count, t=round(t, 4), dropped=False, release_t=round(release, 4),
                    digest=digest(msg, self.kind), **summary)

    def due(self, t):
        out = []
        while self.pending and self.pending[0][0] <= t:
            out.append(self.pending.popleft()[1])
        return out


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    import yaml
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import Image, Imu, LaserScan
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', required=True)
    parser.add_argument('--trace', default='')
    options = parser.parse_args(rclpy.utilities.remove_ros_args()[1:])
    scenario = validate(yaml.safe_load(Path(options.scenario).read_text()))
    node = Node('sensor_fault_relay')
    types = dict(LaserScan=LaserScan, Imu=Imu, Odometry=Odometry, Image=Image)
    trace = open(options.trace, 'a', buffering=1) if options.trace else None
    state = dict(start=None, clock=None)
    relays = []

    def scenario_time():
        return 0. if state['clock'] is None or state['start'] is None else state['clock'] - state['start']

    def on_clock(msg):
        state['clock'] = msg.clock.sec + msg.clock.nanosec * 1e-9
        if state['start'] is None:
            state['start'] = state['clock']
    node.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data)
    for index, spec in enumerate(scenario['relays']):
        relay = Relay(spec, scenario.get('seed', 0), index)
        publisher = node.create_publisher(types[spec['type']], spec['output'], qos_profile_sensor_data)
        def receive(msg, relay=relay, spec=spec):
            row = relay.receive(msg, scenario_time())
            if trace:
                trace.write(json.dumps(dict(relay=spec['output'], **row)) + '\n')
        node.create_subscription(types[spec['type']], spec['input'], receive, qos_profile_sensor_data)
        relays.append((relay, publisher))

    def flush():
        t = scenario_time()
        for relay, publisher in relays:
            for msg in relay.due(t):
                publisher.publish(msg)
    node.create_timer(.005, flush)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if trace:
            trace.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
