"""Named, bounded effort commands shared with a ROS effort-controller profile."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml


class JointEffortCommand:
    """Resolve the configured command order and fail closed on invalid input."""

    def __init__(self, config_path, urdf):
        config = yaml.safe_load(Path(config_path).read_text())
        controllers = config['controller_manager']['ros__parameters']
        names = [name for name, value in controllers.items()
                 if isinstance(value, dict) and value.get('type') ==
                 'effort_controllers/JointGroupEffortController']
        if len(names) != 1:
            raise ValueError('exactly one joint-group effort controller is required')
        self.topic = '/' + names[0] + '/commands'
        self.names = tuple(config[names[0]]['ros__parameters']['joints'])
        if not self.names or len(set(self.names)) != len(self.names):
            raise ValueError('effort joint list must be nonempty and unique')
        root = ET.fromstring(urdf)
        joints = {joint.get('name'): joint for joint in root.findall('joint')}
        self.limits = []
        self.dynamics = {}
        for name in self.names:
            if name not in joints or joints[name].get('type') != 'revolute':
                raise ValueError('expected a revolute effort joint: ' + name)
            limit = float(joints[name].find('limit').get('effort'))
            if not math.isfinite(limit) or limit <= 0:
                raise ValueError('invalid effort limit for ' + name)
            self.limits.append(limit)
            dynamics = joints[name].find('dynamics')
            self.dynamics[name] = {
                target: float(dynamics.get(source, '0')) if dynamics is not None else 0.0
                for source, target in [('friction', 'frictionloss'), ('damping', 'damping')]}
            if any(not math.isfinite(v) or v < 0 for v in self.dynamics[name].values()):
                raise ValueError('invalid joint dynamics for ' + name)
        self.limits = np.asarray(self.limits)
        self.clear()

    def clear(self):
        self.values = np.zeros(len(self.names))
        self.received_at = None

    def receive(self, values, now):
        values = np.asarray(values, dtype=float)
        if values.shape != self.limits.shape or not np.isfinite(values).all():
            self.clear()
            return False
        self.values = np.clip(values, -self.limits, self.limits)
        self.received_at = now
        return True

    def command(self, now, timeout):
        if self.received_at is None or now - self.received_at > timeout:
            return np.zeros(len(self.names))
        return self.values.copy()

    def add_actuators(self, mjcf):
        root = ET.fromstring(mjcf)
        joints = {joint.get('name'): joint for joint in root.iter('joint') if joint.get('name')}
        actuator = root.find('actuator')
        if actuator is None:
            actuator = ET.SubElement(root, 'actuator')
        existing = {child.get('joint') for child in actuator}
        for name, limit in zip(self.names, self.limits):
            if name not in joints or name in existing:
                raise ValueError('missing or already-actuated effort joint: ' + name)
            # URDF friction is not preserved by every MuJoCo importer version.
            # Copy declared physical losses, never invent damping to pass a test.
            joints[name].attrib.update({key: str(value) for key, value in self.dynamics[name].items()})
            ET.SubElement(actuator, 'motor', name=name + '_effort', joint=name, gear='1',
                          ctrllimited='true', ctrlrange=f'{-limit} {limit}',
                          forcelimited='true', forcerange=f'{-limit} {limit}')
        return ET.tostring(root, encoding='unicode')
