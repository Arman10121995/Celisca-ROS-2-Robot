"""Named, bounded effort commands shared with a ROS effort-controller profile."""
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

#: The joint-loss attributes this module manages, in MJCF attribute names.
_DYNAMICS_KEYS = ("frictionloss", "armature", "damping")

#: URDF ``<dynamics>`` source attribute for each managed MJCF attribute.
#: The URDF has no armature, and its ``damping`` is a Gazebo stabiliser (5.0
#: for the Berkeley Humanoid Lite, whose own MJCF declares no damping at all).
_URDF_DYNAMICS_SOURCES = (("friction", "frictionloss"), ("damping", "damping"))


def _mjcf_default_classes(root):
    """Resolve every MJCF default class to the joint losses it declares.

    MJCF joint attributes are inherited through nested ``<default>`` classes
    - the Berkeley Humanoid Lite declares ``<joint frictionloss="0.1"
    armature="0.005"/>`` in a class every body selects with ``childclass`` -
    so a per-joint read has to walk that chain, not just the joint element.
    """
    classes = {}
    main = None
    for element in root.findall('default'):
        if element.get('class') is None:
            main = element
            break

    def visit(element, parent):
        name = element.get('class') or ''
        attrs = dict(classes.get(parent, {}))
        joint = element.find('joint')
        if joint is not None:
            for key in _DYNAMICS_KEYS:
                if joint.get(key) is not None:
                    attrs[key] = float(joint.get(key))
        classes[name] = attrs
        for child in element.findall('default'):
            visit(child, name)

    if main is not None:
        visit(main, '')
    for element in root.findall('default'):
        if element is not main:
            # A top-level <default class="x"> inherits the main class.
            visit(element, '')
    return classes


def mjcf_joint_dynamics(text):
    """Effective joint losses of every named joint in an MJCF document.

    Returns ``{joint_name: {attr: float}}`` with all
    :data:`_DYNAMICS_KEYS` resolved the way MuJoCo resolves them: an
    explicit attribute on the joint wins, then its class (the ``class``
    attribute, or the enclosing bodies' ``childclass`` chain), then the
    default-class chain. Unmentioned attributes read 0.0.
    """
    root = ET.fromstring(text)
    classes = _mjcf_default_classes(root)
    resolved = {}

    def visit_body(element, classname):
        for child in element:
            if child.tag == 'body':
                visit_body(child, child.get('childclass', classname))
            elif child.tag == 'joint' and child.get('name'):
                losses = {key: 0.0 for key in _DYNAMICS_KEYS}
                losses.update(classes.get(child.get('class', classname), {}))
                for key in _DYNAMICS_KEYS:
                    if child.get(key) is not None:
                        losses[key] = float(child.get(key))
                resolved[child.get('name')] = losses

    for worldbody in root.findall('worldbody'):
        visit_body(worldbody, '')
    return resolved


class JointEffortCommand:

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
            self.dynamics[name] = {target: 0.0 for target in _DYNAMICS_KEYS}
            if dynamics is not None:
                for source, target in _URDF_DYNAMICS_SOURCES:
                    self.dynamics[name][target] = float(dynamics.get(source, '0'))
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

    def joint_losses(self, native=None):
        """The joint-loss attributes to author, per configured joint.

        ``native`` is an optional ``{joint: {attr: value}}`` mapping read from
        the robot's own MJCF (see :func:`mjcf_joint_dynamics`).  When given it
        replaces the URDF ``<dynamics>`` values outright for those joints -
        every managed attribute is then written, missing ones as 0.0 - so the
        Gazebo-substitute ``damping="5.0"`` cannot leak into a plant that is
        supposed to mirror the model the robot was validated on.
        """
        if native is None:
            return {name: dict(self.dynamics[name]) for name in self.names}
        missing = [name for name in self.names if name not in native]
        if missing:
            raise ValueError('native dynamics incomplete for ' + ', '.join(missing))
        losses = {}
        for name in self.names:
            values = {key: float(native[name].get(key, 0.0)) for key in _DYNAMICS_KEYS}
            if any(not math.isfinite(v) or v < 0 for v in values.values()):
                raise ValueError('invalid native joint dynamics for ' + name)
            losses[name] = values
        return losses

    def add_actuators(self, mjcf, native=None):
        root = ET.fromstring(mjcf)
        joints = {joint.get('name'): joint for joint in root.iter('joint') if joint.get('name')}
        actuator = root.find('actuator')
        if actuator is None:
            actuator = ET.SubElement(root, 'actuator')
        existing = {child.get('joint') for child in actuator}
        losses = self.joint_losses(native)
        for name, limit in zip(self.names, self.limits):
            if name not in joints or name in existing:
                raise ValueError('missing or already-actuated effort joint: ' + name)
            # URDF friction is not preserved by every MuJoCo importer version.
            # Copy declared physical losses, never invent damping to pass a test.
            joints[name].attrib.update({key: str(value) for key, value in losses[name].items()})
            ET.SubElement(actuator, 'motor', name=name + '_effort', joint=name, gear='1',
                          ctrllimited='true', ctrlrange=f'{-limit} {limit}',
                          forcelimited='true', forcerange=f'{-limit} {limit}')
        return ET.tostring(root, encoding='unicode')
