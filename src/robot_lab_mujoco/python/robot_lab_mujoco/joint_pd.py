"""Per-joint PD targets applied inside MuJoCo at every physics step.

A learned locomotion policy sends joint targets at its own rate (rl_sar:
dt x decimation, 50 Hz); the PD law it was trained with,
``tau = kp (q* - q) + kd (dq* - dq)`` limited to the joint torque, belongs
at the physics rate.  Doing it in ROS adds message latency to a stiff
loop, so each joint gets a MuJoCo ``general`` actuator whose force is
exactly that law:

* position joints (legs): ``ctrl = q*``, gain kp, bias (0, -kp, -kd);
* velocity joints (wheels): ``ctrl = dq*``, gain kd, bias (0, 0, -kd);

with ``forcerange`` = the torque limit.  The model integrator is set to
implicitfast so the velocity terms stay stable on light leg links.

Config (JSON, the ``joint_pd_config`` bridge parameter)::

    {"topic": "/legged_policy/joint_targets", "timeout_s": 0.5,
     "joints": [names], "mode": ["position"|"velocity"], "kp": [], "kd": [],
     "torque_limit": [], "default": [targets used before the first command
     and after command loss]}

Messages are Float64MultiArray targets in the config joint order.
"""
import json
import math
import xml.etree.ElementTree as ET

import numpy as np


class JointPDCommand:
    def __init__(self, config):
        if isinstance(config, str):
            config = json.loads(config)
        self.topic = str(config.get('topic', '/legged_policy/joint_targets'))
        self.timeout = float(config.get('timeout_s', 0.5))
        self.stale_mode = config.get('stale_mode','default')
        if self.stale_mode not in ('default','hold'):
            raise ValueError('PD stale_mode must be default or hold')
        self.names = tuple(config['joints'])
        count = len(self.names)
        if not count or len(set(self.names)) != count:
            raise ValueError('PD joint list must be nonempty and unique')

        def vector(key, default=None):
            value = config.get(key, default)
            array = np.asarray(value, dtype=float).ravel()
            if array.size != count or not np.all(np.isfinite(array)):
                raise ValueError('%s must have %d finite values' % (key, count))
            return array

        self.mode = tuple(config.get('mode', ['position']*count))
        if len(self.mode) != count or any(m not in ('position', 'velocity') for m in self.mode):
            raise ValueError('mode must be position or velocity per joint')
        self.kp = vector('kp')
        self.kd = vector('kd')
        self.limits = vector('torque_limit')
        self.default = vector('default', [0.0]*count)
        if np.any(self.kp < 0) or np.any(self.kd < 0) or np.any(self.limits <= 0):
            raise ValueError('gains must be nonnegative and torque limits positive')
        if self.timeout <= 0 or not math.isfinite(self.timeout):
            raise ValueError('timeout_s must be positive')
        self.clear()

    def clear(self):
        self.values = self.default.copy()
        self.received_at = None

    def receive(self, values, now):
        values = np.asarray(values, dtype=float)
        if values.shape != self.default.shape or not np.all(np.isfinite(values)):
            return False
        self.values = values
        self.received_at = now
        return True

    def command(self, now):
        """Targets for this physics step: the latest command, else the default."""
        if self.received_at is None or (now - self.received_at > self.timeout and self.stale_mode != 'hold'):
            return self.default.copy()
        return self.values.copy()

    def stale(self, now):
        return self.received_at is not None and now - self.received_at > self.timeout

    def add_actuators(self, mjcf, armature=0.0):
        """Add one PD actuator per joint (named <joint>_pd) to an MJCF string."""
        root = ET.fromstring(mjcf)
        joints = {joint.get('name'): joint for joint in root.iter('joint') if joint.get('name')}
        actuator = root.find('actuator')
        if actuator is None:
            actuator = ET.SubElement(root, 'actuator')
        actuated = {child.get('joint') for child in actuator}
        for name, mode, kp, kd, limit in zip(self.names, self.mode, self.kp, self.kd, self.limits):
            if name not in joints:
                raise ValueError('PD joint missing from the model: ' + name)
            if name in actuated:
                raise ValueError('PD joint already actuated: ' + name)
            if armature > 0 and not joints[name].get('armature'):
                joints[name].set('armature', '%g' % armature)
            if mode == 'position':
                gain, bias = kp, '0 %g %g' % (float(-kp), float(-kd))
            else:
                gain, bias = kd, '0 0 %g' % float(-kd)
            ET.SubElement(actuator, 'general', name=name + '_pd', joint=name,
                          gaintype='fixed', gainprm='%g' % float(gain),
                          biastype='affine', biasprm=bias,
                          forcelimited='true',
                          forcerange='%g %g' % (float(-limit), float(limit)))
        return ET.tostring(root, encoding='unicode')
