"""Optional source-bounded URDF lift/slide/roller control shared by plants.

This implements a simulator interface, not a hardware controller or runtime
qualification. No body poses or passive joints are commanded.
"""
import hashlib
import json
import math
from pathlib import Path
import threading
import time
import uuid
import xml.etree.ElementTree as ET


class URDFArticulation:
    """Measured-position hold, bounded targets and one explicit coupling tree."""
    def __init__(self, configuration, urdf, model_path):
        config = json.loads(configuration) if isinstance(configuration, str) else configuration
        if not isinstance(config, dict) or config.get('schema') != 'robot_lab.urdf_articulation.v1':
            raise ValueError('Expected robot_lab.urdf_articulation.v1 articulation config')
        self.config = config
        self.model_path = str(Path(model_path).resolve())
        self.model_sha256 = hashlib.sha256(Path(model_path).read_bytes()).hexdigest()
        self.urdf_sha256 = hashlib.sha256(urdf.encode()).hexdigest()
        source = {joint.get('name'): joint for joint in ET.fromstring(urdf).findall('joint')}
        entries = config.get('joints')
        if not isinstance(entries, dict) or not entries:
            raise ValueError('articulation joints must be a nonempty mapping')
        self.masters = tuple(entries)
        self.joints, self.couplings = {}, dict(config.get('couplings', {}))
        self.lock = threading.RLock()

        def number(value, label, positive=False):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError('Articulation '+label+' must be finite')
            if positive and value <= 0:
                raise ValueError('Articulation '+label+' must be positive')
            return float(value)

        def checked(name, entry):
            if name not in source or source[name].get('type') not in ('prismatic', 'revolute', 'continuous'):
                raise ValueError('Articulation movable source joint missing: '+name)
            joint = source[name]
            limit = joint.find('limit')
            if limit is None:
                raise ValueError('Articulation source limits missing: '+name)
            spec = dict(entry)
            if spec.get('mode') not in ('position', 'velocity'):
                raise ValueError('Articulation mode must be position/velocity: '+name)
            spec['type'] = joint.get('type')
            for key in ('effort', 'max_velocity'):
                spec[key] = number(spec[key], name+' '+key, True)
            if spec['effort'] > float(limit.get('effort', '0')) + 1e-9 or spec['max_velocity'] > float(limit.get('velocity', '0')) + 1e-9:
                raise ValueError('Articulation effort/rate exceeds source: '+name)
            spec['kp'], spec['kd'] = number(spec.get('kp', 0), name+' kp'), number(spec.get('kd', 0), name+' kd')
            if spec['kp'] < 0 or spec['kd'] < 0 or (spec['mode'] == 'position' and spec['kp'] <= 0) or (spec['mode'] == 'velocity' and spec['kd'] <= 0):
                raise ValueError('Articulation needs positive position/velocity gains: '+name)
            if spec['mode'] == 'position':
                spec['lower'] = number(spec['lower'], name+' lower')
                spec['upper'] = number(spec['upper'], name+' upper')
                if not spec['lower'] < spec['upper'] or spec['type'] == 'continuous' or spec['lower'] < float(limit.get('lower', '0')) - 1e-9 or spec['upper'] > float(limit.get('upper', '0')) + 1e-9:
                    raise ValueError('Articulation position range exceeds source: '+name)
            else:
                spec['lower'], spec['upper'] = -spec['max_velocity'], spec['max_velocity']
            return spec

        for name, entry in entries.items():
            self.joints[name] = checked(name, entry)
        for name, coupling in self.couplings.items():
            if name in self.joints or not isinstance(coupling, dict) or coupling.get('joint') not in self.masters:
                raise ValueError('Couplings must reference one independent master: '+name)
            master = self.joints[coupling['joint']]
            if master['mode'] != 'position':
                raise ValueError('Only bounded position joints can be coupled')
            coupling['multiplier'] = number(coupling.get('multiplier', 1), name+' multiplier', True)
            coupling['offset'] = number(coupling.get('offset', 0), name+' offset')
            follower = dict(master)
            follower['lower'] = master['lower'] * coupling['multiplier'] + coupling['offset']
            follower['upper'] = master['upper'] * coupling['multiplier'] + coupling['offset']
            follower['max_velocity'] *= coupling['multiplier']
            self.joints[name] = checked(name, follower)
            mimic = source[name].find('mimic')
            if mimic is not None and (mimic.get('joint') != coupling['joint'] or abs(float(mimic.get('multiplier', 1))-coupling['multiplier']) > 1e-9 or abs(float(mimic.get('offset', 0))-coupling['offset']) > 1e-9):
                raise ValueError('Configured coupling differs from source mimic: '+name)
        self.reset()

    def reset(self):
        with self.lock:
            self.plant_id = uuid.uuid4().hex
            self.measured, self.velocities, self.targets, self.references = {}, {}, {}, {}
            self.heartbeat_at = -math.inf
            self.initialized, self.active = False, False
            self.holding = True
            self.fault, self.status = '', 'waiting for measured articulation'
            self.base_inhibited = True

    def heartbeat(self, now=None):
        with self.lock:
            self.heartbeat_at = time.monotonic() if now is None else now

    def stop(self, reason='stopped; holding measured positions'):
        with self.lock:
            # A delayed pre-Stop command must not revive the prior motion.
            # The operator must observe this generation and re-arm heartbeat.
            self.plant_id = uuid.uuid4().hex
            self.heartbeat_at = -math.inf
            if self.initialized:
                for name, spec in self.joints.items():
                    # Physics joint limits permit tiny penetration. Retain
                    # measured truth, but never publish an illegal hold target.
                    value = max(spec['lower'], min(spec['upper'], self.measured[name])) if spec['mode'] == 'position' else 0.
                    self.targets[name] = self.references[name] = value
            self.status = reason
            self.holding = True

    def command(self, message, now=None, base_moving=False):
        with self.lock:
            now = time.monotonic() if now is None else now
            try:
                message = json.loads(message) if isinstance(message, str) else message
                targets = message['targets']
                if not self.initialized or self.fault or message.get('plant_id') != self.plant_id or now-self.heartbeat_at > .5 or base_moving:
                    raise ValueError('Requires current plant, fresh heartbeat and stopped base')
                if not isinstance(targets, dict) or not targets or set(targets)-set(self.masters):
                    raise ValueError('Targets must name independent configured joints')
                for name, value in targets.items():
                    spec = self.joints[name]
                    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not spec['lower'] <= value <= spec['upper']:
                        raise ValueError('Finite target outside source bounds: '+name)
                self.targets.update({name: float(value) for name, value in targets.items()})
                self.holding = False
                self.status = 'executing bounded articulation targets'
                return True
            except (ValueError, TypeError, KeyError) as exc:
                self.stop('rejected: '+str(exc))
                return False

    def step(self, positions, velocities, dt, now=None, base_moving=False):
        with self.lock:
            now = time.monotonic() if now is None else now
            try:
                measured = {name: float(positions[name]) for name in self.joints}
                rates = {name: float(velocities[name]) for name in self.joints}
                if not all(math.isfinite(value) for value in [*measured.values(), *rates.values()]):
                    raise ValueError('Non-finite articulation feedback; reset required')
                for name, value in measured.items():
                    spec = self.joints[name]
                    if spec['mode'] == 'position' and not spec['lower']-.01 <= value <= spec['upper']+.01:
                        raise ValueError('Measured articulation outside source bounds: '+name)
                self.measured, self.velocities = measured, rates
            except (ValueError, KeyError, TypeError) as exc:
                self.fault = str(exc)
                self.stop('fault; holding last finite measured articulation')
            if not self.measured:
                self.base_inhibited = True
                return {'position': {}, 'velocity': {}}
            if not self.initialized:
                self.initialized = True
                self.stop('neutral; holding initial measured articulation')
            if (self.fault or base_moving or now-self.heartbeat_at > .5) and not self.holding:
                self.stop('fault/ownership/watchdog hold' if self.fault or base_moving else 'heartbeat absent; holding measured articulation')
            dt = max(0., min(.05, float(dt))) if math.isfinite(dt) else 0.
            result = {'position': {}, 'velocity': {}}
            for name in self.masters:
                spec = self.joints[name]
                if spec['mode'] == 'position':
                    previous = self.references[name]
                    delta = max(-spec['max_velocity']*dt, min(spec['max_velocity']*dt, self.targets[name]-previous))
                    self.references[name] = max(spec['lower'], min(spec['upper'], previous+delta))
                else:
                    self.references[name] = self.targets[name]
                result[spec['mode']][name] = self.references[name]
            for name, coupling in self.couplings.items():
                spec = self.joints[name]
                if not self.holding:
                    # Stop preserves each actual stage independently. When
                    # movement is explicitly requested again, recouple through
                    # the follower's own bounded reference rate, not a jump.
                    master = coupling['joint']
                    goal = self.targets[master]*coupling['multiplier']+coupling['offset']
                    self.targets[name] = max(spec['lower'], min(spec['upper'], goal))
                    reference = self.references[master]*coupling['multiplier']+coupling['offset']
                    reference = max(spec['lower'], min(spec['upper'], reference))
                    previous = self.references[name]
                    delta = max(-spec['max_velocity']*dt, min(spec['max_velocity']*dt, reference-previous))
                    self.references[name] = max(spec['lower'], min(spec['upper'], previous+delta))
                result['position'][name] = self.references[name]
            self.active = any((not self.holding and abs(self.targets[name]-self.measured[name]) > .002) or abs(self.velocities[name]) > .01 if spec['mode'] == 'position' else abs(self.references[name]) > .001 or abs(self.velocities[name]) > .05 for name, spec in self.joints.items())
            coupling_errors = [self.measured[name]-(self.measured[c['joint']]*c['multiplier']+c['offset']) for name,c in self.couplings.items()]
            self.base_inhibited = bool(self.fault or self.active
                                       or any(self.measured.get(name, 0.) > .005 for name in ('telescope_stage1_joint', 'telescope_stage2_joint'))
                                       or any(abs(error) > .01+1e-9 for error in coupling_errors))
            return result

    def state(self):
        with self.lock:
            coupling_error = {name: self.measured.get(name, 0.)-(self.measured.get(c['joint'], 0.)*c['multiplier']+c['offset']) for name,c in self.couplings.items()}
            coupling_inhibited = any(abs(error) > .01+1e-9 for error in coupling_error.values())
            return dict(schema=self.config['schema'], plant_id=self.plant_id,
                state_monotonic=time.monotonic(),
                model_path=self.model_path, model_sha256=self.model_sha256, urdf_sha256=self.urdf_sha256,
                initialized=self.initialized, status=self.status+('; coupling deviation inhibits base' if coupling_inhibited else ''), fault=self.fault,
                base_inhibited=self.base_inhibited, active=self.active,
                qualification='implemented adapter; runtime validation pending',
                coupling_error=coupling_error, coupling_inhibited=coupling_inhibited,
                channels=[dict(name=name, mode=spec['mode'], units=('m' if spec['type']=='prismatic' else 'rad')+('/s' if spec['mode']=='velocity' else ''), measured=self.measured.get(name), target=self.targets.get(name), reference=self.references.get(name), measured_velocity=self.velocities.get(name), limits=[spec['lower'],spec['upper']], max_velocity=spec['max_velocity'], coupled=name in self.couplings) for name,spec in self.joints.items()])


class ArticulationROS:
    """ROS endpoints; the plant supplies actual measurements and applies targets."""
    def __init__(self, node, configuration, urdf, model_path):
        from std_msgs.msg import Empty, String
        from std_srvs.srv import Trigger
        self.contract = configuration if isinstance(configuration, URDFArticulation) else URDFArticulation(configuration, urdf, model_path)
        self.base_moving = False
        self.publisher = node.create_publisher(String, '/molar/articulation/state', 10)
        self._string = String
        node.create_subscription(Empty, '/molar/articulation/heartbeat', lambda _: self.contract.heartbeat(), 1)
        node.create_subscription(String, '/molar/articulation/command', lambda message: self.contract.command(message.data, base_moving=self.base_moving), 1)
        def stop(_request, response):
            self.contract.stop()
            response.success, response.message = True, 'Holding measured articulation; roller stopped'
            return response
        node.create_service(Trigger, '/molar/articulation/stop', stop)
        self.last_publish = -math.inf

    @property
    def base_inhibited(self):
        return self.contract.base_inhibited

    def reset(self):
        self.contract.reset()

    def step(self, positions, velocities, dt, base_moving=False):
        self.base_moving = bool(base_moving)
        targets = self.contract.step(positions, velocities, dt, base_moving=self.base_moving)
        now = time.monotonic()
        if now-self.last_publish >= .1:
            self.publisher.publish(self._string(data=json.dumps(self.contract.state(), allow_nan=False)))
            self.last_publish = now
        return targets


def add_mujoco_articulation_actuators(mjcf, contract):
    """Physical finite-force servos on the configured hinges/slides only."""
    root = ET.fromstring(mjcf)
    option = root.find('option')
    if option is None:
        option = ET.SubElement(root, 'option')
    option.set('integrator', 'implicitfast')
    actuator = root.find('actuator')
    if actuator is None:
        actuator = ET.SubElement(root, 'actuator')
    existing = {child.get('joint') for child in actuator}
    for name, spec in contract.joints.items():
        if name in existing:
            raise ValueError('Articulation joint already controlled: '+name)
        position = spec['mode'] == 'position'
        gain = spec['kp'] if position else spec['kd']
        bias = '0 %g %g' % (-spec['kp'], -spec['kd']) if position else '0 0 %g' % -spec['kd']
        ET.SubElement(actuator, 'general', name=name+'_articulation', joint=name,
            gaintype='fixed', gainprm=str(gain), biastype='affine', biasprm=bias,
            ctrllimited='true', ctrlrange='%g %g'%(spec['lower'],spec['upper']),
            forcelimited='true', forcerange='%g %g'%(-spec['effort'],spec['effort']))
    return ET.tostring(root, encoding='unicode')
