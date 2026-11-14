"""Private joint-space planning and guarded source articulation execution.

The live plant is never pose-written. Only the existing finite-force joint
servos receive rate-bounded targets. A plan is tied to one plant, compiled
scene snapshot and measured start, and requires a separate Execute request.
"""
import copy
import hashlib
import math
import threading
import time
import uuid

import mujoco as mj
import numpy as np

from robot_lab_utils.bounded_control_planner import connect_controls, PlanningBudgetExceeded

SCHEMA = 'robot_lab.articulation_plan.v1'
POSITION_TOL = {'prismatic': .0005, 'revolute': .002}


def geometry_digest(model):
    """Geometry/contact changes invalidate a snapshot; servo budgets do not."""
    digest = hashlib.sha256()
    for array in (model.body_pos, model.body_quat, model.geom_pos, model.geom_quat,
                  model.geom_size, model.geom_type, model.geom_dataid,
                  model.geom_contype, model.geom_conaffinity):
        digest.update(array.tobytes())
    return digest.hexdigest()


class JointScene:
    """Independent model/data, original losses/exclusions and frozen scene."""
    def __init__(self, model, data, root_body, specs, names):
        self.model = copy.copy(model)
        self.data = mj.MjData(self.model)
        mj.mj_copyData(self.data, self.model, data)
        self.snapshot = data.qpos.copy()
        self.mocap_pos, self.mocap_quat = data.mocap_pos.copy(), data.mocap_quat.copy()
        self.equalities = data.eq_active.copy()
        self.specs, self.names = copy.deepcopy(specs), tuple(names)
        self.geometry = geometry_digest(model)
        digest = hashlib.sha256(self.geometry.encode())
        for array in (model.mesh_vert, model.mesh_face, model.hfield_data):
            digest.update(array.tobytes())
        self.scene_sha256 = digest.hexdigest()
        digest = hashlib.sha256(self.scene_sha256.encode())
        for array in (self.snapshot, self.mocap_pos, self.mocap_quat, self.equalities):
            digest.update(array.tobytes())
        self.snapshot_sha256 = digest.hexdigest()
        self.robot_root = int(model.body_rootid[root_body])
        self.robot_bodies = set(np.flatnonzero(model.body_rootid == self.robot_root).tolist())
        self.addresses = {}
        for name in specs:
            joint = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)
            if joint < 0 or int(model.jnt_type[joint]) not in (int(mj.mjtJoint.mjJNT_HINGE), int(mj.mjtJoint.mjJNT_SLIDE)):
                raise ValueError('Source position joint missing from compiled plant: '+name)
            self.addresses[name] = int(model.jnt_qposadr[joint])
        self.scene_addresses = []
        self.root_addresses = []
        for joint in range(model.njnt):
            start = int(model.jnt_qposadr[joint])
            end = int(model.jnt_qposadr[joint+1]) if joint+1 < model.njnt else model.nq
            if int(model.jnt_bodyid[joint]) not in self.robot_bodies:
                self.scene_addresses.extend(range(start, end))
            elif int(model.jnt_type[joint]) == int(mj.mjtJoint.mjJNT_FREE):
                self.root_addresses.extend(range(start, end))
        mj.mj_forward(self.model, self.data)
        self.baseline = {tuple(sorted(map(int, contact.geom))): float(contact.dist)
                         for contact in self.data.contact[:self.data.ncon]}

    def values(self, data):
        return np.array([float(data.qpos[self.addresses[n]]) for n in self.names])

    def settled_start(self, data):
        if (not np.array_equal(data.eq_active, self.equalities)
                or not np.allclose(data.mocap_pos, self.mocap_pos, atol=.001, rtol=0.)
                or not np.allclose(data.mocap_quat, self.mocap_quat, atol=.001, rtol=0.)):
            return False
        if self.scene_addresses and np.max(np.abs(data.qpos[self.scene_addresses]-self.snapshot[self.scene_addresses])) > .001:
            return False
        if self.root_addresses and np.max(np.abs(data.qpos[self.root_addresses]-self.snapshot[self.root_addresses])) > .001:
            return False
        return True

    def configuration_matches(self, data, names=None, factor=2.):
        for name in names if names is not None else self.specs:
            tolerance = POSITION_TOL.get(self.specs[name]['type'], .002)*factor
            if abs(float(data.qpos[self.addresses[name]])-self.snapshot[self.addresses[name]]) > tolerance:
                return False
        return True

    def collision_free(self, values, live=None):
        self.data.qpos[:] = self.snapshot if live is None else live.qpos
        self.data.qvel[:] = 0.
        for name, value in zip(self.names, values):
            self.data.qpos[self.addresses[name]] = value
        mj.mj_forward(self.model, self.data)
        for contact in self.data.contact[:self.data.ncon]:
            owners = [int(self.model.geom_bodyid[g]) for g in contact.geom]
            if not any(body in self.robot_bodies for body in owners):
                continue
            pair = tuple(sorted(map(int, contact.geom)))
            # Keep existing resting contacts; refuse new/worsening penetration.
            if contact.dist < -.0005 and contact.dist < self.baseline.get(pair, 0.)-.0002:
                return False
        return True


def plan_joints(scene, targets, *, budget=8.):
    """Existing bounded RRT-Connect with exact independent-channel slew edges."""
    deadline = time.monotonic()+min(8., float(budget))
    specs, names = scene.specs, scene.names
    start = scene.values(scene.data)
    limits = np.array([[specs[n]['lower'], specs[n]['upper']] for n in names])
    if np.any(start < limits[:, 0]-.00001) or np.any(start > limits[:, 1]+.00001):
        raise ValueError('Measured start is outside source limits')
    start = np.clip(start, limits[:, 0], limits[:, 1])
    goal = np.array([targets[n] for n in names], dtype=float)
    rates = np.array([specs[n]['max_velocity'] for n in names])
    # Angular samples <=0.005 rad, linear <=1 mm and time <=25 ms.
    resolution = np.array([.001 if specs[n]['type'] == 'prismatic' else .005 for n in names])
    sample_dt = min(.025, float(np.min(resolution/rates)))
    inspections = 0
    def edge(a, b):
        nonlocal inspections
        duration = float(np.max(np.abs(b-a)/rates))
        count = max(1, int(math.ceil(duration/sample_dt)))
        for elapsed in np.linspace(0., duration, count+1):
            if time.monotonic() >= deadline or inspections >= 12000:
                raise PlanningBudgetExceeded('Joint collision inspection deadline/budget exceeded')
            inspections += 1
            candidate = a+np.clip(b-a, -rates*elapsed, rates*elapsed)
            if not scene.collision_free(candidate):
                return False
        return True
    result = connect_controls(start, goal, limits, edge, deadline=deadline, seed=0)
    if len(result['path']) > 64:
        raise ValueError('Joint plan exceeds the 64-waypoint execution bound')
    return dict(waypoints=[dict(zip(names, map(float, point))) for point in result['path'][1:]],
                algorithm=result['planner'], nodes=result['nodes'], edges=result['collision_edges'],
                inspections=inspections, max_sample_dt_s=sample_dt,
                position_rates=dict(zip(names, map(float, rates))))


class ArticulationPlanner:
    """Requests enqueue in ROS; snapshots/execution run under the physics lock."""
    def __init__(self, node, contract):
        if contract.couplings or any(s['mode'] != 'position' for s in contract.joints.values()):
            raise ValueError('Joint planner requires independent source position channels')
        self.node, self.contract = node, contract
        self.lock = threading.RLock()
        self.worker = None
        self.generation = 0
        self.pending = self.result = self.scene = self.path = None
        self.phase, self.message = 'idle', 'Plan joint targets, then explicitly Execute.'
        self.plan_id = self.request_id = ''
        self.targets, self.summary = {}, {}
        self.rates = {}
        self.index = 0
        self.ready_at = self.execution_at = self.segment_at = 0.
        self.settled_since = None
        self.plant = contract.plant_id

    @property
    def busy(self):
        return self.phase in ('requested', 'planning', 'executing')

    def state(self):
        with self.lock:
            available = not self.node._model.nflex
            return dict(schema=SCHEMA, available=available, phase=self.phase,
                        message=self.message if available else 'Flex scenes require a separate full-geometry planning adapter',
                        request_id=self.request_id, plan_id=self.plan_id, targets=dict(self.targets),
                        waypoint=self.index, waypoints=len(self.path or []), planner=dict(self.summary),
                        scene_sha256=self.scene.scene_sha256 if self.scene is not None else '',
                        snapshot_sha256=self.scene.snapshot_sha256 if self.scene is not None else '',
                        ready_age_s=max(0., time.monotonic()-self.ready_at) if self.phase == 'ready' else None)

    def cancel(self, reason='Plan canceled', phase='stopped', stop=True):
        with self.lock:
            self.generation += 1
            self.pending = self.result = self.path = self.scene = None
            self.plan_id = ''
            self.phase, self.message = phase, reason
            if stop:
                self.contract.stop(reason)
                self.plant = self.contract.plant_id

    def request(self, request):
        with self.lock:
            self.pending = copy.deepcopy(request)
            if request.get('operation') == 'plan':
                self.phase = 'requested'

    def identity(self, request):
        c = self.contract
        if (request.get('plant_id') != c.plant_id or request.get('model_sha256') != c.model_sha256
                or request.get('urdf_sha256') != c.urdf_sha256):
            raise ValueError('Planning request does not match the current plant/source')
        if (not c.initialized or c.fault or time.monotonic()-c.heartbeat_at > .5):
            raise ValueError('Planning requires fresh operator lease and fault-free measured joints')

    def _start(self, request, data):
        self.identity(request)
        if self.worker is not None and self.worker.is_alive():
            raise ValueError('Previous bounded planner is still finishing; retry')
        if self.contract.active:
            raise ValueError('Wait for measured articulation to stop before planning')
        if self.node._model.nflex or getattr(self.node, '_flex_culling', None) is not None:
            raise ValueError('Flex scenes need a separate full-geometry planning adapter')
        targets = request.get('targets')
        request_id = request.get('request_id')
        if not isinstance(request_id, str) or not 1 <= len(request_id) <= 128:
            raise ValueError('Planning needs a bounded request identity')
        if not isinstance(targets, dict) or not targets or set(targets)-set(self.contract.masters):
            raise ValueError('Plan targets must name independent source position joints')
        for name, value in targets.items():
            spec = self.contract.joints[name]
            if (isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)
                    or not spec['lower'] <= value <= spec['upper']):
                raise ValueError('Plan target exceeds source limits: '+name)
        self.cancel('Holding measured joints while planning', 'planning', stop=False)
        self.contract.hold_for_planning()
        self.plant = self.contract.plant_id
        self.request_id, self.targets = request_id, dict(targets)
        self.scene = scene = JointScene(self.node._model, data, self.node._body_id,
                                       self.contract.joints, sorted(targets))
        # Keep the measured tracking guard rather than increasing its allowed
        # error: cap planned slew by source rate and the servo's kd/kp lag.
        self.rates = {}
        for name in targets:
            spec = self.contract.joints[name]
            tracking = .003 if spec['type'] == 'prismatic' else .04
            self.rates[name] = min(.5*spec['max_velocity'], .5*tracking*spec['kp']/max(spec['kd'], 1e-9))
            scene.specs[name]['max_velocity'] = self.rates[name]
        generation = self.generation
        def work():
            try:
                result, error = plan_joints(scene, targets), ''
            except Exception as exc:
                result, error = None, str(exc)
            with self.lock:
                if generation == self.generation:
                    self.result = (result, error)
        self.worker = threading.Thread(target=work, daemon=True)
        self.worker.start()

    def _scene_matches(self, data, start=False):
        if self.scene is None or self.node._model.nq != len(self.scene.snapshot):
            return False
        if geometry_digest(self.node._model) != self.scene.geometry or not self.scene.settled_start(data):
            return False
        held = set(self.contract.joints)-set(self.targets)
        return self.scene.configuration_matches(data, names=None if start else held)

    def tick(self, positions, velocities, dt, base_moving=False):
        """Before the existing source contract advances its servo references."""
        with self.lock:
            try:
                data, c = self.node._data, self.contract
                if c.plant_id != self.plant:
                    self.cancel('Plant reset/Stop; request a new plan', stop=False)
                    self.plant = c.plant_id
                if self.phase in ('requested', 'planning', 'ready', 'executing'):
                    if base_moving or c.fault or time.monotonic()-c.heartbeat_at > .5:
                        raise ValueError('Planning/execute interrupted by base motion, fault or operator lease loss')
                if self.pending is not None:
                    request, self.pending = self.pending, None
                    operation = request.get('operation')
                    if operation == 'plan':
                        self._start(request, data)
                    elif operation == 'execute_plan':
                        self.identity(request)
                        if (self.phase != 'ready' or request.get('plan_id') != self.plan_id
                                or time.monotonic()-self.ready_at > 10. or not self._scene_matches(data, start=True)):
                            raise ValueError('Plan expired or robot/scene changed; Plan again')
                        self.phase, self.message = 'executing', 'Executing source-bounded collision-checked waypoints'
                        self.index = 0
                        self.settled_since = None
                        self.execution_at = self.segment_at = time.monotonic()
                        self._command_waypoint()
                    else:
                        raise ValueError('Unknown planning operation')
                if self.phase == 'planning' and self.result is not None:
                    result, error = self.result
                    self.result = None
                    if error or not self._scene_matches(data, start=True):
                        raise ValueError(error or 'Robot/scene moved while planning; Plan again')
                    self.path = result.pop('waypoints')
                    self.summary = result
                    self.plan_id = uuid.uuid4().hex
                    self.phase, self.message = 'ready', 'Plan ready; inspect targets and explicitly Execute within 10 seconds'
                    self.ready_at = time.monotonic()
                elif self.phase == 'ready':
                    if time.monotonic()-self.ready_at > 10. or not self._scene_matches(data, start=True):
                        raise ValueError('Plan expired or robot/scene changed; Plan again')
                elif self.phase == 'executing':
                    if c.holding or not self._scene_matches(data):
                        raise ValueError('Plan interrupted by hold or changed robot/scene')
                    if time.monotonic()-self.execution_at > 90. or time.monotonic()-self.segment_at > 30.:
                        raise ValueError('Planned joint motion did not settle within its bounded deadline')
                    # Both actual pose and the next rate-limited servo reference
                    # are checked against the immutable compiled collision plant.
                    scene = self.scene
                    if not scene.collision_free(scene.values(data), live=data):
                        raise ValueError('New/worsening live robot contact; holding measured joints')
                    if any(abs(positions[n]-c.references[n]) > (.003 if c.joints[n]['type'] == 'prismatic' else .04)
                           for n in scene.names):
                        raise ValueError('Measured joint motion diverged from the planned servo reference')
                    next_values = np.array([c.references[n]+np.clip(c.targets[n]-c.references[n],
                        -self.rates[n]*max(0., min(.05, dt)),
                        self.rates[n]*max(0., min(.05, dt))) for n in scene.names])
                    if not scene.collision_free(next_values, live=data):
                        raise ValueError('Next servo reference intersects the current scene')
                    waypoint = self.path[self.index]
                    settled = all(abs(positions[n]-v) <= POSITION_TOL.get(c.joints[n]['type'], .002)
                           and abs(velocities[n]) <= (.003 if c.joints[n]['type'] == 'prismatic' else .02)
                           for n, v in waypoint.items())
                    self.settled_since = (float(data.time) if self.settled_since is None else self.settled_since) if settled else None
                    if self.settled_since is not None and float(data.time)-self.settled_since >= .2:
                        self.index += 1
                        if self.index == len(self.path):
                            self.phase, self.message = 'succeeded', 'All planned waypoints measured and settled; holding joints'
                            c.stop(self.message)
                            self.plant = c.plant_id
                        else:
                            self.segment_at = time.monotonic()
                            self.settled_since = None
                            self._command_waypoint()
            except (ValueError, KeyError, TypeError) as exc:
                self.cancel(str(exc), 'failed')

    def _command_waypoint(self):
        if not self.contract.command(dict(plant_id=self.contract.plant_id, targets=self.path[self.index])):
            raise ValueError('Existing articulation controller rejected a planned waypoint')
