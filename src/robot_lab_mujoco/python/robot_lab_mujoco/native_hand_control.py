"""Coordinated source hand poses and measured contact/effort interruption.

No qpos/qvel writes, source gain/transmission/force-limit changes or object
attachment. Guard thresholds stop advancement; they are not hard contact
force limits or proof of a stable grasp.
"""
import json
import time

import numpy as np

from robot_lab_utils.native_hand import hand_pose_targets
from .native_articulation_control import NativeArticulationControl


class NativeHandControl(NativeArticulationControl):
    def __init__(self, node, contract):
        self.contract = contract
        self.guard = None
        self.motion = dict(state='idle', completion_scope='Hand coordinates only; object grasp unassessed')
        self.finger_bodies = {group['name']: set(group['bodies']) for group in contract['groups']}
        super().__init__(node, contract['channels'], enable_servo=False)
        if self.planning:
            raise ValueError('Dexterous hand targets require their own planning adapter')
        self.target = self.bounded(self.target)
        self.commanded = self.target.copy()
        self.home = self.bounded(self.home)
        # PID controls are length coordinates. Seed only controller memory;
        # measured source joints and physical object states stay untouched.
        self.reset_pid_memory()

    def bounded(self, values):
        return np.clip(values, *np.asarray([channel['limits'] for channel in self.channels]).T)

    def measured_controls(self):
        gain = self.model.actuator_gainprm[self.indices, 0]
        bias = self.model.actuator_biasprm[self.indices]
        values = (-bias[:, 0]-bias[:, 1]*self.data.actuator_length[self.indices])/gain
        for slot, channel in enumerate(self.channels):
            if channel.get('actuation') == 'pid_position':
                values[slot] = self.data.actuator_length[channel['index']]
        # Measurement must not hide the source's soft-limit excursions.
        # Command projection happens independently in hold/stop_group.
        return values

    def reset_pid_memory(self, names=None):
        for slot, channel in enumerate(self.channels):
            if channel.get('actuation') != 'pid_position' or names is not None and channel['name'] not in names:
                continue
            address = int(self.model.actuator_actadr[channel['index']])
            self.data.act[address] = 0.  # Owned integral force, not a joint pose.
            self.data.act[address+1] = self.commanded[slot]  # Owned previous setpoint.

    def hold(self, reason='stopped; holding measured hand coordinates'):
        super().hold(reason)
        self.target = self.bounded(self.target)
        self.commanded = self.target.copy()
        self.guard = None
        self.motion.update(state='interrupted', reason=reason)
        self.reset_pid_memory()

    def receive(self, message):
        try:
            request = json.loads(message.data)
            if not isinstance(request, dict) or request.get('plant_id') != self.plant_id:
                return
            if request.get('operation') != 'hand_pose':
                self.guard = None
                self.motion.update(state='manual', reason='Individual target or taught sequence')
                return super().receive(message)
            if self.fault or time.monotonic()-self.last_heartbeat > .8:
                raise ValueError('Fresh operator heartbeat required')
            if (request.get('source_sha256') != self.source_sha256
                    or request.get('contract_sha256') != self.contract['contract_sha256']):
                raise ValueError('Hand pose belongs to another source/motor contract')
            pose = str(request['pose'])
            targets = hand_pose_targets(self.contract, pose, request.get('closure', 1.),
                request.get('spread', 0.), request.get('finger', 'All'))
            effort_fraction = float(request.get('effort_fraction', .6))
            contact_limit = float(request.get('contact_limit_n', 1.))
            if (not np.isfinite(effort_fraction+contact_limit) or not .05 <= effort_fraction <= 1.
                    or not .2 <= contact_limit <= 5. or not isinstance(request.get('guarded', True), bool)):
                raise ValueError('Effort fraction 0.05–1; contact threshold 0.2–5 N')
            self.hold('New source hand pose')
            for slot, channel in enumerate(self.channels):
                if channel['name'] in targets:
                    self.target[slot] = targets[channel['name']]
            # Only groups actually advancing toward curl participate. Opening
            # and spread work explicitly after a guard without auto-resume.
            selected = []
            for group in self.contract['groups']:
                if any(name in targets and (targets[name]-self.commanded[slot]) *
                        (self.contract['curled_targets'][name]-self.contract['open_targets'][name]) > 1e-8
                        for slot, channel in enumerate(self.channels)
                        for name in [channel['name']] if name in group['channels']):
                    selected.append(group['name'])
            self.guard = dict(groups=selected, blocked={}, contact_limit_n=contact_limit,
                effort_fraction=effort_fraction, started=time.monotonic()) if (
                    request.get('guarded', True) and pose not in ('Open', 'Spread') and selected) else None
            self.motion = dict(state='advancing', pose=pose, finger=str(request.get('finger', 'All')),
                closure=float(request.get('closure', 1.)), spread=float(request.get('spread', 0.)),
                guarded=bool(self.guard), completion_scope='Coordinates/contact stops only; stable grasp unassessed')
            self.status = 'jogging'
        except (ValueError, TypeError, KeyError, IndexError) as exc:
            self.hold('Rejected hand pose: '+str(exc))

    def contacts(self):
        """Actual normal forces per finger group, excluding hand self-contact."""
        groups = {name: dict(normal_force_n=0., contact_count=0, contacts=[]) for name in self.finger_bodies}
        all_hand = set().union(*self.finger_bodies.values())
        for index, contact in enumerate(self.data.contact[:self.data.ncon]):
            geoms = list(map(int, contact.geom))
            bodies = [int(self.model.geom_bodyid[g]) for g in geoms]
            if all(0 < body < self.node.robot_bodies for body in bodies):
                continue  # Source self contacts do not certify object contact.
            if not any(body in all_hand for body in bodies):
                continue
            force = np.zeros(6)
            self.node.mujoco.mj_contactForce(self.model, self.data, index, force)
            normal = max(0., float(force[0]))
            for name, owned in self.finger_bodies.items():
                if not any(body in owned for body in bodies):
                    continue
                record = groups[name]
                record['normal_force_n'] += normal
                record['contact_count'] += 1
                if len(record['contacts']) < 24:
                    record['contacts'].append(dict(geom_ids=geoms, body_ids=bodies,
                        geom_names=[self.node.mujoco.mj_id2name(self.model,
                            self.node.mujoco.mjtObj.mjOBJ_GEOM, g) or '' for g in geoms],
                        normal_force_n=normal, penetration_m=max(0., -float(contact.dist))))
        return groups

    def stop_group(self, group, reason):
        measured = self.bounded(self.measured_controls())
        names = set(group['channels'])
        for slot, channel in enumerate(self.channels):
            if channel['name'] in names:
                self.target[slot] = measured[slot]
                self.commanded[slot] = measured[slot]
        self.reset_pid_memory(names)
        self.guard['blocked'][group['name']] = reason
        self.motion.update(state='contact_or_effort_limited', reason=reason)
        self.status = 'Hand advancement limited; inspect measured contacts, then Open or Stop'

    def check_guard(self):
        if not self.guard:
            return
        if time.monotonic()-self.guard['started'] > 60:
            self.hold('Hand guard deadline; holding measured coordinates')
            return
        contacts = self.contacts()
        for group in self.contract['groups']:
            if group['name'] not in self.guard['groups'] or group['name'] in self.guard['blocked']:
                continue
            reason = ''
            if contacts[group['name']]['normal_force_n'] >= self.guard['contact_limit_n']:
                reason = 'Measured external normal-contact threshold reached'
            for channel in self.channels:
                if channel['name'] not in group['channels']:
                    continue
                index = channel['index']
                measured_force = float(self.data.actuator_force[index])
                if not np.isfinite(measured_force):
                    self.fault = 'Non-finite hand effort; reset required'
                    self.hold(self.fault)
                    return
                if self.model.actuator_forcelimited[index]:
                    # Asymmetric native force bounds retain their own signs.
                    low, high = self.model.actuator_forcerange[index]
                    cap = (high if measured_force >= 0 else -low)*self.guard['effort_fraction']
                else:
                    # UMI authors no tendon force clamp. This explicit 2 N
                    # advancement threshold is not a hard source force limit.
                    cap = 2.*self.guard['effort_fraction']
                if abs(measured_force) >= cap:
                    reason = 'Measured source actuator-effort threshold reached'
            if reason:
                self.stop_group(group, reason)

    def before_step(self):
        self.check_guard()
        super().before_step()

    def after_step(self):
        super().after_step()
        self.check_guard()
        if self.motion['state'] == 'advancing':
            tolerances = np.array([.01 if c['units'] == 'rad' else .002 if c['units'] == 'm'
                else .005*(c['limits'][1]-c['limits'][0]) for c in self.channels])
            if np.all(np.abs(self.measured_controls()-self.target) <= tolerances):
                self.motion['state'] = 'coordinates_reached'
                self.status = 'Hand coordinates reached; grasp stability unassessed'

    def publish(self):
        # mj_step integrates qpos/qvel after computing derived values. Refresh
        # once per display tick so lengths/contacts/body TF and JointState
        # describe the same simulation stamp. This does not advance physics
        # or replace the source actuators; Panda/other controller paths retain
        # their own existing publication contracts.
        self.node.mujoco.mj_forward(self.model, self.data)
        super().publish()

    def extra_state(self):
        return dict(hand=dict(adapter=self.contract['adapter'],
            actuation='authored mujoco.pid position plugin' if self.contract['adapter'] == 'shadow_dexee'
                else 'authored coupled position tendon',
            contract_sha256=self.contract['contract_sha256'],
            groups=[dict(name=group['name'], channels=group['channels']) for group in self.contract['groups']],
            poses=self.contract['poses'], motion=self.motion,
            contacts=self.contacts(), blocked_groups=self.guard['blocked'] if self.guard else {},
            guard_scope='Stop advancement at measured thresholds; source hard bounds unchanged; no grasp guarantee',
            source_effort_limits={c['name']: self.model.actuator_forcerange[c['index']].tolist()
                if self.model.actuator_forcelimited[c['index']] else None for c in self.channels},
            qualification='experimental; object grasp/release and other backend missions pending'))
