"""Bounded native Cartesian jogging through joint actuator targets.

The Jacobian and collision prediction use the executed native model. Position
writes occur only in a private prediction MjData; the live robot uses motors.
This experimental local servo is distinct from global MoveIt motion planning.
"""
import json
import time

import numpy as np


class NativeCartesianServo:
    def __init__(self, node, joints, limits, max_rates):
        self.node, self.model, self.data, self.mj = node, node.model, node.data, node.mujoco
        self.joints = np.asarray(joints, dtype=int)
        self.qpos = self.model.jnt_qposadr[self.joints]
        self.dofs = self.model.jnt_dofadr[self.joints]
        self.limits = np.asarray(limits, dtype=float)
        self.max_rates = np.asarray(max_rates, dtype=float)
        self.probe = self.mj.MjData(self.model)
        controlled = set(map(int, self.joints))
        def affected(body):
            while body > 0:
                first, count = int(self.model.body_jntadr[body]), int(self.model.body_jntnum[body])
                if controlled.intersection(range(first, first+count)):
                    return True
                body = int(self.model.body_parentid[body])
            return False
        self.frames = {self.mj.mj_id2name(self.model, self.mj.mjtObj.mjOBJ_SITE, index): index
                       for index in range(self.model.nsite)
                       if int(self.model.site_bodyid[index]) < node.robot_bodies
                       and affected(int(self.model.site_bodyid[index]))}
        self.frames = {name: index for name, index in self.frames.items() if name}
        # Many licensed native arms contain no authored site. Expose their
        # actual terminal body origins rather than guessing a TCP offset.
        parents = set(map(int, self.model.body_parentid[1:node.robot_bodies]))
        for body in range(1, node.robot_bodies):
            if body not in parents and affected(body):
                name = self.mj.mj_id2name(self.model, self.mj.mjtObj.mjOBJ_BODY, body)
                if name:
                    self.frames['body:'+name] = -body-1
        self.clear()

    def clear(self):
        self.velocity = np.zeros(6)
        self.site = None
        self.received = -float('inf')
        self.error = ''

    def receive(self, message):
        try:
            request = json.loads(message.data)
            if not isinstance(request, dict):
                raise ValueError('Cartesian command must be a JSON object')
            if request.get('frame', 'native_world') != 'native_world':
                raise ValueError('Use native_world Cartesian velocity axes')
            self.site = self.frames[request['site']]
            velocity = np.asarray([*request['linear'], *request['angular']], dtype=float)
            if velocity.shape != (6,) or not np.all(np.isfinite(velocity)):
                raise ValueError('Finite XYZ and angular XYZ velocity vectors required')
            self.velocity = np.clip(velocity, [-.04]*3+[-.2]*3, [.04]*3+[.2]*3)
            self.received = time.monotonic()
            self.error = ''
            return True
        except (ValueError, TypeError, KeyError) as exc:
            self.clear()
            self.error = str(exc)
            return False

    def active(self):
        return self.site is not None and time.monotonic()-self.received <= .25 and np.any(self.velocity)

    def target(self, dt):
        if not self.active():
            return None
        jacp, jacr = np.zeros((3, self.model.nv)), np.zeros((3, self.model.nv))
        if self.site >= 0:
            self.mj.mj_jacSite(self.model, self.data, jacp, jacr, self.site)
        else:
            self.mj.mj_jacBody(self.model, self.data, jacp, jacr, -self.site-1)
        jacobian = np.vstack((jacp[:, self.dofs], jacr[:, self.dofs]))
        try:
            velocity = jacobian.T @ np.linalg.solve(
                jacobian @ jacobian.T + .0025*np.eye(6), self.velocity)
            velocity = np.clip(velocity, -self.max_rates, self.max_rates)
            candidate = self.data.qpos[self.qpos] + velocity*dt
            if not np.all(np.isfinite(candidate)):
                raise ValueError('Non-finite Cartesian solution')
            if np.any(candidate < self.limits[:, 0]) or np.any(candidate > self.limits[:, 1]):
                raise ValueError('Cartesian jog reaches a source joint/control limit')
            self.mj.mj_copyData(self.probe, self.model, self.data)
            baseline = {tuple(sorted(map(int, contact.geom))): float(contact.dist)
                        for contact in self.data.contact[:self.data.ncon]}
            self.probe.qpos[self.qpos] = candidate
            self.mj.mj_forward(self.model, self.probe)
            if any(contact.dist < -.001 and contact.dist < baseline.get(
                    tuple(sorted(map(int, contact.geom))), 0.)-.0002 and any(
                    2 <= int(self.model.geom_bodyid[geom]) < self.node.robot_bodies for geom in contact.geom)
                    for contact in self.probe.contact[:self.probe.ncon]):
                raise ValueError('Predicted native contact blocks the Cartesian jog')
            return candidate
        except (ValueError, np.linalg.LinAlgError) as exc:
            self.clear()
            self.error = str(exc)
            return None

    def state(self):
        return dict(sites=list(self.frames), active=bool(self.active()), error=self.error,
            frame='native_world', max_linear_m_s=.04, max_angular_rad_s=.2,
            qualification='experimental local native Jacobian/collision servo; validation pending')
