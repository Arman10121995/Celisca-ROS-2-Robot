"""Private native FK in authored actuator coordinates, including linear coupling.

Supports joint/fixed-tendon position transmissions and linear joint equality
constraints. Unsupported spatial tendons, nonlinear couplings and rank defects
are refused. Live robot state, actuator gains and forces are never modified.
"""
import time

import numpy as np


class ControlKinematics:
    def __init__(self, model, snapshot, channels, robot_joints, excluded=()):
        import mujoco as mj
        self.mj, self.model, self.channels = mj, model, channels
        self.probe = mj.MjData(model)
        mj.mj_copyData(self.probe, model, snapshot)
        self.seed = snapshot.qpos.copy()
        self.indices = np.array([c['index'] for c in channels], dtype=int)
        self.joints = np.array([j for j in range(robot_joints) if j not in excluded
            and int(model.jnt_type[j]) in (int(mj.mjtJoint.mjJNT_HINGE), int(mj.mjtJoint.mjJNT_SLIDE))], dtype=int)
        self.qpos = model.jnt_qposadr[self.joints]
        self.dofs = model.jnt_dofadr[self.joints]
        self.limits = np.array([c['limits'] for c in channels])
        for actuator in self.indices:
            kind = int(model.actuator_trntype[actuator])
            if kind == int(mj.mjtTrn.mjTRN_TENDON):
                tendon = int(model.actuator_trnid[actuator, 0])
                a, n = int(model.tendon_adr[tendon]), int(model.tendon_num[tendon])
                if any(int(w) != int(mj.mjtWrap.mjWRAP_JOINT) for w in model.wrap_type[a:a+n]):
                    raise ValueError('Spatial tendon IK requires a separate source adapter')
            elif kind != int(mj.mjtTrn.mjTRN_JOINT):
                raise ValueError('Unsupported source position transmission')
        mj.mj_forward(model, self.probe)
        self.length = self.probe.actuator_length[self.indices].copy()
        jacobian = np.empty((len(channels), len(self.joints)))
        for column, address in enumerate(self.qpos):
            self.probe.qpos[:] = self.seed
            self.probe.qpos[address] += 1e-5
            mj.mj_forward(model, self.probe)
            jacobian[:, column] = (self.probe.actuator_length[self.indices]-self.length)/1e-5
        equations, residual = [], []
        positions = {int(j): i for i,j in enumerate(self.joints)}
        for index in range(model.neq):
            if not snapshot.eq_active[index]:
                continue
            kind = int(model.eq_type[index])
            if kind != int(mj.mjtEq.mjEQ_JOINT):
                raise ValueError('Non-joint equality requires a separate source IK adapter')
            a, b = int(model.eq_obj1id[index]), int(model.eq_obj2id[index])
            if a not in positions and b not in positions:
                continue
            coefficients = model.eq_data[index, :5]
            if np.any(np.abs(coefficients[2:]) > 1e-12):
                raise ValueError('Nonlinear source coupling requires a separate IK adapter')
            row = np.zeros(len(self.joints))
            qa = int(model.jnt_qposadr[a])
            delta = self.seed[qa]-model.qpos0[qa]-coefficients[0]
            if a in positions:
                row[positions[a]] = 1.
            if b >= 0:
                qb = int(model.jnt_qposadr[b])
                delta -= coefficients[1]*(self.seed[qb]-model.qpos0[qb])
                if b in positions:
                    row[positions[b]] -= coefficients[1]
            equations.append(row); residual.append(-delta)
        self.matrix = np.vstack([jacobian, *[e[None, :] for e in equations]])
        self.equality_rhs = np.array(residual)
        self.inverse = np.linalg.pinv(self.matrix, rcond=1e-10)
        # Reject a control group which cannot independently realize every channel.
        controllable = self.matrix @ self.inverse
        if not np.allclose(controllable[:len(channels), :len(channels)], np.eye(len(channels)), atol=1e-5):
            raise ValueError('Source controls are not independently reachable with these coupling constraints')
        gain = model.actuator_gainprm[self.indices, 0]
        bias = model.actuator_biasprm[self.indices]
        self.gain, self.bias = gain.copy(), bias.copy()
        if np.any(gain <= 0) or np.any(bias[:, 1] >= 0):
            raise ValueError('Bounded source position gains required')

    def forward(self, controls):
        controls = np.asarray(controls, dtype=float)
        if controls.shape != (len(self.channels),) or not np.all(np.isfinite(controls)):
            raise ValueError('Finite full source controls required')
        if np.any(controls < self.limits[:,0]) or np.any(controls > self.limits[:,1]):
            raise ValueError('IK control exceeds source limits')
        lengths = -(self.gain*controls+self.bias[:,0])/self.bias[:,1]
        rhs = np.concatenate((lengths-self.length, self.equality_rhs))
        delta = self.inverse @ rhs
        if not np.allclose(self.matrix @ delta, rhs, atol=2e-5):
            raise ValueError('Source actuator/coupling equations are inconsistent')
        self.probe.qpos[:] = self.seed
        self.probe.qpos[self.qpos] += delta
        limited = self.model.jnt_limited[self.joints].astype(bool)
        values = self.probe.qpos[self.qpos][limited]
        bounds = self.model.jnt_range[self.joints][limited]
        if np.any(values < bounds[:,0]-1e-6) or np.any(values > bounds[:,1]+1e-6):
            raise ValueError('IK reaches a coupled source joint limit')
        self.mj.mj_forward(self.model, self.probe)
        if not np.allclose(self.probe.actuator_length[self.indices], lengths, atol=2e-5):
            raise ValueError('Source transmission is not affine in joint positions')
        return self.probe

    def midpoint(self, bodies):
        centers = []
        for body in bodies:
            geoms = [g for g in range(self.model.ngeom) if self.model.geom_bodyid[g]==body
                     and (self.model.geom_contype[g] or self.model.geom_conaffinity[g])]
            if not geoms:
                raise ValueError('TCP requires physical collision pads on both selected bodies')
            centers.append(np.mean(self.probe.geom_xpos[geoms], axis=0))
        return np.mean(centers, axis=0)

    def midpoint_jacobian(self, bodies):
        jacobians = []
        for body in bodies:
            geoms = [g for g in range(self.model.ngeom) if self.model.geom_bodyid[g]==body
                     and (self.model.geom_contype[g] or self.model.geom_conaffinity[g])]
            if not geoms:
                raise ValueError('TCP requires physical collision pads')
            center = np.mean(self.probe.geom_xpos[geoms],axis=0)
            jacp, jacr = np.zeros((3,self.model.nv)), np.zeros((3,self.model.nv))
            self.mj.mj_jac(self.model,self.probe,jacp,jacr,center,body)
            jacobians.append(jacp[:,self.dofs])
        dq_dc = self.inverse[:,:len(self.channels)] * (-self.gain/self.bias[:,1])[None,:]
        return np.mean(jacobians,axis=0) @ dq_dc

    def reach(self, controls, bodies, target, movable, deadline=None, tolerance=.008):
        """Damped numerical IK over selected source controls; local, not OMPL."""
        from scipy.optimize import least_squares
        target = np.asarray(target, dtype=float)
        if target.shape != (3,) or not np.all(np.isfinite(target)):
            raise ValueError('Finite world XYZ reach target required')
        deadline = time.monotonic()+3. if deadline is None else deadline
        result = np.asarray(controls, dtype=float).copy()
        movable = list(movable)
        if not movable:
            raise ValueError('No source reach controls selected')
        def residual(values):
            if time.monotonic() > deadline:
                raise ValueError('Source IK planning deadline exceeded')
            result[movable] = values
            self.forward(result)
            return self.midpoint(bodies)-target
        lower, upper = self.limits[movable].T
        def jacobian(values):
            residual(values)
            return self.midpoint_jacobian(bodies)[:,movable]
        solved = least_squares(residual, np.clip(result[movable],lower,upper),
            bounds=(lower,upper), jac=jacobian, max_nfev=80)
        if np.linalg.norm(residual(solved.x)) > tolerance:
            raise ValueError('Object/target is outside this stopped-base reach; reposition the base')
        return result.copy()
