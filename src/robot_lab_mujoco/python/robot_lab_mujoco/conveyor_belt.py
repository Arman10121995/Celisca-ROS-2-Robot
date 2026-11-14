"""A belt surface for MuJoCo: bodies resting on a deck are carried along it (R11 payload transfer).

MuJoCo has no moving-surface contact (Gazebo's TrackController equivalent),
so the belt owns the deck's friction instead: the deck geoms get zero
sliding friction and a higher contact priority (MuJoCo then uses their
friction for every deck contact), and every free body touching the deck gets
a horizontal force driving its velocity relative to the deck towards the
belt velocity (belt speed along the deck's x axis, zero across it), bounded
by Coulomb friction (``mu`` times the contact's normal force). A stopped belt
therefore still grips. The belt speed is the measured roller angular
velocity times the roller radius. Forces are written to ``xfrc_applied`` and
replaced, never accumulated, every step.
"""
import numpy as np


class ConveyorBelt:
    def __init__(self, model, data, deck_body, roller_joint, roller_radius, mu=0.8, gain=20.0, robot_root=None):
        import mujoco
        self.mj, self.model, self.data = mujoco, model, data
        self.deck = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, deck_body)
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, roller_joint)
        if self.deck < 0 or joint < 0:
            raise ValueError('Belt deck body or roller joint missing: %s, %s' % (deck_body, roller_joint))
        self.roller_dof = int(model.jnt_dofadr[joint])
        self.radius, self.mu, self.gain = float(roller_radius), float(mu), float(gain)
        self.deck_geoms = {g for g in range(model.ngeom) if model.geom_bodyid[g] == self.deck}
        top = int(model.geom_priority.max()) + 1
        for geom in self.deck_geoms:
            model.geom_priority[geom] = top
            model.geom_friction[geom, 0] = 0.
        root = robot_root if robot_root is not None else int(model.body_rootid[self.deck])
        self.robot_bodies = {b for b in range(model.nbody) if int(model.body_rootid[b]) == root}
        self.applied = {}          # body -> force we added last step
        self.force = np.zeros(6)

    def speed(self):
        return float(self.data.qvel[self.roller_dof]) * self.radius

    def step(self):
        """Replace last step's belt forces with this step's; returns {body: force_x_along_deck}."""
        for body, force in self.applied.items():
            self.data.xfrc_applied[body, :3] -= force
        self.applied = {}
        belt = self.speed()
        axis = self.data.xmat[self.deck].reshape(3, 3)[:, 0].copy()
        axis[2] = 0.
        norm = np.linalg.norm(axis)
        if norm < 1e-6:
            return {}
        axis /= norm
        normal = {}
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            g1, g2 = int(contact.geom1), int(contact.geom2)
            if (g1 in self.deck_geoms) == (g2 in self.deck_geoms):
                continue
            other = int(self.model.geom_bodyid[g2 if g1 in self.deck_geoms else g1])
            if other in self.robot_bodies or other == 0:
                continue
            self.mj.mj_contactForce(self.model, self.data, i, self.force)
            normal[other] = normal.get(other, 0.) + abs(float(self.force[0]))
        report = {}
        for body, n in normal.items():
            # Relative velocity along the deck axis (cvel: angular then linear, at the body's CoM frame origin).
            velocity = np.zeros(6)
            self.mj.mj_objectVelocity(self.model, self.data, self.mj.mjtObj.mjOBJ_BODY, body, velocity, 0)
            deck_velocity = np.zeros(6)
            self.mj.mj_objectVelocity(self.model, self.data, self.mj.mjtObj.mjOBJ_BODY, self.deck, deck_velocity, 0)
            relative = velocity[3:] - deck_velocity[3:]
            relative[2] = 0.
            slip = belt * axis - relative
            mass = float(self.model.body_subtreemass[body])
            force = self.gain * mass * slip
            limit = self.mu * n
            size = float(np.linalg.norm(force))
            if size > limit:
                force *= limit / size
            self.data.xfrc_applied[body, :3] += force
            self.applied[body] = force
            report[body] = float(np.dot(force, axis))
        return report
