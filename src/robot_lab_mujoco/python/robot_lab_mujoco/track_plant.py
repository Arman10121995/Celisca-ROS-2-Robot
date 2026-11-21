"""Tracks for MuJoCo (R10.7): the shared track law on normal-only track contacts.

MuJoCo has no moving contact surface, so the track geoms get normal-only
contacts (condim 1) at a raised priority, as the MOLAR deck belt, and
``robot_lab_utils.track_drive`` supplies the tangential force at each
contact with the world. Forces go to ``qfrc_applied`` through
``mj_applyFT`` and are replaced, never accumulated, every step.
"""
import numpy as np

from robot_lab_utils.track_drive import TrackDrive, track_collisions


class TrackPlant:
    def __init__(self, model, data, description, config, base_body):
        import mujoco
        self.mj, self.model, self.data = mujoco, model, data
        self.base = int(base_body)
        root = int(model.body_rootid[self.base])
        self.robot_bodies = {b for b in range(model.nbody) if int(model.body_rootid[b]) == root}
        mass = float(model.body_subtreemass[root])
        self.drive = TrackDrive(config, mass)
        self.geoms = {}
        for name, side in track_collisions(description, self.drive.config).items():
            geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
            if geom < 0:
                raise ValueError('Track collision geometry missing from the compiled model: ' + name)
            self.geoms[geom] = side
        self.track_mask = np.zeros(model.ngeom, dtype=bool)
        self.track_mask[list(self.geoms)] = True
        self.robot_mask = np.zeros(model.nbody, dtype=bool)
        self.robot_mask[list(self.robot_bodies)] = True
        top = int(model.geom_priority.max()) + 1
        for geom in self.geoms:
            model.geom_priority[geom] = top
            model.geom_condim[geom] = 1
        self.applied = np.zeros(model.nv)
        self.wrench = np.zeros(6)
        self.velocity = np.zeros(6)
        self.report = dict(contacts=0, normal_N=0.0)

    def reset(self):
        self.drive.reset()
        self.applied[:] = 0.0

    def step(self, now, dt):
        """Before mj_step: replace last step's track forces with this step's."""
        data, model = self.data, self.model
        data.qfrc_applied[:] -= self.applied
        self.applied[:] = 0.0
        self.drive.advance(now, dt)
        ncon = data.ncon
        if ncon == 0:
            self.report = dict(contacts=0, normal_N=0.0)
            return
        contact = data.contact
        g1, g2 = np.asarray(contact.geom1), np.asarray(contact.geom2)
        mine1 = self.track_mask[g1]
        mine2 = self.track_mask[g2]
        own = np.where(mine1, g1, g2)
        other = np.where(mine1, g2, g1)
        rows = (mine1 != mine2) & ~self.robot_mask[model.geom_bodyid[other]] & (np.asarray(contact.efc_address) >= 0)
        if not rows.any():
            self.report = dict(contacts=0, normal_N=0.0)
            self.drive.forces([], np.zeros((0, 3)), [], np.zeros((0, 3)), np.zeros(3), dt)
            return
        own, rows_mine1 = own[rows], mine1[rows]
        # Normal-only (condim 1) contacts have one constraint row: the normal force.
        load = np.abs(data.efc_force[np.asarray(contact.efc_address)[rows]])
        # contact.frame[:3] points from geom1 to geom2; turn it into the robot.
        normals = np.asarray(contact.frame)[rows, :3] * np.where(rows_mine1, -1.0, 1.0)[:, None]
        points = np.asarray(contact.pos)[rows]
        bodies = model.geom_bodyid[own]
        velocities = np.empty_like(points)
        for body in np.unique(bodies):
            self.mj.mj_objectVelocity(model, data, self.mj.mjtObj.mjOBJ_BODY, int(body), self.velocity, 0)
            at = bodies == body
            # mj_objectVelocity reports a body's linear velocity at its centre
            # of mass (xipos), not its frame origin (xpos). Measuring from the
            # origin made a turn about the centre of mass look like one about
            # the origin, 0.07 m of uncorrected drift (2026-11-21).
            velocities[at] = self.velocity[3:] + np.cross(self.velocity[:3], points[at] - data.xipos[body])
        sides = [self.geoms[int(g)] for g in own]
        hull = data.xmat[self.base].reshape(3, 3)
        forces = self.drive.forces(sides, normals, load, velocities, hull[:, 1], dt,
                                   points, data.xpos[self.base], hull[:, 2])
        for body in np.unique(bodies):
            at = bodies == body
            centre = data.xpos[body]
            self.mj.mj_applyFT(model, data, forces[at].sum(axis=0), np.cross(points[at] - centre, forces[at]).sum(axis=0),
                               centre, int(body), self.applied)
        data.qfrc_applied[:] += self.applied
        self.report = dict(contacts=int(rows.sum()), normal_N=float(load.sum()))
