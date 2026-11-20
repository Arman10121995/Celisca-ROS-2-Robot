#!/usr/bin/env python3
"""Isaac Sim child of the marine Isaac plant (ROADMAP R10.4).

Runs under the Isaac Sim Python, not ROS. ``marine_isaac_plant`` starts it
and exchanges line JSON: the configuration and then one command per tick
on stdin; one state per tick on the FIFO named by ``ISAAC_EVENT_FIFO``
(Kit takes over the child's stdout).

The vehicle is one PhysX rigid body: the hull box with the source mass,
centre of mass and inertia, over a fixed seabed. Each 2 ms step it applies
the wrench about the centre of mass: ``marine_mujoco_plant.WaterForces``
for the ROV (``kind`` rov) or ``marine_tethys_plant.TethysForces`` with its
fin and propeller states for the Tethys AUV (``kind`` tethys), the same
ported forces as the MuJoCo and PyBullet plants.
"""
import json
import math
import os
import sys

TIMESTEP = .002


def _rotation(quaternion_wxyz):
    import numpy as np
    w, x, y, z = quaternion_wxyz
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _quaternion_wxyz(R):
    """Unit quaternion (w, x, y, z) of a rotation matrix (no scipy in every Isaac Python)."""
    trace = R[0][0] + R[1][1] + R[2][2]
    if trace > 0:
        k = 2 * math.sqrt(trace + 1.)
        return (.25 * k, (R[2][1] - R[1][2]) / k, (R[0][2] - R[2][0]) / k, (R[1][0] - R[0][1]) / k)
    i = max(range(3), key=lambda j: R[j][j])
    j, k_ = (i + 1) % 3, (i + 2) % 3
    k = 2 * math.sqrt(1. + R[i][i] - R[j][j] - R[k_][k_])
    q = [0., 0., 0.]
    q[i] = .25 * k
    q[j] = (R[j][i] + R[i][j]) / k
    q[k_] = (R[k_][i] + R[i][k_]) / k
    return ((R[k_][j] - R[j][k_]) / k, q[0], q[1], q[2])


def main():
    cfg = json.loads(sys.stdin.readline())
    for path in cfg.get('pythonpath', []):
        sys.path.insert(0, path)
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True})
    events = open(os.environ['ISAAC_EVENT_FIFO'], 'w', buffering=1)
    try:
        import numpy as np
        from pxr import Gf, UsdPhysics
        from isaacsim.core.api import World
        from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
        from isaacsim.core.prims import RigidPrim
        tethys = cfg.get('kind') == 'tethys'
        if tethys:
            # The Tethys AUV: propeller, fins and its own hydrodynamics
            # (marine_tethys_plant.TethysForces), from the source model.sdf.
            from robot_lab_utils.marine_tethys_plant import TethysForces
            forces = TethysForces()
            forces._setup(cfg['description'], cfg['surface_z'], cfg['density'], cfg.get('coriolis', 'off'))
            mass, centre = float(forces.source['mass']), np.asarray(forces.source['centre'], dtype=float)
            eigen, principal = np.linalg.eigh(forces.source['inertia'])
            if np.linalg.det(principal) < 0:
                principal[:, 0] = -principal[:, 0]
            hull_xyz, hull_size = forces.source['boxes'][0]
        else:
            from robot_lab_utils.marine_mujoco_plant import WaterForces
            from robot_lab_utils.marine_model import SurfaceModel
            forces = WaterForces()
            if cfg.get('surface'):
                forces.model_class = SurfaceModel
            forces._load(cfg['description'], cfg['surface_z'], cfg['bottom_z'], cfg['density'])
            mass, centre = float(forces.model_source.mass), np.asarray(forces.model_source.centre, dtype=float)
            eigen, principal = np.asarray(forces.inertia, dtype=float), np.eye(3)
            hull_xyz, hull_size = forces.boxes[0]
        hull_xyz, hull_size = np.asarray(hull_xyz, dtype=float), np.asarray(hull_size, dtype=float)
        x, y, z, yaw = cfg['spawn']
        R0 = np.array([[math.cos(yaw), -math.sin(yaw), 0.], [math.sin(yaw), math.cos(yaw), 0.], [0., 0., 1.]])
        world = World(physics_dt=TIMESTEP, stage_units_in_meters=1.0)
        world.scene.add(FixedCuboid(prim_path='/World/seabed', name='seabed',
                                    position=np.array([0., 0., cfg['bottom_z'] - .5]), scale=np.array([400., 400., 1.])))
        hull = DynamicCuboid(prim_path='/World/rov', name='rov',
                             position=np.array([x, y, z]) + R0 @ hull_xyz,
                             orientation=np.array([math.cos(yaw / 2), 0., 0., math.sin(yaw / 2)]),
                             scale=hull_size, mass=mass)
        world.scene.add(hull)
        mass_api = UsdPhysics.MassAPI.Apply(world.stage.GetPrimAtPath('/World/rov'))
        # The cube prim is scaled: its centre-of-mass attribute is in the
        # unscaled unit-cube frame.
        offset = (centre - hull_xyz) / hull_size
        mass_api.CreateCenterOfMassAttr().Set(Gf.Vec3f(*[float(v) for v in offset]))
        mass_api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(*[float(v) for v in eigen]))
        w_, x_, y_, z_ = _quaternion_wxyz(principal)
        mass_api.CreatePrincipalAxesAttr().Set(Gf.Quatf(float(w_), float(x_), float(y_), float(z_)))
        world.reset()
        view = RigidPrim('/World/rov')
        clock = 0.
        events.write(json.dumps(dict(event='ready', scope='Isaac PhysX (CPU) rigid body with ported Fortress water forces')) + '\n')
        for line in sys.stdin:
            command = json.loads(line)
            if command.get('stop'):
                break
            if tethys:
                forces.command_thrust(float(command['thrust']))
                for name, target in command['fins'].items():
                    forces.command_fin(name, float(target))
            else:
                forces.forces[:] = command['forces']
            forces.current = np.asarray(command.get('current', [0., 0., 0.]), dtype=float)
            for _ in range(max(1, int(round(float(command['dt']) / TIMESTEP)))):
                positions, quaternions = view.get_world_poses()
                velocities = view.get_velocities()[0]
                R = _rotation(quaternions[0])
                box_centre = np.asarray(positions[0], dtype=float)
                origin = box_centre - R @ hull_xyz
                com = origin + R @ centre
                linear_com, angular = np.asarray(velocities[:3], dtype=float), np.asarray(velocities[3:], dtype=float)
                linear_origin = linear_com + np.cross(angular, origin - com)
                if tethys:
                    force, torque = forces.tethys_wrench(R, origin, com, R.T @ linear_origin, R.T @ angular)
                else:
                    force, torque = forces.water_wrench(R, origin, com, R.T @ linear_origin, R.T @ angular, TIMESTEP)
                view.apply_forces_and_torques_at_pos(forces=force[None, :], torques=torque[None, :],
                                                     positions=com[None, :], is_global=True)
                world.step(render=False)
                if tethys:
                    forces.advance_joints()
                clock += TIMESTEP
            positions, quaternions = view.get_world_poses()
            R = _rotation(quaternions[0])
            origin = np.asarray(positions[0], dtype=float) - R @ hull_xyz
            w, qx, qy, qz = (float(v) for v in quaternions[0])
            events.write(json.dumps(dict(event='state', t=clock, position=origin.tolist(), quaternion_xyzw=[qx, qy, qz, w],
                                         wrench=forces.wrench.tolist(),
                                         **(dict(joints=forces.joints(), fins=forces.fin_angle) if tethys else {}))) + '\n')
    except Exception as exc:  # noqa: BLE001 - reported to the parent
        events.write(json.dumps(dict(event='error', message=repr(exc)[:500])) + '\n')
    finally:
        events.close()
        os._exit(0)


if __name__ == '__main__':
    main()
