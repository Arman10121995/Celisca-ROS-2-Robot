#!/usr/bin/env python3
"""Isaac Sim child of the marine Isaac plant (ROADMAP R10.4).

Runs under the Isaac Sim Python, not ROS. ``marine_isaac_plant`` starts it
and exchanges line JSON: the configuration and then one command per tick
on stdin; one state per tick on the FIFO named by ``ISAAC_EVENT_FIFO``
(Kit takes over the child's stdout).

The vehicle is one PhysX rigid body: the hull box with the source mass,
centre of mass and diagonal inertia, over a fixed seabed. Each 2 ms step
it applies the ``marine_mujoco_plant.WaterForces`` wrench about the centre
of mass: the same ported Fortress buoyancy, hydrodynamics and thrusters as
the MuJoCo and PyBullet plants.
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
        from robot_lab_utils.marine_mujoco_plant import WaterForces

        class Forces(WaterForces):
            pass

        forces = Forces()
        forces._load(cfg['description'], cfg['surface_z'], cfg['bottom_z'], cfg['density'])
        centre = np.asarray(forces.model_source.centre, dtype=float)
        hull_xyz, hull_size = forces.boxes[0]
        x, y, z, yaw = cfg['spawn']
        R0 = np.array([[math.cos(yaw), -math.sin(yaw), 0.], [math.sin(yaw), math.cos(yaw), 0.], [0., 0., 1.]])
        world = World(physics_dt=TIMESTEP, stage_units_in_meters=1.0)
        world.scene.add(FixedCuboid(prim_path='/World/seabed', name='seabed',
                                    position=np.array([0., 0., cfg['bottom_z'] - .5]), scale=np.array([400., 400., 1.])))
        hull = DynamicCuboid(prim_path='/World/rov', name='rov',
                             position=np.array([x, y, z]) + R0 @ hull_xyz,
                             orientation=np.array([math.cos(yaw / 2), 0., 0., math.sin(yaw / 2)]),
                             scale=np.asarray(hull_size, dtype=float), mass=float(forces.model_source.mass))
        world.scene.add(hull)
        mass_api = UsdPhysics.MassAPI.Apply(world.stage.GetPrimAtPath('/World/rov'))
        # The cube prim is scaled: its centre-of-mass attribute is in the
        # unscaled unit-cube frame.
        offset = (centre - hull_xyz) / np.asarray(hull_size, dtype=float)
        mass_api.CreateCenterOfMassAttr().Set(Gf.Vec3f(*[float(v) for v in offset]))
        mass_api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(*[float(v) for v in forces.inertia]))
        mass_api.CreatePrincipalAxesAttr().Set(Gf.Quatf(1., 0., 0., 0.))
        world.reset()
        view = RigidPrim('/World/rov')
        clock = 0.
        events.write(json.dumps(dict(event='ready', scope='Isaac PhysX (CPU) rigid body with ported Fortress water forces')) + '\n')
        for line in sys.stdin:
            command = json.loads(line)
            if command.get('stop'):
                break
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
                force, torque = forces.water_wrench(R, origin, com, R.T @ linear_origin, R.T @ angular, TIMESTEP)
                view.apply_forces_and_torques_at_pos(forces=force[None, :], torques=torque[None, :],
                                                     positions=com[None, :], is_global=True)
                world.step(render=False)
                clock += TIMESTEP
            positions, quaternions = view.get_world_poses()
            R = _rotation(quaternions[0])
            origin = np.asarray(positions[0], dtype=float) - R @ hull_xyz
            w, qx, qy, qz = (float(v) for v in quaternions[0])
            events.write(json.dumps(dict(event='state', t=clock, position=origin.tolist(), quaternion_xyzw=[qx, qy, qz, w],
                                         wrench=forces.wrench.tolist())) + '\n')
    except Exception as exc:  # noqa: BLE001 - reported to the parent
        events.write(json.dumps(dict(event='error', message=repr(exc)[:500])) + '\n')
    finally:
        events.close()
        os._exit(0)


if __name__ == '__main__':
    main()
