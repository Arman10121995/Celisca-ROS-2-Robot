# R10.4: the lab ROV on Isaac Sim (November 19, 2026)

**Status: implemented, experimental.** `lab_rov` runs on Isaac Sim's PhysX
behind the unchanged marine controller
(`marine_simulator.launch.py simulator:=isaac`), with the same water forces
as on MuJoCo and PyBullet. The ROV now runs on all four backends.

## What was added

- **`marine_isaac_runtime.py`**, the Isaac child process, run under the
  Isaac Sim Python:
  - one PhysX rigid body (the hull box, with the source mass, centre of
    mass and diagonal inertia) over a fixed seabed;
  - each 2 ms step applies the shared `marine_mujoco_plant.WaterForces`
    wrench about the centre of mass, through
    `RigidPrim.apply_forces_and_torques_at_pos`.
- **`marine_isaac_plant.py`**, the ROS side:
  - the same topics as the other plants (`run_plant`);
  - each period it sends the thruster forces and current over stdin and
    reads the pose back through a FIFO (Kit takes over the child's stdout);
  - it starts the child with the Isaac library preload the spawner already
    uses, and logs Kit's output to `$ROBOT_LAB_RUNTIME_ROOT`;
  - the FIFO is read non-blocking: a child that dies or stops answering
    fails the plant instead of hanging it.
- On this Jetson, GPU PhysX fails to load its CUDA modules (logged), so
  PhysX runs on the CPU.

**Offline parity** (10 s of surge plus yaw from a 0.3 rad spawn): Isaac and
MuJoCo end 2.5 cm apart, with yaw −1.96 against −1.81 rad.

**First launch failed** (`isaac-1-failed-runtime-path`): the installed
launcher script lives outside the package, so the runtime path was wrong
and the parent waited forever on the FIFO. Both are fixed.

## Measured (`rov_probe.py`, `marine_tank`; same probe as the other backends)

| Phase | Isaac | PyBullet | MuJoCo | Gazebo |
|---|---|---|---|---|
| Hold while arming | 8.2 cm | 7.9 cm | 5.7 cm | 2 cm |
| Surge (10 s at 0.2 m/s) | 1.038 m | 1.041 m | 1.030 m | 1.074 m |
| Yaw (6 s at 0.3 rad/s) | 1.452 rad | 1.439 rad | 1.437 rad | 1.495 rad |
| 15 s in a 0.1 m/s current | 3.2 cm | 3.5 cm | 0.9 cm | 1.0 cm |
| Operator loss | disarmed, thrust zero, sank 0.27 m in 6 s | 0.23 m | 0.23 m | 0.29 m |

The result is `isaac-2-result.json`. Launch logs are on SSD under
`robot_lab_runtime/marine-pybullet-20261118` (hashes here). The run shared
the host with Go2 policy training.

## Limits

- One run. The world is the seabed only; no sensors.
- PhysX on the CPU (GPU PhysX did not load on this host).
- `lab_rov` only; Tethys on Isaac remains.
