# R8.2 Isaac Sim on Jetson AGX Orin, with a cross-backend sensor probe (2026-09-16)

Host: [`host.json`](host.json) — Jetson AGX Orin, L4T R36.5.2, CUDA 12.6,
Isaac Sim 6.0.1 source build under its Python 3.12, ROS 2 Humble. PhysX runs
on the CPU (its CUDA module fails to load with error 222).

## Sensor probe: PyBullet, MuJoCo and Isaac against the same map

`scripts/sim_sensor_probe.sh <backend> nav_maze probe_<backend>.json --camera --reset`
launches Bumperbot in display mode and checks the published topics against
the SDF world, using the laser and camera link poses from the robot
description (`probe_*.json`, launch logs beside them).

| Check | PyBullet | MuJoCo | Isaac Sim |
|---|---|---|---|
| Rest pose z / roll, pitch (rad) | 0.000 / 0.00, -0.01 | 0.000 / 0.00, 0.004 | 0.000 / 0.00, 0.01 |
| `/scan` rays within 5 cm of the map ray-cast | 360/360 (median 0.000 m) | 360/360 (0.000 m) | 360/360 (0.000 m) |
| Camera centre depth vs map ray-cast | 7.342 m vs 7.342 m | 7.339 m vs 7.339 m | 7.340 m vs 7.338 m |
| Colour image mean / non-black pixels | 186 / 1.00 | 18 / 0.75 | 164 / 0.98 |
| 0.3 m/s straight: speed, yaw rate, twist abs vy | 0.298 m/s, -0.000, 0.0000 | 0.292 m/s, 0.001, 0.0002 | 0.298 m/s, -0.003, 0.0044 |
| 0.15 m/s + 0.6 rad/s turn: speed, yaw rate | 0.137 m/s, 0.496 rad/s | 0.134 m/s, 0.588 rad/s | 0.135 m/s, 0.587 rad/s |
| `/robot_lab/reset`: distance from start after, clock monotonic | 0.000 m, yes | 0.000 m, yes | 0.000 m, yes |

Speeds and yaw rates are averaged over the last 2.5 s of a 3 s sim-time
command, from ground-truth poses.  MuJoCo's colour image is dim (its default
scene lighting), not empty.  PyBullet turns at 83 % of the commanded rate in
this run.  Only box geometry is ray-cast, so the comparison needs a box arena.

## Isaac wheel drive: what was wrong

With the velocity drives correctly authored (damping 1e4, unbounded force,
targets set), Bumperbot's wheel joint velocities swung between -24 and
+31 rad/s for a 9.09 rad/s target and the robot veered. `drive_variants.txt`
holds the offline runs of `isaac_drive_diag.py` (no ROS; 3 s phases,
averages over the last 2 s):

| Colliders | Wheel armature | Merge fixed joints | Straight (0.3 m/s): speed, yaw rate | Turn (0.6 rad/s): yaw rate |
|---|---|---|---|---|
| visual meshes | 0 | no | erratic wheels (see file) | erratic |
| URDF `<collision>` | 0 | no | erratic wheels (see file) | erratic |
| visual meshes | 0.005 | no | 0.307 m/s, 0.103 rad/s drift | 0.620 |
| visual meshes | 0.005 | yes | 0.306 m/s, -0.046 rad/s drift | 0.595 |
| URDF `<collision>` | 0.005 | no | 0.299 m/s, 0.000 | 0.595 |
| URDF `<collision>` | 0.005 | yes | 0.300 m/s, 0.000 | 0.590 |

Two causes: the 53 g wheels (about 2e-5 kg m^2 about the axle) make the
velocity drive unstable without rotor inertia, and colliders built from the
visual meshes give faceted convex-hull wheels that bounce and drift. The
runtime now authors `physxJoint:armature` 0.005 on driven wheel joints, and
the spawner imports the description's own `<collision>` geometry by default,
as Gazebo, PyBullet and MuJoCo do (every robot description has collision
geometry on its structural links).
