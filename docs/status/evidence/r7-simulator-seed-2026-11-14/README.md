# Simulator-applied seeds and seeded sensor faults on every backend (R7, R6.2)

November 14, 2026, Claude. Implemented, experimental.

## Gazebo noise seed

`sim_seed:=<n>` (`simulated_robot.launch.py`, `gazebo.launch.py`) adds the
`robot_lab::SimulationSeed` world system (`robot_lab_maps/systems/simulation_seed.cc`).
At world load it calls `ignition::math::Rand::Seed(n)`. gz-sensors draws all
Gaussian sensor noise (IMU, lidar, cameras) from that generator.

`scripts/gazebo_seed_check.py` runs a headless world holding one noisy IMU,
built through the launch's own `world_with_seed`. Each run starts paused,
subscribes, unpauses, and compares 300 samples by simulation stamp
([result.json](result.json)):

| Runs | Identical samples |
|---|---|
| seed 42 vs seed 42 | 300 of 300 |
| seed 42 vs seed 7 | 0 of 300 |
| no seed vs no seed | 0 of 300 |

Without the system, Gazebo's noise differs on every run. With it, the
same seed repeats the noise sample for sample.

This was measured on one IMU in a static world. A multi-sensor world whose
rendering sensors draw from the shared generator on other threads has not
been checked for repeatability.

## MuJoCo, PyBullet and Isaac

These three bridges publish exact sensors: they have no simulator-side noise
to seed. Their seeded noise is the sensor-fault relay, which now runs on
every backend. `sensor_faults:=<scenario>` remaps the simulator's own sensor
publishers onto the relay inputs (`sensor_fault_relay.launch_actions`).
Before, the launch refused every backend except Gazebo.

The relay script was also never installed. `sensor_faults:=` therefore failed
at launch on Gazebo too ("executable 'sensor_fault_relay.py' not found");
unit tests had not caught it. It is now installed.

Smoke run: Bumperbot on `nav_obstacle`, scenario [fault_scenario.yaml](fault_scenario.yaml)
(+0.5 m bias on `/scan`), one sample from each topic:

| Simulator | `/scan_clean` publisher | `/scan` publisher | Median range clean → faulted |
|---|---|---|---|
| MuJoCo | mujoco_spawner | sensor_fault_relay | 1.711 → 2.211 m |
| Gazebo | ros_gz_bridge | sensor_fault_relay | 1.732 → 2.232 m |

The first trace rows are in [gazebo_fault_trace_head.jsonl](gazebo_fault_trace_head.jsonl)
and [mujoco_fault_trace_head.jsonl](mujoco_fault_trace_head.jsonl). PyBullet
and Isaac use the same remapping but were not run.

## Not shown

- Actor reseeding: Gazebo actors follow scripted trajectories, and SUMO trips
  carry their own seed.
- A degraded mission, or two faulted runs compared trace for trace (V9).
