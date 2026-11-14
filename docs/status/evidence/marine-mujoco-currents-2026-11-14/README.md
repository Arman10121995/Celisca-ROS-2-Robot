# MuJoCo marine plant, current feedforward and volumetric AUV routes (R10.4 / R10.5)

Recorded November 14, 2026, on `master` after `cdbdbc73`, with this
delivery's working changes. Jetson AGX Orin, ROS Humble, Gazebo Fortress
6.18, MuJoCo (venv). `lab_rov` in `marine_tank`. Implemented, experimental;
R10.4 and R10.5 stay partial.

## What was added

- `robot_lab_utils/marine_mujoco_plant.py`, started with
  `marine_simulator.launch.py simulator:=mujoco`. It is a MuJoCo rigid body
  carrying ports of the Fortress systems that the ROV URDF declares:
  - **buoyancy:** collision-box displacement, clipped at the surface, applied
    at the centre of volume;
  - **hydrodynamics:** linear and quadratic damping on the current-relative
    velocity, plus added mass (the added-mass Coriolis terms are omitted);
  - **thrusters:** force at each source mount, within the source bounds.

  It consumes the same `/marine/thrusters/<joint>/force` topics and publishes
  `/marine/odometry` and `/clock`, so the unchanged `marine_controller` runs
  on it. World collision is a seabed plane only, with no sensors or SLAM.
- **`marine_controller` current feedforward:** the feedforward is now the
  declared damping at the current, `(lin + quad·|c|)·c` in the body frame.
  Before, it was the velocity gain times the current, which pushed the ROV
  upstream.
- **`auv_routes.plan_auv`, volumetric routes:** each Dubins curve gets a
  depth profile, solved by dynamic programming over 1.5 m stations. The
  profile climbs over or dives under obstacles within the 0.1 slope and the
  depth clearance, and stays on the straight ramp when the water is clear.
- **`auv_routes.plan_auv`, current-aware ranking:** with an operator-authored
  current, routes are ranked by travel time at 0.3 m/s through the water.
  Routes needing more than 0.5 rad of crab, or falling below 0.08 m/s ground
  speed, are refused. `marine_navigation` passes the fresh `/marine/current`
  to the planner.

## Results: the same probe on both backends (`rov_probe.py`)

The phases are: arm and hold for 12 s; surge at 0.2 m/s for 10 s; settle for
6 s; yaw at 0.3 rad/s for 6 s; hold for 10 s; hold in a 0.1 m/s +y current
for 15 s; then operator heartbeat loss for 6 s. Pose comes from
`/marine/odometry`, the plant observation on both backends.

| Phase | MuJoCo | Gazebo |
|---|---|---|
| Surge (10 s) | 1.030 m | 1.074 m |
| Yaw (6 s) | 1.437 rad | 1.495 rad |
| Depth deviation while manoeuvring | ≤ 4 mm | ≤ 1 mm |
| 15 s in 0.1 m/s current, earlier feedforward (runs 1) | 0.370 m **upstream** | 0.388 m upstream |
| 15 s in 0.1 m/s current, damping feedforward (runs 2) | **0.009 m** | **0.010 m** |
| Operator loss | disarmed, thrust zero, sank 0.23 m in 6 s | disarmed, thrust zero, sank 0.29 m |

The first hold drifted 6 cm on MuJoCo and 2 cm on Gazebo while arming.

Volumetric and current-aware AUV routing has unit tests only: a seabed block
climbed within the slope, a straight ramp in clear water, time ranking, and
crab refusal. No AUV mission was flown.

## Retained failures

- The plant first failed to start: `/usr/bin/python3 -s` hides the MuJoCo
  venv. It now uses `/usr/bin/env python3`.
- Runs 1 show the earlier feedforward error on both backends: the ROV drifted
  upstream at about 0.025 m/s. Runs 2 are after the fix.
- A first reef unit test put the obstacle too close to climb at a 0.1 slope.
  The test geometry was corrected; the planner was right to refuse it.
- One Gazebo server (`ign gazebo`) outlived its launch's SIGINT. It was
  stopped by its exact PID.

## Reproduce

```bash
python3 launch_marine.py <dir>/launch.log simulator:=mujoco map_name:=marine_tank   # or omit simulator for Gazebo
CASE_OUTPUT=<dir>/run CASE_BACKEND=mujoco python3 rov_probe.py
```

Run with an isolated `ROS_DOMAIN_ID`, then stop the launch's process group.
Traces and logs are in
`/workspace/molar/robot_lab_runtime/marine-mujoco-20261114/`.

## Not covered

- No marine adapters for PyBullet or Isaac.
- No AUV (Tethys) MuJoCo plant: it has fins, not only thrusters.
- No sensor-only AUV localization mission on a saved map.
- No added-mass Coriolis terms.
- The BlueROV2 candidate still awaits its license review (R9.1).
