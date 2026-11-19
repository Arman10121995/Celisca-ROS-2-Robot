# R10.5: a Tethys mission flown from its sensor estimate (November 19, 2026)

**Status: implemented, experimental; done-when met for one mission.** The
Tethys AUV planned and flew a route from its own sensor estimate on
Gazebo, and stopped 0.20 m from the goal by simulator truth.

## What ran (no new runtime code)

`auv_simulator.launch.py map_name:=marine_tank slam:=true spawn_z:=-3.0`
on Gazebo starts the existing pipeline:

- the estimator (`marine_sensors.py estimate`: IMU, pressure, DVL) on
  `/marine/estimated_odometry`;
- RTAB-Map mapping from the RGB-D kit;
- the mission controller (`marine_navigation.py`). It plans and follows
  from the estimate only, with a planned-route corridor and a
  flow-guidance envelope.

`auv_mission_probe.py` (new):

1. arms the vehicle;
2. sends `plan` to a target, then `execute` once the route is planned;
3. waits for a terminal mission status.

Simulator truth (`/marine/odometry`) is only recorded, to judge the result
and the estimate error.

## Measured

| Run | Route | Result | Estimate error (max) | Truth to goal at the end |
|---|---|---|---|---|
| 1 | spawn 1.5 m deep, target 2 m deep | not planned: "AUV route violates source-inflated depth clearance". The planner keeps radius 1.3 m + 3σ from the surface and the seabed, so the vehicle must be about 2–4 m deep in this 6 m tank. While the probe idled, the stationary estimate drifted 1.37 m in 600 s. | 1.37 m | — |
| 2 | 3 m deep, 6.5 m straight ahead | canceled after 22 s, 1.4 m short: "AUV course error exceeds flow-guidance envelope". It tracked within 6 cm for 3 m, then drifted 0.55 m left and 0.33 m up. | 0.13 m | 1.41 m |
| **3** | **3 m deep, 3.5 m straight ahead** | **completed** in 17.5 s: braked and settled ("signed-thrust braking settled; propulsion stopped") | **0.13 m** (σ 0.07 m) | **0.20 m** |

Run 2's divergence is the vehicle's own: a slow coupled yaw and pitch
drift under the cruise controller. The estimate stayed within 0.13 m of
truth throughout.

Results are in `mission-*.json`. Launch logs are on SSD under
`robot_lab_runtime/auv-mission-20261119` (hashes here). The runs shared the
host with Go2 policy training.

## Limits

- One completed short mission. The longer route diverged.
- Mapping runs online; relocalizing on a saved map
  (`localization:=true database:=...`) was not run.
- Ideal DVL and pressure emulation, not native sensor plugins.
- The tank is obstacle-free on this route; no obstacle avoidance was
  exercised.
