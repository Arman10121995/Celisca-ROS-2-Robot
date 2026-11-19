# R10.6: saved-map Nav2 goals for every multi-leg model (November 19, 2026)

**Status: implemented, experimental.** Each gait model maps `nav_empty` on
MuJoCo and saves the map. In a fresh launch it navigates to a goal on that
saved map, and the endpoint is judged by independent physical truth.

| Model | Saved map | Passing runs |
|---|---|---|
| `hexapod_ros2` | 32,502 known cells | 1 of 1 |
| `lab_octopod` | 32,680 known cells | 1 of 1 |
| `lab_centipede` | 32,650 / 32,704 known cells | 1 of 2 |

The October 27 hexapod Nav2 case used the world's static map. This is the
first saved-map goal for these models.

## How it ran (`scripts/qualify_gait_saved_map.py`, new)

The script uses the Experiments runner (`ConcurrentRunner`, an isolated
ROS domain and partition per run) with the same extension workloads as
**Gait map acquisition** and **Gait navigation**.

1. **Map:** `gait_simulator.launch.py task:=slam`. The workload drives the
   model at least 0.2 m and saves the acquired grid through
   `/gait/mapping/save`. It succeeds only with physical travel and a
   non-empty archived map.
2. **Navigate:** `task:=nav map_yaml:=<that saved map>`, with one Nav2 goal
   0.5 m ahead along the measured heading after acquisition. The workload
   passes only when the body, by independent `/odom/ground_truth`, settles
   within 0.15 m and 0.15 rad, below 0.03 m/s and 0.05 rad/s, for 1
   simulated second. Nav2's own "Goal succeeded" does not count.

No runtime code changed.

## Measured (MuJoCo, `nav_empty`)

| Model | Run | Saved map (known cells) | Endpoint error | Result |
|---|---|---|---|---|
| `hexapod_ros2` | 1 | 32,502 | 0.090 m, 0.025 rad | settled (83 s) |
| `lab_octopod` | 1 | 32,680 | 0.081 m, 0.001 rad | settled (49 s) |
| `lab_centipede` | 1 | 32,650 | 0.157 m, 0.136 rad | **not passed**: Nav2 reported "Goal succeeded" after 52 s, but truth was 0.157 m from the goal (contract 0.15 m), so the workload timed out |
| `lab_centipede` | 2 | 32,704 | 0.119 m, 0.103 rad | settled (52 s) |

The failed centipede run's 0.16 m gap is its pose estimate's error at the
goal; the robot stopped where it believed the goal was.

Results are `<model>-<run>-*.json` and `summary.json`. Run directories
(launch logs, traces, telemetry) are on SSD under
`robot_lab_runtime/gait-saved-map-20261119-*`.

## Limits

- One world (open `nav_empty`), one backend, one short goal per run. The
  hexapod and octopod ran once each.
- Terrain tuning and slip-recovery missions for each model are still open.
