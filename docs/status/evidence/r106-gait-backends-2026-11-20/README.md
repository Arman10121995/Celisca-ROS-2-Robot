# R10.6: multi-leg saved-map goals on Gazebo and Isaac, a stance footprint and a lidar self-filter (November 20, 2026)

**Status: implemented, experimental.** This is the first saved-map Nav2 run
of the multi-leg gait on Gazebo and Isaac.

- **Gazebo:** the octopod and centipede pass. The hexapod reaches its goal
  by Nav2 but is 0.157 m from it by truth, outside the 0.15 m contract.
- **Isaac:** the octopod maps, but its navigation is cut off by the 900 s
  workload limit while still closing in, 0.113 m away.

Three gait-navigation changes came out of these runs:

- a stance-polygon footprint;
- a lidar self-filter;
- run budgets for Isaac and Gazebo, and a thinner workload trace.

Each was re-checked on MuJoCo.

## Changes

- **Stance footprint** (`gait_pipeline.leg_footprint`).
  - Before: the costmaps used a circle about the root of radius
    (farthest foot + 0.15 m): 0.55 m for the hexapod, whose root is a body
    corner, and 1.09 m for the centipede, whose root is its head.
  - Now: both costmaps use the convex hull of the feet, each widened by
    0.15 m. The serpentine models keep their radius.
- **Lidar self-filter** (`gait_sensor_relay.py`, `self_filter_polygon`).
  - Returns that end inside the stance polygon (feet + 0.05 m) are set to
    +inf. The pipeline passes the polygon.
  - Gazebo's GPU lidar renders the robot's own legs. The hexapod's Gazebo
    map had about 45,000 known cells against about 35,000 for the others,
    and its start read as lethal.
- **Qualifier budgets** (`scripts/qualify_gait_saved_map.py`):
  - `--max-rss-mb`, default 12288 for Isaac: its runtime alone uses
    about 6 GB.
  - `--max-cpu-percent`, default 1200 for Isaac and Gazebo: start-up
    peaks reached 907 % and 1049 %.
  - `--max-log-mb`, default 512 for Isaac.
  - `--map-timeout`, default 600 s for Isaac: it had reached 6 s of
    simulation in 120 s.
- **Workload trace** (`extension_workload.py`). `native_state` is recorded
  once per wall second, and on every change of arming, owner or reason.
  The 10 Hz copies filled 30 of the 32 MiB trace after 37 s of simulated
  time on Isaac. The trace now stays at 6 MiB after 41 s.
- **Fix during the work.** A shadowed `json` import made the first launches
  with the self-filter fail at once. A new test builds the mapping
  pipeline offline. Those launches held no measurements and were removed.

## Measured (`nav_empty` unless noted, goal 0.5 m ahead, 0.15 m / 0.15 rad by truth, one run each)

| Backend | Model | Changes in effect | Result | End to goal | Heading |
|---|---|---|---|---|---|
| Gazebo | octopod | root-frame turn | **passed** | 0.069 m | 0.002 rad |
| Gazebo | centipede | root-frame turn | **passed** | 0.057 m | 0.145 rad |
| Gazebo | hexapod | root-frame turn | not passed: "Starting point in lethal space" | — | — |
| Gazebo | hexapod | + footprint | not passed: lethal start again | — | — |
| Gazebo | hexapod | + self-filter | not passed: a start-up CPU spike (1049 %) exceeded the 800 % budget | — | — |
| Gazebo | hexapod | + 1200 % budget | **not passed**: Nav2 "Goal succeeded"; truth 0.157 m from the goal | 0.157 m | — |
| Isaac | all three | root-frame turn | not passed: budget exceeded at start-up (RSS 6 GB) | — | — |
| Isaac | all three | + RSS budget | not passed: CPU 907 % > 800 % | — | — |
| Isaac | all three | + CPU budget | not passed: the 120 s mapping window ended after 6 s of simulation | — | — |
| Isaac | octopod | + map timeout | **mapped** (32,898 cells); navigation exceeded the 64 MB artifact budget | — | — |
| Isaac | octopod | + log budget, 900 s | mapped; trace exceeded 32 MiB (36.6 s of simulation) | — | — |
| Isaac | octopod | + thinned trace | **mapped** (32,986 cells); navigation **timed out at 900 s**, 0.113 m away and still closing (41.3 s of simulation, RTF ≈ 0.05) | 0.113 m | 0.002 rad |
| MuJoCo | hexapod / octopod / centipede | + footprint | passed | 0.086 / 0.080 / 0.142 m | 0.010 / 0.001 / 0.136 rad |
| MuJoCo | hexapod | + self-filter | passed | 0.103 m | 0.007 rad |
| MuJoCo, `nav_obstacle` world (corner) spawn | **centipede** | + footprint | **passed** | 0.125 m | 0.120 rad |
| MuJoCo, `nav_obstacle` world spawn | **hexapod** | + footprint | **passed** | 0.107 m | 0.120 rad |

- **Corner spawn.** On 11-20 the centipede's corner start was lethal ("start
  in lethal space"), and the hexapod stopped 0.152 m short. Both pass now:
  the polygon replaces the 1.09 m and 0.55 m circles.
- **Gazebo hexapod.** Nav2's 0.1 m goal tolerance plus about 0.06 m of
  estimate error put the robot 0.157 m away by truth. The centipede failed
  the same way on 11-19. The tolerances were not changed.

## Limits

- One run per case; the centipede on Gazebo ran before the footprint and
  the self-filter.
- Isaac runs at about 0.05× real time on this host; a 0.5 m goal does not
  fit the 900 s workload limit. The hexapod and centipede were not run on
  Isaac after the budget changes.
- The self-filter is a static polygon; it does not follow the swinging
  legs (the 0.05 m margin covers the ±0.0225 m stride).

## Reproduce

```bash
python3 scripts/qualify_gait_saved_map.py --output <new dir> --simulator gazebo --timeout 600 \
  --robots lab_octopod hexapod_ros2 lab_centipede
python3 scripts/qualify_gait_saved_map.py --output <new dir> --simulator isaac --timeout 920 --robots lab_octopod
python3 scripts/qualify_gait_saved_map.py --output <new dir> --map nav_obstacle --robots lab_centipede hexapod_ros2
```

`*-report.json` are the qualifier reports, named as in the runtime folder
(`gazebo`/`isaac` first pass, `fp-*` footprint, `sf*-*` self-filter and
budgets). Logs are on SSD under
`robot_lab_runtime/gait-savedmap-backends-20261120` (hashes in
`runtime-sha256.txt`).
