# R8.2 live R4 mission — Isaac Sim on the Jetson, 5 evaluation seeds (2026-09-16)

Full R4 `point_to_point_navigation` missions driven through the live stack
(no test doubles) on the Isaac Sim backend, one isolated ROS domain per seed,
so the numbers below are simulation measurements rather than unit results.
This is the mission half of R8.2 acceptance; the startup/drive/sensor/reset
half is in `../r82-isaac-jetson-2026-09-16/`.

- Harness: `src/robot_lab/robot_lab_benchmark/robot_lab_benchmark/live_mission.py`
  (`robot-lab-live-mission`, opt in with `ROBOT_LAB_RUN_LIVE_MISSION=1`;
  launch it as `ros2 run robot_lab_benchmark robot-lab-live-mission`, the
  repo's convention — `source install/setup.bash` does not put the console
  script on `PATH`)
- Robot / world / goal: bumperbot, `nav_empty`, 1.5 m ahead of the spawn pose
- Seeds: 1201–1205 (evaluation seeds; R4.3 requires >= 5 and these do not
  overlap the MuJoCo 1001–1005 or PyBullet 1101–1105 sets)
- Revision: `be1a131` (clean tree, 2026-09-16)
- Host: see `../r82-isaac-jetson-2026-09-16/host.json` — Jetson AGX Orin,
  L4T R36.5.2, Isaac Sim 6.0.1 under its own Python 3.12, PhysX on the CPU
  (its CUDA module fails to load with error 222)
- Backends run sequentially so a concurrent recording cannot skew the RTF

## Reproduce

```bash
cd <ws> && source /opt/ros/humble/setup.bash && source install/setup.bash
export ROBOT_LAB_RUN_LIVE_MISSION=1 ROS_LOCALHOST_ONLY=1
ros2 run robot_lab_benchmark robot-lab-live-mission \
    --simulator isaac --seeds 1201 1202 1203 1204 1205 \
    --goal 1.5 --world nav_empty --timeout 40.0 --domain 200
```

The harness launches `robot_lab_isaac/isaac_simulator.launch.py`
(`gui:=false`, `world_name:=nav_empty`, `robot_xacro:=bumperbot/urdf/bumperbot.urdf.xacro`),
waits for `/clock` and `/odom/ground_truth` on the isolated domain, drives the
waypoint controller on `/cmd_vel`, and records a real `rosbag2` capture of
`/clock /odom/ground_truth /scan /cmd_vel`. Per-topic bag counts are parsed
from the bag's `metadata.yaml` by the harness itself, so recorded provenance
is recoverable from these committed files.

## Reproduce through the test entry point

`src/robot_lab/robot_lab_benchmark/test/test_live_mission.py::test_live_mission_isaac_seed1201`
(added 2026-09-16, opt-in via `ROBOT_LAB_RUN_LIVE_MISSION=1`, mirroring the
MuJoCo 1001 and PyBullet 1101 live tests) runs seed 1201 through the same
`ros2 run` route and asserts the truthful outcome plus the measured-metric and
bag-provenance fields:

```bash
cd <ws> && source /opt/ros/humble/setup.bash && source install/setup.bash
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROBOT_LAB_RUN_LIVE_MISSION=1
python3 -m pytest src/robot_lab/robot_lab_benchmark/test/test_live_mission.py -k isaac
# 1 passed, 20 deselected in 70.65 s (2026-09-16)
```

The default suite still skips it (`20 passed, 3 skipped`), so a normal test run
never touches a ROS graph.

## Artifacts

- `summary_isaac.json` — all 5 run records, verbatim harness output
- `stats.json` — distributions per metric plus a `per_seed` breakdown;
  every number cited in `docs/status/platform-status.yaml` comes from here
- `seed_<n>/` — that seed's `manifest.json`, `metrics.json`, first 40 lines
  of `trace.csv` and `launch.log`, and its `progress.log`

Raw `trace.csv` (one row per command tick) and the `bags/*.db3` recordings
stayed in `/tmp/r82_isaac_mission/`; they are large and fully regenerable
with the command above. All five runs were executed fresh in one batch on
2026-09-16 against the same build.

## Result

All 5 seeds reached the goal: outcome `success` (R4.1 truthful outcome, not a
collapsed boolean), 0 contact events, no collisions, no timeouts. Trajectory
distance 1.3507 m ± 0.0004 m (min 1.3501, max 1.3510) against a 1.5 m
straight-line goal, i.e. a bounded short-of-goal tolerance stop at the 0.15 m
goal tolerance rather than a raw 1.5 m path; mean 5.44 s sim time, mean RTF
0.131 on this host. Bags recorded 4840–4897 messages per run.

Cross-backend comparison of the same scenario and harness (all `nav_empty`,
goal 1.5 m, bumperbot, 5 evaluation seeds each):

| Backend | Seeds | Outcome | Trajectory (m) | Mean RTF | Bag msgs/run |
|---|---|---|---|---|---|
| MuJoCo (`../r81-live-mujoco-2026-09-15/`) | 1001–1005 | 5 success | 1.3531 ± 0.0027 | 0.378 | 814–860 |
| PyBullet (`../r81-live-pybullet-2026-09-15/`) | 1101–1105 | 5 success | 1.3535 ± 0.0014 | 0.349 | 801–804 |
| Isaac Sim (this set) | 1201–1205 | 5 success | 1.3507 ± 0.0004 | 0.131 | 4840–4897 |

The Isaac trajectory spread is the tightest of the three; the RTF is the
lowest because PhysX runs on the CPU on this host. Bag message counts are not
comparable across backends: Isaac publishes `/clock` and `/odom/ground_truth`
once per physics step (≈ 2100 msgs) while the PyBullet/MuJoCo spawners publish
them at a lower fixed cadence, and Isaac publishes `/scan` every 12 physics
steps (27 msgs per run in all five seeds).

## Honest limits

- One open arena only. Obstacle-bearing worlds are not qualified for Isaac;
  the R6.1 spawn-clearance work covers the box-arena reference routes, and the
  Isaac scan cadence above means scan-driven navigation would need its own
  check rather than an assumption.
- The mission controller uses `/odom/ground_truth` (truth, not an estimate) for
  waypoint control; this qualifies the backend's drive/odom/scan/clock contract
  and lifecycle, not a localization stack on Isaac.
- The ~15 % wheel/body speed gap recorded during the 2026-09-14 drive work is
  still not isolated; it did not prevent goal attainment here.
- This is simulator qualification. No hardware was involved and none is
  authorized by this evidence.
