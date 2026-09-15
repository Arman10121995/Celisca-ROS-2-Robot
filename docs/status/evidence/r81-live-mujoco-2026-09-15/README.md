# R8.1 live R4 mission — MuJoCo, 5 evaluation seeds (2026-09-15)

Full R4 `point_to_point_navigation` missions driven through the live stack
(no test doubles) on the MuJoCo backend, one isolated ROS domain per seed,
so the numbers below are simulation measurements rather than unit results.

- Harness: `src/robot_lab/robot_lab_benchmark/robot_lab_benchmark/live_mission.py`
  (`robot-lab-live-mission`, opt in with `ROBOT_LAB_RUN_LIVE_MISSION=1`;
  launch it as `ros2 run robot_lab_benchmark robot-lab-live-mission`, the
  repo's convention — `source install/setup.bash` does not put the console
  script on `PATH`)
- Robot / world / goal: bumperbot, `nav_empty`, 1.5 m ahead of the spawn pose
- Seeds: 1001–1005 (evaluation seeds; R4.3 requires >= 5, no tuning/eval overlap)
- Backends run sequentially so a concurrent recording cannot skew the RTF

## Reproduce

```bash
cd <ws> && source /opt/ros/humble/setup.bash && source install/setup.bash
export ROBOT_LAB_RUN_LIVE_MISSION=1 ROS_LOCALHOST_ONLY=1
ros2 run robot_lab_benchmark robot-lab-live-mission \
    --simulator mujoco --seeds 1001 1002 1003 1004 1005 \
    --goal 1.5 --world nav_empty --timeout 40.0 --domain 170
```

Per run the harness writes `manifest.json`, `metrics.json`, `trace.csv`,
`launch.log` and a real `rosbag2` capture under `bags/`. Per-topic bag
counts are parsed from the bag's `metadata.yaml` by the harness itself, so
recorded provenance is recoverable from these committed files and does not
depend on a hand-run `ros2 bag info`.

## Artifacts

- `summary_mujoco.json` — all 5 run records, verbatim harness output
- `stats.json` — distributions per metric plus a `per_seed` breakdown;
  every number cited in `docs/status/platform-status.yaml` comes from here
- `seed_<n>/` — that seed's `manifest.json`, `metrics.json`, first 40 lines
  of `trace.csv` and `launch.log`, and its `progress.log`

Raw `trace.csv` (one row per command tick) and the `bags/*.db3` recordings
stayed in `/tmp/r81_mujoco_multi/`; they are large and fully regenerable
with the command above. All five runs here were executed fresh on
2026-09-15 (seed 1001 was re-run rather than reusing the removed, superseded
single-seed 2026-09-14 artifact so every seed in this set came from the same
batch and build).

## Result

All 5 seeds reached the goal: outcome `success` (R4.1 truthful outcome, not a
collapsed boolean), 0 contact events, no collisions, no timeouts. Trajectory
distance 1.3531 m ± 0.0027 m (min 1.3504, max 1.3560) against a 1.5 m
straight-line goal, i.e. a bounded short-of-goal tolerance stop at the 0.15 m
goal tolerance rather than a raw 1.5 m path; mean 5.54 s sim time, mean RTF
0.378 on this host (software rendering). Bags recorded 814–860 messages per
run (`/clock`, `/odom/ground_truth`, `/scan`, `/cmd_vel`).
