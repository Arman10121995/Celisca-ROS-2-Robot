# R10.6: saved-map Nav2 goals on PyBullet (November 20, 2026)

**Status: implemented, experimental.** The octopod reaches a Nav2 goal on
its own saved map on PyBullet (`nav_empty`): it settled 0.091 m from the
goal by truth. This is the first saved-map goal for a multi-leg model on a
second backend; the 11-19 runs were MuJoCo only. The hexapod and centipede
do not pass on PyBullet.

## What changed

`scripts/qualify_gait_saved_map.py --simulator mujoco|pybullet|isaac|gazebo`
selects the backend for both launches. The PyBullet gait launch branch
already passed the lidar/RGB-D sensor configuration, so no runtime change
was needed. PyBullet ran the gait at about 0.36× real time, so the
navigation budget is `--timeout 600` (wall seconds).

## Measured (PyBullet, `nav_empty`, goal 0.5 m ahead, 0.15 m / 0.15 rad settle contract)

| Model | Budget | Map | Result | End to goal | Heading error | Sim / wall time |
|---|---|---|---|---|---|---|
| **octopod** | 600 s | 32896 cells | **passed** | **0.091 m** | 0.001 rad | 60 s / 87 s |
| hexapod | 220 s | 32472 cells | **not passed**: timed out while still closing in (0.009 → 0.001 m/s) | 0.173 m | 0.015 rad | 79 s / 220 s |
| hexapod | 600 s | 32330 cells | **not passed**: reached 0.186 m at 74 s, then crept backward while turning in place to −0.25 rad | 0.441 m | 0.259 rad | 208 s / 580 s |
| centipede | 600 s | — | **not passed**: mapping did not reach the required 0.2 m of travel in 120 s | — | — | — |

The hexapod's in-place turn drifts backward on PyBullet, and its approach
slows to almost nothing. Both are gait-calibration matters on this
backend; neither was tuned here. The centipede is too slow on PyBullet to
complete the mapping step.

## Limits

- PyBullet only; Isaac and Gazebo saved-map goals are not run.
- One goal per model; the octopod passed once.
- Per-backend gait tuning for the hexapod and centipede remains.

## Reproduce

```bash
python3 scripts/qualify_gait_saved_map.py --output <new dir> --simulator pybullet --timeout 600 \
  --robots hexapod_ros2 lab_octopod lab_centipede
```

`summary.json` lists every run. Logs are on SSD under
`robot_lab_runtime/gait-saved-map-pybullet*-20261120` (hashes in
`runtime-sha256.txt`).
