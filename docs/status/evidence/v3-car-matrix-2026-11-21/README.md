# V3: the three cars' nav_obstacle mission on four backends (November 21, 2026)

**Status: 11 of 12 passed** (validation repeat of round 45, October 16).

The mission is the October car protocol, `scripts/sim_nav_mission_matrix.sh`
with `CHECK_ARGS="--max-yaw-error-deg 0"`: detour goal A, cancel, resume,
then near-obstacle goal B, judged by truth within 0.30 m, heading not
checked (cars cannot turn in place). Seed 1.

| Car | MuJoCo (A / B) | PyBullet | Gazebo | Isaac |
|---|---|---|---|---|
| Ackermann | passed (0.065 / 0.213 m) | passed (0.064 / 0.151 m) | passed (0.053 / 0.188 m) | **failed**: A 0.130 m; B aborted 7.8 m short |
| rear-steer | passed (0.149 / 0.130 m) | passed (0.121 / 0.126 m) | passed (0.096 / 0.108 m) | passed (0.050 / 0.045 m) |
| anti-Ackermann | passed (0.067 / 0.193 m) | passed (0.061 / 0.208 m) | passed (0.105 / 0.250 m) | passed (0.049 / 0.030 m) |

The base stops 0.15–0.30 s after the cancel in every run.

**First attempt, kept in `wrong-protocol/`:** the same matrix ran first with
the default 15° heading check meant for robots that turn in place. All 12
failed only on heading (for example Gazebo Ackermann: A 0.093 m but 26°).
Those reports are not indexed; they would have replaced measured cells with
the wrong protocol.

Traces, launch logs and TF bags are on SSD under
`robot_lab_runtime/v3-car-matrix-20261121`. Reports are indexed with
`mission-batch.yaml`.
