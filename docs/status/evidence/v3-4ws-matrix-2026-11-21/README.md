# V3: four-wheel-steering patterns and mecanum on the nav_obstacle mission (November 21, 2026)

**Status: 20 of 20 passed** (validation repeat of the October 13 round,
whose first run passed 13 of 20).

`run_patterns.sh` runs `scripts/sim_nav_mission.sh <sim> four_wheel_steer_car nav_obstacle
<out> steering_mode:=<pattern>` (or `mecanum_car`) with `CHECK_ARGS="--robot-radius 0.35"`.
The mission is a detour goal, cancel, resume and a near-obstacle goal, each
within 0.30 m and 15° of truth. Seed 1, one simulator at a time.

| Backend | ackermann | in_phase | crab | pivot | mecanum |
|---|---|---|---|---|---|
| MuJoCo | passed (0.017 / 0.027 m) | passed (0.054 / 0.055) | passed (0.060 / 0.072) | passed (0.064 / 0.008) | passed (0.098 / 0.059) |
| PyBullet | passed (0.069 / 0.062) | passed (0.072 / 0.008) | passed (0.055 / 0.059) | passed (0.098 / 0.038) | passed (0.094 / 0.090) |
| Gazebo | passed (0.014 / 0.028) | passed (0.087 / 0.039) | passed (0.085 / 0.077) | passed (0.072 / 0.037) | passed (0.061 / 0.084) |
| Isaac | passed (0.120 / 0.081) | passed (0.047 / 0.097) | passed (0.028 / 0.102) | passed (0.088 / 0.050) | passed (0.072 / 0.015) |

Values are goal A / goal B errors from truth. Every heading error is
within 4°. Traces, launch logs and TF bags are on SSD under
`robot_lab_runtime/v3-4ws-matrix-20261121`.
