# V2/V11: Bumperbot and Labbot navigation missions on four backends (November 21, 2026)

**Status: 16 of 16 cells passed** (validation repeat). Revision 0f918f21.

`scripts/sim_nav_mission_matrix.sh <dir> "mujoco pybullet gazebo isaac"
"bumperbot labbot" "nav_obstacle nav_maze"`, one cell at a time on a
quiet host. Each cell runs `sim_nav_mission.py`: a detour goal, cancel,
resume and a near-obstacle goal, judged by `/odom/ground_truth`.

| Backend | Bumperbot nav_obstacle | Bumperbot nav_maze | Labbot nav_obstacle | Labbot nav_maze |
|---|---|---|---|---|
| MuJoCo | passed (11.6 m) | passed (17.2 m) | passed (11.7 m) | passed (12.0 m) |
| PyBullet | passed (7.6 m) | passed (11.9 m) | passed (11.8 m) | passed (14.7 m) |
| Gazebo | passed (7.6 m) | passed (11.8 m) | passed (7.9 m) | passed (12.2 m) |
| Isaac | passed (11.6 m) | passed (17.4 m) | passed (7.5 m) | passed (16.9 m) |

Distances are truth forward travel over the whole mission. The reports
are in `runs/`, indexed with `mission-batch.yaml`. Traces, launch logs and
TF bags are on SSD under `robot_lab_runtime/v2-nav-matrix-20261121`.

One seed per cell. These are repeats of cells measured since October, not
new robot or world coverage.
