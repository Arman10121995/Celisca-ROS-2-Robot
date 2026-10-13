# R4.3 rerun with tf2 0.25.24 (2026-10-13)

The R4.3 evaluation protocol run again, unchanged, with tf2/tf2_ros 0.25.24
(the workspace overlay from 06:52 and, from 10:32, apt's own 0.25.24; see
the end of this page): Bumperbot in Gazebo on nav_maze, A* and NavFn, seeds
11/23/37/41/53, three goals each, Regulated Pure Pursuit, started 11:30 at
`7ae6548`.

| Planner | Success | Sim time median | Final truth error median / max | Min clearance |
|---|---|---|---|---|
| A* (Smac 2D) | 15/15 | 39.2 s | 0.066 / 0.164 m | 0.215 m |
| NavFn | 15/15 | 39.7 s | 0.074 / 0.176 m | 0.280 m |

All 15 goal pairs succeeded with identical goal positions. The full table
is in [report.md](report.md), per-trial records next to it.

**Stale-transform signatures: none.** No trial logged "Transform data too
old" or "Resulting plan has 0 poses", and the probe
(`scripts/tf_freeze_probe.sh`) wrote no report. The original evaluation
(2026-10-12, tf2 0.25.23) lost one A* goal to the stale-plan signature.

**All launches since the overlay.** The same holds for every Nav2 launch
after 06:52 on 2026-10-13: 33 launches, 0 signatures.

- Legged rounds 2 and 3: 13.
- 4WS reruns: 3.
- Gazebo anti-Ackermann rerun: 1.
- Isaac mecanum: 2.
- Mecanum round 3: 4.
- This rerun: 10.

Before the overlay, about 1 in 10 trials froze (9 of about 90). At that
rate about 3 freezes would be expected in 33 launches. Zero is consistent
with the deadlock fix; it is not yet a large sample.

**Later the same day: 46 launches, still 0.** Thirteen more Nav2 launches
followed by 16:05: legged round 4 (4), the MuJoCo rear-steer car (1), the
R7.3 scan recordings (6) and Isaac mecanum (2). None shows a signature.
At the earlier rate about 4-5 freezes would be expected.

**Where 0.25.24 came from.** The overlay (`scripts/build_tf2_deadlock_fix.sh`)
was installed at 06:52. Apt upgraded the system `ros-humble-tf2` and
`ros-humble-tf2-ros` to 0.25.24 at 10:32 (`/var/log/dpkg.log`), so the runs
from 10:32 had the fix twice over. The overlay was removed at 16:30, and
Nav2 now loads `/opt/ros/humble/lib/libtf2_ros.so` (0.25.24).

The trial bags, traces and launch logs stay on the SSD in
`robot_lab_runtime/claude-r43-tf2fix/`.
