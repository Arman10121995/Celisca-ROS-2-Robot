# R5.6 measured wheel control and Nav2 screens — October 2

Host: Jetson AGX Orin, Ubuntu 22.04, ROS Humble arm64. Baseline `2a0aae2`
plus the source snapshot in `source-hashes.json`. Physics trials ran serially
in separate ROS domains and simulator partitions. Wheel Gazebo uses Fortress
6.18; drone Harmonic is a separate runtime. Isaac uses native Isaac Sim 6.0.1
with CPU PhysX. All runs below use `nav_empty`, headless GUI/RViz.

## Navigation results

All rows use real Nav2, stack AMCL and independent `/odom/ground_truth`.
Except the historical first row, goals go through `/robot_lab/goal_pose`,
the same relay as RViz's goal tool. These are single clear-map screens, not
obstacle or mapping qualification. Times exclude startup/AMCL settling.

| Report | ROS domain / steering | Goal XY / requested yaw | Action / wall s | Final truth XY | Truth position / heading error |
|---|---|---|---|---|---|
| `mujoco-4ws-nav.json` | 191 / opposite-phase, previous RPP default | (1, −0.5) / 0° | succeeded / 10.4 | (0.697, −0.479) | final yaw not recorded |
| `mujoco-4ws-crab-dwb-nav.json` | 194 / crab, constrained DWB | (1, 1) / 90° | succeeded / 17.9 | (0.777, 0.915) | final yaw not recorded |
| `mujoco-4ws-inphase-dwb-nav.json` | 195 / in-phase, constrained DWB | (1, 1) / 90° | succeeded / 19.5 | (0.799, 0.896) | final yaw not recorded |
| `mujoco-4ws-pivot-dwb-nav.json` | 196 / pivot, DWB | (1, 1) / 90° | succeeded / 17.7 | (0.828, 0.839) | final yaw not recorded |
| `pybullet-4ws-crab-dwb-nav.json` | 197 / crab, constrained DWB | (1, 1) / 90° | succeeded / 24.8 | (0.799, 0.953) | final yaw not recorded |
| `gazebo-4ws-crab-dwb-nav.json` | 198 / crab, constrained DWB | (1, 1) / 90° | succeeded / 16.0 | (0.774, 0.891) | final yaw not recorded |
| `isaac-4ws-crab-dwb-nav-fixed.json` | 200 / crab, constrained DWB | (1, 1.01) / 90° | succeeded / 76.1 | (0.820, 0.903) | 0.210 m / −16.13° |
| `mujoco-4ws-ackermann-dwb-nav.json` | 201 / opposite-phase, current DWB default | (1, 1) / 90° | succeeded / 16.2 | (0.833, 0.823) | 0.243 m / −13.64° |
| `gazebo-mecanum-nav.json` | 186 / physical mecanum, DWB | (1, −0.5) / 0° | succeeded / 14.0 | (0.727, −0.522) | final yaw not recorded |

The default goal checker accepts 0.25 m / 0.25 rad in the localization
estimate. Independent truth may differ: Isaac's final yaw is 73.87°, while
Nav2 accepts its estimated pose. Precise body-heading qualification remains
open. Early reports record accumulated absolute turn rather than final yaw;
the probe now reports final truth yaw and goal error explicitly.

Reproduce the diagonal screen after rebuilding the changed packages:

```bash
source scripts/ssd_env.sh
export ROS_DOMAIN_ID=201 TIMEOUT=180
export CHECK_ARGS='--via-topic --offset-x 1.5 --offset-y 1.5 --goal-yaw-deg 90'
scripts/sim_nav_check.sh mujoco four_wheel_steer_car nav_empty \
  /workspace/molar/robot_lab_runtime/new-wheel-trial.json steering_mode:=ackermann
```

Use a fresh domain below 233 and run one simulator at a time. Substitute
`crab`, `in_phase` or `pivot`, and the named backend. Isaac's corrected screen
used `TIMEOUT=240`. `navigation-log-excerpts.txt` records actual critic load
and goal events; full launch logs remain on SSD beside each JSON report.

## Physical motion and collision

`gazebo-4ws-pivot-drive-fixed.json` (domain 183) records 0.994 rad/s body yaw
on a 1.0 command, with 0.014 m translation over the measured two-second
window. Pods align at ±π/4 tangent directions; stopping holds their angles.
Its lateral phase remains stationary because the selected pivot pattern
intentionally ignores lateral-only input. Exact per-contact rolling allocation
and ±π/2 limits replace the old straight-wheel skid pivot and 45° crab limit.

`gazebo-mecanum-proxy-drive.json` (domain 185) records physical passive-roller
body travel: forward 0.397 m/s on 0.4, lateral 0.278 m/s on 0.3 with 0.001
rad/s yaw, and zero at stop. Phases use odometry simulation time with a wall
timeout; measured durations and wall times are in JSON. Gazebo truth is a
native model-pose system independent of wheel odometry.

The current proxy's lateral repeats are also recorded: MuJoCo domain 202
(`mujoco-mecanum-proxy-drive.json`) reaches 0.253 m/s and PyBullet domain 203
(`pybullet-mecanum-proxy-drive.json`) 0.276 m/s on a 0.3 command. Respective
forward speeds are 0.390/0.403 m/s on 0.4; stopped tails are zero or within
0.001 m/s. These are physical movement screens, not full Nav2 qualification.

The source roller has 5,844 triangles. `roller-proxy-check.json` verifies the
deterministic 68-face convex collision proxy against 4,000 seeded directions:
maximum support difference 0.419 mm. Visual meshes are unchanged. Reproduce:

```bash
source scripts/ssd_env.sh
python3 scripts/simplify_mecanum_collision.py --check
export ROS_DOMAIN_ID=185 MODE=localization ODOM=/odom/ground_truth
export CHECK_ARGS='--quick --strafe'
scripts/sim_drive_check.sh gazebo mecanum_car nav_empty \
  /workspace/molar/robot_lab_runtime/new-mecanum-drive.json
```

`mujoco-4ws-crab-drive.json` is the earlier wall-duration probe: body lateral
speed 0.211 m/s with all steering joints at π/2. It lacks phase-duration
fields and is not interchangeable with the later simulation-clock probes.

## Negative trials and repairs

- `gazebo-4ws-pivot-drive.json`: no truth odometry before the Gazebo reset
  script executable/install fix. The later native-truth run supersedes it.
- Original 60 full collision meshes stalled Gazebo with no progressing clock
  or odometry. The interrupted `gazebo-mecanum-drive` raw log remains on SSD;
  the proxy run measures movement rather than treating startup as success.
- `mujoco-4ws-crab-nav.json`: an invalid initial checker CLI produced no report;
  its stderr is retained on SSD. This was not a robot mission.
- `mujoco-4ws-crab-diagonal-nav.json`: unconstrained RPP aborted after 87.2 s,
  estimated goal error 0.781 m. Parallel steering cannot execute simultaneous
  translation/yaw. The C++ DWB `ParallelSteeringCritic` rejects those sampled
  trajectories; the corrected real DWB goals above succeed.
- `isaac-4ws-crab-dwb-nav.json`: accepted/moving goal timed out at 180 s,
  with truth (0.541, −0.5). Isaac discarded lateral velocity for 4WS; it now
  forwards `vy` for both mecanum and 4WS. The old probe misleadingly labeled
  terminal-status absence as “no goal accepted”; the new probe distinguishes
  accepted-goal timeout. The corrected domain-200 mission succeeds.
- Duplicate dispatch YAML keys replaced genuine Nav2 planner/controller
  plugins with educational nodes. Real Smac2D/DWB and one stack AMCL authority
  are restored. Educational AMCL's callback imports are fixed and its pose
  output is namespaced. This does not qualify five localization methods.

## Verification and remaining acceptance

`continuation-fast-verified-2026-10-02.log`: 733 passed, 1 skipped in the fast
tier. Focused dispatch/tutorial checks: 64 passed. Tk GUI command/composition/
compatibility checks: 66 passed; later selected drone/4WS checks: 5 passed.
After the final registry descriptions/evidence update, registry/composition
checks pass 21 tests and the five selected Tk checks pass again. Registry
rebuild passes. Their logs accompany this report.
The actual installed GUI Health/Flight smoke passes without mocked tabs,
launching a robot or arming. C++ plugin and changed ROS package builds pass.
The source kinematics/resolver checks and early GUI/dispatch logs are retained.

Remaining R5.6 work includes other steering patterns on Gazebo/PyBullet/Isaac,
obstacle routes, tighter repeated final body heading, reset, publisher loss,
Localization and 2D/RGB-D SLAM missions. Qualify mecanum Nav2/obstacle routes
on MuJoCo/PyBullet and physical mecanum on Isaac. The added OAK-D description and
available launch options establish implementation, not mapping acceptance.
Broader gaps are in `docs/status/continuation-2026-10-02.md` and the ledger.
