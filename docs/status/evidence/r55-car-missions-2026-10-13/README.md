# R5.5: car-like robots on an obstacle mission in every backend (2026-10-13)

## Mission

Each cell runs `scripts/sim_nav_mission.sh <sim> <car> nav_obstacle` in nav
mode, one simulator at a time. Three cars are tested:

- `ackermann_car` (front steering);
- `rear_steer_car`;
- `anti_ackermann_car`.

Each car uses its own defaults: Smac Hybrid-A* with its 0.5 m turning
radius, and Regulated Pure Pursuit. The mission:

1. A detour goal behind the obstacle row.
2. A cancel after 1 m of travel; the car must stop.
3. A resume to the same goal.
4. A goal next to an obstacle.

Each goal must succeed within 0.30 m of truth. A car cannot turn on the
spot, so the goal heading is not checked (`--max-yaw-error-deg 0`); the
final headings are in the records. `motion_reverse_m` is the distance
driven backwards.

The cells ran from 01:32 to 05:55 at revisions `7b1b19e` to `b8e8ba5`;
the Gazebo anti_ackermann_car rerun ran at 07:22 at `42d1ccb`.
None of those commits changes the car drive or its Nav2 configuration. The
first Gazebo runs ran without ground truth: the car description had no
truth publisher. They were rerun after `30f5e83`; the originals are kept
on the SSD in `claude-car-missions/gazebo_no_truth_before_30f5e83/`.

## Results

| Simulator | ackermann_car | rear_steer_car | anti_ackermann_car |
|---|---|---|---|
| MuJoCo | **pass** (0.20 / 0.20 m) | fail (TF freeze) | **pass** (0.16 / 0.22 m) |
| PyBullet | **pass** (0.21 / 0.23 m) | **pass** (0.21 / 0.28 m) | **pass** (0.16 / 0.19 m) |
| Gazebo | fail (resume aborted 1.51 m short) | **pass** (0.18 / 0.21 m) | **pass** (0.17 / 0.23 m), rerun at 07:22 |
| Isaac | **pass** (0.19 / 0.23 m) | **pass** (0.20 / 0.27 m) | **pass** (0.14 / 0.16 m) |

The pairs are the final truth errors of the resumed detour goal and of the
near-obstacle goal. **10 of 12 cells pass.** Most cells also drove
0.8-1.3 m backwards at some point (`motion_reverse_m`).

## Failures

- **MuJoCo rear_steer_car: TF freeze.**
  - About 15 s into the resumed goal, controller_server's `map -> odom`
    stopped updating ("Transform data too old"). Its goal check then
    compared against a frozen pose.
  - The car drove 1.34 m past the goal, and the second goal began from a
    pose the planner placed in lethal space.
  - This is the failure described in the
    [global planning tutorial](../../../tutorials/global_planning_comparison.md).
- **Gazebo ackermann_car: the resumed detour was aborted.** RPP stopped 15
  times with "collision ahead" in the detour, and the planner failed once.
  The near-obstacle goal then succeeded (0.25 m).
- **Gazebo anti_ackermann_car: no truth, twice.**
  - The first fix (`30f5e83`) gave the description a truth publisher
    under the spawned model name. It reached only the robot_description
    parameter, not the description Gazebo spawns.
  - `anti_ackermann_car` reuses the Ackermann file, so its truth went to
    `/model/ackermann_car/...`.
  - Fixed in the spawn path as well (`71b2dbb`). The rerun at 07:22 had
    truth and passed: 0.17 m and 0.23 m, with 1.07 m driven backwards.
    It ran with the tf2 0.25.24 overlay installed (06:52); the other
    cells ran before it.

## MuJoCo rear-steer rerun with the tf2 overlay (`rerun-tf2/`)

The MuJoCo rear_steer_car cell ran again at 12:09 with tf2 0.25.24
installed. **No stale transform**: the log has no "Transform data too
old". The detour goal passed (0.18 m). Nav2 also reported the
near-obstacle goal reached, but truth was **0.301 m** away, 1 mm over the
0.30 m limit, so the cell still fails.

That miss is the cars' arrival window, not localization:

- the cars use the base 0.25 m goal tolerance (they cannot creep into a
  tighter one);
- Nav2 stopped 0.233 m from the goal by its estimate;
- the estimate was 0.071 m from truth.

A 0.25 m window plus normal localization error leaves almost no margin
under the checker's 0.30 m. The count stays **10 of 12**.

