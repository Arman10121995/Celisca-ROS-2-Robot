# R5.6 / P5 / P6: four-wheel-steering patterns and mecanum on an obstacle mission (2026-10-13)

## Mission

Each cell runs `scripts/sim_nav_mission.sh <sim> <robot> nav_obstacle` in
nav mode. The robot is `four_wheel_steer_car` with `steering_mode:=` one of
ackermann, in_phase, crab or pivot, or `mecanum_car`. The mission is
`scripts/sim_nav_mission.py`:

1. A detour goal behind the obstacle row.
2. A cancel after 1 m of travel; the robot must stop.
3. A resume to the same goal.
4. A second goal next to an obstacle.

Each goal must succeed within 0.30 m of truth. The checker ran with
`--robot-radius 0.35` and its default 15° heading limit. One simulator ran
at a time.

The cells ran 02:19-03:01 while master moved from `58b8a70` to `b8e8ba5`.
None of those commits changes the 4WS or mecanum drive, their Nav2
configuration or the mission checker. From `4e9b2de` on, the mission also
keeps a `/tf` bag and runs the stale-transform probe.

## First run (`v1/`)

| Simulator | ackermann | in_phase | crab | pivot | mecanum |
|---|---|---|---|---|---|
| MuJoCo | fail (TF freeze) | **pass** | fail (stale plan start) | fail (arrival) | **pass** |
| PyBullet | **pass** | **pass** | **pass** | fail (arrival) | **pass** |
| Gazebo | fail (arrival) | **pass** | **pass** | fail (arrival) | **pass** |
| Isaac | fail (TF freeze) | **pass** | **pass** | **pass** | **pass** |

**13 of 20 cells pass.** Every pattern passes in at least one backend.
Mecanum and in-phase pass everywhere. Final errors of passing goals are
0.019-0.142 m from truth.

## Failures

- **Arrival (pivot in MuJoCo, PyBullet and Gazebo; Ackermann in Gazebo).**
  - The robot reached the goal position, 0.03-0.12 m from truth.
  - DWB's final in-place alignment toward the 0.015 rad (0.9°) goal heading
    crept at about 0.05 rad/s.
  - The base progress checker allows 10 s to move 0.5 m, and a rotation does
    not count as movement, so it aborted the goal.
  - The 4WS overlay now uses Bumperbot's checker (0.1 m in 30 s) and a 0.05
    rad arrival heading.
- **TF freeze (Ackermann in MuJoCo and Isaac).** controller_server's
  `map -> odom` stopped updating while AMCL kept publishing.
  - Isaac: the probe (`v1/isaac_ackermann_nav_obstacle.json.tf_freeze.txt`)
    shows a fresh listener receiving `map -> odom` up to 52.8 s, while the
    controller's newest stayed at 47.4 s.
  - MuJoCo: the controller logged the same error for the rest of the launch.
  - This is the failure described in the
    [global planning tutorial](../../../tutorials/global_planning_comparison.md);
    it is not a drive fault.
- **Stale plan start (crab in MuJoCo).** On the second goal the controller
  received plans that did not pass near the robot ("Resulting plan has 0
  poses"). Patience was exceeded twice. This is the planner_server form of
  the stale-pose signature (R4.3).

## Second run (`v2/`)

All 20 cells ran again from 06:06 to 06:50 (master `feb9a3b` to
`8a57b76`; none of those commits changes the drives, their Nav2
configuration or the checker). The 4WS overlay now had Bumperbot's
progress checker (0.1 m in 30 s) and a 0.05 rad arrival heading.

| Simulator | ackermann | in_phase | crab | pivot | mecanum |
|---|---|---|---|---|---|
| MuJoCo | **pass** | **pass** (rerun) | **pass** (rerun) | **pass** | fail (arrival) |
| PyBullet | **pass** | **pass** | **pass** | **pass** | **pass** |
| Gazebo | **pass** | **pass** | **pass** | **pass** | **pass** |
| Isaac | **pass** | **pass** | **pass** | **pass** | **pass** |

**19 of 20 cells pass**: every 4WS pattern passes in every backend,
including all four pivot cells. Final truth errors of passing goals are 0.017-0.192 m.

**Rerun cells.** The first v2 attempts at MuJoCo in_phase, crab and
mecanum are kept in `v2/loaded_host/`.

- in_phase and crab failed on a **TF freeze**: controller_server repeated
  one `map -> odom` stamp 407 and 286 times while a fresh listener kept
  receiving new ones (`*.tf_freeze.txt`). This is the tf2 0.25.23
  deadlock.
- I was also running tests, a build and the offline benchmarks on the
  host at the time; the extra load likely makes the deadlock more
  frequent.
- The reruns (07:16-07:22) ran on a quiet host **and** with the tf2
  0.25.24 overlay, installed at 06:52. in_phase and crab passed; the two
  changes cannot be separated from these runs.

**MuJoCo mecanum: arrival, twice.** Both attempts brought the robot to
0.03-0.09 m of the goal, and the base progress checker ("Failed to make
progress") aborted the final alignment: once on the detour goal, and on
both goals in the loaded attempt. The mecanum overlay had not been given
the 4WS arrival window. It now has it (`f36bc5e`). Its "control loop
missed its desired rate" warnings are not caused by load: the quiet
rerun had 560, the loaded attempt 640, and the passing first run 527.

## Mecanum rerun (`v3-mecanum/`)

Validation round 3 ran the mecanum cell again in every backend
(11:21-11:30, from `f36bc5e`, with the 4WS arrival window and the tf2
0.25.24 overlay; later commits by then touched documentation, the
walking-robot settings and an Isaac roller change that was not yet
built). None of the four logs has a stale-transform or progress-checker
error. **All four pass**: truth errors 0.035-0.108 m.

| Simulator | Detour goal | Near-obstacle goal |
|---|---|---|
| MuJoCo | 0.066 m | 0.060 m |
| PyBullet | 0.068 m | 0.096 m |
| Gazebo | 0.035 m | 0.108 m |
| Isaac | 0.082 m | 0.084 m |

With this rerun, **every 4WS pattern and the mecanum car pass the
obstacle mission in all four backends** (the four 4WS patterns from the
second run, mecanum from this one).
