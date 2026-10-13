# R5.6 / P5 / P6: four-wheel-steering patterns and mecanum on an obstacle mission (2026-10-13)

## Mission

Each cell runs `scripts/sim_nav_mission.sh <sim> <robot> nav_obstacle` in
nav mode. The robot is `four_wheel_steer_car` with `steering_mode:=` one of
ackermann, in_phase, crab or pivot, or `mecanum_car`. The mission is
`scripts/sim_nav_mission.py`:

1. A detour goal behind the obstacle row.
2. A cancel after 1 s; the robot must stop.
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

## Rerun

All 20 cells are queued again with the new arrival window
(`claude-4ws-missions-v2`).
