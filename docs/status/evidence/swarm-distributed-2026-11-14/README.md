# Distributed swarm behaviours, sensed coverage and member-loss replanning (R10.3)

Recorded November 14, 2026, on `master` after `62d4ed62`, with this delivery's
working changes. Jetson AGX Orin, ROS Humble, Gazebo Fortress. The test is
three members (Bumperbot alpha and beta, Labbot gamma) in `nav_empty` with
per-member AMCL/Nav2. Implemented, experimental; R10.3 stays partial.

## What was added

- `robot_lab_utils/swarm_behaviours.py`: a port of
  [ROS2swarm](https://github.com/ROS2swarm/ROS2swarm) (Apache-2.0, master
  `10499321`; license copied here). It covers dispersion (potential field),
  attraction (repulsion field), aggregation (explore/join/stay/leave) and
  minimalist flocking, plus its hardware protection layer, with the
  TurtleBot3 burger parameters. Each member decides from **its own scan
  only**: no peer poses and no central planner. The module docstring lists
  the deviations: physical-width robot recognition, which also requires the
  object to stand free; seconds instead of scan counts; and a bounded
  leave.
- `robot_lab_utils/swarm_coverage.py`: coverage measured from scans. Free
  cells crossed by each member's own rays, up to 2 m, count as seen. Idle
  members go to the nearest reachable unseen cells, away from other members'
  goals.
- Fleet controller:
  - `local_*` swarm controllers;
  - `coverage` with `sensed: true`;
  - a `loss_policy` of `stop` (the default) or `replan`. With `replan`, a lost
    member leaves the group. The others continue, lane goals are
    redistributed, and the lost member's last pose stays an obstacle
    (+0.3 m). Stop and heartbeat loss end coverage missions.
  - Scan bearings now include the lidar mounting yaw. The Bumperbot lidar is
    turned 3.14 rad, so the existing scan guard had been checking the wrong
    side of the robot for "ahead".
- Aerial swarm controller: `loss_policy: replan` continues a formation,
  rendezvous, dispersion or flocking group with ≥ 2 members. Landing is
  requested only for a lost member that is still fresh and flying (one that
  is no longer flying is left alone). Its last horizontal position becomes a
  2.5 m no-fly column.
- GUI:
  - Swarm page: `local_*` modes and "Continue without a lost member".
  - Coverage page: "Measure coverage from scans" and a live measured fraction.
  - Aerial swarm: the same loss option.

## Results

| Case | Result |
|---|---|
| Distributed dispersion (run 5) | Started as a 0.9 m triangle; mean pairwise distance 0.94 → 3.15 m in 35 s; closest pair 0.87 m |
| Distributed aggregation (run 6, after 25 s dispersion) | Beta and gamma formed a group (0.8 m) and stayed ~40 s; alpha kept leaving while it still saw robot-like corners. Mean distance did **not** fall over 150 s (4.4 → 8.7 m). Not passed. |
| Sensed coverage with a real loss (run 5) | 6 × 6 m area, 14 400 free cells. Gamma's AMCL was stopped at 40 s, at 65.3 % coverage; gamma was excluded, and alpha and beta continued to **90.1 %** (the goal) at 61.8 s. 8 goals succeeded, 4 failed and were retried elsewhere, 2 were canceled at the end. No contact stop |
| Separation | Closest pair over all phases 0.77 m (radii 0.30 m) |

## Retained failures

- **Run 1** (RGB-D off): dispersion spread the robots, but aggregation drove
  them into walls. A Nav2 abort then held a coverage queue forever (56 %),
  and coverage kept assigning goals after ALL STOP. Fixed: the ROS2swarm
  protection layer, paused-queue reassignment, and Stop ending the mission.
- **Runs 2–4** (RGB-D off): both Bumperbots reported **90 contiguous −inf
  rays** (scan indices 135–224, the robot's front ±45°) once motion started.
  The scan guard treats −inf as contact, so they could not move. Raising the
  lidar 8 cm and then 15 cm did not change it, so the OAK-D was not the cause
  (`run4-rgbd-off-inf-sector.json`). With RGB-D on (run 5) no −inf rays
  appeared. The earlier R10.2 sessions also had RGB-D on. The raise was
  reverted. The controller now reports `scan_too_close_rays` per member, and
  the GUI warns about a blinded sector. Use RGB-D on until this Gazebo
  behaviour is understood.
- Aggregation in run 5 also failed to gather (3.2 → 7.2 m): wall fragments
  were counted as robots. Robot recognition now requires farther background
  on both sides. Run 6 then grouped two of three. A leave timeout was added
  afterwards and is unit-tested only.

## Reproduce

Source `scripts/ssd_env.sh`, ROS and the workspace; use an isolated domain
and partition.

```bash
python3 launch_case.py swarm-session-rgbd.yaml <dir>/launch.log
CASE_OUTPUT=<dir>/run /usr/bin/python3 -s swarm_probe.py
CASE_PHASES=local_dispersion:25,local_aggregation:150 CASE_SKIP_COVERAGE=1 CASE_OUTPUT=<dir>/agg /usr/bin/python3 -s swarm_probe.py
```

`launch_case.py` is the copy in `../fleet-zones-energy-2026-11-14/`. Traces,
launch logs and journals are on the SSD in
`/workspace/molar/robot_lab_runtime/swarm-distributed-20261114/`.

## Not covered

- Attraction and flocking have unit tests only.
- Aggregation has not passed.
- Aerial loss replanning has unit tests only.
- No 3–4 drone flight: the code supports 2–4 FCUs, but the flight is
  validation (V5/V7).
- Lane-coverage goal redistribution after a loss has unit tests only; the
  measured coverage used the sensed mode.
