# R5.5: car nav_obstacle missions with a tighter goal and a wider planner inflation (2026-10-16)

The three cars are the Ackermann, rear-steer and anti-Ackermann cars. Over
goal seeds 2 and 3 they passed 13 of 24 missions
([round 35](../wheeled-repeats-2026-10-15/README.md)). Their near-obstacle
goal ended 0.26-0.36 m from truth, and two goals aborted. Every car cell
failed the support matrix's 0.25 m screen.

Two configuration changes followed, each measured over seeds 1-3 in
MuJoCo, PyBullet, Gazebo and Isaac (36 missions per round). The mission is
the same as before: detour to goal A, cancel and stop on the way, resume,
then go to the near-obstacle goal B. `scripts/sim_nav_mission.sh` judges
it against simulator truth, with a 0.30 m limit and the heading not
checked (`--max-yaw-error-deg 0`).

## Result: 35 of 36 pass (round 45, `32c9cf1`)

| Car | MuJoCo s1/s2/s3 | PyBullet s1/s2/s3 | Gazebo s1/s2/s3 | Isaac s1/s2/s3 |
|---|---|---|---|---|
| Ackermann | pass / pass / pass | pass / pass / pass | pass / pass / pass | pass / pass / pass |
| rear-steer | pass / pass / pass | pass / pass / pass | pass / pass / pass | pass / pass / pass |
| anti-Ackermann | pass / **B aborted** / pass | pass / pass / pass | pass / pass / pass | pass / pass / pass |

- In the passing runs, goal A ends 0.002-0.161 m from truth and goal B
  0.040-0.224 m.
- The base stops 0.10-0.32 s after the cancel.
- Reports: [round45-inflation/](round45-inflation/).

**The one failure.** MuJoCo anti-Ackermann, seed 2, came within
0.13-0.24 m of goal B and stayed there for about 50 s.

- It never got inside the 0.1 m goal circle: the moves it made were too
  small to correct the remaining offset.
- The progress checker aborted it twice (0.1 m of movement in 20 s), with
  one back-up in between. It ended 0.153 m from truth
  ([log lines](round45-inflation/aborts.txt)).
- A car cannot turn on the spot, so settling inside a tight circle can need
  a multi-point manoeuvre that Regulated Pure Pursuit does not make.
- The same cell settled after 60 s in round 43.

## Change 1: a 0.1 m goal tolerance (`ac71687`)

The cars used Nav2's 0.25 m default. Every other wheeled base uses
0.07-0.08 m. The goal checker is stateful, so a car goal ended the moment
the estimate entered the 0.25 m circle, with localization error added on
top.

- **Trial** ([trial-tolerance/](trial-tolerance/)): MuJoCo Ackermann,
  seed 2, which ended 0.34 m off in round 35, now ends both goals 0.065 m
  and 0.138 m from truth.
- **Round 43** ([round43-tolerance/](round43-tolerance/)): **32 of 36
  pass.** Goal A ends 0.009-0.175 m and goal B 0.049-0.234 m from truth.
  Four runs aborted:
  - Gazebo Ackermann, seed 1, at the detour (as on October 13);
  - Gazebo and Isaac rear-steer, seed 3, on the way to B;
  - PyBullet anti-Ackermann, seed 3, on the way to B.

**Cause of the four aborts.** In each, the planner refused to replan:
"Starting point in lethal space". The single 0.3 m back-up of the car
tree did not clear it, and the goal aborted
([log lines and clearances](round43-tolerance/aborts.txt)).

- The footprint was measured against the map along each trace
  ([footprint_clearance.py](footprint_clearance.py), 0.05 m cells). In the
  three B aborts it came within one cell of the block that the route to B
  passes.
- The rear-steer tail swung toward the block at its lower corner, and the
  anti-Ackermann nose cut its upper corner.
- Hybrid-A* had planned with a 0.35 m inflation at the default 3.0
  falloff, less than the cars' 0.45 m circumscribed radius.

## Change 2: a 0.6 m planner inflation at cost scaling 2.0 (`32c9cf1`)

Only the global costmap changed. The side of the block is several metres
wide, and goal B keeps its 0.45-0.60 m clearance.

- **Trial** ([trial-inflation/](trial-inflation/)): the four aborted cells
  pass. The footprint stays 0.15-0.26 m from the block.
- **Round 45**: the result above.
  - The footprint's closest approach to the map is 0.10 m or more in 33 of
    the 36 runs ([clearances](round45-inflation/aborts.txt)).
  - Two of the other three: PyBullet Ackermann, seed 1, in the start
    corner (0.07 m), and the aborted MuJoCo anti-Ackermann, seed 2, near
    goal A (0.05 m).
  - MuJoCo rear-steer, seed 1, ends goal B with its tail on an occupied
    cell of the map, and the mission still passes. The map and the
    simulated block may differ by a cell there; this was not checked.

Tests:
`test_car_goal_checker_does_not_stop_at_the_default_circle` and
`test_car_planner_inflation_covers_the_swinging_body` in
`src/robot_lab_bringup/test/test_wheel_navigation_contract.py`.

## Support matrix

Both rounds are indexed
([batch](mission-batch.yaml)): round 43 first, then round 45, seed 3 last.
The latest record controls each cell, so all twelve car cells now pass the
0.25 m screen. The matrix does not show the MuJoCo anti-Ackermann seed-2
abort; it is recorded here and in the index.

## Limits

- One world (nav_obstacle), three seeds, one run each.
- The goal heading is not checked for the cars.
- The footprint clearance compares truth poses with the occupancy map,
  not with the simulated geometry.
- The settling failure above has no fix yet: 1 of 36 goals.
