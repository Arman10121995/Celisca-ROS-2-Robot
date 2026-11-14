# Car course: all 12 cells pass (R5.5, 2026-11-15)

Claude. Implemented, experimental.

The run: `scripts/sim_car_course.sh <sim> <car>` with `CHECK_ARGS=--trim`.
The course is a five-cone slalom, then parking in a bay between two parked
cars. The parking adds the heading shuffle and the straight depth trim.

**Pass criteria:**

- every gate within 0.30 m of truth;
- the bay within 0.15 m and 10° of the target;
- the footprint clear of every obstacle.

One run per cell, plus one repeat for Gazebo anti-Ackermann.

## What failed before

Round 62 (2026-11-05) passed 10 of 12 cells. Gazebo rear-steer and Isaac
anti-Ackermann aborted at the first slalom goal:

- **Start:** the spawn (−0.5, −0.5) is in line with the cones, 1.5 m behind
  cone 0. The car must swerve 0.37 m sideways in about 1 m of travel: an
  S-curve of two 0.5 m-radius arcs.
- **Tracking:** Regulated Pure Pursuit's 0.8 m lookahead cut that S-curve.
  The base peaked at 17° heading and straightened early, so a footprint
  corner passed 5–6 cm from the cone.
- **Collision stop:** RPP stopped ("collision ahead").
- **Replan refused:** Hybrid-A* refused to plan from a footprint already
  touching lethal cells.
- **Recovery failed:** the recovery back-up was blocked too ("Collision Ahead
  - Exiting DriveOnHeading"), and the goal failed.

Re-running today's code reproduced it once: Gazebo anti-Ackermann aborted at
cone 0 with 0.057 m clearance.

## Changes

| Change | Where |
|---|---|
| Car RPP lookahead 0.8 → 0.6 m (min 0.45, max 0.9): 1.25 × the 0.5 m turning radius, as already used for `forward_arc` robots | `robot_lab_navigation/launch/navigation.launch.py` (`CAR_PARAMS`) |
| Recovery: if the 0.30 m back-up is blocked, drive 0.20 m forward on the current heading (Nav2 `DriveOnHeading`). Both are collision-checked, and there is never an in-place spin | `behavior_tree/car_navigation_w_replanning_and_recovery.xml`; `drive_on_heading` plugin in `config/behavior_server.yaml` |

## Result

All 13 runs pass ([runs/](runs/)):

| Cell | Slalom min clearance (m) | Bay error (m) | Bay yaw error (°) | Bay min clearance (m) |
|---|---:|---:|---:|---:|
| Gazebo Ackermann | 0.139 | 0.024 | −1.4 | 0.112 |
| Gazebo anti-Ackermann (a) | 0.130 | 0.015 | −1.2 | 0.138 |
| Gazebo anti-Ackermann (b) | 0.132 | 0.038 | −4.5 | 0.092 |
| Gazebo rear-steer | 0.089 | 0.085 | −2.7 | 0.095 |
| MuJoCo Ackermann | 0.112 | 0.067 | −3.8 | 0.078 |
| MuJoCo anti-Ackermann | 0.141 | 0.015 | −2.0 | 0.088 |
| MuJoCo rear-steer | 0.080 | 0.062 | −0.9 | 0.059 |
| PyBullet Ackermann | 0.118 | 0.098 | −2.0 | 0.060 |
| PyBullet anti-Ackermann | 0.108 | 0.054 | −3.9 | 0.083 |
| PyBullet rear-steer | 0.114 | 0.035 | 0.0 | 0.122 |
| Isaac Ackermann | 0.095 | 0.049 | −0.9 | 0.125 |
| Isaac anti-Ackermann | 0.099 | 0.035 | −2.2 | 0.080 |
| Isaac rear-steer | 0.097 | 0.030 | −0.1 | 0.134 |

The slalom's closest approach was 0.08–0.14 m, against 0.049–0.059 m in
the round 62 aborts. The recovery back-up ran one to four times per run.
The forward escape never ran in these 13. In the one run where it did
(an earlier test, [experiments/](experiments/)), the car was already wedged
against the cone, both directions were in collision, and it failed.
So it widens recovery but does not replace keeping clear.

Measured curvature (|yaw rate| / speed, `max_curvature_per_m`) is reported,
not gated. It reaches 2.05–3.69 per metre at low speed, the same range as
round 62 (2.04–3.96).

## Limits

- One run per cell; flat floor, scripted course, no traffic.
- The other repeats, mapping and localization remain validation (V3).
