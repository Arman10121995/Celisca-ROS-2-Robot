# Drone moving obstacles and sensor loss — 2026-11-13 (ROADMAP R5.11)

PX4 X500 with depth camera (`x500_depth`), Gazebo Fortress, RTAB-Map, observed
obstacles (`aerial_obstacles:=mapped_depth`), world `aerial_moving`. Two Gazebo
actors walk across the route: walker 0 at x = 0 (14 s loop, y −6…6 m) and walker 1
at x = 5 (15 s loop). They do not react to the drone. The drone spawns at
(−10, 0) and flies to (10, 0, 1.2).

```bash
python3 scripts/qualify_drone_moving.py \
  --output /workspace/molar/robot_lab_runtime/drone-moving-20261113/run-NN
```

The check:

1. checks takeoff clearance and takes off;
2. descends to walker height;
3. publishes the goal at a time when walker 0 is about to cross;
4. stops the camera bridge 3 s after the goal is reached (sensor loss).

**Pass** requires all of:

- no error;
- the goal reached;
- at least one response to moving people: a replan, a walkway hold or an
  evasive climb;
- the true drone–walker distance above 0.6 m for both walkers at every sample;
- a hold, then a landing, after the camera loss.

Distances are computed from:

- the drone's simulator truth (`/px4/odometry_truth`);
- each walker's scripted trajectory at the Gazebo clock (`gz` clock samples,
  linear fit). Fortress animates actors only in rendering and publishes no
  actor poses, so the script is the walker truth.

A walker is a 1.8 m column standing on the ground.

## Result

Final code paths (all fixes below), runs 32–34:

| Run | Result | Closest walker 0 / walker 1 | Responses | Goal reached | Camera loss → hold / land |
|---|---|---|---|---|---|
| 32 | **pass** | 1.97 m / 1.44 m | 2 walkway holds (look both ways, overfly at 3.49 m) | 166 s | 1.75 s / 11.9 s |
| 33 | fail before the mission | — | — | — | — |
| 34 | **pass** | 2.32 m / 1.08 m | 4 walkway holds, 3 replans | 535 s | 1.68 s / 12.3 s |

In run 33, the restarted PX4 estimator's height ran away on the ground. PX4
armed on it during a health-check flicker, and the drone climbed to 5.4 m,
where the camera saw too little ground (counted as sensor loss). Takeoff now
also requires a steady resting height (run 34). Run 30, before the floor fix,
also passed once the response rule was broadened. Run 31, on the same code,
did not.

## What changed, run by run

Runs 1–19 developed the basic loop: takeoff retries, map age, voxel clustering
with static-world filtering, Kalman tracks, the sensor-loss grace period and
automatic replanning after holds. Runs 14–17 reached the goal but touched
walkers (0.14–0.42 m). From run 19 on, every change came from a measured
failure:

| Run | Failure | Change |
|---|---|---|
| 19 | Hover orbited up to 7 m; execute refused (plan start stale) | ULog: PX4's EKF began with gyro biases of (−0.03, −0.04, −0.09) rad/s while the raw gyro read zero. Heading drifted 5–6°/s on the ground; the magnetometer never corrected it. In 19 flights that day, about half drifted or sat 40–80° off. **Takeoff now waits for a resting heading steadier than 0.25°/s over 4 s; otherwise the runtime restarts PX4's `ekf2` (at most 3 times).** |
| 20 | Plan rejected at execute: mapped occupancy blocked the route | RTAB-Map adds nodes only when the camera moves, so walkers stayed in the map as walls along their paths. **Swept memory: mapped points inside columns moving tracks swept in the last 5 min are dropped.** |
| 21 | Goal reached; 0.26 m to walker 1 (side approach, out of the camera's view) | **Look both ways:** hold at a walkway edge, turn ±75°, cross when no track is predicted on the route. Also found `time_conflict` returned "clear" for a hovering drone (speed 0), so the evasive climb could never fire. |
| 22 | Stuck after crossing | `set_path` drops the first point (the start); the resumed route was empty. Evasion did not record its hold reason, so no replan followed. |
| 23–24 | Script faults | Route end within 5 cm of the last waypoint; goal requests now numbered so a stale status cannot answer. |
| 25 | 0.43 m to walker 0 after an evasion replan | Every walkway crossing is timed; inside a walkway, a predicted conflict climbs. |
| 26 | 0.555 m over walker 1's head at 2.19 m | Tracks were 0.5–0.8 m tall (the camera sees part of a person). **Prior: a grounded moving object is at least 2 m tall.** |
| 27 | 0.51 m to walker 1 inside its walkway | A 7.5 s walker period leaves no window for an 11 s crossing at walker height. **Walkways are crossed above their swept columns.** |
| 28 | 0.24 m; tracker saw nothing | See run 31. |
| 29 | Overflight at 4.2–4.6 m, then a sensor-loss landing | A level camera saw too little ground within its 8 m range (counted as sensor loss). **Overflight capped where the ground is visible within 80 % of range (about 3.5 m).** |
| 30 | **Pass** (rescored) | The response rule then counted only replans; this flight responded by a walkway hold and overflight. The rule now counts replans, walkway holds and evasions; the report keeps both verdicts. |
| 31 | 0.20 m; at most one depth cluster | The floor filter used a fixed band above the FCU's ground height, but the height estimate errs by 0.2–0.6 m per run. The floor ahead joined the walkers in one oversized blob that was never tracked (same as run 28). **The floor is now taken from the depth itself (lowest point per 1 m cell near the expected ground).** |
| 32 | **Pass** | |
| 33 | Restarted EKF's height ran away on the ground (4.4 m, 3.5 m/s in 4 s); PX4 armed during a health flicker; climbed to 5.4 m and lost the ground | **Takeoff also requires a steady resting height** (limits from today's logs: resting estimates move 0.05–1.7 m in 4 s; the runaway moved 4.4 m). |
| 34 | **Pass** | |

Reports: [run-19](run-19-report.json) (orbit), [run-21](run-21-report.json),
[run-25](run-25-report.json), [run-27](run-27-report.json),
[run-28](run-28-report.json), [run-29](run-29-report.json),
[run-30](run-30-report.json), [run-31](run-31-report.json),
[run-32](run-32-report.json), [run-33](run-33-report.json);
[all runs](runs-summary.json).

## Scope

This evidence covers one world, two non-reacting walkers, one route and
Gazebo only. It does not cover:

- people who react or turn;
- vehicles, or objects taller than the overflight;
- crowds;
- repeated statistics (two passes in three flights of the final code paths; the third failed before
  the mission, on an estimator fault that the takeoff gate now refuses);
- other worlds;
- the MuJoCo plant (see below).

The 2 m standing-height prior and the overflight assume open space above
walkways. Earlier failures (runs 1–31) are retained.

## Second backend: PX4 with a MuJoCo plant

`robot_lab_adapter/px4_mujoco_plant.py` runs PX4 SITL with its external-simulator airframe
(`10016_none_iris`) against a MuJoCo quadrotor:
- lockstep HIL sensor and actuator messages over TCP 4560;
- the Iris mass, inertia, motor model and rotor geometry, matching PX4's `CA_ROTOR` parameters;
- IMU, magnetometer, barometer and GPS with declared noise.

The robot is `px4_iris` on MuJoCo; the launch adds `--plant mujoco`.

```bash
python3 scripts/qualify_drone_moving.py --robot px4_iris --simulator mujoco --map aerial_course \
  --plain --goal -35 -40 2.0 --output .../mujoco-3
```

**Pass** ([report](mujoco-iris-report.json)). From the spawn at (−40, −40), the drone:

1. passed the takeoff clearance check (resting heading drift 0.003°/s, no estimator restart);
2. took off;
3. flew a checked 5 m route, reaching (−35.02, −39.99, 2.02) after 56.6 s (FCU estimate);
4. landed and disarmed at 67.5 s.

Two earlier attempts failed and are kept as history:

* `mujoco-1`: the runtime set a Gazebo plugin path that exists only for Gazebo (fixed).
* `mujoco-2`: the plant's body rested 0.14 m up on its gear, so the X500 takeoff envelope reached into the
  ground and the takeoff check refused. The plant now rests at 0.24 m like the X500.

Moving obstacles are not available in this backend: there is no depth camera in the plant.
