# Flipper-aware terrain policy and gait slip recovery (R10.7 / R10.6)

Recorded November 14, 2026, on `master` after `649f3c4a`, with this
delivery's working changes. Gazebo Fortress with the private native track
engine. The robot is `lab_tracked_articulated` in `terrain_stairs`, whose
steps are 0.18 m high with 1.1 m treads. Implemented, experimental; neither
task is passed.

## What was added

- `robot_lab_utils/tracked_flippers.py`:
  - **terrain ahead** from the robot's own forward depth camera: the
    90th-percentile rise in a 0.5 m corridor 0.55–1.4 m ahead, plus a drop
    cue (floor missing or lower than flat ground);
  - **pitch and roll** from its IMU;
  - **policy states:** travel, approach (front flippers up to catch the edge
    and rear flippers pressing down early), climb (front up, rear down),
    descend (front reaching down) and roll hold (drive stopped);
  - **joint targets:** elevations map to joints as q_front = 0.5 − θ and
    q_rear = θ − 0.5, derived from the source SDF.
- `tracked_control`:
  - `/tracked/flipper_command` `op: auto` runs the policy; a manual target or
    Hold ends it;
  - state reports `auto_state`;
  - IMU pitch is converted to nose-up for the policy.
- GUI Tracks page: an **Automatic terrain flippers** button and a live state
  display.
- `robot_lab_utils/gait_recovery.py` with `gait_controller`: measured slip or
  terrain support loss now pauses the gait instead of halting it. Feet return
  to the source stance, and walking resumes at half speed for 3 s once
  support returns and slip has been clear for 1 s. Each pause is bounded to
  10 s and at most 3 are allowed per minute, otherwise the gait stops. Tilt
  and body contact still stop at once.

## Results: one 0.18 m step face, 0.15 m/s forward, 40 s

The vehicle starts at (−6.65, −5.9) heading +y toward the first step's
0.18 m side face at y = −4.55.

| Run | End y / z | Max nose-up pitch | Outcome |
|---|---|---|---|
| Fixed flippers (`fixed-2`) | −4.605 / 0.228 | 0.657 rad | Reared up on the edge; the existing 0.6 rad tilt guard stopped the drive. Not on the tread |
| Auto v1: front flippers press in climb (`auto-1-front-press`) | −4.886 / 0.105 | 0.130 rad | Pitch stayed low, but the tracks stalled at the face |
| Auto v2: rear presses only in climb (`auto-2-rear-late`) | −4.607 / 0.227 | 0.652 rad | Rear flippers arrived too late (0.2 rad/s slew); tilt guard |
| Auto v3: rear pre-pressed in approach (`auto`) | −4.605 / 0.229 | 0.657 rad | Rear flippers were at −0.69 rad, but the hull weight back-drove them to −0.03 rad during the climb; tilt guard |

**Finding:** with the source derivative's 4 Nm flipper PID cap, the flippers
cannot lift or support the hull. A step climb needs a stronger flipper
actuator or a different chassis, not a different policy. The sensing and
state sequence did work on the real depth camera and IMU (approach, then
climb). No variant was changed.

Gait slip recovery has unit tests only; no slip mission was run.

## Retained failures

- The first fixed run spawned at (−7.9, −4.0) against the west wall. The
  wall is only 0.8 m from the first step, and the vehicle with flippers is
  1.08 m long. It wedged there and the rear flippers rode up the wall
  (`fixed-wedged-at-wall-result.json`). The spawn was moved to face the
  step's side.
- The policy initially took ROS pitch (positive nose-down) as nose-up. It
  was fixed before the auto runs.
- A new test module clashed with an existing `test_tracked_flippers.py` and
  was renamed `test_flipper_terrain_policy.py`.

## Reproduce

```bash
python3 launch_tracked.py <dir>/launch.log robot_model:=lab_tracked_articulated map_name:=terrain_stairs \
  spawn_x:=-6.65 spawn_y:=-5.9 spawn_yaw:=1.5708
CASE_OUTPUT=<dir>/auto CASE_FLIPPERS=auto CASE_DURATION=40 python3 stairs_probe.py   # or CASE_FLIPPERS=fixed
```

Traces and logs are in
`/workspace/molar/robot_lab_runtime/flipper-stairs-20261114/`.
