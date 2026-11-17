# URDF arms on Gazebo: shared articulation contract and pick and place (November 17, 2026)

**Status: implemented, experimental.**

URDF robots without a `<ros2_control>` block (the URDF Panda, UR5, UR10,
YuMi) now run the shared URDF articulation contract on Gazebo Fortress, as
they already did on PyBullet, MuJoCo and Isaac. The URDF Panda pick and
place passes on Gazebo, so it now passes on all four backends.

These runs were measured while a Go2 training job used most of the CPU (load
about 9 on 12 cores). Times are therefore not comparable with the earlier
quiet-host runs.

## What was added

- **`urdf_gazebo_articulation.py`** (robot_lab_bringup):
  - runs `ArticulationROS` (`/molar/articulation/{state,command,heartbeat,stop}`)
    on the bridged `/joint_states`;
  - sends each joint's bounded contract reference to the display hold's
    `JointPositionController`;
  - if feedback is lost for 0.3 s, it stops and holds the last measured pose.
- **Display hold topics** (`robot_lab_utils.gazebo_display`):
  - **Command topic:** each hold controller now listens on
    `hold_command_topic(model, joint)` = `/model/<model>/joint/<joint>/cmd_pos`.
    Gazebo's default `.../joint/<j>/0/cmd_pos` cannot be bridged, because ROS
    rejects topic name tokens that start with a digit.
  - **Mimic joints:** these now get a hold controller too. Gazebo ignores
    `<mimic>`, so the relay drives them as contract couplings (the Panda's
    second finger, YuMi's grippers).
- **`gazebo_free_bodies.py`:** publishes loose-object truth on Gazebo, the
  same `/simulation/free_bodies` JSON that MuJoCo, PyBullet and Isaac
  publish. It reads `/world/<world>/dynamic_pose/info` and keeps the world's
  dynamic bodies.
- **Launch:** on Gazebo, a non-MOLAR robot with an articulation profile and
  no ros2_control gets:
  - the command bridges;
  - the relay;
  - `urdf_pick_place.py` when the profile has `pick_place`;
  - the free-body truth node when the world has dynamic bodies.

  MOLAR keeps `molar_gazebo_control.py`.
- **`screen_urdf_arm.py`:**
  - **Stricter pass rule:** a move now passes only if the joint, after
    reaching its target, stays within twice the tolerance (0.06 rad) for one
    second. Before, reaching the target once was enough; this missed the
    UR10 case below.
  - **`--trace`:** records every joint and the reference during each move.
  - **Earlier screens still pass:** the R3.6 worst case was 0.030 rad.

## Measured

### URDF Panda pick and place (`arm_pick_place_lab`)

Run with `scripts/run_with_launch.py` and `scripts/arm_pick_place_check.py`:
pick `pickup_cube`, place it on `place_table`.

| Run | Result | Time | Placed height error | Inside support | Movement in last second |
|---|---|---|---|---|---|
| 1 | not started: relay crashed (topic name) | — | — | — | — |
| 2 | passed | 27.8 s | 0.0 mm | yes | 0.0 m |
| 3 | passed | 30.6 s | 0.0 mm | yes | 0.0 m |

**Observation:** in run 2 the cube slid about 2 cm along the finger pads as
it was lifted, and kept that offset through the carry. Placement was not
affected. Earlier runs on other backends: PyBullet 23.1 s, MuJoCo 22.3 s,
Isaac 20.2 s.

### Arm screens (Display, `nav_empty`)

| Robot | Joints | Worst steady error | Stop | Lost heartbeat | Result |
|---|---|---|---|---|---|
| UR5 | 6 (shoulder_lift retried the other way) | 0.026 rad | holds, 0 drift | holds, 0 drift | passed |
| YuMi | 16 + 2 coupled | 0.030 rad | holds | holds | passed |
| UR10, on the floor | 6 | 1.48 rad (wrist_1) | holds | holds | **not passed** |
| UR10, spawned 0.5 m up | 6 | 0.026 rad | holds | holds | passed |

**UR10 on the floor** (three runs, all with the same failure):

- The zero pose lays the arm out horizontally about 0.13 m above the floor.
- On the return move, wrist_1 swings between about −1.5 and 0 rad at the
  hold's 5 rad/s cap, while the contract reference ramps smoothly from 0.49
  to 0 (`--trace` file).
- The first run passed under the old rule, which ignored what happened after
  the target was reached.

Spawned 0.5 m higher, every joint settles, so the trigger is contact with the
floor. Arms should be mounted on a stand or table on Gazebo. UR5 and YuMi
are unaffected.

## Files

- `pick-place-panda{1-failed,2,3}.json`
- `screen-*.json`
- `launch-log-sha256.txt`: hashes of the launch logs, which stay on SSD
  under `robot_lab_runtime/gazebo-arm-20261117`.

## Limits

- **Few runs:** two Panda pick-and-place runs and one screen per arm; all
  were measured under CPU load.
- **TIAGo on Gazebo not tried:** it has a ros2_control description, so it
  needs a separate path.
- **Gravity:** the display hold disables link gravity on Gazebo; it is a
  preview, not a physical arm. The pick and place still lifts a cube that
  has gravity.
- **Untested robots:** robots with an articulation but no `pick_place`
  profile get joint control only. Other display-hold robots now get mimic
  holds and renamed topics; they were not screened again.
