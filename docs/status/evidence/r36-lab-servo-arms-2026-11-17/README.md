# R3.6 lab servos for URDF arms without a control interface (November 17, 2026)

**Status:** implemented, experimental. UR10, UR5 (robot_assets `ur5_gripper`)
and ABB YuMi are now controllable on PyBullet and MuJoCo. Display-only
profiles fall from 73 to 69, and profiles with no control interface from 19
to 15 (this also drops the URDF Panda's stale entry, controllable since
R5.7).

## What was added

**`robot_lab_utils/urdf_lab_servos.py`** derives a shared URDF
articulation config from a URDF's own joint limits:

- every limited revolute or prismatic joint becomes a position channel;
- kp = 40 × effort and kd = 8 × √effort;
- speed capped at 0.35 rad/s (0.05 m/s for prismatic joints);
- source mimic joints become couplings (YuMi's two grippers).

This is the rule already declared for the URDF Panda and TIAGo. These are lab
simulation servos, not vendor tuning.

**Overlays:** `installed_overlays.yaml` gives the three robots these
servos. All three URDFs are welded to `world` and have complete limits.

**Kept display-only:** Kinova and KUKA iiwa. Their roots are not anchored,
so they would float, and both have controllable Menagerie siblings.

## Measured (`scripts/screen_urdf_arm.py`, Display, `nav_empty`)

| Robot | Backend | Joints | Worst steady error | Stop | Lost heartbeat |
|---|---|---|---|---|---|
| UR5 | PyBullet | 6 settle out and back | 0.016 rad | holds, 0 drift | holds, 0 drift |
| UR5 | MuJoCo | 6 | 0.027 rad | holds | holds |
| UR10 | PyBullet | 6 | 0.025 rad | holds | holds |
| UR10 | MuJoCo | 6 | 0.029 rad | holds | holds |
| YuMi | PyBullet | 16 + 2 coupled | 0.029 rad | holds | holds |
| YuMi | MuJoCo | 16 + 2 coupled | 0.030 rad | holds | holds |

**First screens** (`first-*.json`): five failed, every time on a +0.5 rad
shoulder move from the zero pose. The arm met the floor or its own body:

- UR shoulder_lift stopped at 0.06–0.27 rad;
- the UR10 elbow on PyBullet stopped at 0.11 rad;
- YuMi joint_2 stopped at 0.18–0.19 rad.

The servos held, and every return settled. The screen now retries a blocked
direction with −0.5 rad and records it (`blocked_direction`). The reruns
all pass.

## Limits

- Joint control only; no planning or pick and place for these arms yet.
- Gazebo needs a ros2_control description, which these URDFs do not ship.
  (Later the same day: a Gazebo relay drives them without one; see
  [the Gazebo receipt](../r57-gazebo-urdf-arms-2026-11-17/README.md).)
- Isaac is not screened.
