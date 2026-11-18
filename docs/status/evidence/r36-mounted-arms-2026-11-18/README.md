# R3.6: KUKA iiwa and Valkyrie forearms controllable with lab servos (November 18, 2026)

**Status: implemented, experimental.** Display-only profiles drop from 69
to 66, and profiles with no control interface from 15 to 12.

## What was added

**Lab servos** (`robot_lab_utils/urdf_lab_servos.py`, the rule used for
UR5/UR10/YuMi) for three more robots, set in `installed_overlays.yaml`:

| Robot | Joints | Anchoring |
|---|---|---|
| KUKA iiwa (robot_assets) | 7 | `fixed_base: true` in its articulation config |
| Valkyrie forearm stand, left | 16 | its URDF root is `world` |
| Valkyrie forearm stand, right | 16 | its URDF root is `world` |

**`fixed_base`:**

- The iiwa URDF has no `world` link, so on a floating base it would fall
  over. (PyBullet's own example loads it with a fixed base.)
- With `fixed_base`, the launch writes a copy of the resolved description
  with a `world` link fixed to the root, under
  `robot_lab_runtime/mounted_models/`, and gives every simulator that copy.
- The existing fixed-base handling then applies unchanged on all four
  simulators.
- Code: `robot_lab_utils.urdf_base.mount_to_world`.

**`fixed_to_world`** now accepts any non-floating joint out of a `world`
root. Valkyrie's first forearm joint turns against `world`, and before this
the forearm was spawned free.

**Refused: the R2 family (8 variants).**

- Their hand joints declare effort 10,000,000 and ±4 rad: placeholders, not
  actuator limits.
- Servos built from those values froze the joints (first screens, kept).
- `urdf_lab_servos` now rejects efforts over 10,000.

**GUI reasons:** `display_reasons` now says why lab servos cannot be derived
for a robot:

- placeholder limits (R2);
- unlimited joints (Kinova, Fetch, PR2).

## Measured (`scripts/screen_urdf_arm.py`, Display, `nav_empty`)

| Robot | Backend | Worst steady error | Stop | Lost heartbeat |
|---|---|---|---|---|
| KUKA iiwa | MuJoCo | 0.028 rad | holds | holds |
| KUKA iiwa | PyBullet | 0.029 rad | holds | holds |
| Valkyrie forearm left | MuJoCo | 0.027 rad | holds | holds |
| Valkyrie forearm left | PyBullet | 0.030 rad | holds | holds |
| Valkyrie forearm right | MuJoCo | 0.030 rad | holds | holds |

The Valkyrie wrist roll is blocked at +0.5 rad and was retried the other
way (`blocked_direction`).

## Fixed on the way (both kept as failed screens)

1. **The arm screen sent unreachable targets.** It sent each joint back to
   its measured start. A Valkyrie finger that settled a hair past its 0 rad
   stop (reported as −0.0) got a target the contract refuses, so the joint
   never returned (`screen-…forearm_left-mujoco.json`). The screen now
   clamps the start into the joint's limits.
2. **The MuJoCo spawner crashed on many-joint robots.** Its gravity
   feed-forward read each joint's actuator from a cache that is filled when
   the joint gets its first target. With 18–54 joints, some had no target
   yet, so the spawner raised `KeyError` and never initialised the arm
   (`screen-…r2b_sim_upperbody_dynamics-mujoco.json`). Both paths now look
   the actuator up the same way.

## Limits

- One screen per robot and backend: the right forearm on MuJoCo only;
  Isaac and Gazebo not screened.
- Lab servos, not vendor tuning.
- Still with no control interface (12): eight R2 variants, Kinova, Fetch,
  PR2 and Ginger.
- Measured while Go2 stairs training loaded the CPU. Launch logs are on SSD
  under `robot_lab_runtime/r36-mounted-20261118`.
