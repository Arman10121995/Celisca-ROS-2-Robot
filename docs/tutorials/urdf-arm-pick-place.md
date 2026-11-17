# Pick and place with a URDF arm (PyBullet and MuJoCo)

A backend-neutral pick and place for fixed URDF arms whose profile has a
`pick_place` block. The first such arm is the robot_assets Franka Panda
(`asset_franka_panda_panda`). It is experimental, and object poses come from
simulator truth.

## Run

From the GUI:

1. Choose **Manipulators → franka_panda → asset_franka_panda_panda**, world
   **arm_pick_place_lab**, simulator **PyBullet** or **MuJoCo**, task
   **Display**, and launch.
2. Open the **Mobile arm** page. Press **Home (direct)** and wait for it to
   settle. The Panda's all-zero start pose is in self-collision, and the
   planner refuses it.
3. In **Pick / place (simulator truth)**, choose `pickup_cube`, choose
   `place_table`, keep the 0.10 m lift and press **Run pick / place**.
4. **Stop task** or **Stop / hold** stops it at any time. The last result
   and its receipt path appear under the buttons.

From the command line, with a launch and a checker the script owns:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash && source scripts/ssd_env.sh
python3 scripts/run_with_launch.py --log run.log --launch \
  "robot_model:=asset_franka_panda_panda simulator:=pybullet mode:=display map_name:=arm_pick_place_lab" \
  -- python3 scripts/arm_pick_place_check.py --object pickup_cube --support place_table
```

## What it does

`urdf_pick_place.py` plans with `robot_lab_mujoco/arm_pick_place.py`, using a
private MuJoCo model of the arm and the world's static boxes. Nothing in the
plan touches the plant. It executes the plan through the articulation
contract, in these stages:

1. open
2. above the object
3. straight down
4. close
5. lift
6. over the support
7. straight down
8. release
9. retreat

**Checks.** Each stage must pass before the next one starts:

- **Grasp:** the fingers stop on the object.
- **Lift:** the object rises with the tool and holds for a second.
- **Place:** the object rests on the support.

**Receipts.** Each task writes one to `$ROBOT_LAB_RUNTIME_ROOT/pick-place/`.

**Coupled fingers.** The Panda's second finger is coupled *mechanically*,
like the real one-motor hand. PyBullet uses a gear constraint and MuJoCo a
joint equality. Independent fingers lost the cube.

[Measured runs and limits](../status/evidence/r57-urdf-arm-pick-place-2026-11-17/README.md).

## Add another arm

Give the arm's installed profile these blocks:

- an `articulation` block (`panel: arm`, position channels);
- a `pick_place` block:
  - `arm_joints`, `finger_joints`;
  - `tool_body`: the MuJoCo body after fixed links merge;
  - `tcp_offset`, `tool_yaw_offset`;
  - `finger_open`, `finger_closed`, `max_object_width`.

Couple fingers that share one motor with `mechanical: true`. The world needs
loose objects as single-link primitive models with
`<static>false</static>` and a mass.
