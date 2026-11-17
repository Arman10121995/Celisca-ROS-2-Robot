# Pick and place with a URDF arm (PyBullet, MuJoCo, Isaac and Gazebo)

A backend-neutral pick and place for URDF arms whose profile has a
`pick_place` block. It is experimental, and object poses come from simulator
truth. Two robots have the block:

- the robot_assets Franka Panda (`asset_franka_panda_panda`), a fixed arm;
- the official PAL TIAGo (`asset_pal_tiago`), a mobile manipulator, which
  **fetches**: it drives to the object, picks it, carries it and places it.
  See [Fetch with TIAGo](#fetch-with-tiago-mobile-manipulation).

## Run

From the GUI:

1. Choose **Manipulators → franka_panda → asset_franka_panda_panda**, world
   **arm_pick_place_lab**, simulator **PyBullet** or **MuJoCo**, task
   **Display**, and launch. Isaac and Gazebo pass from the command line too
   (`simulator:=isaac` or `simulator:=gazebo`).
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
joint equality. Independent fingers lost the cube. Gazebo ignores `<mimic>`,
so there the second finger has its own hold controller and the contract
drives it as a coupling.

**Gazebo.** URDF arms without `<ros2_control>` get one JointPositionController
per joint (the display hold, on `/model/<robot>/joint/<joint>/cmd_pos`).
`urdf_gazebo_articulation.py` runs the same contract on them, and
`gazebo_free_bodies.py` publishes the object truth. Mount arms above the
floor: the UR10 lying at its zero pose chattered on floor contact.

[Measured runs and limits](../status/evidence/r57-urdf-arm-pick-place-2026-11-17/README.md).

## Fetch with TIAGo (mobile manipulation)

From the GUI: choose **Mobile manipulators → PAL Robotics TIAGo** (official source variant),
world **mobile_fetch_lab**, simulator **MuJoCo** or **PyBullet**, task
**Display**, and launch. On the **Mobile arm** page press **Home (direct)**,
then in **Fetch** choose `fetch_block` and `drop_table` and press **Run
fetch**. From the command line:

```bash
python3 scripts/run_with_launch.py --log run.log --launch \
  "robot_model:=asset_pal_tiago simulator:=mujoco mode:=display map_name:=mobile_fetch_lab" \
  -- python3 scripts/arm_pick_place_check.py --robot asset_pal_tiago \
     --object fetch_block --support drop_table --probe-interlock
```

**Phases.** One request runs five phases:

1. **to_object:** drive to a standoff facing the object's table edge.
2. **pick:** planned on arrival from the measured base pose.
3. **carry:** back off, turn, drive to a standoff at the destination
   table.
4. **place:** planned on arrival.
5. **clear:** back off.

**Driving.** Drives are straight legs on `/cmd_vel`, steered by
`/odom/ground_truth`. A request is refused when a leg would pass too close
to a table: the base radius for straight approaches and back-offs, the
carried-arm radius for the other legs. There is no Nav2 in this mode.

**Base and arm never move together.**

- The node sends no drive command while the articulation is active, and
  no arm target while the base moves.
- The articulation contract enforces the same: an active arm inhibits
  `/cmd_vel`, and a moving base makes the contract reject arm targets.
- `--probe-interlock` checks both from outside. It sends drive commands
  during arm motion, and an arm target during the carry.

**Carrying.**

- `grip_joints` let the fingers keep squeezing across the Stop that ends
  each arm phase and while the base drives. TIAGo keeps up to 20 mm, its
  16 N grasp force.
- The block must keep its pose relative to the base within 3 cm.
- The receipt has a 2 Hz trace of the base, object and fingers.

**Why a tall block.** PAL's finger pads reach 0.09 m past
`gripper_grasping_frame`. A 40 mm cube on a table cannot be gripped from
above, so the world uses a 40 × 40 × 150 mm, 100 g block. The planner grips
it higher up, so the fingertips clear the table.

## Add another arm

Give the arm's installed profile these blocks (TIAGo's in
`installed_overlays.yaml` is the mobile example; add `mobile` and
`grip_joints` for a mobile base):

- an `articulation` block (`panel: arm`, position channels);
- a `pick_place` block:
  - `arm_joints`, `finger_joints`;
  - `tool_body`: the MuJoCo body after fixed links merge;
  - `tcp_offset`, `tool_yaw_offset`;
  - `finger_open`, `finger_closed`, `max_object_width`.

Couple fingers that share one motor with `mechanical: true`. The world needs
loose objects as single-link primitive models with
`<static>false</static>` and a mass.
