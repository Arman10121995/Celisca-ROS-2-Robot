# Pick and place with a URDF arm (PyBullet, MuJoCo, Isaac and Gazebo)

A backend-neutral pick and place for URDF arms whose profile has a
`pick_place` block. The workflow is experimental. TIAGo can also use
configured RGB-D targets for the grasp. The catalog includes:

- the robot_assets Franka Panda (`asset_franka_panda_panda`), a fixed arm;
- the official PAL TIAGo (`asset_pal_tiago`), a mobile manipulator, which
  **fetches**: it drives to the object, picks it, carries it and places it.
  See [Fetch with TIAGo](#fetch-with-tiago-mobile-manipulation).
- the myAGV Pro with myCobot Pro630 and the F100 gripper, measured through
  a complete MuJoCo Display fetch. [F100 setup](myagv-pro.md).

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

## Pick the cube with the workcell camera

Select the URDF arm and hand variant of Franka Panda, **MuJoCo**,
**Display**, and the **Cube** variant of the fixed-arm pick-and-place lab.
Enable experimental robot workflows if the robot selector requires it.
After Launch, select `pickup_cube` and `place_table` in the arm panel,
choose **RGB-D camera** as the pose source, and request pick and place.

The overhead workcell camera publishes synchronized colour, depth and
calibration. The executor checks three fresh estimates against each other
before it plans the grasp. It uses physical object feedback to judge lift
and release. The fixture recognizes the authored red 40 mm cube.

To add a stationary camera to another MuJoCo fixed-arm workcell, set
`workcell_rgbd_pose` on the map entry in `sim_maps.yaml`. Give the camera's
world XYZ position in metres and WXYZ orientation of its ROS optical frame.
Add `rgbd_targets.yaml` alongside `worlds/` in the map directory with the
object identity, collision dimensions and measured HSV range. Keep the
object visible from the arm's start pose.

## What it does

`urdf_pick_place.py` plans with `robot_lab_mujoco/arm_pick_place.py`, using a
private MuJoCo model of the arm and the world's static collision primitives.
Nothing in the plan touches the plant. It executes through the articulation
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

The planner transforms each reported world model pose through the declared
SDF link and collision poses. It uses the physical collision centre and axes
for grasp, carried clearance and release checks. Configured camera estimates
already use that centre. The support selector uses level collision boxes.
During the lift, only the source surface supporting the object is exempt
from payload collision checks. Other tables remain obstacles.
The private arm scene checks large walls, sloped collision boxes, cylinders,
spheres and multi-part tables. For each model, the support selector uses
the highest eligible level box. The private model welds the robot base to
its measured pose and omits the broad ground slab. Mesh and heightfield
collision planning remain open.

The carried payload is a mocap body in the planning model. The planner
queries its distance to each static obstacle because the ordinary contact
search skips pairs of bodies without dynamic degrees of freedom.
[MuJoCo collision detection](https://mujoco.readthedocs.io/en/latest/computation.html#collision-detection).
Isaac's loose-body importer keeps the model frame as the rigid body and
places offset box or cylinder colliders beneath it. It publishes the model
pose on `/simulation/free_bodies`. The Isaac Sim 6.0.1 stage check covered
collider transforms, mass and friction material bindings. Physical
pick-and-place with offset objects across backends remains to be measured.
Mobile fetch checks its direct approach, docking and retreat against the
top-down footprints of authored static boxes, spheres and cylinders. These
checks include walls and posts that are unavailable as drop supports. Nav2
owns the longer transit and uses its own costmaps. The direct checks cover
primitive collision geometry, with mesh and terrain clearance still pending.

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

## Pick a cylinder on its side

Choose **Arm pick-and-place lab → Cylinder on its side** in the map selector.
Launch the URDF Franka Panda in **Display**, press **Home (direct)**, then choose
`pickup_can` and `place_table` in **Pick / place**. The planner estimates the
horizontal axis from the measured simulator pose, closes across the diameter
and checks the full projected cylinder against the support. It checks the
measured resting axis and height after release. The scene and private Panda
plans are checked. A physical side-lying run remains experimental.

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
carried-arm radius for the other legs. There is no Nav2 in Display.

**Driving with Nav2 (Nav mode, experimental).** Launch with `mode:=nav
allow_experimental_assets:=true` (in the GUI, tick **Enable experimental
robot workflows** and choose **Nav**). The node then sends the transit legs to Nav2:
to the run-up point before the pickup table, and, while carrying, to the
run-up point before the drop table.

- **Hand-off:** once AMCL puts the base within 0.15 m of the run-up point,
  the node requests cancellation. It waits for the owned action to end and
  the measured base to stop before starting the 0.6 m truth-steered dock.
- **Short legs:** a leg under 0.5 m stays straight.
- **Carry footprint:** after the measured lift, the node sends the base
  circle, projected source collision bounds at measured joints and the
  oriented payload bounds. Both Nav2
  costmaps must publish matching polygons before loaded transit. The loaded
  footprint remains through docking and is cleared after supported release.
  If Stop interrupts the carry, **Run fetch** stays disabled until you start
  a new simulation.
- **Forcing straight legs:** `--transit straight` (request
  `"transit": "straight"`) keeps the straight legs in Nav mode.

[Measured Nav2 runs](../status/evidence/r59-tiago-nav2-fetch-2026-11-17/README.md).

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

### Grasp from the head camera

On the Fetch section, set **Object pose → RGB-D camera** before **Run
fetch** (script: `arm_pick_place_check.py --pose-source rgbd`).

1. After the drive, TIAGo tilts its head to the profile's
   `pick_place.look` pose.
2. Each new RGB-D frame is fitted using the object's declared shape, size
   and OpenCV HSV range. The lab reads `rgbd_targets.yaml` for boxes and
   upright cylinders and still accepts `rgbd_boxes.yaml` for existing worlds.
   Both formats must name a loose object with matching collision dimensions
   and shape in the selected world.
3. Three estimates within 1 cm of each other make the grasp pose.
4. The place uses where the object sat in the hand at the grasp.

Simulator truth still picks the table to drive to and judges the result.
The receipt's `perceived` block records the estimate and its error against
truth. A world without a binding for the object offers only **Simulator
truth**. If no estimate comes within 30 s, the task stops, and the last
camera frame is saved next to the receipt.
[Measured runs](../status/evidence/r59-tiago-rgbd-fetch-2026-11-18/README.md).

To exercise the cylinder fit, select **Mobile fetch lab → Upright cylinder
for TIAGo**, choose `fetch_can`, and use the camera pose option. The estimator
uses the observed circular lid and the declared height to recover the body
center. Circular symmetry gives no observable yaw, so the estimate uses a
canonical upright orientation. A clipped or incomplete lid is rejected.
The cylinder fixture and estimator passed source checks. A physical camera
fetch of the cylinder remains unmeasured.

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
