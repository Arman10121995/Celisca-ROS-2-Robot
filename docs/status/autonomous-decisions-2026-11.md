# Autonomous roadmap work: decision log (November 2026)

On 2026-11-17 the user asked Claude to work through the whole roadmap without
waiting for input, choosing the best option whenever a decision arises, and
to report the decisions at the end. Claude is the only agent on the
repository.

Each entry gives the date, the roadmap item, the decision and the reason.
Results and failures live in the linked receipts; this file records only the
choices.

## Decisions so far

| Date | Item | Decision | Why |
|---|---|---|---|
| 11-15 | R13.3 | Deployed the multi-map retrain at score 0.75, not 0.6 | At 0.75, false positives drop sharply (e.g. cable 6 → 2, shoe 1 → 0) for a small recall cost |
| 11-15 | R13.3 | Kept ideal segmentation as the default detector | The learned camera passed only one run, on a training map |
| 11-15 | R12 | Kept the user's Go2W wheel-gain edit (kd 1.0) although robot_lab trained with 0.5 | It measured better (strafe 0.206 → 0.248 m/s) |
| 11-15 | R12 | Added a `wheeled_walk` task with robot_lab's own Go2W reward weights instead of re-tuning `walk` | Vendor recipe; the quadruped air-time reward discouraged rolling |
| 11-15 | R5.3 | Trained BHL from a generated template of the vendored upstream config, not a hand-written contract | One source of truth for joints, gains and limits |
| 11-15 | R5.3 | Stopped BHL training after four failed CPU runs and recorded it as not passed | From scratch it never steps; warm starts collapse; this host has no usable GPU |
| 11-15 | R11 | Gazebo MOLAR belt: used the existing private DART plugin rather than writing a new belt system | It already existed (gz-sim#2495 workaround for the tracked vehicle) |
| 11-15 | R11 | Modelled the station as a passive roller table (μ 0.05) | A static 0.8-friction table holds a box partly on the deck; real stations have rollers |
| 11-15 | R11 | Limited the Gazebo MOLAR front scan to ±105° | A 360° scan from the front mount hit the robot's own body (216/540 beams) |
| 11-15 | R11 | Used MOLAR's own ros2_control controllers in every Gazebo mode, with the EKF owning odom → base_link | The shared wheel controller is not in its description |
| 11-17 | R5.2 | Recorded `get_up_roll` (2/10) as not passed and moved on | One shaping change; further progress needs curriculum or GPU-scale training |
| 11-17 | R5.7–R5.9 | Took over the former Codex manipulation lanes (user approved) | No Codex commits since 11-15; the lanes are still partial |
| 11-17 | R5.7 | Used the robot_assets URDF Panda for the second-backend pick and place, not the native MJCF Panda | The URDF path runs on PyBullet, MuJoCo and Isaac through one contract; the native Panda is MuJoCo-only |
| 11-17 | R5.7 | Gave the URDF Panda the TIAGo servo formula (kp 40 × effort, kd 8 × √effort) | One declared lab rule for URDFs without ros2_control gains |
| 11-17 | R5.7 | Planned with a private MuJoCo model of the URDF and own IK, not MoveIt | Backend-neutral; MoveIt needs an SRDF and controllers this URDF does not ship |
| 11-17 | R5.7 | Welded URDFs whose root is a `world` link with fixed joints on every backend | They are fixed arms; floating them made the base drift |
| 11-17 | R5.7 | Coupled the Panda fingers mechanically (PyBullet gear, MuJoCo equality) | The real hand has one motor; independent fingers lost the cube; PyBullet's own Panda example does this |
| 11-17 | R5.7 | Refused to plan from a start pose in collision; the operator moves Home first | Same as MoveIt; the Panda's zero pose is in self-collision |
| 11-17 | R5.7 | Renamed `/mujoco/free_bodies` to `/simulation/free_bodies` and added it to PyBullet | One truth topic for every backend |
| 11-17 | R5.7 | Accepted that PyBullet loose objects are not in the RGB-D mirror camera | Pick and place uses truth; perception is a separate remaining item |
| 11-17 | R5.7 | Rasterized the new world's 2D map from its boxes rather than running the Gazebo map generator | Exact for box worlds and needs no simulator run |
| 11-17 | R5.9 | Built the mobile mission on TIAGo with the new URDF pick-and-place executor, not on the native Stretch pipeline | TIAGo runs on the shared articulation contract on three backends, and its base/arm interlock already exists |
| 11-17 | R5.9 | Planned each arm phase on arrival from the measured base pose | Base stops are never exact; a plan from the planned stop would miss by centimetres |
| 11-17 | R5.9 | Drove straight, truth-steered legs with table-clearance checks instead of Nav2 | TIAGo's adapter is Display-only (no Nav2 mode); the legs are checked and refused when blocked |
| 11-17 | R5.9 | Added `grip_joints` to the articulation contract: a still gripper keeps a bounded squeeze across Stop and base motion, and counts as active only while it moves | Otherwise a squeezing gripper inhibits the base for ever, and a base-motion hold drops the object |
| 11-17 | R5.9 | Set TIAGo's retained squeeze to its 16 N grasp force (20 mm), not the 4 N default | At 4 N the block crept down the fingers in MuJoCo |
| 11-17 | R5.9 | Used a 40 × 40 × 150 mm block, not a 40 mm cube | PAL's finger pads reach 9 cm past the grasp frame; a top grasp of a cube on a table is impossible |
| 11-17 | R5.9 | Planner: try all grasp sides and yaws, raise the grasp until the fingertips clear the table, route joint moves via high poses and then the repo's bounded RRT-Connect | Linear joint moves swept TIAGo's long fingers through the table edge |
| 11-17 | R5.9 | MuJoCo grasping contacts (elliptic cone, impratio 10) only for robots with `grip_joints` | Even at 16 N the block crept 1.7 mm/s with the default pyramidal cone; scoping keeps earlier measured runs unchanged |
| 11-17 | R5.9 | PyBullet: grip force proportional to error, a symmetric-grip gear for TIAGo, and friction anchors on fingers and loose objects | The saturated motors let the fingers drift sideways, and the block yawed 0.5 rad in the grip while turning |
| 11-17 | R5.9 | Judged finger stillness from positions, not reported velocities | Bullet's reported finger velocities jitter while squeezing |
| 11-17 | R5.8 | Lifted fixed-mount hands relative to a dropping lab support stage instead of mounting them on a moving stage or an arm | The hand model stays untouched (DEX-EE has four separate roots and PID plugins); the grasp carries the full weight either way |
| 11-17 | R5.8 | Made the fixture pedestal collide only with the object | It stood between the Robotiq fingers; the linkage chattered and the grasp never closed |
| 11-17 | R5.8 | Debounced grasp holding (0.3 s, object still) and loss (0.1 s) | A first fingertip touch bounces; it was latched as contact lost |
| 11-17 | R5.8 | Settled the Robotiq unit test before grasping instead of loosening the effort guard | The 4 N transient is the model's first-step settling, not a grasp |
| 11-17 | R5.8 | Recorded DEX-EE as not passed and moved on | Five friction/force/geometry variants all tilt; the next step is grasp synthesis, a separate task |
| 11-17 | R13.3 | Fine-tuned retrain 2 on new weighted renders instead of retraining from COCO | Five epochs on CPU (~45 min) instead of 16 (~2.3 h); provenance chains through the installed parent |
| 11-17 | R13.3 | Installed retrain 3 as `auto` | It is better or equal on most held-out cells, and the learned mission still passes with more clearance |
| 11-17 | R13.3 | Marked R13.3 implemented (done-when met); repeats and held-out rooms belong to V1 | The done-when (zero-contact job passing cleaning_screen) holds for both detectors |
| 11-17 | R3.6 | Gave UR10, UR5 and YuMi declared lab servos; kept Kinova and KUKA display-only | The first three are welded to `world` with full limits; the other two would float and have controllable Menagerie siblings |
| 11-17 | R6.6 | Polylines become convex triangle-prism STL meshes in the shared SDF reader | No consumer changes needed, and MuJoCo's convex hulls stay exact |
| 11-17 | R6.6 | Read "upper floors" as already covered by `--height` slicing | No installed world has two storeys in one file; the CELISCA floors are separate worlds |
| 11-17 | R5.7 | Isaac loose objects as `DynamicCuboid`s reported through the runtime's state events | Same truth topic as the other backends, no new ROS path in the Isaac process |
| 11-17 | R5.7 | Planner tries up-and-over routes before direct sweeps | A direct sweep grazed the TIAGo block on Isaac; standard practice, and the MuJoCo/PyBullet regressions still pass |
| 11-17 | R5.9 | Reverted the fingertip-clearance pre-grasp and recorded TIAGo-on-Isaac as not passed | The change broke a passing MuJoCo mission; the Isaac cause was not visible in the joint/TCP data |
| 11-17 | R3.6 | Arm screen retries a blocked joint direction instead of choosing a new home pose | Tests the servo, not the floor; the blocked attempt stays in the report |
| 11-17 | R11 | Reverted a trial raise of MOLAR's base-motion limits and recorded the PyBullet transfer as not passed | Raising them did not help (the base yawed 0.127 rad/s), and loosening the interlock would hide a real coupling |
| 11-17 | R11 | Ran the CELISCA transfer in Gazebo with a station spawned ahead of MOLAR, and raised display-mode spawner switch timeouts to 60 s | The done-when needs a transfer on a CELISCA floor; the slow world timed out the default 5 s controller switch |
| 11-17 | P4 | Did not pursue RTF ≥ 1 further for now | Every remaining option (per-triangle prisms, fewer triangles, SDF/convex static collision) changes contacts or adds per-step cost of the same size; 92k triangles on celisca_floor_1 |
| 11-17 | R5.2 | Continued `get_up_roll` training to 4500 iterations from its 1500-iteration checkpoint | No open-source plain-Go2 recovery policy found (only Go2+Z1); side-lying starts are already a third of the start pool |
| 11-17 | R5.7 | Ran Gazebo arm tests while Go2 training used the CPU (the user asked to keep working during training), and labelled the times as loaded | Function and pass/fail are unaffected by load; the evidence says the times are not comparable |
| 11-17 | R5.7 | Gave the Gazebo display hold valid command topics (`/model/<m>/joint/<j>/cmd_pos`) and hold controllers on mimic joints | ROS rejects Gazebo's default `/0/` topic token; Gazebo ignores `<mimic>`, so the second finger needs its own controller |
| 11-17 | R3.6 | Made the arm screen require the error to stay within 2 × tolerance for a settled second, and recorded the UR10 lying on the floor as not passed on Gazebo | The old rule passed a wrist swinging 1.5 rad; the earlier PyBullet/MuJoCo screens still pass (worst 0.030 rad) |
| 11-17 | R5.9 | Nav2 drives only the fetch's transit legs; the 0.6 m docking moves and arm planning stay on simulator truth | AMCL error is 0.07–0.13 m in the two-table world, too coarse to dock for a ~1 cm grasp; perception is a separate item |
| 11-17 | R5.9 | Hand Nav2 legs over to docking at 0.15 m by localization, and keep legs under 0.5 m straight | DWB circled the 7 cm goal tolerance for over 100 s, and aborted a 0.23 m leg after turning away |
| 11-17 | R5.9 | Carry footprint = hull of the base circle and an arm/object strip, not the 0.85 m sweep circle | The grid planner plans for the base centre only; the big circle then overlapped a table and every replan failed at its start |
