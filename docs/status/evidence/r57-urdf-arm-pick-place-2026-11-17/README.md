# R5.7 URDF-arm pick and place on MuJoCo and PyBullet (November 17, 2026)

**Status: implemented, experimental. R5.7 done-when met for one arm.**

The done-when is a GUI pick → place of a named object, with a measured object
pose, on MuJoCo plus one other backend. It was met with the URDF Franka Panda
(`asset_franka_panda_panda`) on PyBullet and MuJoCo. Each backend has one GUI
run and one script run; repeats remain validation (V6).

## What was built

- **Arm control on three backends.** The URDF Panda gets the shared URDF
  articulation contract (`installed_overlays.yaml`): seven arm channels, a
  master finger and a **mechanical** coupling of the second finger. The
  limits are the source URDF's; the gains are the declared lab servos also
  used for TIAGo.
- **Backend-neutral pick and place.**
  - `robot_lab_mujoco/arm_pick_place.py` plans a top-down grasp, a lift, a
    transfer and a place. It uses a private MuJoCo kinematics and collision
    model of the URDF plus the world's static boxes, with damped
    least-squares IK, chained-IK Cartesian lines and collision-checked joint
    segments.
  - `robot_lab_bringup/scripts/urdf_pick_place.py` executes the plan through
    `/molar/articulation/*` and judges it by simulator truth. It is started
    by the launch for profiles with a `pick_place` block, and acts only on
    an explicit `/pick_place/request`.
- **GUI.** The arm page has a **Pick / place (simulator truth)** section:
  object, support, lift, Run and Stop task. Coupled channels are shown and
  never commanded.
- **World.** `arm_pick_place_lab`: two 0.3 m tables and a loose 40 mm, 50 g
  cube. It is registered for the CLI and the GUI, with a rasterized 2D map.
- **Loose objects stay loose.**
  - `sdf_world.extract_dynamic_bodies` reads them from the world file.
  - The MJCF generator and the PyBullet loader build them as free bodies.
  - Both backends publish them on `/simulation/free_bodies`; MuJoCo's former
    `/mujoco/free_bodies` was renamed to this.

## Fixes needed on the way

The failed runs are kept on SSD under
`robot_lab_runtime/panda-urdf-20261117`.

**The arm was floating.** MuJoCo and Isaac floated a URDF anchored to its own
`world` link, so the arm stood loose: joints 1–3 never moved and the base
motion tripped the articulation hold.

- `robot_lab_utils/urdf_base.fixed_to_world` now keeps the weld in MuJoCo
  and sets Isaac's `fix_base`.
- MuJoCo places a welded robot at the spawn pose and takes the world as its
  base body, so a loose cube is not taken for the robot.

**PyBullet lifted the fixed arm by 0.11 m** for "ground clearance", so the
fingers closed above the cube. Fixed arms are no longer lifted.

**The Panda's all-zero pose is in self-collision** (link5 inside a finger).
The planner refuses a colliding start, as MoveIt does; the operator moves
Home first.

**Independent fingers lost the cube.** The URDF's two independent finger
joints shoved the cube aside or dropped it in transfer.

- The coupling is now mechanical: a PyBullet gear constraint (PyBullet's own
  Panda example) and a MuJoCo joint equality.
- 5/5 offset trials then held and placed the cube in a PyBullet replica.

**The node crashed writing its receipt** (numpy values in JSON) after a
successful task. Encoding is fixed, and an error in a tick now stops the
task instead of the node.

**The GUI launch was blocked by a modal dialog**: the new map was missing
from the GUI's environment registry. It is now registered.

## Measured

**Arm screen** (`scripts/screen_urdf_arm.py`, `screen_*.json`):

- All joints settle on PyBullet, MuJoCo and Isaac.
- Stop holds with 0 drift; a lost heartbeat holds with 0 drift.
- These screens ran before the finger coupling, with two independent finger
  channels.

**Pick and place** (`pickup_cube` → `place_table`, lift 0.10 m). All four runs
pass every stage check.

| Run | Backend | Seconds | Rise | Hold drift | Placed (height error, inside) |
|---|---|---|---|---|---|
| Script | PyBullet | 21.9 | 0.097 m | 0.6 mm | 0.0 mm, yes |
| Script | MuJoCo | 22.0 | 0.096 m | 1.6 mm | −0.1 mm, yes |
| **GUI** (`qualify_urdf_arm_gui.py --pick-place`) | PyBullet | 22.2 | see receipt | | yes |
| **GUI** | MuJoCo | 20.5 | see receipt | | yes |

Script runs use `scripts/arm_pick_place_check.py` (`cli_*.json`). The GUI
runs press Home, choose the object and support, and press **Run pick /
place** on the normal arm page (`gui_*_report.json`, `gui_*_receipt.json`,
`gui_pybullet_carrying.png`).

On 2026-11-22 the normal GUI selected **RGB-D camera** on MuJoCo Display
in `arm_pick_place_lab`. The stationary camera estimated `pickup_cube`
from three frames with a 1.1 mm position error. The arm lifted the cube
96.2 mm and rested it within `place_table` with a −0.1 mm height error.
The task took 26.52 s and the owned launch exited with status 0.
[GUI receipt](mujoco-gui-workcell-rgbd-2026-11-22.json).

**Stage checks:**

- **Grasp:** the fingers stop on the cube (about 0.02 m each).
- **Lift:** the cube rises at least 80 % of the lift and keeps within 1 cm of
  the tool over a one-second unsupported hold.
- **Place:** the cube rests at the support top (±1 cm), inside its footprint,
  and moves less than 5 mm in the last second.

## Limits

- One arm, one cube and top grasps of boxes only. Object poses come from
  simulator truth; there is no perception.
- Not done: Gazebo (the URDF has no ros2_control block), and Isaac pick and
  place (the mechanical finger coupling has no Isaac adapter yet).
- The other eleven arms, repeats and interruption tests remain V6
  validation.
