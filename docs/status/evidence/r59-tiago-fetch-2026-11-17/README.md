# R5.9 TIAGo fetch: drive, pick, carry, place (November 17, 2026)

**Status: implemented, experimental. R5.9 done-when met on MuJoCo; also passes on PyBullet.**

The done-when is one measured mission with object pose and stability traces,
in which base and arm commands cannot interfere. The robot is the official
PAL TIAGo (`asset_pal_tiago`) in the new `mobile_fetch_lab` world.

One request runs the whole task:

1. drive to `pickup_table`;
2. pick `fetch_block` (40 × 40 × 150 mm, 100 g);
3. back off and carry it about 2.3 m to `drop_table`;
4. place it;
5. back off.

Simulator truth judges every stage. The receipt has a 2 Hz trace of the base
pose, the object pose, the object in the base frame, and the fingers'
positions and references.

## Measured

Script runs used `scripts/arm_pick_place_check.py --probe-interlock`
(`cli_*.json`, receipts `cli_*_receipt.json`).

**Script run results** (MuJoCo and PyBullet, one run each):

| Measure | MuJoCo | PyBullet |
|---|---|---|
| Result | passed, 107.4 s | passed, 121.5 s |
| Pick: rise / hold drift | 0.099 m / 0.2 mm | 0.098 m / 0.0 mm |
| Carry: max object drift vs base | 5.7 mm | 16.6 mm |
| Place: height error, inside footprint | −0.1 mm, yes | 0.0 mm, yes |
| Place: motion in the last second | 0 | 0 |
| Base stop error at the pick standoff | 3 mm | 6 mm |

**Interlock evidence** (same runs):

| Measure | MuJoCo | PyBullet |
|---|---|---|
| Max base speed during arm phases | 0.0004 m/s | 0.0026 m/s |
| Drive ticks while the arm was active (base held at zero) | 0 of 839 | 0 of 390 |
| Probe: `/cmd_vel` 0.1 m/s for 1 s during arm motion | base moved 0.2 mm | base moved 1.0 mm |
| Probe: arm target sent during the carry | rejected, head moved 0 rad | rejected, head moved 0 rad |

### GUI runs

`scripts/qualify_urdf_arm_gui.py --pick-place fetch_block drop_table` on the
normal GUI: select TIAGo, `mobile_fetch_lab` and the backend, launch, press
**Home (direct)**, choose the block and table, and press **Run fetch**.
Files: `gui_*_report.json` and `gui_*_receipt.json`. The receipts include the
full 2 Hz trace.

| Measure | MuJoCo | PyBullet |
|---|---|---|
| Result | passed, 108.9 s | passed, 104.6 s |
| Pick: rise / hold drift | 0.099 m / 0.2 mm | 0.098 m / 0.0 mm |
| Carry: max object drift vs base | 5.5 mm | 17.1 mm |
| Place: height error, inside footprint | −0.1 mm, yes | 0.0 mm, yes |
| Max base speed during arm phases | 0.0006 m/s | 0.0020 m/s |

In each GUI run the node waited one control tick for the arm to settle
before driving, and sent a zero command on that tick.

The GUI screenshots show the RViz window over the arm page, so they are not
kept.

## Interference cannot happen

**Inside the node:** it sends no `/cmd_vel` while the articulation is active,
and no arm target while the base moves.

**In the contract** (`robot_lab_utils/urdf_articulation`, on every backend):

- an active articulation sets `base_inhibited`, and the spawner then
  zeroes `/cmd_vel`;
- base motion holds the articulation and rejects new targets.

The probes test both directions from outside the node.

## What was built

- **Executor phases** (`urdf_pick_place.py`). For a profile with
  `pick_place.mobile`:
  - drives are straight, truth-steered legs, and each leg's clearance to
    every table is checked before the request is accepted;
  - each arm phase is planned on arrival from the measured base pose;
  - while carrying, the object must stay within 3 cm of its pose in the base
    frame.
- **Planner** (`arm_pick_place.py`):
  - separate `plan_pick` and `plan_place`; the fixed-arm `plan_pick_place`
    composes them;
  - a mobile scene welds the robot at the base pose and holds the torso and
    head at their measured values;
  - contacts between robot parts the plan never moves are ignored;
  - every grasp side and direction is tried;
  - the grasp rises until the fingertips clear the support;
  - joint moves go via high poses, and then via the repo's bounded
    RRT-Connect;
  - standoff, place-standoff and route-clearance helpers.
- **Grip joints** (articulation contract):
  - a still gripper keeps a bounded squeeze across Stop, base-motion holds
    and heartbeat loss, and counts as active only while it moves;
  - TIAGo keeps up to 20 mm, its 16 N grasp force.
- **Backend contact fixes**, only for robots with `grip_joints`:
  - **MuJoCo:** elliptic friction cone with impratio 10.
  - **PyBullet:** grip force proportional to error, a symmetric-grip gear
    (`pybullet_symmetric_grip`), and friction anchors on the fingers and
    loose objects.
- **GUI:** for mobile profiles the section reads **Fetch** and the button
  **Run fetch**.
- **World:** `mobile_fetch_lab`, registered for the CLI and the GUI, with a
  rasterized map.

## Failures on the way

Logs and receipts are on SSD under `robot_lab_runtime/tiago-fetch-20261117`.

**Planning:**

- The imported TIAGo has a floating root, so the planner scene would not
  compile. The planner now welds it.
- Wheel/chassis overlaps in the source model read as a start collision.
  Contacts between parts the plan never moves are now ignored.
- Linear joint moves from Home swept the fingers through the table edge,
  for every grasp yaw.
- A 40 mm cube cannot be gripped from above: PAL's pads reach 9 cm past the
  grasp frame. This is why the world uses a tall block.
- The place standoff first faced the wrong table side.

**Carry legs:**

- The route check refused the back-off leg, which starts next to the table
  by design. That leg is now checked against the base radius only.

**MuJoCo grip:**

- At 4 N retained squeeze the block slid 2.6 mm/s down the fingers.
- At 16 N it still crept 1.7 mm/s with the pyramidal cone. The elliptic
  cone fixed it.

**PyBullet grip:**

- Saturated finger motors let the fingers drift sideways and shove the
  block 2.4 cm.
- With the gear, the block yawed 0.5 rad in the grip while the base turned,
  forcing the fingers to 27 mm. One such run placed the block, but it is not
  counted (`pybullet4`).
- With friction anchors, the close stage never settled: the reported finger
  velocities jitter. Stillness is now judged from positions.

**Run setup:** the first GUI attempts never started, because the output
directories already existed.

## Limits

- One robot, one block, top grasps of boxes. Object and base poses come from
  simulator truth; there is no perception and no localization.
- No Nav2: the legs are straight, and the request is refused if a table is
  in the way.
- Not on Isaac yet: no `/simulation/free_bodies` truth there, and no
  `grip_joints` contact handling.
- One run per backend. Repeats, resets, cancellation mid-carry, obstacles
  and other maps are V6 validation.

## Checks

- Focused tests pass: planner 7, contract 36, GUI arm page 9.
- The utils and MuJoCo suites pass: 287.
- The bringup, GUI and registry suites: 1902 passed and 4 failed. None of
  the four touches this change:
  - Robotiq native-hand contact: `native_hand_control`;
  - PX4 autofill for `mobile_manipulation_lab`;
  - `cartesian_controls` grasp-pose request;
  - terrain raster height range.
- The Panda regression after the planner changes passes: PyBullet 23.1 s,
  MuJoCo 22.3 s.
