# TIAGo fetch with the grasp planned from its head camera (November 18, 2026)

**Status: implemented, experimental.** R5.9's "perception" item: the
fetch's object pose for the grasp comes from TIAGo's head RGB-D camera, not
from simulator truth.

## What was added

**`"pose_source": "rgbd"`** in a `/pick_place/request`
(`urdf_pick_place.py`; GUI: Pick / place → **Object pose → RGB-D camera**;
script: `arm_pick_place_check.py --pose-source rgbd`):

1. **Drive** to the pick standoff, as before. Truth still chooses the table
   and the standoff: the "go to that table" hint.
2. **Look** (new phase):
   - the head goes to the profile's `pick_place.look` pose (TIAGo:
     `head_2_joint` −0.85 rad, tilted down);
   - 0.5 s after it settles, each new RGB-D frame is fitted by the existing
     `robot_lab_utils.rgbd_boxes.estimate_box`:
     - a colour and size declared for the world (`rgbd_boxes.yaml` next to
       its `worlds/` folder);
     - metric depth and the camera calibration;
     - the camera pose: TF from the base frame, with the base pose from
       `/odom/ground_truth`, as the arm plan uses;
   - three consecutive estimates within 1 cm of their median make the
     object pose.
3. **Pick:** the grasp is planned from that pose.
4. **Place:** planned from the object's grasp-time offset to the tool,
   carried by the measured arm, not from truth.
5. **Judging:** simulator truth judges every stage, as before. The receipt
   records the estimate and its error against truth (`perceived`).

**Fetch-world bindings:** `mobile_fetch_lab/rgbd_boxes.yaml` declares
`fetch_block`:

- 0.04 × 0.04 × 0.15 m;
- OpenCV HSV 0–10 / 150–255 / 40–255.

A test checks that the world's block colour falls inside that range and
that no other model's colour does.

## Measured

Command: `run_with_launch.py --launch "robot_model:=asset_pal_tiago
simulator:=<sim> mode:=display map_name:=mobile_fetch_lab
allow_experimental_assets:=true spawn_x:=-1.0 spawn_y:=-1.0"` with
`arm_pick_place_check.py --robot asset_pal_tiago --object fetch_block
--support drop_table --pose-source rgbd` (straight transit legs).

Runs were measured while a 512-environment legged-training run used the
CPU.

| Run | Result | Total | Look | Frames | Estimate error (x, y, z) | Spread | Declared bound | Carry drift | Place (height error, xy from plan) |
|---|---|---|---|---|---|---|---|---|---|
| MuJoCo 1 | passed | 167.7 s | 3.9 s | 3 of 3 | 1.5 mm (−1.5, 0.1, −0.1) | 0.3 mm | 10.1 mm | 5.5 mm | −0.1 mm, 2.9 mm |
| PyBullet 1, 2 | failed: no estimate in 30 s (64 frames) | — | — | 0 | — | — | — | — | — |
| PyBullet 3 (after the mirror fix below) | passed | 142.5 s | 6.0 s | 3 of 3 | 5.1 mm (−4.0, −0.4, 3.2) | 0.2 mm | 9.9 mm | 16.6 mm | 0.0 mm, 6.1 mm |

**Other numbers from MuJoCo 1:**

- grasp: fingers stopped at 19.3 / 18.9 mm;
- lift: 99.2 mm, hold drift 0.2 mm;
- the base moved at most 0.8 mm/s during arm phases, with no drive tick
  while the arm was active.

## Fixed on the way: PyBullet's camera did not see the block

All 64 PyBullet frames were rejected ("need exactly one visible object of
the configured color"). The executor now saves the last frame of a failed
look next to its receipt; `pybullet-2-look-frame-before-fix.png` is all
grey. PyBullet renders its camera from a mirror client, which had two
faults:

- it drew the loose block as a **static grey shape frozen at its start
  pose**, not the moving red free body;
- it drew every static shape grey, ignoring the SDF material colour.

Now the mirror loads the world's visual geometry with its SDF colours (what
a camera sees). Loose objects are visual-only copies, posed from the
physics state at each frame's snapshot. The physics client is unchanged.

Receipts: `fetch-*.json` (with 2 Hz traces). Launch logs are on SSD under
`robot_lab_runtime/r59-rgbd-20261118`.

## Limits

- One object, one colour binding, one world. The camera fits known-size
  upright boxes of a declared colour, with no recognition of anything else.
- The base pose is still simulator truth (as the arm plan has always
  used), and truth still picks the table to drive to.
- The look pose is fixed per profile, and the head must see the whole top
  face from the standoff.
- PyBullet's estimate is less accurate (5.1 mm, against 1.5 mm on
  MuJoCo); its tiny software renderer is the likely cause, not measured.
- Isaac is not run: its TIAGo grasp does not close yet (see
  `r59-isaac-tiago-approach-2026-11-18`).
- Repeats are V6.
