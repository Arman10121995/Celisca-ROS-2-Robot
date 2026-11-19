# R5.8: DEX-EE holds a lifted box with a synthesized tripod (November 19, 2026)

**Status: implemented, experimental.** The dexterous DEX-EE passes the
normal-GUI hand workflow on MuJoCo (2 of 2 runs): grasp, lift 5 cm on the
support stage, 3 s hold, return and supported release. With it, every hand
family has a measured stable grasp, lift, hold and release of a documented
object on one backend (R5.8's done-when). The host was also training a Go2
policy.

## What changed

- **`robot_lab_utils/hand_grasp_synthesis.py`** chooses the contacts for the
  object first, then the joints that reach them. On November 17 the Curl
  template's contacts lay along a tilted line and the box rolled out.
  - **Contacts:** the thumb (F0) presses the centre of the face toward it.
    F1 and F2 press the opposite face at ±0.6 of its half width. All three
    are in the horizontal plane through the box centre.
  - **Joints:** per finger, bounded least squares on the executed model's
    kinematics puts the tip pad's contact patch on its point, facing the
    face. F1 and F2 reach within 0.13 mm but face it 22° off (their J0 range
    cannot square them up). Their rounded tips still press along the face
    normal, so up to 35° is accepted.
  - **Placement:** a grid over the box centre in the hand frame keeps the
    reachable fit with the best Ferrari–Canny force-closure ε (friction 0.5,
    below the source's 1.0). For the 4 cm box the centre is (0, 25, 315) mm
    in `hand_base`, with ε 0.132.
  - **Targets:** the grasp aims 4 mm inside each face, so the authored PID
    servos press with bounded effort. The approach pose is 15 mm outside
    each face.
- **Display plant** (`native_asset_display`):
  - For DEX-EE the free-object fixture uses the synthesized box (4 cm,
    50 g, oriented to `hand_base`) instead of the pad-gap placement, which
    has no opening at the source's zero pose.
  - A lab `home` keyframe starts the hand at the approach pose, because at
    zero a fingertip would sit inside the box.
- **Hand contract:**
  - With a plan, DEX-EE lists **Synthesized tripod** first.
  - **Open** returns to the approach pose, so opening never sweeps the
    fingers into the planned box.
  - The plan is part of the contract hash.
- **GUI pose selector:** a new Run's contract selects its first pose. It
  had stayed on **Curl** because Curl is still listed, so the GUI grasped
  with the template (`failed_dexee_1`).
- **Support-stage return fixes (all fixed-mount hands):**
  - The seating step set the stage straight to its final target. It
    jumped 12 mm in 0.2 s and struck the held box at more than 2 N. The
    stage now ramps at 2 cm/s, as `lift_object` already intended.
  - A firm tripod's position servos slowly take the whole weight and lift
    the returned box microns off the stage, about 4 s after it is seated.
    The stage now resumes its slow seating inside the same 3 mm budget
    (after a 0.1 s debounce), instead of reporting a lost support.
- **Training tab:**
  - While any trainer runs, the Runs list rescans every run every 10 s. It
    parsed all 24 progress files (26 MB of JSON) and stalled the GUI about
    1 s. That tripped the 0.8 s hand heartbeat mid-hold
    (`failed_dexee_2`, `failed_dexee_3`).
  - Summaries now read only the last 16 KB, and both readers cache by size
    and mtime. A scan of all runs takes 16 ms (4 ms cached).

## Measured (normal GUI, MuJoCo, `nav_empty`, `qualify_native_hand_gui.py`, contact stop 2 N, effort 0.8)

| Run | Grasp L / R (thumb / F1+F2) | Lift | 3 s hold | Max angular slip | Return | Release |
|---|---|---|---|---|---|---|
| `gui_dexee_1` | 4.13 / 4.69 N | 49.3 mm | steady at 49.3 mm, L/R 5.2 / 5.9 N | 0.032 rad | supported, seated 1.0 mm | supported release |
| `gui_dexee_2` | 3.97 / 4.51 N | 49.3 mm | steady at 49.3 mm | 0.032 rad | supported, seated 1.0 mm | supported release |
| `gui_robotiq_2f85_regression` | 11.79 / 11.79 N | 49.9 mm | steady | 0.000 rad | supported | supported release |

The Robotiq run checks that the stage-return changes kept the November 17
result (11.9 N/side, 49.9 mm).

## Retained failures

| Report | What happened |
|---|---|
| `failed_dexee_1` | GUI grasped with **Curl** (stale pose selector); the box slid and rotated 0.08 rad, and the lift was interrupted as slipping |
| `failed_dexee_2`, `failed_dexee_3` | Tripod held 49.3 mm for 2.3 s, then a 1.0–1.3 s GUI stall (Training-tab rescan) missed the 0.8 s heartbeat and the hand released, as designed |

Before the GUI runs, the in-process test also caught both stage-return
faults: the 2 N press, and a lost support about 4 s after seating.

## Checks

- `test_hand_grasp_synthesis.py` (new, 5 tests) on the installed source
  hand and the real controller:
  - force-closure ε separates a tripod from one-sided contact;
  - the plan is reachable;
  - the contract opens to the approach pose and refuses out-of-range
    plans;
  - the home pose starts clear of the box;
  - the grasp lifts, holds, returns and releases.
- 40 existing hand tests and the training-tab tests pass, plus a new
  progress-tail test.

## Limits

- One box size per run (4 cm; 3–5 cm also held in an offline physics
  sweep), MuJoCo only, a fixed hand over a lab support stage. There is no
  arm transport and no other object shapes or backends.
- Synthesis is a model-based plan on the executed kinematics. Its quality
  number is an estimate; the hold is judged only by measured contacts and
  motion.
- The approach `home` and the synthesized pose are lab choices, not vendor
  poses.

## Reproduce

```bash
xvfb-run -a -s "-screen 0 1600x1000x24" python3 scripts/qualify_native_hand_gui.py \
  --robot menagerie_shadow_dexee --output /workspace/molar/robot_lab_runtime/hands/dexee
```

GUI logs are on SSD under `robot_lab_runtime/hands-20261119`, with hashes
in `runtime-sha256.txt`.
