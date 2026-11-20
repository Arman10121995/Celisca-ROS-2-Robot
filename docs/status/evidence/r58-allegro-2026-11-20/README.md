# R5.8: Wonik Allegro hand grasps, lifts and releases a box (November 20, 2026)

**Status: implemented, experimental.** The Wonik Allegro right hand from
MuJoCo Menagerie (BSD-2-Clause, pinned `4d038b3`) is a new native hand. In
the normal Robot Lab GUI on MuJoCo it passes grasp, 5 cm lift, 3 s hold,
return and supported release, 2 of 2 runs. It is the fifth hand family with
a measured object cycle, and the second dexterous one after DEX-EE.

## What changed

- **Installed source** (`docs/status/asset-sources-2026-10-05.yaml`,
  `provision_extension_assets.py`): Menagerie entry `wonik_allegro`. Its
  `right_hand.xml` is installed as `menagerie_wonik_allegro`, because the
  provisioner now prefers `right_hand.xml` in two-handed sources. The fixed
  mount is raised to z 0.15 m, since the authored palm sits at the root
  origin and would otherwise touch the ground.
- **Hand contract** (`native_hand.py`):
  - It binds the sixteen authored joint position servos (kp 1 N·m/rad, no
    source effort clamp) to the executed plant. No new transmission is added.
  - Finger groups are Thumb, Index, Middle and Ring.
  - Poses are Open (the authored zero, with the thumb rotation at its
    0.263 rad lower bound) and Curl. With the object fixture, **Synthesized
    tripod** is added and listed first.
  - Unclamped motors use the controller's existing lab effort threshold.
- **Tripod synthesis** (`hand_grasp_synthesis.py`), generalized to reviewed
  hands (`HANDS`):
  - Allegro is authored palm-up with its fingers along world +x. The thumb
    crosses the palm and meets index + middle along the finger axis, with
    the box held above the palm.
  - Its fingertips are capsules, so the contact is the capsule point
    furthest along the face's inward normal.
  - Placements where another collision geom of the hand would enter the box
    are rejected (sampled box and capsule points, 2 mm margin).
  - **Press along the face normal:** approach, contact and press targets
    lie on one joint line, d = K⁻¹Jᵀn. The tip meets the face at the
    planned point, and at the press target the servo torques push it along
    the normal with 2 N.
  - The ring finger stays at its reference pose.
  - DEX-EE keeps its geometric squeeze and gives the same plan as before.
- **Object fixture** (`native_asset_display.py`):
  - When the hand frame is not z-up (Allegro), the cube gets a yaw-only
    orientation, so its support stage moves vertically.
  - The fixture now refuses any orientation that tilts the stage axis.
  - The Allegro scene adds MuJoCo's no-slip friction pass (10 iterations).
    The Robotiq, UMI and DEX-EE scenes are unchanged.

## Measured (normal GUI, MuJoCo, `nav_empty`, `qualify_native_hand_gui.py`, contact stop 2 N, effort 0.8)

| Run | Grasp thumb / index+middle | Lift (measured) | Max angular slip in hold | Return | Release |
|---|---|---|---|---|---|
| `gui_allegro_1` | 2.14 / 2.61 N | 48.7 mm | 0.056 rad | supported, 0.8 mm seating | supported, 0 N |
| `gui_allegro_2` | 2.11 / 2.58 N | 48.5 mm | 0.065 rad | supported | supported, 0 N |
| `gui_dexee_regression` (DEX-EE) | 3.97 / 4.51 N | 49.3 mm | 0.022 rad | supported | supported |

The whole GUI cycle takes about 25 s. Pass conditions are those of the
DEX-EE receipt: a measured two-sided hold, an unsupported lift within 1 cm
of 5 cm, slip under 1 cm and 0.15 rad, a measured supported return, and no
finger contact at release.

### Failures on the way (in-process, `test_allegro_hand.py` harness)

1. With DEX-EE's 4 mm geometric squeeze, kp 1 servos pressed about 0.7 N.
   The box sank onto the stage, and the guard stopped the lift for a new
   external contact.
2. The box took the palm frame's orientation, so the support stage slid
   sideways (world x) instead of down.
3. A 15 mm geometric squeeze dragged each tip along its curl arc. The box
   twisted 8.6° once unsupported (slip limit 0.15 rad).
4. Normal-force targets with separately solved approach poses touched
   millimetres off the plan. The thumb pressed 3 mm below the fingers, and
   that 0.006 N·m couple pitched the box about 5°. Adding a weight share to
   each contact did not help.
5. With one joint line the contacts land within about 1 mm of the
   mid-plane. The box still sank 7 mm over the hold, at 1.2 mm/s. This was
   MuJoCo soft-friction creep at only about 7 % of the friction limit:
   `impratio` 100 or the no-slip pass each stopped it. The no-slip pass was
   kept, because it leaves contact stiffness unchanged.

## Limits

- One box (4 cm, 50 g), on MuJoCo only, in a lab support-stage fixture: the
  hand itself does not move. There is no arm transport.
- The no-slip pass is a solver setting of the Allegro object scene.
- The Shadow Hand (coupled tendons, J1 + J2 under one servo) is not added.
  Its underactuated split needs a different IK.

## Reproduce

```bash
python3 scripts/provision_extension_assets.py --kind robots --source mujoco_menagerie --entry wonik_allegro
xvfb-run -a -s "-screen 0 1600x1000x24" python3 scripts/qualify_native_hand_gui.py \
  --robot menagerie_wonik_allegro --output /workspace/molar/robot_lab_runtime/hands/allegro
```

Logs are on SSD under `robot_lab_runtime/hands/allegro-{1,2}` (hashes in
`runtime-sha256.txt`).
