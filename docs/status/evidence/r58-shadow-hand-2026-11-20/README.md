# R5.8: Shadow Hand grasps, lifts and releases a box with its coupled tendons (November 20, 2026)

**Status: implemented, experimental.** The Shadow Hand E3M5 right hand from
MuJoCo Menagerie (Apache-2.0, pinned `4d038b3`) is a new native hand. Each
of its fingers has one tendon servo that drives the J1 and J2 joints
together. In the normal Robot Lab GUI on MuJoCo it passes grasp, 5 cm lift,
3 s hold, return and supported release, 2 of 2 runs. This was the
roadmap's remaining "tendon hand adapter".

## What changed

- **Installed source:** Menagerie entry `shadow_hand` (`right_hand.xml`) is
  installed as `menagerie_shadow_hand`, with its fixed mount at z 0.15 m.
- **Hand contract** (`native_hand.py`):
  - It binds the twenty authored position servos to the executed plant: the
    wrist and thumb joints, and per finger J4, J3 and the J0 tendon. Each
    J0 tendon must couple J1 + J2 with unit coefficients. Source effort
    clamps are kept (±1 N·m on the fingers).
  - Groups are Wrist, Thumb, First, Middle, Ring and Little.
  - Poses are Open (the authored zero), Curl, and **Synthesized tripod**
    with the object fixture.
- **Tripod synthesis for an underactuated finger** (`hand_grasp_synthesis.py`):
  - Per-finger layouts let one channel drive several joints.
  - A tendon gives J2 and J1 the same torque. The finger is therefore at
    rest only when the tip force has no turning effect between them, which
    means it lies along the middle phalanx.
  - An exact normal press would need the face normal along that phalanx,
    and that was unreachable (4 mm off). Instead the IK keeps the tangential
    share the finger needs within 0.6 of the normal force. The source
    fingertip friction is 1.0, so this stays inside the friction cone. The
    press uses that tilted force.
  - The compliant servos (kp 1 and 0.5) sagged about 5 mm under the box. So
    each contact's press also carries a third of the declared 50 g weight,
    balanced the same way.
  - The plan carries a joint-space approach pose, so the home key sets
    J1/J2 and not just the tendon sum.
- **Object fixture:** the Shadow object scene adds MuJoCo's no-slip pass, as
  for Allegro. Without it the box crept past the 0.15 rad slip limit.
  With it, the box settles at about 0.09 rad.

DEX-EE and Allegro give the same plans as before (their GUI regressions
pass).

## Measured (normal GUI, MuJoCo, `nav_empty`, `qualify_native_hand_gui.py`, contact stop 2 N, effort 0.8)

| Run | Grasp thumb / first+middle | Lift (measured) | Max angular slip in hold | Return | Release |
|---|---|---|---|---|---|
| `gui_shadow_1` | 1.67 / 1.03 N | 45.5 mm | 0.092 rad | supported, 0.9 mm seating | supported, 0 N |
| `gui_shadow_2` | 1.87 / 1.53 N | 46.3 mm | 0.067 rad | supported, 0.9 mm seating | supported, 0 N |
| `gui_allegro_regression` | 2.11 / 2.58 N | 48.5 mm | 0.065 rad | supported | supported |

A full GUI cycle takes about 31 s. The pass conditions are those of the
DEX-EE and Allegro receipts. The Shadow lift is about 4 mm short of 5 cm,
inside the 5 mm settle band, because its servos still sag. It also tilts the
most of the three dexterous hands.

### Failures on the way (in-process, `test_shadow_hand.py` harness)

1. An exact normal press with balanced tendon joints: no placement reached
   within 1.5 mm (best 4.1 mm for the first finger).
2. Balance within a 0.4 force ratio: still no placement (most fits needed
   0.5–0.53).
3. No-slip pass off: the box rotated past 0.15 rad during the lift, and
   transport was interrupted.
4. No weight share: the box sagged 5 mm once unsupported, and the lift never
   settled within the 5 mm band.

## Limits

- One box (4 cm, 50 g), on MuJoCo only, in a support-stage fixture. The
  ring and little fingers and the wrist stay at zero.
- The weight share uses the fixture's declared mass. An unknown object
  would need a measured or estimated mass.
- A MOLAR RMF building graph is Codex's lane (see the ledger); it is not
  part of this work.

## Reproduce

```bash
python3 scripts/provision_extension_assets.py --kind robots --source mujoco_menagerie --entry shadow_hand
xvfb-run -a -s "-screen 0 1600x1000x24" python3 scripts/qualify_native_hand_gui.py \
  --robot menagerie_shadow_hand --output /workspace/molar/robot_lab_runtime/hands/shadow
```

Logs are on SSD under `robot_lab_runtime/hands/shadow-{1,2}` (hashes in
`runtime-sha256.txt`).
