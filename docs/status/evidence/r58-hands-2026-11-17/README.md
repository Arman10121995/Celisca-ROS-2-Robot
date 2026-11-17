# R5.8 native hands: GUI grasp, lift, hold, return and release (November 17, 2026)

**Status: implemented, experimental.**

R5.8's done-when is a measured stable grasp, lift, hold and release of a
documented object for each hand family on at least one backend. It is met
for three hand families on MuJoCo. The dexterous DEX-EE does not pass yet.

## Measured (normal GUI, MuJoCo, `nav_empty`, one run each)

`scripts/qualify_native_hand_gui.py` drives the real Robot Lab window:

1. select the hand;
2. tick **Add free grasp object on next Run**, then Run;
3. on the Hand page, press **Grasp object**, then **Lift and hold object**
   (0.05 m);
4. hold 3 s;
5. press **Lower to start**, then **Release object**.

The hand controller's measured states judge each stage. Reports with a 4 Hz
trace are in `gui_*_report.json`.

The object is the fixture's free 30 mm, 50 g cube.

| Hand family (robot) | Lift | Grasp force L / R | Height during 3 s hold | Slip / rotation in hold |
|---|---|---|---|---|
| Adaptive linkage (Robotiq 2F-85) | support stage | 11.9 / 11.9 N | 49.88–49.92 mm | 0.1 mm / 0 rad |
| Adaptive linkage (Robotiq 2F-85 v4) | support stage | 13.0 / 13.0 N | 49.94–49.96 mm | 0 / 0 |
| Soft parallel gripper (UMI) | source vertical motor | 0.71 / 0.71 N | 49.00 mm | 0 / 0.0001 rad |

**Every run reached the end states:**

- the return reached `supported`, with seating of 0.1 mm (Robotiq) and
  1.5 mm (UMI);
- the release reached `supported_release`: no finger contact, supported and
  settled for one simulated second.

**Other hand families measured elsewhere:**

- The parallel-jaw family is covered by the native Panda gripper (78.1 mm
  physical lift), the URDF Panda (R5.7) and TIAGo's PAL gripper (R5.9 carry).
- This is UMI's first normal-GUI run. Its earlier lift was a pinned
  component check.

## What changed

- **The pedestal now touches only the object** (collision bit 2). It stands
  between the fingers. The Robotiq linkage chattered against it, so the hand
  never settled and the grasp closed at about 0.4 of 255. This also broke
  the existing `test_real_robotiq_grasp_…` test.
- **Lab support stage** (`add_hand_object_fixture`):
  - added for fixed-mount hands without a vertical source axis (Robotiq,
    DEX-EE);
  - the pedestal is a gravity-compensated slide with a bounded position
    motor (20 kN/m, ±200 N, −0.20 to +0.02 m), moving at no more than
    2 cm/s;
  - **Lift and hold object** drops it, so the object is lifted relative to
    its support and hangs in the fingers; the hand model is unchanged;
  - heights are measured relative to the support;
  - on return the stage stops at first measured support and may press at
    most 2 N.
- **Grasp state debounce:**
  - two-sided contact must last 0.3 simulated seconds, with the object
    still, before a grasp counts as holding;
  - a loss must last 0.1 s before it counts.

  DEX-EE's first touch bounced and was latched as `contact_lost`.
- **Torsional friction on the fixture object** (`condim` 4).
- **GUI:** the help text says which lift a hand uses; the state reports
  `object_lift_mode`.

## Not passed: DEX-EE (dexterous)

The fixture needs an open pre-grasp pose (J1 = −0.8 rad on all three
fingers, a 9.8 cm gap); the zero pose has no opening.

**What works:** Curl and Pinch both close with two-sided contact (2–7 N) and
reach `holding`.

**What fails:** as the support starts to drop, the cube tilts in the
fingertips and stays on the pedestal. The grasp is judged slipping after
0.15 rad, within 4–8 mm of drop. This held across:

- grip thresholds of 1–5 N;
- `condim` 4 and 6, and higher torsional and rolling friction;
- taller boxes.

The fingertips meet the cube at points along a tilted line; the source
already uses soft fingertip contacts, the elliptic cone and noslip. This
needs grasp synthesis (chosen contact points and an object placed for them),
not fixture tuning. It stays open under R5.8.

Runs are on SSD under `robot_lab_runtime/hands-20261117`. One GUI attempt
failed in the script itself, which read the transport state left over from
the grasp request; it is kept as `robotiq_2f85_failed_stale_state`.

## Checks

43 hand tests pass. Three changed:

- support-force: loss is now debounced;
- the fixed-hand refusal now applies without the stage;
- new: the stage lift moves no source target.

## Limits

- One run per hand, MuJoCo only, one object.
- The support stage tests holding against gravity with a static hand; it is
  not arm transport.
- Hands on other backends, DEX-EE grasp synthesis, raw torque/tendon hands
  and repeats remain (V6).
