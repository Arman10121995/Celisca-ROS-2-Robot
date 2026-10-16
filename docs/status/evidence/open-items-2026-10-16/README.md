# Open items, 2026-10-16

On October 16 the open items were worked through one by one. Each section
gives the cause found, the change made and what was measured. Runs went
one simulation at a time; the reports are in the subfolders.

**Provenance.** Another agent (ledger owner `codex-root`, task R11
molar_hri) edited the same packages at the same time and rebuilt the
copy-installed Isaac, MuJoCo and PyBullet packages from its uncommitted
work.
- Its unguarded `"{}"` articulation config broke every simulator launch
  for a while. Its missing `import time` stopped every Isaac runtime
  five seconds after start, which failed all Isaac runs of round 47.
- Both were fixed (`7647d0a`, and a launch line later committed with its
  work). The runs kept here are from after the fixes.
- Their installs still held the other agent's gated-off articulation
  code. Report revisions name the HEAD they ran at.

## Cars settle with retries (R5.5)

**Cause.** With the 0.1 m goal circle, a car could stall 0.1-0.2 m out.
Settling would have needed a multi-point manoeuvre, and the car tree gave
it one retry. MuJoCo anti-Ackermann seed 2 aborted 0.153 m short.

**Change** (`51c7543`). The car tree backs up and retries three times.

**Measured** ([cars-settling/](cars-settling/), round 47, `38486f4`).
The three weakest cells of round 45 all pass:

| Cell | Goal A from truth | Goal B from truth | B time |
|---|---|---|---|
| MuJoCo anti-Ackermann, seed 2 | 0.035 m | 0.134 m | 31 s |
| PyBullet anti-Ackermann, seed 2 | 0.017 m | 0.139 m | 29 s |
| MuJoCo rear-steer, seed 1 | 0.034 m | 0.062 m | 25 s |

## G1 goal tolerance (P7)

**Cause.** With a 0.2 m goal circle the G1 stopped at its edge. Leg
odometry and AMCL added 0.1-0.15 m, and Isaac seed 3 ended goal B
0.256 m from truth, failing the matrix's 0.25 m screen.

**Change** (`51c7543`). xy goal tolerance 0.12 m.

**Measured** ([g1-goal/](g1-goal/), [legged-isaac/](legged-isaac/)):

- MuJoCo (round 47): 3 of 3 pass, both goals 0.035-0.171 m from truth.
- Isaac (round 47b): 3 of 3 pass, 0.015-0.141 m from truth, against
  0.081-0.256 m before.

## MPPI arrival (R7.7)

**Cause.** MPPI came within 0.05-0.19 m of 8 of 15 Bumperbot goals but
never settled into the 0.07 m / 0.03 rad window. Stronger goal critics
made it worse (6 of 15).

**Change** (`99e2e78`). With MPPI on a differential base, the
controller server also hosts an `Arrival` controller (Regulated Pure
Pursuit, the robot's own speeds). A new tree finishes a failed MPPI
approach with it on the same path. `arrival_controller:=false` turns it
off.

**Measured** ([mppi-arrival/](mppi-arrival/), round 47, the R7.7
protocol: Gazebo, nav_maze, seeds 11-53, 15 goals):

- **15 of 15 goals succeed**, against 7 of 15 before. They end
  0.028-0.148 m from truth (median 0.07 m); median time 33 s, longest
  136 s.
- 9 follow-path attempts ended at the progress checker
  ([counts](mppi-arrival/progress_failures.txt)). Three goals also
  needed the normal spin, wait and back-up.
- This measures MPPI plus the assist, not MPPI alone.

## MuJoCo/PyBullet floor lift (R7.2)

**Cause.** `scripts/depth_render_check.py` renders the lab's OAK-D model
exactly 0.2 m above a floor through each backend's own camera path. It
reproduces the recorded lift to the millimetre (MuJoCo +1.1/+2.3/+4.1/
+5.8 mm, PyBullet +1.5/+3.0/+5.4/+7.7 mm at 0.5-4 m), while wall depths
are exact. So neither the robot pose nor the depth values cause it:

- **MuJoCo** reads depth from one sample of its 4x multisampled buffer,
  3/8 px from the pixel centre. Fitted principal-point row: 119.125; with
  `offsamples = 0` it is 119.500.
- **PyBullet's TinyRenderer** puts the optical axis at (w/2, h/2 - 1),
  (160, 119) for 320 x 240 (fitted rows and columns, and a centred pole
  and bar).

**Change** (`7332e8a`):
- MuJoCo renders depth with a single-sample renderer
  (`robot_lab_mujoco.camera_render`); colour keeps its anti-aliasing.
- PyBullet's camera info gives (160, 119) (`camera_model.TINY_RENDERER`).

**Measured.** The controlled floor now reads 0.00-0.01 mm at every
range in both. A test pins MuJoCo's single-sample row at 119.5.

## MuJoCo small_house: geometry and rendering (R7.2)

**Cause of the geometry mismatch.** The shared mesh cache held 42 of 175
world surfaces in Collada centimetres. They had been staged before the
`<unit>` scaling applied, and the unchanged cache key kept serving them.
MuJoCo's small_house floor spanned 1926 m instead of 19 m, so the robot
got stuck and its map matched nothing.

**Changes** (`38486f4`, `d994300`):
- The converter version is now 4.
- MuJoCo, PyBullet and Isaac draw world models from their textured
  Collada visuals (`robot_lab_utils.textured_mesh`: one OBJ + MTL + PNG
  part per material). The models' collision geometry stays for contacts
  and rays but is not drawn.
- A headlight lights what a MuJoCo camera sees.

**Measured**
([renders](mujoco-world/small_house_plain_vs_textured.png):
untextured left, textured right):

- small_house shows its wood floor, furniture and pictures as in Gazebo.
- nav_obstacle's mean grey goes from 21 to 48
  ([before/after](mujoco-world/nav_obstacle_headlight_before_after.png)).
- `scripts/r7_2_image_quality.py` on still-then-turning small_house
  captures (round 47) gives these scores:

  | | Edge correlation with depth | Grey std | Temporal noise | ORB repeat |
  |---|---|---|---|---|
  | Gazebo | 0.458 | 53.6 | 0.0 | 1.00 |
  | MuJoCo, textured | 0.470 | 59.9 | 0.03 | 0.995 |
  | MuJoCo, before (R7.2 round 25) | | 18 | | |

## B2 in Isaac (P7)

**Cause.** With implicit PhysX drives B2 overshot every forward command.
Its policy's gains and limits (kp 160, kd 5, 200/320 Nm, 23/14 rad/s)
match Isaac Lab's DC motor actuator, whose torque in the direction of
motion falls to zero at the joint velocity limit.

**Measured** ([b2-isaac/](b2-isaac/), rounds 47b and 47c; truth speeds
for a 0.4 m/s command):

| Variant | Straight | Left arc | Right arc | Spin | Stop (vx) |
|---|---|---|---|---|---|
| Implicit drives (before) | 0.679 | 0.491 | 0.599 | 1.052 | 0.178 |
| + armature 0.01 | 0.684 | 0.472 | 0.574 | 1.070 | 0.166 |
| + friction-1 floor | 0.648 | 0.430 | 0.505 | 1.046 | 0.090 |
| + both | 0.654 | 0.444 | 0.537 | 1.027 | 0.072 |
| DC motor | 0.082 | -0.015 | -0.081 | 0.976 | 0.000 |
| **DC motor + friction-1 floor** | **0.292** | 0.263 | 0.218 | 0.885 | **-0.002** |
| MuJoCo (2026-10-12, passing) | 0.258 | 0.243 | 0.262 | 0.891 | 0.000 |

**Change** (`4313ea5`, `26b961e`). `isaac_pd_actuator: dc_motor` and a
friction-1, no-restitution floor for unitree_b2 (robots.yaml). Isaac
Lab trains on that floor. On PhysX's default floor the DC motor alone
barely walks.

## Bumperbot slam in Gazebo (P7)

**Findings** (`scripts/scan_geometry_check.py` on recorded drives):
- The scans contain no self-hits.
- They sit -0.3 deg from the map in both nav_obstacle and nav_maze.
- Bumperbot's odometry TF trails truth by 20 ms, against 5 ms for the
  mecanum car and 0 ms for the 4WS car, which slam maps to 1-2.5 cm.

**Test** ([slam-gazebo-bumperbot/](slam-gazebo-bumperbot/), round
47b). Removing the EKF does not help: slam ends 10.5 cm RMSE with the
EKF and 10.2 cm with wheel odometry. The 20 ms lag is the same in both.
Its source is the diff-drive controller's twist, averaged over 10
updates (0.1 s).

**Change** (`5a995e3`). `velocity_rolling_window_size: 1` for the
Bumperbot/Labbot diff-drive controller. Result pending (round 49).

## Pending

The next rounds cover:
- the Isaac camera render variants;
- B2 in Isaac with its new defaults (drive and nav_obstacle seeds 1-3);
- Go2 seeds 1-3 in Isaac, MuJoCo and Gazebo after its wider inflation;
- the Bumperbot slam drive with the one-sample window;
- calibration in all four simulators;
- the car course;
- car publisher loss in Gazebo and Isaac;
- the PyBullet and Isaac textured small_house captures and drives;
- the clean host with Gazebo and Isaac.
