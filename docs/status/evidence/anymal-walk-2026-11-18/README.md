# robot_assets ANYmal walk training (R3.6/R12), November 18, 2026

**Status: implemented, experimental. r2 walks** in a normal MuJoCo launch (two drive screens); r1 is kept as not passed.

## Template

`robot_lab_adapter/legged_templates.py` takes the configuration from
legged_gym's ANYmal C:

- standing pose;
- PD 80 / 2 on every joint;
- action scale 0.5;
- 0.005 s physics, decimation 4 (50 Hz).

The URDF's joint names match legged_gym's exactly. A smoke policy trains,
exports bound to `asset_anymal`, and runs in a normal MuJoCo Loc launch
(`legged_policy:=<dir>`, experimental mode declared in
`installed_overlays.yaml`).

## r1 (1500 iterations, 18.4 M steps, the trainer's walk weights): not passed

**Training:**

- The robot stands but does not walk.
- Tracking terms stay flat (lin 0.008, ang 0.003 per step).
- Action noise grows from 1.0 to 1.9.

**Evaluation** (`r1-evaluate_walk.json`): about 0.02 m/s for every command
(0.5 forward, 0.8 rad/s spin, −0.3 back, 0.3 sideways); no falls.

**Cause:** the trainer's walk reward weights are sized for Go2. The torque
penalty (−2e-4 × Σ τ²) averaged −0.16 per step for this 30 kg robot,
against −0.001 for Go2, and drowned the tracking reward.

## Change for r2

A template now also carries its source configuration's reward weights for
the walk task. ANYmal uses legged_gym's `AnymalCFlatCfg` weights:

- orientation −5;
- torques −2.5e-5;
- feet air time 2.

## r2 (2000 iterations, 24.6 M steps, legged_gym ANYmal weights): walks

**Training** (`r2-curve.json`):

- Mean return rose from about 0 to 22.5 after iteration 500.
- Forward tracking rose from 0.008 to 0.0185 per step.
- Action noise fell from 1.0 to 0.26.
- `policy.onnx` sha256 `e373abf2…6898`, on SSD under
  `robot_lab_runtime/training/anymal_walk_r2/policy`.

**Training-environment evaluation** (`r2-evaluate_walk.json`): 0.5 m/s →
0.36, spin 0.8 → 0.77 rad/s, −0.3 → −0.24, sideways 0.3 → 0.07, standing
command → still; no falls.

**Drive screens in a normal launch:**

- `MODE=loc scripts/sim_drive_check.sh mujoco asset_anymal nav_empty …
  allow_experimental_assets:=true legged_policy:=<r2>` with `--strafe`;
- measured truth velocities; both runs pass (exit 0).

| Phase (command) | MuJoCo run 1 | MuJoCo run 2 |
|---|---|---|
| straight 0.4 m/s | 0.287 m/s | 0.237 m/s |
| left arc 0.4 m/s, 0.5 rad/s | 0.315, 0.48 rad/s | 0.316, 0.47 rad/s |
| right arc 0.4 m/s, −0.5 rad/s | 0.198, −0.48 | 0.192, −0.48 |
| reverse −0.3 m/s, 0.3 rad/s | −0.242, 0.30 | −0.226, 0.29 |
| tight left 0.3 m/s, 1.5 rad/s | 0.242, 0.96 | 0.224, 0.98 |
| spin 1.0 rad/s | 0.975 rad/s | 0.974 rad/s |
| strafe 0.3 m/s | 0.060 m/s | 0.063 m/s |
| stop | stopped | stopped |

Strafe is weak (about a fifth of the command), and forward speed is
60–70 % of the command.

**Isaac** (`r2-drive-isaac.json`, one run, exit 0):

- The first attempt was refused at launch: Isaac's mode check needs the
  profile's `legged_policy` feature, which is now declared.
- straight 0.4 m/s: **0.002 m/s**, so the first phase did not move,
  probably while the policy was still settling after start-up (not
  confirmed);
- arcs 0.22 / 0.20 m/s at 0.51 / −0.47 rad/s;
- reverse −0.18 m/s;
- tight left 0.80 rad/s; spin 0.87 rad/s;
- strafe 0.02 m/s;
- stop: residual 0.13 rad/s.

**GUI status:** `display_reasons` counts a profile with the
`legged_policy` feature as controllable. Display-only robots go from 66 to
65, and the two Menagerie ANYmals now name `asset_anymal` as their
controllable sibling.

## Limits

- Two MuJoCo drive screens and one Isaac screen, whose first phase did not
  move; experimental Loc mode only.
- No terrain, navigation or repeats.
- The policy is kept on SSD, not provisioned into the package.
