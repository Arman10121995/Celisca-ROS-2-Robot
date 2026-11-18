# robot_assets ANYmal walk training (R3.6/R12), November 18, 2026

**Status: r1 not passed; r2 queued.**

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

r2 (2000 iterations) starts after Go2 stairs r3.
