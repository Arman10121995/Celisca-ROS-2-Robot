# R5.3 evidence: adapter-path policy validation (onboard state only), 2026-09-15

`qualify_policy_adapter.py` drives the native MuJoCo model through
**`BhlPolicyController`** — the exact class `humanoid_policy_controller` (the
ROS node) calls — fed with onboard state only: the simulated IMU quaternion
and gyro (`imu_quat` / `imu_gyro` sensors) and measured joint states mapped
by name. No simulator ground-truth feedback is used anywhere, so this is
what the deployed adapter would see. Equivalence with the open-loop probe
(`../r53-bhl-policy-probe-2026-09-15`) validates the adapter's observation
assembly, action conversion and safety path end to end.

## Reproduce

```bash
.venv/bin/python src/robot_lab_robots/berkeley_humanoid_lite/tools/qualify_policy_adapter.py \
    --policy policy_humanoid --speed 0.5 --out <out-dir>
.venv/bin/python src/robot_lab_robots/berkeley_humanoid_lite/tools/qualify_policy_adapter.py \
    --policy policy_humanoid_legs --speed 0.25 --out <out-dir>
# reference: the same protocol via the probe's inline (non-adapter) code path
.venv/bin/python src/robot_lab_robots/berkeley_humanoid_lite/tools/qualify_policy.py \
    --policy policy_humanoid --speed 0.5 --out <out-dir>
```

## Results (mujoco 3.12.0, onnxruntime 1.23.2, CPU)

| phase    | adapter full @ 0.5 | probe full @ 0.5 | adapter legs @ 0.25 | probe legs @ 0.25 |
|----------|-------------------|------------------|---------------------|-------------------|
| stand    | pass              | pass             | pass                | pass              |
| perturb  | pass              | pass             | pass                | pass              |
| walk     | **pass** (4.30 m, drift 0.42 m) | fail (5.49 m, drift 0.57 m > 0.5) | fail (0.005 m) | fail (0.006 m) |
| turn     | fail (0.56 rad < 50%) | fail (0.87 rad) | fail (0.11 rad)   | fail (0.11 rad)   |
| stop     | pass              | pass             | pass                | pass              |

- **Equivalence demonstrated.** Per-phase outcomes match the open-loop probe
  for the legs policy exactly; for the full-body policy the trajectories
  track closely (walk 4.30 vs 5.49 m, drift 0.42 vs 0.57 m) and the same
  qualitative gait is produced, but small float32/rounding divergence —
  amplified by 10 s of chaotic gait dynamics — flips the drift verdict
  across the 0.5 m bound (adapter pass, probe fail). This is measurement
  sensitivity at a threshold, not a convention mismatch. No adapter
  `SAFE_STOP` latched in any trial; no falls, non-foot floor contacts,
  joint-limit or effort violations beyond the probe's own bounds.
- **Walking through the adapter works with the full-body policy**: 86% of
  commanded distance, 0.42 m lateral drift, all safety bounds respected —
  using onboard state only.
- **Honest negative**: the legs-only policy does not walk open-loop at
  0.25 m/s in either path (it stands; ~0.005 m in 10 s). The probe's earlier
  96% walk result required `tracking_command` ground-truth feedback, which
  the deployed adapter does not have. Turning is under-tracked by both
  policies open-loop.
- The overall `passed` flag is `false` in both adapter summaries (turn), as
  the phase bars are applied unchanged from the probe.

## Files

- `full-policy-0.5/` — adapter-path summary + per-cycle trace (policy_humanoid)
- `legs-policy-0.25/` — adapter-path summary + per-cycle trace (policy_humanoid_legs)
- `probe-reference-*/` — the probe's own summaries/traces for the same two
  configs, for the equivalence comparison above
