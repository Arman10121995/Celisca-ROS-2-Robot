# R5.3 BHL native MuJoCo policy probe (2026-09-15)

Headless qualification of the vendored Berkeley Humanoid Lite ONNX policies on
their native MuJoCo model, using
`src/robot_lab_robots/berkeley_humanoid_lite/tools/qualify_policy.py`. This is
a development-tool probe on the robot's vendored native model and upstream
checkpoints; it does not establish ROS adapter integration.

- Policy: `policy_humanoid_legs` (`checkpoints/policy_humanoid_legs.onnx`)
- Phases: stand 5 s, perturb 5 s (5 N lateral, 0.2 s, 1 N·s impulse), walk,
  turn (0.6 rad/s), stop 5 s
- Physics dt 0.0005 s, policy dt 0.04 s; fall at 0.70 rad tilt, non-foot floor
  contact, joint-limit excess > 0.03 rad and effort saturation all abort the
  trial as safety failures
- Observation convention: command, gyro, gravity, q − nominal, dq, previous
  action (upstream low-level controller order, 75-dim feed-forward)

## Reproduce

```bash
cd <ws>
.venv/bin/python src/robot_lab_robots/berkeley_humanoid_lite/tools/qualify_policy.py \
    --policy policy_humanoid_legs --speed 0.5 --out /tmp/bhl_probe_legs
.venv/bin/python src/robot_lab_robots/berkeley_humanoid_lite/tools/qualify_policy.py \
    --policy policy_humanoid_legs --speed 0.25 --tracking-feedback \
    --out /tmp/bhl_probe_legs_tf
```

## Artifacts

- `summary_open_loop_0.5ms.json` — verbatim tool output, open-loop commands
- `summary_tracking_0.25ms.json` — verbatim tool output with the bounded
  ground-truth tracking feedback (gains xy 1/s, yaw 2/s; development-only,
  declared in the tool and in the summary JSON)
- `trace_*_head.csv` — first 40 rows of each run's per-policy-step trace;
  the full traces stay in `/tmp/bhl_probe_legs{,_tf}/trace.csv` and are fully
  regenerable with the commands above. Each summary JSON embeds
  `artifacts_sha256` over the exact config, checkpoint, MJCF and meshes used.

## Results

Open-loop at the 0.5 m/s command (2026-09-15): stand, perturb and stop pass;
no safety failures across all 30 s. Walk reaches 4.36 m of the 5.0 m commanded
(87%) but drifts 0.67 m laterally, over the 0.5 m acceptance bound; the
open-loop turn reaches only 0.94 rad of the 3.0 rad commanded (31%). The
`policy_humanoid` full-body policy shows the same pattern (5.49 m walked,
0.57 m drift; turn 0.87 rad). Honest finding: the upstream policies track
heading and forward speed imperfectly without feedback.

With the bounded ground-truth tracking feedback at a 0.25 m/s command: all
five phases pass —

| phase | measured | criteria |
| --- | --- | --- |
| stand | last-second speed 1e-4 m/s, max tilt 0.052 rad | speed ≤ 0.05, tilt ≤ 0.25 |
| perturb | recovers, last-second speed 4e-5 m/s, max tilt 0.007 rad | bounded recovery |
| walk | 2.40 m of 2.50 m (96%), lateral drift 0.01 m | ≥ 60%, ≤ 0.5 m |
| turn | 1.85 rad of 3.0 rad commanded (62%) | ≥ 50% |
| stop | drift < 0.001 m in the last second, tilt 0.010 rad | ≤ 0.05 m |

No phase in either run hit a fall, non-foot contact, joint-limit or effort
safety failure (worst joint-limit excess 0.0008 rad; effort fraction reaches
1.0 momentarily, which the monitor permits at the clamped limit).

## Upstream pins and convention verification (2026-09-15)

- `Berkeley-Humanoid-Lite-Lowlevel` pinned at `652777cc7c49884e7cd7ddfada758dc1979bf627`
  (current upstream `main` HEAD; the same commit whose conventions the
  `qualify_policy.py` docstring cites).
- `Berkeley-Humanoid-Lite-Assets` pinned at `fc90fedd008b1e56a22e3c5221548d6b24f49707`
  (current upstream `main` HEAD, 199 MB — not vendored; the robot package's
  own `mjcf/` + flattened `meshes/` serve the probe).
- Observation order verified against the vendored training config
  `source/berkeley_humanoid_lite/.../velocity/config/humanoid/env_cfg.py`
  (order preserved): velocity_commands, base_ang_vel, projected_gravity,
  joint_pos_rel, joint_vel, last_action — 3+3+3+22+22+22 = 75 dims, exactly
  the layout `qualify_policy.py` assembles. The command ranges in
  `CommandsCfg` (lin_vel_x ±1, lin_vel_y ±0.5, ang_vel_z ±1.5) are the same
  bounds `tracking_command` clamps to; the 0.25 action scale with
  default-offset targets and the `R(q).T @ [0,0,-1]` projected-gravity
  convention match `quat_rotate_inverse` with `gravity_vector [0,0,-1]` in
  the vendored `environments/mujoco.py`. The 55-dim sim2real deployment
  loop (`mode, quat, gyro, q, dq, commands`) is a different, gamepad-driven
  path and is not what these checkpoints consume.

