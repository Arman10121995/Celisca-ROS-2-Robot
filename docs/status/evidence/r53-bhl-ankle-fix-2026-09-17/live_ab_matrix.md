# Live Gazebo A/B matrix - R5.3 stance divergence (2026-09-21)

All runs: dispatch-path bringup (`simulated_robot.launch.py`, empty map,
BHL sim profile), trace-before-spawn ordering, corrected spawn z=-0.038,
`humanoid_standing_controller` effort interface, 45 s probe, isolated
ROS domain per run. Recorded artifacts under /tmp/r53* (ephemeral; each
run's `balance_report.json` + `standing.log` are the primary evidence).

| # | Variant | Fall (first nonzero tilt, node time) | Outcome |
|---|---------|--------------------------------------|---------|
| 1 | 50 Hz node, 100 Hz physics (prior baseline, spawn 0.078) | ~5.2 s | FALLS, pivot 0.005->1.27 rad ~0.6 s |
| 2 | 250 Hz node, 100 Hz physics | ~5.5-5.7 s | FALLS, same exponential runaway |
| 3 | 250 Hz node, 500 Hz physics (`max_step_size` 0.01->0.002 in empty+nav_empty worlds) | ~5.5 s | FALLS, same; period-mismatch warning gone |
| 4 | #3 + `imu_invert_tilt:=true` | ~5.3 s | FALLS, same (sign-convention hypothesis NOT confirmed) |
| 5 | #3 + arms zero drive (STANCE_PD_ARMS 0/0, reverted) | ~6.1 s | FALLS, same (arm-joint hypothesis NOT confirmed) |
| 6 | #3 + `use_sim_time:=true` | ~5.5 s | FALLS, same (wall-vs-sim clock hypothesis NOT confirmed) |
| - | Passive (no balance node at all), gz | <60 s | collapse (IMU ends ~pi-flipped, joints at limits) |

## Interpretation

The live divergence is invariant to: control rate (50->250 Hz), physics
step (0.01->0.002), IMU tilt sign, arm drive, and node clock domain. The
runaway growth constant is ~0.1 s in every variant. In MuJoCo the exact
committed law at 250 Hz is stable on the full model under noise+delay
stress and actively rejects perturbations (plant_rate_probe.txt), so the
committed control law is not the cause of the Gazebo divergence; the
cause sits in the gz_ros2_control command/measurement loop or the ODE
import (joint mapping, plugin latency, contact/friction parameters).

## Diagnostic parameter added

`humanoid_standing_controller` gained `imu_invert_tilt` (bool, default
false): negates IMU roll/pitch before the balance law. Diagnostic only;
default behavior unchanged (62 balance tests pass).

## Recommended next steps

1. Verify the effort-interface joint mapping inside gz_ros2_control:
   command a single-joint nonzero effort through the dispatch path and
   confirm exactly that joint moves (rules out ordering/mapping defects).
2. Compare ODE contact parameters for the soles against MuJoCo
   (friction cone, ERP/CFM); the URDF `joint_properties friction=0.1`
   imports as a joint property in gz but as `frictionloss` in MuJoCo.
3. Prefer the MuJoCo backend for R5.3 live validation (project goal);
   the committed law + 250 Hz rate is validated there end to end.
