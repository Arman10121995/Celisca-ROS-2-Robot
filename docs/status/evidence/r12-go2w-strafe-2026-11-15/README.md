# R12 Go2W strafe: wheel gains and a wheeled training task (November 15, 2026)

**Status:** implemented, experimental. One drive screen per cell, so these
results are not navigation or terrain qualification.

## What was tested

The pending R12 check was "PD gains updated in `go2w_robot_lab/config.yaml`;
simulation test pending".

The November 11 edit (`1c665db4`) changed only the four wheel entries:

| Wheel gain | Before | After |
|---|---|---|
| `rl_kp` | 0 | 10 |
| `rl_kd` | 0.5 | 1.0 |

The MuJoCo bridge and the trainer drive wheels in velocity mode
(`joint_pd.py`: gain kd only), so the edit is effectively a doubled wheel
damping. For reference, robot_lab's own `UNITREE_GO2W_CFG` trained with wheel
stiffness 0 and damping 0.5, which are the "before" values.

## New wheeled training task

**Why r1 could not strafe.** `go2w_walk_r1` (November 5) was trained with the
quadruped `walk` rewards, which give `feet_air_time` a weight of +1.0. That
rewards stepping, and r1 does not strafe (0.006 m/s).

**What changed.** The new `wheeled_walk` task (`legged_training.py`) uses
robot_lab's Go2W weights from its `rough_env_cfg.py`:

- `track_lin_vel_xy_exp` 3.0 and `track_ang_vel_z_exp` 1.5;
- no air-time reward;
- `stand_still` −2.0, applied only when the command is zero
  (`stand_still_without_cmd`);
- torques −2.5e-5;
- the existing regularizers otherwise.

**Training run:** `go2w_walk_r2`.

- Settings: 512 environments, 1500 iterations, 18.4 M steps, seed 1, flat
  floor, 86 min on CPU.
- Gains: the current wheel gains.
- Policy: `policy.onnx` sha256 `f38ad831…0015`.
- Location: SSD, `robot_lab_runtime/training/go2w_walk_r2`.

## Drive screens

All runs on `nav_empty` with `scripts/sim_drive_check.sh … --strafe` in
`mode:=loc`. Values are measured ground-truth velocities.

| Policy, backend | Straight 0.4 | Spin 1.0 | Reverse −0.3 | Strafe 0.3 |
|---|---|---|---|---|
| Vendor, old wheel gains, MuJoCo | 0.311 | 0.863 | −0.181 | 0.206 |
| Vendor, new wheel gains, MuJoCo | 0.319 | 0.892 | −0.162 | **0.248** |
| `go2w_walk_r1`, MuJoCo | 0.418 | 0.588 | −0.021 | 0.006 |
| **`go2w_walk_r2`**, MuJoCo | 0.393 | 0.952 | −0.230 | 0.116 |
| Vendor, Isaac | 0.281 | 0.890 | −0.211 | 0.277 |
| **`go2w_walk_r2`**, Isaac | 0.427 | 1.010 | −0.077 | 0.271 |

Findings:

- The new wheel gains raise the vendor policy's MuJoCo strafe from 0.206 to
  0.248 m/s.
- `go2w_walk_r2` now reverses in MuJoCo and strafes at 0.116 m/s there. r1 did
  neither.
- In Isaac, r2 strafes at 0.271 m/s and tracks straight and spin better than
  the vendor policy.
- r2's reverse is weak in Isaac (−0.077 m/s), and its MuJoCo strafe is about
  half the vendor's.
- Gazebo refuses Go2W in `loc` mode: its description has no lidar and no
  ros2_control stance.

Files: `*_mujoco.json` and `*_isaac.json` are the screen records.
`go2w_walk_r2-run.json`, `-source.json` and `-final-progress.json` are the
training identity.
