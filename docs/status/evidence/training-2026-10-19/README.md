# First trained Go2 walk policy (`go2_walk_r1`), 2026-10-19

This is the first policy trained with the lab's own trainer (R12,
[guide](../../../tutorials/legged-policy-training.md)). It ran through the
normal launch in three simulators.

## Training

`ros2 run robot_lab_adapter legged-train train --robot unitree_go2 --task walk --envs 512 --iterations 2000 --seed 1`
at `eedec40`, 15:39-16:53 on the 12-core Orin (CPU only). Settings are in
[run.json](run.json).

- **Environment:** MuJoCo, built from the bridge's own Go2 import and PD
  actuators, on a flat floor.
- **Contract:** the `go2_himloco` joint order, default pose, gains and
  50 Hz period, with a 45-wide observation (no history, no linear velocity).
- **Options:** no randomisation and no observation noise.

| Iteration | Wall time | Env steps | Mean return | Episode (s) | Speed error (m/s) | Yaw-rate error (rad/s) | Action std |
|---|---|---|---|---|---|---|---|
| 1 | 3 s | 12 k | – | – | 0.524 | 0.681 | 1.00 |
| 100 | 3.7 min | 1.2 M | 2.2 | 19.6 | 0.433 | 0.396 | 0.86 |
| 500 | 16.5 min | 6.1 M | 16.3 | 20.0 | 0.144 | 0.157 | 0.52 |
| 1000 | 34 min | 12.3 M | 24.0 | 20.0 | 0.096 | 0.103 | 0.31 |
| 2000 | 73.4 min | 24.6 M | 25.9 | 20.0 | 0.077 | 0.078 | 0.24 |

The errors are averaged over the training commands (vx −0.6 to 1.0, vy
±0.4, wz ±1.0) in the training environment. Every 20th iteration is in
[progress_every_20.csv](progress_every_20.csv).

The exported directory, [policy/](policy/), is an ordinary policy directory:
- `base.yaml`, `config.yaml`, `policy.onnx` (sha256 `1ed110fb…`) and
  `source.json`;
- bound to `unitree_go2`, variant `lab_walk_go2_walk_r1`.

## Training-environment evaluation

`legged-train evaluate` runs the exported ONNX through the deployment code
(`LeggedPolicy`), 8 s per command with the first second excluded
([training_env_evaluation.json](training_env_evaluation.json)):

| Command (vx, vy, wz) | Measured vx, vy, wz |
|---|---|
| 0.5, 0, 0 | 0.480, 0.002, 0.007 |
| 1.0, 0, 0 | 0.993, −0.003, 0.008 |
| −0.4, 0, 0 | −0.406, −0.007, 0.002 |
| 0, 0.3, 0 | −0.019, 0.286, −0.021 |
| 0, 0, 0.8 | −0.002, −0.046, 0.782 |
| 0, 0, −0.8 | 0.012, 0.035, −0.809 |
| 0, 0, 0 | 0.001, 0.001, −0.022 |

No falls.

## Through the normal launch (round T1, 16:53-16:59)

`MODE=loc WARMUP=10 CHECK_ARGS=--strafe scripts/sim_drive_check.sh <sim> unitree_go2 nav_empty <out> legged_policy:=<go2_walk_r1/policy>`
at `330d3b9`. Every launch log shows
`Legged policy go2/lab_walk_go2_walk_r1 ... 12 joints, 50 Hz`.

Each phase gives vx, wz from simulator truth (vy for strafe). The rl_sar
`go2_himloco` columns are the October 12 runs with the same command
([p7 evidence](../p7-rl-sar-mujoco-walk-2026-10-12/README.md)).

| Phase (command) | MuJoCo trained | MuJoCo himloco | Gazebo trained | Gazebo himloco | Isaac trained | Isaac himloco |
|---|---|---|---|---|---|---|
| straight (0.4, 0) | 0.378, 0.004 | 0.355, 0.071 | 0.332, −0.001 | 0.374, 0.072 | **0.221**, 0.007 | 0.385, 0.035 |
| left arc (0.4, 0.5) | 0.328, 0.494 | 0.310, 0.435 | 0.272, 0.458 | 0.348, 0.376 | 0.210, 0.439 | 0.360, 0.372 |
| right arc (0.4, −0.5) | 0.319, −0.491 | 0.344, −0.335 | 0.292, −0.450 | 0.348, −0.341 | 0.174, −0.404 | 0.357, −0.376 |
| reverse left (−0.3, 0.3) | −0.277, 0.291 | −0.228, 0.219 | −0.159, 0.326 | −0.237, 0.179 | −0.112, 0.300 | −0.248, 0.201 |
| tight left (0.3, 1.0 clamped) | 0.203, 0.990 | 0.227, 0.740 | 0.174, 0.917 | 0.239, 0.635 | 0.120, 0.813 | 0.254, 0.672 |
| spin in place (0, 1.0) | 0.003, **0.981** | 0.009, 0.014 | 0.039, **0.833** | −0.004, 0.020 | −0.038, **0.641** | 0.003, 0.429 |
| strafe (vy 0.3) | vy **0.285** | vy 0.113 | vy 0.224 | vy 0.120 | vy 0.131 | vy 0.096 |
| stop | 0.009, −0.013 | 0.002, −0.062 | 0.008, 0.027 | 0.001, −0.062 | −0.020, 0.018 | 0.006, −0.079 |

What this shows:

- **MuJoCo, where it trained:** it tracks every phase better than the rl_sar
  policy, and it turns on the spot (0.98 of 1.0 rad/s), which `go2_himloco`
  does not do in any backend.
- **Gazebo:** without retraining it walks every phase, turns on the spot
  (0.83 rad/s) and strafes, with forward speed at 0.33 of 0.4 m/s.
- **Isaac:** it walks and turns (0.64 rad/s on the spot), but forward speed
  is only 0.22 of 0.4 m/s and strafe 0.13 m/s. The sim-to-sim gap is
  largest here. PhysX drives at 200 Hz differ from MuJoCo's in-step PD, and
  this run had no domain randomisation.
- **All three:** it comes to rest on the stop phase and never fell.

The three runs are indexed as `drive_screen` cells with task type
`trained_policy_drive` ([training-batch.yaml](training-batch.yaml)). They
stay separate from the rl_sar `legged_policy_drive` cells. All three pass
the screen (forward speed at least half the command, then at rest). Isaac
passes narrowly: 0.221 against the 0.200 threshold.

Limits:
- one run per backend on `nav_empty`;
- no terrain, pushes, navigation or long turns (the spin phase is 3 s);
- the policy saw only MuJoCo on a flat floor.

## Held turn on the spot (round T3, 2026-10-29, `4e1fd92`)

`CHECK_ARGS="--hold-spin 20"`: a 0.4 m/s straight, then 0.8 rad/s on the
spot for 20 s, measured in four 4 s windows
([hold-spin/](hold-spin/)). Round 52 ran `go2_himloco` in Isaac the same
way.

| Window | Isaac go2_walk_r1 | Isaac go2_himloco (round 52) | MuJoCo go2_walk_r1 |
|---|---|---|---|
| 0-4 s | 0.568 | 0.296 | 0.779 |
| 5-9 s | 0.565 | 0.000 | 0.780 |
| 10-14 s | 0.552 | 0.000 | 0.781 |
| 15-19 s | 0.556 | 0.000 | 0.780 |
| drift while turning | 0.07-0.14 m | | 0.09-0.13 m |

The trained policy holds the turn in both backends; the rl_sar policy stops
after 1-2 s. Isaac turns at 0.70 of the command. Navigation with in-place
rotation is therefore possible with this policy: the
`spin_in_place:=true` launch override (`eb094b1`) lets Nav2 plan it. The
nav_obstacle missions that test it are in round 58.

## After this run

`go2_walk_r2` (same task, with domain randomisation and observation noise)
and the navigation missions follow; see the
[rounds README](../rounds-2026-10-19/README.md).
