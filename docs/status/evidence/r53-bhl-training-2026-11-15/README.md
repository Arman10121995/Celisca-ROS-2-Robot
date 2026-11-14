# R5.3 BHL retraining in the target domain: first four runs (November 15, 2026)

**Implemented, experimental. Not passed:** no trained policy walks or turns better
than the vendored checkpoint yet.

## What was built

The trainer contract and export path the roadmap asked for:

- `robot_lab_adapter/bhl_training.py` generates an rl_sar-layout training
  template from the vendored `configs/policy_humanoid.yaml`:
  - 22 joints;
  - kp/kd 10/2 for the arms and 20/2 for the legs;
  - 4/6 N m effort limits;
  - unscaled observations;
  - PD step 1/250 s (the deployed effort-command rate) and 25 Hz decisions.
- Export also writes `<policy>/bhl/`: an ONNX whose first layer is permuted
  to BHL's commands-first observation order, plus the unchanged upstream
  config.
- `humanoid_policy_controller` loads it with `policy_dir`; the launch
  argument is `bhl_policy_dir:=<policy>/bhl`.
- Trainer height: where the root frame sits at the soles (this model), it
  measures height at the root body's centre of mass.
- `humanoid_walk` task: the upstream BHL velocity rewards.
  - `feet_air_time_positive_biped` with a 0.5 s cap;
  - foot slide from velocimeters;
  - L1 deviation of hip yaw/roll, ankle roll and the arm joints;
  - upstream tracking weights, regularizers and command ranges.
- Warm start: `--init-onnx` copies the vendored actor into PPO, with the
  identity normaliser; `--critic-warmup N` trains only the critic for N
  iterations.
- `scripts/bhl_policy_screen.py`: the GUI drive path (/key_vel → twist_mux →
  policy) in MuJoCo with the retained probe, covering a 10 s walk and held
  ±0.3 rad/s turns. On the retained vendor records it reproduces the
  held-turn stall (late yaw rate 0.002 rad/s).

Unit tests: `test_bhl_training.py` (5) and the BHL cases in
`test_legged_training.py`. They check that:

- the warm-started actor reproduces the vendored checkpoint's outputs exactly;
- the exported BHL ONNX matches the trained actor on BHL-order input.

## Runs

All runs: 512 environments, 0–8 ms actuation latency, seed 1, CPU. Progress
summaries and `run.json` files are in this directory.

| Run | Start | Task | Iterations | Result |
|---|---|---|---|---|
| `bhl_walk_r1` | scratch | `walk` (quadruped rewards) | 1500 (6917 s) | stands (episodes about 19.6 of 20 s) but never tracks: velocity error 0.49 m/s throughout |
| `bhl_walk_r2` | scratch | `humanoid_walk` | 290, stopped | stands; the biped stepping term stays about 1e-4, so it never steps |
| `bhl_walk_r3` | vendor actor, std 0.3, lr 1e-3 | `humanoid_walk` | 157, stopped | warm-up raised the adaptive rate to 1e-2; the first actor update destroyed the policy (episodes 4.2 → 1.1 s). Rate adaptation now holds during warm-up |
| `bhl_walk_r4` | vendor actor, std 0.1, lr 1e-4 | `humanoid_walk` | about 125, stopped | KL exceeds its target on the first actor updates (22 joints, 0.1 std), the rate drops to 1e-5, and episodes fall from 6.4 to about 2.3 s |

**Plumbing check.** The r1 export ran through the normal GUI drive path in
MuJoCo (`r1-screen/`, `bhl_policy_dir:=…/bhl_walk_r1/policy/bhl`):

- the controller logged the trained policy;
- the robot stayed upright (end tilt 0.36 rad);
- it moved 0.18 m in 10 s at a 0.25 m/s command, the standing policy that
  training predicted.

**The vendor policy in the training environment.** Deterministic actions,
12 s, three copies:

| Command | Result |
|---|---|
| Walk 0.25 m/s | 0.86 m |
| Turn +0.3 rad/s | ends at −2.49 rad of yaw |
| Turn −0.3 rad/s | −0.21 rad |

One copy fell. Under random commands and 0.1–0.3 noise, the vendor policy
lasts only 4–6 s per episode in this environment. The warm start therefore
begins from a fragile policy.

## What remains

Not passed: R5.3's held ±0.3 rad/s turn and sustained walk are unchanged.

Next options:

- a KL-free fine-tune (lower `desired_kl` or a fixed small rate) at modest
  command ranges;
- a GPU trainer with thousands of environments, as upstream used.

This host's CUDA build does not run (driver too old), so training is
CPU-only.
