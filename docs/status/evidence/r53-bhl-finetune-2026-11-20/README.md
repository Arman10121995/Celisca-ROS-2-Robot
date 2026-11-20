# R5.3: BHL fine-tune r5 and a corrected turn reading (November 20, 2026)

**Status: not passed.** A gentle fine-tune of the vendored Berkeley
Humanoid Lite actor (`bhl_walk_r5`: fixed learning rate, small action
noise) still collapsed at the first actor updates, as r3 and r4 did. The
held-turn done-when is unchanged.

Separately, a reading in the 11-15 receipt was wrong. The training
environment does reproduce the deployed behaviour, so it is a sound place
to train.

## Corrected: the vendor policy turns the right way in the training environment

The [11-15 receipt](../r53-bhl-training-2026-11-15/README.md) says a +0.3
rad/s turn command "ends at −2.49 rad of yaw". That value is the robot's
*absolute* yaw, and the environment resets with a random heading. With the
initial heading subtracted (`vendor_turn_test.py`: three environments, 12 s,
deterministic actions, fixed commands):

| Command | Yaw change (three copies) | Travel |
|---|---|---|
| turn +0.3 rad/s | +0.99, +1.12, +0.99 rad (about 0.08 rad/s) | — |
| turn −0.3 rad/s | −0.19, −0.12, −0.15 rad, then **stalls after about 2 s** | — |
| walk 0.25 m/s | ±0.2 rad | about 0.9 m along the heading |

None fell. The −0.3 stall is the same held-turn stall as in the deployed
MuJoCo launch, so this environment reproduces what the done-when has to
fix. There is no sign or ordering error in the observation contract.

## r5 (new `--desired-kl` option)

`legged-train train --desired-kl 0` keeps `--learning-rate` fixed. The PPO
class already skipped adaptation when the target is 0; the trainer now
exposes it and records it in `run.json`.

`bhl_walk_r5` run settings:

- `humanoid_walk` task;
- 512 environments, 4 ms latency;
- warm start from `policy_humanoid.onnx`;
- action std 0.05;
- learning rate 3e-5, fixed (`desired_kl 0`);
- critic-only warm-up for 100 iterations.

| Iterations | Mean episode | KL | Notes |
|---|---|---|---|
| 1–100 (actor frozen) | 4–5 s (6.6 s at best) | 0 | the vendor actor itself survives only this long under random commands and resets |
| 101 | 4.66 s | 1045 | first actor update |
| 111 | 3.67 s | 1960 | |
| 129 | 3.53 s, return −6.96 | 331 | stopped |

Even at a fixed 3e-5 rate, the first update moves the 22-joint mean by
about 0.5 rad, against the 0.05 noise. The policy then degrades as in r3
and r4.

## What remains

- The vendor actor starts fragile here: under random commands it falls
  within about 4.5 s. A fine-tune needs commands kept within the range it
  survives, or a trust-region step bounded in action space rather than in
  learning rate.
- Training from scratch (r1, r2) stands but never steps. Upstream trains
  with thousands of GPU environments, and this host's CUDA build does not
  run.

Logs are on SSD under `robot_lab_runtime/training/bhl_walk_r5`.
