# Go2 get-up: continued `get_up_roll` training, and get-ups in a normal launch (R5.2/R12) — November 17, 2026

**Status: implemented, experimental.** The get-up half of R5.2's done-when
is met on MuJoCo. Stairs remain.

## Training (r2)

r2 continues r1 ([receipt](../go2-get-up-roll-2026-11-17/README.md)) from its
iteration-1500 checkpoint:

- same task (`get_up_roll`), 512 environments, seed 1;
- 4500 more iterations, ending at 6000 (73.7 M environment steps);
- CPU only, about 2 s per iteration;
- identity in `run.json`, `policy-source.json` and `final-progress.json`;
- `policy.onnx` sha256 `74addce2…7f62`, on SSD under
  `robot_lab_runtime/training/go2_get_up_roll_r2/policy`.

## Training-environment evaluation

`legged-train evaluate --task get_up`: 10 settled collapsed starts with zero
command; it stands if gravity z < −0.9 and height > 0.8 × standing after 5 s.

The 10 starts are given as `--commands "0,0,0;…"`. The default command list
gives only 3 trials, and with moving commands.

| Policy | Seed 3 | Seed 7 | Seed 11 | Total |
|---|---|---|---|---|
| r1, iteration 1500 (rechecked with this procedure) | 2/10 | 3/10 | 2/10 | 7/30 |
| r2, iteration 3000 | 10/10 | — | — | — |
| r2, iteration 4500 | 10/10 | — | — | — |
| **r2, iteration 6000 (final)** | **10/10** | **9/10** | **10/10** | **29/30** |

- The r1 recheck reproduces the earlier 2/10 exactly, so the procedure is the
  same.
- r1's failures end side-lying (gravity z 0.158). The r2 miss (seed 7) ends
  tilted, at gravity z −0.157 and 0.49 × standing height.

## In a normal launch (MuJoCo, `nav_empty`)

Launch: `robot_model:=unitree_go2 simulator:=mujoco mode:=loc
legged_policy:=<r2 policy dir>`, the normal `legged-policy-controller`
path.

The new `scripts/screen_get_up.py` judges from `/odom/ground_truth` in sim
time:

- **fallen:** tilt > 60° or height < 0.6 × 0.32 m;
- **up:** tilt < 26° and height > 0.8 × 0.32 m, held for 1 s, and still up
  when the 30 s window ends.

| Start or push | Fell | Lowest | Up after | Upright at end |
|---|---|---|---|---|
| Spawned on its side (roll 1.57) | — | up z 0.0 | 0.52 s | yes |
| Spawned on its back (roll 3.0) | — | up z −0.99 | 1.40 s | yes |
| 60 N sideways, 0.2 s (twice) | **no** | — | — | yes |
| 200 N sideways, 0.2 s | yes, onto its back | up z −1.00, 0.076 m | 3.34 s | yes |
| 400 N sideways, 0.2 s | yes | up z −0.99 | 2.08 s | yes |
| 200 N forward, 0.2 s | yes, onto its back | up z −0.99 | 1.50 s | yes |

**The 60 N collapse:** 60 N is R5.2's push. It collapsed the old stance
controller in September, but it does not knock this policy over. The
stronger pushes do, and the policy gets up every time.

## What changed

**`legged_policy_controller.py`:**

- A policy whose `source.json` task is a get-up task (`get_up`,
  `get_up_roll`) now acts at any tilt.
- Other policies still latch SAFE_STOP past `fall_tilt_rad` (1.0 rad).
  Before this change, the get-up policy was stopped the moment it was
  needed: the first launch runs (`screen-roll*.json`) lay still.
- Stale state and policy failures still stop every policy.

**Launch:** the opt-in MuJoCo push (`go2_perturbation_*`) now also reaches
`legged_policy` runs, not only the old stance controller.

## Not passed (kept)

- `screen-roll1.57.json` and `screen-roll3.0.json`: before the controller
  change, SAFE_STOP stopped the policy and the robot lay still.
- `screen-push60-1.json`, `screen-b-push60.json`: the 60 N push did not
  topple the robot, so get-up was not tested.

## Limits

- **MuJoCo only.** Isaac and Gazebo get-ups were not run.
- **One run per row.**
- **Standing only:** the get-up policy is the robot's only controller here,
  so it stands but does not walk. Switching from the walking policy to this
  one after a fall is not implemented yet.
- **Stairs**, the other half of R5.2's done-when, remain.
- Traces here are thinned to 5 Hz. Launch logs stay on SSD under
  `robot_lab_runtime/go2-get-up-screen-20261117`.
