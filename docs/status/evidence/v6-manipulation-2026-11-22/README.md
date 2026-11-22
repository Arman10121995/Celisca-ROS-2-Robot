# V6: manipulation missions repeated, and a TIAGo regression fixed (November 22, 2026)

**Status: 13 of 13 passed, after a fix.** The TIAGo camera-guided fetch
had regressed on MuJoCo and PyBullet. A bisect found the cause in the
arm IK, and the fix is implemented, experimental.

`run_round.sh` repeats the measured manipulation protocols with each
receipt's own command, one shared lock hold per run, at revision ddeb3c85
plus the fix below:

| Protocol (receipt) | Backend | Result | Time | Lift rise / 1 s hold drift | Earlier |
|---|---|---|---|---|---|
| URDF Panda cube through the normal GUI ([r57](../r57-urdf-arm-pick-place-2026-11-17/README.md)) | MuJoCo ×2 | **passed ×2** | 25.4, 25.3 s | 96.3, 96.1 mm / 1.6 mm | 20.5 s |
| | PyBullet ×2 | **passed ×2** | 26.1, 26.4 s | 99.8 mm / 0 mm | 22.2 s |
| URDF Panda upright can by script ([r57 cylinder](../r57-cylinder-pick-place-2026-11-18/README.md)) | MuJoCo | **passed** | 28.1 s | 92.9 mm / 1.5 mm | 28.8 s, 92.6 mm |
| | PyBullet | **passed** | 24.9 s | 98.1 mm / 0 mm | 24.0 s, 98.1 mm |
| | Gazebo | **passed** | 23.6 s | 96.8 mm / 0.4 mm | 25.2 s, 96.5 mm |
| TIAGo fetch with the head RGB-D estimate ([r59](../r59-tiago-rgbd-fetch-2026-11-18/README.md)) | MuJoCo | **passed** after the fix | 168.1 s | 99.3 mm / 0.2 mm; estimate 1.5 mm | 167.7 s, 1.5 mm |
| | PyBullet | **passed** after the fix | 173.2 s | 99.9 mm / 0 mm; estimate 6.5 mm | 142.5 s, 5.1 mm |
| Shadow hand GUI screen ([r58](../r58-shadow-hand-2026-11-20/README.md)) | MuJoCo ×2 | **passed ×2** | | | passed |

## The TIAGo regression

**Measured.** In the first pass both TIAGo fetches failed
(`before-fix-*.json`). The head camera's estimate was good (1.5 mm), but
the arm never reached `above_object`: "stage above_object timed out (worst
joint arm_6_joint 0.054 rad from its target; 6 over the 0.01 rad settle
tolerance)". PyBullet failed the same way (0.035 rad). Both had passed on
November 18–19.

**Bisect** (`bisect_tiago.sh`, `auto_bisect.sh`, results in `bisect/`).
Each step takes `urdf_pick_place.py` and `arm_pick_place.py` from a
revision, rebuilds and runs the MuJoCo fetch, then restores HEAD:
- The November 19 base (34c82c7d) passed.
- Over the 33 commits that touched these files since, the first bad one is
  **f7f476cf** (R14.3, November 20: the contact-checked F100 fetch
  candidate). The step before it, 2f041fb4, passed.
- In that commit, HEAD with only the IK rotation error reverted to its
  earlier cross-product form passed (`bisect/rotvec-cross.json`).

**Cause.** f7f476cf changed the damped-least-squares IK's rotation error
from the cross-product form, which behaves like sin θ, to the exact
rotation vector θ. The exact form is right for judging convergence,
because the cross-product form also reads zero at 180°. As the step
direction, however, its larger rotational steps moved TIAGo's solution
onto another wrist branch, which the arm then never settled into.

**First fix, then the F100.** The first fix stepped every arm along the
cross-product form while judging convergence on the exact rotation
vector. TIAGo then passed on both backends, and that is the round above.
The F100 (R14.3) fetch on Isaac then timed out at `above_object`: its
base joint swung from −0.02 to −2.96 rad and was still moving
(`ik-step/f100-isaac-claude-isaac-align-1233.json`).

A second try kept whichever of the two step forms' solutions lay
nearer the seed. That failed TIAGo again
(`ik-step/nearest-branch-tiago-mujoco.json`), because its bad wrist branch
is the nearer one.

**Fix** (`robot_lab_mujoco/arm_pick_place.py`, `ArmSpec.ik_rotation_step`):
- the step form is declared per arm;
- the default is `cross`, which every arm measured before November 20
  used;
- the F100 profile declares `exact`, which Codex developed it on
  (`scripts/provision_myagv_pro.py`, the installed profile);
- convergence is always judged on the exact rotation vector;
- a test pins the default and the validation.

With it, TIAGo passes again (`ik-step/per-arm-step-tiago-mujoco.json`).

The F100 Isaac fetch then passes `above_object` and fails in its reanchor
replan: "no IK above the object / along a straight move"
(`ik-step/f100-isaac-claude-isaac-align-1252.json`). The base had docked at
+0.028 rad and 5 mm closer. In Codex's 07:38 run it docked at −0.015 rad,
and the same replan succeeded. This is R14.3's open docking sensitivity,
not this IK.

78 focused checks pass. The F100 live acceptance (R14.3) remains open.

**Final code (dfe98603).** The Panda runs in the table above used the IK
from before any fix, which stepped along the exact rotation vector. With
the final code the Panda uses the cross-product default, so two runs
confirm it:
- the GUI cube on MuJoCo: **passed**
  (`ik-step/final-code-panda-cube-gui-mujoco-report.json`);
- the can on Gazebo: **passed** (`ik-step/final-code-panda-can-gazebo.json`).

## Limits

- One run per backend for the cylinder and TIAGo; truth poses, except for
  the TIAGo grasp from the head camera.
- Isaac was not repeated here (TIAGo fetch on Isaac was not passed before
  either).
- PyBullet's TIAGo run took 31 s longer than on November 18.

## Reproduce

```bash
docs/status/evidence/v6-manipulation-2026-11-22/run_round.sh
docs/status/evidence/v6-manipulation-2026-11-22/bisect_tiago.sh <rev> <tag>
```

Reports are copied here. Launch logs, GUI captures and logs are on SSD under
`robot_lab_runtime/v6-manipulation-20261122`. Their hashes are in
`runtime-sha256.txt`.
