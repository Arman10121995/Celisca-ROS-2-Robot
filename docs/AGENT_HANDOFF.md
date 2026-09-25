# Agent Handoff Protocol

These are the repository's current operating rules. The dated
[2026-09-07 audit](status/audit-2026-09-07.md) is historical evidence, not the
current source snapshot; use [`platform-status.yaml`](status/platform-status.yaml)
for live task state and the [workflow](WORKFLOW.md) for commands.

## Read first, in this order

1. [`docs/status/platform-status.yaml`](status/platform-status.yaml) — the authoritative
   task ledger: states, owners, dependencies, evidence, `next_task`.
2. [`ROADMAP.md`](../ROADMAP.md) — scope and acceptance criteria for every task.
3. [`docs/status/audit-2026-09-07.md`](status/audit-2026-09-07.md) — what was inspected,
   what passed, what failed, and what was excluded at the audited revision.

## Session rules

- **Claim before you change.** Set `owner` on your task in `platform-status.yaml`
  (state `active`) before editing shared files. One coordinator owns the ledgers;
  one owner per task.
- **Work the declared order.** Default sequence R0 → R1 → R2 → R3 → R4 → R5/R6 →
  R7 → R8 → R9. `next_task` in the ledger names the current front of the queue.
  Parallel-ready tasks are listed under `parallel_ready_after_R0`.
- **Respect scope paths.** Keep changes inside the task's `scope_paths`. If you
  need to touch something else, note it in the task and coordinate with its owner.
- **Evidence or it did not happen.** A task moves to `done` only when its
  acceptance criteria (ROADMAP.md) pass and the evidence paths exist and support
  the claim. Record commands, revision and negative results, not just successes.
- **States:** `queued` → `active` → (`partial` | `blocked`) → `done`.
  `partial` means useful work exists but acceptance has not passed. A `blocked`
  task must name the exact prerequisite, the attempted checks and the unblock action.
- **Do not silently weaken tests.** If evidence fails, reopen the task and record it.
- **Record scope with every claim.** Name the source revision, host/backend,
  command, seed, artifact paths and what was not tested. Do not copy a historical
  audit count into a current qualification claim.

## Commit discipline

- Update `platform-status.yaml` in the same change that completes, blocks or
  re-scopes a task (state, owner, evidence, `next_task`, `updated` date).
- Keep the ledger and ROADMAP consistent; ROADMAP owns scope/acceptance,
  the YAML owns state/ownership.
- One logical change per commit; include the task ID in the message
  (e.g. `R1.1: fix benchmark executable ROS placement`).

## Current continuation

After baseline revision `0be23d2` (2026-09-25), `R5.2` is the active next task.
Continue from the Go2 evidence under
`docs/status/evidence/r52-go2-policy-2026-09-25/`: the feed-forward and
opt-in inverse reverse sweeps are recorded. The inverse map reduces low-speed
overdrive but leaves -0.15 m/s in a deadband. Two five-case inverse flat-ground
suites passed all bounded screening checks (four runs total, including two
later repeats). The named `terrain_stairs` task then
failed at the first ledge with 0.115 m drive displacement and 0.527 rad peak tilt;
the process recovered and cleaned up, so this is a failed task rather than a
launch crash.

Bounded perturbation recovery is now measured. With the opt-in diagnostic force
pulse, 5 N and 20 N produce no response above stance noise and are retained as
negative controls; 35 N produces a measurable 0.070 rad peak tilt and settles
upright with no SAFE_STOP; 60 N exceeds the envelope, collapses to 0.139 m and
correctly latches `safe_stop`. The original 60 N re-stand run was inconclusive:
the brief >0.70 rad interval tripped SAFE_STOP but never latched `fallen`, so
the attempt never began. The corrected detector latches after 25 consecutive
warning-or-higher samples following a fall-threshold trip. A clean repeat in
domain 228 records `/go2/fallen` at 3.796 s and `/go2/recovery_state`
`idle → attempting → failed` at 3.796/10.628 s; it finishes upside down at
0.057 m. Recovery success now also requires measured standing height. The
nominal-pose attempt is a valid negative. A pinned MIT-licensed NJU-RLC
recovery actor was then exported to ONNX and wired behind
`go2_recovery_policy_path:=auto` plus `enable_fall_recovery:=true`. Its valid
bounded 60 N trials started at 3.800/3.808 s, timed out at 10.696/10.840 s
and both ended inverted at 0.057 m; the first raw-action trial produced a non-finite action and
failed closed. Rebuild `robot_lab_adapter` after Python edits: its installed
module is a copy despite `--symlink-install`, and three intermediate runs
using a stale installed module were discarded. Fall recovery remains
unqualified; next adapt/retrain the actor to this plant or implement another
measured whole-body get-up strategy. Keep `go2_reverse_command_map:=inverse`, the
`go2_perturbation_*` arguments and `enable_fall_recovery` opt-in.

Four five-case flat-ground suites now pass bounded screening. Note for future
runs: this host caps a usable `ROS_DOMAIN_ID` at about 232; higher values fail
at node creation and are a setup error, not a system result. Also check contract
topic message counts before trusting a trial — a launch override typed as a
string once killed the controller at startup and left the robot uncontrolled.

`R5.3` is partial: its contact-fidelity defect is fixed, but the BHL held-turn/walk
stall remains a policy fixed point and further rate/filter/contact tuning is
retired.

The current fast check is 467 passed/1 skipped after the fall-detector change; the latest map suite is 35
passed. These are scoped checks, not a platform-wide qualification. Start with
[`docs/WORKFLOW.md`](WORKFLOW.md) and the [Go2 tutorial](tutorials/go2.md).
