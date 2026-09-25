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

At source revision `d06a411` (2026-09-25), `R5.2` is the active next task.
Continue from the Go2 evidence under
`docs/status/evidence/r52-go2-policy-2026-09-25/`: the feed-forward and
opt-in inverse reverse sweeps are recorded. The inverse map reduces low-speed
overdrive but leaves -0.15 m/s in a deadband. Two five-case inverse flat-ground
suites passed all bounded screening checks. The named `terrain_stairs` task then
failed at the first ledge with 0.115 m drive displacement and 0.527 rad peak tilt;
the process recovered and cleaned up, so this is a failed task rather than a
launch crash.

Bounded perturbation recovery is now measured. With the opt-in diagnostic force
pulse, 5 N and 20 N produce no response above stance noise and are retained as
negative controls; 35 N produces a measurable 0.070 rad peak tilt and settles
upright with no SAFE_STOP; 60 N exceeds the envelope, collapses to 0.139 m and
correctly latches `safe_stop`. Fall detection is implemented (latched, debounced,
distinct from SAFE_STOP) and an opt-in nominal-pose re-stand attempt is
unit-tested, but the measured 60 N trial shows that attempt does **not** right
the robot. Whole-body repositioning is not implemented, so fall recovery is
still unqualified and the next step is a real get-up strategy rather than
nominal-pose PD. Keep `go2_reverse_command_map:=inverse`, the
`go2_perturbation_*` arguments and `enable_fall_recovery` opt-in.

Four five-case flat-ground suites now pass bounded screening. Note for future
runs: this host caps a usable `ROS_DOMAIN_ID` at about 232; higher values fail
at node creation and are a setup error, not a system result. Also check contract
topic message counts before trusting a trial — a launch override typed as a
string once killed the controller at startup and left the robot uncontrolled.

`R5.3` is partial: its contact-fidelity defect is fixed, but the BHL held-turn/walk
stall remains a policy fixed point and further rate/filter/contact tuning is
retired.

The current fast check is 465 passed/1 skipped after the inverse-map change; the latest map suite is 35
passed. These are scoped checks, not a platform-wide qualification. Start with
[`docs/WORKFLOW.md`](WORKFLOW.md) and the [Go2 tutorial](tutorials/go2.md).
