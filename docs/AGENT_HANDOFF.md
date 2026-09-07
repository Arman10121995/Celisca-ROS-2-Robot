# Agent Handoff Protocol

How to resume work on Robot Lab. Read this before changing anything.

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
- **No runtime implementation was completed by the R0 documentation revision.**
  Documentation claims must match the audited revision `dff388f` until newer
  evidence exists.

## Commit discipline

- Update `platform-status.yaml` in the same change that completes, blocks or
  re-scopes a task (state, owner, evidence, `next_task`, `updated` date).
- Keep the ledger and ROADMAP consistent; ROADMAP owns scope/acceptance,
  the YAML owns state/ownership.
- One logical change per commit; include the task ID in the message
  (e.g. `R1.1: fix benchmark executable ROS placement`).

## Hardware

Physical HIL work (R9.4) requires equipment, a safe setup and explicit operator
authorization. No documentation update authorizes hardware motion.
