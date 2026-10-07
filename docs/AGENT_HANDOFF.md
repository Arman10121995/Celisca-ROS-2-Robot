# Agent Handoff Protocol

These are the repository's current operating rules. The dated
[2026-09-07 audit](status/audit-2026-09-07.md) is historical evidence, not the
current source snapshot; use [`platform-status.yaml`](status/platform-status.yaml)
for live task state, [current status](status/CURRENT_STATUS.md) for the
published checkpoint, and the [workflow](WORKFLOW.md) for commands.

## Read first, in this order

1. [`docs/status/platform-status.yaml`](status/platform-status.yaml) — the authoritative
   task ledger: states, owners, dependencies, evidence, `next_task`.
2. [`ROADMAP.md`](../ROADMAP.md) — scope and acceptance criteria for every task.
3. [`docs/status/audit-2026-10-02.md`](status/audit-2026-10-02.md) and
   [`docs/status/continuation-2026-10-07.md`](status/continuation-2026-10-07.md) —
   audited claim boundaries, latest measured work and retained failures.
4. [`docs/PATCH_EXECUTION_GUIDE.md`](PATCH_EXECUTION_GUIDE.md) and
   [`docs/ASSET_EXTENSION_GUIDE.md`](ASSET_EXTENSION_GUIDE.md) — execution steps,
   upstream pins and acceptance for stabilization and newly requested work.

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

- The user requests work directly on the existing `master` branch, with pushes
  to `origin/master` when requested. Do not create extra branches or worktrees
  unless explicitly requested.
- Update `platform-status.yaml` in the same change that completes, blocks or
  re-scopes a task (state, owner, evidence, `next_task`, `updated` date).
- Keep the ledger and ROADMAP consistent; ROADMAP owns scope/acceptance,
  the YAML owns state/ownership.
- One logical change per commit; include the task ID in the message
  (e.g. `R1.1: fix benchmark executable ROS placement`).

## Current continuation

Published runtime checkpoint **`091d388`** passes required build/fast/physics/
integration/registry CI. Final GUI and normal physical Drive/Panda acceptance
have separate actual artifacts. The user additionally requests all maintained
documentation to match this state: README, roadmap, status, tutorials, workflow,
architecture, tests, package guides and agent entry points. Keep dated reports
historical, correct broken commands/links, and retain full-release blockers.

The latest October 7 GUI priority is implemented as a **control column**, not
a row. Read [the workspace guide](tutorials/gui-workspace.md): Drive/Arm/Hand/
Drone pages belong to Launch's `control_notebook`; the main workspace notebook
is reached through the sidebar. Preserve `show_tab` aliases for saved workflows
and real producers. Structural labels in `robot_taxonomy.yaml` never grant
mode/controller support. Shared GUI changes require actual neutral/Drive and
normal native Panda acceptance before refreshing strict source certificates.

The latest October 5 user instruction prioritizes the new extensions before
the older roadmap: R3.6/R6.5–R6.7, R5.10, then R5.7–R5.9. Preserve existing
stabilization fixes; return to R5.6 and the remaining original acceptance next.
Check live task ownership: another agent owns the R6.7 shared heightfield lane.
Use the [checklist](status/CHECKLIST.md) and current ledger when coordinating.

Read the [2026-10-02 completion audit](status/audit-2026-10-02.md) first.
It reconciles baseline `2a0aae2`, retains actual measured robot work, and
withdraws unsupported R6/R7/R8/R9 completion from demo/metadata reports.
Do not promote those generators' PASS strings to mission qualification.

All large build, trial, bag and log artifacts on this Jetson must live under
`/workspace` on the SSD. Read [STORAGE.md](STORAGE.md) and source
`scripts/ssd_env.sh`. The 65 dormant PX4 `/tmp` artifacts now resolve through
compatibility symlinks to `/workspace/molar/robot_lab_runtime/px4/legacy-tmp/`.
Their hashes and metadata are retained; do not duplicate them back to eMMC.
Use PX4's `-d` option for a directly launched unattended FCU to avoid pxh
prompt spam. Explicit `/tmp/name` redirections ignore TMPDIR and must be
changed to a persistent SSD run directory.

The user's 2026-09-30 instruction prioritizes the stabilization patches in
[`PATCH_EXECUTION_GUIDE.md`](PATCH_EXECUTION_GUIDE.md) before further expansion.
`priority_patches` in the status ledger records their current state. Follow
that queue when continuing the current session. BHL/Go2's MuJoCo startup
regression is repaired and recorded under
`status/evidence/r52-r53-mujoco-regression-2026-09-30/`: preserve map/robot
spawn defaults and keep catch-up disabled for effort-controlled robots.
The full walking/turning/terrain milestones remain partial. The user now explicitly includes drone work. R5.4
SITL Flight acceptance is measured and its GUI path is available; see
[tutorial](tutorials/px4_x500.md) and [current continuation](status/continuation-2026-10-07.md).
Continue R5.6 four-wheel navigation and the remaining qualification matrix;
do not use the historical no-thrust diagnosis to restart PX4 plant changes.
October 5 adds strict body-pose and obstacle screens, selected real mapping,
reset and publisher-loss work. Each source stage is named; these do not close
the full pattern/backend/map/mode matrix. Use `steering_mode:=crab`, not the
undeclared `four_wheel_steer_mode` argument. The new probes query the running
drive configuration before attributing a steering trial. One mistaken launch
is retained separately from actual crab evidence. Preserve negative RPP,
pre-fix Isaac and mapping/reset trials.

PX4 GUI altitude controls and native `nav_obstacle` flight are measured. All
installed worlds can be selected, but remaining spawn/ceiling/flight cells
remain R5.10. The Worlds tab generates actual occupancy previews; its connected
free region is not a whole-building map. Downloaded extensions use the normal Launch selectors; Installed Extensions shows import status and opens Launch. Provision assets with scripts/provision_extension_assets.py on the SSD; do not add operator download buttons.
R3.6/R6.6 now have 113 installed robot profiles and 17 worlds with exact
import/autofill and named live Panda display/state evidence. Read
[the installation checkpoint](status/evidence/extensions-integrated-2026-10-05/README.md).
Stable pose/texture/backend review, additional fixtures, controllers and robot
missions remain. Official TurtleBot 4 Standard/Lite now have forty named
localization/reset/resume, actual 2D/3D Save Map and clear/obstacle Nav2 screens,
plus eight final source-frame lidar/RGB-D/Drive screens across all four engines.
Normal Launch enables their five modes with compatible defaults/autofill;
see [mode evidence](status/evidence/turtlebot4-modes-2026-10-06/README.md)
and [sensor evidence](status/evidence/turtlebot4-sensors-2026-10-06/README.md).
Preserve Gazebo's selected-world 2 ms cap, Isaac pose-derived ideal twist and
sensor cadence, bounded console/lifecycle queues and normal export cleanup.
Other maps, longer routes, materials and vendor firmware remain unqualified.
Official TurtleBot3 Burger/Waffle/Waffle Pi now have twelve final normal GUI
source LDS/physical Drive/joint/TF/Stop/watchdog screens across all four engines;
forty-eight separate localization/reset/resume, actual 2D Save Map and clear/obstacle
Nav2 screens now pass. Normal GUI enables the four measured modes on every
engine with compatible defaults/autofill. Keep 3D SLAM unavailable for RGB-only cameras.
Read the [TurtleBot3 guide](tutorials/turtlebot3.md) and
[Drive evidence](status/evidence/turtlebot3-sensors-2026-10-07/README.md).
Preserve the original 3.5 m LDS, named nav_empty spawn, virtual-frame inertia
regularizer and profile-local small-wheel servo/watchdog settings. Source
RGB-only cameras do not qualify 3D SLAM. Preserve failed replacement imports
and negative physics trials; do not purge previous working profiles.
Native Panda joint/Hand controls and physical cube lift/release remain. Its
actual MoveIt KDL/OMPL/FCL GUI Plan/Execute now passes two physical TCP targets,
collision/unreachable rejection, joint/finger invalidation, interruptions,
reset, Live Monitor and clean planner shutdown in static `nav_empty`; see
[October 7 control-column regression](status/evidence/panda-cartesian-controls-column-2026-10-07/README.md).
Servo, dynamic/attached-object scenes, repeated pick/place and other backends
remain R5.7–R5.9. Preserve failed trials and the original actuator contracts.
R6.7 terrain and R5.8/R5.9 hand/mobile manipulation need their own
implementation and actual mission proof. The Health tab loads
the ledger and latest report; update those when advancing any task.

The GUI now uses reviewed `asset_groups.yaml` parent families with exact
source-variant IDs and nested components. Do not overwrite runtime/controller
identity with a family alias or guess component mounts. The native Registry
Tk/OpenGL viewport owns its GLX context and bounded background geometry loader;
close it before destroying the Tk window. Read
[the operator guide](tutorials/registry-3d.md) and
[actual render/live-plant evidence](status/evidence/registry-3d-2026-10-07/README.md).
Source SDF visuals are an optional inspection path; physics still prefers
collision geometry. Hospital has six extreme source-pose fixtures; camera Fit
is not a world/collision/navigation repair. Keep buffers and full meshes on SSD.

The support generator now indexes exact hashed screens and derives blocked
release gates from unfinished tasks. Do not run legacy demonstration scripts
as evidence or overwrite the corrected ledger with their completion flags.

## Historical Go2 continuation and negative evidence

At baseline revision `0be23d2` (2026-09-25), `R5.2` was the next task.
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
failed closed. Fall recovery itself is now a bounded sequenced ladder
(`tuck -> roll -> crouch -> stand`) in `go2_locomotion.py`: each phase has a
predeclared waypoint and a time bound, the roll count is bounded, the nominal
stance pose is commanded only once *measured* tilt is under the 0.8 rad gate,
a trunk past 2.4 rad ends the attempt as `unrecoverable`, and an unmeasured
attitude holds the phase instead of guessing one. `/go2/recovery_state`
publishes the phase with the status (`attempting:tuck`, `attempting:roll`, ...).
Its 60 N trial (domain 230) recorded
`idle -> attempting:tuck -> attempting:roll -> unrecoverable:roll` at
3.864/4.176/4.800 s and stopped driving 5.8 s earlier than the single-phase
attempt; the debounced fall latch fires 0.66 s after the pulse at 1.042 rad and
~3.6 rad/s, so the inversion is still not prevented. A matched 45 N control
never topples and leaves the ladder idle. A 21-trial sweep since then
(`analyze_ladder_sweep.py` -> `ladder_sweep.json`; lateral impulses 45-60 N,
sustained lateral 40-50 N over 1.2-1.5 s, forward 45-70 N, drops from 0.9/1.4 m)
shows the window the ladder can act in is **empty on these maps**: every
toppling trial latches `fallen` at 0.51-1.16 rad already rolling 2.6-7.2 rad/s
and ends inverted at 3.1416 rad / 0.057 m, and everything gentler stays
upright. The trigger, not the sequence, limits a fall trial -- *later measured
to be the wrong attribution: where the fall is catchable the latch is already
on time, and the limit is the 0.42 s catchable collapse against a ~2.3 s
get-up, not the trigger (see the trigger-latency result below)*. To measure the
sequence anyway, `mujoco_spawner.py` now takes `spawn_pitch`/`spawn_roll`
(composing Rz(yaw)*Ry(pitch)*Rx(roll), 0.0 by default, plumbed through both
launches and recorded in the manifest) so a settled fallen pose can be
*placed*; `run_perturbation_trial.sh` gained `PERTURBATION_AXIS`, `SPAWN_Z`,
`SPAWN_PITCH` and `SPAWN_ROLL` for it. From a placed 1.4 rad nose-down pose the
ladder runs `tuck -> roll -> crouch -> stand`, reported `succeeded:stand` at
4.948 s with 0.33 m and 27-36 N on all four feet, and then **fell**, because a
succeeded attempt returned zero effort and the node keeps calling
`FallRecovery.update()` while `fallen` is latched: zero effort *was* the
command. That is fixed - a succeeded attempt now holds the nominal stance
while the same measured evidence that granted the success persists, and stops
the instant it is gone (3 tests; `failed`/`unrecoverable` never hold). The
repeat is bit-identical to the success and holds 0.329 m at 0.01 rad with
28-35 N per foot for the rest of the 20 s trial. A placed 1.4 rad flank pose
is a measured negative in both runs: the sagittal-only brace (hips at zero)
does not right a trunk lying on a stable flank, so the roll phase needs a
qualified lateral hip input. The `stand` phase used to *step* its target from
the crouch waypoint to the nominal stance, which catapulted the robot (0.56 m
airborne, tilt to 0.81 rad) and cost three retries; it now slews the target in
over `FALL_RECOVER_STAND_S` (0.6 s) and the same placed trial succeeds at
2.324 s with no retry and a monotone 0.222 -> 0.331 m rise, holding 0.329 m
afterwards. Read that trial's flags carefully: the probe's whole-run `passed`
is `false` *by construction* (a trial that starts fallen cannot meet
`min_height > 0.15 m` / `max_tilt < 0.35 rad`) and `recovery_screening_pass`
only covers 3.2-5.2 s, so the trace is the evidence. Eight placed-pose trials
(`fall_ladder_envelope_20260928T*`, domains 226-230) then measured the **capture
envelope**: pitch 0.9/1.2/1.6 rad all recover (1.464/2.356/2.296 s) and end at
0.01 rad / 0.329 m, pitch 1.8 rad inverts inside the retraction, 0.6 rad never
trips the fall threshold, and roll 0.9/1.2/1.4 rad all end `failed:roll` at
0.52 rad / 0.139 m. So pitch-dominated collapses are recoverable from ~0.8 to
~1.6 rad, and the roll axis is not recoverable at all above the gate: a
pre-fall retraction has to engage below ~1.6 rad of *pitch*, which is not the
axis any perturbation here produces, and a lateral hip input for the brace has
to come first. That input is now measured: `roll_phase_pose(..., hip_rad=)`
threads an opt-in, per-side-mirrored hip target for the braced pair through
`fall_recovery_roll_brace_hip_rad:=` (default 0.0) and the runner's
`ROLL_BRACE_HIP`. Four placed 1.4 rad flank trials
(`fall_ladder_hip_20260928T{p,n}{04,08}`, domains 226-229) show the sign is
**positive/outward** (negative drives the trunk onto its back: 3.14 rad /
0.057 m), that +0.4 only slows the tilt, and that **+0.8 rad brings the trunk
to 0.66 rad, under the 0.8 rad gate, so the ladder leaves the roll phase for the
first time and reaches `crouch`** — where it then fails, because the crouch
drives the hips back to zero. Next: carry the measured hip input through the
crouch phase and re-measure; the default stays the qualified sagittal-only
brace until then.

Done next: `crouch_phase_pose()` keeps the splay on the same measured pair
while roll dominates (a pitch-dominated pose keeps the plain crouch waypoint,
which is the path that works). Three trials
(`fall_ladder_hipcrouch_20260928T{p08r14,p08r12,p10r14}`, domains 226-228) now
walk the **whole** ladder on the roll axis -- `tuck -> roll -> crouch -> stand` --
with the trunk measured under the gate at the stand entry (min tilt 0.68, 0.68,
0.51 rad), where before it never left `roll`. All three then tip onto their back
within 0.4 s of the stand entry, because `stand` puts the hips back to zero
while the trunk still carries 0.5-0.7 rad of roll. The next increment is a
*graded* release of the measured splay through `stand`; the sign is settled and
is not to be re-guessed.

Done next: `_stand_target()` releases the splay with the trunk's *remaining*
roll -- full at or beyond the 0.35 rad success tilt, tapering to the nominal
stance as the trunk comes upright, only while roll dominates. The same three
trials again (`fall_ladder_hipstand_20260928T*`) now all end **not** inverted at
0.70-0.76 rad / 0.139 m, holding a splayed crouch in the `stand` phase until the
bounded attempt window expires at ~6.8 s (`failed:stand`), where every one of
them used to end at 3.1416 rad / 0.057 m. That is a safety win on an axis that
had none, and it is *not* a stand-up: the robot settles balanced on the splay
(8 N left feet, 2.5 N right), a fixed point -- the splay rolls the trunk up and
also stops the legs gathering under the hips, so the trunk never reaches the tilt
at which the splay would taper. Next candidate: a gather step between `roll` and
`crouch`, or a release keyed on measured load rather than roll angle. Still open:
trigger latency, and the roll-axis get-up.

Two more increments after that, both kept, both measured:

* `roll_phase_pose()` gained a second opt-in input `free_hip_rad`
  (`fall_recovery_roll_free_hip_rad:=`, runner `ROLL_FREE_HIP`): splay the
  *other* pair, leave the braced pair straight ("plant the upper legs for the
  moment, push with the lower ones"). Measured on the placed 1.4 rad flank
  (`fall_ladder_freehip_20260928T*`): same sign requirement (negative still
  inverts), and it does **not** hold the trunk under the gate while driving --
  the trunk dips to 0.50 rad only *after* the attempt stops at 0.96 s, which is
  the reading that makes it look like a success. Not the better primitive.
* The blind roll-cycle count is gone. A cycle is now repeated only while the
  measured tilt keeps improving by `FALL_RECOVER_ROLL_PROGRESS_RAD` (0.15 rad),
  with `FALL_RECOVER_MAX_ROLL_CYCLES` (4) as the absolute cap and the attempt
  window as the outer bound, so a flailing attempt ends on evidence
  ("roll cycle bought less than 0.15 rad of tilt") rather than arithmetic. A
  trunk that merely climbs back out of the gate after a successful roll still
  re-tucks on the count -- that regression was caught by an existing test while
  this was being built, and is now pinned by
  `test_a_roll_cycle_is_repeated_only_while_the_trunk_keeps_improving`.

So the roll axis is where it stops: two primitives measured (braced-pair splay,
free-pair splay), two schedule ideas measured and refuted (release the splay in
the crouch, spend more roll cycles), and the surviving pose is stable, bounded
and *not* standing. On this plant an open-loop waypoint ladder does not stand
the robot up from a flank-lying trunk; it rolls the trunk most of the way to
upright (1.4 -> 0.5-0.7 rad) and stops without inverting it. Closing that gap
needs a closed-loop primitive (a waypoint keyed on more than trunk attitude, or
an actor retrained on this plant) -- a different class of work.

What that primitive must achieve is now measured rather than guessed, from the
recorded foot forces. The obvious first instance -- a contact-keyed release, "let
go of the splay once the measured load shows the legs carry the trunk" -- is a
**provable no-op** on the case it targets: median measured foot load after 3 s
is 126.5 N on a standing robot (and the two independent pitch runs agree to
0.1 N, a useful determinism check) but only **23.8-23.9 N on the flank fixed
point**, 19% of it, with the free pair at 8.1 N and the braced pair at 15.8 N.
The legs are not holding the trunk there, the ground is. So the missing piece is
not a smarter *release*, it is **support creation**: 19% -> ~100% of the weight
while the trunk is still rolled over, which needs the trunk coming up and the
feet gathering at the same time. Whoever builds it should also note the
transient loads peak at 997 N in these trials, so a contact-keyed rule needs a
dwell or it fires on a slam.

Trigger latency is now measured too, and it is **not** the missing piece.
Forward sustained pushes (fall_ladder_pitch_20260928T*, domains 226-229) were
the only untested way to make the robot fall *forward*: 40 N x 2.0 s and 50 N x
1.5 s never topple (peak tilt 0.16 / 0.09 rad), 70 N x 1.0 s inverts by 3.9 s,
and 60 N x 1.5 s is the informative one -- the trunk ramps 0.07 -> 1.50 rad of
*pitch* over 0.42 s (inside the 0.8-1.6 rad capture window) and `fallen` latches
0.10 s after the 0.70 threshold, at 1.1 rad, i.e. **on time**. 0.1 s later it is
at 3.11 rad, because the 60 N force is still applied until 4.5 s. So: where the
latch is late (lateral impulses) the fall is ballistic, and where the fall is
slow enough to catch (forward) the latch is already on time. 0.42 s of catchable
collapse against a ~2.3 s get-up is an order of magnitude no trigger closes.
Both open items are now closed as *measurements*: the trigger is not the lever,
and the roll axis needs a closed-loop primitive rather than another schedule.

One more axis closed while I was in there: the recovery's *authority*. Every
trial until now ran the launch default gain/damping 0.5/0.5, never swept.
`fall_ladder_gain_20260928T*` (domains 226-228, gain/damping 1.0/1.0) shows
doubling it **destroys the one working get-up** -- the placed pitch ladder goes
from `succeeded:stand` at 2.324 s holding 0.329 m to `unrecoverable:roll` at
3.14 rad / 0.057 m -- and rescues neither flank configuration. So 0.5/0.5 is a
measured operating point next to a cliff, and the two catapults this work had to
slew away (the 0.56 m airborne stand entry, and the flip the graded release
prevented) are the same over-drive failure mode. I then wrote that the axis goes
*lower* if it goes anywhere, and tested that too
(`fall_ladder_lowgain_20260928T*`, domains 226-229, 0.3/0.3 and 0.2/0.2): the
prediction is **refuted** — all four placed poses end identically at 0.52 rad /
0.139 m with `failed:roll`, because at that authority the drive cannot alter any
pose. So 0.5/0.5 is a narrow window bounded on both sides, 1.0 over-drives and
inverts, 0.3/0.2 under-drive into a no-op. One thing worth keeping from the low
end: there the ladder **cannot invert anything** (all four ended `failed`, none
`unrecoverable`), so its failure mode is "leave the robot where it fell and say
so", which is the right way to fail.

The one positive result has also been *qualified* rather than trusted: the
placed 1.4 rad nose-down get-up was repeated three times on separate domains
(`fall_ladder_repeat_20260928Tr1..r3`, domains 226-228) and reproduces **4 of 4**
-- `succeeded:stand` at 2.324 / 2.404 / 2.236 / 2.408 s, every one ending at
0.01 rad and 0.329 m. That matters because this work has already seen trials
diverge run-to-run (the control loop is wall-clocked and interleaves with sim
time differently under load), and because a single lucky run is not a claim. It
is the only claim here with repeat evidence; the negatives are consistent across
13 perturbation trials and three placed-pose families. The *envelope* has been
qualified the same way: pitch 0.9 / 1.2 / 1.4 / 1.6 rad now recover **13 of 13**
runs (`fall_ladder_envrep_20260928T*`, domains 226-230) — 3/3/4/3, every one
ending at 0.01 rad and 0.329 m, with the success time saturating at ~2.3 s above
1.2 rad — so the reachable window for a pitch collapse is 0.9-1.6 rad and
reproducible, with the upper edge between 1.6 and 1.8 rad. Fall recovery stays
off by default. Combined poses were run after that, because every placed pose so
far had been single-axis and the roll-vs-pitch dispatch was therefore tested
only on pure single-axis bodies (`fall_ladder_combo_*` and
`fall_ladder_neardiag_*`, domains 226-229). Recovered: pitch 1.0 + roll 0.7
(2.228 s), 1.2 + 0.6, 1.2 + 1.0 and 1.4 + 0.8 (2.216-2.412 s), all ending at
0.01 rad / 0.329 m. Failed: 0.9 + 0.8, 0.9 + 0.9 and 1.0 + 0.9, all on their
backs at 0.057 m. Two corrections came out of it, both to claims of mine:
"nothing on the roll axis" was too strong (a roll-dominant pose with a real
pitch component recovers fine), and my explanation of the diagonal -- the
roll-versus-pitch tie-break -- was refuted by the near-diagonals, since 1.0 + 0.7
recovers and 1.0 + 0.9 fails, both with a clear pitch margin. The real rule is
that roll *substitutes* for pitch: the ladder wants ~1.2 rad of pitch, and below
that only a near-pure pitch pose works.

Attribution for the diagonal band, since those poses end on their *backs*
(0.057 m) rather than on a stable side: re-run at gain 0.2/0.2
(`fall_ladder_passive_20260928T*`, domains 226-227), both come upright **on their
own** within 0.6-1.2 s (tilt 0.90 -> 0.05 and 1.00 -> 0.01). So a corner-rest
diagonal is not passively stable, and at the qualified 0.5 authority the ladder's
own drive is what ends it on its back -- the pose would have risen by itself. The
two low-authority runs disagree on the final state (0.280 m vs 0.057 m), so the
band is chaotic. Deliberately *not* turned into a guard rule: that would be a
control rule fitted to three chaotic samples.

Null controls close the loop, and they are the most useful table in the evidence
README (`fall_ladder_null_20260928T*`, domains 226-228, recovery disabled):

  placed pose     no recovery            gain 0.2          gain 0.5        gain 1.0
  pitch 1.4       prone 0.057 m          success 0.280 m   success 0.329 m  inverted
  diagonal .9/.9  prone 0.057 m          success 0.280 m   inverted         --
  flank 1.4       side rest 0.52/0.139   0.52/0.139        0.52-0.76/0.139 inverted

So: the pitch value is **real and large** (the null control never gets up, and
0.5 authority reaches the full nominal 0.329 m, 4 runs out of 4); the diagonal is
real at 0.2 and harmful at 0.5; and for a pure flank the ladder's measured
contribution is doing nothing or worse -- stopped or disabled the trunk rests at
0.52 rad / 0.139 m, a running ladder lands there in one run and inverts in its
post-fix repeat -- while at 1.0 authority it always inverts a pose that would
have rested stably. The next roll primitive has to change the outcome, not the
ending pose.

The most important result is a null control on the *real* use case, not a placed
pose (`fall_ladder_null_perturb_20260928Tf60*`, domains 226-230): 60 N lateral
with recovery **off** ends in the same settled side-rest (0.52 rad / 0.139 m)
3 runs out of 3, while with it **on** every run ends inverted (3.14 rad /
0.057 m), and 1.0 s / 2.0 s start delays do not help. So on every reachable
perturbation here, `enable_fall_recovery:=true` ends **worse** than `false`.
The delays localise it to the ladder's engagement, not to timing: the core's
`safe_stop` zeroes effort in *both* arms (the null rest is passive, at 0.0 Nm
from 3.75 s on), the trunk holds the null's rest for the whole waiting window,
and the inversion begins at `attempting:tuck` -- the ungated `TUCK_POSE`
waypoint PD engages on the settled 0.52 rad / 0.139 m rest and rolls it to
3.14 rad in ~0.4 s, ending `unrecoverable:tuck` in every delayed run; a repeat
in domain 229 reproduces the original to within 0.06 s on every transition.
The spike size is not the mechanism (`analyze_tuck_entry.py` ->
`tuck_entry.json`): attempts invert under engagement samples spanning
0.0-35.55 Nm, and successful get-ups clamp the same 35.55 Nm -- what differs
is the rest pose. An earlier reading of these
trials blaming the recovery's zero-effort window is superseded: no stance
drive is being lost, and `fall_recovery_start_delay_s` cannot help because it
only moves *when* the ladder engages, never *whether* it engages on a rest the
null control keeps forever. The "keep the stance drive until the attempt
starts" policy proposed alongside that reading rests on the same refuted
premise.

Refuted after that: slewing the splay *out* over the crouch so the legs can
gather under the hips. The same three trials
(`fall_ladder_gather_20260928T*`, domains 226-228) all end **inverted** again
(3.09 / 2.41 / 2.58 rad, 0.057 m): the moment the splay comes out the trunk falls
back out of the gate, so the ladder re-tucks and re-rolls twice and each cycle
ends the same way. So the 0.70-0.76 rad pose is *stable*, not a trap, and the
splay is what holds the trunk there. The change is reverted and
`test_crouch_holds_the_measured_splay_for_the_whole_phase` pins it, with the
negative result in its comment. A roll-axis stand-up needs a different primitive
(trunk up *and* feet under the hips at once), not a release schedule -- that is
the open item now (trigger latency is closed above: it is not the lever).
Rebuild `robot_lab_adapter` after Python edits: its installed
module is a copy despite `--symlink-install`, and three intermediate runs
using a stale installed module were discarded. Two more trials were lost by
editing `run_perturbation_trial.sh` while it was executing (bash reads a script
incrementally); keep ROS domains <= 230 or CycloneDDS refuses to bind. Fall
recovery remains off by default **on evidence**; what could change that is a
first phase that cannot tip a settled trunk -- a change to the primitive, not
to its schedule. Keep `go2_reverse_command_map:=inverse`, the
`go2_perturbation_*` arguments and `enable_fall_recovery` opt-in.

Open follow-up questions (item 1 now measured; the rest open, not queued work):

1. What in the tuck path flips a *settled* side-lying trunk? **Measured**: the
   ungated `TUCK_POSE` PD engaging on the rest converts into a single
   unilateral down-side foot strike -- FL 198.2 N at +28 ms and RL 151.0 N at
   +32 ms after `attempting:tuck`, with the other three feet at 0.0 N -- after
   which every foot force reads 0.0 N for the rest of the roll (the repeat run
   reaches pi with 15.2 s of no touch at all). The striking leg's hip/thigh/
   calf all clamp on the engagement sample and are damping-limited by the
   strike itself, so the strike cannot be attributed to one joint from the
   command topic; per-foot force is the finest attribution the probe publishes
   (`fall_ladder_delayed_trace_20260928Td1.0{,b}`, `..._null_trace_...Tf60`,
   `..._placed_trace_...Tpitch1.4`, domains 226-229, `TRACE_JOINTS=true
   TRACE_INTERVAL_S=0.02`, `analyze_tuck_forces.py` -> `tuck_forces.json`).
   The recovery-off null holds the rest (roll -0.518 constant, foot medians
   <= 9.2 N, zero effort) and the placed 1.4 rad pitch control loads the front
   pair *together* (30 -> 70 N, no unloading) and stands, so the settled
   side-rest is the discriminator -- now at contact granularity.
2. Does the diagonal band's harm at the qualified 0.5 authority need a guard?
   At 0.5 the ladder's own drive ends a corner-rest diagonal on its back while
   at 0.2 the same pose rights itself; this is recorded as a hazard and
   deliberately *not* turned into a guard rule, because a rule fitted to three
   chaotic samples would be over-fitting.
3. Would a first phase that cannot tip a settled trunk make the feature
   shippable? Nothing else measured can: delays do not help, the trigger is
   not the lever, and the authority axis is bounded on both sides. This is the
   condition for revisiting `enable_fall_recovery`'s default.

A 1 s zero-effort start delay was tried next. The first delayed actor trial
reported `succeeded` at 4.948 s only because the body was airborne at 0.472 m
with at most one loaded foot; it subsequently fell to 0.089 m. Recovery success
now requires height and upright tilt **plus at least three feet above 2 N for
0.5 s**. The rebuilt repeat in domain 220 recorded
`waiting → attempting → failed` at 3.796/4.612/11.604 s, ended inverted at
0.057 m and returned zero from probe/launch. The delay is another measured
negative, while the support/dwell check fixes a false success. The delay flag
`fall_recovery_start_delay_s` defaults to 0. Keep all recovery options opt-in.

Four five-case flat-ground suites now pass bounded screening. Note for future
runs: this host caps a usable `ROS_DOMAIN_ID` at about 232; higher values fail
at node creation and are a setup error, not a system result. Also check contract
topic message counts before trusting a trial — a launch override typed as a
string once killed the controller at startup and left the robot uncontrolled.

Fall *diagnosis* is now separated from fall *tuning*. `classify_fall_pose()`
reads the terminal pose from measured attitude and height as
`upright`/`collapsed`/`inverted`/`unknown`, and an attempt expiring with the
trunk past `FALL_INVERTED_TILT_RAD` (2.4 rad) reports the new terminal state
`unrecoverable` instead of `failed`. `failed` means the controller ran its
window from a recoverable pose and fell short; `unrecoverable` means the pose
itself is out of reach of standing effort. Both are terminal. The recorded
terminal poses are 35 N → upright (0.056 rad, 0.373 m), 60 N without recovery →
collapsed (0.518 rad, 0.139 m), and 60 N with the actor → inverted (3.142 rad,
0.057 m), so the actor trials are re-filed as out-of-envelope, not weak gains.

The actor was also re-mapped onto the measured per-joint Go2 gains (it had used
one flat `RECOVERY_KP = 40.0`, 2x hot on the hips and ~0.13x cold on thigh and
calf) with a 3 rad/s target slew. A valid 60 N repeat in domain 214 cut peak
measured joint velocity 5.1x, 55.31 → 10.81 rad/s, and returned
`attempting → unrecoverable` at 3.792/10.820 s — but the trunk still crossed
2.4 rad at 4.364 s and ended inverted, so **the get-up is still unqualified and
the next change must target the actor's action at a collapsed pose, not the
gains.** Do not keep tuning actuator bandwidth against this task.

One discarded defect worth remembering: the first corrected run
(`fall_invalid_numpy_type_20260925T60N/`) returned `numpy.float32` efforts,
which raised `AssertionError` in the `std_msgs/Float64MultiArray` setter and
killed the controller 0.4 s after the fall. It *looked* like a clean
non-inversion only because a dead node commands no torque. The float cast is
restored and a regression test pins the published type; treat any trial whose
contract message count collapses as invalid, not as a good result.

`R5.3` is partial: its contact-fidelity defect is fixed, but the BHL held-turn/walk
stall remains a policy fixed point and further rate/filter/contact tuning is
retired.

The tuck-entry contact question is now answered at the resolution the harness
publishes. Four traced ladder runs (`TRACE_JOINTS=true TRACE_INTERVAL_S=0.02`,
domains 226-229) record every joint's commanded effort/position/velocity, all
four foot forces and the signed trunk roll at ~50 Hz (`probe_stance.py` now
emits signed `roll_rad`/`pitch_rad`); `analyze_tuck_forces.py` ->
`tuck_forces.json` reads them. The 60 N delayed tucks convert into a single
unilateral down-side foot strike (FL 198.2 N at +28 ms; RL 151.0 N at +32 ms;
other feet 0.0 N), the whole foot set then reads 0.0 N for the rest of the
roll (the `b` run contact-free to pi, 15.2 s), and the striking leg's three
joints clamp together -- so the strike pins to one foot, not to one joint.
The recovery-off null holds the same rest (roll -0.518, foot medians
<= 9.2 N, zero effort) and the placed 1.4 rad pitch control loads both front
feet together (30 -> 70 N, no unloading) and stands. The traced trials ran
with `tracked_modified_files_at_launch: 0`; `test_r5_2_go2_tuck_forces.py`
pins the pattern in the fast tier.

The current fast check is 619 passed after adding the Go2 core, velocity and
recovery adapter tests, the fall-pose classification, `unrecoverable`
terminal-state and actor gain/slew parity tests, and the tuck-entry and
tuck-force attribution tests to the CI fast tier; the latest map suite is 35
passed. These are scoped checks, not a platform-wide qualification. Start with
[`docs/WORKFLOW.md`](WORKFLOW.md) and the [Go2 tutorial](tutorials/go2.md).
