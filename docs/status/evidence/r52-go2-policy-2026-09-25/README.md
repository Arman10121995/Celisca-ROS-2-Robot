# R5.2 Go2 pretrained policy trial (2026-09-25)

The optional `go2_policy_path:=auto` launch argument loads the bundled
BSD-3-Clause Go2 flat-ground ONNX model described in
`src/robot_lab_adapter/policies/go2_velocity_flat/SOURCE.md`. The adapter
uses the model's 45-observation order, 12-joint action order, 50 Hz policy
step, 250 Hz bounded effort loop, 0.5 action scale and declared PD gains.
The standard Go2 launch still uses the guarded stance controller unless the
policy path is explicitly selected. In the Robot Lab GUI, the
"Go2 flat-ground policy (experimental)" checkbox is enabled for Go2 + MuJoCo
localization and fills `go2_policy_path:=auto` into the command preview.
ONNX Runtime is required for the opt-in
path; it is installed on the test host.

The flat-ground run used the standard MuJoCo launch and the same ROS probe
as the stance record. Start the probe before launch to capture startup:

```sh
python3 docs/status/evidence/r52-go2-2026-09-25/probe_stance.py \
  --duration 7 --drive-vx 0.25 --drive-start 1 --drive-end 4
ros2 launch robot_lab_bringup simulated_robot.launch.py \
  mode:=loc simulator:=mujoco robot_model:=unitree_go2 \
  map_name:=nav_empty gui:=false start_rviz:=false go2_policy_path:=auto
```

Results are from `/odom/ground_truth`, `/joint_states` and the named effort
command topic. The `passed` boolean in the JSON is the **stance-only**
criterion (which rejects deliberate travel); use the `motion` metrics and
traces for moving trials.

| Trial | Measured result | Verdict |
| --- | --- | --- |
| `go2_policy_auto_forward` | +0.805 m X during +0.25 m/s command for 3 s; 0.004 m drift after a 1 s settle; max tilt 0.029 rad; max effort 18.92 N m | Forward/stop pass on flat ground |
| `go2_policy_watchdog` | +0.793 m during the same command; publisher then ceased without sending zero; 0.013 m later drift | Command-loss stop pass |
| `go2_policy_turn_live` | +1.776 rad yaw during +0.5 rad/s command for 3 s; 0.356 m XY drift; max tilt 0.050 rad | Turns, but drift/tracking need tighter qualification |
| `go2_policy_reverse_live` | -0.25 m/s request yielded only ~0.02 m net X and 0.278 rad yaw | Low-speed reverse fails |
| `go2_policy_reverse06_live` | -0.6 m/s request yielded about -1.0 m X; max tilt 0.063 rad | Reverse displacement works above the dead zone; tracking is not calibrated |
| `go2_policy_reverse_comp025` | Continuous feedforward dead zone compensation (-0.25 m/s maps to -0.55 policy command): -1.125 m drive delta X over 3 s, 0.079 m settle drift, max tilt 0.079 rad | Reverse dead zone successfully bypassed; linear tracking remains approximate |
| `go2_policy_turn_repeat` | +0.5 rad/s yaw command for 3 s repeated with contact telemetry: +1.487 rad yaw change, 0.224 m X delta, 0.114 m stop settle drift, max tilt 0.072 rad | Repeat trial confirms turning capability and measures turning drift |
| `go2_policy_stairs` | Started at (-7.5,-4.0) facing the first `terrain_stairs` ledge; +0.5 m/s for 7 s moved only +0.118 m, max effort reached 35.55 N m | Named terrain task fails at first ledge |

After direct MuJoCo foot-contact telemetry was added,
`go2_policy_contact_forward.json` repeated the flat-ground command:
+0.831 m during the drive window, 0.005 m stopped drift, 0.033 rad peak
tilt and 1,751 direct foot-contact messages. In the 29 trace samples taken
during motion, 25 had exactly two feet loaded above 2 N. This measures the
diagonal support pattern; it does not establish robust terrain traversal.

The four-command rerun is in [`reverse_sweep_20260925_rerun/`](reverse_sweep_20260925_rerun/summary.json). It used sequential isolated ROS domains 191–194, the same 7 s probe, `nav_empty`, `go2_policy_path:=auto`, and commands from 1–4 s. All four trials completed with bounded tilt (`0.061–0.078 rad`), 1,750–1,752 direct foot-contact messages, and no launch/probe failure:

| requested reverse command | drive ΔX | observed mean vx | ratio | verdict |
|---:|---:|---:|---:|---|
| -0.15 m/s | -0.730 m | -0.243 m/s | 1.62 | motion, overdriven |
| -0.25 m/s | -1.084 m | -0.361 m/s | 1.45 | motion, overdriven |
| -0.35 m/s | -1.283 m | -0.428 m/s | 1.22 | motion, overdriven |
| -0.45 m/s | -1.493 m | -0.498 m/s | 1.11 | motion, closest |

The current reverse feed-forward map is
`policy_command = -min(1.0, 0.8 * |requested| + 0.35)`; the sweep measurements
are consistent with that map, but the resulting low-speed response is not
velocity tracking. The sweep therefore retires the “does reverse move at all?”
question for this plant and makes the next task an opt-in inverse-map/retraining
A/B, not an unconditional controller replacement. The runner and analyzer are
`run_reverse_sweep.sh` and `analyze_reverse_sweep.py`; their hermetic regression
tests are in `src/robot_lab_adapter/test/test_r5_2_go2_reverse_sweep.py`.

The opt-in inverse-map A/B is in [`reverse_sweep_20260925_inverse/`](reverse_sweep_20260925_inverse/summary.json), using the same probe, world, timing and command grid in isolated domains 201–204. It reduces low-speed overdrive but is not a qualification:

| requested reverse command | inverse drive ΔX | observed mean vx | ratio | verdict |
|---:|---:|---:|---:|---|
| -0.15 m/s | -0.067 m | -0.022 m/s | 0.15 | inside candidate deadband |
| -0.25 m/s | -0.684 m | -0.228 m/s | 0.91 | improved, still approximate |
| -0.35 m/s | -1.107 m | -0.369 m/s | 1.05 | near requested |
| -0.45 m/s | -1.261 m | -0.420 m/s | 0.93 | near requested |

All four inverse trials completed with peak tilt `0.038–0.079 rad`,
1,751–1,752 direct foot-contact messages and no launch/probe failure. Keep
`go2_reverse_command_map:=inverse` opt-in; it improves this measured grid but
does not establish repeatable velocity tracking, terrain traversal or fall
recovery.

The five-case inverse-map suite is in
[`flat_ground_suite_20260925/`](flat_ground_suite_20260925/summary.json). It used
the same MuJoCo launch, `nav_empty`, 7 s probe and command window in isolated
ROS domains 211–215. All five cases completed with zero probe/launch return codes,
`0.030–0.078 rad` peak tilt and 1,750–1,751 direct foot-contact messages each.
The bounded screening checks passed: forward `+0.277 m/s` observed for
`+0.25 m/s` requested, reverse `-0.355 m/s` for `-0.35 m/s`, turn
`+0.499 rad/s` for `+0.5 rad/s`, command-loss stop drift `0.015 m`, and zero
command with no drive displacement or yaw. This is one sequential suite, not
velocity-tracking, terrain or navigation qualification. The suite
runner and analyzer are `run_flat_ground_suite.sh` and
`analyze_flat_ground_suite.py`; hermetic coverage is in
`src/robot_lab_adapter/test/test_r5_2_go2_reverse_sweep.py`.

## Bounded perturbation and recovery (2026-09-25)

A diagnostic body-frame force pulse was added behind explicit opt-in launch
arguments (`go2_perturbation_force_n`, `go2_perturbation_start_s`,
`go2_perturbation_duration_s`, `go2_perturbation_axis`, all defaulting to
disabled). The MuJoCo spawner applies the pulse through `xfrc_applied` on the
free-joint body, and the Go2 controller now republishes its latched safety state
on `/go2/safety_state` so the probe can record transitions instead of inferring
them. The runner is `run_perturbation_trial.sh`; the probe records pre/pulse/
recovery windows plus the first threshold breach.

All trials used `nav_empty`, the opt-in policy with `go2_reverse_command_map:=inverse`,
a 0.2 s lateral (y-axis) pulse starting at 3.0 s, and a 2.0 s recovery window.

| pulse | pre-pulse max tilt | pulse window | recovery window | final height | safety state | recovery screening |
|---:|---:|---:|---:|---:|---|---|
| 5 N | 0.030 rad | 0.026 rad | 0.029 rad | 0.368 m | `nominal` | pass (below noise floor) |
| 20 N | 0.030 rad | 0.026 rad | 0.029 rad | 0.368 m | `nominal` | pass (below noise floor) |
| 35 N | 0.030 rad | 0.032 rad | 0.066 rad | 0.373 m | `nominal` | **pass — measured, bounded** |
| 60 N | 0.030 rad | 0.059 rad | 0.717 rad | 0.139 m | `safe_stop` | **fail — collapse** |

Interpretation, with its limits:

- 5 N and 20 N produce **no response distinguishable from nominal stance noise**;
  the pulse-window tilt is actually below the pre-pulse window. These are not
  stability evidence. The static tipping estimate for this stance
  (`m·g·half_width / trunk_height ≈ 30 N`) is consistent with the foot friction
  and the balance controller absorbing the smaller pulses.
- 35 N is the smallest tested magnitude with a clearly measurable response
  (peak tilt 0.070 rad, roughly 2.3x the 0.030 rad stance baseline). The
  disturbance appears in the recovery window and the robot settles upright,
  remaining `nominal` with no SAFE_STOP. This is a single measured
  perturbation-recovery point, not a qualified disturbance envelope.
- 60 N exceeds the recovery envelope: the robot collapses to 0.139 m and the
  controller latches `safe_stop`, first breaching the 0.35 rad threshold at
  3.548 s. The fail-safe behaved correctly, but this is a fall, not a recovery.
- Safety-state telemetry is itself measured: 1,729–1,750 `/go2/safety_state`
  messages per trial, with a recorded transition to `safe_stop` only in the
  60 N case.
- Two environment/harness findings are retained as negative evidence: the first
  attempt used `ROS_DOMAIN_ID=241`, which is outside this host's CycloneDDS port
  range and failed at node creation
  (`perturbation_trial_20260925T152342Z/`), and a 20 N parameter-probe launch
  was left running and was explicitly terminated.

Fall **recovery** (standing back up after the collapse) is still not
implemented or qualified. Only disturbance rejection below the tipping

## Fall detection and the re-stand attempt (2026-09-25)

`SafetyState` now carries a latched `fallen` flag that is distinct from
SAFE_STOP: SAFE_STOP means "stop driving", while `fallen` means "a get-up
attempt is required". It is debounced by `FALL_CONFIRM_CYCLES` consecutive
warning-or-higher observations *after* a fall-threshold trip, so a single tilt
spike cannot report a fall,
and it is cleared only by an explicit `reset()` — never by attitude recovering.

`FallRecovery` is an opt-in, bounded **re-stand attempt** (default off,
`enable_fall_recovery:=true`). It drives the nominal stance pose with elevated
bounded gains for at most `fall_recovery_timeout_s`, succeeds only when
*measured* tilt returns below the warn threshold **and** simulator ground-truth
body height reaches at least 0.25 m, gives up to zero effort when
the window expires, and never clears the latched `fallen` flag by itself. It is
deliberately labelled an attempt, not a get-up.

Live result against the 60 N collapse
([`fall_recovery_trial_20260925T60N_fixed/`](fall_recovery_trial_20260925T60N_fixed/probe.json),
domain 231, 16 s trial, 8 s recovery window):

| metric | value |
|---|---|
| peak tilt | 0.754 rad |
| final height | 0.139 m |
| final safety state | `safe_stop` |
| `/go2/safety_state` messages | 3,995 |
| first threshold breach | 3.58 s at 0.357 rad |
| recovery screening | **fail** |

Correction: that run did **not** establish whether the re-stand attempt worked.
The brief crossing above 0.70 rad entered `safe_stop`, then settled near
0.52 rad before the old 25-cycle, above-0.70-rad fall detector could latch.
Recovery was gated on the unobserved `fallen` flag and likely never started.
Its low body height proves a collapse, but its result cannot be attributed to
an attempted recovery.

The corrected detector latches after 25 warning-or-higher cycles following a
fall-threshold trip, and `/go2/fallen` and `/go2/recovery_state` now expose
the actual controller state. A clean repeat with the measured-height success
check is in
[`fall_trigger_height_valid_20260925T60N/`](fall_trigger_height_valid_20260925T60N/probe.json).
It used domain 228, the same 0.2 s 60 N pulse, a 16 s trial and an opt-in 8 s
attempt. Probe and launch return codes were both zero; each controller topic
published 3,991 messages.

| measured transition/result | value |
|---|---|
| `fallen: false → true` | 3.796 s |
| `recovery_state: idle → attempting → failed` | 3.796 s → 10.628 s |
| peak tilt | 3.142 rad; the body rolled onto its back |
| final body height | 0.057 m |
| final safety/recovery state | `safe_stop` / `failed` |

This is a **valid negative**: the nominal-pose PD attempt ran and did not
right the robot. It remains off by default. Whole-body repositioning is still
needed for a get-up strategy.

### Opt-in learned get-up actor

The MIT-licensed [NJU-RLC Go2 recovery actor](../../../../src/robot_lab_adapter/policies/go2_recovery_nju/SOURCE.md)
was exported to a bundled ONNX graph with the upstream 57-value proprioceptive
observation, 10-frame history and 12-joint output. It is enabled only with
`enable_fall_recovery:=true go2_recovery_policy_path:=auto`; the default remains
off. The adapter uses direct foot forces, measured joint state and IMU, a 50 Hz
actor step, 250 Hz effort output, 40/1 PD, bounded actions/targets/efforts, and
the same latched timeout/success logic. The exported ONNX graph matched the
upstream PyTorch output within `3.875e-7` on the recorded zero-input parity
check and within `1.336e-5` on 50 seeded random input pairs. See the source
note for the pinned checkpoint and license.

The first 60 N trial
([`fall_nju_recovery_trial_20260925T60N/`](fall_nju_recovery_trial_20260925T60N/probe.json))
used the unbounded raw actor action. Recovery began at 3.800 s, moved the
legs, then produced a non-finite action and failed closed at 8.588 s. The
robot ended inverted at 0.057 m. Intermediate runs after an action-clipping
source edit still used the old installed Python module, so they were discarded
as setup errors rather than described as guarded-policy evidence.

The valid bounded repeat
([`fall_nju_recovery_bounded_20260925T60N/`](fall_nju_recovery_bounded_20260925T60N/probe.json))
rebuilt the installed adapter and verified its action guard before launch. It
used ROS domain 223, `nav_empty`, a 0.2 s 60 N lateral pulse, a 16 s probe and
an 8 s opt-in recovery window. Probe and launch returned zero; 3,997 safety
messages were observed. `fallen` latched and recovery started at 3.800 s.
There was no non-finite-action error; recovery timed out at 10.696 s and the
robot again ended inverted at 0.057 m. Height rose only to 0.199 m during the
attempt. After the final inference exception guard was rebuilt, a matched
repeat in domain 222
([`fall_nju_recovery_bounded_repeat_20260925T60N/`](fall_nju_recovery_bounded_repeat_20260925T60N/probe.json))
confirmed the negative: probe and launch returned zero, 3,998 safety messages
arrived, recovery began at 3.808 s and timed out at 10.840 s with no inference
error, and the final height was again 0.057 m with the robot inverted. Its
manifest records SHA-256 of the final installed source inputs and model. This
actor is **not a qualified get-up controller on this plant**.
The explicit guard and timeout prevent continued drive after failure, but
target-domain adaptation or a different whole-body strategy is still needed.

### Delayed-start hypothesis and false-success correction

`fall_recovery_start_delay_s` adds an opt-in, zero-effort settling interval;
its default is 0 and the active timeout starts when the interval ends. This
tested whether the learned actor works better from a settled fallen pose.
Both trials used the same `nav_empty` 60 N pulse, a 1.0 s delay, an 8 s active
window and an 18 s probe.

The initial delayed run
([`fall_nju_recovery_delay1_20260925T60N/`](fall_nju_recovery_delay1_20260925T60N/probe.json))
recorded `waiting → attempting → succeeded` at 3.816/4.608/4.948 s. That
`succeeded` state was **false**: the body briefly rose to 0.472 m in the air,
but at 5.004 s only one foot showed contact (13.97 N) and at 5.104 s no foot
had contact. The controller cut effort and the body fell to 0.089 m. Tilt and
height alone cannot establish supported standing.

`FallRecovery` now requires at least three feet loaded above 2 N while tilt
stays below 0.35 rad and body height stays above 0.25 m for 0.5 s. A drop in
any condition resets the dwell timer. The rebuilt repeat
([`fall_nju_recovery_delay1_supported_20260925T60N/`](fall_nju_recovery_delay1_supported_20260925T60N/probe.json))
recorded `waiting → attempting → failed` at 3.796/4.612/11.604 s, 4,498
safety messages, zero probe/launch return codes and no inference error. The
robot never met the support criterion and ended inverted at 0.057 m. The
delayed actor is therefore **not** a recovery solution; the corrected success
test prevents a midair rebound from being mislabeled as one.

### Terminal-pose diagnosis: `unrecoverable` vs `failed`

Classifying the terminal pose of every recorded 60 N trial changed how the
negative results should be read. Upright is tilt ~0 at standing height; a body
lying on its back reads tilt ~pi.

| trial | final tilt | final height | pose |
|---|---|---|---|
| 35 N (no recovery) | 0.056 | 0.373 m | upright |
| 60 N, no recovery | 0.518 | 0.139 m | collapsed |
| 60 N, actor (3 trials) | 3.142 | 0.057 m | **inverted** |

`classify_fall_pose()` now reports `upright` / `collapsed` / `inverted` /
`unknown` from measured attitude and height, and an attempt expiring with the
trunk past `FALL_INVERTED_TILT_RAD` (2.4 rad) reports the new terminal state
`unrecoverable` instead of `failed`. This separates two different negative
results: a stand-up controller that ran its full window from a recoverable pose
and fell short (`failed`) versus a pose no standing effort can right
(`unrecoverable`). Both stay terminal and neither restarts. Missing attitude or
height reports `unknown` rather than guessing.

### Plant-parity fix and its measured effect

The actor used one flat `RECOVERY_KP = 40.0` for all 12 joints while the
measured Go2 gains are hip 100 / thigh 300 / calf 300 — 2x hot on the hips and
~0.13x cold on the load-bearing thigh and calf — and it stepped its 50 Hz joint
target with no rate limit. The first corrected run
([`fall_invalid_numpy_type_20260925T60N/`](fall_invalid_numpy_type_20260925T60N/probe.json))
is **invalid and kept only as a defect record**: the rewritten effort path
returned a `numpy.float32`, which raised `AssertionError` in the
`std_msgs/Float64MultiArray` setter and killed the controller 0.4 s after the
fall. That run appeared to show a clean non-inversion (max tilt 0.775 rad at
0.139 m) purely because a dead node commands no torque. The type cast was
restored and a regression test now pins every published effort to `float`. This
is exactly the message-count check in [`WORKFLOW.md`](../../../WORKFLOW.md)
catching a real controller kill rather than a launch error.

The rebuilt valid repeat
([`fall_nju_recovery_gainslew_20260925T60N/`](fall_nju_recovery_gainslew_20260925T60N/probe.json),
ROS domain 214, 4,491 effort messages, zero probe/launch return codes) used the
measured per-joint gains at a 0.2 scale plus a 3 rad/s target slew:

| | before | after |
|---|---|---|
| peak measured joint velocity | 55.31 rad/s | **10.81 rad/s** |
| recovery states | `waiting → attempting → failed` | `attempting → unrecoverable` |
| terminal pose | inverted, 0.057 m | inverted, 0.057 m |

The plant-parity change cut peak actuator violence by 5.1x and produced the
correct `unrecoverable` verdict, but **it did not prevent the roll-over**. The
trunk was at 0.671 rad when the attempt began at 3.792 s and then rose
monotonically through 0.962/1.674/1.933 rad to 3.130 rad by 4.47 s, crossing the
2.4 rad inverted threshold at 4.364 s — about 0.6 s into the attempt. The
learned actor is still driving the roll-over on this plant, only far less
violently, so the 60 N get-up remains unqualified. The next hypothesis must
address the actor's action at a collapsed pose (retraining or a different
whole-body strategy), not the actuator bandwidth.

Two defects found and fixed while producing this evidence, both worth keeping:

- The first attempt produced a physically contradictory result (0.040 rad peak
  tilt with a 0.057 m body height) and **zero** safety messages. The controller
  had died at startup: `InvalidParameterTypeException`, because the launch
  passed `fall_recovery_timeout_s` as a string into a DOUBLE parameter. That
  trial was invalid and has been removed rather than reported. Numeric
  recovery parameters are now converted with `_as_float` in the launch, the
  controller tolerates string-typed overrides, and a launch-contract test
  (`test_go2_fall_recovery_launch_flags_are_opt_in_and_typed`) prevents
  recurrence.
- Fall recovery must never be inferred from a surviving process. The
  pre-existing `joy_teleop` and `imu_republisher` teardown races still appear
  in the logs; they are unrelated to this feature.


### Sequenced get-up ladder (2026-09-28)

Every single-phase attempt above drove the nominal stance pose straight from a
down trunk, and the traces say why that ends inverted: from a fallen pose the
stance drive's thigh/calf torques lever the trunk over its feet instead of
raising it. `FallRecovery` now advances a bounded ladder of predeclared
joint-space waypoints, and the standing pose is only ever commanded from its
last phase:

| phase | waypoint (hip / thigh / calf per leg) | purpose |
|---|---|---|
| `tuck` | 0.00 / 1.35 / -2.70 | fold the legs in, retracting the lever that flips a down trunk |
| `roll` | loaded pair 0.00 / 0.10 / -0.90, other pair tucked | brace the legs the trunk is *measured* to rest on and push the trunk back over them |
| `crouch` | 0.00 / 1.10 / -2.20 | feet under the hips at a low standing height |
| `stand` | nominal stance | the standing pose, entered only under the gate |

`brace_legs(roll_rad, pitch_rad)` picks the pair from measured attitude (a
positive roll lifts the left flank, so a trunk resting on its right side braces
FR/RR; pitch decides when it dominates). Each phase is bounded (0.4 / 0.7 /
0.6 s), the roll count is bounded (2 cycles), the attempt window still bounds
everything, a trunk past `FALL_INVERTED_TILT_RAD` (2.4 rad) ends the attempt as
`unrecoverable`, and an unmeasured attitude holds the phase instead of guessing
one. `/go2/recovery_state` now publishes the phase with the status
(`attempting:tuck`, `attempting:roll`, ...), which is what makes the trace below
readable.

The 60 N lateral pulse that produced every negative above
([`fall_ladder_20260928T60N/`](fall_ladder_20260928T60N/probe.json), ROS domain
230, 0.2 s pulse at 3.0 s, 14 s probe, 8 s window, no learned actor, revision
`1893f65` with four tracked files modified and five source/model hashes in the
manifest):

| measured transition/result | value |
|---|---|
| probe / launch return codes | `0` / `0` |
| `fallen: false → true` | 3.864 s |
| `recovery_state` | `idle → attempting:tuck` (3.864 s) → `attempting:roll` (4.176 s) → `unrecoverable:roll` (4.800 s) |
| retraction, RR calf | -1.183 rad (3.760 s) → -2.700 rad (3.968 s) |
| tilt while the ladder ran | 0.652 → 1.042 → 1.683 → 1.931 → 2.170 → 3.041 rad (~3.6 rad/s) |
| peak tilt / final height | 3.1416 rad / 0.057 m |
| commanded effort after the verdict | 0.0 N·m |

Two honest readings of the same trace. The ladder did what it guarantees: the
standing pose was never driven from a trunk past the gate, the retraction
executed within 0.1 s of the latch, the attempt ended 0.94 s after it began as
`unrecoverable`, and the robot was then left at zero effort — 5.8 s earlier
than the single-phase attempt, which kept driving until 10.628 s. The ladder
did **not** prevent the inversion. The debounced `fallen` latch fires 0.66 s
*after* the 0.2 s pulse, when the trunk is already at 1.042 rad and rolling at
~3.6 rad/s, and no joint-space PD effort available on this plant arrests that.
Preventing the inversion therefore needs the retraction to start *before* the
fall is confirmed, or a perturbation-triggered pre-emptive retraction — not a
stronger get-up sequence.

A matched 45 N control
([`fall_ladder_20260928T45N/`](fall_ladder_20260928T45N/probe.json), same domain
and pulse width, 12 s probe) shows the ladder is not self-triggering: peak tilt
0.1485 rad, minimum body height 0.3539 m, no SAFE_STOP, `fallen` never latched
and `recovery_state` stayed `idle` for all 2,995 messages.


#### Amplitude sweep: the window the ladder can act in is empty (2026-09-28)

[`analyze_ladder_sweep.py`](analyze_ladder_sweep.py) reduces every trial
directory to what the trace measured — whether `fallen` latched, at what tilt
and tilt *rate*, how far the ladder advanced, and where the robot ended — and
writes [`ladder_sweep.json`](ladder_sweep.json). 42 trials, all with probe and
launch return code 0 (21 unrecoverable, 8 failed, 6 recovered, 7 no-fall): the five
perturbation families below, plus the placed-pose runs in the sections that
follow.

| family | trials | outcome |
|---|---|---|
| lateral impulse, 0.2 s | 45, 48 N | never topple (peak 0.149 / 0.283 rad) |
| lateral impulse, 0.2 s | 50–60 N | latch at 3.81–4.08 s, all end inverted at 3.1416 rad / 0.057 m |
| lateral sustained, 1.2–1.5 s | 40, 45, 50 N | a 3–7x larger impulse, still all inverted |
| forward (body x) | 45 N / 1.0 s, 60 N / 0.3 s, 70 N / 0.5 s | no topple, then inverted, inverted |
| drop from `spawn_z` | 0.9 m, 1.4 m | lands on its feet, peak tilt 0.040 rad, never topples |

Every trial that toppled latched `fallen` with the trunk already at 0.51–1.16 rad
and rolling at **2.6–7.2 rad/s** (the rate is averaged over the 0.2 s after the
latch: a slope across the 0.1 s samples that bracket the latch is meaningless on
a tumbling body, and reported a *falling* tilt while the trunk was rolling over).
The consequence is the important one: on these maps there is **no** measured
amplitude, axis, pulse width or drop height that leaves the robot down but not
inverted, so no fall trial can show a get-up, and the binding constraint in every
one of them is the *trigger* — the debounced latch fires 0.2–0.5 s after the
trunk is already on the floor and rotating at ~6 rad/s.


#### Placed-pose harness: measuring the ladder from a settled fallen pose

Because a perturbation can no longer produce a recoverable collapse, the fallen
pose has to be *placed*. `mujoco_spawner.py` gained `spawn_pitch` / `spawn_roll`
(after `spawn_yaw`, both 0.0 by default), composing
`Rz(yaw) * Ry(pitch) * Rx(roll)` in the same ZYX convention the adapter extracts
attitude with; the root is still lifted by the existing orientation-aware floor
clearance, so a tilted robot is placed *settled* rather than dropped. Both launch
files and `run_perturbation_trial.sh` (`SPAWN_PITCH`, `SPAWN_ROLL`) forward them,
and every manifest records the requested pose. Two tests in `test_mujoco_reset.py`
pin the convention (yaw-only is bit-identical to the old quaternion, a 90 deg
pitch is a pure y rotation, and the adapter's ZYX read-back returns what was
requested) and the floor clearance of a pitched root.


#### Placed 1.4 rad nose-down: the ladder stands the robot up

[`fall_ladder_placed_20260928Tpitch1.4/`](fall_ladder_placed_20260928Tpitch1.4/probe.json)
(domain 229, no perturbation, 20 s probe, 8 s window, revision `1893f65`) latched
`fallen` at 0.100 s and ran the ladder: `tuck` (0.104) → `roll` (0.408) → `tuck`
(0.900) → `crouch` (1.212) → `stand` (1.748) → three retries → `succeeded:stand`
(4.948 s). The sequence did the job: measured tilt went 1.40 → 0.31 → 0.17 → 0.03
rad and the trunk stood at 0.33 m with 27–36 N on all four feet.

It then fell, and the trace says why in one sample:

| measured | pre-fix | post-fix repeat |
|---|---|---|
| `recovery_state` | `succeeded:stand` at 4.948 s | `succeeded:stand` at 5.688 s |
| `max_effort_nm` in that sample | 0.0 | 4.8 |
| final tilt / height | 0.00 rad / **0.057 m** (flat on its belly) | 0.01 rad / **0.329 m** |
| feet after success | 6.4–7.0 N (trunk resting) | 28–35 N each, held to 20 s |

The node calls `FallRecovery.update()` for as long as `fallen` is latched, and a
succeeded attempt returned zero effort — so zero effort *was* the command, and
the ladder dropped the robot it had just stood up. A succeeded attempt now holds
the nominal stance, conditional on the *same* measured evidence that granted the
success (tilt under the gate, height ≥ 0.25 m, ≥ 3 loaded feet); the moment that
evidence is gone the drive stops, so the hold cannot mask a fall, and neither
`failed` nor `unrecoverable` ever holds. The repeat
([`fall_ladder_placed_hold_20260928Tpitch1.4/`](fall_ladder_placed_hold_20260928Tpitch1.4/probe.json),
bit-identical to the latch and to the success) then holds 0.329–0.330 m at
0.01 rad with ~5 N·m of stance effort for the remaining 14 s. Three unit tests in
`TestFallRecoveryLadder` pin the hold, the stop, and the no-leak-into-a-bounded-stop.

What was still wrong, and measured: the first `stand` extension **catapulted** the
robot (0.44 → 0.56 m with no feet loaded, tilt growing to 0.81 rad), the ladder
then re-tucked and needed 4.5 s and three retries before it settled.


#### Slewing the stand pose in (the catapult, fixed and measured)

The catapult was the `stand` phase stepping its target straight from the crouch
waypoint to the nominal stance, so the legs were driven 0.75 rad of knee error in
one cycle. `FALL_RECOVER_STAND_S` (0.6 s) now slews that target in from the
crouch waypoint instead; the end of the slew *is* the nominal stance, so the
standing pose is still reached, and still only from the `stand` phase. The
post-success hold is unaffected: a robot measured standing gets the full pose
immediately.

[`fall_ladder_placed_slew_20260928Tpitch1.4/`](fall_ladder_placed_slew_20260928Tpitch1.4/probe.json)
— same placed 1.4 rad nose-down pose, same domain 229, same 20 s probe and 8 s
window, source hash `49aaf4bd…`:

| measured | step target | slewed target |
|---|---|---|
| ladder timeline | 7 transitions, 3 retries | `tuck` 0.100 → `roll` 0.392 → `tuck` 0.812 → `crouch` 1.128 → `stand` 1.660 |
| `succeeded:stand` | 5.688 s | **2.324 s** |
| peak tilt after the spawn pose | 1.931 rad (the catapult) | 1.402 rad (the spawn value, never exceeded) |
| minimum height | 0.076 m | 0.104 m |
| height through the stand phase | 0.221 → 0.443 → 0.561 → 0.456 m (airborne) | 0.222 → 0.248 → 0.274 → 0.301 → 0.323 → 0.331 m (monotone, feet loaded 25–38 N throughout) |
| final tilt / height | 0.01 rad / 0.329 m | 0.01 rad / 0.329 m |

So the get-up went from 5.7 s with three self-inflicted retries to 2.3 s with
none, and the robot is standing and loaded at the end of both. Two flags on this
trial must be read carefully rather than quoted: the probe's
`recovery_screening_pass` is `true` with `first_failure: null`, but that check
covers only the window *after* the (empty) pulse, 3.2–5.2 s; and the whole-run
`passed` flag is `false` **by construction**, because a trial that starts fallen
cannot satisfy a whole-run screen of `min_height > 0.15 m` and
`max_tilt < 0.35 rad` (this one: 0.104 m and 1.402 rad). The evidence is the
trace, not the flags.



#### Capture envelope: how far the ladder can be handed a fallen robot

With placement in place, the useful question is no longer "did a perturbation
produce a recoverable fall" but "**from which settled poses can the ladder still
put the robot up**". Eight further trials
(`fall_ladder_envelope_20260928T*`, domains 226–230, no perturbation, 20 s probe,
8 s window) walk both axes, with the slewed build:

| placed pose | `fallen` latched | ladder | final tilt / height | outcome |
|---|---|---|---|---|
| pitch 0.6 rad | no (0.60 < 0.70 rad trip) | never engaged | 0.02 rad / 0.365 m | robot recovers by itself |
| pitch 0.9 rad | yes | `tuck,crouch,stand` | 0.01 rad / 0.329 m | **recovered at 1.464 s** |
| pitch 1.2 rad | yes | `tuck,roll,tuck,crouch,stand` | 0.01 rad / 0.329 m | **recovered at 2.356 s** |
| pitch 1.6 rad | yes | `tuck,roll,tuck,crouch,stand` | 0.01 rad / 0.329 m | **recovered at 2.296 s** |
| pitch 1.8 rad | yes | `tuck,tuck` | 3.1416 rad / 0.057 m | inverted: the trunk rolls over inside the retraction |
| roll 0.6 rad | no (0.60 < 0.70 rad trip) | never engaged | 0.02 rad / 0.365 m | robot recovers by itself |
| roll 0.9 / 1.2 / 1.4 rad | yes | `tuck,roll,tuck,roll` | 0.52 rad / 0.139 m | `failed:roll`, left on its right flank |

Two numbers fall out of that, and they are what the remaining work has to be
built against:

- **Pitch-dominated collapses are recoverable from ~0.8 to ~1.6 rad**, and the
  time is flat at ~2.3 s across that range. The upper edge is between 1.6 and
  1.8 rad, where the trunk's own weight rolls it over inside the retraction.
- **Roll-dominated collapses are not recoverable at all** above the 0.8 rad gate:
  0.9, 1.2 and 1.4 rad all end identically at 0.52 rad / 0.139 m on the right
  flank. The sagittal-only brace cannot right a trunk lying on its side, so a
  lateral hip input has to exist *before* any trigger work can pay off for the
  case the perturbation trials actually produce (they all roll).

A future pre-fall retraction therefore has a measurable target: engage below
~1.6 rad of pitch, and expect nothing from the roll axis until the brace does.


#### The roll sign, measured (the hip input that the ladder was missing)

The brace held the hips at zero because this project had no measured
ground-contact torque sign for the roll axis. The placed-pose harness can produce
one. `roll_phase_pose()` takes an optional `hip_rad` for the braced pair, signed
like the spawn pose's splayed stance (right leg positive, left leg mirrored,
clamped to the measured hip range ±1.0472 rad) and threaded through
`fall_recovery_roll_brace_hip_rad:=` and the runner's `ROLL_BRACE_HIP`. It is
**0.0 by default** — the default is still the qualified sagittal-only brace, and
the four trials below are the measurement, not a new default.

Same placed 1.4 rad flank pose, domains 226–229, 20 s probe, 8 s window
(`fall_ladder_hip_20260928T*`):

| braced hip | trunk tilt during the brace | minimum | where it ended | verdict |
|---|---|---|---|---|
| 0.0 (default) | 1.57 → 1.87 rad (worse) | 1.56 rad | 0.52 rad / 0.139 m on the right flank | `failed:roll` |
| **+0.4 rad** | 1.25 → 1.39 rad | 1.08 rad | 0.52 rad / 0.139 m | `failed:roll` |
| **+0.8 rad** | 1.08 → **0.86 rad** | **0.66 rad** | 1.43 rad / 0.139 m | **`failed:crouch`** |
| −0.4 rad | 1.88 → 3.07 rad | 1.88 rad | 3.1416 rad / 0.057 m | `unrecoverable:roll` (onto its back) |
| −0.8 rad | 2.08 → 3.09 rad | 2.08 rad | 2.72 rad / 0.057 m | `unrecoverable:roll` (onto its back) |

Three things follow, and they are the first positive result on the roll axis:

1. **The sign is positive** (the braced pair splays *outward*). The negative sign
   is not merely worse, it drives the trunk onto its back.
2. **0.8 rad is the magnitude that beats the gate** from a 1.4 rad flank: the
   trunk comes down to 0.66 rad, under the 0.8 rad gate, so the ladder does what
   it has never done on this axis — it leaves the roll phase and enters `crouch`.
3. The failure then moves to `crouch`, which drives the hips back to zero and
   levers the trunk over again. So the follow-up is to carry the measured hip
   input through the crouch phase and re-measure, not to re-guess the sign.

That follow-up is measured too. `crouch_phase_pose()` keeps the same input on the
same measured pair, and **only while roll dominates** — a pitch-dominated pose
still gets the plain crouch waypoint, which is the path that works. Three trials
(`fall_ladder_hipcrouch_20260928T*`, domains 226–228):

| placed roll | hip | minimum tilt | ladder reached | then |
|---|---|---|---|---|
| 1.2 rad | +0.8 | **0.68 rad** | `crouch` 1.88 s → **`stand` 2.41 s** | tips over 0.03 s later, `failed:stand`, inverted |
| 1.4 rad | +0.8 | 0.68 rad | `crouch` 1.02 s → `stand` 1.56 s | re-tucks, `unrecoverable:roll`, inverted |
| 1.4 rad | +1.0 | **0.51 rad** | `crouch` 1.04 s → `stand` 1.57 s | re-tucks, `unrecoverable:tuck`, inverted |

So the roll axis now walks the **whole** ladder — `tuck → roll → crouch → stand` —
for the first time, with the trunk measured under the gate (0.51–0.68 rad) at the
stand entry. And the next failure is now precisely located: in all three the
`stand` phase drives the hips back to zero while the trunk still carries
0.5–0.7 rad of roll, and the trunk goes over backwards (tilt 0.70 → 1.29 → 1.90 →
2.66 → 3.04 rad in 0.4 s). The remaining step is a *graded* release of the
measured splay through `stand`, not a new sign.

#### The graded release: the roll axis stops flipping (and where it stops)

`_stand_target()` now releases the splay with the trunk's *remaining* roll —
full at or beyond the 0.35 rad success tilt, tapering to exactly the nominal
stance as the trunk comes upright, and only while roll still dominates. The same
three trials again (`fall_ladder_hipstand_20260928T*`, domains 226–228, 20 s
probe, 8 s window):

| placed roll | hip | min tilt | where it ended | verdict |
|---|---|---|---|---|
| 1.2 rad | +0.8 | 0.67 rad | **0.76 rad / 0.139 m** | `failed:stand` at 6.86 s |
| 1.4 rad | +0.8 | 0.65 rad | **0.76 rad / 0.139 m** | `failed:stand` at 6.73 s |
| 1.4 rad | +1.0 | 0.50 rad | **0.70 rad / 0.139 m** | `failed:stand` at 6.87 s |

Every one of these used to end **inverted** at 3.1416 rad / 0.057 m. They now stop
*not* inverted, on their feet in a splayed crouch, and they hold it: the trunk
sits at 0.70–0.76 rad and the ladder stays in the `stand` phase, driving, until
the bounded attempt window expires at ~6.8 s and the sequence stops. That is the
bounded behaviour the rest of this work has been asking for, on the axis where
it previously had none.

It is not a stand-up, and the trace says why in one line: the robot settles with
8 N on the left feet and 2.5 N on the right, i.e. it is *balanced on the splay*.
That looked like a fixed point to break — the splay is what rolls the trunk up,
and it is also what keeps the legs from gathering under the hips.


#### Refuted: slewing the splay out to "gather the legs"

The obvious way out is to release the splay during the crouch so the feet can
plant under the hips, and the crouch now owns the splay so the release is one
line: slew it out over the crouch's own bounded duration. Measured, it is worse.
The same three trials (`fall_ladder_gather_20260928T*`, domains 226–228):

| placed roll | hip | min tilt | ladder | ended |
|---|---|---|---|---|
| 1.2 rad | +0.8 | 0.62 rad | `crouch` 0.96 → **`tuck` 1.47** → `roll` 1.78 → `crouch` 2.33 | 3.09 rad / 0.057 m |
| 1.4 rad | +0.8 | 0.64 rad | `crouch` 1.01 → `tuck` 1.54 → `roll` 1.88 → `crouch` 2.49 | 2.41 rad / 0.057 m |
| 1.4 rad | +1.0 | 0.48 rad | `crouch` 1.01 → `tuck` 1.54 → `roll` 1.86 → `crouch` 2.49 | 2.58 rad / 0.057 m |

All three end **inverted** again, and the phase trace says why: the moment the
splay starts to come out, the trunk falls back out of the gate (0.64 → over
0.8 rad), so the ladder drops back down to `tuck` and re-rolls — twice — and
each cycle ends the same way. So the pose the graded release leaves is not a
trap to be escaped; it is a **stable** pose, and the splay is what holds the
trunk there. The change is reverted, the crouch holds the splay for its whole
phase, and the test now pins that (with the negative result in its comment) so a
future attempt to "improve" it has to confront this evidence first.

What a roll-axis stand-up still needs is therefore not a release schedule but a
different primitive: the trunk has to be brought up *and* have the feet planted
under the hips at the same time, which one waypoint per phase cannot express.


This is a measured sign, not a tuned constant, and it stays opt-in until the
crouch-side behaviour is measured too.


#### Placed 1.4 rad flank: a measured negative (before the hip input)

[`fall_ladder_placed_20260928Troll1.4/`](fall_ladder_placed_20260928Troll1.4/probe.json)
and its post-fix repeat
([`...placed_hold_20260928Troll1.4/`](fall_ladder_placed_hold_20260928Troll1.4/probe.json))
both end `failed:roll` at ~1.9 s after two bounded roll cycles, one on the right
side (0.52 rad, 0.139 m, 10 N on FR/RR) and one inverted (3.1416 rad, 0.057 m).
The trunk lies on a *stable* flank here — it does not roll on to its back by
itself — and measured tilt got slightly *worse* during the attempt (1.40 →
1.87 rad). That is the result the hip measurement above answers: the
sagittal-only brace could not right it, and the next section shows which
lateral input does.


#### Where this leaves the ladder

One demonstrated get-up (placed chest-down → standing, loaded, and held, in
2.324 s and without a retry, once the stand pose is slewed in), a measured
capture envelope (**pitch 0.8–1.6 rad recoverable at ~2.3 s; nothing on the roll
axis above the 0.8 rad gate**), two measured negatives (a ballistic fall cannot
be caught after the debounced latch, and a flank-lying trunk cannot be righted by
the current brace), and a harness that can now measure the sequence at all. The
feature stays **off by default**: on these maps the only route to a recoverable
collapse is a retraction that starts before the fall is confirmed, which is
still unmeasured work — and it would have to engage below ~1.6 rad of *pitch*,
which no lateral perturbation here ever produces.



The trial does **not** complete R5.2. The policy does not reliably track
small reverse commands and cannot climb the tested ledge. Direct foot-ground
forces are now published, but the blind ONNX policy does not consume them;
other maps, fall recovery and navigation have not been qualified. GUI velocity-base,
SLAM and navigation support for Go2 therefore stay disabled. Next work is
bounded fall/perturbation recovery, then retraining or a new measured hypothesis
for terrain and navigation.

The matched repeat is in
[`flat_ground_suite_20260925_repeat/`](flat_ground_suite_20260925_repeat/summary.json),
with the same five cases, world, timing and inverse map in isolated domains
221–225. It again passed all five bounded screening checks: forward drive ΔX
`+0.836 m`, reverse `-1.111 m`, turn `+1.481 rad`, command-loss forward `+0.802 m`,
and zero displacement/yaw. Compared with the first suite, the largest absolute
drive-delta difference was `0.048 m` for reverse; peak tilt remained
`0.0298–0.0783 rad`, with 1,750–1,752 contact messages and clean process exits.
This supports repeatable bounded flat-ground screening, not velocity tracking,
terrain or navigation qualification.

The current inverse-map terrain trial is
[`terrain_stairs_inverse_20260925.json`](terrain_stairs_inverse_20260925.json).
It starts at the recorded first-ledge spawn and commands `+0.5 m/s` from 1–8 s.
The probe and launch return codes were both `0`, but the task failed: drive ΔX
was only `0.115 m`, tilt crossed the `0.35 rad` warning threshold at `3.212 s`,
peak tilt reached `0.527 rad`, and maximum effort reached `35.55 N m`. The
controller later reported attitude recovery and the launch cleaned up, so this
is a failed named terrain task with clean teardown, not a launch crash. The
checksum record is
[`terrain_stairs_inverse_20260925_manifest.json`](terrain_stairs_inverse_20260925_manifest.json).

The next R5.2 experiment is bounded fall/perturbation recovery. Record the
perturbation time, body height, tilt, contact pattern, effort, safety state and
first-failure trace. A command-loss stop, tilt warning/recovery and clean process
exit are not equivalent to recovering from a fall.

The affected Go2 core/policy, bringup-profile, MuJoCo-effort, GUI-drive and
registry Go2 tests passed together (335 tests). After the GUI checkbox was
added, 34 GUI-drive/policy tests passed under Xvfb. The adapter and bringup
packages built, and `go2_policy_path:=auto` resolved the installed ONNX graph
and its external data in the live forward run.
After direct-contact telemetry was added, the focused Go2 core/policy and
MuJoCo effort tests passed (88 tests).
