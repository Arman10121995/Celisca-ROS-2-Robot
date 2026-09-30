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
writes [`ladder_sweep.json`](ladder_sweep.json). 90 trials, every one with
launch return code 0 and all but one (`fall_ladder_drop_20260928Tz1.4`, probe
return code 1) with probe return code 0 (31 unrecoverable, 29 failed,
21 recovered, 9 no-fall): the five
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
inverted *while the ladder is engaged* — every sweep trial ran it on — so no
fall trial in this sweep can show a get-up, and the binding constraint in every
one of them is the *trigger* — the debounced latch fires 0.2–0.5 s after the
trunk is already on the floor and rotating at ~6 rad/s. (The recovery-off null
control in *Null controls on the actual use case* below later produces exactly
that missing pose: a 60 N lateral resting at 0.52 rad / 0.139 m — and shows the
engaged ladder is what inverts it.)


#### Placed-pose harness: measuring the ladder from a settled fallen pose

Because a perturbation with the ladder engaged never leaves the robot down but
not inverted, the fallen pose used to measure the ladder has to be *placed*
(a start delay can also reach the settled 60 N rest before engaging — the
capstone section — but only at that one attitude). `mujoco_spawner.py` gained `spawn_pitch` / `spawn_roll`
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


##### The envelope, qualified with repeats

The table above is one run per pose, and the whole ladder was shown to be
reproducible only after being repeated. So the envelope edges — the numbers any
future work would be built against — were repeated too
(`fall_ladder_envrep_20260928T*`, domains 226–230):

| placed pitch | runs | `succeeded:stand` | final tilt / height |
|---|---|---|---|
| 0.9 rad | **3** (1.464 / 1.500 / 1.476 s) | 3 of 3 | 0.01 rad / 0.329 m |
| 1.2 rad | **3** (2.356 / 2.352 / 2.308 s) | 3 of 3 | 0.01 rad / 0.329 m |
| 1.4 rad | **4** (2.324 / 2.404 / 2.236 / 2.408 s) | 4 of 4 | 0.01 rad / 0.329 m |
| 1.6 rad | **3** (2.296 / 2.364 / 2.428 s) | 3 of 3 | 0.01 rad / 0.329 m |

Thirteen of thirteen, every one ending at the same pose to three decimals, with
the success time saturating at ~2.3 s above 1.2 rad and only ~1.5 s at 0.9 rad.
Together with the single 1.8 rad failure, the reachable window for a pitch
collapse is **0.9–1.6 rad, reproducible**, and the upper edge is between 1.6 and
1.8 rad. That is the one number in this document a future recovery feature can be
sized against, so it is the one worth having repeats behind it.


#### Combined poses: the axis dispatch is validated, and the envelope is axis-aware

Every placed pose so far has been single-axis, so the roll-vs-pitch dispatch
(`brace_legs`, and the crouch's "only while roll dominates" rule) had unit tests
with pure single-axis bodies and no physics behind them. Real falls are not
single-axis, so seven combined poses were run (`fall_ladder_combo_*` and
`fall_ladder_neardiag_*`, domains 226–229), alongside the pure-pitch envelope:

| pitch | roll | outcome |
|---|---|---|
| 0.9 | 0.0 | **`succeeded:stand` ~1.48 s** (the 3/3 envelope runs) |
| 1.0 | 0.7 | **`succeeded:stand` 2.228 s**, 0.01 rad / 0.329 m |
| 1.2 | 0.6 | **`succeeded:stand` 2.412 s**, 0.01 rad / 0.329 m |
| 1.2 | 1.0 | **`succeeded:stand` 2.412 s**, 0.01 rad / 0.329 m |
| 1.4 | 0.8 | **`succeeded:stand` 2.216 s**, 0.01 rad / 0.329 m |
| 0.9 | 0.8 | `failed:roll`, 0.00 rad / **0.057 m** (on its back) |
| 0.9 | 0.9 (tie) | `failed:roll`, 0.00 rad / **0.057 m** |
| 1.0 | 0.9 | `failed:roll`, 0.00 rad / **0.057 m** |

Three things follow, and the first two correct what this document said earlier:

1. **The dispatch works.** Pitch-dominant combined poses recover at the same
   ~2.2–2.4 s as pure pitch, so `brace_legs` picking the front/rear pair and the
   crouch keeping its hips at zero is not just a unit-test artefact.
2. **"Nothing on the roll axis" was too strong.** A *roll-dominant* pose with a
   substantial pitch component (1.2 rad of pitch against 1.0 of roll) recovers
   just as well, because the crouch and stand phases act in the sagittal plane
   and a trunk that is also pitched forward can be gathered by them. The
   accurate statement is narrower: poses with **little or no pitch** — a pure
   flank — are the ones that do not recover.
3. **The axis mix matters, and it is not the tie-break.** A first reading blamed
   the roll-versus-pitch tie-break in `brace_legs`, which sends an equal
   pitch/roll pose down the roll path. Testing near-diagonals **refutes that**:
   pitch 1.0 with roll 0.7 has a clear pitch margin and recovers, while pitch 1.0
   with roll 0.9 also has a clear margin and fails. The discriminator is the
   *roll magnitude* against the pitch budget: the ladder wants roughly **1.2 rad
   of pitch**, and below that only a near-pure pitch pose works. Roll is not
   free — it substitutes for pitch — so the envelope is a function of the axis
   mix, not of total tilt.

##### Is the ladder toppling the diagonals, or are they unstable on their own?

The failing diagonals end *on their back* (0.057 m), which is worse than a pure
flank's stable side-rest (0.52 rad / 0.139 m), so the drive could be doing the
damage rather than merely failing to help. Two were re-run at gain 0.2/0.2, which
this work has already measured to be too weak to alter a pose
(`fall_ladder_passive_20260928T*`, domains 226-227):

| placed pose | tilt through the first 1.6 s at gain 0.2 | ended |
|---|---|---|
| pitch 0.9, roll 0.9 | 0.90 -> 0.74 -> 0.60 -> **0.21 -> 0.11 -> 0.05** | 0.05 rad / 0.280 m, `succeeded:stand` |
| pitch 1.0, roll 0.9 | 1.00 -> 0.92 -> 0.84 -> 0.83 -> 0.59 -> **0.19 -> 0.01** | 0.00 rad / 0.057 m, `failed:roll` |

The trunk comes upright **on its own** in both, within 0.6-1.2 s: a corner-rest
diagonal is not a stable pose, it just rolls over by gravity given time. So in
this band the ladder at its qualified 0.5 authority is not neutral -- its own
drive is what ends these poses on their backs, while the same poses would have
risen by themselves. The two low-authority runs also disagree on the final state
(0.280 m versus 0.057 m), so the band is *chaotic*, not deterministic.

That is a stronger and less comfortable statement than "the ladder cannot help
here", and it deliberately does **not** become a guard rule: refusing to attempt
whenever a pose looks like a corner-rest would be a control rule fitted to three
chaotic samples, which is the over-fitting this document has avoided everywhere
else. It is recorded as a hazard for whoever owns the recovery policy next.


##### Null controls: what the ladder is actually worth

The authority work raised an obvious worry -- the placed pitch pose "succeeded"
at gain 0.2, so maybe it rights itself under gravity and the ladder is riding a
self-righting motion. Three null controls with `enable_fall_recovery:=false`, no
get-up drive at all, settle that (`fall_ladder_null_20260928T*`, domains
226-228). With the authority runs they give the one table that says what the
ladder is worth per pose:

| placed pose | no recovery | gain 0.2 | gain 0.5 (qualified) | gain 1.0 |
|---|---|---|---|---|
| pitch 1.4 | collapses prone, 0.00 rad / 0.057 m | `succeeded:stand` 2.38 s, **0.280 m** | `succeeded:stand` 2.32-2.41 s, **0.329 m** | `unrecoverable`, 3.14 rad / 0.057 m |
| diagonal 0.9 / 0.9 | collapses prone, 0.00 rad / 0.057 m | `succeeded:stand` 1.54 s, 0.280 m | `failed:roll`, 0.00 rad / 0.057 m | not run |
| flank 1.4 | side rest, **0.52 rad / 0.139 m** | 0.52 rad / 0.139 m | 0.52-0.76 rad / 0.139 m, or inverted 3.14 rad / 0.057 m (one repeat diverged) | `unrecoverable`, 3.14 rad / 0.057 m |

Read row by row:

- **Pitch: the ladder's value is real and large.** With no drive the robot falls
  prone at 0.057 m; at the qualified authority it reaches the full nominal
  0.329 m and holds it, four runs out of four. The self-righting worry is
  refuted -- the null control never gets up.
- **Diagonal: real at low authority, harmful at the qualified one.** 0.2 authority
  reaches 0.280 m; 0.5 drives it onto its back.
- **Flank: the ladder's measured contribution is doing nothing, or worse.**
  Stopped or disabled, the trunk rests at 0.52 rad / 0.139 m; a running ladder
  lands back in that same rest in one run and inverted in its post-fix repeat
  (§ *Placed 1.4 rad flank* below), and at 1.0 authority it always inverts a
  pose that would otherwise have rested stably. That is the sharpest statement
  of the roll axis's state: the next primitive has to change the *outcome*, not
  the pose the robot ends in.


##### Null controls on the actual use case: the ladder is worse than nothing

Every null control above disables the recovery on a *placed* pose. The real use
case is a perturbed fall, so the same control was run on the headline 60 N lateral
impulse, with repeats, and with the start-delay knob turned up to separate the
waiting window from the attempt itself
(`fall_ladder_null_perturb_*`, `fall_ladder_perturbrep_*`,
`fall_ladder_delayed_*`, domains 226–230):

| 60 N lateral impulse | runs | peak tilt | terminal tilt / height |
|---|---|---|---|
| **recovery disabled** | **3** | 0.77 / 0.75 / 0.72 rad | **0.52 rad / 0.139 m** every time |
| ladder engaged, no start delay (one run at 50 N) | 3 | 3.14 rad every time | **3.14 rad / 0.057 m** every time |
| ladder engaged, 1.0 s start delay | 2 | 3.14 rad | **3.14 rad / 0.057 m** every time |
| ladder engaged, 2.0 s start delay | 1 | 3.14 rad | **3.14 rad / 0.057 m** |

Three runs with the recovery off all end in the *same* settled side-rest; six
runs with it on all end inverted. On the reachable falls, enabling
`fall_recovery` makes the outcome worse than not enabling it.

The delayed runs say *why*, and the answer is the ladder's engagement, not its
timing. They give the recovery its longest zero-effort window, and during that
window the trunk does exactly what the null control does: it settles into the
same 0.52 rad / 0.139 m rest and holds it to within 0.01 rad / 0.001 m for the
whole waiting window, at 0.0 Nm under the core's `safe_stop` (every 0.1 s
sample from 3.85 s to 4.58 s in the repeat). Then the attempt starts and the
inversion starts with it: `attempting:tuck` at 4.68 s with a 35.5 Nm command sample,
tilt 0.53 rad at 4.69 s, 1.50 rad at 4.79 s, 3.14 rad by 5.30 s — with the
tuck briefly levering the trunk *up* to 0.18 m on the way over. All three
delayed runs end `unrecoverable:tuck`, and the repeat (domain 229) reproduces the
original (domain 226) to within 0.06 s on all three transitions:
`waiting` 3.79/3.80 s, `attempting:tuck` 4.68/4.69 s, `unrecoverable:tuck`
11.66/11.60 s.

This supersedes the mechanism written when these trials were first committed
(`8129f4a`), which blamed the recovery's zero-effort publication for dropping
the robot. The traces rule that out: the core's `safe_stop` already zeroes
effort in **both** arms — the null rest itself happens at 0.0 Nm from 3.75 s
onward — so no stance drive is being lost, and the rest needs none. The calf
trace rules out a leg sweep as *the* cause too: the RR calf barely moves across
the flip in the domain-226 run (−2.729 → −2.70 rad) while it folds 1.2 rad
during engagement in its repeat (−1.49 → −2.69 rad) — a sweep present in one,
absent in the other, the same inversion in both, and the successful runs fold
the same calf without ever flipping. What differs with recovery on is only
that the ladder *re-engages* from a rest the null control keeps forever, and it does so at
every delay measured: `fall_recovery_start_delay_s` cannot help, because it
only moves *when* the ladder engages, never *whether* it engages on a pose
that should be left alone. The "keep the stance drive until the attempt
starts" policy proposed in that same commit rests on this refuted premise and
is not motivated by any measurement here.

What the engagement actually commands is pinned down as well — the source read
against every recorded attempt by [`analyze_tuck_entry.py`](analyze_tuck_entry.py)
→ [`tuck_entry.json`](tuck_entry.json). Entering `tuck` issues the `TUCK_POSE`
waypoint PD — hip 0.0 / thigh 1.35 / calf −2.70 at Kp 100/300/300 with the
launch authority 0.5/0.5 — clamped to 23.7 Nm on hip/thigh and 35.55 Nm on
calf, and it engages with no pose gate at all (the 0.8 rad gate guards only
`stand`). Three things that follow are checkable in the records and bound what
the mechanism can be:

- The engagement spike does not classify outcomes. Across the 74 recorded
  attempts the first command sample spans 0.0–35.55 Nm, and the settled
  0.52 rad / 0.139 m rest inverts under a 35.55 Nm sample (`d1.0`, `d1.0b`),
  a 6.9 Nm sample (`d2.0`), and a 3.15 Nm sample (the no-delay 50 N run). The
  probe samples the command topic instantaneously every ~0.1 s, so a clamp
  episode between samples is invisible, and the run-wide `max_command_nm`
  (35.55 Nm) carries no timestamp — the invariant is the command engaging on
  that rest, not the size of the spike it was sampled at.
- The same command saturates the same clamp in runs that *succeed*: the placed
  1.4 rad pitch rest (0.266 m, calf at −1.80) engages at 35.55 Nm and rights
  1.40 → 1.28 rad within 0.4 s. What differs is the rest, not the command — a
  0.266 m pitched rest rights, a 0.139 m settled side-rest flips.
- The traced RR calf cannot explain the spike either: it is 1.2 rad off the
  tuck target in `d1.0b` (its own command clamps) but already at target in
  `d1.0` and `d2.0`, and all three invert identically. `trace_joints` was off
  for every ladder trial recorded before the traced runs below; those four
  runs record every leg's angles and efforts at 0.02 s.

The tracing is the measurement the phase-1 question was waiting for. Four 20 s
runs repeat the ladder with per-joint tracing on (`TRACE_JOINTS=true
TRACE_INTERVAL_S=0.02`, domains 226–229; every joint's commanded effort,
position and velocity, all four foot forces and the signed trunk roll at
~50 Hz) and are read by [`analyze_tuck_forces.py`](analyze_tuck_forces.py) →
[`tuck_forces.json`](tuck_forces.json):

| run | config | engagement | strike | after the strike | ended |
| --- | --- | --- | --- | --- | --- |
| `delayed_trace` `d1.0` | 60 N, delay 1.0 | `attempting:tuck` 4.636 s | FL 198.2 N at +28 ms | free roll 0.384 s / 0.77 rad, ends on a 16 N graze | peak 1.72 rad, `failed:roll`, settled back on the side rest |
| `delayed_trace` `d1.0b` | repeat | 4.744 s | RL 151.0 N at +32 ms | free roll to −π, then 15.2 s with no touch at all | `unrecoverable:tuck`, 0.057 m |
| `null_trace` `f60` | recovery off | none | none | roll −0.518 constant, foot medians ≤ 9.2 N, peak effort 0.0 Nm | rest held to 20 s |
| `placed_trace` `pitch1.4` | placed 1.4 rad pitch | 0.1 s | front pair together, 30 → 70 N ramp | no unloading, no free roll | pitch 1.40 → 1.16 rad, roll 0.000, `succeeded:stand` at 0.329 m |

The tipping reaction is a single down-side foot strike, and the records hold
no other: on the strike sample one foot reads 151–198 N while the other three
read 0.0 N, 28–32 ms after `attempting:tuck`, and from that sample on every
foot force stays 0.0 N while the trunk rolls — in the repeat run all the way
to π, without one further touch for the rest of the trial. The strike cannot
be assigned to one joint of the striking leg, because all three publish
clamp-level commands on the engagement sample (the FL leg: +23.7 / −23.7 /
−35.55 Nm) and are damping-limited by the strike itself (the calf's −33 rad/s
fold turns its command positive), so a per-foot force is the finest
attribution this harness publishes; splitting it further needs per-link
contact wrenches, a new probe. What the strike also is not is a push: on the
placed pitch rest the same command loads the front pair *together* (30 → 70 N
by +0.15 s, no unloading, no free roll) while pitch decreases
monotonically, and the settled side-rest converts the same sweep into one
unilateral impulse. The measured answer to *which* ground reaction tips it is
therefore at foot granularity: the down-side foot's strike, after which no
contact participates — the flip completes on momentum, with the trunk's own
ground contact unpublished.

That is the sharpest safety statement in this document, and it is the reason the
feature stays off by default on evidence rather than caution: on every reachable
perturbation measured here, `enable_fall_recovery:=true` ends worse than
`false`, and no timing knob measured so far changes that. What could change it
is a first phase that cannot tip a settled trunk — a change to the primitive,
not to its schedule — and that belongs with whoever owns the recovery next.

These stay open as *questions*, not as queued work — except the first, which
the tracing above has now measured: what in phase 1 flips a *settled*
side-lying trunk is the ungated `TUCK_POSE` PD engaging on that rest and
converting into a single down-side foot strike that unloads the whole foot set
and leaves the trunk rolling on momentum. Whether a first phase that cannot do
that would make the feature shippable at all stays open in the same sense. So
does the diagonal hazard above — the drive ending corner-rests on their backs
at 0.5 authority: recorded, deliberately not guarded.


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


#### The second hypothesis (free-pair splay) and a better stopping rule

`roll_phase_pose()` now takes a second, independent opt-in input,
`free_hip_rad` (`fall_recovery_roll_free_hip_rad:=`, runner `ROLL_FREE_HIP`): it
splays the *other* pair and leaves the braced pair straight, which is the classic
"plant the upper legs for the moment, push with the lower ones" split. Both
inputs are 0.0 by default.

Measured on the placed 1.4 rad flank (`fall_ladder_freehip_20260928T*`, domains
226–228), it behaves like its sibling — same sign requirement — and is **not**
the better primitive:

| braced / free splay | minimum tilt while driving | ended |
|---|---|---|
| 0 / **+0.8** | 1.36 rad (trunk dips to 0.50 rad only *after* the attempt ends) | 0.52 rad / 0.139 m, `failed:roll` |
| 0 / −0.8 | 1.38 rad | 3.1416 rad / 0.057 m, `unrecoverable:roll` |
| +0.8 / +0.8 | 0.60 rad | 0.76 rad / 0.139 m, `failed:stand` |

The trap in the first row is worth stating plainly: the trunk *does* reach
0.50 rad, but the trace shows it doing so at ~1.2 s, after the attempt had
already stopped driving at 0.96 s. Read as a "the primitive got it to 0.49 rad"
result, it would be wrong.

That trial did produce a change worth keeping. The roll phase used to be
repeated on a blind count, and the count is not a measurement: it now repeats a
cycle only while the measured tilt keeps improving by
`FALL_RECOVER_ROLL_PROGRESS_RAD` (0.15 rad), with
`FALL_RECOVER_MAX_ROLL_CYCLES` (4) as the absolute cap and the attempt window
as the outer bound. So a flailing attempt now ends on evidence —
`"roll cycle bought less than 0.15 rad of tilt"` — instead of on arithmetic, and
a trunk that merely *climbs back* out of the gate after a successful roll still
re-tucks on the count. The trial above ended that way, one cycle earlier and
with a reason attached, instead of two cycles of the same nothing.


#### What the closed-loop primitive would actually have to do

The remaining item is a roll primitive keyed on more than trunk attitude, and
the obvious first instance is a *contact-keyed release*: let go of the splay once
the measured foot load shows the legs are carrying the trunk. The recorded
traces answer that before any of it is written — median measured foot load after
3 s, per pair, braced = FR/RR for a right-flank pose:

| trial | total | feet ≥ 2 N | braced pair | free pair |
|---|---|---|---|---|
| pitch get-up (works) | **126.5 N** | 4 | 63.3 N | 63.2 N |
| pitch repeat 2 | **126.5 N** | 4 | 63.3 N | 63.2 N |
| flank 1.2, braced splay 0.8 | 23.8 N | 4 | 15.8 N | 8.1 N |
| flank 1.4, braced splay 0.8 | 23.9 N | 4 | 15.8 N | 8.1 N |
| flank 1.4, free splay 0.8 | 19.9 N | 2 | 19.9 N | 0.0 N |
| flank 1.4, both splays 0.8 | 23.9 N | 4 | 15.8 N | 8.1 N |

(As a side check on determinism: the two independent pitch runs agree to 0.1 N on
the settled load.)

A standing robot carries 126.5 N; the flank fixed point carries **24 N — 19% of
it**. The legs are not holding the trunk there, the *ground* is, which is exactly
why releasing the splay drops it, and it means a release rule keyed on "the
measured load shows the legs carry the trunk" is **provably a no-op** on the case
it was meant to fix: the condition it keys on is measurably false, permanently.

So the missing primitive is not a smarter *release*, it is **support creation**:
getting the legs from ~19% to ~100% of the weight while the trunk is still
rolled over, which needs the trunk coming up and the feet gathering under the
hips *at the same time* — something one waypoint per phase cannot express. That
is a precise requirement now, with a number attached, and it is what a
closed-loop primitive (or a retrained actor) has to deliver. One more constraint
for whoever builds it: the transient loads in these trials peak at 997 N during
the get-up's crouch, so any contact-keyed rule needs a dwell or it will fire on
a slam.


#### Where the roll axis actually stops

Two primitives measured (splay the braced pair, splay the free pair), two
schedule ideas measured and refuted (release the splay in the crouch, spend more
roll cycles), and the pose that survives is stable, bounded and *not* standing.
On this plant, an open-loop waypoint ladder does not stand the robot up from a
flank-lying trunk; what it can do is roll the trunk most of the way to upright
(1.4 → 0.5–0.7 rad) and then stop without inverting it. Closing that last gap
needs a closed-loop primitive — a waypoint that depends on more than the trunk's
attitude, or an actor retrained on this plant — which is a different class of
work from the ladder.


#### Trigger latency, measured on both axes (the last open item)

The remaining open item was a retraction that starts *before* the debounced
`fallen` latch. It is now measured on both axes, and the two results point in
opposite directions — which is why neither produces a recoverable fall:

*Lateral impulses* (50–60 N, 0.2 s): the latch is **late** — it fires 0.66 s
after the 0.2 s pulse, with the trunk already at 1.04 rad and rolling at
~3.6 rad/s. This is the case a pre-fall trigger would help, and it is also the
case where the collapse is ballistic, so there is nothing left to help.

*Forward sustained* (60 N for 1.5 s, `fall_ladder_pitch_20260928Tf60p1.5`,
domain 227) is the interesting one, because the collapse is *gradual* and lands
inside the pitch capture envelope:

| sim s | tilt | height | ladder |
|---|---|---|---|
| 3.33 | 0.07 rad | 0.333 m | `idle` |
| 3.54 | 0.70 rad | 0.280 m | `idle` |
| 3.64 | 1.19 rad | 0.219 m | `fallen` latches at 3.632 s → `attempting:tuck` |
| 3.74 | 1.50 rad | 0.249 m | `attempting:tuck` |
| 3.85 | **3.11 rad** | 0.229 m | `attempting:tuck` |
| 4.06 | 3.14 rad | 0.053 m | `attempting:tuck` |

So on this axis the latch is **on time** — 0.10 s after the 0.70 rad threshold,
at 1.1 rad of *pitch*, squarely inside the 0.8–1.6 rad window the ladder
recovers from in 2.3 s. A pre-fall trigger would be engaging earlier into a
collapse that is already being handled. The problem is the next line: 0.1 s
later the trunk is at 3.11 rad, because the 60 N force is *still being applied*
(until 4.5 s) and the retraction cannot hold against it. The catchable window is
the 0.42 s ramp; the get-up needs ~2.3 s.

That is the answer, and it is not the one the handoff expected: **the trigger is
not the missing piece.** On the axis where the latch is late the fall is
ballistic; on the axis where the fall is slow enough to be catchable the latch is
already on time. The mismatch is 0.42 s of catchable collapse against a 2.3 s
get-up — an order of magnitude, and not something a trigger can close. Two
companion trials bound the forward axis: 40 N × 2.0 s and 50 N × 1.5 s never
topple at all (peak tilt 0.16 and 0.09 rad — the robot leans and slides), and
70 N × 1.0 s tops over faster still (1.40 rad at 3.6 s, inverted by 3.9 s).


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


#### Qualifying the one positive result: 4 of 4

The get-up above is a single trial, and this work has already seen trials
diverge run-to-run (the wall-clocked control loop interleaves differently with
sim time under load). So the one positive claim was repeated rather than
trusted: the same placed 1.4 rad nose-down pose, the same configuration, three
more runs on separate ROS domains (`fall_ladder_repeat_20260928Tr1..r3`,
domains 226–228) against the original (`fall_ladder_placed_slew_*`).

| run | `succeeded:stand` | final tilt / height |
|---|---|---|
| original (`placed_slew`) | 2.324 s | 0.01 rad / 0.329 m |
| repeat 1 | 2.404 s | 0.01 rad / 0.329 m |
| repeat 2 | 2.236 s | 0.01 rad / 0.329 m |
| repeat 3 | 2.408 s | 0.01 rad / 0.329 m |

Four out of four, with a 0.17 s spread on the success time and the same terminal
pose to three decimals, on four different domains. That is the difference
between "a demonstrated get-up" and one lucky run, and it is the only claim in
this section that now has repeat evidence behind it. The negatives do not need
it: they are consistent across 13 perturbation trials and three placed-pose
families, with the same terminal pose every time.


#### The operating point is measured, and 0.5/0.5 is a cliff edge

Every trial so far ran the recovery at the launch default authority,
`fall_recovery_gain_scale=0.5` / `fall_recovery_damping_scale=0.5`. That was
never swept, and it turns out to matter more than the waypoints
(`fall_ladder_gain_20260928T*`, domains 226–228, same placed poses, gain 1.0 /
damping 1.0):

| configuration | at 0.5 / 0.5 | at 1.0 / 1.0 |
|---|---|---|
| placed pitch 1.4 | min tilt 0.48 rad, **`succeeded:stand` at 2.324 s**, 0.01 rad / 0.329 m | min tilt 1.38 rad, `unrecoverable:roll`, 3.14 rad / 0.057 m |
| placed flank 1.2, free splay +0.8 | min tilt 0.48 rad, 0.52 rad / 0.139 m, `failed:roll` | min tilt 1.20 rad, `unrecoverable`, 3.14 rad / 0.057 m |
| placed flank 1.4, braced splay +0.8 | min tilt 0.65 rad, 0.76 rad / 0.139 m, `failed:stand` | min tilt 0.62 rad, `unrecoverable`, 3.12 rad / 0.057 m |

Doubling the authority **destroys the one get-up that works** — the pitch ladder
goes from standing and held to inverted — and it does not rescue either flank
configuration. So 0.5/0.5 is not a timid default: it is a measured operating
point sitting next to a cliff, and it explains the two catapults this work had to
slew away (the 0.56 m airborne stand entry, and the flip the graded release
prevented). Both are the same failure mode: over-driving the legs levers or
launches the trunk.


#### …and lower is not the answer either (a refuted prediction)

That first section ended by predicting the axis goes *lower* if it goes anywhere.
It was tested, and the prediction is wrong (`fall_ladder_lowgain_20260928T*`,
domains 226–229, the same four placed poses at 0.3/0.3 and 0.2/0.2):

| configuration | 0.5 / 0.5 | 0.3 / 0.3 | 0.2 / 0.2 |
|---|---|---|---|
| placed pitch 1.4 | `succeeded:stand` 2.324 s, 0.01 rad / 0.329 m | `failed:roll`, 0.52 rad / 0.139 m | `failed:roll`, 0.52 rad / 0.139 m |
| placed flank 1.2, free splay +0.8 | `failed:roll`, 0.52 rad / 0.139 m | `failed:roll`, 0.52 rad / 0.139 m | `failed:roll`, 0.52 rad / 0.139 m |

All four low-authority trials end *identically* — 0.52 rad / 0.139 m, the same
phase trace (`tuck → roll → failed:roll`) — including the two that succeed at
0.5. At 0.3 and 0.2 the drive is simply too weak to alter any pose, so both a
chest-down and a flank-lying robot converge on the same passive resting pose and
the ladder reports `failed:roll`, which is the honest verdict for "measured no
progress".

So the authority window is narrow and bounded on *both* sides: 1.0 over-drives
and inverts, 0.5 is the only setting that gets the robot up, and 0.3/0.2
under-drive into a no-op. One thing is worth keeping from the low end, though:
at that authority the ladder **cannot invert anything** — all four trials ended
`failed`, none `unrecoverable` — so its failure mode is "leave the robot where it
fell and say so", which is the right way to fail.


One demonstrated get-up (**4 of 4 repeats**, `succeeded:stand` at 2.236–2.408 s,
ending at 0.01 rad / 0.329 m on four separate ROS domains), a measured capture
envelope (**pitch 0.8–1.6 rad recoverable at ~2.3 s; nothing on the roll axis
above the 0.8 rad gate**), and a harness that can measure the sequence at all.
Both remaining open items are now measured rather than open:

- **Trigger latency is not the missing piece.** On lateral impulses the debounced
  latch is 0.66 s late *and* the fall is ballistic; on a sustained forward push
  the collapse is slow enough to be catchable (a 0.42 s ramp to 1.50 rad) and
  the latch is already on time (0.10 s after the threshold, at 1.1 rad of pitch,
  inside the capture window) — the trunk then flips because the 60 N force is
  still on. 0.42 s of catchable collapse against a 2.3 s get-up is an order of
  magnitude a trigger cannot close.
- **The operating point is measured, on both sides.** The recovery's authority
  (gain/damping 0.5/0.5) was never swept until now, and it matters more than the
  waypoints: at 1.0/1.0 the *working* pitch get-up inverts (3.14 rad / 0.057 m
  instead of standing at 0.329 m) and neither flank configuration improves. I then
  predicted the axis goes *lower* and tested it: at 0.3/0.3 and 0.2/0.2 all four
  placed poses end identically at 0.52 rad / 0.139 m with `failed:roll`, because
  the drive is too weak to alter any pose at all. So 0.5/0.5 is a narrow window
  bounded on both sides, and the two catapults this work had to slew away are the
  same over-drive failure mode.
- **The capture envelope is qualified, not sampled once**: pitch 0.9 / 1.2 / 1.4 /
  1.6 rad recover **13 of 13** runs (3/3/4/3, all ending at 0.01 rad / 0.329 m,
  success saturating at ~2.3 s), with 1.8 rad inverting. Combined poses recover
  too — pitch 1.2 + roll 1.0 included — so the envelope is a function of the
  *axis mix*: roll substitutes for pitch, the ladder wants ~1.2 rad of pitch, and
  a pose with ~1.0 rad of pitch and 0.9 of roll does not recover.

The feature stays **off by default**, and the reason is now measured rather than
precautionary: on every perturbation this harness can produce, the lateral
impulse ends **worse** with the recovery enabled (6 of 6 runs invert — five at
60 N and one at 50 N, at every start delay measured — to 3.14 rad / 0.057 m)
than with it disabled (3 of 3 settling at 0.52 rad / 0.139 m and staying
there). The delayed runs localise why: the rest is identical and passive in
both arms, and the inversion begins the moment the ladder engages. Its value is
real but confined: it stands the robot up from a settled chest-down pose and
holds it (4 of 4, 0.329 m), and on a placed flank it can hold a bounded
non-inverted rest (0.70–0.76 rad, `failed:stand`) — though a repeat of the
plain flank run inverted, and at 1.0 authority the flank inverts as well. What
it cannot do is recover a fall it did not choose. The open item now has a
number attached (see *What the closed-loop primitive would actually have to
do*): the legs carry 126.5 N when the ladder succeeds and 24 N when it stalls,
and closing that gap is the work.



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

### The entry phase, measured and then reverted (2026-09-29)

The ladder above always *entered* at the fold-all `tuck`. For one increment
`FallRecovery` was changed to key the entry on attitude instead: a
roll-dominant trunk carrying under 1.0 rad of pitch started at the `roll`
phase's braced push, the waypoint designed to push a trunk back over the pair
it rests on. That change is reverted, and the tooling that refuted it is
[`analyze_entry_phase.py`](analyze_entry_phase.py) →
[`entry_phase.json`](entry_phase.json).

The analyzer reads the entry from each trial's own `recovery_transitions`
record, so its grouping spans both eras and never depends on a trial's name.
Across the 104 recorded ladder trials 86 attempted a recovery: 6 entered at
`roll`, 80 at `tuck`. Per trial it reports the rest the attempt engaged from
(signed roll/pitch, per-foot load, and the roll *rate* over the 0.1 s before
the entry), the entry window's contact pattern, and whether the trunk reached
inverted tilt inside 1.5 s.

**The dispatch worked as written.** Three settled side rests (roll 0.512,
0.512, -0.520 rad at a roll rate of 0.0 rad/s) entered at `attempting:roll`
instead of `attempting:tuck`, and the placed pitch-1.4 control kept its
tuck-first ladder and still stood up (`succeeded:stand`, 0.329 m), so the one
working get-up was provably untouched.

**It did not achieve what it was changed for.** On a matched settled-rest
harness — placed roll 0.9 rad, 1.0 s start delay, no perturbation force,
identical manifests across all four runs — the attempt begins from the measured
rest at 0.512 rad / 0.0 rad/s. Two repeats per entry pose:

| entry pose | strike peak (sum) | loaded feet | inverted |
|---|---|---|---|
| braced push (`roll`) | 819.1 / 747.6 N | FR+RR (412 / 407) | 2 of 2 |
| fold (`tuck`) | 541.1 / 465.2 N | FR+RR (339 / 260) | 2 of 2 |

So the entry pose does not decide the outcome. The first phase cannot be fixed
by choosing a waypoint: every measured single-waypoint stroke on a settled side
rest is a 0.18–0.82 kN leg drive applied *while the trunk is still resting on
the legs*, and the trunk is ballistic after it (all four feet at 0.0 N). The
requirement stays with a closed-loop primitive, now measured from the entry
side: bound the stroke by the measured contact load, not by the waypoint. The
tuck entry is restored because it has the smaller stroke and the better
measured population: of the three traced delayed runs, the one that stopped
instead of inverting is a fold (1.72 rad, `failed:roll`,
[`fall_ladder_delayed_trace_20260928Td1.0/`](fall_ladder_delayed_trace_20260928Td1.0/probe.json)),
while all three settled braced entries inverted.

The rate, not the pose, is what separates a fall from a rest. The 60 N no-delay
family reads a roll close to a settled rest's (-0.53, -0.85 rad) but at
1.8–2.0 rad/s, so no entry pose is a rest for it; the settled rests read
0.0–0.05 rad/s. Two repeats were lost to this host's CycloneDDS range (ROS
domain 233 and up: a multicast port out of range); the harness kept their logs
and return codes but no `probe.json`, and the analyzer skips them rather than
reading an empty result.

#### The trunk's own ground load is now published and measured

Every limit above was inferred, because the trunk's contact force was not
published: the analyzer had to state that "whether the trunk itself was resting
on the ground during an entry is inferred from its roll rate, not measured."
`/go2/trunk_contact_forces` now carries it, measured by the same rule as the
foot forces (normal force against static world geoms only, so robot
self-contact never counts as support), and `probe_stance.py` records it per
trace point. The geom is `trunk_contact_0` — MuJoCo's URDF importer names
collision geoms `<link>_contact_<index>` and the *expanded xacro's* root link
is `trunk`, while the checked-in `go2_description.urdf` still says `base`. A
wrong name here is silent rather than fatal (the telemetry simply never
publishes), which is why a contract now pins the name against the xacro.

The first live trial on it
([`trunk_contact_20260929Troll0.9/`](trunk_contact_20260929Troll0.9/probe.json),
placed roll 0.9, 1.0 s start delay, 8 s, ROS domain 142, 2,001 trunk messages,
all 336 trace points populated) measures, on the *same* settled rest the entry
analysis above uses:

| moment | roll | trunk | feet (sum) |
|---|---|---|---|
| settled on its side, before the entry | 0.512 rad | **0.0 N** | 25.9 N |
| the entry's first trunk contact | -3.127 rad | **715.7 N** (peak) | 0.0 N |
| settled inverted, after the entry | 3.142 rad | 126.5 N | **0.0 N** |

This is the measurement the whole entry discussion was missing, and it is
sharper than the roll-rate inference it replaces. On a settled flank the trunk
carries **nothing** — at 0.512 rad the robot is resting on its *side*, on hip
and thigh geoms, so a primitive keyed on trunk load would see a true zero and
correctly do nothing. The trunk only loads once the entry has already rolled
the robot past 3.0 rad, and then it takes **126.5 N with all four feet at
0.0 N** — the entire weight on the trunk, with the legs contributing nothing.
That is the mechanism behind "the trunk is ballistic after the stroke", now
measured rather than argued, and it confirms the 997 N crouch transient as a
real hazard a contact-keyed rule must not fire on.

So the missing primitive is still *support creation*, and the number to beat is
now unambiguous: get the legs from ~0 N to carrying the trunk's 126.5 N **while
the trunk is still rolled over**, rather than after it. The contact signal a
closed-loop rule needs exists and is published; what is still missing is a way
to raise leg load without levering the trunk over.

#### What the two topics cover: a settled flank is not a support reading

`/go2/foot_contact_forces` and `/go2/trunk_contact_forces` are read as "how the
ground is holding the robot", but the trunk trial already showed that sum is
not a support reading on the flank: after its landing transient, the settled
side rest read 23–85 N of the 126.5 N robot, with the trunk at 0.0 N. What carried the rest was unmeasured,
and planning the entry's load bound against geoms the topics cannot see would
be planning against nothing. So
[`probe_support_attribution.py`](probe_support_attribution.py) partitions the
settled normal force over *every* colliding robot geom. It does not replay a
recorded pose — pinning the root would remove the very reaction being measured
— it drops the plant from 0.4 m at the rolls the ladder produced, settles for
4 s under the spawner's own joint-hold springs, and reports the equilibrium it
lands in. Every reported rest is checked: `qvel` is 0.000 at all seven, so
each row below is a static equilibrium, not a snapshot of motion.

Run output: [`support_attribution.log`](support_attribution.log); record:
[`support_attribution.json`](support_attribution.json).

| drop roll | contact | published | hidden | cover | resting on |
|---:|---:|---:|---:|---:|---|
| −0.50 rad | 126.5 N | **7.4 N** | 119.2 N | **5.8 %** | FL+RL hips (59.5 / 59.7 N) |
| 0.00 rad | 126.5 N | 126.5 N | 0.0 N | 100 % | the four feet (27.1 / 27.1 / 36.1 / 36.1 N) |
| 0.50 rad | 126.5 N | **7.3 N** | 119.2 N | **5.8 %** | FR+RR hips (59.2 / 60.0 N) |
| 0.90 rad | 126.5 N | **7.3 N** | 119.2 N | **5.8 %** | FR+RR hips (59.2 / 60.0 N) |
| 1.40 rad | 126.5 N | **7.4 N** | 119.1 N | **5.9 %** | FR+RR hips (59.2 / 60.0 N) |
| 2.40 rad | 126.5 N | 126.5 N | 0.0 N | 100 % | the trunk (126.5 N) |
| 3.14 rad | 126.5 N | 126.5 N | 0.0 N | 100 % | the trunk (126.5 N) |

The result is sharper than "on hip and thigh": the settled flank rests on
exactly **two geoms — the down-side hips** — at ~59.6 N each, and everything
else in contact is a rounding error (the two down-side feet at 3.7 N each; the
trunk at 0.0 N). The two topics publish 5 of the model's 18 colliding geoms;
the 13 they miss include both hips, thighs and calves of every leg and the imu
link, and on the flank those hidden geoms carry **94.2 % of the robot**. The
upright and inverted rests are the two cases the topics measure completely —
but those are the attitudes *after* the robot has rolled, not the one the entry
has to fix. The synthetic drop and the live trial differ in detail (the live
robot rocked through 23–85 N of feet while its trunk stayed at 0.0 N; the drop
settles flatter, at 7.4 N), and in neither does the feet+trunk sum ever hold the
whole robot.

So the requirement from the previous subsection gains a corollary, measured:
the support that has to move is the **hip load the topics do not publish**
(119.2 N on the flank), and a contact-keyed rule watching only feet and trunk
sees 7.4 N of it. The next entry primitive either measures the support it is
trying to move — which means publishing more than feet and trunk — or the
stroke stays open-loop. The number to beat is unchanged; where the primitive
must measure it is now known.

The contracts are
[`test_r5_2_go2_entry_phase.py`](../../../../src/robot_lab_adapter/test/test_r5_2_go2_entry_phase.py)
(10 tests) and
[`test_r5_2_go2_support_attribution.py`](../../../../src/robot_lab_adapter/test/test_r5_2_go2_support_attribution.py)
(5 tests); both are in the CI fast tier, and the R5.2 Go2 selection passes
(168 passed, 0 skipped).
