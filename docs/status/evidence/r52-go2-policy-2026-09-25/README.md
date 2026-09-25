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
threshold has been measured.



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
