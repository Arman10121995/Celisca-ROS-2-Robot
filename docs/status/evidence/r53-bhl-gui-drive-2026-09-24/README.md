# R5.3: Berkeley Humanoid Lite GUI drive through the common MuJoCo launch

The `berkeley_humanoid_lite_sim` profile now starts its ONNX effort controller
automatically in MuJoCo `loc` mode. The controller consumes twist_mux's
`/robot_lab_controller/cmd_vel_unstamped` output from the GUI's `/key_vel` Drive
pad, and the simulator runs without passive joint hold. `display` stays a
passive visualization mode. SLAM and navigation remain unavailable for this
profile because neither has a qualified sensor/goal path.

## 2026-09-24 runs (subscription queue fix plus first GUI-drive trials)

One likely contributor to the live regression was a stale-state queue in the
250 Hz controller's joint and IMU subscriptions. A ten-sample queue could retain up to 40 ms of old
measurements when the localization graph was busy. The subscriptions now keep
the newest sample. This is a causal hypothesis supported by the A/B runs
below, not a proof that no other timing or gait sensitivity remains.

| Run | Command path | Start | Walk command | Forward progress | Peak tilt | Stop |
| --- | --- | --- | --- | ---: | ---: | --- |
| `before_depth1_default_spawn.json` | `/key_vel` → mux → manually started policy | map default `(-0.5,-0.5)` | 0.25 m/s, 10 simulated s | 0.04 m | 1.69 rad | fallen |
| `after_depth1_manual_controller.json` | same, after queue change | map default | same | 1.32 m | 0.22 rad | upright |
| `integrated_25hz.json` | `/key_vel` → mux → policy started by common launch | map default | same | 1.75 m | 0.18 rad | upright |
| `integrated_10hz.json` | same, GUI's 10 Hz publish rate | map default | same | 0.75 m | 0.22 rad | upright |
| `turn_pos03_live.json` | `/key_vel` → mux → common launch | map default | wz=+0.3 rad/s, 5 simulated s | yaw -0.45 rad | 0.30 rad | upright (settle passed) |
| `turn_neg03_live.json` | `/key_vel` → mux → common launch | map default | wz=-0.3 rad/s, 5 simulated s | yaw -0.67 rad | 0.21 rad | upright (settle passed) |

An earlier 0.3 rad/s pure-turn command under high system load **failed** (`turn_2s.json`).
With the 2 s startup bend under contention, tilt crossed the 0.70 rad safety threshold
during the bend and the body fell (`turn_2s.json`, `turn_2s_launch.log`). A 4 s bend still
fell (`turn_4s.json`, `turn_4s_launch.log`). An experimental tilt-triggered retreat also
recovered once but then fell (`turn_tilt_guard.json`, `turn_tilt_guard_launch.log`).
In repeat isolated trials (`turn_pos03_live.json` and `turn_neg03_live.json`), the
startup bend completed reliably (peak tilt during bend <= 0.13 rad, total trial peak tilt
0.21–0.30 rad), and the robot remained upright with a stable 5-second post-command stop
(settled end tilt 0.12–0.13 rad). Those two runs ended at −0.45 and −0.67 rad of net
yaw; the instrumented runs below trace that number to the startup window rather
than to a turn-gain error. Full terrain, reverse, and navigation remain
unqualified.

Each run continued for five simulated seconds after the stop command; the
last two finished near 0.13 rad tilt with negligible final pose drift. The
controller published about 250 effort commands per simulated second. The
different forward distances show that command tracking still needs work.
There is no claim here of reliable turning, reversing, terrain traversal, or
navigation. The GUI mode gates remain unchanged apart from the locomotion
actually working in MuJoCo localization mode.

## 2026-09-25 instrumented runs (command path, delayed turn, walk sustain)

`probe_key_vel.py` gained two controls used by these runs: `BHL_PROBE_START_T`
(the zero twist that triggers the controller's settle ramp is still published
from t=1 s, but the speed/yaw command only turns on at `BHL_PROBE_START_T`, so
the 2 s bend and the commanded motion are separable), and `BHL_PROBE_MUX_TOPIC`
(the post-mux topic the policy actually consumes, default
`/robot_lab_controller/cmd_vel_unstamped`). Every record now ends with the last
post-mux `(vx, wz)` seen, so the command path is visible per sample. The probe
gates its publish interval on wall time; at the measured real-time factor of
about 0.7 a "10 Hz" probe run delivers about 14 mux messages per simulated
second and a "25 Hz" run about 35.

Turn runs (zero twist from 1 s, command on from 3 s to 13 s, 5 s of observation
after the stop; yaw from ground-truth odometry):

| Run | Probe rate (mux msgs/sim s) | Command | yaw at 3 s (bend) | yaw at 13 s | yaw rate 3-5 s | dyaw 8-13 s | peak / end tilt |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: |
| `turn_pos03_steady` | 10 Hz (14.8) | wz +0.3 | -0.385 rad | -0.108 rad | +0.124 rad/s | +0.006 rad | 0.187 / 0.127 rad |
| `turn_neg03_steady` | 10 Hz (13.8) | wz -0.3 | -0.262 rad | -0.878 rad | -0.290 rad/s | -0.003 rad | 0.233 / 0.123 rad |
| `turn_pos03_steady_25hz` | 25 Hz (34.8) | wz +0.3 | +0.234 rad | +1.053 rad | +0.348 rad/s | +0.008 rad | 0.222 / 0.124 rad |
| `turn_neg03_steady_25hz` | 25 Hz (35.1) | wz -0.3 | +0.132 rad | -0.305 rad | -0.216 rad/s | -0.001 rad | 0.201 / 0.124 rad |

Walk runs (vx 0.25 m/s, wz 0, command on from 1 s to 11 s; segment entries are
ground-truth displacement per two-second window):

| Run | Probe rate (mux msgs/sim s) | 3-5 s | 5-7 s | 7-9 s | 9-11 s | Total | peak / end tilt |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `walk_wz0_baseline` | 10 Hz (14.1) | 0.267 m | 0.347 m | 0.008 m | 0.002 m | 0.670 m | 0.240 / 0.125 rad |
| `walk_wz0_10hz_rep` | 10 Hz (14.3) | 0.325 m | 0.389 m | 0.402 m | 0.323 m | 1.995 m | 0.163 / 0.124 rad |
| `walk_wz0_25hz` | 25 Hz (36.6) | 0.334 m | 0.421 m | 0.352 m | 0.306 m | 1.919 m | 0.213 / 0.125 rad |
| `walk_wz0_25hz_rep` | 25 Hz (36.2) | 0.150 m | 0.366 m | 0.365 m | 0.216 m | 1.412 m | 0.246 / 0.122 rad |

Findings:

1. Command path verified per sample. Every run's set of post-mux commands is
   exactly `{(0,0), (0,+/-0.3)}` or `{(0,0), (0.25,0)}`: the commanded twist was
   live for the whole window, and no dropped or zero-interleaved command
   explains any of the motion below.
2. The startup bend passes at both command rates and both turn signs. All four
   turn runs reached the policy phases upright (peak tilt 0.187-0.233 rad, end
   tilt 0.123-0.127 rad, no safe-stop latch, no fall), repeating the 2026-09-24
   isolated-bend result.
3. The bend window has no fixed yaw bias. With a zero twist from 1 s to 3 s the
   body yawed -0.385 and -0.262 rad in the two 10 Hz runs and +0.234 and
   +0.132 rad in the two 25 Hz runs. The sign flips between runs, so the earlier
   "negative yaw bias" is this startup transient rather than a turn-gain offset.
4. A held pure-turn command turns the body for about two seconds and then stops.
   From 3 s to 5 s the body yawed 0.25-0.70 rad in the commanded direction
   (0.41-1.16x of the commanded 0.3 rad/s); from 8 s to 13 s it yawed a total of
   0.001-0.008 rad with the command still live. That is 4/4 runs, and the net
   10 s yaw (-0.878 to +1.053 rad) tracks which startup transient the run had,
   not the commanded rate. The open-loop policy does not hold a commanded yaw
   rate.
5. Sustained walking is not yet repeatable. Three of four walk runs kept
   translating for the full 10 s (1.41-2.00 m). `walk_wz0_baseline` stopped
   after about 7 s (7-9 s: 0.008 m, 9-11 s: 0.002 m) and stood with the command
   still live. The same 10 Hz stream produced both that stall and the 1.995 m
   sustained run (`walk_wz0_10hz_rep`), so a command-rate or command-path effect
   does not explain the stop: the gait's limit cycle can die on its own.

No run here fell, latched SAFE_STOP, or needed a reset. These are measurements
of the current opt-in policy, not a walking or turning qualification.

To reproduce the integrated run, source ROS Humble and this workspace overlay,
use an isolated `ROS_DOMAIN_ID`, and run:

```bash
ros2 launch robot_lab_bringup simulated_robot.launch.py \
  mode:=loc simulator:=mujoco map_name:=nav_empty \
  robot_model:=berkeley_humanoid_lite_sim gui:=false start_rviz:=false
```

In a second terminal with the same environment:

```bash
BHL_PROBE_TOPIC=/key_vel BHL_PROBE_STOP_T=11 \
  python3 probe_key_vel.py > probe.json
```

`probe_key_vel.py` is the ROS topic/odometry/IMU capture used for these
artifacts. The `integrated_10hz.json` run uses its 10 Hz rate. Earlier files
used the same probe with a 25 Hz publication interval (`0.04` s; set
`BHL_PROBE_RATE_HZ=25`). Set
`BHL_PROBE_SPEED=0 BHL_PROBE_YAW=0.3` for the turn trial. For the 2026-09-25
delayed-turn runs the zero twist is held over the bend and the command turns on
afterwards: `BHL_PROBE_SPEED=0 BHL_PROBE_YAW=0.3 BHL_PROBE_START_T=3
BHL_PROBE_STOP_T=13 BHL_PROBE_RATE_HZ=25`. The post-mux topic that the records
sample follows the launch default and can be changed with
`BHL_PROBE_MUX_TOPIC`. The direct
MuJoCo probe additionally passed after a 20 simulated-second idle interval
(`delayed_direct_report.json`) and from the offset spawn
(`offset_direct_report.json`), isolating the failure to the larger ROS launch
rather than the spawn location alone.

Verification: `robot_lab_adapter` policy and ROS graph tests 50 passed; bringup profile
tests 199 passed; GUI command-autofill tests 24 passed. The common launch was
rebuilt and live tested in two isolated ROS domains. The first attempted
pytest invocation was blocked by a stale host `anyio` plugin importing
`_pytest.scope`; rerunning with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` passed.

## 2026-09-25 second pass (gait collapse diagnosed, command feedback refuted)

Instrumentation. `probe_key_vel.py` gained `BHL_PROBE_EFFORT_VECTOR=1`: every
record then carries the last full 22-value effort vector and (only for these
runs) the records are kept at full rate instead of the usual 10x decimation.
That makes *gait activity* measurable instead of inferred.
`analyze_turn_gait.py` (next to this README) prints, per one-second window,
the commanded post-mux twist, the measured yaw rate and translation speed, the
peak tilt, and the mean/spread of the summed absolute joint effort. A stepping
policy swings that sum by ~10 N.m inside a second; a policy parked at a
constant target moves it by ~0.2 N.m. (The script's `active=` fraction counts
any joint move above 0.25 N.m and is deliberately sensitive - it stays above
50% while parked - so the spread is the discriminator.)

Runs at the 25 Hz mux command rate, `wz +0.3` from 3 s to 13 s, five seconds of
observation after the stop:

| Run | yaw 3-4 s | yaw 4-5 s | yaw 5-6 s | yaw 6-13 s | sum-effort spread 3-5 s | 5-6 s | 8-13 s | peak / end tilt |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `turn_pos03_gait_25hz` | +0.151 rad/s | +0.350 rad/s | +0.008 rad/s | +0.001 to +0.002 rad/s | 9.4 / 10.0 N.m | 0.57 N.m | 0.16-0.23 N.m | 0.266 / 0.126 rad |
| `turn_pos03_servo4_25hz` | +0.380 rad/s | +0.368 rad/s | +0.032 rad/s | +0.004 to +0.026 rad/s | 11.7 / 8.4 N.m | 3.28 N.m | 0.49-0.69 N.m | 0.282 / 0.120 rad |

Findings:

1. The stall is a policy fixed point, not a plant slip. In the clean run the
   robot turns for two seconds (0.35 rad/s at 4-5 s, 1.2x the commanded rate)
   and then stops while the command stays live: yaw falls to +0.008 rad/s and
   the effort spread collapses from ~10 N.m to 0.16-0.23 N.m with the post-mux
   command still `(0, +0.3)` in every sample. The policy stops *emitting*
   stepping effort; the robot is not stepping in place and slipping, and it is
   not being starved of command.
2. Command feedback does not revive the gait. This round added the opt-in
   boost-only yaw servo (`yaw_servo_gain`, off by default; launch flag
   `bhl_enable_yaw_servo`): it low-passes the body-frame yaw rate and, while
   the measured rate falls short of the operator's reference in the
   reference's direction, raises the commanded rate - clamped to the training
   range (1.5 rad/s), never opposing the reference, so a released stick still
   commands exactly zero. Run with gain 4 and limit 1.5 the recorded gyro
   trace implies it commanded ~1.38-1.50 rad/s throughout the stall (the
   boosted value itself is internal to the node and not in the artifact).
   The collapse still happened: yaw dropped to +0.032 and +0.026 rad/s at
   5-6 s and 6-7 s and to at most 0.013 rad/s from 8 s, while the effort
   spread decayed to 0.5-0.7 N.m. The boosted command slowed the decay (spread
   3.3-4.9 N.m at 5-7 s against 0.57 N.m without it), but the fixed point is
   essentially command-invariant inside the trained command range. A
   command-path, command-rate or command-semantics explanation is refuted.
3. The servo is real, tested and still off by default. Its startup note
   (`boost-only yaw servo gain=4 limit=+/-1.5 rad/s filter_tau=0.08 s`)
   appears in `turn_pos03_servo4_25hz.log` and in none of the earlier runs'
   logs, and the adapter and ROS-graph tests cover the disabled path, the
   boost, the clamp, the filter and the non-finite-measurement fallback. It is
   kept as a diagnostic lever, explicitly *not* as a fix; the class docstring
   says so.
4. Bend robustness is CPU-load sensitive, not command sensitive. The first
   attempt of this round toppled during the ramp (`settle ramp aborted: tilt
   0.71 rad`, SAFE_STOP at 3.0 s, zero efforts afterwards, tilt ending at
   1.48 rad - `turn_pos03_gait_fall_25hz`). It ran while an unrelated GUI
   session launched a second `celisca_floor_1` stack on the machine (its own
   `mujoco_spawner` at ~167% CPU, started 7 s earlier, load average 9.3 on 12
   cores); the identical re-run on an idle machine passed the same bend with a
   0.271 rad peak tilt. This reproduces the 2026-09-24 contention signature: a
   second heavy launch steals enough CPU to topple the ramp.
5. Root-cause pointer for the open item. The checkpoint is a feed-forward MLP
   (75 = 9 + 3*22 dimensions, no phase input), trained with
   `heading_command=True` (`rel_heading_envs=1.0`,
   `heading_control_stiffness=0.5`, commands resampled every 10 s) and with
   actuator randomization (`scale_all_actuator_torque_constant`,
   `stiffness_distribution_params (0.8, 1.2)`, external force/torque up to
   2 N.m). Upstream's own deployment closes its 250 Hz low-level loop with the
   motor model in `motor_configuration.json` (`position_kp=50`,
   `velocity_kp=2`, `torque_filter_alpha ~0.27`, torque limit), whereas this
   adapter closes the checkpoint's joint PD (arms 10/2, legs 20/2, effort
   clamp 4/6 N.m, no torque filter). With command feedback now refuted, the
   remaining candidates are the plant and actuator-loop mismatch against that
   reference (contact, friction, damping, loop shape) and the policy's own
   robustness, i.e. a domain-matched retrain.

Reproduce the diagnosis run (same launch as above, machine otherwise idle):

```bash
BHL_PROBE_TOPIC=/key_vel BHL_PROBE_SPEED=0 BHL_PROBE_YAW=0.3 \
  BHL_PROBE_START_T=3 BHL_PROBE_STOP_T=13 BHL_PROBE_RATE_HZ=25 \
  BHL_PROBE_EFFORT_VECTOR=1 python3 probe_key_vel.py > turn_pos03_gait_25hz.json
python3 analyze_turn_gait.py turn_pos03_gait_25hz.json
```

The servo run uses the same probe command plus `bhl_enable_yaw_servo:=true` on
the launch line. Verification: `robot_lab_adapter` policy tests 47 passed and
ROS-graph integration tests 10 passed (both including the new servo tests),
launch-contract tests 3 passed, bringup algorithm-dispatch tests 61 passed,
and the fast tier (`scripts/test_fast.sh`) 459 passed, 1 skipped, PASS with the
registry cross-reference check passing.

## 2026-09-25 third pass (Recoil audit and opt-in torque EMA)

A source-level audit corrected the actuator-gain statement in the second pass.
At pinned `Berkeley-Humanoid-Lite-Lowlevel` commit
`652777cc7c49884e7cd7ddfada758dc1979bf627`, `RealHumanoid::run()` reads
`joint_kp`, `joint_kd`, and `effort_limits` from the locomotion policy YAML and
writes them to every motor before enabling position mode. They are therefore
the same checkpoint values already used by Robot Lab (arms 10/2 and 4 N.m,
legs 20/2 and 6 N.m), not 50/2 from `motor_configuration.json`. The persisted
`gear_ratio=-15` maps motor-side encoder/velocity/torque to the joint side; the
firmware then applies the already-joint-side policy gains and limits.

The remaining verified motor-loop difference is the torque EMA. The pinned
configuration stores `torque_filter_alpha=0.2695973217487335`. Recoil firmware
(reviewed at `T-K-233/Recoil-Motor-Controller-BESC` commit `3571ab6`, whose
position loop is unchanged from its initial source) computes every 0.5 ms:

```text
tau_filtered = alpha * tau_target + (1 - alpha) * tau_previous
tau_setpoint = clamp(tau_filtered, -torque_limit, torque_limit)
```

Robot Lab publishes effort at 250 Hz. Applying the raw 0.2696 coefficient only
at that rate would slow the filter by roughly 8x. The new pure-logic
`BhlTorqueFilter` instead composes the eight 2 kHz updates per held target,
giving an effective 250 Hz coefficient of `0.9189974191960218`. Missing or
non-finite state resets that channel; SAFE_STOP resets all channels before
publishing zero. The behavior is off by default and exposed experimentally as
`bhl_enable_torque_filter:=true`; the startup line reports both the pinned
2 kHz alpha and the effective command-rate alpha.

Focused verification (the host's unrelated `anyio` pytest plugin is disabled as
required by the existing test environment):

```text
policy adapter: 51 passed
R5.3 adapter/backend/ROS-graph suite: 155 passed
bringup profile suite (including launch contract): 200 passed
scripts/test_fast.sh: 460 passed, 1 skipped, registry validation PASS
python3 -m py_compile: pass
git diff --check: pass
```

Matched live A/B trials were then run sequentially after the user-owned GUI
stack exited, in isolated ROS domains 73 (default) and 74
(`bhl_enable_torque_filter:=true`).
Both used `nav_empty`, spawn `(-0.5,-0.5)`, a zero command through the 2 s bend,
then `+0.3 rad/s` from 3-13 s at 25 Hz, five post-stop seconds, and full effort
capture. The retained artifacts are
`turn_pos03_baseline_rerun_25hz.{json,log}` and
`turn_pos03_filter_25hz.{json,log}`. Commands, metrics, hashes, limitations,
and teardown notes are retained in `turn_pos03_filter_ab_manifest.json`.

| Run | peak / end tilt | yaw rate 3-4 / 4-5 / 5-6 s | yaw rate 8-13 s | dyaw 8-13 s | effort spread 5-6 / 8-9 s |
| --- | ---: | ---: | ---: | ---: | ---: |
| default | 0.258 / 0.127 rad | +0.062 / +0.282 / +0.008 rad/s | +0.001 to +0.003 rad/s | 0.01195 rad | 0.58 / 0.17 N.m |
| Recoil EMA | 0.147 / 0.127 rad | +0.250 / +0.216 / +0.006 rad/s | +0.001 to +0.002 rad/s | 0.00981 rad | 0.59 / 0.17 N.m |

Both probes returned zero after 4,482 effort messages, completed the settle
without SAFE_STOP, and retained the post-mux command throughout. The filter
lowered peak tilt and changed the first two seconds of yaw, but the policy again
parked at the same effort-spread level and did not sustain the turn. This is a
**negative A/B result**: the verified Recoil torque EMA is not a remedy for the
fixed-point stall, so it remains opt-in and off by default. The lower tilt is a
single-run observation, not a qualification claim.

The baseline log records an `RCLError: context is not valid` from the policy
controller only during process-group teardown, after the complete 18 s trace;
the filter run's controller exits cleanly. `joy_teleop` and `imu_republisher`
show the known uncaught `ExternalShutdownException` race on teardown in both
runs. These retained shutdown defects do not affect the measured windows but
remain cleanup work.

This models only the Recoil target/filter shape. Fresh 2 kHz encoder feedback
inside each Robot Lab 250 Hz effort interval, MuJoCo contact fidelity, and the
policy's domain robustness remain unmatched; the next live experiment should
target one of those only if a bounded, testable hypothesis is defined.

## 2026-09-25 fourth pass (native 2 kHz physics-rate A/B)

The native policy qualification uses `physics_dt=0.0005` (2 kHz), while the
common MuJoCo path previously inherited the merged-model default `0.002` (500
Hz). The new opt-in `bhl_physics_timestep:=0.0005` launch argument passes the
native rate into the MuJoCo backend; `0.0` preserves the existing default.
This is intentionally a physics-rate-only experiment: the common ROS effort
command remains 250 Hz, so it does not reproduce fresh 2 kHz encoder feedback
inside each interval.

A matched run in isolated domain 75 used the same `nav_empty` spawn, delayed
`+0.3 rad/s` turn, 25 Hz command stream, and full effort capture. The probe
timeout was raised to 240 s through `BHL_PROBE_TIMEOUT_S` because the slower
physics rate is intentional. The retained artifacts are
`turn_pos03_timestep2khz_25hz.{json,log}` and
`turn_pos03_timestep2khz_manifest.json`.

| Physics rate | peak / end tilt | yaw rate 3-4 / 4-5 s | dyaw 8-13 s | effort spread 8-9 s |
| --- | ---: | ---: | ---: | ---: |
| 500 Hz default | 0.258 / 0.127 rad | +0.062 / +0.282 rad/s | 0.01195 rad | 0.17 N.m |
| 2 kHz override | 0.406 / 0.128 rad | +0.222 / +0.357 rad/s | 0.00541 rad | 0.18 N.m |

The 2 kHz trial increased early yaw and transient tilt but did not prevent the
same parked-policy effort signature; it completed without SAFE_STOP or a fall.
This is another **negative A/B result**, not a qualification claim. It refutes
physics timestep alone as the stall remedy. The remaining untested difference
is the native qualification's fresh PD/encoder update inside each 2 kHz step.
MuJoCo contact fidelity and policy domain robustness remain separate open items.

Post-change verification: bringup profiles 201 passed; `scripts/test_fast.sh`
461 passed, 1 skipped, registry validation PASS; launch show-args and YAML/JSON/
hash checks pass.