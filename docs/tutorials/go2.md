# Go2 flat-ground policy: bounded runtime evidence

This tutorial exercises the opt-in Go2 flat-ground ONNX policy through the
standard MuJoCo ROS route. It is a controlled evidence walkthrough, not a
claim that Go2 terrain traversal, navigation or the GUI velocity-base mode is
qualified.

The recorded evidence is under
[`docs/status/evidence/r52-go2-policy-2026-09-25/`](../status/evidence/r52-go2-policy-2026-09-25/README.md).
At the current source snapshot, the policy has produced measured forward
motion, a stop, a large turn, command-loss stop, feed-forward/inverse reverse A/Bs
and two five-case inverse-map flat-ground screening suites. The suites passed all
bounded checks, but the named stairs task failed at the first ledge; low-speed
tracking, terrain, fall recovery and navigation remain open.

## Prerequisites and safety

Use Ubuntu 22.04/ROS 2 Humble or the documented development environment, a
built workspace, MuJoCo and ONNX Runtime. This is a simulation tutorial only.
It must not be used to command hardware. Use an unused `ROS_DOMAIN_ID`, do not
share a simulator process with another run, and stop only processes started by
this workflow.

From the workspace root:

```bash
cd /home/molar1/bumperbot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=78
```

The policy is deliberately opt-in. The normal Go2 stance controller remains
the default. A GUI checkbox is available for the Go2/MuJoCo localization
profile and fills the same launch argument shown below.

## Run a stance and forward/stop trial

Start the probe in one terminal before the launch when startup samples are
required. The probe prints JSON to stdout; redirect it to a file rather than
passing an unsupported output option. The policy launch is run in a second
terminal with the same environment.

```bash
# Terminal 1: start the probe and leave it running.
python3 docs/status/evidence/r52-go2-2026-09-25/probe_stance.py \
  --duration 7 --drive-vx 0.25 --drive-start 1 --drive-end 4 \
  > /tmp/go2-forward.json 2> /tmp/go2-forward.log
```

```bash
# Terminal 2: start the opt-in policy launch in the same ROS_DOMAIN_ID.
ros2 launch robot_lab_bringup simulated_robot.launch.py \
  mode:=loc simulator:=mujoco robot_model:=unitree_go2 \
  map_name:=nav_empty gui:=false start_rviz:=false \
  go2_policy_path:=auto
```

Record the process exit, output JSON/log, initialization pose, command window,
tilt, displacement, effort and contact samples. The recorded forward trial
moved about 0.805 m during the command and settled within about 0.004 m after
the stop window. Those numbers describe that host/trial, not a guarantee.

## Run the turn and command-loss checks

Repeat the same launch with a bounded yaw command. For command loss, add
`--drop-command-after-drive` to the probe so it stops publishing after the
drive window. Keep the initialization, map, policy path, physics step and probe
windows fixed when comparing trials.

```bash
# Use --drop-command-after-drive for the command-loss variant.
python3 docs/status/evidence/r52-go2-2026-09-25/probe_stance.py \
  --duration 7 --drive-wz 0.5 --drive-start 1 --drive-end 4 \
  --drop-command-after-drive \
  > /tmp/go2-turn-command-loss.json 2> /tmp/go2-turn-command-loss.log
```

For command loss, stop the publisher during the drive window and verify that
the controller's watchdog produces a bounded stop. Do not call the trial a
success merely because the process remains alive. Record yaw change, XY drift,
maximum tilt, maximum effort and the direct `/go2/foot_contact_forces` trace.

## Calibrate reverse motion

The first low-speed reverse request showed a dead zone. The current evidence
uses feedforward compensation that maps a requested `-0.25 m/s` to a more
active policy command. Treat this as a calibration parameter, not as a
validated reverse controller.

```bash
python3 docs/status/evidence/r52-go2-2026-09-25/probe_stance.py \
  --duration 7 --drive-vx -0.25 --drive-start 1 --drive-end 4 \
  > /tmp/go2-reverse.json 2> /tmp/go2-reverse.log
```

For the next R5.2 iteration, run the reproducible suite first; it records raw
probe JSON/logs and a summary instead of relying on one hand-copied trial. The
five-case runner also includes command loss and zero-command settle:

```bash
docs/status/evidence/r52-go2-policy-2026-09-25/run_flat_ground_suite.sh \
  --out-dir /tmp/go2-flat-suite \
  --domain-base 211 \
  --map inverse
```

The recorded two-run flat-ground comparison is:

| case | first drive ΔX | repeat drive ΔX | first/repeat peak tilt |
|---|---:|---:|---:|
| forward +0.25 m/s | +0.832 m | +0.836 m | 0.034 / 0.031 rad |
| reverse -0.35 m/s | -1.065 m | -1.111 m | 0.078 / 0.078 rad |
| turn +0.5 rad/s | +1.496 rad | +1.481 rad | 0.059 / 0.056 rad |
| forward command loss | +0.797 m | +0.802 m | 0.032 / 0.041 rad |
| zero command | 0.000 m | 0.000 m | 0.030 / 0.030 rad |

Both suites passed all five bounded screening checks, with 1,750–1,752 direct
foot-contact messages per trial and clean process exits. This supports repeatable
bounded screening, not velocity-tracking qualification. The raw records are under
[`flat_ground_suite_20260925/`](../status/evidence/r52-go2-policy-2026-09-25/flat_ground_suite_20260925/summary.json)
and [`flat_ground_suite_20260925_repeat/`](../status/evidence/r52-go2-policy-2026-09-25/flat_ground_suite_20260925_repeat/summary.json).

The opt-in inverse-map calibration remains a deadband-limited candidate: the
recorded inverse sweep improves the -0.25 to -0.45 m/s grid but leaves -0.15 m/s
inside its deadband. Run the candidate explicitly with `--map inverse`; do not
change the default or claim velocity tracking from displacement alone.

## Named terrain and fall handling

The current inverse-map named terrain trial is
[`terrain_stairs_inverse_20260925.json`](../status/evidence/r52-go2-policy-2026-09-25/terrain_stairs_inverse_20260925.json).
It starts at the recorded first-ledge spawn, commands +0.5 m/s from 1–8 s, and
records 0.115 m drive displacement, a 0.35 rad tilt-warning crossing at 3.212 s,
0.527 rad peak tilt and 35.55 N.m maximum effort. The controller later reports
attitude recovery and the process exits cleanly, but the task fails at the first
ledge. Do not relabel this as a successful terrain traversal.

Bounded perturbation recovery is now measured. A diagnostic body-frame force
pulse is opt-in through `go2_perturbation_force_n`, `go2_perturbation_start_s`,
`go2_perturbation_duration_s` and `go2_perturbation_axis`, and the controller
republishes its latched safety state on `/go2/safety_state` so the probe records
real transitions:

```bash
ROS_DOMAIN_ID=231 FORCE_N=35.0 \
  docs/status/evidence/r52-go2-policy-2026-09-25/run_perturbation_trial.sh \
  /tmp/go2-perturbation
```

The recorded sweep used a 0.2 s lateral pulse at 3.0 s with a 2.0 s recovery
window:

| pulse | peak tilt | final height | safety state | screening |
|---:|---:|---:|---|---|
| 5 N | 0.030 rad | 0.368 m | `nominal` | pass, below noise floor |
| 20 N | 0.030 rad | 0.368 m | `nominal` | pass, below noise floor |
| 35 N | 0.070 rad | 0.373 m | `nominal` | pass, measurable and bounded |
| 60 N | 0.717 rad | 0.139 m | `safe_stop` | fail, collapse |

Read this carefully. The stance baseline is about 0.030 rad, so 5 N and 20 N are
**negative controls**: their response is not distinguishable from noise and must
not be cited as stability. 35 N is the smallest tested magnitude with a clearly
measurable response, and the robot settles upright. 60 N exceeds the envelope:
the robot collapses and the fail-safe latches correctly, which is a fall and not
a recovery.

## Fall detection and the re-stand attempt

Fall detection is always on and latches a `fallen` flag that is distinct from
`safe_stop`. It is debounced so a single tilt spike cannot report a fall, and it
clears only on an explicit safety reset. After a trip above 0.70 rad, 25
consecutive warning-or-higher samples latch `fallen`; `/go2/fallen` and
`/go2/recovery_state` show the actual transition.

The re-stand attempt is **opt-in and off by default**:

```bash
ros2 launch robot_lab_bringup simulated_robot.launch.py \
  mode:=loc simulator:=mujoco robot_model:=unitree_go2 \
  map_name:=nav_empty go2_policy_path:=auto \
  enable_fall_recovery:=true fall_recovery_timeout_s:=8.0
```

It drives a **bounded ladder of joint-space waypoints** rather than a single
pose, because the traces show what a single pose does from a fallen trunk: the
stance drive levers the trunk over its feet and ends inverted at 0.057 m.

| phase | waypoint (hip / thigh / calf per leg) | purpose |
|---|---|---|
| `tuck` | 0.00 / 1.35 / -2.70 | fold the legs in: retract the lever that flips a down trunk |
| `roll` | loaded pair 0.00 / 0.10 / -0.90 | brace the legs the trunk is measured to rest on, retract the others |
| `crouch` | 0.00 / 1.10 / -2.20 | feet under the hips at a low standing height |
| `stand` | nominal stance | the standing pose, and only then |

Every transition reads measured attitude, each phase is time-bounded (0.4 / 0.7
/ 0.6 s), the roll count is bounded (2), and the standing pose is commanded only
from `stand`, which is entered only once measured tilt is under the 0.8 rad
gate. `/go2/recovery_state` publishes the phase with the status
(`attempting:tuck`, `attempting:roll`, ...) so a trial trace shows where the
attempt is. Success still requires measured tilt below 0.35 rad, 0.25 m
ground-truth body height, three loaded feet and a 0.5 s dwell; an unmeasured
attitude holds the phase instead of guessing one.

In the 60 N repeat the ladder behaved as designed but the fall was already
ballistic ([`fall_ladder_20260928T60N/`](../status/evidence/r52-go2-policy-2026-09-25/fall_ladder_20260928T60N/probe.json),
domain 230, 14 s probe, 8 s window, no learned actor):

| measured transition/result | value |
|---|---|
| `fallen: false → true` | 3.864 s |
| `recovery_state` | `idle → attempting:tuck` (3.864 s) → `attempting:roll` (4.176 s) → `unrecoverable:roll` (4.800 s) |
| retraction, RR calf | -1.183 rad → -2.700 rad within 0.1 s of the latch |
| tilt while the ladder ran | 0.652 → 1.042 → 1.683 → 1.931 → 2.170 → 3.041 rad (~3.6 rad/s) |
| peak tilt / final height | 3.1416 rad / 0.057 m |
| commanded effort after the verdict | 0.0 N·m |

The ladder's guarantee held — the standing pose was never driven from a trunk
past the gate, and the attempt stopped 0.94 s after it began, 5.8 s earlier than
the old single-phase attempt — but it did not prevent the inversion, because the
debounced `fallen` latch fires 0.66 s after the pulse when the trunk is already
at 1.042 rad and rolling at ~3.6 rad/s. A matched 45 N control
([`fall_ladder_20260928T45N/`](../status/evidence/r52-go2-policy-2026-09-25/fall_ladder_20260928T45N/probe.json))
never topples at all (0.1485 rad peak tilt, no `safe_stop`, `fallen` never
latched, `recovery_state` stayed `idle`).

A 21-trial sweep
([`ladder_sweep.json`](../status/evidence/r52-go2-policy-2026-09-25/ladder_sweep.json),
reduced by `analyze_ladder_sweep.py`) widened that gap and closed it: lateral
impulses 45–60 N, sustained lateral 40–50 N over 1.2–1.5 s, forward shoves up to
70 N and drops from 0.9/1.4 m. Everything either stays upright or rolls fully
over; every toppling case latches `fallen` at 0.51–1.16 rad and 2.6–7.2 rad/s.
There is no perturbation on these maps that leaves the robot down but not
inverted, so the *trigger* — not the sequence — is what limits a fall trial.

To measure the sequence itself, spawn the robot already down. `spawn_pitch` and
`spawn_roll` place a settled fallen pose (both default to `0.0`):

```bash
# 1.4 rad nose-down, then the ladder: measured stand at 0.33 m, held to 20 s
FORCE_N=0.0 SPAWN_PITCH=1.4 FALL_RECOVERY=true \
  FALL_RECOVERY_TIMEOUT_S=8.0 DURATION_S=20 \
  ../status/evidence/r52-go2-policy-2026-09-25/run_perturbation_trial.sh \
  /tmp/placed_pitch
```

From that pose the ladder walks `tuck → roll → crouch → stand`, reports
`succeeded:stand` at **2.324 s** and holds the robot standing (0.329 m, 28–35 N
per foot) for the rest of the trial. A flank-down pose (`SPAWN_ROLL=1.4`) is a
measured negative: the brace holds the hips at zero, and the trunk gets no
closer to upright. The measured capture envelope is pitch 0.8–1.6 rad
(recovered in ~2.3 s) and **nothing on the roll axis above the 0.8 rad gate**
with the default brace.

The brace's lateral input is a separate, **unqualified** experiment. The default
keeps the hips at zero; `fall_recovery_roll_brace_hip_rad:=0.8` splays the braced
pair outward (mirrored per side, clamped to the ±1.0472 rad hip range), which is
also what the runner's `ROLL_BRACE_HIP` sets. Measured on a placed 1.4 rad flank
pose, the sign is **positive** — negative drives the trunk onto its back at
3.14 rad / 0.057 m — and +0.8 rad is the magnitude that brings the trunk under
the gate (to 0.66 rad), at which point the ladder leaves the roll phase for
`crouch`. That input is now also kept through `crouch` (and only while roll
dominates), which walks the roll axis through the whole ladder —
`tuck → roll → crouch → stand` — and the splay is then *released* there with the
trunk's remaining roll, because dropping it at the stand entry put the robot on
its back (3.1416 rad / 0.057 m). With both in place the same trials end **not**
inverted: 0.70–0.76 rad / 0.139 m, balanced on the splay in a bounded stop at
~6.8 s. That is a safety result, not a stand-up — a "gather the legs" step is
still missing. Treat the flag as a measurement knob, not a setting.

Two things to know before relying on this: it is measured from
a *placed* pose, because no perturbation on these maps leaves the robot down but
not inverted, and the hold after success is conditional on measured standing
evidence — lose it and the drive stops. Do not enable this expecting a perturbed
robot to stand up.


An opt-in learned actor from the MIT-licensed NJU-RLC Go2 recovery checkpoint
can replace the nominal-pose attempt for experiments:

```bash
ros2 launch robot_lab_bringup simulated_robot.launch.py \
  mode:=loc simulator:=mujoco robot_model:=unitree_go2 \
  map_name:=nav_empty go2_policy_path:=auto \
  enable_fall_recovery:=true go2_recovery_policy_path:=auto \
  fall_recovery_timeout_s:=8.0
```

It uses measured joints, IMU and direct foot forces, and requires ONNX Runtime.
On the same 60 N collapse its bounded action path entered `attempting`, kept
the controller alive, then timed out with the robot inverted at 0.057 m.
This is an experimental comparison, not a qualified recovery behavior. Its
model provenance, MIT license and observation contract are in
`src/robot_lab_adapter/policies/go2_recovery_nju/SOURCE.md`.
An optional `fall_recovery_start_delay_s:=1.0` waits with zero effort before
starting the active timeout. It did not solve the 60 N fall. An initial delayed
trial falsely reported success while the body was airborne with only one foot
loaded, then collapsed. The supported-standing dwell check above corrected
that result; the rebuilt delayed repeat reported `failed` and ended inverted.

### `unrecoverable` is not `failed`

The two verdicts answer different questions, and the state you see tells you
which one you are looking at:

- `failed` — the attempt ran its full window from a pose a stand-up controller
  can act from, and did not stand up. This is a controller shortfall.
- `unrecoverable` — the attempt ended with the trunk rolled past
  `FALL_INVERTED_TILT_RAD` (2.4 rad, i.e. past vertical, body on its back).
  Standing effort cannot right that; it needs a roll-over primitive that is not
  implemented. This is an out-of-envelope pose, not weak gains.

Both are terminal and neither restarts on its own. Reading `unrecoverable` as
`failed` sends you off tuning gains that were never the problem.

The actor is mapped onto this plant through the **measured** per-joint Go2
gains (hip 100/5, thigh 300/8, calf 300/8 at a 0.2 scale) instead of one flat
gain, and its 50 Hz joint target is slew-limited to 3 rad/s. On the recorded
60 N repeat this cut peak measured joint velocity from 55.3 to 10.8 rad/s — but
the trunk still rolled past vertical during the attempt, so the verdict is
`unrecoverable` and the get-up is still unqualified. Lower actuator violence
did not buy a successful stand-up on its own.

A caution learned here: a launch override typed as a string can kill the
controller at startup (`InvalidParameterTypeException`), leaving the robot
completely uncontrolled while the process list still looks healthy. The same
applies to any type error in a published message: a `numpy.float32` effort
raised `AssertionError` inside the `std_msgs/Float64MultiArray` setter and killed
the controller 0.4 s after the fall. The run then *looked* like a clean
non-inversion because a dead node applies no torque. Check
`/go2/safety_state`, `/go2/fallen` and `/go2/recovery_state` message counts and
transitions before trusting any trial — a trial with zero contract messages is
invalid, not a good result.

## Interpret the result correctly

- A successful forward displacement is not successful velocity tracking.
- A turn is not a terrain or navigation qualification.
- A settle after command loss is useful safety evidence, not proof of balanced
  recovery from a fall.
- A perturbation below the noise floor is not evidence of disturbance rejection.
  Compare the pulse window against the pre-pulse window before reporting.
- A latched `safe_stop` proves the fail-safe fired, not that the robot recovered.
- The opt-in re-stand attempt is a measured negative: it does not right the
  robot. Enabling it is an experiment, not a fix.
- Zero messages on a contract topic means the producer died; that trial is
  invalid regardless of how good the other numbers look.
- Direct foot-force telemetry is required before claiming a support pattern;
  it is not automatically consumed by the blind ONNX policy.
- A failed stairs trial remains a failed named task. Do not hide it by
  switching to an easier map without recording the new cell separately.
- This host's CycloneDDS port range caps usable `ROS_DOMAIN_ID` at about 232.
  Higher values fail at node creation with an out-of-range multicast port.

After any trial, update the R5.2 evidence README and the machine-readable
ledger with the revision, exact command, seed/protocol, raw artifact paths and
remaining limitations. Follow the general [workflow](../WORKFLOW.md) and
[roadmap](../../ROADMAP.md) before changing defaults or enabling a new mode.
