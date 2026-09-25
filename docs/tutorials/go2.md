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

The next R5.2 experiment is bounded fall/perturbation recovery with an explicit
first-failure trace. A command-loss stop, a tilt warning/recovery, and a process
that exits cleanly are not equivalent to recovering from a fall. Record the
perturbation time, body height, tilt, contact pattern, effort, safety state and
cleanup outcome.

## Interpret the result correctly

- A successful forward displacement is not successful velocity tracking.
- A turn is not a terrain or navigation qualification.
- A settle after command loss is useful safety evidence, not proof of balanced
  recovery from a fall.
- Direct foot-force telemetry is required before claiming a support pattern;
  it is not automatically consumed by the blind ONNX policy.
- A failed stairs trial remains a failed named task. Do not hide it by
  switching to an easier map without recording the new cell separately.

After any trial, update the R5.2 evidence README and the machine-readable
ledger with the revision, exact command, seed/protocol, raw artifact paths and
remaining limitations. Follow the general [workflow](../WORKFLOW.md) and
[roadmap](../../ROADMAP.md) before changing defaults or enabling a new mode.
