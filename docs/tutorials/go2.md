# Go2 flat-ground policy: bounded runtime evidence

This tutorial exercises the opt-in Go2 flat-ground ONNX policy through the
standard MuJoCo ROS route. It is a controlled evidence walkthrough, not a
claim that Go2 terrain traversal, navigation or the GUI velocity-base mode is
qualified.

The recorded evidence is under
[`docs/status/evidence/r52-go2-policy-2026-09-25/`](../status/evidence/r52-go2-policy-2026-09-25/README.md).
At the current source snapshot, the policy has produced measured forward
motion, a stop, a large turn, command-loss stop and a reverse dead-zone
compensation trial. Low-speed reverse tracking, the first stairs ledge, fall
recovery and navigation remain open.

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

For the next R5.2 iteration, run the reproducible sweep first; it records
raw probe JSON/logs and a summary instead of relying on one hand-copied trial:

```bash
docs/status/evidence/r52-go2-policy-2026-09-25/run_reverse_sweep.sh \
  --out-dir /tmp/go2-reverse-sweep \
  --domain-base 201 \
  --commands=-0.15,-0.25,-0.35,-0.45
```

The recorded baseline is
[`reverse_sweep_20260925_rerun/`](reverse_sweep_20260925_rerun/summary.json).
It bypasses the old reverse dead zone but overdrives low-speed commands
(observed/requested ratios 1.11–1.62). Treat the current feed-forward map as
a measured baseline, not a qualified velocity controller. Next test an explicit
opt-in inverse-map or retrained policy A/B; do not change the default or claim
velocity tracking from displacement alone.

For the next R5.2 iteration, sweep several negative commands around the measured
dead zone with identical initialization. Report command, actual displacement,
tracking error, yaw drift, tilt, effort and contact count. Then repeat
forward/reverse/turn/stop before attempting terrain.

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
