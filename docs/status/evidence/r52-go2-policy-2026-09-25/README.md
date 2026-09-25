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
| `go2_policy_stairs` | Started at (-7.5,-4.0) facing the first `terrain_stairs` ledge; +0.5 m/s for 7 s moved only +0.118 m, max effort reached 35.55 N m | Named terrain task fails at first ledge |

The trial does **not** complete R5.2. The policy does not reliably track
small reverse commands and cannot climb the tested ledge. Its contact
heuristic still lacks a direct foot-ground measurement, and other maps,
fall recovery and navigation have not been qualified. GUI velocity-base,
SLAM and navigation support for Go2 therefore stay disabled. Next work is
reverse calibration or training, direct foot-contact telemetry, flat-ground
tracking and terrain tests with a policy trained for those conditions.

The affected Go2 core/policy, bringup-profile, MuJoCo-effort, GUI-drive and
registry Go2 tests passed together (335 tests). After the GUI checkbox was
added, 34 GUI-drive/policy tests passed under Xvfb. The adapter and bringup
packages built, and `go2_policy_path:=auto` resolved the installed ONNX graph
and its external data in the live forward run.
