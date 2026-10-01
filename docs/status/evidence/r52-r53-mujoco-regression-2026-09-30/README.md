# MuJoCo localization startup and drive regression

Date: 2026-09-30. Source baseline: `e3d63b3` plus the resolver and MuJoCo
launch changes in this working tree. Host: Ubuntu 22.04 / ROS 2 Humble.
Runs were sequential, headless, with RViz disabled and isolated ROS domains.

Two independent defects were found:

1. The GUI/CLI resolver copied a registry environment's spawn zone into every
   launch command. Celisca's stale `(0, 0, 0)` zone replaced the corrected map
   start `(0, 1.32)` and BHL's calibrated root height `-0.038`. The resolver now
   forwards only explicitly requested spawn coordinates. Bringup merges the
   map and robot defaults and initializes localization at the same position.
2. Commit `dc55512` added physics catch-up after an overrun. These extra ticks
   reused an effort command without publishing fresh joint/IMU feedback between
   them. BHL's startup bend fell on both Celisca and `nav_empty`. MuJoCo launch
   now sets `max_catch_up_ticks=0` for effort-controlled robots (BHL and Go2).
   Wheel velocity drives retain the configured catch-up budget.

Source/GUI checks: 34 Tk command/Drive tests and 61 resolver, timing and
kinematics tests passed. The changed adapter, simulator and GUI packages were
built into this workspace. `source_sha256.txt` identifies the files used for
the recorded physics trials. The final combined fast suite passed 668 tests,
with one skipped; registry cross-reference validation passed.

## Recorded results

| Artifact | Condition | Result |
|---|---|---|
| `bhl_before_celisca.json` | Correct map/robot spawn; catch-up enabled; zero command before drive | Fell: peak tilt 1.858 rad, final tilt 1.592 rad |
| `bhl_before_nav_empty.json` | Correct spawn; catch-up enabled; 0.12 m/s command | Fell: peak tilt 1.649 rad, final tilt 1.573 rad |
| `bhl_after_nav_empty.json` | Effort catch-up disabled; 0.12 m/s command | Upright: peak tilt 0.127 rad; little displacement (existing low-command policy stall) |
| `bhl_after_celisca.json` | Effort catch-up disabled; 0.25 m/s command | Walked 1.242 m in x by the end of the command; peak tilt 0.195 rad, final tilt 0.124 rad |
| `go2_after_celisca.json` | Effort catch-up disabled; bundled velocity policy; 0.25 m/s command | 9 simulated seconds, 2.780 m net x displacement, peak tilt 0.099 rad, nominal safety state, no fall |

The attached user log contained the BHL GUI command and its zero spawn
overrides. It did not contain a Go2 run, so no instrumented Go2 before-result
is claimed here. Go2's after-trial records 2250 effort messages and 2251 truth
samples; BHL's Celisca after-trial records 3249 effort messages and 3251 IMU
and truth samples.

## Reproduce

Build `robot_lab_adapter`, `robot_lab_mujoco`, and `robot_lab_gui`; source
`/opt/ros/humble/setup.bash` and this workspace's `install/setup.bash` in both
terminals. Run one simulator at a time and use the same ROS domain in its probe.

```bash
export ROS_DOMAIN_ID=94
ros2 launch robot_lab_bringup simulated_robot.launch.py \
  robot_model:=berkeley_humanoid_lite_sim simulator:=mujoco mode:=loc \
  map_name:=celisca_floor_1 gui:=false start_rviz:=false
```

```bash
export ROS_DOMAIN_ID=94
BHL_PROBE_TOPIC=/key_vel BHL_PROBE_SPEED=0.25 BHL_PROBE_START_T=1 \
  BHL_PROBE_STOP_T=8 BHL_PROBE_TIMEOUT_S=150 \
  python3 docs/status/evidence/r53-bhl-gui-drive-2026-09-24/probe_key_vel.py
```

Go2 uses domain 95 and this launch:

```bash
ros2 launch robot_lab_bringup simulated_robot.launch.py \
  robot_model:=unitree_go2 simulator:=mujoco mode:=loc \
  map_name:=celisca_floor_1 go2_policy_path:=auto gui:=false start_rviz:=false
```

```bash
python3 docs/status/evidence/r52-go2-2026-09-25/probe_stance.py \
  --duration 9 --wall-timeout 150 --drive-vx 0.25 --drive-start 1 --drive-end 7
```

In Robot Lab GUI, select MuJoCo + Localization + the robot and map. BHL's
walking policy defaults on; Go2's flat-ground policy must be checked to walk.
Regenerate the command after reopening the GUI. Normal commands should have
no explicit `spawn_x/y/z/yaw` arguments. Saved manifests with explicit stale
zero overrides retain those overrides and should be regenerated.

## Limits and follow-up

This restores bounded flat-ground stance/walk behavior on these recorded cells.
BHL's held-turn and low-command policy stalls, Go2 terrain/get-up failures, and
other backends remain separate open tasks. These trials did not use a physical
joystick or GUI viewers. Disabling blind effort catch-up can reduce real-time
factor on heavy maps; optimize sensing/collision cost while preserving feedback
cadence before enabling faster effort stepping again.
