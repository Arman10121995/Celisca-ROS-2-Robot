# R5.3: Berkeley Humanoid Lite GUI drive through the common MuJoCo launch

The `berkeley_humanoid_lite_sim` profile now starts its ONNX effort controller
automatically in MuJoCo `loc` mode. The controller consumes twist_mux's
`/robot_lab_controller/cmd_vel_unstamped` output from the GUI's `/key_vel` Drive
pad, and the simulator runs without passive joint hold. `display` stays a
passive visualization mode. SLAM and navigation remain unavailable for this
profile because neither has a qualified sensor/goal path.

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

An additional 0.3 rad/s pure-turn command **failed** twice. With the original
2 s startup bend, tilt crossed the 0.70 rad safety threshold during the bend
and the body fell (`turn_2s.json`, `turn_2s_launch.log`). A 4 s bend still
fell (`turn_4s.json`, `turn_4s_launch.log`), so that experiment was reverted.
An experimental tilt-triggered retreat also recovered once but then fell
(`turn_tilt_guard.json`, `turn_tilt_guard_launch.log`); it was reverted too.
The turn command does not affect the joint targets until the bend completes;
the failure exposes startup-bend sensitivity in the full ROS graph, not a
qualified turning defect or success. More startup/stance work is required.

Each run continued for five simulated seconds after the stop command; the
last two finished near 0.13 rad tilt with negligible final pose drift. The
controller published about 250 effort commands per simulated second. The
different forward distances show that command tracking still needs work.
There is no claim here of reliable turning, reversing, terrain traversal, or
navigation. The GUI mode gates remain unchanged apart from the locomotion
actually working in MuJoCo localization mode.

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
`BHL_PROBE_SPEED=0 BHL_PROBE_YAW=0.3` for the turn trial. The direct
MuJoCo probe additionally passed after a 20 simulated-second idle interval
(`delayed_direct_report.json`) and from the offset spawn
(`offset_direct_report.json`), isolating the failure to the larger ROS launch
rather than the spawn location alone.

Verification: `robot_lab_adapter` policy and ROS graph tests 50 passed; bringup profile
tests 199 passed; GUI command-autofill tests 24 passed. The common launch was
rebuilt and live tested in two isolated ROS domains. The first attempted
pytest invocation was blocked by a stale host `anyio` plugin importing
`_pytest.scope`; rerunning with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` passed.
