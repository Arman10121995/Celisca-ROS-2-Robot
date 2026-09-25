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
