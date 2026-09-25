# R5.2 Go2 MuJoCo live checkpoint (2026-09-25)

The standard `simulated_robot.launch.py` path now loads the commandable Go2
xacro, 12 named effort actuators, a measured-joint ROS stance controller,
standing joint initialization, a 1 ms MuJoCo integration step and 0.01 kg m²
effort-joint armature. The latter two are scoped to the Go2 effort launch;
the URDF-imported model was unstable at 2 ms and at 250 Hz sampled feedback
without armature. The node limits efforts to the URDF limits, has a command
watchdog and latched tilt/effort safety stop with an explicit reset service.
The original motor-effort residual was only a contact heuristic: the MuJoCo
ideal motor reports actuator effort, not direct foot-ground force. A later
backend update now publishes `/go2/foot_contact_forces` in FL, FR, RL, RR
order from MuJoCo contact normal forces against world geometry. The core
uses those direct forces when available. `go2_contact_stance.json` records
1,002 contact messages during a four-second upright stance; all four feet
were loaded after touchdown.

Command (the probe was started before launch to capture startup):

```sh
ros2 launch robot_lab_bringup simulated_robot.launch.py \
  mode:=loc simulator:=mujoco robot_model:=unitree_go2 \
  map_name:=nav_empty gui:=false start_rviz:=false
python3 docs/status/evidence/r52-go2-2026-09-25/probe_stance.py \
  --duration 8 --wall-timeout 45
```

`stance_qualified.json`/`.log`: eight simulated seconds, 2,001 truth and
joint-state samples, 1,999 effort messages, minimum trunk height 0.332 m,
maximum tilt 0.0032 rad, 0.0042 m XY drift, maximum command 19.93 N m.
The probe passed. An earlier 8-second run (`stance_armature001.json`) also
stayed upright but was falsely marked failed by an exact-duration comparison
of 7.996 against 8.000 seconds; the probe now allows one sample interval.
The later gated-command run (`stance_drive_gated.json`) also passed six
seconds while a +0.25 m/s request was sent. A 20-second stance
(`stance_long20.json`) remained upright but slowly sagged from about 0.35 m
to 0.291 m and drifted 0.073 m, with peak tilt 0.146 rad. This is not
long-duration balance qualification.

The initial joint-neutral spawn (`loc.log`) and the nominal standing-pose
spawn without tuned physics/control (`stance_trace.json`) both fell. The
2 ms step and the original controller damping also failed under sampled
feedback (`stance_timestep001.json`, `stance_early_trace.json`,
`stance_sampled_gain02.json`). These are negative evidence, not successful
stance runs.

Locomotion **does not pass R5.2**. The experimental trot stayed upright but
both +0.25 m/s and -0.25 m/s commands moved the root backward about 0.15 m
in the seven-second runs (`drive_sweep_forward025.json`,
`drive_sweep_reverse025.json`). The earlier simplistic gait also moved
backward (`drive_forward025.json`). The nominal controller therefore ignores
nonzero drive commands until gait is qualified. To reproduce the failing
motion experiment, opt in with
`go2_enable_experimental_gait:=true` and run `probe_stance.py --duration 7
--drive-vx 0.25 --drive-start 1 --drive-end 4`. The recorded drive runs
predate that explicit opt-in flag; they used the same gait code with the
gate open by default. No forward/reverse tracking, turning, stopping or
terrain task is claimed.

The focused Go2 core, bringup-profile, MuJoCo effort and GUI drive tests
passed together (305 tests); the additional Go2 xacro/effort integration
test passed in a subsequent focused run (80 tests). The Go2 adapter,
MuJoCo and bringup packages
built with `colcon build --symlink-install`. The long-run sag and failed
tracking still keep R5.2 partial.

The next R5.2 step is a velocity-tracking policy/controller and flat-ground
forward/reverse/turn/stop proof before any GUI navigation/SLAM enablement.
The [official Unitree RL Gym](https://github.com/unitreerobotics/unitree_rl_gym)
documents a MuJoCo sim-to-sim deployment path; its model, joint ordering,
observation convention and policy assets need to be checked against this
workspace before adoption. The [official Unitree MuJoCo simulator](https://github.com/unitreerobotics/unitree_mujoco)
also has Go2 stand examples but does not itself establish velocity tracking
in this ROS launch.
