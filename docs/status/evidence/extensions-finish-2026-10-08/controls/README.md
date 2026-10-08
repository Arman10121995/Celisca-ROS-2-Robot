# Current GUI motion controls: October 8

These are actual owned GUI/physics screens on the Jetson, using predecessor
`73fcc47` plus exact pre-trial working-tree and installed source manifests.
Original reports are copied unchanged. Large traces stay on the workspace SSD;
`raw-artifacts.json` and the parent collection manifest preserve paths, sizes
and SHA-256 values. These screens do not close full robot/map/backend tasks.

## Native Panda / MuJoCo / static nav_empty

Select the native Panda, MuJoCo and Display, Run, then Arm. **Use current tool
pose** reads measured FK; the default 1 cm X/Y/Z buttons edit a target with
orientation held. **Plan** and **Execute Plan** remain separate.

The [normal GUI report](panda-normal/report.json) measures X/Y/Z movements of
9.73/10.00/10.47 mm and endpoint errors of 0.366/0.490/0.640 mm. Target selection
itself causes no motion. Full collision/unreachable rejection, joint/finger
invalidation, Cancel/Stop/watchdog, monotonic Reset/Home, telemetry and clean
planner shutdown pass. A [separate Hand repeat](panda-hand/report.json) measures
78.1 mm physical cube lift, release and 0.845 s heartbeat loss under unchanged
0.5 N per-finger limits.

The [sag failure](panda-sag-negative/report.json) is preserved: a 1 cm Z target
moved 2.9 mm, missing the declared displacement gate. Bounded gravity/Coriolis
bias now feeds through the original position actuators with ±0.03 rad offsets;
native control/force ranges, gains and contact checks remain. The initial
[producer error](panda-producer-error/report.json) is a failed diagnostic,
not runtime acceptance. The legacy [contact-point negative](panda-contact-point-negative/report.json)
is retained: aligned pads distribute force across sixteen points, so the
updated probe uses the existing certificate's total force per finger rather
than requiring one individual point above 0.05 N. Real lift/release remains
mandatory. [Source revalidation](panda-source-revalidation.json).

## Native X500 / PX4 / Gazebo Harmonic / nav_empty

The [normal owned flight report](px4/report.json) exercises the actual seven
Drone buttons, Takeoff, altitude Up/Down, release, Stop, publisher loss and
Land/disarm. Enabling WASD/joystick publishes no command and leaves the vehicle
disarmed. Two-second left/right inputs produce +0.601/−0.609 rad body yaw with
0.3 rad/s command caps. Loss hold drift is 0.041 m. Independent body, rotor,
FCU/controller and native model/world hashes accompany the screen.

Reproduce serially, with one plant:

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=224 GZ_PARTITION=current_controls_repeat xvfb-run -a \
  python3 scripts/qualify_px4_drive_gui.py \
  --output /workspace/molar/robot_lab_runtime/px4/new-controls-repeat \
  --map nav_empty
```

Use a new output directory. Land before stopping an operator flight. This is
manual open-map flight; aerial mapping, obstacle planning and all-world ceiling/
spawn qualification remain open.

## Shared Drive source guard

Four current Husky physical Drive/source-sensor/neutral/Stop/loss/owned-relaunch
screens pass after the GUI update. [Revalidation receipt](husky-source-revalidation.json)
retains each exact source stage and the consumed expanded URDF. The expanded
description normalizes XML whitespace and absolute mesh URIs to `file://`;
parsed tags/attributes/text/order match the unchanged pinned original after
that URI normalization. Original twenty higher-mode measurements are retained
at their original stages; this GUI regression does not rerun that matrix.

## Core-base Display routing and Bumperbot braking

Bumperbot and Labbot now opt into the existing Display twist-mux command route.
The normal Labbot/MuJoCo/nav_empty screen passes Drive, neutral enablement,
Stop, command loss and owned relaunch. Bumperbot's first stronger-servo screen
fails the unchanged stopping gate at 0.110 rad/s residual yaw. With profile-local
reflected wheel inertia 0.02 kg·m², gain 10 and the unchanged 5 N·m torque cap,
its normal repeat passes both directions/turns and settled Stop/loss. Final
turn rates are +1.981/−1.981 rad/s; settled residual yaw is below 4e−12 rad/s
in this run. Neutral enablement publishes zero commands; full relaunch returns
to the original XY exactly. This is a named Display screen, not all-map or
navigation acceptance. Both stages and their original traces remain under
`core/` and in the collection manifest.

The motor-inertia change uses MuJoCo's documented [joint armature](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-joint-armature)
contract; its value is a declared lab simulation parameter, not a measured
vendor motor. The isolated small-wheel physics test retains Euler integration
and checks the actual gain/inertia pair through reverse and braking.
