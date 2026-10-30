# Native surface vessel

`lab_usv` is a complete first-party catamaran with two independent propellers,
RGB-D camera and IMU. Gazebo Fortress supplies free-body motion, graded
buoyancy, hydrodynamics and actual bounded thrust. Its 70 kg hull, low ballast
centre, drag and thrust curve are authored laboratory parameters. There are no
attitude locks or body-pose/velocity writes. Waves and hardware calibration
are outside the current model.

In Launch select **Surface vessels → Twin-hull USV → lab_usv**, **Gazebo**,
**Marine**, and **surface_harbor**. The command and surface-height spawn fill
automatically. The harbor includes above-water walls, a dock, pilings and
channel markers. Other installed water worlds remain selectable; their
missions need their own acceptance.

```bash
source scripts/ssd_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_lab_bringup simulated_robot.launch.py \
  robot_model:=lab_usv simulator:=gazebo mode:=marine \
  map_name:=surface_harbor marine_slam:=true
```

In the Marine control column, Connect, wait for fresh body feedback, then
**Arm surface thrust**. Enable WASD/joystick or click the latched direction
buttons. These actions begin neutral. Forward/reverse and yaw use differential
propeller forces; strafe and depth controls are disabled. Stop/Space removes
thrust; the free hull can coast. Turning is a rate command, followed by heading
hold when released. A native 0.4 s force watchdog removes stale ROS requests,
including a stalled command owner.

Marine Missions uses the separate noisy pressure/DVL simulation and native
IMU orientation/rates. Use **Waypoint ahead**, or enter XY and retain the
estimated draft Z, then **Plan** and explicit **Execute**. Routes stay in a
surface-height slab and reject diving goals. Source/observed obstacle checks
remain active. Arrival checks both translational speed and IMU yaw rate.

Enable RGB-D mapping before Run to acquire a map, save the actual database,
and select a protected saved copy for localization. Experiments → **Marine
map acquisition** or **Marine route** runs the same plant in an isolated
session, with independent body observations. The harbor acquisition and
two-goal arrival cases have measured receipts; this is not acceptance for
every water world, currents, docking, waves or loss scenario.

Source/model generation: `scripts/generate_surface_assets.py`. Preserve the
native force watchdog and explicit source-water boundary. The first high-CG
trial tipped under Fortress's graded-surface approximation; that failed run
is retained. The declared low ballast centre stabilizes the physical plant
without holding its body in place.
