# Native Tethys AUV

The complete **MBARI Tethys AUV** is installed in the normal robot registry
with its retained source/CC-BY attribution. Its single native propeller and
two source fins provide a different control plant from the eight-thruster
Laboratory ROV. Gazebo Fortress 6.18 supplies actual buoyancy, hydrodynamics,
fin lift/drag and propulsion. This is experimental simulator guidance, not
MBARI's hardware autopilot or independent underwater navigation.

## GUI workflow

1. Source `scripts/ssd_env.sh`, `/opt/ros/humble/setup.bash` and
   `install/setup.bash`. Start `ros2 run robot_lab_gui robot_lab_gui`.
2. Select **Underwater robots → AUV → tethys_auv**. Gazebo, Marine and
   `marine_tank` are selected; the command fills automatically. All five
   installed marine worlds are selectable. Tank/reef use explicit long-body
   spawns; the other three have not received physical spawn/mission acceptance.
3. Run. In **Marine**, Connect and wait for body/joint feedback. **Arm AUV
   cruise guidance** starts neutral with zero propeller force.
4. Click Forward to latch and ramp surge. Click a diagonal, or combine
   forward and yaw latches, to turn. Yaw alone cannot pivot this vehicle.
   Its source rudder direction is converted at the adapter boundary.
5. Request **Depth + / −** or Set depth **while cruising forward**. Guidance
   uses the actual body/fin response and clamps source fin ranges. Strafe
   buttons are disabled. Reverse is propulsion-only; depth/heading fin
   guidance is active only during forward flow above 0.15 m/s.
6. Click a direction again to brake toward neutral. Stop/Space removes
   propulsion and neutralizes fins; the body can coast and change depth.
   This underactuated AUV cannot hover or hold depth at zero flow. Re-arm
   explicitly after Stop or feedback/operator loss.

Linear/yaw fields remain increments per 0.1 seconds. The panel shows real
position/velocity/depth, force requests, measured joints, fin targets and
whether cruise guidance has enough flow. Keyboard/gamepad starts neutral;
physical joystick qualification remains pending.

Display refuses propulsion ownership and shows actual body/joint motion.
Ground Localization/SLAM/Nav2 modes and other simulators remain unavailable.
The five worlds contain real static obstacles and fluid density boundaries;
they are not flat 2D occupancy maps.

```bash
ros2 launch robot_lab_bringup simulated_robot.launch.py \
  robot_model:=tethys_auv simulator:=gazebo mode:=marine map_name:=marine_reef
```

## Source and handoff

Read [model provenance](../../src/robot_lab_robots/tethys_auv/SOURCE.md) and
[actual diagnostic scope](../status/auv-2026-10-15.md). The original Fuel v3
model and Fortress example are retained; no network download occurs in the GUI.
`scripts/generate_tethys_assets.py` creates the derived native SDF, four
individual display submeshes, URDF and explicit trim/actuator calibration.

The 1025 kg/m³ worlds require a declared internal trim mass, not an invisible
hull-mass change. Positive mass diagonals implement the installed Fortress
6.18 **−Ma × acceleration** convention. Keep original source files, signed
drag, actual Coriolis/added mass and the recorded earlier wrong-sign failures.
Do not copy the old example coefficients back into the derived plant.
Use the explicitly version-6 launch; PX4 has a separate Harmonic runtime.

`auv_simulator.launch.py` copies the **selected** water world's geometry,
changes the buoyancy inclusion to this owned AUV, resolves local resources,
and bridges propeller/fins/body/joints/IMU. `auv_controller.py` reuses the ROV
observer/operator protocol but supplies its own source-frame cruise law and
actuator limits. Preserve zero-output neutral enable, session/lease/clock/
body/joint watchdogs, no strafe/pivot/hover assumptions and actual force PID.
Source fields `use_velocity_commands=false` prevent perfect-position fins.

The observer currently uses actual simulator poses. `/marine/pressure_ideal`
is explicitly pose-derived emulation; it is not an independent pressure
instrument, underwater GPS or SLAM estimate. Native IMU and real joint streams
are separate. Later work needs independent measurements/filtering, sonar/DVL,
saved 3D SLAM and feasible volumetric AUV routes with turning/flow constraints.
The ROV and AUV cannot share an unconstrained hovering waypoint controller.
Source current, disturbance, reverse, loss/reset/contact and longer inspection
missions remain later qualification work; initial implementation precedes
those broad campaigns.
