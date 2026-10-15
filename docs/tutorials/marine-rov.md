# Laboratory ROV and underwater worlds

The later [native Tethys AUV workflow](tethys-auv.md) is also installed. Both
marine plants now use positive added-mass diagonals for Fortress 6.18's
**−Ma × acceleration** convention; signed damping stays negative. Read the
[latest marine receipt](../status/auv-2026-10-15.md) for the corrected ROV and
the distinct propeller/fin AUV. Original diagnostics retain their old sources.

The first marine workflow is an experimental, complete, first-party eight-
thruster ROV in **Gazebo Fortress 6**. It uses actual buoyancy, hydrodynamic
drag/added mass and joint-mounted thruster forces. It does not write body poses
to imitate swimming. Source mass, collision volume, thruster directions and
force limits determine the allocation and weight compensation.

## Run from the normal GUI

Source `scripts/ssd_env.sh`, `/opt/ros/humble/setup.bash` and
`install/setup.bash`, then run `ros2 run robot_lab_gui robot_lab_gui`.

1. In Launch, select **Underwater robots → ROV → lab_rov**. The compatible
   Gazebo backend, Marine mode and `marine_tank` default are selected; the
   command box fills automatically. Reef, harbor, cave and wreck are also
   installed water worlds, with their own submerged spawns.
2. Run the command. Open **Marine** in the right control column and Connect.
   Connecting and enabling WASD/joystick start neutral. Wait for actual body
   feedback, then press **Arm depth/heading hold**.
3. Click a direction to latch it, click again to brake. Forward/reverse,
   diagonal/yaw and strafe controls command bounded velocities through actual
   thrusters. The displayed linear/yaw values are increments per 0.1 seconds.
   WASD uses the same ramp; Space stops. A real joystick requires later
   hardware-specific validation.
4. **Depth +** requests 0.2 m deeper; **Depth −** requests 0.2 m shallower.
   Set depth accepts metres below the water surface. Hold clears planar input
   and holds the currently observed depth/heading. Near surface requests
   0.35 m depth, keeping the body submerged.
5. Stop disarms and clears all eight thruster commands. This slightly
   negatively buoyant vehicle can sink or drift afterward: water forces and
   gravity remain active. Re-arm explicitly to hold again. Lost operator,
   body observations or clock feedback also remove thrust.

Display mode provides observation without actuator ownership. Ground
Localization, SLAM and Nav2 modes and other simulators remain unavailable for
this model. A water volume is not a 2D occupancy map.

Equivalent command:

```bash
ros2 launch robot_lab_bringup simulated_robot.launch.py \
  robot_model:=lab_rov simulator:=gazebo mode:=marine map_name:=marine_tank
```

## What is measured, and what is simulated

The panel reports the body's actual Gazebo pose and finite-difference
velocity, target depth, applied force requests and sample count. The controller
currently uses this **simulator-pose observer**. `/marine/pressure_ideal` is
an explicitly ideal pressure emulation from that pose, not an independent
pressure instrument. Native IMU and RGB-D camera observations and real rotor
joint states are bridged separately. This does not establish marine state
estimation, DVL/sonar, underwater GPS, saved SLAM or autonomous navigation.

Source defaults: hull 0.4 × 0.28 × 0.16 m, complete model mass 18.5417 kg,
displaced collision volume 0.017985475 m³, water density 1025 kg/m³, eight
thrusters bounded to ±12 N. Planar caps are 0.35 m/s surge, 0.25 m/s sway and
0.35 rad/s yaw; heave is capped at 0.2 m/s. These are educational authored
parameters, with no hardware calibration claim. The five worlds have a real
seafloor at −6 m, water surface at 0 m and collision obstacles; the surface
visual itself has no solid collision. Currents can be supplied to the native
`/marine/current` Vector3 bridge in world axes; UI/current-rejection missions
remain pending.

## Agent continuation

Read [the measured scope](../status/marine-2026-10-15.md) and the task ledger
before editing. Reuse the lower `robot_lab_utils` controller/model contract;
do not introduce a bringup/adapter dependency cycle. The generator is
`scripts/generate_marine_assets.py`, provenance is
`src/robot_lab_robots/lab_rov/SOURCE.md`, and the dedicated physical launch is
`marine_simulator.launch.py`. Main bringup branches to it before ground spawn,
2D-map and ros2_control logic. Keep `auto` parent launch arguments distinct
from explicit numeric spawn overrides. Use the **Fortress** plugins shipped
with Humble, not the separate Harmonic 8 plugins used by PX4.

ROS interfaces are `/marine/{command,operator_heartbeat,state,key_vel}`, actual
`/marine/{odometry,imu}`, `/joint_states`, `/oakd/...` and per-joint
`/marine/thrusters/<joint>/force`. Commands identify the live session and
operator; arm requires fresh actual observations and operator lease. Keep
force bounds, uniform wrench scaling, NaN rejection, zero-output Stop,
single-operator ownership and wall-clock watchdogs in every adapter.

The bounded normal-GUI diagnostic measures depth/surge/Stop in the tank only.
Retain the failed first spawn run and exact source hashes; do not generalize
it to the other four worlds or all six axes. The separate native AUV contract
is now installed; continue independent marine measurements and a sensor-based
volumetric mapping/route pipeline. Then hand off disturbance,
contact, communication-loss and repeated mission qualification. R10 gait and
remaining fleet integrations still precede the old roadmap. Broad campaigns
are deferred under the user's implementation-first priority.

Primary implementation references: [Fortress underwater systems](https://gazebosim.org/api/gazebo/6/underwater_vehicles.html)
and [the installed-generation thruster implementation](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/systems/thruster/Thruster.cc).
