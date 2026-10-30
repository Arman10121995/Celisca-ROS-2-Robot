# Amphibious rover

Select **Mobile robots → Amphibious rover → lab_amphibious**, **Gazebo**,
**Marine**, and **amphibious_shore**. Launch chooses the source spawn on the
bank and fills the command. Connect Marine telemetry, wait for real feedback,
then **Arm wheels / wet propellers**. Connect and keyboard enable are neutral.
The familiar latched buttons, WASD/joystick and Space control forward/reverse
and yaw. Depth and strafe are unavailable.

```bash
source scripts/ssd_env.sh
source install/setup.bash
ros2 launch robot_lab_bringup simulated_robot.launch.py \
  robot_model:=lab_amphibious simulator:=gazebo mode:=marine \
  map_name:=amphibious_shore
```

The complete authored model has four 8 Nm wheel motors, four passive spring/
damper suspensions and two 35 N propellers. Wheel motion comes from actual
joint torques/contact; native buoyancy supports the hull as it enters water.
Propeller forces are gated by physical immersion. The native wall-time leases
remove stale actuator requests. Stop brakes wheel rotation and removes
propulsion; the free hull can coast. Springs remain passive when unpowered.
Mass, spring/drag/thrust coefficients and the calm-water draft are laboratory
parameters, not a calibrated commercial vehicle.

The ramp connects a real bank to the seabed. Marine Missions supports a
source-aware shore route: enter target XY, **Plan**, inspect the waypoints and
explicitly **Execute** after Arm. Route Z comes from the declared support
geometry and calm-water draft prior; the controller never commands body Z or
sets body pose. Other water worlds retain surface routing; arbitrary terrain
models require their own support contract. Other terrestrial worlds can use
manual Drive/mapping, with spawn clearance checked against collision geometry.

Enable **RGB-D mapping** before Run to acquire and save the actual database/
cloud. The hybrid estimator uses authored noisy downward-motion/range
emulation, native IMU and the known static support map. It performs no image
matching and makes no hardware optical-flow/DVL claim. Atmospheric pressure
cannot determine a land vehicle's height, so that channel never flattens the
land estimate onto the water plane. The hybrid motion sensor is at the body
origin; the ROV's lower DVL origin is retained for the ROV alone.

Only cloud returns matching the declared support within 2 cm are ground;
unknown regions and other returns remain obstacles. Saved maps must match
the world and `amphibious_downward_flow_range_v1` estimator contract. Mapping,
saved-map relocalization and long autonomous missions retain separate measured
acceptance; startup or an acquired cloud does not qualify every route.

The earlier rigid-wheel source, manual bank/water/bank run and failed
autonomous return are preserved. That return unloaded two wheels at the ramp
edge. The explicit sprung derivative keeps motor limits unchanged and adds
passive compliance. Use the [current delivery](../status/amphibious-facility-2026-10-30.md)
for exact model revisions, actual endpoints and remaining scope.
