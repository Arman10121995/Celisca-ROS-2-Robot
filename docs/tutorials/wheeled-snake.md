# Native passive-wheel snake

`lab_snake` is a complete first-party articulated robot: ten massive segments,
nine yaw motors and twenty **unpowered** wheels. A bounded serpenoid joint
wave moves it through actual MuJoCo ground contacts. It is a separate family
from the legged centipede. Source geometry, motor limits and provenance live
under `robot_lab_robots/lab_snake`; `generate_serpentine_assets.py` reproduces
the model. Gains and wave parameters are authored, not hardware calibration.

Select **Articulated robots → Wheeled snake → lab_snake**, **MuJoCo**,
**Gait** and a ground world. In the Gait control column choose the sensor task
before Run. The normal command box fills automatically:

```bash
ros2 launch robot_lab_bringup simulated_robot.launch.py \
  robot_model:=lab_snake simulator:=mujoco mode:=gait \
  map_name:=nav_empty gait_task:=slam
```

Connect Gait telemetry, wait for actual joint/contact feedback, then Arm.
Use the shared latched Drive pad, WASD/joystick and Space. Forward/reverse
select wave direction; yaw bends the travelling wave. There is no strafe,
stationary pivot or foot IK. The Gait panel reports zero physical legs and the
`serpenoid` schedule. Stop halts phase progression and holds bounded motor
targets while the body and passive wheels remain free. Native PD command loss
also holds the last target; it does not teleport the body or straighten it
abruptly. Return to source stance is an explicit bounded operation.

SLAM, 3D SLAM, saved-map localization and curvature-constrained Nav2 are wired
as experimental tasks. Wheel encoders, exact spine FK, measured rolling
contacts and IMU orientation/rates produce separate estimated odometry.
Simulator body pose is visualization/tilt/independent measurement only.
The navigation heading is the mean segment tangent, rather than the oscillating
head angle. A measured dynamic transform retains the camera/head geometry;
2D ray angles are reprojected into a fixed virtual laser frame for AMCL.
Lateral scrub is allowed; an almost straight body needs an explicit local
head-contact no-side-slip assumption. Lost observations accumulate uncertainty
and a conservative missing-travel bound. Stale/inconsistent odometry stops
autonomous control. A fresh, bounded AMCL correction can recover relative-pose
uncertainty; lost localization requires a fresh session. Instantaneous source
wheel-centre Jacobians use measured spine velocities and at least three current
rolling contacts. These are wheel constraints, not continuous planted feet.

Navigation uses a head-centred envelope covering the whole articulated body,
a sufficiently large local costmap and curvature-bounded paths. It cannot
fit every corridor that a compact differential base fits. Source encoders and
contact assumptions still need terrain/slip qualification. The current MuJoCo
IMU acceleration channel remains a stationary-gravity approximation; this
estimator uses its orientation and angular rates only.

Select SLAM or 3D SLAM before Run to acquire a map. Save it with the Gait
controls, then select that protected map for Localization or Navigation in a
fresh launch. Goals are expressed in the configured world/map registration;
the visualization transform does not move the requested destination.

The measured `nav_empty` cases include 0.268 m travel with a populated 2D grid,
0.259 m with a real RGB-D database/cloud, an installed-map goal and a goal
after reloading the acquired 2D map. Both goals physically settle within
0.15 m / 0.15 rad, below 0.03 m/s and 0.05 rad/s for 1 simulated second.
Arrival holds the current shape; straightening after action success previously
moved the head outside tolerance. [Actual results and limitations](../status/native-forms-2026-10-30.md).
Earlier backward-wave, clipped-costmap, missing static sensor TF and
rolling-odometry failures are retained. Longer curved/reverse routes, 3D-map
relocalization, terrain and other backends remain separate acceptance.
