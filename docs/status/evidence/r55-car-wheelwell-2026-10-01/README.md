# R5.5 — Car steering was geometry-blocked, not controller-blocked (2026-10-01)

## What was wrong

The recorded "weak MuJoCo arc response" (`r55-car-watchdog-2026-10-01/`: left
arc 0.067 m/s and 0.108 rad/s for a commanded 0.4 m/s / 0.5 rad/s) was **not**
a drive, servo or physics-tuning problem. The chassis collision box overlapped
the steered tires' swept volume:

* wheels at `y = ±0.14 m`, tire radius 0.05 m, tire width 0.04 m, steering
  joint limit ±0.8 rad → the tire's innermost point reaches
  `0.14 − 0.02·cos(0.8) − 0.05·sin(0.8) = 0.090 m`;
* the chassis collision box was `0.48 × 0.22 × 0.08 m`, i.e. its edge sits at
  `y = ±0.11 m` — inside the tire's sweep.

MuJoCo therefore resolved the steering motion as a **chassis/tire contact of
~113 N** (offline probe, `offline/offline_before_fix.txt`, contact `[17]`
`front_left_wheel_link_contact_0 | <chassis box>` with `dist = −0.91 mm`), whose
lever arm about the steering axis saturated the 10 N·m steering servo:

```text
FL steering: commanded 0.4516 rad -> measured 0.2288 rad, qfrc_actuator = 10.000 (saturated)
FR steering: commanded 0.3281 rad -> measured 0.2159 rad, qfrc_actuator = 5.588
qfrc_constraint on the FL steer dof = −10.000 N·m (the chassis contact)
front wheels stalled: 0.70 / 1.51 rad/s of 7.3 / 9.8 rad/s commanded, 5.0 N·m saturated
body vx 0.059 m/s, wz 0.042 rad/s
```

The same command reproduced offline with no ROS graph at all
(`offline/offline_car_plant_probe.py`), which is what separated the geometry
defect from the drive model and the actuator gains. PyBullet had masked the
defect because its steering motor drives the joint hard enough to win against
the contact, which is why the PyBullet traces looked correct while MuJoCo
stalled.

## The fix

The chassis **collision** box is now narrower than the body shell — a wheel
well — in `ackermann_car/urdf/ackermann_car_macro.xacro` (0.22 m → 0.16 m wide
collision; the visual shell is unchanged) and in
`holonomic_wheels/urdf/holonomic_wheels_macro.xacro` (0.30 m → 0.20 m). The
clearance is now ~11 mm at the steering limit. Nothing else about the car
changed: same wheels, same limits, same drive model, same 5 N·m wheel servos and
10 N·m steering servos.

`test_xacro_expansion.py::test_steered_wheel_sweep_clears_the_chassis_collision_box`
pins the clearance from the expanded URDF numbers for both bases.

## Measured after the fix (MuJoCo, simulator ground truth, ROS domains 210–212)

Speeds are measured from the odometry **header stamps** (sim time): the probe
now uses the stamp for `dt`, because arrival-time velocities on this host
under-report by the real-time factor (a 0.400 m/s command measured 0.281 m/s
that way). This is a measurement fix in `scripts/sim_drive_check.py`, not a
change to any controller.

| Profile | straight 0.4/0 | left arc 0.4/+0.5 | right arc 0.4/−0.5 | reverse −0.3/+0.3 | tight 0.3/1.5 (clamped) | steer FL/FR on left arc |
|---|---|---|---|---|---|---|
| `ackermann_car` | 0.400 / 0.000 | 0.374 / 0.486 | 0.368 / −0.487 | −0.294 / 0.301 | 0.279 / 0.600 | +0.445 / +0.322 |
| `rear_steer_car` | 0.400 / 0.000 | 0.366 / 0.499 | 0.367 / −0.499 | −0.295 / 0.292 | 0.277 / 0.608 | −0.459 / −0.334 (rear axle) |
| `anti_ackermann_car` | 0.400 / 0.000 | 0.369 / 0.473 | 0.368 / −0.469 | −0.291 / 0.300 | 0.277 / 0.516 | +0.326 / +0.448 (outer steers more) |

(vx in m/s, wz in rad/s. Left-arc yaw tracking is now 95–100 % of the command
against 21 % before.)

Command loss (publisher silent, 2.2 s window, no explicit zero sent):

| Profile | coast | yaw after loss | final 0.5 s vx / wz |
|---|---|---|---|
| `ackermann_car` | 0.122 m | 0.244 rad | 0.000 / 0.000 |
| `rear_steer_car` | 0.115 m | 0.237 rad | 0.000 / 0.000 |
| `anti_ackermann_car` | 0.122 m | 0.202 rad | 0.000 / 0.000 |

Offline, after the fix (`offline/offline_after_fix.txt`): stationary steering
reaches **0.4516 / 0.3281 rad with `qfrc_actuator = 0.000`**, and the left arc
measures `wz = 0.488 rad/s` of 0.5 commanded with every wheel rate tracking its
target.

## Reproduction

```bash
# one profile (ROS_DOMAIN_ID below 233 on this host)
ROS_DOMAIN_ID=210 MODE=display ODOM=/odom/ground_truth TIMEOUT=120 WARMUP=2 \
  CHECK_ARGS='--command-loss --command-loss-duration 3' \
  bash scripts/sim_drive_check.sh mujoco ackermann_car nav_empty /tmp/ack.json

# the offline plant probe (no ROS): straight / stationary steer / left arc
python3 offline/offline_car_plant_probe.py
```

Source revision `906d7fe` plus the documented working-tree changes; ROS
2 Humble, Gazebo Harmonic, MuJoCo 3.12, NVIDIA Jetson Orin (12 cores). The
installed `robot_lab_mujoco` / `robot_lab_pybullet` packages are **copies** on
this host, so a bridge edit needs `colcon build --packages-select … --symlink-install`
before a live run (this was verified by the "60 joint(s) held" log line of a
stale run).

## Gazebo repeat attempt (2026-10-01, later) — still open

The Gazebo cells were repeated for all three profiles (`MODE=loc`,
`ODOM=/robot_lab_controller/odom`, ROS domains 220–222) and **did not produce
controller odometry**: every run ended `{"error": "no odometry on
/robot_lab_controller/odom"}`.

One packaging fault was found and fixed on the way: the launch aborted with
`executable 'gazebo_reset_bridge.py' not found on the libexec directory
'install/robot_lab_description'`, i.e. a stale install of that package.
`colcon build --packages-select robot_lab_description --symlink-install`
restored the executable and the launch then ran to completion - but the car's
Gazebo controller still publishes no odometry in this configuration, so the
Gazebo publisher-loss/heading repeat remains **unmeasured**, and no Gazebo
number from this round may be compared with the MuJoCo/PyBullet rows above.
Also unverified here: whether Gazebo reproduces the wheel-well repair (the
measurement path itself is missing).

## Still open (not claimed by this record)

* Slalom, parking and obstacle missions with a feasibility/curvature check are
  still unmeasured — only the drive phases above are.
* Publisher-loss repeats exist for the three profiles on MuJoCo and PyBullet,
  not for Gazebo/Isaac in this increment; the Gazebo cells still use controller
  odometry rather than independent truth.
* `rear_steer_car` versus `anti_ackermann_car` are distinguished here by
  measured joint targets and measured arc response (rear axle moves for
  rear-steer; the outer wheel steers more for anti-Ackermann); the GUI/tutorial
  wording for the user's "reverse Ackermann" terminology is unchanged.
