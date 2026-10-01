# R5.4 — PX4 SITL FCU built and reachable; flight still blocked at preflight (2026-10-01)

## What changed

R5.4 was `queued` with no FCU on this host. This increment builds and runs a
real **PX4-Autopilot SITL** FCU and drives it over MAVLink, which removes the
"no FCU at all" blocker. It is pinned, licence-clean and reproducible.

* Source: `PX4/PX4-Autopilot` @ `9be7c6f391fb` (BSD-3-Clause), shallow clone in
  `/workspace/molar/px4/PX4-Autopilot` (1.9 GB). Chosen over ArduPilot SITL
  because PX4 ships the Gazebo Harmonic model used by this workspace
  (`gz_x500`), and over jMAVSim because PX4 main no longer carries it.
* Build: `make px4_sitl gz_x500` → `build/px4_sitl_default/bin/px4` (58 MB).
  The build **requires no root** on this host: `kconfiglib` is pip-installed,
  and PX4's kconfig helpers come from `PYTHONPATH=$PX4/Tools/kconfig` (PX4's
  `Tools/setup/ubuntu.sh` would have used apt, which this host cannot run —
  there is no passwordless sudo, so `ros-humble-mavros` is *not* installable
  here either; see "What is still blocked").
* The FCU runs with the Gazebo bridge attached and answers MAVLink on
  `udp 127.0.0.1:14580` (`px4_boot_excerpt.log`).

## The three faults between "PX4 runs" and "PX4 flies"

1. **The vehicle was never in the Gazebo world.** PX4's gz_bridge logs
   `Spawning Gazebo model` and `world: default, model: x500_0`, but
   `gz model --list` showed only `ground_plane`. PX4 therefore had no IMU/GPS
   and the estimator never converged: `Preflight Fail: height estimate not
   stable`, plus `No connection to the GCS`. Cause: this host runs **gz-sim
   8.15.0 (SDF 1.10)** while PX4 main's Gazebo models are **SDF 1.11**, and the
   entity insert is dropped without an error.
2. **`gz sdf -p` is not a workaround.** It cannot resolve
   `<include><uri>model://x500_base</uri></include>` ("Tried to use callback in
   sdf::findFile(), but the callback is empty"), so the "resolved" model keeps
   the plugins and has **no links** - a vehicle with no body, no rotors, no
   sensors. Spawning that produced an entity the estimator could not use.
   `scripts/px4_sitl_model.py` now resolves the include graph itself
   (9 links, 8 joints, 4 rotor plugins, 4 sensors, no `<include>` left) and
   posts one entity to the world's `create` service. With the vehicle present
   the estimator converges: **GPS fix type 3, 10 satellites, 0.7 m eph**.
3. **The rotor command never reached the model.** PX4 publishes
   `/x500_0/command/motor_speed`; an entity inserted through the create service
   subscribes on `/model/x500_0/command/motor_speed`. The vehicle is armed and
   in OFFBOARD but never lifts off. Neither a `gz topic` republish pipe nor
   rewriting the plugins' `commandSubTopic` to the absolute FCU topic fixed it
   in this session; the vehicle *does* carry `rotor_0..rotor_3`.

## What is now measured (`flight_attempt3_armed.json`)

* **The FCU arms.** Keeping the setpoint stream alive *while* the arm request is
  evaluated is what makes it work: PX4 leaves OFFBOARD about half a second after
  the last setpoint, and the arm check then runs in a manual mode and is
  refused. Streaming the hold through the arm request gives
  **`fcu_base_mode = 145` (ARMED | STABILIZE | CUSTOM)**, **OFFBOARD accepted**
  (`custom_mode = 393216` = nav_state 6 << 16), and `COMMAND_ACK` result 0 for
  `MAV_CMD_COMPONENT_ARM_DISARM`. The same script armed nowhere when it stopped
  streaming to wait for the ACK.
* **Takeoff is still not achieved.** Altitude stays at ~0.0–0.05 m through the
  ramp, the three waypoints and the land command, because of fault 3 above.
  Nothing about hover, waypoint tracking, landing or the command-loss failsafe
  is claimed.

## Measured interface state (`flight_attempt2_with_arm_params.json`)

`px4_offboard_flight.py` is a minimal offboard client (pymavlink 2.4.50, no
mavros). What is verified against the live FCU:

| item | measured |
|---|---|
| MAVLink link | heartbeat + telemetry received, 21 641 packets |
| position estimate | 47.397971 N, 8.546164 E, `landed_state = 1` (ground) |
| `PARAM_SET` + read-back | `COM_ARM_WO_GPS = 1.0`, `COM_ARM_AUTH_REQ = 0.0` |
| OFFBOARD mode | **accepted**: `fcu_custom_mode = 393216` (= nav_state 6 << 16) |
| setpoint streaming | 20 Hz `SET_POSITION_TARGET_LOCAL_NED` sent throughout |
| arming | **refused** — `fcu_base_mode` stays 17 (no 0x80 armed bit) |
| altitude | ~0.02 m for the whole attempt; the vehicle never left the ground |

Attempt 1 (`flight_attempt1_no_gps_param.json`) is the same flight without the
arm parameters: identical outcome, so the refusal is not the GPS arm check
alone. PX4's own log is explicit — `WARN [health_and_arming_checks] Preflight
Fail: height estimate not stable` and `Preflight Fail: No connection to the
GCS`: **the SITL EKF never converges a usable height fix in this session**, so
the FCU refuses to arm regardless of the offboard setpoints being streamed.

## The remaining fault is a Gazebo/PX4 version skew

Reading the PX4 sources pins the topic contract exactly:

* `GZMixingInterfaceESC.cpp`: rotor speeds are **published** on
  `/<model>/command/motor_speed`;
* `GZBridge.cpp`: sensors are **subscribed** on
  `/world/<world>/model/<model>/link/...`;
* `ROMFS/px4fmu_common/init.d-posix/px4-rc.gzsim`: the vehicle is spawned as a
  *scoped* entity (`name:` in the create request, an absolute `file://` include
  that the server resolves) so the rotor plugins' relative `commandSubTopic`
  resolves where the FCU publishes.

Every arrangement was measured:

| vehicle inserted as | rotor topic | simulated? | FCU sees sensors | lifts off |
|---|---|---|---|---|
| PX4's own spawn (scoped) | `/x500_0/command/motor_speed` ✓ matches | yes | yes | **no** |
| ours, flattened, unscoped | `/model/x500_0/command/motor_speed` | yes | yes | no (mismatch) |
| ours, PX4 request form (scoped) | `/x500_0/command/motor_speed` ✓ matches | **no** | no | n/a |
| ours, unscoped + rotor topic rewritten | absolute path ignored | yes | yes | no |

With PX4's own vehicle the topics match by construction, the estimator converges
(GPS fix type 3, 10 satellites, 0.7 m eph), the FCU **arms in OFFBOARD**
(base_mode 145, custom_mode 393216, COMMAND_ACK result 0) and streams 20 Hz
setpoints — and the rotors still produce no lift. That is the signature of a
plugin/plant that loads but does not act: this host runs **gz-sim 8.15.0 (Harmonic,
SDF 1.10)** while PX4 main's Gazebo models are **SDF 1.11** and target a newer
Gazebo (the server logs `XML Element[gz_frame_id] not defined in SDF` while
loading the model). The FCU side is healthy; the Gazebo side is not.

`scripts/px4_sitl_model.py` and the include flattener stay in the tree: they are
what makes the estimator converge at all on this host, and they are what a
Harmonic-matched PX4 would need if its own spawn is still unreliable.

## Attempt 4: a Harmonic-matched PX4 (v1.16.2) still does not lift

The version-skew hypothesis was tested directly: PX4 **v1.16.2** (the newest
line whose Gazebo models are SDF 1.9) was checked out with its submodule,
built, and flown
(`flight_attempt4_px4_v1_16_2.json`). The model loads on gz-sim 8.15 with no
SDF-version complaints, the estimator converges, the FCU arms in OFFBOARD
(`base_mode 145`) - and altitude still stays within +/-0.07 m of the ground
through the takeoff ramp, all three waypoints and the land command.

So the skew is not the whole cause either: the FCU is healthy, the model
matches the installed Gazebo, the topics match, and the rotors still produce
no thrust on this host. What is left is inside the Gazebo-side rotor actuation
for this model (the MulticopterMotorModel system plugin under gz-sim 8), which
this session could not repair without the missing dev-time debugging budget or
a newer Gazebo. No flight is claimed, and R5.4 stays `blocked`.

## What is still blocked (and how to unblock)

* **Takeoff is not achieved.** No altitude was measured above the ground, so
  nothing about hover, waypoints, landing or the command-loss failsafe can be
  claimed. The unblock is FCU-side and concrete: give the SITL EKF a valid
  source before arming — keep the session running until
  `vehicle_local_position.xy_valid`/`z_valid` are true, and/or set
  `EKF2_EV_CTRL`/`EKF2_HGT_REF` (or start the model with the Gazebo GPS sensor
  enabled) so the "height estimate not stable" preflight check clears.
* **No MAVROS interface.** `ros-humble-mavros` needs apt/root, which this host
  does not grant (`sudo -n true` fails). Two rootless options exist and are not
  yet chosen: (a) keep the pymavlink bridge above and bind
  `mavros_offboard_controller.py` to it, or (b) build the Micro XRCE-DDS agent
  from source and use PX4's uXRCE-DDS client with `px4_msgs`. The choice is
  recorded as open per the roadmap's "select/pin one FCU-SITL integration with
  license/dependency decision".
* `src/robot_lab_adapter/robot_lab_adapter/mavros_offboard_controller.py` is
  still the old stub (zero-thrust attitude target, `type_mask = 0`), and
  `quadrotor_sitl`'s URDF rotors remain fixed links — both are untouched by
  this increment and must not be described as flight.

## Reproduction

```bash
cd /workspace/molar/px4/PX4-Autopilot
export PYTHONPATH=$PWD/Tools/kconfig:$PYTHONPATH
make px4_sitl gz_x500 -j10           # builds, then launches FCU + Gazebo
python3 px4_offboard_flight.py --port udpout:127.0.0.1:14580 --out flight.json
```
