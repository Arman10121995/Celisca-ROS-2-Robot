# R11 MOLAR payload transfer to a station: Gazebo and MuJoCo (November 15, 2026)

**Status:** implemented, experimental. Measured 3/3 in Gazebo and 4/4 in
MuJoCo on `nav_empty`.

These are simulator runs with a primitive payload and station, not a physical
or CELISCA-floor qualification.

## The check

`scripts/molar_station_transfer_check.py --simulator gazebo|mujoco` launches
`molar_hri` in display mode and drives it only through the normal
articulation interface: `/molar/articulation/*`, the current plant ID and a
10 Hz heartbeat. The sequence:

1. A 0.20 × 0.15 × 0.10 m, 0.5 kg box rests on the deck, 0.10 m behind the
   plate centre.
2. The deck lifts by 0.10 m.
3. Both coupled telescope stages extend to 0.14 m.
4. A static station table sits end to end with the extended deck:
   - near edge 1 cm past the deck end;
   - top 5 mm below the deck;
   - friction 0.05, modelling a passive roller table.
5. The roller runs at 8 rad/s, so the deck belt surface moves at
   8 × 0.012 = 0.096 m/s.
6. The belt runs until the box is entirely on the station, or has stopped
   moving for 2 s.
7. The roller stops; the telescope retracts and the deck lowers.

**Pass criteria.** After retraction and lowering, the box must:

- rest on the station: resting height ±2 cm and inside its footprint;
- have its centre past the deck end;
- have been dragged less than 1 cm by the retraction.

Truth sources:

- **Gazebo:** `/world/<w>/pose/info`. Link poses there are relative to the
  model and are composed with the model pose.
- **MuJoCo:** the new `/mujoco/free_bodies` truth topic plus
  `/odom/ground_truth`.

## Results

| Run | Box centre from base | Rear edge past deck end | Retract drag | Result |
|---|---|---|---|---|
| Gazebo 1 / 2 / 3 | 0.671 / 0.671 / 0.671 m | 3.6 / 3.4 / 3.4 mm | 0 | pass ×3 |
| MuJoCo 1 / 2 / 3 / 4 | 0.666 m each | 8.1 / 8.1 / 7.9 / 8.1 mm | 0 | pass ×4 |

Files: `gazebo_*.json` and `mujoco_*.json` (each records its steps, belt
command trace and channels), `mujoco_scene_nav_empty.xml` (the prepared
MuJoCo scene) and `launch-log-sha256.txt` (the logs stay on SSD under
`robot_lab_runtime/molar-transfer-20261115`).

## Fixes made to get here

The failed attempts are kept on SSD.

**Gazebo belt did nothing.** Fortress's system DART plugin lacks the contact
callback TrackController uses
([gz-sim#2495](https://github.com/gazebosim/gz-sim/issues/2495)). Belt
commands arrived but moved nothing, whatever their sign.

- Fix: `gazebo.launch.py` now selects the private Physics 5.4 DART plugin
  that `scripts/provision_tracked_physics.py` builds for the native tracked
  vehicle, whenever a description has a TrackController.
- The check is `gazebo_physics_world.track_physics_engine()`: the receipt
  hash must match.
- Without the plugin, the launch prints that belts and tracks will not move.

**Gazebo belt moved the box the wrong way.** TrackController moves bodies
resting on top of the track opposite to its command, since its surface
motion is the ground contact's.

- Fix: `molar_gazebo_control` publishes the negated surface speed, so a
  positive roller carries the payload to +x as in MuJoCo.

**MuJoCo used the payload as the robot.** `_find_body_and_joints` took the
first free joint as the robot, and world bodies come first in the merged
model.

- Fix: the spawner now uses the `<robot_name>_freejoint` that
  `_add_floating_base` creates (`test_spawner_robot_body.py`).

**MuJoCo belt destabilized the contacts.** The zero-friction,
high-priority deck contact (pyramidal cone) blew up: the box reached
20 m/s in one step.

- Fix: `ConveyorBelt` now gives the deck normal-only contacts (condim 1) on
  its collision geoms.

**Script errors.**

- The first station placement (0.40 m ahead of the base) blocked the
  extending deck. The station now meets the extended deck end to end.
- A static 0.8-friction station held the box with 4 mm (Gazebo) or 31 mm
  (MuJoCo) still over the deck. The station is now a 0.05-friction passive
  roller table.

**New interfaces.**

- `mujoco_world_xml:=<file>` lets a prepared MJCF scene override the map
  lookup.
- `/mujoco/free_bodies` gives truth for loose objects in any MuJoCo session.

## Limits and what remains

- `nav_empty` only: one payload, one station geometry, a fixed 0.10 m lift.
- The MuJoCo station is pre-placed from the Gazebo-measured deck geometry;
  the base settled about 8 mm from its nominal spawn.
- Not covered: PyBullet and Isaac belts, a Nav2 route to a CELISCA station,
  multi-floor operation through the fleet lift, and calibrated belt or
  roller parameters.
