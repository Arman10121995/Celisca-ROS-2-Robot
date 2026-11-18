# R10.2: doors, chargers and energy in a world without a lift (November 18, 2026)

**Status: implemented, experimental.** This closes R10.2's remaining item:
a physical door, docking and modeled energy in a world other than
`fleet_facility`, with no lift.

## What changed

**The infrastructure node works without a lift** (`fleet_infrastructure.py`):

- `lift` is optional. Without it, the facility has one floor, `L0`. Doors,
  docks and energy work as before. Lift transport, lift recovery and manual
  lift commands are refused ("This facility has no lift"), and no RMF lift
  state is published.
- Facility-specific geometry moved into the configuration:
  - the lift car's footprint is `lift.area` (it was hard-coded at x
    2.8–5.2 m);
  - Nav2 paths open a door when they cross its `path_zone` (it was
    hard-coded for the door named `main`).
- `fleet_facility/infrastructure.yaml` and its generator declare both
  values, unchanged.
- The session bridges, and the GUI's floor list and lift line, handle a
  facility without a lift.

**New world `fleet_ward`** (`scripts/generate_fleet_ward.py`):

- two 5 × 7 m rooms joined by one 1.6 m doorway;
- its sliding door is `fleet_facility`'s main door: the same panel, travel,
  gains and path-zone rule;
- two authored charger docks, `west_charger` and `east_charger`, with
  painted pads;
- an occupancy map, `sim_maps.yaml` and registry entries.

**Energy certificates re-bound to the installed readback library.**

- The source certificates in `infrastructure.yaml` bind the robot
  description and the SHA-256 of `libgz_hardware_plugins.so`.
- A system update on November 17 upgraded `ros-humble-gz-ros2-control` to
  0.7.21, so the old certificates no longer matched. Energy correctly
  stayed unknown in run 1: "Actuator source not qualified".
- A new source-bound readback (`power_probe.py` from
  `fleet-energy-2026-11-02`, unchanged) passed in this session:
  - Bumperbot: 835/835 complete samples, max effort 0.032 N m, positive
    power 0.059 W;
  - Labbot: 832/832 complete samples, max effort 0.058 N m, positive power
    0.190 W.
- Both certificates, in `fleet_facility` and `fleet_ward`, now carry the
  new library hash and cite this receipt.
- The RMF mechanical fits (`rmf_mechanical`) were made under the previous
  library build and were not refit.

## Measured (Gazebo, Nav2, two robots)

Session: Bumperbot `alpha` in the west room and Labbot `beta` in the east
room (`ward-session.yaml`). `ward_probe.py` runs two phases:

1. Both robots are dispatched at once through the one door, in opposite
   directions.
2. When both have arrived, each docks at the far room's charger through
   the infrastructure's `dock` command. Each must be docked (within pose
   tolerance and still for 1 s), with its modeled energy rising over 10 s.

| Run | Verdict | Crossing | Closest approach (radii 0.30 m) | Contacts | Door modes seen | Docked after | Modeled charge over 10 s |
|---|---|---|---|---|---|---|---|
| 1 | not passed: energy unknown (old certificates) | both succeeded; alpha waited for `door:ward` | 1.02 m | 0 | closed, moving, open | 22.7 / 22.8 s | unknown (120 W charging on) |
| 2 | probe verdict bug | both succeeded; alpha waited | 1.05 m | 0 | closed, moving, open | 21.2 s each | 0.7988 → 0.8048 (both) |
| **3** | **passed** | both succeeded; alpha waited | 1.05 m | 0 | closed, moving, open | 21.6 s each | 0.7988 → 0.8047 (both) |

**Run 2's verdict bug:** the dock commands dispatch operator tasks too.
Four successes appeared, and the probe's check expected exactly one per
member. The check now counts each member's first task as its crossing and
needs every operator task to succeed. Run 3 used the fixed probe.

In every run the door reservation worked as in `fleet_facility`: alpha was
held 1 m before the door zone while beta held it, then was granted the
zone and crossed.

## Reproduce

Source `scripts/ssd_env.sh`, ROS and the workspace, and use an isolated
`ROS_DOMAIN_ID` and `IGN_PARTITION`:

```bash
python3 launch_case.py ward-session.yaml $R/launch.log
CASE_OUTPUT=$R/run /usr/bin/python3 -s ward_probe.py
kill -INT -<launch pid>
```

Traces, session journals and launch logs are on SSD under
`robot_lab_runtime/fleet-ward-20261118` (launch-log hashes here).

## Limits

- One door and two docks, two robots, three runs. These runs share the
  host with Go2 policy training.
- Docking is a pose berth with an authored 120 W charge model; there is no
  connector contact.
- Native fleet control only: no RMF run in `fleet_ward`.
- Larger fleets, three-robot passages and loss remain V7.
