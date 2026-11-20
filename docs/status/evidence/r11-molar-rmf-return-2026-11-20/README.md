# MOLAR Open-RMF return clearance (2026-11-20)

**Measured round trip:** one MOLAR in Gazebo Fortress on the revised
`fleet_facility` completed L0 → L1 → L0 through the normal Open-RMF
patrol interface. The probe submitted two patrols so it could independently
measure the first arrival before starting the return.

This follows the [first SDK lift trial](../r11-molar-rmf-2026-11-20/README.md),
where MOLAR left the car on L1, then the scan guard stopped its turn toward
the closed L1 gate. The native workflow opens the gate before commanding
boarding. Humble's [building graph parser](https://github.com/open-rmf/rmf_ros2/blob/humble/rmf_fleet_adapter/src/rmf_fleet_adapter/agv/parse_graph.cpp)
creates the lift session event on the entry lane. A local SDK planning probe
still inserted the turn before that event when the event was moved to a
same-position intermediate lane. That graph experiment was not deployed.

The generated `fleet_facility` upper platform extends 0.70 m west, from
`x = 1.00` to `x = 0.30` m. Its rail collision models and
`map_L1.pgm` use the same generator dimensions. The lower floor, car,
gate and native transport locations stay where they were. MOLAR's RMF route
now has a separate `L1_approach` at (1.75, 0) m, while the settled
`L1_landing` is still (2.10, 0) m. The graph requires clearance for both
and retains the native lift-session events, leveling, and scan guard.
The directed retreat lane requires forward travel from the exit to the
waiting point. This keeps the robot facing west on that leg: without the
constraint, RMF planned to turn at the closed gate and reverse to the
waiting point.

The four focused fleet suites pass (43 checks). The actual Humble SDK
produces a downward plan whose first movement is from (2.10, 0) m to
(1.75, 0) m with the heading held at π. It then schedules the turn at
(1.75, 0) m, before boarding. The maps and utils packages build. The
initial maps build referenced the removed version 8.7.1 of the system
Ignition messages library; reconfiguring CMake picked up 8.7.2.

**Gazebo run 1 on the widened layout:** MOLAR settled at L1 in 114.7 s at
(2.166, 0.002, 3.000) m, on `map_L1.yaml`, with both measured rates zero.
It then drove west clear of the gate to the staged approach. The position
goal for the turn again caused Nav2 to circle while its estimate shifted.
At the last sample it was at (1.797, −0.004, 3.000) m, still turning,
with the gate closed. The watcher stopped only this launch when another
simulator started. The run did not complete the return.

The subsequent fix sends **Nav2 Spin** at that heading waypoint. The same
dispatcher now handles both the cabin and staged approach, with an observed
floor, fresh localization and proximity to the declared station required.
The cabin alone still requires both doors open. A separate measured
position and heading check remains required after the Spin action.
The 43 focused checks pass with both Spin branches and the updated utils
package builds. Run 2 stopped at startup because another simulator
started; it provides no motion evidence for the new Spin dispatch.

**Gazebo run 3 with Spin:** MOLAR turned at the upper waiting point with
the gate closed. Between the first and last samples of that turn, its
heading changed from −3.105 to 0.002 rad and its center moved about 2.2 mm.
The SDK then opened the gate, sent MOLAR into the car and took it back to L0.

| Settled floor | Wall seconds from probe start | Position (m) | Distance from exit target | Active map |
|---|---:|---|---:|---|
| L1 | 115.1 | (2.174, 0.009, 3.000) | 0.075 m | `map_L1.yaml` |
| L0 | 223.7 | (2.167, 0.016, 0.000) | 0.069 m | `map.yaml` |

Both arrivals met the 0.15 m position and 0.15 rad heading limits and
had zero measured linear and angular rates. This is one simulated robot
without payload; it does not qualify a mixed fleet, payload carriage,
hardware or the GUI's combined patrol macro.

The [manifest](sha256.txt) binds the revised source and SDK plan to this
check, starting from `b72ed737`. The
[stage-turn manifest](stage-turn-sha256.txt) binds runs 1–3 and the final
runtime sources. The first launch attempt waited 180 s for a simulator
slot and exited without starting a trial.

Reproduce from the revised installed packages with the
[SDK and Gazebo probes](../r11-molar-rmf-2026-11-20/README.md#reproduce).
Run 3 used the documented session with a 370 s probe budget, ROS domain 115
and Gazebo partition `rmf_molar_stage_3_20261120`.
The runtime traces and plans for this layout are kept under
`/workspace/molar/robot_lab_runtime/rmf-molar-return-20261120`.
