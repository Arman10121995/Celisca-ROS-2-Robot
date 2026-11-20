# MOLAR Open-RMF return clearance (2026-11-20)

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

**Physical round trip remains pending.** The launch waited up to 180 s
for the other Gazebo trial to finish, then exited before starting a second
simulator. The prior physical result is scoped to the original layout.
The [manifest](sha256.txt) binds the revised source and SDK plan to this
check, starting from `b72ed737`.

Reproduce from the revised installed packages with the
[SDK and Gazebo probes](../r11-molar-rmf-2026-11-20/README.md#reproduce).
The runtime traces and plans for this layout are kept under
`/workspace/molar/robot_lab_runtime/rmf-molar-return-20261120`.
