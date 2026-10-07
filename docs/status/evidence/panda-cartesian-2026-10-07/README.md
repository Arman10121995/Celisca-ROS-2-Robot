# Native Panda normal GUI regression, October 7

This repeats the October 6 physical MoveIt/native-actuator acceptance after
adding optional robot-specific named-map spawn handling to the shared Launch
and GUI command paths for TurtleBot3. Native Panda has no such overrides; its
source geometry, controllers, planning scene and original actuator contracts
are preserved. The normal installed profile was used, with no experimental
planning flag override.

Jetson AGX Orin / ROS Humble / MoveIt 2.5.10 / MuJoCo, native
`menagerie_franka_emika_panda`, Display in static `nav_empty`:

| Actual GUI request | Physical TCP error | Orientation error |
|---|---:|---:|
| +0.05 m z | 0.00641 m | 0.755° |
| +0.04 m y | 0.00708 m | 0.798° |

The independent physical TCP observer, floor/self-collision and unreachable
rejections, actual joint/finger plan invalidation, invalid-input hold,
Cancel/Stop/heartbeat loss, monotonic Reset/Home, Live Monitor and owned-child
cleanup all pass the unchanged numeric acceptance. MoveGroup, scene publisher,
native simulator and RViz exit cleanly; the launch and producer exit zero.
No robot pose or joint state is written to make planned motion pass.

The new source-matched certificate points to this report and includes the
optional spawn helper. The original hand/cube-contact certificate remains
unchanged. [Report](report.json), [actual producer](producer.py),
[source/installed/native hashes](source-manifest.json), [SDK hashes](sdk-manifest.json)
and [SSD artifact manifest](manifest.json) are retained. The full trace, source
snapshot and raw logs remain at the run directory named in the manifest.
The concurrent independent TurtleBot3 trial used a different ROS domain;
this regression's real-time factor is not a performance benchmark.

The [October 6 negatives and implementation](../panda-cartesian-2026-10-06/README.md)
remain preserved. Servo, dynamic/attached-object scenes, repeated arbitrary
pick/place, other worlds and other robot/backends remain unqualified.
