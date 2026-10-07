# Native Panda after GUI consolidation, October 7

The normal installed Panda profile repeats the full physical MoveIt/native
actuator screen after introducing parent/variant selectors and the optional
SDF visual-reader path. The previous
[October 7 spawn regression](../panda-cartesian-2026-10-07/README.md) is retained.
This repeat used no experimental planning override.

| Actual GUI request | Independent physical TCP error | Orientation error |
|---|---:|---:|
| +0.05 m z | 0.00656 m | 0.771° |
| +0.04 m y | 0.00607 m | 0.762° |

Actual KDL/OMPL/FCL planning and native PD execution pass both targets, floor/
self-collision and unreachable rejection, physical joint/finger invalidation,
Cancel, Stop, missing heartbeats, monotonic Reset/Home, Live Monitor and clean
planner/scene/simulator/RViz exit. Root and producer exit zero. No pose writes
are used to produce motion. The native arm/hand actuator files and original
physical cube grasp certificate are preserved.

[Report](report.json), [actual producer](producer.py),
[pre-trial source/installed/native hashes](source-manifest.json),
[SDK hashes](sdk-manifest.json) and [SSD manifest](manifest.json) retain exact
provenance. The strict planning certificate uses the source-matched GUI,
SDF reader and actual report. The original source sweep omitted the map
package: the unchanged named world is checked against recorded HEAD and
installed bytes at collection, explicitly distinguished from pre-trial hashes.
Later inspector-only changes have their own rendering/live-plant evidence;
they do not replace this physical report's original fingerprints.

Static MuJoCo/`nav_empty` only. Servo, moving/attached objects, repeated
arbitrary pick/place and other maps/arms/backends remain unqualified.
