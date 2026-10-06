# Native Panda Cartesian control — October 6, 2026

A real Tk GUI launch using the normal installed profile completed on Jetson
AGX Orin, ROS Humble / MoveIt 2.5.10 / MuJoCo. Scope is native
`menagerie_franka_emika_panda`, Display, static `nav_empty`. Source baseline
`61335fe` is supplemented by pre-trial source/installed hashes, the exact
native asset hashes and the installed MoveIt plugin/version manifest.

## Measured result

The normal GUI autofilled `arm_control:=panda arm_planning:=moveit`. Plan and
Execute sent a real MoveIt plan to the original native actuator controller.
An independent observer measured native hand-body TF and the source-derived
TCP offset (0.1029 m). FK agreed with the measured TCP within 3.2e-12 m.
The selected-world scene acknowledged nine collision geometries.

| Offset | Physical TCP position error | Orientation error | Planned simulation duration |
|---|---:|---:|---:|
| +0.05 m z | 0.00732 m | 0.831° | 0.821 s |
| +0.04 m y | 0.00659 m | 0.837° | 0.361 s |

Acceptance remains 0.02 m / 5°, fresh physical TF and actual displacement above
0.02 m. The floor-colliding pose was rejected by FCL; a self-colliding pose
produced actual native contacts between body 2 and both fingers, and FCL also
rejected those pairs. Blocked and unreachable planning requests returned no
waypoints and no physical motion.

A real joint jog changed a joint 0.01979 rad and a Hand closure moved a finger
0.01448 m; both removed the cached plan and disabled Execute. Invalid GUI
input caused less than 0.00001 rad drift. Cancel and Stop held measured joints;
heartbeat loss aborted in 1.001 wall seconds. All held velocities stayed below
0.03 rad/s and drift below 0.02 rad. GUI Reset removed its cached plan,
restored Home within 0.00659 rad and returned the physical TCP within 0.004 m
without rewinding simulation time. Live Monitor received clock updates while
controls ran. MoveGroup, scene publisher, native simulator and RViz exited
cleanly; root exit was zero.

The preceding seeded-IK trial also passed (0.00693/0.00654 m and
0.814/0.792°). Earlier pose-only planning runs retain their own source stage;
they do not qualify the final seeded request.

## Implementation and retained failures

MoveIt KDL solves IK seeded by measured joints with collision avoidance;
OMPL/FCL checks the joint path against native model geometry and the selected
static SDF world. All original collision exclusions and 0.005 rad limit margins
remain. Cubic retiming stays on the checked joint edges, targets at most
0.35 rad/s and rejects paths outside the native 15 s action contract.
No joint or robot pose is written to make planned motion pass.

Trials 1–4 retain interface/soft-limit/stale-observer failures and MoveGroup
shutdown crashes. This host exhibited a plugin-lifetime crash matching the
[upstream report](https://github.com/moveit/moveit2/issues/1597).
The launch now keeps its real plugins resident only in MoveGroup's process;
physical trials 5, 6, 9 and the normal-profile trial 10 then exited cleanly.
Trial 7 clicked Hand Open while disabled after Cancel; the producer now waits
for the visible button state. Trial 8 rejected a pose-only plan longer than
15 s, exposing unnecessary redundant IK branches for a small offset; measured
joint seeding repaired that without increasing the limit. Retained logs and
negative reports are indexed in [manifest.json](manifest.json).

## Artifacts and limits

[report.json](report.json) contains actual physical measurements;
[producer.py](producer.py) drove the real GUI and independent observer.
[source-manifest.json](source-manifest.json) records the pre-trial source,
installed modules, normal profile and all native asset files;
[sdk-manifest.json](sdk-manifest.json) records MoveIt package versions and
plugin hashes. Full independent trace, source snapshots and raw ROS/GUI logs
remain on SSD under
`/workspace/molar/robot_lab_runtime/extensions-2026-10-06/panda-planning/physical-gui-10-normal`.
Their hashes and preceding trials' hashes are retained in the manifest.

The support guard recomputes numeric acceptance and requires matching native
XML/resources, planner/controller/world source and report checksums before
provisioning `arm_planning: moveit`. Unit fixtures are not mission evidence.
Read the [GUI guide](../../../tutorials/panda_arm.md).

This qualifies two named targets and rejection/interruption/reset screens.
Actor/heightmap scenes, other maps/robots/backends, Servo, attached payloads,
the free-cube grasp fixture in MoveIt and arbitrary repeated pick/place remain
open. The separate proven joint/Hand cube-grasp path stays available.
