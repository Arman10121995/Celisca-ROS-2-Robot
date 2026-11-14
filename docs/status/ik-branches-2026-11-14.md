# Bounded alternate source IK configurations — November 14, 2026

Codex continues ROADMAP R5.9 in the native manipulation files. Claude owns the
algorithm/seed/launch lane. The [normal GUI task workflow](../tutorials/native-object-tasks.md#what-the-planner-actually-does)
automatically tries alternate source-control IK configurations after a
measured-seed solution is unavailable, collides or has a blocked direct path.
No launch flag, source-robot change or mode-support promotion is introduced.

Up to six starts comprise the measured controls, source-range midpoint and
four seed-0 samples of reviewed movable controls. Each must pass original
source FK pose tolerances, coupling and limits. Duplicate configurations and
colliding endpoints are excluded. Head/base/gripper and keep-mode wrists
remain held. The measured-seed solve has at most two seconds; alternatives
have up to 0.5 seconds each, with three seconds reserved for configuration
search under the default original eight-second total worker budget.

An inspected direct motor path is preferred. Otherwise all accepted endpoints
seed one bidirectional RRT-Connect goal forest. Directed edge validation
retains the existing controller's actual per-channel motor slew and measured
payload relation in private data. The original 384-node, 2,048-edge,
12,000-collision-sample and 64-waypoint bounds remain. The task journal records
real attempt outcomes, selected attempt and available goal count; the GUI
execution phase names an alternate attempt. An attempt or geometry solution
does not certify a grasp or physically completed mission.

The [43 focused checks](evidence/ik-branches-2026-11-14/checks.log) pass,
including real Tk recipe construction and prior source/pose/task contracts.
New cases cover blocked and disconnected first goals, directional multi-goal
search and budget interruption during optional shortening. An explicitly
synthetic MuJoCo source-control fixture has a redundant wrist and a physical
box tool extension. Its initial IK endpoint collides with a box obstacle; a
different actual IK configuration is selected. An independent 101-sample
audit of each returned motor segment checks native blocker contacts, target
position, unchanged live qpos/gains and held grip. This is native geometry
evidence, not installed Stretch dynamics or a full physical pick/place mission.

Initial failures are preserved on SSD. One found route was lost when optional
shortening exceeded the caller's collision budget; a specific budget exception
now ends shortening while retaining only the previously inspected route.
Required search failures and unrelated callback errors still propagate. The
fixture was also refined to separate an unrelated free object from the tested
route and use box geometry with an unambiguous native collision, rather than a
capsule contact at the one-millimetre inspection threshold. No source collision
threshold or actuator limit was relaxed to pass the test.

Utils, MuJoCo and GUI [build](evidence/ik-branches-2026-11-14/build.log).
[Source hashes](evidence/ik-branches-2026-11-14/source-manifest.json) identify
the implementation. Raw output remains on SSD at
`/workspace/molar/robot_lab_runtime/ik-branches-20261114/`.
Exhaustive IK, perception, additional grasp strategies, further robot/backend
adapters and complete loaded source-robot missions remain open.
