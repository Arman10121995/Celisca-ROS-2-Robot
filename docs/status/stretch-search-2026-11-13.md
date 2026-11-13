# Native Stretch configuration detours — November 13, 2026

Codex owns this R5.9 continuation; Claude retains drone/import work. The
normal local object recipes now try a direct source-control path, then bounded
seeded bidirectional configuration search when that edge is blocked. No new
launch flag or automatic support promotion is added. The task's current phase
and journal report whether the accepted path was direct or `rrt_connect`.
[Operator guide and budgets](../tutorials/native-object-tasks.md#what-the-planner-actually-does).

The search operates only on reviewed source lift/telescope/wrist coordinates.
The source FK adapter retains linear tendon/equality relationships and actual
position gains. Head, gripper compression and base stay held. Every candidate
edge inspects the existing executor's per-channel rate-clipped motor motion,
including channel stopping times. A measured held object's relative transform
is applied only to the private planning copy; real object motion remains
contact physics. Source limits, watchdog, live contact interruption, worker
generation/drift checks and the 64-waypoint executor bound remain in force.

This is a local implementation of the published bidirectional
[RRT-Connect algorithm](https://ompl.kavrakilab.org/classompl_1_1geometric_1_1RRTConnect.html),
not an OMPL binding. Goal-tree edges are validated in their actual execution
direction; asymmetric motor slew cannot be assumed reversible. Search is
finite and sampled: seed 0, 384 nodes, 2,048 edge queries, 12,000 native collision
samples and the original eight-second total planning deadline. No route inside
those budgets is an explicit refusal, not proof that a route cannot exist.

The [15 focused checks](evidence/stretch-search-2026-11-13/checks-final.log)
cover seeded detours/repeatability, directional edges, actual native
tendon/mimic FK, blocked endpoints, disconnected regions, source limits and
deadline refusal. A small explicitly synthetic MuJoCo lift/telescope fixture
adds a real obstacle between start and goal. The planner finds a detour; a
separate 101-sample-per-returned-segment contact audit follows the original
rate clipping and checks native blocker contacts. Original plant qpos/gains
and gripper control remain unchanged. This is a geometry/source-contract
check, not a dynamic or installed Stretch mission.

Two packages build: utils and MuJoCo. The [source manifest](evidence/stretch-search-2026-11-13/source-manifest.json)
and compact logs identify this implementation. Raw output remains on SSD at
`/workspace/molar/robot_lab_runtime/stretch-search-20261113/`.

Remaining R5.9 implementation includes orientation-aware/multiple-IK-branch
goals, perception poses, other source adapters and grasp strategies. A complete
loaded source-robot pick/transport/place mission still needs physical
qualification. A single fixed IK endpoint can itself collide; configuration
search deliberately refuses that endpoint rather than altering its meaning.
