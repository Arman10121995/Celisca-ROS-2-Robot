# Native Panda measured object targets — November 13, 2026

Codex owns this R5.7 continuation; Claude retains drone/import work. Native
Panda now publishes a source/plant-bound catalog of actual free boxes and
measured collision poses. The Arm page explicitly selects an object, generates
geometry candidates and requests collision-aware IK/OMPL approach, grasp or
lift plans. Execution remains a separate existing command. See the
[operator procedure](../tutorials/panda_arm.md#select-a-measured-object-and-plan-grasp-stages).

Candidates respect the source 80 mm jaw opening, projected box width and
current planning padding. Set gap field changes only the Hand input; opening,
closing and execution each require their existing explicit control. Lift
requires measured opposing contacts and acknowledged planning attachment.
Attachment does not change physical constraints. Source gains, transmissions,
joint limits and robot/object poses are not modified by selection or planning.

The compiled collision geometry supplies the object pose, including any geom
offset from its body's origin, and full box dimensions. Static/compound/mesh
bodies are excluded with reasons. Selecting another object requires idle
controllers and release/detachment of the previous object. The native contact
observer waits for a matching acknowledged object scene before switching.
Known canonical-world collision IDs are retired from the planning snapshot
when their measured object replaces them. Unknown bindings leave planning
unready; restarting after an invalid binding may be necessary. This is a
selected-object adapter, not continuous tracking of every dynamic world body.

Two fixture publication defects are repaired: Panda no longer accesses an
uninitialised hand-only `object_size` attribute, and native hand geometry
publication no longer tests a NumPy vector's truth value or publishes box half
extents as full dimensions. Existing source models remain unchanged.

The [focused run](evidence/arm-objects-2026-11-13/checks-final.log) passes
**14 checks**. It covers actual MuJoCo box/offset geometry, no selection qpos
writes, scene ACK/binding contracts, contact/idle refusals, candidate geometry,
real ROS IK request fields and both original fixture publishers. Scene clients
are test doubles for request/acknowledgement checks; this does not establish a
running MoveIt pick/place mission. Optional source-fixture tests ran locally
with the installed pinned Panda and Robotiq models.

Four packages [build](evidence/arm-objects-2026-11-13/build-final.log): utils,
bringup, MuJoCo and GUI. The installed Xvfb
[widget-only check](evidence/arm-objects-2026-11-13/gui.log) exercises candidate
creation, padding, field-only editing, explicit plan dispatch, attachment
requirements and refusal of moved targets. It runs no physical trajectory.
[Source hashes](evidence/arm-objects-2026-11-13/source-manifest.json) identify the
implementation; raw logs remain on SSD at
`/workspace/molar/robot_lab_runtime/arm-objects-20261113/`.

R5.7 remains partial: complete repeated physical pick/transfer/place, mesh and
compound objects, perception, source-specific other grippers, coupled arms and
other backend controllers are still open. Historical Panda reach/grasp
certificates remain scoped to their original revision and fixture.
