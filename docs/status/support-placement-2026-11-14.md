# Native Panda support-surface placement — November 14, 2026

Codex continues R5.7; Claude's drone/import work is preserved. The existing Arm
grasp-target panel now lists measured fixed horizontal box tops and calculates
an above-surface and lowering target for a held box. See the
[operator procedure](../tutorials/panda_arm.md#choose-a-placement-surface-instead-of-guessing-coordinates).

The physical source catalog excludes robot bodies and every box with a moving
ancestor, plus sloped/mesh/plane surfaces. Sizes and transforms come from the
compiled model, not catalog labels. Surface listing is bounded to 256 entries;
box tops must have at least 4 cm sides and height 0.05–2.5 m. Selection is not
a reachability claim or a guarantee that the entire scene is tracked.

The scene adapter publishes the actual object-in-tool transform calculated
from measured FK/object state only after MoveIt acknowledges attachment.
Detach clears it after acknowledgement. Placement requires that transform to
match the current object's ID, source and plant, plus fresh object/catalog
data, live opposing contacts and a ready scene. The object retains its measured
orientation. Its projected footprint must fit the selected support inside the
declared edge margin. Inverting the measured rigid grasp relation gives the
TCP goal; the object centre is not assumed to equal the tool origin.

Editing the surface/gap commands no movement. **Plan above surface** and
**Plan lower** invoke the existing collision-aware IK/OMPL path only. Execute
Plan and Hand Open remain separate explicit controls. The default 30 mm
release gap leaves the free object to settle under gravity; no physical weld
or object-pose write is introduced. Existing source actuation, collision,
heartbeat and cancellation rules remain.

Focused tests cover rotated grasp transforms, projected support height and
fit, invalid/sloped geometry, source/plant mismatch, real compiled geometry,
dynamic-ancestor exclusion, actual ROS scene request/acknowledgement contracts
and explicit GUI plan requests. The Tk check constructs the real widget with
a protocol test double: editing dispatches nothing, the two explicit buttons
dispatch two plan requests, and loss of attachment disables them. This is
software evidence, not a simulator or physical placement mission. All **25
focused checks** pass under Xvfb; the two GUI checks also pass against the
installed packages. Utils, bringup, MuJoCo and GUI build successfully.

Compact logs and source hashes are in [the evidence directory](evidence/support-placement-2026-11-14/README.md).
Raw output remains at
`/workspace/molar/robot_lab_runtime/support-placement-20261114/` on SSD.

Remaining R5.7 implementation includes unattended multi-stage sequencing,
placement/retreat perception, more geometry and grippers, coupled arms and
other backend controllers. A complete source-robot physical
pick/transport/place/release mission remains unqualified.
