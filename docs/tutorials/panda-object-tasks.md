# Automatic native Panda object tasks

## Run

Use the normal native Panda / MuJoCo / Display launch with MoveIt enabled.
The `mobile_manipulation_lab` map supplies a free cube and measured bench
supports. In **Arm → Object grasp targets**, explicitly select a named free
box and wait for the measured collision-scene acknowledgement. Generate a
grasp candidate and choose a measured placement support.

The candidate list includes source-finger-aware upper pinches when the
compiled source bounds allow them. These give the fingers more clearance
above a support while preserving the normal 1 cm planning padding. Centred
pinches remain available. Geometry candidates still need valid IK, collision
plans and actual grasp contacts.

In **Automatic pick / place**, choose 1–5 cycles, a **Lift / approach height**
of 2–15 cm (default 5 cm) and press **Run pick / place**. A pose that is
reachable at the bench can still exceed the arm's workspace when lifted;
choose the clearance for the actual source geometry and selected support.
This explicitly authorizes planning and executing the complete task. It:

1. Opens the fingers, plans/executes approach and grasp, then closes.
2. Waits for measured opposing contacts and acknowledged planning attachment.
3. Plans/executes lift and checks one simulated second of unsupported grasp.
4. Plans/executes approach above the selected support and lowers the object.
5. Opens, verifies sustained physical release on that support, detaches the
   planning object and plans/executes a retreat.

Further cycles pick the same named box from its current measured pose and
place it on the selected support again. No body or object pose is written.
Unreachable, colliding or expired plans stop the task. A completed action alone
does not establish grasp or release. Physical contact force, relative
translation/rotation, environmental support and settled object speed provide
those observations; the whole box must fit the selected support.

**Stop task**, Arm/Hand Stop or Cancel, Drive/Space, Reset, manual motion,
target/selection/force/gap/height changes, lost ownership and stale feedback interrupt
the sequence. Stop preserves the original native controlled-hold behavior.
After interruption, inspect the measured state and start deliberately again.
There is no automatic resumption from an unknown grasp.

Each task is bound to object ID, robot source, current plant and selected
support. It has 60 wall seconds per stage and 1,200 seconds total, with at
most 128 stages. The existing planning and native action deadlines remain
tighter where applicable. Actual stage receipts go to
`$ROBOT_LAB_RUNTIME_ROOT/panda-object-tasks/<id>.jsonl` on SSD. A terminal
`measured_stages_reached` journal is not a platform qualification certificate.

This first implementation handles the native Panda and single physical boxes.
Other geometry, grippers, backends and perception-driven Panda grasps remain
separate integrations. Source-specific failures and current robot mission
results are recorded in the roadmap and current status.
