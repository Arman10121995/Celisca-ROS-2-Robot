# TIAGo on Isaac: the block no longer tips; the grasp now drifts (November 18, 2026)

**Status: implemented, experimental; TIAGo fetch on Isaac still not passed.**

## The block-tipping cause (found with link truth)

**New diagnostic: true link poses on Isaac.**

- The Isaac runtime can now report world poses of named links
  (`trace_links`). The spawner publishes them on `/simulation/link_poses`.
- The launch sets the links for robots with a `pick_place` block: the tool
  body and the finger links.
- `urdf_pick_place.py` then writes the true TCP beside its kinematic
  estimate into the 2 Hz trace.

**Findings** (`isaac1.json`):

1. **Kinematics are fine.** The true TCP matched the estimate within about
   1 cm through the whole move (0.1 mm at rest).
2. **The route was wrong.**
   - The hand reached the block moving sideways in −y at block-top height
     (TCP z 0.63 m; the pads reach 9 cm below the TCP).
   - It was 2.3 cm off-centre along the fingers' closing axis, so one finger
     caught the block's edge and pushed it over.
   - The planner's up-and-over vias (0.15 and 0.30 m above the pre-grasp)
     were out of TIAGo's reach. The route then fell back to a direct joint
     path, and the only object check was penetration (> 2 mm deep).
3. **Why MuJoCo got away with it:** it tracks the plan closely enough to pass
   within millimetres. Isaac does not.

## Planner changes (`robot_lab_mujoco/arm_pick_place.py`)

**Clearance on transit legs:**

- Routes that may not touch the object keep every robot part at least
  15 mm from it (`OBJECT_CLEARANCE_M`).
- The object geom gets a 15 mm margin, so near-misses are detected.
- Moves that allow finger contact (descent, lift, carrying) keep the
  penetration rule.

**Up and over, then down:**

- A high via may now be reached by any route: direct, other vias, or the
  bounded search. Before, only two straight joint segments were tried, and
  the approach was dropped if the first one failed.
- The last short drop into the pre-grasp keeps only the penetration rule,
  because that pose straddles the block's top.

**A reachable via:** a new 0.10 m via lifts TIAGo's fingertips about 4 cm
above the block, within its reach.

**Tried and reverted (`isaac2/3.json`):**

- clearance alone; the sideways route stayed, because the pre-grasp itself
  is only 12.7 mm from the block;
- a pre-grasp with the fingertips clear of the top. It broke the MuJoCo
  planner test, and the gripper's own hull came within 1 mm of the block.

**Also added:** a stage timeout now names the worst joint and its error.

## Result on Isaac (`isaac4.json`)

| Stage | Result |
|---|---|
| Drive to the pickup table | passed |
| Over the top to the pre-grasp | passed, block untouched |
| Descent | passed |
| Close | **timed out** |

**The close timeout:**

- Both fingers squeeze the block (0.019–0.021 m each, block 40 mm).
- The hand and block then creep +x along the closing axis, 1.8 cm in
  30 s.
- The wrist roll joint (`arm_7_joint`) ends 0.024 rad off its target, so the
  stage never settles.

This looks like the asymmetric finger drift PyBullet had before its
symmetric grip gear. **Next:** couple TIAGo's fingers on Isaac, for example
with a PhysX mimic or gear joint.

## Regressions with the new planner (all pass)

| Robot | Backend | Time | Place height error |
|---|---|---|---|
| TIAGo fetch | MuJoCo | 109.1 s (also 107.3 s with the clearance alone) | −0.1 mm |
| TIAGo fetch | PyBullet | 99.9 s | 0.0 mm |
| URDF Panda | MuJoCo | 24.3 s | −0.1 mm |
| URDF Panda | PyBullet | 24.7 s | 0.0 mm |
| URDF Panda | Gazebo | 27.3 s | 0.0 mm |

All runs were made while Go2 stairs training loaded the CPU. Launch logs are
on SSD under `robot_lab_runtime/isaac-tiago-truth-20261118`.
