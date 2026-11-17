# Isaac pick and place: URDF Panda passes; TIAGo fetch does not yet (November 17, 2026)

**Status:** implemented, experimental. One run each.

## Loose objects on Isaac

**Spawner:** the Isaac spawner hands the runtime the world's loose
single-link models (`sdf_world.extract_dynamic_bodies`) and keeps them out
of the static shapes.

**Runtime:** builds them as `DynamicCuboid`s with their world-file mass and
friction (boxes only; other shapes are logged and skipped). It reports
their poses about every 0.1 s in its state events.

**Truth topic:** the spawner republishes them on `/simulation/free_bodies`,
the same truth topic as MuJoCo and PyBullet, so `urdf_pick_place.py` works
unchanged on Isaac.

## URDF Panda, Isaac, `arm_pick_place_lab` (passed)

`scripts/arm_pick_place_check.py`, `pickup_cube` → `place_table`, lift
0.10 m. **Passed in 20.2 s** (`isaac_panda.json`):

- **grasp:** fingers stop on the cube at 0.020 / 0.020 m;
- **lift:** rise 0.0997 m, hold drift 0.2 mm;
- **place:** height error 0.0 mm, inside the support, still.

The mechanical finger coupling has no Isaac adapter. The contract drives
the follower with the master's reference, and both fingers held the cube.

Tracebacks at the end of the log are `KeyboardInterrupt`s from the
launch's own SIGINT at teardown.

## TIAGo fetch, Isaac, `mobile_fetch_lab` (not passed)

Drive, standoff and the interlock probe work on Isaac in all three runs:

- arrival within 5 mm of the standoff;
- an outside drive command during arm motion moves the base 0–0.3 mm.

**The grasp fails:**

- **Runs 1–2** (`isaac_tiago1/2_not_passed.json`): the block tips over at
  the end of the move to the pre-grasp pose. The open fingertips (pads reach
  9 cm past the TCP) already straddle its top there.
- **Run 2** used the new up-and-over route; the block still tipped.
- **Run 3** (`isaac_tiago3_not_passed.json`): with the pre-grasp raised so
  that the fingertips clear the top, the block stays upright, but the
  descent never settles.
- The TCP estimate from measured joints is within 1 mm of the block, as on
  MuJoCo.

**Where it stands:** Isaac's finger contact geometry or tracking differs
from the planner's model in a way the joint and TCP data do not show. Not
isolated yet.

## Planner changes kept

**Kept:** joint routes now go **up and over** first. The arm moves to an IK
pose 0.15 m or 0.30 m above the target, then down, before trying a direct
sweep, other vias or RRT-Connect. Regressions pass:

- TIAGo fetch on MuJoCo: 106.8 s, carry drift 5.5 mm, place −0.1 mm
  (`regression_tiago_mujoco.json`);
- Panda on PyBullet: 27.4 s, place 0.0 mm (`regression_panda_pybullet.json`).

**Reverted:** a fingertip-clearance pre-grasp (raise until the open
fingertips clear the object top by 3 cm). With it, TIAGo's descent on
MuJoCo also stalled (`regression_tiago_mujoco_fingertip_clearance_reverted.json`).
