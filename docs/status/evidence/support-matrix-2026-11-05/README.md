# R9.3: support matrix with round 58 and go2_walk_r2 (2026-11-05)

`scripts/r9_3_index_missions.py` appended 21 records to
[runtime-evidence-index.yaml](../../runtime-evidence-index.yaml), now 647:

- [round58-batch.yaml](../rounds-2026-10-19/round58-batch.yaml): the car
  course with the parking heading shuffle in all twelve cells
  (`course_screen`), and Go2 `nav_obstacle` seeds 1-3 in Isaac and MuJoCo
  with the trained `go2_walk_r1` policy and `spin_in_place:=true`. Their
  task type is `trained_policy_navigation_go2_walk_r1`, kept apart from the
  rl_sar navigation cells.
- [training-r2-batch.yaml](../training-2026-10-19/go2_walk_r2/training-r2-batch.yaml):
  `go2_walk_r2` drive checks in MuJoCo, Gazebo and Isaac
  (`trained_policy_drive_go2_walk_r2`).

`scripts/r9_3_support_matrix.py` validated every record again and wrote
[support-matrix-current.md](support-matrix-current.md) and
[support-matrix-current.json](support-matrix-current.json).

| | October 29 (626 reports) | November 5 (647 reports) |
|---|---|---|
| Measured cells | 294 | **299** |
| Passed screens | 282 | **289** |
| Failed screens | 11 | 9 |
| Unverified | 1 | 1 |

Changes, latest record per cell:

- **Car course, failed → passed:** Ackermann in MuJoCo, PyBullet and Isaac;
  rear-steer in Isaac; anti-Ackermann in MuJoCo and Isaac. The shuffle
  brought the parking heading from 11.8-20.6 degrees to 1.4-7.4 degrees.
- **Car course, passed → failed:** Ackermann and anti-Ackermann in Gazebo.
  Both aborted slalom gate 4 ("RegulatedPurePursuitController detected
  collision ahead") before parking, so the shuffle never ran there. Round 51
  passed both. The Gazebo rear-steer car aborted gate 1 the same way.
- **Car course, still failed:** PyBullet anti-Ackermann (12.3 degrees after
  8 pairs) and PyBullet rear-steer (0.165 m from the bay).
- **New, failed:** Go2 trained-policy navigation. MuJoCo passes seed 3 only,
  and Isaac passes none: its goals abort 0.40-0.56 m short after
  5-8 minutes.
- **New, passed:** `go2_walk_r2` drive in MuJoCo, Gazebo and Isaac.

All full-release gates stay blocked.
